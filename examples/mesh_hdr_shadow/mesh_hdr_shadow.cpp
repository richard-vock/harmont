#include <iostream>
#include <list>
#include <chrono>
#include <random>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <harmont/harmont.hpp>
#include <harmont/openmesh_traits.hpp>
#include <boost/lexical_cast.hpp>
extern "C" {
#include "rgbe.h"
}
using namespace harmont;

typedef OpenMesh::Vec4f   color_t;
typedef tri_mesh<color_t> mesh_t;

freeglut_application::ptr app_g;
mesh_t mesh_g;
vertex_array::ptr vao_g, debug_vao_g;
vertex_buffer<float>::ptr vbo_g, debug_vbo_g;
index_buffer<uint32_t>::ptr ibo_g, debug_ibo_g;
render_pass::ptr geom_pass_g;
render_pass::ptr shadow_pass_g;
render_pass::ptr debug_shadow_pass_g;
uint32_t num_indices;
texture::ptr diff_g;
std::vector<float> light_dir_g;
Eigen::AlignedBox<float, 3> bb_g;
float l_white_g;

Eigen::Matrix4f shadow_mat_g;
texture::ptr    shadow_tex_g;
texture::ptr    dummy_depth_tex_g;
Eigen::Matrix4f debug_shadow_mat_g;

template <class RNG>
Eigen::Vector2f gen_point(RNG& rng, float radius, float offset, const Eigen::Vector2f& base = Eigen::Vector2f::Zero()) {
    float r = rng() * radius + offset;
    float a = rng() * 2.f * static_cast<float>(M_PI);
    return base + r * Eigen::Vector2f(cos(a), sin(a));
}

std::vector<float> poisson_disk(uint32_t n, float radius, uint32_t k = 30) {
    if (n == 0) return std::vector<float>();

    auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<float> float_dist;
    auto float_rng = std::bind(float_dist, generator);

    float r2 = radius * radius;
    Eigen::Vector2f p = gen_point(float_rng, radius, 0.f);

    std::vector<Eigen::Vector2f> points(1, p);
    std::list<Eigen::Vector2f>   active(1, p);

    Eigen::Vector2f avg_point = p;
    uint32_t point_count = 0;
    while (points.size() < n && active.size()) {
        std::uniform_int_distribution<int> int_dist(0, active.size() - 1);
        std::list<Eigen::Vector2f>::iterator iter = active.begin();
        int random_int = int_dist(generator);
        std::advance(iter, random_int);
        const Eigen::Vector2f& s = *iter;

        bool valid = true;
        for (uint32_t j = 0; j < k; ++j) {
            Eigen::Vector2f np = gen_point(float_rng, radius, radius, s);
            valid = true;
            for (uint32_t pt = 0; pt < points.size(); ++pt) {
                const Eigen::Vector2f& point = points[pt];
                float square_dist = (point - np).squaredNorm();
                if (square_dist < r2) {
                    valid = false;
                    break;
                }
            }
            if (valid) {
                if (point_count > 1) avg_point *= static_cast<float>(point_count);
                avg_point += np;
                avg_point /= static_cast<float>(++point_count);
                points.push_back(np);
                active.push_back(np);
                break;
            }
        }

        if (!valid) {
            active.erase(iter);
        }
    }

    std::vector<float> result(points.size() * 2);
    for (uint32_t i = 0; i < points.size(); ++i) {
        Eigen::Vector2f local = points[i] - avg_point;
        result[2*i + 0] = local[0];
        result[2*i + 1] = local[1];
    }

    return result;
}

void load_hdr_map(const char* file_name) {
	FILE *f;

	f = fopen(file_name, "rb");
	if (f == NULL) {
        std::cout << "Cannot open hdr file \"" << file_name << "\" for reading." << "\n";
        exit(1);
	}

	rgbe_header_info header;
    int width, height;
	RGBE_ReadHeader(f, &width, &height, &header);
    float* data = new float[width*height*3];
	RGBE_ReadPixels_RLE(f, data, width, height);
	fclose(f);
    diff_g = texture::texture_2d(width, height, 3, data, GL_LINEAR, GL_LINEAR);
    delete [] data;
}

void setup_shadow_view() {
    //shadow_mat_g = app_g->current_camera()->projection_matrix() * app_g->current_camera()->view_matrix();
    //Eigen::Matrix4f bias;
    //bias <<
        //0.5f, 0.0f, 0.0f, 0.5f,
        //0.0f, 0.5f, 0.0f, 0.5f,
        //0.0f, 0.0f, 0.5f, 0.5f,
        //0.0f, 0.0f, 0.0f, 1.0f;
    //shadow_mat_g = bias * shadow_mat_g;
    debug_shadow_mat_g = ortho(-1.f, 1.f, -1.f, 1.f, -1.f, 1.f);
    const float margin = 0.01f;
    Eigen::Vector3f light_dir = Eigen::Vector3f(light_dir_g.data());
    Eigen::Vector3f center = bb_g.center();
    float radius = (bb_g.max() - center).norm();
    Eigen::Vector3f forward = light_dir;
    Eigen::Vector3f up = Eigen::Vector3f::UnitZ();
    if (fabs(up.dot(forward)) + Eigen::NumTraits<float>::dummy_precision() > 1.f) {
        up = Eigen::Vector3f::UnitY();
    }
    up -= up.dot(forward) * forward;
    up.normalize();
    Eigen::Vector3f right = up.cross(forward);

    shadow_mat_g.block<1,3>(0, 0) = right.transpose();
    shadow_mat_g.block<1,3>(1, 0) = up.transpose();
    shadow_mat_g.block<1,3>(2, 0) = forward.transpose();
    Eigen::Vector3f light_pos = center + (radius + margin) * light_dir;
    shadow_mat_g.block<3,1>(0, 3) = shadow_mat_g.block<3,3>(0, 0) * (-light_pos);
    shadow_mat_g.row(3) = Eigen::RowVector4f(0.f, 0.f, 0.f, 1.f);
    Eigen::Matrix4f bias = Eigen::Matrix4f::Identity();
    //bias <<
        //0.5f, 0.0f, 0.0f, 0.5f,
        //0.0f, 0.5f, 0.0f, 0.5f,
        //0.0f, 0.0f, 0.5f, 0.5f,
        //0.0f, 0.0f, 0.0f, 1.0f;
    shadow_mat_g = bias * ortho(-radius, radius, -radius, radius, margin, 2.f * radius + 2.f * margin) * shadow_mat_g;
}

void init_shadow() {
    shadow_tex_g = texture::texture_2d<float>(2048, 2048, 4);
    dummy_depth_tex_g = texture::depth_texture<float>(2048, 2048);
    //shadow_pass_g = std::make_shared<render_pass>(vertex_shader::from_file("shadow.vert"), fragment_shader::from_file("shadow.frag"), render_pass::textures(), shadow_tex_g);
    shadow_pass_g = std::make_shared<render_pass>(vertex_shader::from_file("shadow.vert"), fragment_shader::from_file("shadow.frag"), render_pass::textures({shadow_tex_g}), dummy_depth_tex_g);
    setup_shadow_view();
}

void render_shadow(shader_program::ptr program) {
    //glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    int width = app_g->width();
    int height = app_g->height();
    glViewport(0, 0, 2048, 2048);
    vao_g->bind();
    ibo_g->bind();
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDrawElements(GL_TRIANGLES, num_indices, GL_UNSIGNED_INT, nullptr);
    ibo_g->release();
    vao_g->release();
    glViewport(0, 0, width, height);
    glClearColor(0.0, 0.0, 0.0, 1.0);
    //glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
}

void render_debug_shadow(shader_program::ptr program) {
    //glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    int width = app_g->width();
    int height = app_g->height();
    glViewport(0, 0, 200, 200);
    debug_vao_g->bind();
    debug_ibo_g->bind();
    glClear(GL_DEPTH_BUFFER_BIT);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
    debug_ibo_g->release();
    debug_vao_g->release();
    glViewport(0, 0, width, height);
    //glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
}

void init() {
    Eigen::Vector3f light_dir = Eigen::Vector3f(1.f, 1.f, 1.f).normalized();
    light_dir_g = std::vector<float>(light_dir.data(), light_dir.data()+3);

    init_shadow();

    load_hdr_map("pisa_diffuse.hdr");

    // pass
    geom_pass_g = std::make_shared<render_pass>(vertex_shader::from_file("mesh.vert"), fragment_shader::from_file("hdr.frag"));

    std::vector<float> disk = poisson_disk(16, 1.f);
    geom_pass_g->set_uniform("poisson_disk[0]", disk);

    // light
    geom_pass_g->set_uniform("light_dir", light_dir_g);
    geom_pass_g->set_uniform("two_sided", 1);

    // data
    vao_g = std::make_shared<vertex_array>();
    vao_g->bind();

    Eigen::MatrixXf vbo_data;
    Eigen::Matrix<uint32_t, Eigen::Dynamic, 1> ibo_data;
    mesh_traits<mesh_t>::buffer_data(mesh_g, {POSITION, COLOR, NORMAL}, vbo_data, ibo_data, false);
    num_indices = ibo_data.rows();

    vertex_buffer<float>::layout_t vbo_layout = {{"position", 3}, {"color", 1}, {"normal", 3}};
    vbo_g = vertex_buffer<float>::from_data(vbo_data);
    vbo_g->bind_to_array(vbo_layout, geom_pass_g);

    ibo_g = index_buffer<uint32_t>::from_data(ibo_data);

    vao_g->release();

    debug_shadow_pass_g = std::make_shared<render_pass>(vertex_shader::from_file("debug.vert"), fragment_shader::from_file("debug.frag"));

    debug_vao_g = std::make_shared<vertex_array>();
    debug_vao_g->bind();

    Eigen::MatrixXf debug_vbo_data(4, 3);
    debug_vbo_data <<
        -1.f, -1.f, 0.f,
         1.f, -1.f, 0.f,
         1.f,  1.f, 0.f,
        -1.f,  1.f, 0.f;
    Eigen::Matrix<uint32_t, Eigen::Dynamic, 1> debug_ibo_data(6);
    debug_ibo_data << 0, 1, 2, 0, 2, 3;
    vertex_buffer<float>::layout_t debug_vbo_layout = {{"position", 3}};
    debug_vbo_g = vertex_buffer<float>::from_data(debug_vbo_data);
    debug_vbo_g->bind_to_array(debug_vbo_layout, debug_shadow_pass_g);

    debug_ibo_g = index_buffer<uint32_t>::from_data(debug_ibo_data);

    debug_vao_g->release();

    glEnable(GL_DEPTH_TEST);
}

void render(shader_program::ptr program) {
    geom_pass_g->set_uniform("light_dir", light_dir_g);
    geom_pass_g->set_uniform("l_white", l_white_g);
    vao_g->bind();
    ibo_g->bind();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDrawElements(GL_TRIANGLES, num_indices, GL_UNSIGNED_INT, nullptr);
    ibo_g->release();
    vao_g->release();
}

void display(camera::ptr cam) {
    // update near/far
    Eigen::Vector3f bb_min = bb_g.min(), bb_max = bb_g.max();
    Eigen::Vector3f pos = app_g->current_camera()->position();
    Eigen::Vector3f dir = app_g->current_camera()->forward().normalized();
    float near = std::numeric_limits<float>::max(), far = 0.f;
    for (uint32_t c=0; c<8; ++c) {
        // compute corner
        Eigen::Vector3f corner;
        for (uint32_t i = 0; i < 3; ++i) {
            corner[i] = (c & 1 << i) ? bb_max[i] : bb_min[i];
        }
        // compute lambda (depth)
        float lambda = (corner-pos).dot(dir);
        if (lambda < near) near = lambda;
        if (lambda > far)  far = lambda;
    }
    near = std::max(near, 0.01f);
    app_g->current_camera()->set_near(near);
    app_g->current_camera()->set_far(near);


    shadow_pass_g->set_uniform("shadow_matrix", shadow_mat_g);
    shadow_pass_g->render(&render_shadow);

    geom_pass_g->set_uniform("modelview_matrix", cam->view_matrix());
    geom_pass_g->set_uniform("shadow_matrix", shadow_mat_g);
    //geom_pass_g->set_uniform("normal_matrix", cam->view_normal_matrix());
    auto eye = cam->forward().normalized();
    std::vector<float> eye_dir = { eye[0], eye[1], eye[2] };
    geom_pass_g->set_uniform("eye_dir", eye_dir);
    geom_pass_g->render(&render, {{diff_g, "map_diffuse"}, {shadow_tex_g, "map_shadow"}});
    debug_shadow_pass_g->set_uniform("shadow_matrix", debug_shadow_mat_g);
    debug_shadow_pass_g->render(&render_debug_shadow, {{shadow_tex_g, "map_shadow"}});
    //geom_pass_g->render(&render, {{diff_g, "map_diffuse"}});
    //geom_pass_g->render(&render);
}

void reshape(camera::ptr cam) {
    setup_shadow_view();
    geom_pass_g->set_uniform("projection_matrix", cam->projection_matrix());
}


int main (int argc, char* argv[]) {
	OpenMesh::IO::Options opt;
	opt += OpenMesh::IO::Options::FaceNormal;
	opt += OpenMesh::IO::Options::VertexNormal;
	opt += OpenMesh::IO::Options::VertexColor;

	mesh_g.request_vertex_normals();
	mesh_g.request_face_normals();
    if (!OpenMesh::IO::read_mesh(mesh_g, argv[1], opt)) {
        std::cerr << "Unable to read mesh file. Aborting.\n";
        exit(1);
    }
    l_white_g = 0.2f;
    mesh_g.triangulate();
    if (!opt.face_has_normal()) {
        mesh_g.update_face_normals();
    }
    mesh_g.update_normals();
    // let us not expect materials to be set
    for (auto it = mesh_g.vertices_begin(); it != mesh_g.vertices_end(); ++it) {
        mesh_g.set_color(*it, color_t(1.f, 1.f, 1.f, 1.f));
        bb_g.extend(Eigen::Vector3f(mesh_g.point(*it).data()));
    }

    app_g = freeglut_application::create<orbit_camera_model>(800, 600, &display, &reshape);
    app_g->init(argc, argv, "Render Mesh", &init);
    app_g->on_drag_left([&] (Eigen::Vector2i pos, Eigen::Vector2i delta) { l_white_g += static_cast<float>(delta[1]) * 0.001f; if (l_white_g < 0.0001) l_white_g = 0.0001; app_g->update(); });
    app_g->on_click_left([&] (Eigen::Vector2i pos) { Eigen::Vector3f dir = app_g->current_camera()->forward().normalized(); light_dir_g = std::vector<float>(dir.data(), dir.data()+3); setup_shadow_view(); app_g->update(); });

    app_g->run();
}
