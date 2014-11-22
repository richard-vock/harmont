#include <iostream>

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

mesh_t mesh_g;
vertex_array::ptr vao_g;
vertex_buffer<float>::ptr vbo_g;
index_buffer<uint32_t>::ptr ibo_g;
render_pass::ptr geom_pass_g;
uint32_t num_indices;
texture::ptr diff_g;
std::vector<float> light_dir_g;

float l_white_g;

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


void init() {
    load_hdr_map("pisa_diffuse.hdr");

    // pass
    geom_pass_g = std::make_shared<render_pass>(vertex_shader::from_file("mesh.vert"), fragment_shader::from_file("hdr.frag"));

    // light
    light_dir_g = {0.f, 0.f, 1.f};
    geom_pass_g->set_uniform("light_dir", light_dir_g);

    // data
    vao_g = std::make_shared<vertex_array>();
    vao_g->bind();

    Eigen::MatrixXf vbo_data;
    Eigen::Matrix<uint32_t, Eigen::Dynamic, 1> ibo_data;
    mesh_traits<mesh_t>::buffer_data(mesh_g, {POSITION, COLOR, NORMAL}, vbo_data, ibo_data, true);
    num_indices = ibo_data.rows();

    vertex_buffer<float>::layout_t vbo_layout = {{"position", 3}, {"color", 1}, {"normal", 3}};
    vbo_g = vertex_buffer<float>::from_data(vbo_data);
    vbo_g->bind_to_array(vbo_layout, geom_pass_g);

    ibo_g = index_buffer<uint32_t>::from_data(ibo_data);

    vao_g->release();

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
    geom_pass_g->set_uniform("modelview_matrix", cam->view_matrix());
    //geom_pass_g->set_uniform("normal_matrix", cam->view_normal_matrix());
    auto eye = -cam->forward();
    std::vector<float> eye_dir = { eye[0], eye[1], eye[2] };
    //geom_pass_g->set_uniform("eye_dir", eye_dir);
    geom_pass_g->render(&render, {{diff_g, "map_diffuse"}});
    //geom_pass_g->render(&render);
}

void reshape(camera::ptr cam) {
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
    l_white_g = boost::lexical_cast<float>(argv[2]);
    mesh_g.triangulate();
    if (!opt.face_has_normal()) {
        mesh_g.update_face_normals();
    }
    mesh_g.update_normals();
    // let us not expect materials to be set
    for (auto it = mesh_g.vertices_begin(); it != mesh_g.vertices_end(); ++it) {
        mesh_g.set_color(*it, color_t(1.f, 1.f, 1.f, 1.f));
    }

    auto app = freeglut_application::create<orbit_camera_model>(800, 600, &display, &reshape);
    app->init(argc, argv, "Render Mesh", &init);
    app->on_drag_left([&] (Eigen::Vector2i pos, Eigen::Vector2i delta) { l_white_g += static_cast<float>(delta[1]) * 0.001f; if (l_white_g < 0.0001) l_white_g = 0.0001; app->update(); });
    app->on_click_left([&] (Eigen::Vector2i pos) { Eigen::Vector3f dir = app->current_camera()->forward().normalized(); light_dir_g = std::vector<float>(dir.data(), dir.data()+3); app->update(); });

    app->run();
}
