#include <iostream>

#include <harmont/harmont.hpp>
#include <harmont/pcl_traits.hpp>
#include <harmont/deferred_renderer.hpp>
#include <pcl/io/pcd_io.h>
#include <boost/lexical_cast.hpp>
using namespace harmont;

typedef pcl::PointXYZRGBNormal point_t;
typedef cloud<point_t>         cloud_t;

freeglut_application::ptr   app_g;
deferred_renderer::ptr_t    renderer_g;

cloud_t::Ptr                cloud_g;
Eigen::AlignedBox<float, 3> bb_g;

vertex_array::ptr           vao_shadow_g, vao_display_g;
vertex_buffer<float>::ptr   vbo_shadow_g, vbo_display_g;
index_buffer<uint32_t>::ptr ibo_g;
uint32_t                    num_indices_g;


void init_geometry(shader_program::ptr program, pass_type_t type) {
    vertex_buffer<float>::layout_t vbo_layout;
    if (type == SHADOW_GEOMETRY) {
        vao_shadow_g->bind();
        vbo_layout = {{"position", 3}};
        vbo_shadow_g->bind_to_array(vbo_layout, program);
        vao_shadow_g->release();
    } else {
        vao_display_g->bind();
        vbo_layout = {{"position", 3}, {"color", 1}, {"normal", 3}};
        vbo_display_g->bind_to_array(vbo_layout, program);
        vao_display_g->release();
    }
}

void init() {
    Eigen::Vector3f light_dir = Eigen::Vector3f(1.f, 1.f, 1.f).normalized();

    deferred_renderer::render_parameters_t r_params {
        light_dir,
        0.8f,
        0.003f,
        true,
        "pisa_diffuse.hdr"
    };
    deferred_renderer::shadow_parameters_t s_params {
        2048,
        32
    };
    renderer_g = std::make_shared<deferred_renderer>(r_params, s_params, bb_g, app_g->current_camera()->width(), app_g->current_camera()->height());

    vao_shadow_g = std::make_shared<vertex_array>();
    vao_display_g = std::make_shared<vertex_array>();

    Eigen::MatrixXf vbo_data, pos_data;
    Eigen::Matrix<uint32_t, Eigen::Dynamic, 1> ibo_data;
    pointcloud_traits<cloud_t>::buffer_data(*cloud_g, {POSITION, COLOR, NORMAL}, vbo_data, ibo_data);
    pos_data = vbo_data.block(0, 0, vbo_data.rows(), 3);
    num_indices_g = ibo_data.rows();

    vbo_shadow_g = vertex_buffer<float>::from_data(pos_data);
    vbo_display_g = vertex_buffer<float>::from_data(vbo_data);
    ibo_g = index_buffer<uint32_t>::from_data(ibo_data);
    renderer_g->init(&init_geometry);
}

void render(shader_program::ptr program, pass_type_t type) {
    if (type == SHADOW_GEOMETRY) {
        vao_shadow_g->bind();
    } else {
        vao_display_g->bind();
    }
    ibo_g->bind();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDrawElements(GL_POINTS, num_indices_g, GL_UNSIGNED_INT, nullptr);
    ibo_g->release();
    if (type == SHADOW_GEOMETRY) {
        vao_shadow_g->release();
    } else {
        vao_display_g->release();
    }
}

void display(camera::ptr cam) {
    renderer_g->render(&render, cam, bb_g);
}

void reshape(camera::ptr cam) {
    renderer_g->reshape(cam);
}


int main (int argc, char* argv[]) {
    cloud_g = cloud_t::Ptr(new cloud_t);
    if (pcl::io::loadPCDFile(argv[1], *cloud_g) != 0) {
        std::cerr << "Unable to read pcd file. Aborting.\n";
    }
    for (auto& p : *cloud_g) {
        bb_g.extend(p.getVector3fMap());
        p.r = 255;
        p.g = 255;
        p.b = 255;
    }

    app_g = freeglut_application::create<orbit_camera_model>(800, 600, &display, &reshape);
    app_g->init(argc, argv, "Render Mesh", &init);
    app_g->on_drag_left([&] (Eigen::Vector2i pos, Eigen::Vector2i delta) { renderer_g->delta_clipping_height(-delta[1] * 0.01f); app_g->update(); });
    app_g->on_click_left([&] (Eigen::Vector2i pos) { Eigen::Vector3f dir = app_g->current_camera()->forward().normalized(); renderer_g->set_light_dir(dir, bb_g); app_g->update(); });
    app_g->on_char([&] (unsigned char key) {
        if (key == 'b') renderer_g->delta_shadow_bias(-0.001);
        if (key == 'B') renderer_g->delta_shadow_bias(0.001);
        if (key == 'e') renderer_g->delta_exposure(-0.001f);
        if (key == 'E') renderer_g->delta_exposure(0.001f);
        if (key == 'r') renderer_g->delta_ssdo_radius(-0.1f);
        if (key == 'R') renderer_g->delta_ssdo_radius(0.1f);
        if (key == 's') renderer_g->delta_ssdo_exponent(-0.1f);
        if (key == 'S') renderer_g->delta_ssdo_exponent(0.1f);
        if (key == 'a') renderer_g->delta_ssdo_reflective_albedo(-0.1f);
        if (key == 'A') renderer_g->delta_ssdo_reflective_albedo(0.1f);
        if (key == '-') renderer_g->delta_point_size(-0.01f);
        if (key == '+') renderer_g->delta_point_size(0.01f);
        if (key == 'c' || key == 'C') renderer_g->toggle_clipping();
        app_g->update();
    });

    app_g->run();
}
