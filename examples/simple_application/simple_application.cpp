#include <iostream>

#include <harmont/harmont.hpp>
using namespace harmont;

vertex_array::ptr vao;
vertex_buffer<float>::ptr vbo;
render_pass::ptr main_pass;
typedef application::screen_pos_t px;

void init_geometry() {
    vertex_buffer<float>::layout_t vbo_layout = {{"position", 3}, {"color", 3}};
    Eigen::Matrix<float, 24, 6> data;
    data <<
        -1.f, -1.f, -1.f, 1.f, 0.f, 0.f, // front
         1.f, -1.f, -1.f, 1.f, 0.f, 0.f,
         1.f, -1.f,  1.f, 1.f, 0.f, 0.f,
        -1.f, -1.f,  1.f, 1.f, 0.f, 0.f,
        -1.f,  1.f, -1.f, 1.f, 0.f, 0.f, // back
        -1.f,  1.f,  1.f, 1.f, 0.f, 0.f,
         1.f,  1.f,  1.f, 1.f, 0.f, 0.f,
         1.f,  1.f, -1.f, 1.f, 0.f, 0.f,
        -1.f, -1.f, -1.f, 0.f, 1.f, 0.f, // left
        -1.f,  1.f, -1.f, 0.f, 1.f, 0.f,
        -1.f,  1.f,  1.f, 0.f, 1.f, 0.f,
        -1.f, -1.f,  1.f, 0.f, 1.f, 0.f,
         1.f, -1.f, -1.f, 0.f, 1.f, 0.f, // right
         1.f,  1.f, -1.f, 0.f, 1.f, 0.f,
         1.f,  1.f,  1.f, 0.f, 1.f, 0.f,
         1.f, -1.f,  1.f, 0.f, 1.f, 0.f,
        -1.f, -1.f,  1.f, 0.f, 0.f, 1.f, // top
         1.f, -1.f,  1.f, 0.f, 0.f, 1.f,
         1.f,  1.f,  1.f, 0.f, 0.f, 1.f,
        -1.f,  1.f,  1.f, 0.f, 0.f, 1.f,
        -1.f, -1.f, -1.f, 0.f, 0.f, 1.f, // bottom
        -1.f,  1.f, -1.f, 0.f, 0.f, 1.f,
         1.f,  1.f, -1.f, 0.f, 0.f, 1.f,
         1.f, -1.f, -1.f, 0.f, 0.f, 1.f;
    vao = std::make_shared<vertex_array>();
    vao->bind();
    vbo = vertex_buffer<float>::from_data(data);
    vbo->bind_to_array(vbo_layout, main_pass);
    vao->release();
}

void init() {
    glClearColor(.2f, .2f, .2f, 1.f);
    main_pass = std::make_shared<render_pass>(vertex_shader::from_file("simple.vert"), fragment_shader::from_file("simple.frag"));
    init_geometry();
}

void render_geometry(shader_program::ptr) {
    glEnable(GL_DEPTH_TEST);
    vao->bind();
    glDrawArrays(GL_QUADS, 0, 24);
    vao->release();
}

void render(camera::ptr cam) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    main_pass->set_uniform("modelview_matrix", cam->view_matrix());
    main_pass->render(&render_geometry);
}

void reshape(camera::ptr cam) {
    main_pass->set_uniform("projection_matrix", cam->projection_matrix());
}

int main (int argc, char* argv[]) {
    auto app = freeglut_application::create<orbit_camera_model>(1024, 768, std::bind(&render, std::placeholders::_1), std::bind(&reshape, std::placeholders::_1));
    app->init(argc, argv, "Simple Application", std::bind(&init));

    app->run();
}
