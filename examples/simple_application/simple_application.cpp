#include <iostream>

#include <harmont/harmont.hpp>
using namespace harmont;

vertex_array::ptr vao;
vertex_buffer<float>::ptr vbo;
shader_program::ptr program;

void init_geometry() {
    vao = std::make_shared<vertex_array>();
    vao->bind();
    vertex_buffer<float>::layout_t vbo_layout = {{"position", 3}, {"color", 3}};
    vbo = vertex_buffer<float>::from_layout(vbo_layout);
    vbo->bind();
    Eigen::Matrix<float, 6, 6> data;
    data <<
        0.f, 0.f, 0.f, 1.f, 0.f, 0.f,
        1.f, 0.f, 0.f, 1.f, 0.f, 0.f,
        0.f, 0.f, 0.f, 0.f, 1.f, 0.f,
        0.f, 1.f, 0.f, 0.f, 1.f, 0.f,
        0.f, 0.f, 0.f, 0.f, 0.f, 1.f,
        0.f, 0.f, 1.f, 0.f, 0.f, 1.f;
    vbo->set_data(data);
    vbo->bind_to_array(vbo_layout, program);
    vbo->release();
    vao->release();
}

void init() {
    program = std::make_shared<shader_program>(
        vertex_shader::from_file("simple.vert"),
        fragment_shader::from_file("simple.frag")
    );

    init_geometry();
}

void render(camera::ptr cam) {
    program->bind();
    program->variable("modelview_matrix") = cam->view_matrix();
    vao->bind();

    glDrawArrays(GL_LINES, 0, 6);

    vao->release();
    program->release();
}

void reshape(camera::ptr cam) {
    program->bind();
    program->variable("projection_matrix") = cam->projection_matrix();
    program->release();
}

int main (int argc, char* argv[]) {
    auto app = freeglut_application::create<orbit_camera_model>(1024, 768, std::bind(&render, std::placeholders::_1), std::bind(&reshape, std::placeholders::_1));
    app->init(argc, argv, "Simple Application", std::bind(&init));

    app->run();
}
