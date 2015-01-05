#include <render_pass_2d.hpp>

namespace harmont {


render_pass_2d::render_pass_2d(vertex_shader::ptr vs, fragment_shader::ptr fs, const textures& outputs, texture::ptr depth_texture) : render_pass(vs, fs, outputs, depth_texture) {
    init_quad_geometry_();
}

render_pass_2d::render_pass_2d(vertex_shader::ptr vs, fragment_shader::ptr fs, geometry_shader::ptr gs, const textures& outputs, texture::ptr depth_texture) : render_pass(vs, fs, gs, outputs, depth_texture) {
    init_quad_geometry_();
}

render_pass_2d::render_pass_2d(const std::vector<vertex_shader::ptr>& vs, const std::vector<fragment_shader::ptr>& fs, const textures& outputs, texture::ptr depth_texture) : render_pass(vs, fs, outputs, depth_texture) {
    init_quad_geometry_();
}

render_pass_2d::render_pass_2d(const std::vector<vertex_shader::ptr>& vs, const std::vector<fragment_shader::ptr>& fs, const std::vector<geometry_shader::ptr>& gs, const textures& outputs, texture::ptr depth_texture) : render_pass(vs, fs, gs, outputs, depth_texture) {
    init_quad_geometry_();
}

render_pass_2d::~render_pass_2d() {
}

void render_pass_2d::render(const draw_callback_t& pre_draw_call, const named_textures& inputs) {
    render_pass::render(
        [&] (shader_program::ptr prog) {
            vao_->bind();
            ibo_->bind();
            pre_draw_call(prog);
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
            ibo_->release();
            vao_->release();
        },
        {false, false, false},
        inputs
    );
}

void render_pass_2d::render(const named_textures& inputs) {
    render_pass::render(
        [&] (shader_program::ptr prog) {
            vao_->bind();
            ibo_->bind();
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
            ibo_->release();
            vao_->release();
        },
        {false, false, false},
        inputs
    );
}

void render_pass_2d::init_quad_geometry_() {
    vao_ = std::make_shared<vertex_array>();
    vao_->bind();

    Eigen::MatrixXf vbo_data(4, 3);
    Eigen::Matrix<uint32_t, Eigen::Dynamic, 1> ibo_data(6);
    vertex_buffer<float>::layout_t vbo_layout = {{"position", 3}};

    vbo_data <<
        -1.f, -1.f, 0.f,
         1.f, -1.f, 0.f,
         1.f,  1.f, 0.f,
        -1.f,  1.f, 0.f;

    ibo_data <<
        0, 1, 2,
        0, 2, 3;

    vbo_ = vertex_buffer<float>::from_data(vbo_data);
    vbo_->bind_to_array(vbo_layout, program_);

    ibo_ = index_buffer<uint32_t>::from_data(ibo_data);

    vao_->release();
}


} // harmont
