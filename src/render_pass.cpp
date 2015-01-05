#include <render_pass.hpp>

#include <stdexcept>

namespace harmont {

render_pass::render_pass(vertex_shader::ptr vs, fragment_shader::ptr fs, const textures& outputs, texture::ptr depth_texture) : render_pass(vs, fs, nullptr, outputs, depth_texture) {
}

render_pass::render_pass(vertex_shader::ptr vs, fragment_shader::ptr fs, geometry_shader::ptr gs, const textures& outputs, texture::ptr depth_texture) {
    program_ = std::make_shared<shader_program>(vs, fs, gs, false);
    program_->link();
    fbo_ = std::make_shared<framebuffer>(outputs, depth_texture);
}

render_pass::render_pass(const std::vector<vertex_shader::ptr>& vs, const std::vector<fragment_shader::ptr>& fs, const textures& outputs, texture::ptr depth_texture) : render_pass(vs, fs, std::vector<geometry_shader::ptr>(), outputs, depth_texture) {
}

render_pass::render_pass(const std::vector<vertex_shader::ptr>& vs, const std::vector<fragment_shader::ptr>& fs, const std::vector<geometry_shader::ptr>& gs, const textures& outputs, texture::ptr depth_texture) {
    program_ = std::make_shared<shader_program>(vs, fs, gs, false);
    program_->link();
    fbo_ = std::make_shared<framebuffer>(outputs, depth_texture);
}

render_pass::~render_pass() {
}

shader_program::ptr render_pass::program() {
    return program_;
}

shader_program::const_ptr render_pass::program() const {
    return program_;
}

render_pass::textures& render_pass::outputs() {
    return fbo_->outputs();
}

const render_pass::textures& render_pass::outputs() const {
    return fbo_->outputs();
}

texture::ptr render_pass::depth_texture() {
    return fbo_->depth_texture();
}

texture::const_ptr render_pass::depth_texture() const {
    return fbo_->depth_texture();
}

framebuffer::ptr render_pass::fbo() {
    return fbo_;
}

void render_pass::render(const draw_callback_t& draw_call, const depth_params_t& depth_params, const named_textures& inputs) {
    program_->bind();
    fbo_->bind();

    framebuffer::named_textures textures(inputs.size());
    std::transform( inputs.begin(),
                    inputs.end(),
                    textures.begin(),
                    [&] (const named_texture& tex) {
                        return std::make_pair(tex.first, (*this)[tex.second].location());
                    }
    );
    fbo_->bind(textures);
    if (depth_params.clear_depth) {
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glClear(GL_DEPTH_BUFFER_BIT);
    }
    glDepthMask(depth_params.write_depth ? GL_TRUE : GL_FALSE);
    if (depth_params.test_depth) {
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
    } else {
        if (depth_params.write_depth) {
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_ALWAYS);
        } else {
            glDepthFunc(GL_LESS);
            glDisable(GL_DEPTH_TEST);
        }
    }
    draw_call(program_);
    fbo_->release();
    program_->release();
}

framebuffer::const_ptr render_pass::fbo() const {
    return fbo_;
}

void render_pass::bind_program() {
    program_->bind();
}

void render_pass::release_program() {
    program_->release();
}

render_pass::shader_variable render_pass::operator[](std::string name) {
    return (*program_)[name];
}

render_pass::shader_variable render_pass::variable(std::string name) {
    return (*program_)[name];
}

} // harmont
