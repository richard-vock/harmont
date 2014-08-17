#include <render_pass.hpp>

#include <stdexcept>

namespace harmont {

render_pass(const shader_sources& sources, const textures& outputs, texture::ptr depth_texture) {
    if (sources.size == 2) {
        program_ = std::make_shared<shader_program>(vertex_shader(sources[0]), fragment_shader(sources[1]), false);
    } else if (sources.size() == 3) {
        program_ = std::make_shared<shader_program>(vertex_shader(sources[0]), fragment_shader(sources[1]), geometry_shader(sources[2]), false);
    } else {
        throw std::runtime_error("render_pass::render_pass: Invalid number of shader sources" + SPOT);
    }
    program_->link();
    fbo_ = std::make_shared<framebuffer>(outputs, depth_tex);
}

~render_pass() {
}

shader_program::ptr program() {
    return program_;
}

shader_program::const_ptr program() const {
    return program_;
}

textures& outputs() {
    return fbo_->outputs();
}

const textures& outputs() const {
    return fbo_->outputs();
}

texture::ptr depth_texture() {
    return fbo_->depth_texture();
}

texture::const_ptr depth_texture() const {
    return fbo_->depth_texture();
}

framebuffer::ptr framebuffer() {
    return fbo_;
}

framebuffer::const_ptr framebuffer() const {
    return fbo_;
}

void bind_program() {
    program_->bind();
}

void release_program() {
    program_->release();
}

shader_variable operator[](std::string name) {
    return (*program)[name];
}

} // harmont
