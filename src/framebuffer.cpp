/*
 * harmont - c++ opengl wrapper library
 *
 * Written in 2014 by Richard Vock
 *
 * To the extent possible under law, the author(s) have dedicated all copyright and related and neighboring rights to this software to the public domain worldwide.
 * This software is distributed without any warranty.
 *
 * You should have received a copy of the CC0 Public Domain Dedication along with this software.
 * If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
 *
 */

#include <framebuffer.hpp>

#include <tuple>
#include <stdexcept>

namespace harmont {

static std::function<void (GLenum, GLenum, GLenum, GLuint)> attach_func(int dim) {
	switch(dim) {
        case 1: return [&] (GLenum target, GLenum attach, GLenum tex_target, GLuint tex) { glFramebufferTexture1D(target, attach, tex_target, tex, 0); }; break;
        case 2: return [&] (GLenum target, GLenum attach, GLenum tex_target, GLuint tex) { glFramebufferTexture2D(target, attach, tex_target, tex, 0); }; break;
        case 3: return [&] (GLenum target, GLenum attach, GLenum tex_target, GLuint tex) { glFramebufferTexture3D(target, attach, tex_target, tex, 0, 0); }; break;
		default: throw std::runtime_error("attach_func: Invalid texture dimension. Must be in [1,3]"+SPOT);
	}
}


framebuffer::framebuffer(const textures& output_textures, texture::ptr depth_texture) : outputs_(output_textures), depth_(depth_texture) {
    glGenFramebuffers(1, &handle_);
    bind();
    for (uint32_t index = 0; index < outputs_.size(); ++index) {
        bind_texture_(outputs_[index], GL_COLOR_ATTACHMENT0 + index);
    }
    if (depth_ != nullptr) {
        bind_texture_(depth_, GL_DEPTH_ATTACHMENT);
    }
    check_();
    release();
}

framebuffer::~framebuffer() {
}

GLuint framebuffer::handle() const {
    return handle_;
}

framebuffer::textures& framebuffer::outputs() {
    return outputs_;
}

const framebuffer::textures& framebuffer::outputs() const {
    return outputs_;
}

texture::ptr framebuffer::depth_texture() {
    return depth_;
}

texture::const_ptr framebuffer::depth_texture() const {
    return depth_;
}

void framebuffer::bind(const named_textures& input_textures, bool only_bind_input) {
    bool use_output = !only_bind_input && outputs_.size() > 0;

    if (!use_output) {
        last_inputs_.clear();
        release();
    } else {
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, handle_);
    }

    if (use_output) {
        GLenum* buffers = new GLenum[outputs_.size()];
        for (uint32_t i = 0; i < outputs_.size(); ++i) {
            buffers[i] = GL_COLOR_ATTACHMENT0 + i;
        }
        glDrawBuffers(outputs_.size(), buffers);
        delete buffers;
    }

    for (uint32_t i = 0; i < input_textures.size(); ++i) {
        texture::ptr tex; GLuint name;
        std::tie(tex, name) = input_textures[i];
        glUniform1i(name, i);
        glActiveTexture(GL_TEXTURE0 + i);
        tex->bind();
    }

    last_inputs_ = input_textures;
}

void framebuffer::release() {
    for (const auto& named_tex : last_inputs_) {
        named_tex.first->release();
    }
    for (const auto& tex : outputs_) {
        tex->release();
    }
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
}

void framebuffer::bind_texture_(texture::const_ptr tex, GLenum attachment) {
    GLenum target = tex->target();
    int dim = tex->dim();
    attach_func(dim)(GL_DRAW_FRAMEBUFFER, attachment, target, tex->handle());
}

void framebuffer::check_() {
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);

    if (status == GL_FRAMEBUFFER_COMPLETE) {
        return;
    }
    if (status == GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT) {
        throw std::runtime_error("framebuffer::check: Incomplete attachment"+SPOT);
    }
    if (status == GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT) {
        throw std::runtime_error("framebuffer::check: Missing attachments"+SPOT);
    }
    if (status == GL_FRAMEBUFFER_UNSUPPORTED) {
        throw std::runtime_error("framebuffer::check: Format combination is unsupported"+SPOT);
    }
}


} // harmont
