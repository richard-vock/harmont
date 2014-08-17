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
        case 1: return std::bind(&glFramebufferTexture1D, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, 0); break;
        case 2: return std::bind(&glFramebufferTexture2D, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, 0); break;
        case 3: return std::bind(&glFramebufferTexture3D, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, 0, 0); break;
		default: throw std::runtime_error("attach_func: Invalid texture dimension. Must be in [1,3]"+SPOT);
	}
}


framebuffer::framebuffer(const textures& output_textures, texture::ptr depth_texture) : outputs_(output_textures), depth_(depth_texture) {
    handle_ = glGenFramebuffers(1);
    bind();
    for (uint32_t index = 0; index < output_.size(); ++index) {
        bind_texture_(output_[i], GL_COLOR_ATTACHMENT0 + index);
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

textures framebuffer::outputs() {
    return outputs_;
}

const textures& framebuffer::outputs() const {
    return outputs_;
}

textures::ptr framebuffer::depth_texture() {
    return depth_;
}

textures::const_ptr framebuffer::depth_texture() const {
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
        GLenum buffers = new GLenum(outputs_.size());
        for (uint32_t i = 0; i < outputs_.size(); ++i) {
            buffers[i] = GL_COLOR_ATTACHMENT0 + i;
        }
        glDrawBuffers(buffers, outputs_.size());
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
        named_tex.first->releas();
    }
    for (const auto& tex : outputs_) {
        tex->release();
    }
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0)
}

void framebuffer::bind_texture_(texture::const_ptr tex, GLenum attachment) {
    GLenum target = tex->target();
    int dim = tex->dim();
    attach_func(dim)(GL_DRAW_FRAMEBUFFER, attachment, target, tex->handle());
}

void framebuffer::check_() {
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);

    if (status == GL_FRAMEBUFFER_COMPLETE) {
        return
    }
    if (status == GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT) {
        throw std::runtime_error("framebuffer::check: Incomplete attachment"+SPOT);
    }
    if (status == GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS) {
        throw std::runtime_error("framebuffer::check: Inconsistent attachment sizes"+SPOT);
    }
    if (status == GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT) {
        throw std::runtime_error("framebuffer::check: Missing attachments"+SPOT);
    }
    if (status == GL_FRAMEBUFFER_UNSUPPORTED) {
        throw std::runtime_error("framebuffer::check: Format combination is unsupported"+SPOT);
    }
}


} // harmont
