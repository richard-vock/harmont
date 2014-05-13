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

namespace harmont {


framebuffer::framebuffer() : width_(0), height_(0), frame_id_(0), depth_id_(0), stencil_id_(0), buffers_(0) {
	glGetIntegerv(GL_MAX_COLOR_ATTACHMENTS, &max_color_attachments_);
	buffers_ = new GLenum[max_color_attachments_];

	glGenFramebuffers(1, &frame_id_);
}

framebuffer::framebuffer(int width, int height) : width_(0), height_(0), frame_id_(0), depth_id_(0), stencil_id_(0), buffers_(0) {
	glGetIntegerv(GL_MAX_COLOR_ATTACHMENTS, &max_color_attachments_);
	buffers_ = new GLenum[max_color_attachments_];

	glGenFramebuffers(1, &frame_id_);
	width_ = width;
	height_ = height;
}

framebuffer::~framebuffer() {
	GLuint tex_id;

	std::vector<GLuint>::const_iterator cii;
	for(cii = tex_id_.begin(); cii != tex_id_.end(); cii++) {
		tex_id = *cii;
		glDeleteTextures(1, &tex_id);
	}

	if (depth_id_) {
		glDeleteRenderbuffers(1, &depth_id_);
	}
	if (stencil_id_) {
		glDeleteRenderbuffers(1, &stencil_id_);
	}
	glDeleteFramebuffers(1, &frame_id_);
	delete[] buffers_;
}

void framebuffer::resize(int width, int height) {
	width_ = width; height_ = height;
};

void framebuffer::attach_render(GLenum iformat) {
	GLenum attachment;
	GLuint render_id;

	if (width_ == 0 || height_ == 0) {
		throw std::domain_error("framebuffer::attach_render: one of the dimensions is zero"+SPOT);
	}

	if (iformat == GL_DEPTH_COMPONENT24 || iformat == GL_DEPTH_COMPONENT) {
		attachment = GL_DEPTH_ATTACHMENT;
	}
	else if (iformat == GL_STENCIL_INDEX1 || iformat == GL_STENCIL_INDEX4 || iformat == GL_STENCIL_INDEX8 ||
		iformat == GL_STENCIL_INDEX16 || iformat == GL_STENCIL_INDEX) {
		attachment = GL_STENCIL_ATTACHMENT;
	}
	else if (iformat == GL_DEPTH24_STENCIL8 || iformat == GL_DEPTH_STENCIL) {
		attachment = GL_DEPTH_STENCIL_ATTACHMENT;
	}
	else {
		throw std::invalid_argument("framebuffer::attach_render: unrecognized internal format"+SPOT);
	}

	glGenRenderbuffers(1, &render_id);
	glBindFramebuffer(GL_FRAMEBUFFER, frame_id_);
	glBindRenderbuffer(GL_RENDERBUFFER, render_id);
	glRenderbufferStorage(GL_RENDERBUFFER, iformat, width_, height_);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, attachment, GL_RENDERBUFFER, render_id);

	if (attachment == GL_DEPTH_ATTACHMENT || attachment == GL_DEPTH_STENCIL_ATTACHMENT) {
		depth_id_ = render_id;
	}
	else if (attachment == GL_STENCIL_ATTACHMENT) {
		stencil_id_ = render_id;
	}
}

void framebuffer::attach_texture(GLenum iformat, GLint filter) {
	GLenum format;
	GLenum type;
	GLenum attachment;
	GLuint tex_id;

	if (width_ == 0 || height_ == 0) {
		throw std::domain_error("framebuffer::attach_texture: one of the dimensions is zero"+SPOT);
	}

	if (int(tex_id_.size()) == max_color_attachments_) {
		throw std::out_of_range("framebuffer::attach_texture: GL_MAX_COLOR_ATTACHMENTS exceeded"+SPOT);
	}

	attachment = GL_COLOR_ATTACHMENT0 + static_cast<GLenum>(tex_id_.size()); // common attachment for color textures
	if (iformat == GL_RGBA16F_ARB || iformat == GL_RGBA32F_ARB) {
		format = GL_RGBA;
		type = GL_FLOAT;
	}
	else if (iformat == GL_RGB16F_ARB || iformat == GL_RGB32F_ARB) {
		format = GL_RGB;
		type = GL_FLOAT;
	}
	else if (iformat == GL_LUMINANCE_ALPHA16F_ARB || iformat == GL_LUMINANCE_ALPHA32F_ARB) {
		format = GL_LUMINANCE_ALPHA;
		type = GL_FLOAT;
	}
	else if (iformat == GL_LUMINANCE16F_ARB || iformat == GL_LUMINANCE32F_ARB) {
		format = GL_LUMINANCE;
		type = GL_FLOAT;
	}
	else if (iformat == GL_RGBA8 || iformat == GL_RGBA || iformat == 4) {
		format = GL_RGBA;
		type = GL_UNSIGNED_BYTE;
	}
	else if (iformat == GL_RGB8 || iformat == GL_RGB || iformat == 3) {
		format = GL_RGB;
		type = GL_UNSIGNED_BYTE;
	}
	else if (iformat == GL_LUMINANCE8_ALPHA8 || iformat == GL_LUMINANCE_ALPHA || iformat == 2) {
		format = GL_LUMINANCE_ALPHA;
		type = GL_UNSIGNED_BYTE;
	}
	else if (iformat == GL_LUMINANCE8 || iformat == GL_LUMINANCE16 || iformat == GL_LUMINANCE || iformat == 1) {
		format = GL_LUMINANCE;
		type = GL_UNSIGNED_BYTE;
	}
	else if (iformat == GL_DEPTH_COMPONENT24 || iformat == GL_DEPTH_COMPONENT) {
		format = GL_DEPTH_COMPONENT;
		type = GL_UNSIGNED_INT;
		attachment = GL_DEPTH_ATTACHMENT;
		filter = GL_NEAREST;
	}
	else if (iformat == GL_STENCIL_INDEX1 || iformat == GL_STENCIL_INDEX4 || iformat == GL_STENCIL_INDEX8 ||
		iformat == GL_STENCIL_INDEX16 || iformat == GL_STENCIL_INDEX) {
		format = GL_STENCIL_INDEX;
		type = GL_UNSIGNED_BYTE;
		attachment = GL_STENCIL_ATTACHMENT;
		filter = GL_NEAREST;
	}
	else if (iformat == GL_DEPTH24_STENCIL8 || iformat == GL_DEPTH_STENCIL) {
		format = GL_DEPTH_STENCIL;
		type = GL_UNSIGNED_INT_24_8;
		attachment = GL_DEPTH_STENCIL_ATTACHMENT;
		filter = GL_NEAREST;
	}
	else {
		throw std::invalid_argument("framebuffer::attach_texture: unrecognized internal format"+SPOT);
	}

	glGenTextures(1, &tex_id);
	glBindFramebuffer(GL_FRAMEBUFFER, frame_id_);
	glBindTexture(GL_TEXTURE_2D, tex_id);
	glTexImage2D(GL_TEXTURE_2D, 0, iformat, width_, height_, 0, format, type, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
	if (format == GL_DEPTH_COMPONENT) {
		glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
	}
	if (format == GL_DEPTH_STENCIL) { // packed depth and stencil added separately
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, tex_id, 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_STENCIL_ATTACHMENT, GL_TEXTURE_2D, tex_id, 0);
	}
	else {
		glFramebufferTexture2D(GL_FRAMEBUFFER, attachment, GL_TEXTURE_2D, tex_id, 0);
	}

	tex_id_.push_back(tex_id);
	buffers_[tex_id_.size() - 1] = attachment;
}

void framebuffer::bind_input() {
	for (int i = 0; i < int(tex_id_.size()); i++) {
		glActiveTexture(GL_TEXTURE0 + i);
		glBindTexture(GL_TEXTURE_2D, tex_id_[i]);
	}
}

void framebuffer::bind_output() {
	if (tex_id_.empty()) {
		throw std::domain_error("framebuffer::bind_output: no textures to bind" + SPOT);
	}

	glBindFramebuffer(GL_FRAMEBUFFER, frame_id_);
	if (tex_id_.size() == 1) {
		glDrawBuffer(buffers_[0]);
	}
	else {
		glDrawBuffers(static_cast<GLsizei>(tex_id_.size()), buffers_);
	}
}

void framebuffer::bind_tex(int num) {
	if (num + 1 > int(tex_id_.size())) {
		throw std::out_of_range("framebuffer::bind_tex: texture vector size exceeded" + SPOT);
	}

	glBindTexture(GL_TEXTURE_2D, tex_id_[num]);
}

void framebuffer::blit_to(framebuffer *dest, GLbitfield mask, GLenum filter) {
	int old_read, old_draw;

	glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &old_read);
	glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &old_draw);;

	if ((mask & GL_DEPTH_BUFFER_BIT) || (mask & GL_STENCIL_BUFFER_BIT)) {
		filter = GL_NEAREST;
	}

	glBindFramebuffer(GL_READ_FRAMEBUFFER, frame_id_);
	if (dest)
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, dest->frame_id_);
	else
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

	glBlitFramebuffer(0, 0, width_, height_, 0, 0, width_, height_, mask, filter);

	glBindFramebuffer(GL_READ_FRAMEBUFFER, old_read);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, old_draw);
}

void framebuffer::check() {
	GLenum status;

	glBindFramebuffer(GL_FRAMEBUFFER, frame_id_);
	status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (status != GL_FRAMEBUFFER_COMPLETE) {
		throw std::invalid_argument("framebuffer::check: status error"+SPOT);
	}
}

void framebuffer::unbind() {
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glDrawBuffer(GL_BACK);
}


} // harmont
