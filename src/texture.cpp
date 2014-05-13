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

#include <texture.hpp>


namespace harmont {


texture::texture() : id_(0) {
	glGenTextures(1, &id_);
}

texture::texture(GLenum iformat, int width, int height, GLfloat *pixels) : id_(0), bound_to_rb_(false) {
	glGenTextures(1, &id_);
	bind();
	load_(iformat, width, height, pixels);
}

texture::texture(GLenum iformat, int width, int height, GLubyte *pixels) : id_(0) {
	glGenTextures(1, &id_);
	bind();
	load_(iformat, width, height, pixels);
}

texture::~texture() {
	if (bound_to_rb_) glDeleteRenderbuffers(1, &id_);
	glDeleteTextures(1, &id_);
}

void texture::set_filtering(GLenum filter) {
	bind();
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
}

void texture::bind() {
	glBindTexture(GL_TEXTURE_2D, id_);
}

void texture::unbind() {
	glBindTexture(GL_TEXTURE_2D, 0);
}

void texture::bind_to_renderbuffer(GLuint attachment) {
	glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, attachment, GL_TEXTURE_2D, id_, 0);
	bound_to_rb_ = true;
}

GLuint texture::id() const {
	return id_;
}

void texture::load_(GLenum iformat, int width, int height, GLfloat *pixels) {
	GLenum format;

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

	if (iformat == GL_RGB16F || iformat == GL_RGB32F) {
		format = GL_RGB;
	}
	else if (iformat == GL_RGBA16F || iformat == GL_RGBA32F) {
		format = GL_RGBA;
	}
	else if (iformat == GL_RGBA8 || iformat == GL_RGBA || iformat == 4) {
		format = GL_RGBA;
	}
	else if (iformat == GL_RGB8 || iformat == GL_RGB || iformat == 3) {
		format = GL_RGB;
	}
	else if (iformat == GL_LUMINANCE8_ALPHA8 || iformat == GL_LUMINANCE_ALPHA || iformat == 2) {
		format = GL_LUMINANCE_ALPHA;
	}
	else if (iformat == GL_LUMINANCE8 || iformat == GL_LUMINANCE || iformat == 1) {
		format = GL_LUMINANCE;
	}
	else if (iformat == GL_DEPTH_COMPONENT || iformat == GL_DEPTH_COMPONENT16 ||
		iformat == GL_DEPTH_COMPONENT24 || iformat == GL_DEPTH_COMPONENT32 || iformat == GL_DEPTH_COMPONENT32F) {
		format = GL_DEPTH_COMPONENT;
	}
	else {
		throw std::invalid_argument("texture::load_: Unknown internal format"+SPOT);
	}

	glTexImage2D(GL_TEXTURE_2D, 0, iformat, width, height, 0, format, GL_FLOAT, pixels);
}

void texture::load_(GLenum iformat, int width, int height, GLubyte *pixels) {
	GLenum format;

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	if (iformat == GL_RGB8) {
		format = GL_RGB;
	}
	else if (iformat == GL_RGBA8) {
		format = GL_RGBA;
	}
	else {
		throw std::invalid_argument("texture::load_: Unknown internal format"+SPOT);
	}

	glTexImage2D(GL_TEXTURE_2D, 0, iformat, width, height, 0, format, GL_UNSIGNED_BYTE, pixels);
}


} // harmont
