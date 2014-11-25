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
#include "common.hpp"

#include <stdexcept>
#include <cassert>


namespace harmont {

static GLenum infer_targets(int dim) {
	switch(dim) {
		case 1: return GL_TEXTURE_1D; break;
		case 2: return GL_TEXTURE_2D; break;
		case 3: return GL_TEXTURE_3D; break;
		default: throw std::runtime_error("texture: Invalid texture dimension. Must be in either 1, 2 or 3.");
	}
}

static GLenum infer_internal_format(int channels) {
	switch(channels) {
		case 1: return GL_RED; break;
		case 2: return GL_RG; break;
		case 3: return GL_RGB; break;
		case 4: return GL_RGBA; break;
		default: throw std::runtime_error("texture: Invalid number of texture channels. Must be in [1,4].");
	}
}


template <typename Scalar>
texture::ptr texture::texture_1d(int width, int channels, const Scalar* data, GLenum min_filter, GLenum mag_filter, GLenum wrap_s, GLenum wrap_t) {
	GLenum scalar_type = gl_type_enum<Scalar>::value;
	GLenum internal_format = infer_internal_format(channels);
	parameters_t_ params = {min_filter, mag_filter, wrap_s, wrap_t};
    return ptr(new texture(scalar_type, internal_format, width, 0, 0, data, params));
}

template <typename Scalar>
texture::ptr texture::texture_2d(int width, int height, int channels, const Scalar* data, GLenum min_filter, GLenum mag_filter, GLenum wrap_s, GLenum wrap_t) {
	GLenum scalar_type = gl_type_enum<Scalar>::value;
	GLenum internal_format = infer_internal_format(channels);
	parameters_t_ params = {min_filter, mag_filter, wrap_s, wrap_t};
    return ptr(new texture(scalar_type, internal_format, width, height, 0, data, params));
}

template <typename Scalar>
texture::ptr texture::texture_3d(int width, int height, int depth, int channels, const Scalar* data, GLenum min_filter, GLenum mag_filter, GLenum wrap_s, GLenum wrap_t) {
	GLenum scalar_type = gl_type_enum<Scalar>::value;
	GLenum internal_format = infer_internal_format(channels);
	parameters_t_ params = {min_filter, mag_filter, wrap_s, wrap_t};
    return ptr(new texture(scalar_type, internal_format, width, height, depth, data, params));
}

template <typename Scalar>
texture::ptr texture::depth_texture(int width, int height, GLenum format) {
	GLenum scalar_type = gl_type_enum<Scalar>::value;
	parameters_t_ params = {GL_NEAREST, GL_NEAREST, GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE};
    return ptr(new texture(scalar_type, format, width, height, 0, (const Scalar*)nullptr, params, true));
}

texture::~texture() {
}

GLuint texture::handle() const {
	return handle_;
}

int texture::width() const {
	return width_;
}

int texture::height() const {
	return height_;
}

int texture::depth() const {
	return depth_;
}

GLenum texture::target() const {
	return target_;
}

int texture::dim() const {
	return dims_;
}

int texture::size() const {
	return width_ * (height_ ? height_ : 1) * (depth_ ? depth_ : 1);
}

bool texture::is_depth_attachment() const {
	return is_depth_attachment_;
}

void texture::resize(int width, int height, int depth) {
	int dims = !!width + !!height + !!depth;
	ASSERTS(dims == dims_, "texture::resize: Invalid argument count. Must match texture dimension"+SPOT)

	bind();
	width_ = width;
	height_ = height;
	depth_ = depth;
	switch (dims) {
		case 1: glTexImage1D(target_, 0, internal_format_, width_, 0, internal_format_, scalar_type_, nullptr); break;
		case 2: glTexImage2D(target_, 0, internal_format_, width_, height_, 0, internal_format_, scalar_type_, nullptr); break;
		case 3: glTexImage3D(target_, 0, internal_format_, width_, height_, depth_, 0, internal_format_, scalar_type_, nullptr); break;
		default: break;
	}
	release();
}

void texture::set_min_filter(GLenum filter) {
	if (filter != GL_NEAREST && filter != GL_LINEAR && filter != GL_NEAREST_MIPMAP_NEAREST && filter != GL_LINEAR_MIPMAP_NEAREST && filter != GL_NEAREST_MIPMAP_LINEAR && filter != GL_LINEAR_MIPMAP_LINEAR) {
		throw std::runtime_error("texture::set_min_filter: Invalid filter method");
	}
	glTexParameteri(target_, GL_TEXTURE_MIN_FILTER, filter);
	params_.min_filter = filter;
}

void texture::set_mag_filter(GLenum filter) {
	if (filter != GL_NEAREST && filter != GL_LINEAR && filter != GL_NEAREST_MIPMAP_NEAREST && filter != GL_LINEAR_MIPMAP_NEAREST && filter != GL_NEAREST_MIPMAP_LINEAR && filter != GL_LINEAR_MIPMAP_LINEAR) {
		throw std::runtime_error("texture::set_mag_filter: Invalid filter method");
	}
	glTexParameteri(target_, GL_TEXTURE_MAG_FILTER, filter);
	params_.mag_filter = filter;
}

void texture::set_filter(GLenum filter) {
	if (filter != GL_NEAREST && filter != GL_LINEAR && filter != GL_NEAREST_MIPMAP_NEAREST && filter != GL_LINEAR_MIPMAP_NEAREST && filter != GL_NEAREST_MIPMAP_LINEAR && filter != GL_LINEAR_MIPMAP_LINEAR) {
		throw std::runtime_error("texture::set_filter: Invalid filter method");
	}
	glTexParameteri(target_, GL_TEXTURE_MIN_FILTER, filter);
	glTexParameteri(target_, GL_TEXTURE_MAG_FILTER, filter);
	params_.min_filter = filter;
	params_.mag_filter = filter;
}

void texture::set_wrap_s(GLenum mode) {
	if (mode != GL_CLAMP_TO_EDGE && mode != GL_CLAMP_TO_BORDER && mode != GL_MIRRORED_REPEAT && mode != GL_REPEAT) {
		throw std::runtime_error("texture::set_wrap_s: Invalid wrap mode");
	}
	glTexParameteri(target_, GL_TEXTURE_WRAP_S, mode);
	params_.wrap_s = mode;
}

void texture::set_wrap_t(GLenum mode) {
	if (mode != GL_CLAMP_TO_EDGE && mode != GL_CLAMP_TO_BORDER && mode != GL_MIRRORED_REPEAT && mode != GL_REPEAT) {
		throw std::runtime_error("texture::set_wrap_t: Invalid wrap mode");
	}
	glTexParameteri(target_, GL_TEXTURE_WRAP_T, mode);
	params_.wrap_t = mode;
}

void texture::set_wrap(GLenum mode) {
	if (mode != GL_CLAMP_TO_EDGE && mode != GL_CLAMP_TO_BORDER && mode != GL_MIRRORED_REPEAT && mode != GL_REPEAT) {
		throw std::runtime_error("texture::set_wrap: Invalid wrap mode");
	}
	glTexParameteri(target_, GL_TEXTURE_WRAP_S, mode);
	glTexParameteri(target_, GL_TEXTURE_WRAP_T, mode);
	params_.wrap_s = mode;
	params_.wrap_t = mode;
}

template <typename Scalar>
void texture::set_data(const Scalar* data) {
	GLenum format = internal_format_;
    if (is_depth_attachment_) format = GL_DEPTH_COMPONENT;
	switch (dims_) {
		case 1: glTexSubImage1D(target_, 0, 0, width_, format, scalar_type_, data); break;
		case 2: glTexSubImage2D(target_, 0, 0, 0, width_, height_, format, scalar_type_, data); break;
		case 3: glTexSubImage3D(target_, 0, 0, 0, 0, width_, height_, depth_, format, scalar_type_, data); break;
		default: break;
	}
}

void texture::bind() {
	glBindTexture(target_, handle_);
}

void texture::release() {
	glBindTexture(target_, 0);
}

GLint texture::active_unit() {
	GLint unit;
	glGetIntegerv(GL_ACTIVE_TEXTURE, &unit);
	return unit;
}

GLint texture::active_texture(GLenum target, GLenum unit) {
	GLint active = active_unit();
	if (active != static_cast<GLint>(unit)) glActiveTexture(unit);

	GLenum binding;
	switch (target) {
		case GL_TEXTURE_1D: binding = GL_TEXTURE_BINDING_1D; break;
		case GL_TEXTURE_2D: binding = GL_TEXTURE_BINDING_2D; break;
		case GL_TEXTURE_3D: binding = GL_TEXTURE_BINDING_3D; break;
		default: throw std::runtime_error("texture::active_texture: Invalid target.");
	}

	GLint handle;
	glGetIntegerv(binding, &handle);


	if (active != static_cast<GLint>(unit)) glActiveTexture(active);

	ASSERTS(handle >= 0, "texture::active_texture: Negative handle for active texture"+SPOT);
	return static_cast<GLuint>(handle);
}

template <typename Scalar>
texture::texture(GLenum scalar_type, GLenum internal_format, int width, int height, int depth, const Scalar* data, parameters_t_ params, bool is_depth_attachment) :
	width_(width),
	height_(height),
	depth_(depth),
	scalar_type_(scalar_type),
	internal_format_(internal_format),
	params_(params),
	is_depth_attachment_(is_depth_attachment) {
	dims_ = !!width_ + !!height_ + !!depth_;
	target_ = infer_targets(dims_);
	glGenTextures(1, &handle_);

	bind();
	set_min_filter(params.min_filter);
	set_mag_filter(params.mag_filter);
	set_wrap_s(params.wrap_s);
	set_wrap_t(params.wrap_t);
	allocate_();
	if (data) set_data(data);
	release();
}

void texture::allocate_() {
	GLenum format = internal_format_;
    if (is_depth_attachment()) format = GL_DEPTH_COMPONENT;
	switch (dims_) {
		case 1: glTexImage1D(target_, 0, internal_format_, width_, 0, format, scalar_type_, nullptr); break;
		case 2: glTexImage2D(target_, 0, internal_format_, width_, height_, 0, format, scalar_type_, nullptr); break;
		case 3: glTexImage3D(target_, 0, internal_format_, width_, height_, depth_, 0, format, scalar_type_, nullptr); break;
		default: break;
	}
}


#define TEXTURE_SCALAR_INSTANTIATE(type) \
	template texture::ptr texture::texture_1d(int width, int channels, const type* data, GLenum min_filter, GLenum mag_filter, GLenum wrap_s, GLenum wrap_t); \
	template texture::ptr texture::texture_2d(int width, int height, int channels, const type* data, GLenum min_filter, GLenum mag_filter, GLenum wrap_s, GLenum wrap_t); \
	template texture::ptr texture::texture_3d(int width, int height, int depth, int channels, const type* data, GLenum min_filter, GLenum mag_filter, GLenum wrap_s, GLenum wrap_t); \
	template texture::ptr texture::depth_texture<type>(int width, int height, GLenum format); \
	template texture::texture(GLenum scalar_type, GLenum internal_format, int width, int height, int depth, const type* data, parameters_t_ params, bool is_depth_attachment); \
	template void texture::set_data(const type* data);
#include <texture_scalars.def>


} // harmont
