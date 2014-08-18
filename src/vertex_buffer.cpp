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

#include <vertex_buffer.hpp>

namespace harmont {


template <typename Scalar, GLenum Target>
vertex_buffer<Scalar, Target>::vertex_buffer(uint32_t element_count, GLenum usage) : element_count_(element_count), usage_(usage), data_size_(element_count * sizeof(Scalar)) {
    handle_ = glGenBuffers(1);
}

template <typename Scalar, GLenum Target>
vertex_buffer<Scalar, Target>::~vertex_buffer() {
}

template <typename Scalar, GLenum Target>
ptr vertex_buffer<Scalar, Target>::from_layout(const layout_t& layout) {
}

template <typename Scalar, GLenum Target>
ptr vertex_buffer<Scalar, Target>::from_data(const Scalar* data, uint32_t element_count) {
}

template <typename Scalar, GLenum Target>
GLuint vertex_buffer<Scalar, Target>::handle() const {
    return handle_;
}

template <typename Scalar, GLenum Target>
GLenum vertex_buffer<Scalar, Target>::usage() const {
    return usage_;
}

template <typename Scalar, GLenum Target>
uint32_t vertex_buffer<Scalar, Target>::element_count() const {
    return element_count_;
}

template <typename Scalar, GLenum Target>
uint32_t vertex_buffer<Scalar, Target>::data_size() const {
    return data_size_;
}

template <typename Scalar, GLenum Target>
bool vertex_buffer<Scalar, Target>::bound() const {
    return bound_buffer() == handle_;
}

template <typename Scalar, GLenum Target>
GLuint vertex_buffer<Scalar, Target>::bound_buffer() {
}

template <typename Scalar, GLenum Target>
void vertex_buffer<Scalar, Target>::bind() {
}

template <typename Scalar, GLenum Target>
void vertex_buffer<Scalar, Target>::release() {
}

template <typename Scalar, GLenum Target>
void vertex_buffer<Scalar, Target>::bind_to_array(const layout& layout, shader_program::ptr program) {
}

template <typename Scalar, GLenum Target>
void vertex_buffer<Scalar, Target>::bind_to_array(const layout& layout, render_pass::ptr pass) {
}

template <typename Scalar, GLenum Target>
void vertex_buffer<Scalar, Target>::set_data(const Scalar* data, uint32_t element_count) {
}

template <typename Scalar, GLenum Target>
void vertex_buffer<Scalar, Target>::get_data(Scalar* data) {
}


} // harmont


// instantiate
#define SUPPORTED_TYPES \
	X(float)     \
	X(double)    \
	X(int)       \
	X(uint32_t)

#define X(type) \
	template class harmont::vertex_buffer<type, 2>; \
	template class harmont::vertex_buffer<type, 3>; \
	template class harmont::vertex_buffer<type, 4>;
SUPPORTED_TYPES
#undef X
#undef SUPPORTED_TYPES
