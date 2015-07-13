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

#define HARMONT_BUFFER_OFFSET(i) ((const GLvoid*)((const GLchar*)NULL + (i)))

namespace harmont {


template <typename Scalar, GLenum Target>
vertex_buffer<Scalar, Target>::vertex_buffer(uint32_t element_count, GLenum usage) : element_count_(element_count), usage_(usage), data_size_(element_count * sizeof(Scalar)) {
    glGenBuffers(1, &handle_);
    bind();
    allocate_();
    release();
}

template <typename Scalar, GLenum Target>
vertex_buffer<Scalar, Target>::~vertex_buffer() {
	glDeleteBuffers(1, &handle_);
}

template <typename Scalar, GLenum Target>
typename vertex_buffer<Scalar, Target>::ptr vertex_buffer<Scalar, Target>::from_layout(const layout_t& layout, GLenum usage) {
    uint32_t element_count = 0;
    for (const auto& el : layout) {
        element_count += el.second;
    }
    return ptr(new vertex_buffer<Scalar, Target>(element_count, usage));
}

template <typename Scalar, GLenum Target>
typename vertex_buffer<Scalar, Target>::ptr vertex_buffer<Scalar, Target>::from_data(const Scalar* data, uint32_t element_count, GLenum usage) {
    ptr vbo = std::make_shared<vertex_buffer<Scalar, Target>>(element_count, usage);
    vbo->bind();
    vbo->set_data(data, element_count);
    vbo->release();
    return vbo;
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
    GLint buffer;
    GLenum query;
    switch (Target) {
        case GL_TEXTURE_BUFFER:       query = GL_TEXTURE_BINDING_BUFFER; break;
        case GL_ELEMENT_ARRAY_BUFFER: query = GL_ELEMENT_ARRAY_BUFFER_BINDING; break;
        default: query = GL_ARRAY_BUFFER_BINDING; break;
    }
    glGetIntegerv(query, &buffer);
    return buffer;
}

template <typename Scalar, GLenum Target>
void vertex_buffer<Scalar, Target>::bind() {
    if (bound()) return;
    glBindBuffer(Target, handle_);
}

template <typename Scalar, GLenum Target>
void vertex_buffer<Scalar, Target>::release() {
    if (!bound()) {
        throw std::runtime_error("vertex_buffer::release(): Buffer not bound." + SPOT);
    }
    glBindBuffer(Target, 0);
}

template <typename Scalar, GLenum Target>
void vertex_buffer<Scalar, Target>::bind_to_array(const layout_t& layout, shader_program::ptr program) {
    bool was_bound = bound();
    if (!was_bound) {
        bind();
    }
    int offset = 0;
    int size = 0;
    for (const auto& field : layout) {
        size += field.second;
    }
    size *= scalar_size;
    for (const auto& field : layout) {
        GLuint location = (*program)[field.first].location();
        glEnableVertexAttribArray(location);
        gl_attrib_func_t func = gl_attrib_func<Scalar>::get_functor();
        func(location, field.second, size, HARMONT_BUFFER_OFFSET(offset));
        offset += field.second * scalar_size;
    }
    if (!was_bound) {
        release();
    }
}

template <typename Scalar, GLenum Target>
void vertex_buffer<Scalar, Target>::bind_to_array(const layout_t& layout, render_pass::ptr pass) {
    bind_to_array(layout, pass->program());
}

template <typename Scalar, GLenum Target>
void vertex_buffer<Scalar, Target>::set_data(const Scalar* data, uint32_t element_count) {
    bool was_bound = bound();
    if (!was_bound) bind();
    if (element_count != element_count_) {
        element_count_ = element_count;
        data_size_ = element_count * sizeof(Scalar);
        allocate_();
    }
    glBufferSubData(Target, 0, data_size_, reinterpret_cast<const GLvoid*>(data));
    if (!was_bound) release();
}

template <typename Scalar, GLenum Target>
void vertex_buffer<Scalar, Target>::get_data(Scalar* data) {
    if (!bound()) {
        throw std::runtime_error("vertex_buffer::get_data(): Buffer not bound." + SPOT);
    }
    glGetBufferSubData(Target, 0, data_size_, reinterpret_cast<GLvoid*>(data));
}

template <typename Scalar, GLenum Target>
void vertex_buffer<Scalar, Target>::allocate_() {
    if (!bound()) {
        throw std::runtime_error("vertex_buffer::allocate(): Buffer not bound." + SPOT);
    }
    glBufferData(Target, data_size_, nullptr, usage_);
}


} // harmont


// instantiate
#define HARMONT_VBO_INSTANTIATE(scalar) \
    template class harmont::vertex_buffer<scalar, GL_ARRAY_BUFFER>; \
    template class harmont::vertex_buffer<scalar, GL_ELEMENT_ARRAY_BUFFER>; \
    template class harmont::vertex_buffer<scalar, GL_TEXTURE_BUFFER>;
#include <vbo_instantiate.def>
