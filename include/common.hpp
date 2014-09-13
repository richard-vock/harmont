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

#ifndef HARMONT_COMMON_HPP_
#define HARMONT_COMMON_HPP_

#include <iostream>
#include <memory>
#include <stdexcept>
#include <cassert>

// common includes
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <Eigen/Dense>

// common macros
#define SPOT (std::string(" (@__FILE__:__LINE__)."))

#ifdef NDEBUG
#define ASSERTS(expr, msg)
#else
#define ASSERTS(expr, msg)                      \
		if (!(expr)) {                              \
	 		std::cout << (msg) << SPOT << std::endl; \
			std::abort();                          \
		}
#endif

namespace harmont {

template <typename Scalar>
Scalar eps() {
    return Eigen::NumTraits<Scalar>::epsilon();
}

template <typename Scalar>
Scalar tiny() {
    return Eigen::NumTraits<Scalar>::dummy_precision();
}

template<typename T>
struct gl_type_enum;

template <>
struct gl_type_enum<int8_t> {
	static constexpr GLenum value = GL_BYTE;
};

template <>
struct gl_type_enum<uint8_t> {
	static constexpr GLenum value = GL_UNSIGNED_BYTE;
};

template <>
struct gl_type_enum<int16_t> {
	static constexpr GLenum value = GL_SHORT;
};

template <>
struct gl_type_enum<uint16_t> {
	static constexpr GLenum value = GL_UNSIGNED_SHORT;
};

template <>
struct gl_type_enum<int32_t> {
	static constexpr GLenum value = GL_INT;
};

template <>
struct gl_type_enum<uint32_t> {
	static constexpr GLenum value = GL_UNSIGNED_INT;
};

template <>
struct gl_type_enum<float> {
	static constexpr GLenum value = GL_FLOAT;
};

template <>
struct gl_type_enum<double> {
	static constexpr GLenum value = GL_DOUBLE;
};

typedef std::function<void (GLuint, GLint, GLsizei, const GLvoid*)> gl_attrib_func_t;
template<typename T>
struct gl_attrib_func;

template <>
struct gl_attrib_func<int8_t> {
    static gl_attrib_func_t get_functor() {
        return [&] (GLuint index, GLint size, GLsizei stride, const GLvoid* pointer) {
            glVertexAttribIPointer(index, size, GL_BYTE, stride, pointer);
        };
    }
};

template <>
struct gl_attrib_func<uint8_t> {
    static gl_attrib_func_t get_functor() {
        return [&] (GLuint index, GLint size, GLsizei stride, const GLvoid* pointer) {
            glVertexAttribIPointer(index, size, GL_UNSIGNED_BYTE, stride, pointer);
        };
    }
};

template <>
struct gl_attrib_func<int16_t> {
    static gl_attrib_func_t get_functor() {
        return [&] (GLuint index, GLint size, GLsizei stride, const GLvoid* pointer) {
            glVertexAttribIPointer(index, size, GL_SHORT, stride, pointer);
        };
    }
};

template <>
struct gl_attrib_func<uint16_t> {
    static gl_attrib_func_t get_functor() {
        return [&] (GLuint index, GLint size, GLsizei stride, const GLvoid* pointer) {
            glVertexAttribIPointer(index, size, GL_UNSIGNED_SHORT, stride, pointer);
        };
    }
};

template <>
struct gl_attrib_func<int32_t> {
    static gl_attrib_func_t get_functor() {
        return [&] (GLuint index, GLint size, GLsizei stride, const GLvoid* pointer) {
            glVertexAttribIPointer(index, size, GL_INT, stride, pointer);
        };
    }
};

template <>
struct gl_attrib_func<uint32_t> {
    static gl_attrib_func_t get_functor() {
        return [&] (GLuint index, GLint size, GLsizei stride, const GLvoid* pointer) {
            glVertexAttribIPointer(index, size, GL_UNSIGNED_INT, stride, pointer);
        };
    }
};

template <>
struct gl_attrib_func<float> {
    static gl_attrib_func_t get_functor() {
        return [&] (GLuint index, GLint size, GLsizei stride, const GLvoid* pointer) {
            glVertexAttribPointer(index, size, GL_FLOAT, GL_FALSE, stride, pointer);
        };
    }
};

template <>
struct gl_attrib_func<double> {
    static gl_attrib_func_t get_functor() {
        return [&] (GLuint index, GLint size, GLsizei stride, const GLvoid* pointer) {
            glVertexAttribPointer(index, size, GL_DOUBLE, GL_FALSE, stride, pointer);
        };
    }
};

} // harmont

#endif /* HARMONT_COMMON_HPP_ */
