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
#include "config.hpp"

// common macros
#define SPOT (std::string(" (@") + __FILE__ + ":" + std::to_string(__LINE__) + ").")

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

typedef enum {POSITION, NORMAL, COLOR} data_field_t;

typedef Eigen::AlignedBox<float, 3> bbox_t;

template <typename... Args>
using callback_t = std::function<void (Args...)>;

typedef enum {SHADOW_GEOMETRY, DISPLAY_GEOMETRY}  pass_type_t;

template <typename Scalar>
inline Scalar eps() {
    return Eigen::NumTraits<Scalar>::epsilon();
}

template <typename Scalar>
inline Scalar tiny() {
    return Eigen::NumTraits<Scalar>::dummy_precision();
}

template <typename Derived>
void clamp(Eigen::DenseBase<Derived>& m, typename Derived::Scalar lower, typename Derived::Scalar upper) {
    typedef typename Eigen::DenseBase<Derived>::Index idx_t;
    for (idx_t i = 0; i < m.outerSize(); ++i) {
        for (idx_t j = 0; j < m.innerSize(); ++j) {
            if (m(i,j) < lower) m(i,j) = lower;
            if (m(i,j) > upper) m(i,j) = upper;
        }
    }
}

template <typename Scalar>
void clamp(Scalar& s, const Scalar& lower, const Scalar& upper) {
    if (s < lower) s = lower;
    if (s > upper) s = upper;
}

inline std::string gl_type_name(GLenum type) {
    switch (type) {
        case GL_BOOL: return "GL_BOOL"; break;
        case GL_FLOAT: return "GL_FLOAT"; break;
        case GL_FLOAT_VEC2: return "GL_FLOAT_VEC2"; break;
        case GL_FLOAT_VEC3: return "GL_FLOAT_VEC3"; break;
        case GL_FLOAT_VEC4: return "GL_FLOAT_VEC4"; break;
        case GL_INT: return "GL_INT"; break;
        case GL_INT_VEC2: return "GL_INT_VEC2"; break;
        case GL_INT_VEC3: return "GL_INT_VEC3"; break;
        case GL_INT_VEC4: return "GL_INT_VEC4"; break;
        case GL_UNSIGNED_INT: return "GL_UNSIGNED_INT"; break;
        case GL_UNSIGNED_INT_VEC2: return "GL_UNSIGNED_INT_VEC2"; break;
        case GL_UNSIGNED_INT_VEC3: return "GL_UNSIGNED_INT_VEC3"; break;
        case GL_UNSIGNED_INT_VEC4: return "GL_UNSIGNED_INT_VEC4"; break;
        case GL_UNSIGNED_INT_ATOMIC_COUNTER: return "GL_UNSIGNED_INT_ATOMIC_COUNTER"; break;
        case GL_FLOAT_MAT2: return "GL_FLOAT_MAT2"; break;
        case GL_FLOAT_MAT3: return "GL_FLOAT_MAT3"; break;
        case GL_FLOAT_MAT4: return "GL_FLOAT_MAT4"; break;
        case GL_FLOAT_MAT2x3: return "GL_FLOAT_MAT2x3"; break;
        case GL_FLOAT_MAT2x4: return "GL_FLOAT_MAT2x4"; break;
        case GL_FLOAT_MAT3x2: return "GL_FLOAT_MAT3x2"; break;
        case GL_FLOAT_MAT3x4: return "GL_FLOAT_MAT3x4"; break;
        case GL_FLOAT_MAT4x2: return "GL_FLOAT_MAT4x2"; break;
        case GL_FLOAT_MAT4x3: return "GL_FLOAT_MAT4x3"; break;
        default: break;
    }
    return "UNKNOWN_TYPE";
}

template<typename T>
struct gl_type_enum;

template <>
struct gl_type_enum<bool> {
	static constexpr GLenum value = GL_BOOL;
};

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

template <typename T, template <typename> class... Tests>
struct logical_and;

template <typename T, template <typename> class Test>
struct logical_and<T, Test> {
    static constexpr bool value = Test<T>::value;
};

template <typename T, template <typename> class Test, template <typename> class... Tests>
struct logical_and<T, Test, Tests...> {
    static constexpr bool value = Test<T>::value && logical_and<T, Tests...>::value;
};

template <typename T, template <typename> class... Tests>
struct logical_or;

template <typename T, template <typename> class Test>
struct logical_or<T, Test> {
    static constexpr bool value = Test<T>::value;
};

template <typename T, template <typename> class Test, template <typename> class... Tests>
struct logical_or<T, Test, Tests...> {
    static constexpr bool value = Test<T>::value || logical_or<T, Tests...>::value;
};

template <typename T, template <typename> class... Tests>
using require = typename std::enable_if<logical_and<typename std::decay<T>::type, Tests...>::value>::type;


} // harmont

#endif /* HARMONT_COMMON_HPP_ */
