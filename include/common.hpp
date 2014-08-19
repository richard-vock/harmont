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
#include <assert>

// common includes
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>

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

template<typename T>
struct gl_type_enum;

typedef std::function<void (GLuint, GLint, GLenum, GLsizei, const GLvoid*)> gl_attrib_func_t;
template<typename T>
struct gl_attrib_func;

} // harmont

#endif /* HARMONT_COMMON_HPP_ */
