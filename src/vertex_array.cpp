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

#include <vertex_array.hpp>


namespace harmont {


vertex_array::vertex_array() {
    glGenVertexArrays(1, &handle_);
}

vertex_array::~vertex_array() {
}

GLuint vertex_array::handle() const {
    return handle_;
}

void vertex_array::bind() const {
	glBindVertexArray(handle_);
}

void vertex_array::release() const {
	glBindVertexArray(0);
}

void vertex_array::set(GLuint pos, GLuint dim, GLuint type) const {
	glVertexAttribPointer(pos, dim, type, GL_FALSE, 0, 0);
}


} // harmont
