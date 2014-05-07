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


vertex_array::vertex_array() : id_(0) {
}

vertex_array::~vertex_array() {
	if (id_) glDeleteVertexArrays(1, &id_);
	id_ = 0;
}

void vertex_array::init() {
	glGenVertexArrays(1, &id_);
}

void vertex_array::bind() const {
	glBindVertexArray(id_);
}

void vertex_array::release() const {
	glBindVertexArray(0);
}

void vertex_array::set(GLuint pos, GLuint dim, GLuint type) const {
	glVertexAttribPointer(pos, dim, type, GL_FALSE, 0, 0);
}

void vertex_array::enable(int pos) const {
	if (pos < 0) throw std::runtime_error("vertex_array::enable: Invalid attribe index");
	glEnableVertexAttribArray((GLuint)pos);
}

void vertex_array::disable(int pos) const {
	if (pos < 0) throw std::runtime_error("vertex_array::disable: Invalid attribe index");
	glDisableVertexAttribArray((GLuint)pos);
}


} // harmont
