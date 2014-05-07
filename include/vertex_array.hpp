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

#ifndef HARMONT_VERTEX_ARRAY_H_
#define HARMONT_VERTEX_ARRAY_H_

#include "data_buffer.hpp"

namespace harmont {

class vertex_array {
	public:
		vertex_array();
		virtual ~vertex_array();

		void init();

		void bind() const;
		void release() const;

		void set(GLuint pos, GLuint dim, GLuint type = GL_FLOAT) const;
		void enable(int pos) const;
		void disable(int pos) const;

	protected:
		GLuint  id_;
};

} // harmont

#endif /* HARMONT_VERTEX_ARRAY */
