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

#include "common.hpp"

namespace harmont {

class vertex_array {
    public:
        typedef std::shared_ptr<vertex_array>       ptr;
        typedef std::weak_ptr<vertex_array>         wptr;
        typedef std::shared_ptr<const vertex_array> const_ptr;
        typedef std::weak_ptr<const vertex_array>   const_wptr;

	public:
		vertex_array();
		virtual ~vertex_array();

        GLuint handle() const;

		void bind() const;
		void release() const;

	protected:
		GLuint  handle_;
};

} // harmont

#endif /* HARMONT_VERTEX_ARRAY */
