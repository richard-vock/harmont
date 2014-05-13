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

#ifndef HARMONT_TEXTURE_H
#define HARMONT_TEXTURE_H

#include "common.hpp"


namespace harmont {


class texture {
	public:
		typedef std::shared_ptr<texture>        ptr;
		typedef std::weak_ptr<texture>          wptr;
		typedef std::shared_ptr<const texture>  const_ptr;
		typedef std::weak_ptr<const texture>    const_wptr;

	public:
		texture();
		texture(GLenum iformat, int width, int height, GLfloat*pixels);
		texture(GLenum iformat, int width, int height, GLubyte*pixels);
		~texture();

		void set_filtering(GLenum filter);
		void bind();
		static void unbind();

		void bind_to_renderbuffer(GLuint attachment);

		GLuint id() const;

	protected:
		void load_(GLenum iformat, int width, int height, GLfloat*pixels);
		void load_(GLenum iformat, int width, int height, GLubyte*pixels);

	protected:
		GLuint  id_;
		bool    bound_to_rb_;
};


} // harmont


#endif // HARMONT_TEXTURE_H
