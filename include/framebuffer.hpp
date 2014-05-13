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

#ifndef HARMONT_FRAMEBUFFER_H
#define HARMONT_FRAMEBUFFER_H


#include <vector>
#include "common.hpp"


namespace harmont {

class framebuffer {
	public:
		typedef std::shared_ptr<framebuffer> ptr;
		typedef std::weak_ptr<framebuffer>   wptr;
		typedef std::shared_ptr<const framebuffer> const_ptr;
		typedef std::weak_ptr<const framebuffer>   const_wptr;

	public:
		framebuffer();
		framebuffer(int width, int height);
		~framebuffer();

		void resize(int width, int height);
		void attach_render(GLenum iformat);
		void attach_texture(GLenum iformat, GLint filter = GL_LINEAR);
		void bind_input();
		void bind_output();
		void bind_tex(int num = 0);
		void blit_to(framebuffer *dest, GLbitfield mask, GLenum filter = GL_LINEAR);
		void check();

		static void unbind();

	protected:
		int width_;
		int height_;
		GLuint frame_id_;
		GLuint depth_id_;
		GLuint stencil_id_;
		std::vector<GLuint> tex_id_;
		GLenum* buffers_;

		int max_color_attachments_;
};


} // harmont

#endif // HARMONT_FRAMEBUFFER_H
