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

#include "texture.hpp"

namespace harmont {

class framebuffer {
	public:
		typedef std::shared_ptr<framebuffer>       ptr;
		typedef std::weak_ptr<framebuffer>         wptr;
		typedef std::shared_ptr<const framebuffer> const_ptr;
		typedef std::weak_ptr<const framebuffer>   const_wptr;
        typedef std::vector<texture::ptr>          textures;
        typedef std::pair<texture::ptr, GLuint>    named_texture;
        typedef std::vector<named_texture>         named_textures;

	public:
		framebuffer(const textures& output_textures = textures(), texture::ptr depth_texture = nullptr);
		~framebuffer();

        GLuint handle() const;

        textures outputs();
        const textures& outputs() const;

        textures::ptr depth_texture();
        textures::const_ptr depth_texture() const;

        void bind(const named_textures& input_textures = named_textures(), bool only_bind_input = false);
        void release();

    protected:
        static void bind_texture_(texture::const_ptr tex, GLenum attachment);
        static void check_();

	protected:
        textures        output_;
        texture::ptr    depth_;
        GLuint          handle_;
        named_textures  last_inputs_;
};


} // harmont

#endif // HARMONT_FRAMEBUFFER_H
