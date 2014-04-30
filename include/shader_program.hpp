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

#ifndef HARMONT_SHADERPROGRAM_H_
#define HARMONT_SHADERPROGRAM_H_

#include <map>
#include <vector>

#include <boost/optional.hpp>
#include <boost/none.hpp>

#include "shader.hpp"

#include "common.hpp"

namespace harmont {

class shader_program {
	public:
		typedef std::shared_ptr<shader_program>        ptr;
		typedef std::weak_ptr<shader_program>          wptr;
		typedef std::shared_ptr<const shader_program>  const_ptr;
		typedef std::weak_ptr<const shader_program>    const_wptr;

	public:
		shader_program();

		template <int Stage>
		void add_shader(typename shader<Stage>::ptr shader);
		void add_shaders(std::string v_shader, std::string f_shader, std::string g_shader = "");

		void link(boost::optional<std::map<int, std::string>> output_map = boost::none);
		void use();

		void bind_attrib(GLuint pos, const GLchar* name);
		template <typename T>
		void set_uniform(const GLchar* name, const T& value);
		template <typename Scalar>
		void set_uniform(const GLchar* name, GLsizei count, const Scalar* values);
		void set_uniform_var1i(const GLchar* name, GLsizei count, const int* values);
		void set_texture(const GLchar* name, int unit);

		bool linked() const;

	protected:
		GLint uniform_location_(const GLchar* name);

		void print_log_();

	protected:
		GLuint                ref_;
		vertex_shader::ptr    v_shader_;
		fragment_shader::ptr  f_shader_;
		geometry_shader::ptr  g_shader_;
		GLint                 link_status_;
};


} // harmont

#endif /* HARMONT_SHADERPROGRAM_H_ */
