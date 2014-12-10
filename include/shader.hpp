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

#ifndef HARMONT_SHADER_H_
#define HARMONT_SHADER_H_

#include <string>
#include <memory>
#include <map>

#include "common.hpp"


namespace harmont {


template <int Stage>
class shader {
	public:
		typedef std::shared_ptr<shader<Stage>>        ptr;
		typedef std::weak_ptr<shader<Stage>>          wptr;
		typedef std::shared_ptr<const shader<Stage>>  const_ptr;
		typedef std::weak_ptr<const shader<Stage>>    const_wptr;
        typedef std::map<std::string, std::string>    parameters_t;

	public:
		static ptr from_file(std::string filename, bool compile_now = true);
		static ptr from_source(std::string source, bool compile_now = true);
		static ptr from_file(std::string filename, const parameters_t& params, bool compile_now = true);
		static ptr from_source(std::string source, const parameters_t& params, bool compile_now = true);
		virtual ~shader();

		constexpr GLenum stage() { return Stage; };

		GLuint handle();

	protected:
		shader(std::string source, bool compile_now);

		void compile_();
		void print_log_();

        static std::string load_file_(const std::string& filename);

        static std::string render_source_(const std::string& source, const parameters_t& params);

	protected:
		GLuint      handle_;
        std::string source_;
		GLint       compile_status_;
		GLint       link_status_;
};


typedef shader<GL_VERTEX_SHADER>    vertex_shader;
typedef shader<GL_FRAGMENT_SHADER>  fragment_shader;
typedef shader<GL_GEOMETRY_SHADER>  geometry_shader;

} // harmont

#endif /* HARMONT_SHADER_H_ */
