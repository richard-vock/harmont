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
#include <tuple>

#include "common.hpp"
#include "shader.hpp"
#include "uniform_dispatch.hpp"


namespace harmont {

class shader_program {
	public:
		typedef std::shared_ptr<shader_program>        ptr;
		typedef std::weak_ptr<shader_program>          wptr;
		typedef std::shared_ptr<const shader_program>  const_ptr;
		typedef std::weak_ptr<const shader_program>    const_wptr;
        typedef enum {ATTRIBUTE, UNIFORM}              variable_type;
        class variable_t;
        typedef std::shared_ptr<variable_t>            variable_ptr;
        typedef std::map<std::string, variable_ptr>    variables;

	public:
		shader_program(vertex_shader::ptr vs, fragment_shader::ptr fs, bool link_now = true);
		shader_program(vertex_shader::ptr vs, fragment_shader::ptr fs, geometry_shader::ptr gs, bool link_now = true);
        virtual ~shader_program();

        GLuint handle() const;

        void link();
        bool linked() const;

        void bind();
        void release();
        bool bound() const;

        const variable_t& operator[](std::string name) const;
        const variable_t& variable(std::string name) const;

	protected:
        template <int Stage>
        void attach_shader_(typename shader<Stage>::ptr shader);

		void print_log_();

	protected:
		GLuint                handle_;
		vertex_shader::ptr    vs_;
		fragment_shader::ptr  fs_;
		geometry_shader::ptr  gs_;
		GLint                 link_status_;
        variables             uniforms_;
        variables             attributes_;
};

class shader_program::variable_t {
    public:
        typedef std::shared_ptr<variable_t> Ptr;
        typedef std::weak_ptr<variable_t> WPtr;
        typedef std::shared_ptr<const variable_t> ConstPtr;
        typedef std::weak_ptr<const variable_t> ConstWPtr;
        typedef std::tuple<std::string, GLenum, GLint> description_t;

    public:
        variable_t(GLuint program, const description_t& desc, variable_type var_type);
        variable_t(const variable_t& other);
        variable_t& operator=(const variable_t& other);
        variable_t& operator=(variable_t&& other);
        virtual ~variable_t();

        variable_type type() const;
        const std::string& name() const;
        GLenum data_type() const;
        GLint size() const;

        bool is_uniform() const;
        bool is_attribute() const;

        GLuint location() const;

        template <typename T>
        void set(T&& value) const;

        template <typename T>
        void operator =(T&& value) const;

        //template <typename T>
        //T get() const;

    protected:
        GLuint        program_;
        description_t desc_;
        variable_type var_type_;
        std::string   name_;
        GLenum        data_type_;
        GLint         size_;
        GLint         location_;
};

#include <shader_program.ipp>

} // harmont

#endif /* HARMONT_SHADERPROGRAM_H_ */
