#ifndef RENDER_PASS_HPP_
#define RENDER_PASS_HPP_

#include <memory>
#include <vector>
#include <string>

#include "common.hpp"
#include "framebuffer.hpp"
#include "shader_program.hpp"


namespace harmont {


class render_pass {
    public:
        typedef std::shared_ptr<render_pass>         ptr;
        typedef std::weak_ptr<render_pass>           wptr;
        typedef std::shared_ptr<const render_pass>   const_ptr;
        typedef std::weak_ptr<const render_pass>     const_wptr;
        typedef std::vector<std::string>             shader_sources;
        typedef framebuffer::textures                textures;
        typedef std::pair<texture::ptr, std::string> named_texture;
        typedef std::vector<named_texture>           named_textures;
        typedef shader_program::variable_t           shader_variable;
        template <typename T>
        using named_uniform = std::pair<std::string, T>;


    public:
        render_pass(vertex_shader::ptr vs, fragment_shader::ptr fs, const textures& outputs = textures(), texture::ptr depth_texture = nullptr);
        render_pass(vertex_shader::ptr vs, fragment_shader::ptr fs, geometry_shader::ptr gs, const textures& outputs = textures(), texture::ptr depth_texture = nullptr);
        virtual ~render_pass();

        shader_program::ptr program();
        shader_program::const_ptr program() const;

        textures& outputs();
        const textures& outputs() const;

        texture::ptr depth_texture();
        texture::const_ptr depth_texture() const;

        framebuffer::ptr fbo();
        framebuffer::const_ptr fbo() const;

        template <typename Func>
        void render(Func&& draw_call, const named_textures& inputs = named_textures(), bool clear_depth_buffer = true);

        void bind_program();
        void release_program();

        shader_variable operator[](std::string name);
        shader_variable variable(std::string name);

        template <typename... Args>
        void set_uniforms(named_uniform<Args>... uniforms);

    protected:
        template <typename Arg, typename... Args>
        void set_uniforms_(named_uniform<Arg> uniform, named_uniform<Args>... uniforms);
        void set_uniforms_();

    protected:
        shader_program::ptr  program_;
        framebuffer::ptr     fbo_;
};

#include <render_pass.ipp>

} // harmont

#endif /* RENDER_PASS_HPP_ */
