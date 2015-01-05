#ifndef HARMONT_RENDER_PASS_2D_HPP_
#define HARMONT_RENDER_PASS_2D_HPP_

#include "vertex_array.hpp"
#include "vertex_buffer.hpp"

namespace harmont {

class render_pass_2d : public render_pass {
    public:
        typedef std::shared_ptr<render_pass_2d>       ptr;
        typedef std::weak_ptr<render_pass_2d>         wptr;
        typedef std::shared_ptr<const render_pass_2d> const_ptr;
        typedef std::weak_ptr<const render_pass_2d>   const_wptr;

    public:
        render_pass_2d(vertex_shader::ptr vs, fragment_shader::ptr fs, const textures& outputs = textures(), texture::ptr depth_texture = nullptr);
        render_pass_2d(vertex_shader::ptr vs, fragment_shader::ptr fs, geometry_shader::ptr gs, const textures& outputs = textures(), texture::ptr depth_texture = nullptr);
        render_pass_2d(const std::vector<vertex_shader::ptr>& vs, const std::vector<fragment_shader::ptr>& fs, const textures& outputs = textures(), texture::ptr depth_texture = nullptr);
        render_pass_2d(const std::vector<vertex_shader::ptr>& vs, const std::vector<fragment_shader::ptr>& fs, const std::vector<geometry_shader::ptr>& gs, const textures& outputs = textures(), texture::ptr depth_texture = nullptr);
        virtual ~render_pass_2d();

        void render(const draw_callback_t& pre_draw_call, const named_textures& inputs = named_textures());
        void render(const named_textures& inputs = named_textures());

    protected:
        void init_quad_geometry_();

    protected:
        vertex_array::ptr            vao_;
        vertex_buffer<float>::ptr    vbo_;
        index_buffer<uint32_t>::ptr  ibo_;
};


} // harmont

#endif /* HARMONT_RENDER_PASS_2D_HPP_ */
