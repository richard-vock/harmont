#ifndef HARMONT_RENDER_PASS_SAMPLES_HPP_
#define HARMONT_RENDER_PASS_SAMPLES_HPP_

#include <harmont/vertex_array.hpp>
#include <harmont/vertex_buffer.hpp>

namespace harmont {

class render_pass_samples : public render_pass {
    public:
        typedef std::shared_ptr<render_pass_samples>       ptr;
        typedef std::weak_ptr<render_pass_samples>         wptr;
        typedef std::shared_ptr<const render_pass_samples> const_ptr;
        typedef std::weak_ptr<const render_pass_samples>   const_wptr;

    public:
        render_pass_samples(vertex_shader::ptr vs, fragment_shader::ptr fs, const textures& outputs = textures(), texture::ptr depth_texture = nullptr);
        render_pass_samples(vertex_shader::ptr vs, fragment_shader::ptr fs, geometry_shader::ptr gs, const textures& outputs = textures(), texture::ptr depth_texture = nullptr);
        virtual ~render_pass_samples();

        void render(int width, int height, int num_samples, const named_textures& inputs = named_textures());

    protected:
        void init_quad_geometry_();

    protected:
        vertex_array::ptr            vao_;
        vertex_buffer<float>::ptr    vbo_;
        index_buffer<uint32_t>::ptr  ibo_;
};


} // harmont

#endif /* HARMONT_RENDER_PASS_SAMPLES_HPP_ */
