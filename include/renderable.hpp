#ifndef _HARMONT_RENDERABLE_HPP_
#define _HARMONT_RENDERABLE_HPP_

#include "common.hpp"
#include "vertex_array.hpp"
#include "vertex_buffer.hpp"

namespace harmont {


class renderable {
	public:
		typedef std::shared_ptr<renderable>                 ptr_t;
		typedef std::weak_ptr<renderable>                   wptr_t;
		typedef std::shared_ptr<const renderable>           const_ptr_t;
		typedef std::weak_ptr<const renderable>             const_wptr_t;

		typedef vertex_array                                vao_t;
		typedef vertex_buffer<float>                        vbo_t;
		typedef index_buffer<uint32_t>                      ibo_t;

		typedef Eigen::Matrix4f                             transformation_t;
		typedef Eigen::MatrixXf                             vertex_data_t;
		typedef Eigen::Matrix<uint32_t, Eigen::Dynamic, 1>  index_data_t;
		typedef Eigen::Vector4f                             color_t;

		typedef enum {VERTS, SPLATS}                        element_type_t;

        typedef union {
            struct {
                uint8_t b;
                uint8_t g;
                uint8_t r;
                uint8_t a;
            };
            float rgba;
        } internal_color_t;

	public:
		renderable(bool casts_shadows = true);
		virtual ~renderable();

		void init(const vertex_data_t& vertex_data, const index_data_t& index_data);
		void render(shader_program::ptr program, pass_type_t type, const bbox_t& bbox);

        virtual void pre_render(shader_program::ptr program, pass_type_t type);
        virtual void post_render(shader_program::ptr program, pass_type_t type);

        void update_geometry(const vertex_data_t& vertex_data);

		void set_colors(const std::vector<uint32_t>& indices, const std::vector<color_t>& colors);
		void set_colors(const std::vector<uint32_t>& indices, const color_t& color);
		void set_colors(const std::vector<color_t>& colors);
		void set_colors(const color_t& color);
		void reset_colors();

		virtual element_type_t element_type() const = 0;
		virtual bool transparent() const;

		bool initialized() const;

		uint32_t num_elements() const;

		vbo_t::ptr shadow_vertex_buffer();
		vbo_t::const_ptr shadow_vertex_buffer() const;

		vbo_t::ptr display_vertex_buffer();
		vbo_t::const_ptr display_vertex_buffer() const;

		vao_t::ptr shadow_vertex_array();
		vao_t::const_ptr shadow_vertex_array() const;

		vao_t::ptr display_vertex_array();
		vao_t::const_ptr display_vertex_array() const;

		ibo_t::ptr element_index_buffer();
		ibo_t::const_ptr element_index_buffer() const;

        bool active() const;
        void set_active(const bool& active);
        void toggle_active();

        bool casts_shadows() const;
        void set_casts_shadows(bool casts_shadows);
        void toggle_casts_shadows();

		bool clipping() const;
		void set_clipping(bool clipping);
		void toggle_clipping();

		float clipping_height() const;
		void set_clipping_height(float height);
		void delta_clipping_height(float delta);

        const Eigen::Vector3f& clipping_normal() const;
        void set_clipping_normal(const Eigen::Vector3f& clipping_normal);

        bool invert_clipping() const;
        void set_invert_clipping(const bool& invert_clipping);
        void toggle_invert_clipping();

        const transformation_t& transformation() const;
        void set_transformation(const transformation_t& transformation);

        void set_texture(texture::ptr tex);
        void unset_texture();

		bbox_t bounding_box();

        static float color_to_rgba(Eigen::Vector4f col);
        static float alpha_from_rgba(float rgba);

	protected:
		virtual void compute_bounding_box_() = 0;
		virtual GLenum gl_element_mode_() const = 0;
		void set_color_data_(const vertex_data_t& color_data);
        static bool is_transparent(const vertex_data_t& color_data);


	protected:
        bool              active_;
        bool              transparent_;
        bool              bbox_valid_;
        bool              casts_shadows_;
		bool              clipping_;
		float             clipping_height_;
        Eigen::Vector3f   clipping_normal_;
        bool              invert_clipping_;
		uint32_t          num_elements_;
		transformation_t  transform_;
		vbo_t::ptr        shadow_buffer_;
		vbo_t::ptr        display_buffer_;
		vao_t::ptr        shadow_array_;
		vao_t::ptr        display_array_;
		ibo_t::ptr        index_buffer_;
		vertex_data_t     initial_color_data_;
        bbox_t            bbox_;
        texture::ptr      tex_;
};


} // harmont

#endif /* _HARMONT_RENDERABLE_HPP_ */
