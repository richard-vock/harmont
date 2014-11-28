#ifndef _HARMONT_DEFERRED_RENDERER_HPP_
#define _HARMONT_DEFERRED_RENDERER_HPP_

#include <harmont/harmont.hpp>
#include "shadow_pass.hpp"

namespace harmont {


class deferred_renderer {
	public:
		typedef std::shared_ptr<deferred_renderer>        ptr_t;
		typedef std::weak_ptr<deferred_renderer>          wptr_t;
		typedef std::shared_ptr<const deferred_renderer>  const_ptr_t;
		typedef std::weak_ptr<const deferred_renderer>    const_wptr_t;
		typedef shadow_pass::render_callback_t            render_callback_t;
		typedef Eigen::AlignedBox<float, 3>               bounding_box_t;

		struct render_parameters_t {
			Eigen::Vector3f light_dir;
			float exposure;
            bool two_sided;
			std::string vertex_shader;
			std::string fragment_shader;
			std::string hdr_map;
		};
		struct shadow_parameters_t {
			uint32_t resolution;
			uint32_t sample_count;
			std::string vertex_shader;
			std::string fragment_shader;
		};

	public:
		deferred_renderer(const render_parameters_t& render_parameters, const shadow_parameters_t& shadow_parameters, const bounding_box_t& bbox, int width, int height);
		virtual ~deferred_renderer();

		void set_light_dir(const Eigen::Vector3f& light_dir, const bounding_box_t& bbox);

		float exposure() const;
		void set_exposure(float exposure);
		void delta_exposure(float delta);

        bool two_sided() const;
        void set_two_sided(bool two_sided);
        void toggle_two_sided();

		render_pass::ptr geometry_pass();
		render_pass::const_ptr geometry_pass() const;

		void render(const render_callback_t& render_callback, camera::ptr cam, const bounding_box_t& bbox);
		void reshape(camera::ptr cam);

	protected:
		void   load_hdr_map_(std::string filename);
		static std::pair<float, float> get_near_far(camera::const_ptr cam, const bounding_box_t& bbox);

	protected:
		render_pass_2d::ptr  clear_pass_;
		render_pass::ptr     geom_pass_;
		render_pass_2d::ptr  compose_pass_;
		render_pass_2d::ptr  debug_pass_;
		texture::ptr         depth_tex_;
		texture::ptr         gbuffer_tex_;
		texture::ptr         diff_tex_;
		shadow_pass::ptr_t   shadow_pass_;
		Eigen::Vector3f      light_dir_;
		float                exposure_;
        bool                 two_sided_;
};


} // harmont

#endif /* _HARMONT_DEFERRED_RENDERER_HPP_ */
