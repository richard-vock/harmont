#ifndef _HARMONT_DEFERRED_RENDERER_HPP_
#define _HARMONT_DEFERRED_RENDERER_HPP_

#include "harmont.hpp"
#include "shadow_pass.hpp"
#include "ssdo.hpp"

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
			float shadow_bias;
            bool two_sided;
			std::string hdr_map;
		};
		struct shadow_parameters_t {
			uint32_t resolution;
			uint32_t sample_count;
		};

	public:
		deferred_renderer(const render_parameters_t& render_parameters, const shadow_parameters_t& shadow_parameters, const bounding_box_t& bbox, int width, int height);
		virtual ~deferred_renderer();

		void set_light_dir(const Eigen::Vector3f& light_dir, const bounding_box_t& bbox);

		float exposure() const;
		void set_exposure(float exposure);
		void delta_exposure(float delta);

		float shadow_bias() const;
		void set_shadow_bias(float bias);
		void delta_shadow_bias(float delta);

        bool two_sided() const;
        void set_two_sided(bool two_sided);
        void toggle_two_sided();

        bool clipping() const;
        void set_clipping(bool clipping);
        void toggle_clipping();

        float clipping_height() const;
        void  set_clipping_height(float height);
        void  delta_clipping_height(float delta);

		float ssdo_radius() const;
		void  set_ssdo_radius(float radius);
		void  delta_ssdo_radius(float delta);

		float ssdo_exponent() const;
		void  set_ssdo_exponent(float exponent);
		void  delta_ssdo_exponent(float delta);

		float ssdo_reflective_albedo() const;
		void  set_ssdo_reflective_albedo(float reflective_albedo);
		void  delta_ssdo_reflective_albedo(float delta);

        float point_size() const;
        void set_point_size(float point_size);
        void delta_point_size(float delta);

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
		ssdo::ptr_t          ssdo_pass_;
		Eigen::Vector3f      light_dir_;
		float                exposure_;
        float                shadow_bias_;
        bool                 two_sided_;
        bool                 clipping_;
        float                clipping_height_;
        float                clipping_min_z_;
        float                clipping_max_z_;
        float                point_size_;
};


} // harmont


#endif /* _HARMONT_DEFERRED_RENDERER_HPP_ */
