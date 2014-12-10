#ifndef _HARMONT_SSDO_HPP_
#define _HARMONT_SSDO_HPP_

#include <harmont/harmont.hpp>

namespace harmont {

class ssdo {
	public:
		typedef std::shared_ptr<ssdo>        ptr_t;
		typedef std::weak_ptr<ssdo>          wptr_t;
		typedef std::shared_ptr<const ssdo>  const_ptr_t;
		typedef std::weak_ptr<const ssdo>    const_wptr_t;

	public:
		ssdo(uint32_t variation, uint32_t num_samples, float radius);
		virtual ~ssdo();

        void init(int width, int height);
        void reshape(int width, int height);

        void compute(texture::ptr gbuffer, texture::ptr env_map, camera::ptr cam, uint32_t num_blur_passes = 1);

        texture::ptr ssdo_texture();
        texture::ptr sample_texture();
        texture::ptr noise_texture();
        render_pass::textures clear_textures();

		uint32_t variation() const;
		void     set_variation(uint32_t variation);

		uint32_t num_samples() const;
		void     set_num_samples(uint32_t num_samples);

		float radius() const;
		void  set_radius(float radius);
		void  delta_radius(float delta);

		float exponent() const;
		void  set_exponent(float exponent);
		void  delta_exponent(float delta);

		float reflective_albedo() const;
		void  set_reflective_albedo(float reflective_albedo);
		void  delta_reflective_albedo(float delta);

	protected:
        void init_samples_();
        void init_noise_();
		void compute_samples_();

	protected:
        uint32_t             variation_;
        uint32_t             num_samples_;
		float                radius_;
        float                exponent_;
		texture::ptr         tex_samples_;
		texture::ptr         tex_noise_;
		texture::ptr         tex_ssdo_;
		texture::ptr         tex_last_;
		texture::ptr         tex_work_;
        render_pass_2d::ptr  pass_sample_;
        render_pass_2d::ptr  pass_blur_h_;
        render_pass_2d::ptr  pass_blur_v_;
        bool                 first_pass_;
        float                refl_albedo_;
};


} // harmont

#endif /* _HARMONT_SSDO_HPP_ */
