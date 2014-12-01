#ifndef _HARMONT_SSAO_HPP_
#define _HARMONT_SSAO_HPP_

#include <harmont/harmont.hpp>

namespace harmont {

class ssao {
	public:
		typedef std::shared_ptr<ssao>        ptr_t;
		typedef std::weak_ptr<ssao>          wptr_t;
		typedef std::shared_ptr<const ssao>  const_ptr_t;
		typedef std::weak_ptr<const ssao>    const_wptr_t;

	public:
		ssao(uint32_t variation, uint32_t num_samples, float radius);
		virtual ~ssao();

        void init(int width, int height);
        void reshape(int width, int height);

        void compute(texture::ptr gbuffer, camera::ptr cam);

        texture::ptr ssao_texture();

		uint32_t variation() const;
		void     set_variation(uint32_t variation);

		uint32_t num_samples() const;
		void     set_num_samples(uint32_t num_samples);

		float radius() const;
		void  set_radius(float radius);
		void  delta_radius(float delta);

	protected:
        void init_samples_();
		void compute_samples_();

	protected:
        uint32_t             variation_;
        uint32_t             num_samples_;
		float                radius_;
		texture::ptr         tex_samples_;
		texture::ptr         tex_noise_;
		texture::ptr         tex_ssao_;
		//texture::ptr         tex_work_;
        render_pass_2d::ptr  pass_sample_;
        //render_pass_2d::ptr  pass_blur_h_;
        //render_pass_2d::ptr  pass_blur_v_;
};


} // harmont

#endif /* _HARMONT_SSAO_HPP_ */
