#ifndef _HARMONT_EXAMPLE_SHADOW_PASS_HPP_
#define _HARMONT_EXAMPLE_SHADOW_PASS_HPP_

#include "harmont.hpp"

namespace harmont {

class shadow_pass {
	public:
		typedef std::shared_ptr<shadow_pass>                  ptr_t;
		typedef std::weak_ptr<shadow_pass>                    wptr_t;
		typedef std::shared_ptr<const shadow_pass>            const_ptr_t;
		typedef std::weak_ptr<const shadow_pass>              const_wptr_t;
		typedef Eigen::AlignedBox<float, 3>                   bounding_box_t;
		typedef callback_t<shader_program::ptr, pass_type_t>  geometry_callback_t;

	public:
		shadow_pass(uint32_t resolution, uint32_t sample_count);
		virtual ~shadow_pass();

		texture::ptr shadow_texture();
		texture::const_ptr shadow_texture() const;

		Eigen::Matrix4f& transform();
		const Eigen::Matrix4f& transform() const;

		std::vector<float>& poisson_disk();
		const std::vector<float>& poisson_disk() const;

        void init(const geometry_callback_t& init_callback);
		void render(const geometry_callback_t& render_callback, int width, int height, bool clipping, float clipping_z);

		void update(const bounding_box_t& bbox, const Eigen::Vector3f& light_dir);

	protected:
		void render_(shader_program::ptr program, uint32_t num_indices);
		template <class RNG>
		static Eigen::Vector2f gen_point_(RNG& rng, float radius, float offset, const Eigen::Vector2f& base = Eigen::Vector2f::Zero());
		static std::vector<float> poisson_disk_(uint32_t n, float radius, uint32_t k = 30);

	protected:
		uint32_t            res_;
		uint32_t            sample_count_;
		render_pass::ptr    pass_;
		Eigen::Matrix4f     mat_;
		texture::ptr        tex_;
		texture::ptr        dummy_tex_;
		std::vector<float>  disk_;
};

#include "shadow_pass.ipp"

} // harmont

#endif /* _HARMONT_EXAMPLE_SHADOW_PASS_HPP_ */
