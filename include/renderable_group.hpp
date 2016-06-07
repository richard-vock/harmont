#ifndef _HARMONT_RENDERABLE_GROUP_HPP_
#define _HARMONT_RENDERABLE_GROUP_HPP_

#include "renderable.hpp"

namespace harmont {

class renderable_group {
	public:
		typedef std::shared_ptr<renderable_group>        ptr_t;
		typedef std::weak_ptr<renderable_group>          wptr_t;
		typedef std::shared_ptr<const renderable_group>  const_ptr_t;
		typedef std::weak_ptr<const renderable_group>    const_wptr_t;

	public:
		renderable_group(const std::vector<renderable::ptr_t>& objects);
		virtual ~renderable_group();

		void init();

		std::vector<renderable::ptr_t>& objects();
		const std::vector<renderable::ptr_t>& objects() const;

        void add_object(renderable::ptr_t object);

		bool all_active() const;
		bool any_active() const;
		void set_active(const bool& active);
		void toggle_active();

		bool all_casts_shadows() const;
		bool any_casts_shadows() const;
		void set_casts_shadows(bool casts_shadows);
		void toggle_casts_shadows();

		bool all_clipping() const;
		bool any_clipping() const;
		void set_clipping(bool clipping);
		void toggle_clipping();

		float clipping_height() const;
		void set_clipping_height(float height);
		void delta_clipping_height(float delta);

		const Eigen::Vector3f& clipping_normal() const;
		void set_clipping_normal(const Eigen::Vector3f& clipping_normal);

		bool all_invert_clipping() const;
		bool any_invert_clipping() const;
		void set_invert_clipping(const bool& invert_clipping);
		void toggle_invert_clipping();

		const renderable::transformation_t& transformation() const;
		void set_transformation(const renderable::transformation_t& transformation);
		void move(const renderable::transformation_t& transformation);

		void set_object_color(uint32_t object_index, const Eigen::Vector4f& color);
		void reset_colors();

		bbox_t bounding_box();

	protected:
		std::vector<renderable::ptr_t>  objects_;
		float  clipping_height_;
};


} // harmont

#endif /* _HARMONT_RENDERABLE_GROUP_HPP_ */
