#ifndef _HARMONT_CROSSHAIR_OBJECT_HPP_
#define _HARMONT_CROSSHAIR_OBJECT_HPP_

#include "renderable.hpp"

namespace harmont {


class crosshair_object : public renderable {
	public:
		typedef std::shared_ptr<crosshair_object>        ptr_t;
		typedef std::weak_ptr<crosshair_object>          wptr_t;
		typedef std::shared_ptr<const crosshair_object>  const_ptr_t;
		typedef std::weak_ptr<const crosshair_object>    const_wptr_t;
		typedef std::vector<Eigen::Vector3f>             vertices_t;
		typedef std::vector<Eigen::Vector4f>             colors_t;

	public:
		crosshair_object(const Eigen::Vector3f& vertex, Eigen::Vector4f color, float size = 1.f, float line_width = 1.f, bool casts_shadows = false);
		crosshair_object(const vertices_t& vertices, Eigen::Vector4f color, float size = 1.f, float line_width = 1.f, bool casts_shadows = false);
		crosshair_object(const vertices_t& vertices, const colors_t& colors, float size = 1.f, float line_width = 1.f, bool casts_shadows = false);
		virtual ~crosshair_object();

		void compute_vertex_data();

        float size() const;
        void set_size(const float& size);

		float line_width() const;
		void set_line_width(const float& line_width);

		void init();

		void pre_render(shader_program::ptr program, pass_type_t type);
		void post_render(shader_program::ptr program, pass_type_t type);

		element_type_t element_type() const;
		bool transparent() const;

	protected:
		void compute_bounding_box_();
		GLenum gl_element_mode_() const;

	protected:
		vertices_t                 vertices_;
		colors_t                   colors_;
        float                      size_;
		float                      line_width_;
};


} // harmont

#endif /* _HARMONT_CROSSHAIR_OBJECT_HPP_ */
