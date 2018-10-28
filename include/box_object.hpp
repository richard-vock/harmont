#ifndef _HARMONT_BOX_OBJECT_HPP_
#define _HARMONT_BOX_OBJECT_HPP_

#include "renderable.hpp"

namespace harmont {


class box_object : public  renderable {
	public:
		typedef std::shared_ptr<box_object>        ptr_t;
		typedef std::weak_ptr<box_object>          wptr_t;
		typedef std::shared_ptr<const box_object>  const_ptr_t;
		typedef std::weak_ptr<const box_object>    const_wptr_t;
		typedef std::vector<Eigen::Vector3f>       vertices_t;
		typedef std::vector<Eigen::Vector4f>       colors_t;

	public:
		box_object(const vertices_t& vertices, Eigen::Vector4f color, bool as_lines = false, float line_width = 1.f, bool casts_shadows = false);
		box_object(const Eigen::Vector3f& min, const Eigen::Vector3f max, Eigen::Vector4f color, bool as_lines = false, float line_width = 1.f, bool casts_shadows = false);
		box_object(const Eigen::Matrix3f& base, const Eigen::Vector3f& size, const Eigen::Vector3f& center, Eigen::Vector4f color, bool as_lines = false, float line_width = 1.f, bool casts_shadows = false);
		virtual ~box_object();

        float line_width() const;
        void set_line_width(const float& line_width);

		void compute_vertex_data();

        void pre_render(shader_program::ptr program, pass_type_t type);
        void post_render(shader_program::ptr program, pass_type_t type);

		element_type_t element_type() const;
		bool transparent() const;

	protected:
		void compute_bounding_box_();
		GLenum gl_element_mode_() const;
        void setup_from_corners_();

	protected:
        bool                       as_lines_;
        float                      line_width_;
		vertices_t                 vertices_;
		vertices_t                 normals_;
        colors_t                   colors_;
};


} // harmont

#endif /* _HARMONT_BOX_OBJECT_HPP_ */
