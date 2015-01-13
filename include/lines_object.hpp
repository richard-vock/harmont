#ifndef _HARMONT_LINES_OBJECT_HPP_
#define _HARMONT_LINES_OBJECT_HPP_

#include "renderable.hpp"

namespace harmont {


class lines_object : public  renderable {
	public:
		typedef std::shared_ptr<lines_object>        ptr_t;
		typedef std::weak_ptr<lines_object>          wptr_t;
		typedef std::shared_ptr<const lines_object>  const_ptr_t;
		typedef std::weak_ptr<const lines_object>    const_wptr_t;
		typedef std::vector<Eigen::Vector3f>        vertices_t;
		typedef std::vector<Eigen::Vector4f>        colors_t;
		typedef enum { SEPARATE, STRIPED }          mode_t;

	public:
		lines_object(const vertices_t& vertices, Eigen::Vector4f color, float line_width = 1.f, mode_t mode = SEPARATE, bool casts_shadows = false);
		lines_object(const vertices_t& vertices, const colors_t& colors, float line_width = 1.f, mode_t mode = SEPARATE, bool casts_shadows = false);
		virtual ~lines_object();

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

	protected:
		vertices_t                 vertices_;
        colors_t                   colors_;
        float                      line_width_;
        mode_t                     mode_;
};


} // harmont

#endif /* _HARMONT_LINES_OBJECT_HPP_ */
