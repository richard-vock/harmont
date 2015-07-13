#ifndef _HARMONT_TRIANGLES_OBJECT_HPP_
#define _HARMONT_TRIANGLES_OBJECT_HPP_

#include "renderable.hpp"

namespace harmont {


class triangles_object : public  renderable {
	public:
		typedef std::shared_ptr<triangles_object>        ptr_t;
		typedef std::weak_ptr<triangles_object>          wptr_t;
		typedef std::shared_ptr<const triangles_object>  const_ptr_t;
		typedef std::weak_ptr<const triangles_object>    const_wptr_t;
		typedef std::vector<Eigen::Vector3f>        vertices_t;
		typedef std::vector<Eigen::Vector3f>        normals_t;
		typedef std::vector<Eigen::Vector4f>        colors_t;
		typedef std::vector<Eigen::Vector2f>        texcoords_t;

	public:
		triangles_object(const vertices_t& vertices, Eigen::Vector4f color, const texcoords_t& texcoords = texcoords_t(), bool casts_shadows = true);
		triangles_object(const vertices_t& vertices, const colors_t& colors, const texcoords_t& texcoords = texcoords_t(), bool casts_shadows = true);
		virtual ~triangles_object();

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
        normals_t                  normals_;
        texcoords_t                texcoords_;

};


} // harmont

#endif /* _HARMONT_TRIANGLES_OBJECT_HPP_ */
