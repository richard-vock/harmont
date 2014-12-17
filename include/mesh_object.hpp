#ifndef _HARMONT_MESH_OBJECT_HPP_
#define _HARMONT_MESH_OBJECT_HPP_

#include "renderable.hpp"

namespace harmont {

template <typename T>
struct mesh_traits {
	typedef T                          mesh_type_t;
	typedef std::vector<data_field_t>  fields_t;

	static std::shared_ptr<T> load_from_file(const std::string& path);
	static bbox_t bounding_box(std::shared_ptr<const T> mesh, const Eigen::Matrix4f& transformation);
	static void buffer_data(std::shared_ptr<const T> mesh, const fields_t& fields, renderable::vertex_data_t& vertex_data, renderable::index_data_t& indices, bool shared_vertices);
};

template <typename MeshT>
class mesh_object : public  renderable {
	public:
		typedef std::shared_ptr<mesh_object>        ptr_t;
		typedef std::weak_ptr<mesh_object>          wptr_t;
		typedef std::shared_ptr<const mesh_object>  const_ptr_t;
		typedef std::weak_ptr<const mesh_object>    const_wptr_t;

	public:
		mesh_object(std::string path, bool smooth, bool casts_shadows = true);
		mesh_object(std::shared_ptr<MeshT> mesh, bool smooth, bool casts_shadows = true);
		virtual ~mesh_object();

		void init();

        std::shared_ptr<MeshT> mesh();
        std::shared_ptr<const MeshT> mesh() const;

		element_type_t element_type() const;
		bool transparent() const;

	protected:
		void compute_bounding_box_();
		GLenum gl_element_mode_() const;

	protected:
        std::shared_ptr<MeshT>     mesh_;
		renderable::vertex_data_t  vertex_data_;
		renderable::index_data_t   index_data_;
};

#include "mesh_object.ipp"

} // harmont

#endif /* _HARMONT_MESH_OBJECT_HPP_ */
