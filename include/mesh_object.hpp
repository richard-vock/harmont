#ifndef _HARMONT_MESH_OBJECT_HPP_
#define _HARMONT_MESH_OBJECT_HPP_

#include "renderable.hpp"


#ifdef USE_CARTAN
#include <cartan/mesh_traits.hpp>
#include <cartan/eigen_color_cast.hpp>
#endif

namespace harmont {

template <typename T>
struct mesh_traits {
	typedef T                                   mesh_type_t;
	typedef std::vector<data_field_t>           fields_t;
	typedef std::vector<std::vector<uint32_t>>  index_map_t;

	static std::shared_ptr<T> load_from_file(const std::string& path);
	static bbox_t bounding_box(std::shared_ptr<const T> mesh, const Eigen::Matrix4f& transformation);
	static void buffer_data(std::shared_ptr<const T> mesh, const fields_t& fields, renderable::vertex_data_t& vertex_data, renderable::index_data_t& indices, index_map_t& vertex_index_map, index_map_t& face_index_map, bool shared_vertices);
};

template <typename MeshT>
class mesh_object : public renderable {
	public:
		typedef std::shared_ptr<mesh_object>              ptr_t;
		typedef std::weak_ptr<mesh_object>                wptr_t;
		typedef std::shared_ptr<const mesh_object>        const_ptr_t;
		typedef std::weak_ptr<const mesh_object>          const_wptr_t;
		typedef typename mesh_traits<MeshT>::index_map_t  index_map_t;

	public:
		mesh_object(std::string path, bool smooth, bool casts_shadows = true);
		mesh_object(std::shared_ptr<MeshT> mesh, bool smooth, bool casts_shadows = true);
		virtual ~mesh_object();

		void compute_vertex_data();

		std::shared_ptr<MeshT> mesh();
		std::shared_ptr<const MeshT> mesh() const;

		element_type_t element_type() const;

		void set_vertex_colors(const std::vector<uint32_t>& indices, const std::vector<color_t>& colors);
		void set_vertex_colors(const std::vector<uint32_t>& indices, const color_t& color);
		void set_vertex_colors(const std::vector<color_t>& colors);
		void set_vertex_colors(const color_t& color);

		void set_face_colors(const std::vector<uint32_t>& indices, const std::vector<color_t>& colors);
		void set_face_colors(const std::vector<uint32_t>& indices, const color_t& color);
		void set_face_colors(const std::vector<color_t>& colors);
		void set_face_colors(const color_t& color);

	protected:
		void compute_bounding_box_();
		GLenum gl_element_mode_() const;

	protected:
        bool                       smooth_;
		std::shared_ptr<MeshT>     mesh_;
		index_map_t                vertex_index_map_;
		index_map_t                face_index_map_;
};

#include "mesh_object.ipp"

} // harmont

#endif /* _HARMONT_MESH_OBJECT_HPP_ */
