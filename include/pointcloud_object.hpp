#ifndef _HARMONT_POINTCLOUD_OBJECT_HPP_
#define _HARMONT_POINTCLOUD_OBJECT_HPP_

#include "renderable.hpp"

namespace harmont {

template <typename T, template <typename> class PtrT = std::shared_ptr>
struct pointcloud_traits {
	typedef T                          pointcloud_type_t;
	typedef std::vector<data_field_t>  fields_t;

	static PtrT<T> load_from_file(const std::string& path);
	static bbox_t bounding_box(PtrT<const T> pointcloud, const Eigen::Matrix4f& transformation, const std::vector<int>& subset = std::vector<int>());
	static void buffer_data(PtrT<const T> pointcloud, const fields_t& fields, renderable::vertex_data_t& vertex_data, renderable::index_data_t& indices, const Eigen::Vector4f& default_color = Eigen::Vector4f::Ones(), const std::vector<int>& subset = std::vector<int>());
	static void buffer_data(uint32_t point_count, renderable::vertex_data_t& vertex_data, renderable::index_data_t& indices, const Eigen::Vector4f& default_color = Eigen::Vector4f::Ones());
};

template <typename CloudT, template <typename> class PtrT = std::shared_ptr>
class pointcloud_object : public  renderable {
	public:
		typedef std::shared_ptr<pointcloud_object>        ptr_t;
		typedef std::weak_ptr<pointcloud_object>          wptr_t;
		typedef std::shared_ptr<const pointcloud_object>  const_ptr_t;
		typedef std::weak_ptr<const pointcloud_object>    const_wptr_t;

	public:
		pointcloud_object(std::string path, bool casts_shadows = true);
		pointcloud_object(PtrT<const CloudT> pointcloud, bool casts_shadows = true, const std::vector<int>& subset = std::vector<int>());
		pointcloud_object(uint32_t point_count, bool casts_shadows = true);
		virtual ~pointcloud_object();

		void compute_vertex_data();

        PtrT<const CloudT> cloud() const;

		element_type_t element_type() const;

		void set_point_colors(const std::vector<uint32_t>& indices, const std::vector<color_t>& colors);
		void set_point_colors(const std::vector<uint32_t>& indices, const color_t& color);
		void set_point_colors(const std::vector<color_t>& colors);
		void set_point_colors(const color_t& color);

	protected:
		void compute_bounding_box_();
		GLenum gl_element_mode_() const;

	protected:
        PtrT<const CloudT>    pointcloud_;
        std::vector<int>      subset_;
        uint32_t              point_count_;
};

#include "pointcloud_object.ipp"

} // harmont

#endif /* _HARMONT_POINTCLOUD_OBJECT_HPP_ */
