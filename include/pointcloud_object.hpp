#ifndef _HARMONT_POINTCLOUD_OBJECT_HPP_
#define _HARMONT_POINTCLOUD_OBJECT_HPP_

#include "renderable.hpp"

namespace harmont {

template <typename T>
struct pointcloud_traits {
	typedef T                          pointcloud_type_t;
	typedef std::vector<data_field_t>  fields_t;

	static std::shared_ptr<T> load_from_file(const std::string& path);
	static bbox_t bounding_box(std::shared_ptr<const T> pointcloud, const Eigen::Matrix4f& transformation);
	static void buffer_data(std::shared_ptr<const T> pointcloud, const fields_t& fields, renderable::vertex_data_t& vertex_data, renderable::index_data_t& indices, const Eigen::Vector4f& default_color = Eigen::Vector4f::Ones());
};

template <typename CloudT>
class pointcloud_object : public  renderable {
	public:
		typedef std::shared_ptr<pointcloud_object>        ptr_t;
		typedef std::weak_ptr<pointcloud_object>          wptr_t;
		typedef std::shared_ptr<const pointcloud_object>  const_ptr_t;
		typedef std::weak_ptr<const pointcloud_object>    const_wptr_t;

	public:
		pointcloud_object(std::string path, bool casts_shadows = true);
		pointcloud_object(std::shared_ptr<CloudT> pointcloud, bool casts_shadows = true);
		virtual ~pointcloud_object();

		void init();

        std::shared_ptr<CloudT> cloud();
        std::shared_ptr<const CloudT> cloud() const;

		element_type_t element_type() const;
		bool transparent() const;

	protected:
		void compute_bounding_box_();
		GLenum gl_element_mode_() const;

	protected:
        std::shared_ptr<CloudT>    pointcloud_;
		renderable::vertex_data_t  vertex_data_;
		renderable::index_data_t   index_data_;
};

#include "pointcloud_object.ipp"

} // harmont

#endif /* _HARMONT_POINTCLOUD_OBJECT_HPP_ */
