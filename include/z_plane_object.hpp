#ifndef _HARMONT_ZPLANE_OBJECT_HPP_
#define _HARMONT_ZPLANE_OBJECT_HPP_

#include "renderable.hpp"

namespace harmont {


class z_plane_object : public renderable {
	public:
		typedef std::shared_ptr<z_plane_object>        ptr_t;
		typedef std::weak_ptr<z_plane_object>          wptr_t;
		typedef std::shared_ptr<const z_plane_object>  const_ptr_t;
		typedef std::weak_ptr<const z_plane_object>    const_wptr_t;

	public:
		z_plane_object(float z, float x_size, float y_size, bool casts_shadows = true);
		virtual ~z_plane_object();

		void init();

		element_type_t element_type() const;
		bool transparent() const;

        const float& z() const;
        void set_z(const float& z);

        const float& x_size() const;
        void set_x_size(const float& x_size);

        const float& y_size() const;
        void set_y_size(const float& y_size);


	protected:
        void compute_geometry_();
		void compute_bounding_box_();
		GLenum gl_element_mode_() const;

	protected:
		renderable::vertex_data_t  vertex_data_;
		renderable::index_data_t   index_data_;

        float z_;
        float x_size_;
        float y_size_;

        std::vector<Eigen::Vector3f> vertices_;
};


} // harmont

#endif /* _HARMONT_ZPLANE_OBJECT_HPP_ */
