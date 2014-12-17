#include <pointcloud_object.hpp>


template <typename CloudT>
inline pointcloud_object<CloudT>::pointcloud_object(std::string path, bool casts_shadows) : renderable(casts_shadows) {
    pointcloud_ = pointcloud_traits<CloudT>::load_from_file(path);
    pointcloud_traits<CloudT>::buffer_data(pointcloud_, {POSITION, COLOR, NORMAL}, vertex_data_, index_data_);
}

template <typename CloudT>
inline pointcloud_object<CloudT>::~pointcloud_object() {
}

template <typename CloudT>
inline void pointcloud_object<CloudT>::init() {
    renderable::init(vertex_data_, index_data_);
}

template <typename CloudT>
inline typename pointcloud_object<CloudT>::element_type_t pointcloud_object<CloudT>::element_type() const {
    return SPLATS;
}

template <typename CloudT>
inline bool pointcloud_object<CloudT>::transparent() const {
    return false;
}

template <typename CloudT>
inline void pointcloud_object<CloudT>::compute_bounding_box_() {
    bbox_ = pointcloud_traits<CloudT>::bounding_box(pointcloud_, transform_);
}

template <typename CloudT>
inline GLenum pointcloud_object<CloudT>::gl_element_mode_() const {
    return GL_POINTS;
}
