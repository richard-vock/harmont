template <typename CloudT, template <typename> class PtrT>
inline pointcloud_object<CloudT, PtrT>::pointcloud_object(std::string path, bool casts_shadows) : renderable(casts_shadows) {
    pointcloud_ = pointcloud_traits<CloudT, PtrT>::load_from_file(path);
    pointcloud_traits<CloudT, PtrT>::buffer_data(pointcloud_, {POSITION, COLOR, NORMAL, TEXCOORDS}, vertex_data_, index_data_);
}

template <typename CloudT, template <typename> class PtrT>
inline pointcloud_object<CloudT, PtrT>::pointcloud_object(PtrT<CloudT> pointcloud, bool casts_shadows) : renderable(casts_shadows), pointcloud_(pointcloud) {
    pointcloud_traits<CloudT, PtrT>::buffer_data(pointcloud_, {POSITION, COLOR, NORMAL, TEXCOORDS}, vertex_data_, index_data_);
}

template <typename CloudT, template <typename> class PtrT>
inline pointcloud_object<CloudT, PtrT>::~pointcloud_object() {
}

template <typename CloudT, template <typename> class PtrT>
inline void pointcloud_object<CloudT, PtrT>::init() {
    renderable::init(vertex_data_, index_data_);
}

template <typename CloudT, template <typename> class PtrT>
inline PtrT<CloudT> pointcloud_object<CloudT, PtrT>::cloud() {
    return pointcloud_;
}

template <typename CloudT, template <typename> class PtrT>
inline PtrT<const CloudT> pointcloud_object<CloudT, PtrT>::cloud() const {
    return pointcloud_;
}

template <typename CloudT, template <typename> class PtrT>
inline typename pointcloud_object<CloudT, PtrT>::element_type_t pointcloud_object<CloudT, PtrT>::element_type() const {
    return SPLATS;
}

template <typename CloudT, template <typename> class PtrT>
inline bool pointcloud_object<CloudT, PtrT>::transparent() const {
    return false;
}

template <typename CloudT, template <typename> class PtrT>
inline void pointcloud_object<CloudT, PtrT>::compute_bounding_box_() {
    bbox_ = pointcloud_traits<CloudT, PtrT>::bounding_box(pointcloud_, transform_);
}

template <typename CloudT, template <typename> class PtrT>
inline GLenum pointcloud_object<CloudT, PtrT>::gl_element_mode_() const {
    return GL_POINTS;
}
