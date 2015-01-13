template <typename CloudT, template <typename> class PtrT>
inline pointcloud_object<CloudT, PtrT>::pointcloud_object(std::string path, bool casts_shadows) : renderable(casts_shadows) {
    pointcloud_ = pointcloud_traits<CloudT, PtrT>::load_from_file(path);
}

template <typename CloudT, template <typename> class PtrT>
inline pointcloud_object<CloudT, PtrT>::pointcloud_object(PtrT<CloudT> pointcloud, bool casts_shadows) : renderable(casts_shadows), pointcloud_(pointcloud) {
}

template <typename CloudT, template <typename> class PtrT>
inline pointcloud_object<CloudT, PtrT>::~pointcloud_object() {
}

template <typename CloudT, template <typename> class PtrT>
inline void pointcloud_object<CloudT, PtrT>::compute_vertex_data() {
    pointcloud_traits<CloudT, PtrT>::buffer_data(pointcloud_, {POSITION, COLOR, NORMAL, TEXCOORDS}, vertex_data_, index_data_);
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
inline void pointcloud_object<CloudT, PtrT>::set_point_colors(const std::vector<uint32_t>& indices, const std::vector<color_t>& colors) {
    if (indices.size() != colors.size()) throw std::runtime_error("pointcloud_object::set_point_colors: Index count must match colors count"+SPOT);
    renderable::set_colors(indices, colors);
}

template <typename CloudT, template <typename> class PtrT>
void pointcloud_object<CloudT, PtrT>::set_point_colors(const std::vector<uint32_t>& indices, const color_t& color) {
    std::vector<color_t> colors(indices.size(), color);
    set_point_colors(indices, colors);
}

template <typename CloudT, template <typename> class PtrT>
void pointcloud_object<CloudT, PtrT>::set_point_colors(const std::vector<color_t>& colors) {
    std::vector<uint32_t> indices(colors.size());
    std::iota(indices.begin(), indices.end(), 0);
    set_point_colors(indices, colors);
}

template <typename CloudT, template <typename> class PtrT>
void pointcloud_object<CloudT, PtrT>::set_point_colors(const color_t& color) {
    uint32_t num_points = pointcloud_->size();
    std::vector<color_t> colors(num_points, color);
    set_point_colors(colors);
}

template <typename CloudT, template <typename> class PtrT>
inline void pointcloud_object<CloudT, PtrT>::compute_bounding_box_() {
    bbox_ = pointcloud_traits<CloudT, PtrT>::bounding_box(pointcloud_, transform_);
}

template <typename CloudT, template <typename> class PtrT>
inline GLenum pointcloud_object<CloudT, PtrT>::gl_element_mode_() const {
    return GL_POINTS;
}
