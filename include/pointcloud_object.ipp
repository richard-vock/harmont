template <typename CloudT, template <typename> class PtrT>
inline pointcloud_object<CloudT, PtrT>::pointcloud_object(std::string path, bool casts_shadows) : renderable(casts_shadows) {
    pointcloud_ = pointcloud_traits<CloudT, PtrT>::load_from_file(path);
}

template <typename CloudT, template <typename> class PtrT>
inline pointcloud_object<CloudT, PtrT>::pointcloud_object(PtrT<const CloudT> pointcloud, bool casts_shadows, const std::vector<int>& subset) : renderable(casts_shadows), pointcloud_(pointcloud), subset_(subset) {
}

template <typename CloudT, template <typename> class PtrT>
inline pointcloud_object<CloudT, PtrT>::pointcloud_object(uint32_t point_count, bool casts_shadows) : renderable(casts_shadows), pointcloud_(nullptr), subset_(std::vector<int>()), point_count_(point_count) {
}

template <typename CloudT, template <typename> class PtrT>
inline pointcloud_object<CloudT, PtrT>::~pointcloud_object() {
}

template <typename CloudT, template <typename> class PtrT>
inline void pointcloud_object<CloudT, PtrT>::compute_vertex_data() {
    if (pointcloud_) {
        pointcloud_traits<CloudT, PtrT>::buffer_data(pointcloud_, {POSITION, COLOR, NORMAL, TEXCOORDS}, vertex_data_, index_data_, Eigen::Vector4f::Ones(), subset_);
    } else {
        pointcloud_traits<CloudT, PtrT>::buffer_data(point_count_, vertex_data_, index_data_);
    }
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
    uint32_t num_points = subset_.size() ? subset_.size() : (pointcloud_ ? pointcloud_->size() : point_count_);
    std::vector<color_t> colors(num_points, color);
    set_point_colors(colors);
}

template <typename CloudT, template <typename> class PtrT>
inline void pointcloud_object<CloudT, PtrT>::compute_bounding_box_() {
    if (pointcloud_) {
        bbox_ = pointcloud_traits<CloudT, PtrT>::bounding_box(pointcloud_, transform_, subset_);
    } else {
        bbox_.min() = -Eigen::Vector3f::Ones();
        bbox_.max() = Eigen::Vector3f::Ones();
    }
}

template <typename CloudT, template <typename> class PtrT>
inline GLenum pointcloud_object<CloudT, PtrT>::gl_element_mode_() const {
    return GL_POINTS;
}
