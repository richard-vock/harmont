#ifdef USE_CARTAN

template <typename MeshT>
using cartan_mesh_t = typename cartan::mesh_traits<MeshT>::mesh_t;

template <typename MeshT>
using cartan_mesh_ptr_t = std::shared_ptr<cartan_mesh_t<MeshT>>;

template <typename MeshT>
std::shared_ptr<MeshT> mesh_traits<MeshT>::load_from_file(const std::string& path) {
    typedef cartan::mesh_traits<MeshT> mt;

    cartan_mesh_ptr_t<MeshT> mesh(new cartan_mesh_t<MeshT>());
    if (!mt::read(*mesh, path)) {
        throw std::runtime_error("mesh_traits::load_from_file: Unable to read mesh file \""+path+"\""+SPOT);
    }
    typename mt::eigen_color_t default_color = cartan::eigen_color_cast<typename mt::color_scalar_t, float, mt::color_dim, 4>(Eigen::Matrix<float, 4, 1>(1.f, 1.f, 1.f, 1.f));
    for (const auto& handle : mt::vertex_handles(*mesh)) {
        mt::set_eigen_vertex_color(*mesh, handle, default_color);
    }
    return mesh;
}

template <typename MeshT>
bbox_t mesh_traits<MeshT>::bounding_box(std::shared_ptr<const MeshT> mesh, const Eigen::Matrix4f& transformation) {
    typedef cartan::mesh_traits<MeshT> mt;
    bbox_t bbox;
    for (const auto& handle : mt::vertex_handles(*mesh)) {
        auto pos = mt::eigen_vertex_position(*mesh, handle);
        pos = (transformation * pos.homogeneous()).head(3);
        bbox.extend(pos);
    }
    return bbox;
}

template <typename MeshT>
void mesh_traits<MeshT>::buffer_data(std::shared_ptr<const MeshT> mesh, const fields_t& fields, Eigen::MatrixXf& vertex_data, Eigen::Matrix<uint32_t, Eigen::Dynamic, 1>& indices, bool shared_vertices) {
    typedef cartan::mesh_traits<MeshT> mt;

    uint32_t columns = 0;
    for (const auto& field : fields) {
        columns += field == COLOR ? 1 : (field == TEXCOORDS ? 2 : 3);
    }
    uint32_t num_faces = mt::num_faces(*mesh);
    uint32_t rows = shared_vertices ? mt::num_vertices(*mesh) : 3 * num_faces;
    vertex_data.resize(rows, columns);
    indices.resize(3*num_faces);

    if (shared_vertices) {
        uint32_t begin = 0, end;
        for (const auto& field : fields) {
            end = begin + (field == COLOR ? 1 : (field == TEXCOORDS ? 2 : 3));

            uint32_t idx = 0;
            auto vertex_handles = mt::vertex_handles(*mesh);
            for (auto it = vertex_handles.begin(); it != vertex_handles.end(); ++it, ++idx) {
                if (field == POSITION) {
                    Eigen::RowVector3f pos = mt::eigen_vertex_position(*mesh, *it).transpose();
                    vertex_data.block(idx, begin, 1, end-begin) = pos;
                }
                if (field == NORMAL) {
                    Eigen::RowVector3f nrm = mt::eigen_vertex_normal(*mesh, *it).transpose();
                    vertex_data.block(idx, begin, 1, end-begin) = nrm.normalized();
                }
                if (field == COLOR) {
                    auto eigen_color = mt::eigen_vertex_color(*mesh, *it);
                    Eigen::Matrix<uint8_t, 1, 4> uc_col = cartan::eigen_color_cast<uint8_t, typename mt::color_scalar_t, 4, mt::color_dim>(eigen_color).transpose();
                    uint32_t* casted_ptr = (uint32_t*)(&(vertex_data(idx, begin)));
                    *casted_ptr = (uint32_t(uc_col[0]) << 16) | (uint32_t(uc_col[1]) << 8) | uint32_t(uc_col[2]) | (uint32_t(uc_col[3]) << 24);
                }
                if (field == TEXCOORDS) {
                    vertex_data.block(idx, begin, 1, end-begin) = Eigen::RowVector2f::Zero();
                }
            }

            begin = end;
        }

        auto face_handles = mt::face_handles(*mesh);
        uint32_t idx = 0;
        for (const auto& face_handle : mt::face_handles(*mesh)) {
            for (const auto& vertex_handle : mt::face_vertices(*mesh, face_handle)) {
                indices[idx++] = mt::vertex_index(*mesh, vertex_handle);
            }
        }
    } else {
        uint32_t begin = 0, end;
        for (const auto& field : fields) {
            end = begin + (field == COLOR ? 1 : (field == TEXCOORDS ? 2 : 3));

            uint32_t idx = 0;
            for (const auto& face_handle : mt::face_handles(*mesh)) {
                if (field == POSITION || field == COLOR || field == TEXCOORDS) {
                    for (const auto& vertex_handle : mt::face_vertices(*mesh, face_handle)) {
                        if (field == POSITION) {
                            Eigen::RowVector3f pos = mt::eigen_vertex_position(*mesh, vertex_handle).transpose();
                            vertex_data.block(idx, begin, 1, end-begin) = pos;
                        }
                        if (field == TEXCOORDS) {
                            vertex_data.block(idx, begin, 1, end-begin) = Eigen::RowVector2f::Zero();
                        }
                        if (field == COLOR) {
                            auto eigen_color = mt::eigen_vertex_color(*mesh, vertex_handle);
                            Eigen::Matrix<uint8_t, 1, 4> uc_col = cartan::eigen_color_cast<uint8_t, typename mt::color_scalar_t, 4, mt::color_dim>(eigen_color).transpose();
                            uint32_t* casted_ptr = (uint32_t*)(&(vertex_data(idx, begin)));
                            *casted_ptr = (uint32_t(uc_col[0]) << 16) | (uint32_t(uc_col[1]) << 8) | uint32_t(uc_col[2]) | (uint32_t(uc_col[3]) << 24);
                        }
                        ++idx;
                    }
                }
                if (field == NORMAL) {
                    auto vertex_handles = mt::face_vertices(*mesh, face_handle);
                    auto p0 = mt::eigen_vertex_position(*mesh, vertex_handles[0]);
                    auto p1 = mt::eigen_vertex_position(*mesh, vertex_handles[1]);
                    auto p2 = mt::eigen_vertex_position(*mesh, vertex_handles[2]);
                    Eigen::RowVector3f nrm = ((p1-p0).cross(p2-p0).normalized()).transpose();
                    vertex_data.block(idx++, begin, 1, end-begin) = nrm;
                    vertex_data.block(idx++, begin, 1, end-begin) = nrm;
                    vertex_data.block(idx++, begin, 1, end-begin) = nrm;
                }
            }

            begin = end;
        }
        for (uint32_t i = 0; i < indices.size(); ++i) {
            indices[i] = i;
        }
    }
}

#endif // USE_CARTAN


template <typename MeshT>
inline mesh_object<MeshT>::mesh_object(std::string path, bool smooth, bool casts_shadows) : renderable(casts_shadows) {
    mesh_ = mesh_traits<MeshT>::load_from_file(path);
    mesh_traits<MeshT>::buffer_data(mesh_, {POSITION, COLOR, NORMAL, TEXCOORDS}, vertex_data_, index_data_, smooth);
}

template <typename MeshT>
inline mesh_object<MeshT>::mesh_object(std::shared_ptr<MeshT> mesh, bool smooth, bool casts_shadows) : renderable(casts_shadows), mesh_(mesh) {
    mesh_traits<MeshT>::buffer_data(mesh_, {POSITION, COLOR, NORMAL, TEXCOORDS}, vertex_data_, index_data_, smooth);
}

template <typename MeshT>
inline mesh_object<MeshT>::~mesh_object() {
}

template <typename MeshT>
inline void mesh_object<MeshT>::init() {
    renderable::init(vertex_data_, index_data_);
}

template <typename MeshT>
std::shared_ptr<MeshT> mesh_object<MeshT>::mesh() {
    return mesh_;
}

template <typename MeshT>
std::shared_ptr<const MeshT> mesh_object<MeshT>::mesh() const {
    return mesh_;
}

template <typename MeshT>
inline typename mesh_object<MeshT>::element_type_t mesh_object<MeshT>::element_type() const {
    return VERTS;
}

template <typename MeshT>
inline void mesh_object<MeshT>::compute_bounding_box_() {
    bbox_ = mesh_traits<MeshT>::bounding_box(mesh_, transform_);
}

template <typename MeshT>
inline GLenum mesh_object<MeshT>::gl_element_mode_() const {
    return GL_TRIANGLES;
}
