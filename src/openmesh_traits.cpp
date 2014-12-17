#ifdef USE_OPENMESH

#include <openmesh_traits.hpp>


namespace harmont {

template <typename ColorType>
struct mesh_traits<tri_mesh<ColorType>> {
    typedef tri_mesh<ColorType> mesh_type_t;
    typedef std::vector<data_field_t> fields_t;

	static std::shared_ptr<mesh_type_t> load_from_file(const std::string& path);
	static bbox_t bounding_box(std::shared_ptr<const tri_mesh<ColorType>> mesh, const Eigen::Matrix4f& transformation);
	static void buffer_data(std::shared_ptr<const tri_mesh<ColorType>> mesh, const fields_t& fields, renderable::vertex_data_t& vertex_data, renderable::index_data_t& indices, bool shared_vertices);
};


template <typename ColorType>
std::shared_ptr<typename mesh_traits<tri_mesh<ColorType>>::mesh_type_t> mesh_traits<tri_mesh<ColorType>>::load_from_file(const std::string& path) {
    omerr().disable();
    OpenMesh::IO::Options opt;
    opt += OpenMesh::IO::Options::FaceNormal;
    opt += OpenMesh::IO::Options::VertexNormal;
    opt += OpenMesh::IO::Options::VertexColor;

    std::shared_ptr<mesh_type_t> mesh(new mesh_type_t());
    mesh->request_vertex_colors();
    mesh->request_vertex_normals();
    mesh->request_face_normals();
    if (!OpenMesh::IO::read_mesh(*mesh, path, opt)) {
        throw std::runtime_error("Unable to read mesh file. Aborting.");
    }
    mesh->triangulate();
    if (!opt.face_has_normal()) {
        mesh->update_face_normals();
    }
    mesh->update_normals();
    // let us not expect materials to be set
    for (auto it = mesh->vertices_begin(); it != mesh->vertices_end(); ++it) {
        mesh->set_color(*it, ColorType(1.f, 1.f, 1.f, 1.f));
    }

    return mesh;
}

template <typename ColorType>
bbox_t mesh_traits<tri_mesh<ColorType>>::bounding_box(std::shared_ptr<const tri_mesh<ColorType>> mesh, const Eigen::Matrix4f& transformation) {
    bbox_t bbox;
    for (auto it = mesh->vertices_begin(); it != mesh->vertices_end(); ++it) {
        Eigen::Vector3f pnt(mesh->point(*it).data());
        pnt = (transformation * pnt.homogeneous()).head(3);
        bbox.extend(pnt);
    }
    return bbox;
}

template <typename ColorType>
void mesh_traits<tri_mesh<ColorType>>::buffer_data(std::shared_ptr<const tri_mesh<ColorType>> mesh, const fields_t& fields, Eigen::MatrixXf& vertex_data, Eigen::Matrix<uint32_t, Eigen::Dynamic, 1>& indices, bool shared_vertices) {
    uint32_t columns = 0;
    for (const auto& field : fields) {
        columns += field == COLOR ? 1 : 3;
    }
    uint32_t rows = shared_vertices ? mesh->n_vertices() : 3 * mesh->n_faces();
    uint32_t num_faces = mesh->n_faces();
    vertex_data.resize(rows, columns);
    indices.resize(3*num_faces);

    OpenMesh::color_caster<OpenMesh::Vec4uc, ColorType> color_caster;

    if (shared_vertices) {
        uint32_t begin = 0, end;
        for (const auto& field : fields) {
            end = begin + (field == COLOR ? 1 : 3);

            uint32_t idx = 0;
            for (auto it = mesh->vertices_begin(); it != mesh->vertices_end(); ++it, ++idx) {
                if (field == POSITION) {
                    Eigen::RowVector3f pos(mesh->point(*it).data());
                    vertex_data.block(idx, begin, 1, end-begin) = pos;
                }
                if (field == NORMAL) {
                    Eigen::RowVector3f nrm(mesh->normal(*it).data());
                    vertex_data.block(idx, begin, 1, end-begin) = nrm.normalized();
                }
                if (field == COLOR) {
                    OpenMesh::Vec4uc openmesh_color = color_caster.cast(mesh->color(*it));
                    Eigen::Matrix<uint8_t, 1, 4> uc_col(openmesh_color.data());
                    uint32_t* casted_ptr = (uint32_t*)(&(vertex_data(idx, begin)));
                    *casted_ptr = (uint32_t(uc_col[0]) << 16) | (uint32_t(uc_col[1]) << 8) | uint32_t(uc_col[2]) | (uint32_t(uc_col[3]) << 24);
                }
            }

            begin = end;
        }

        uint32_t idx = 0;
        for (auto it = mesh->faces_begin(); it != mesh->faces_end(); ++it) {
            for (auto fv_it = mesh->cfv_iter(*it); fv_it.is_valid(); ++fv_it) {
                indices[idx++] = fv_it->idx();
            }
        }
    } else {
        uint32_t begin = 0, end;
        for (const auto& field : fields) {
            end = begin + (field == COLOR ? 1 : 3);

            uint32_t idx = 0;
            for (auto it = mesh->faces_begin(); it != mesh->faces_end(); ++it) {
                if (field == POSITION || field == COLOR) {
                    for (auto fv_it = mesh->cfv_iter(*it); fv_it.is_valid(); ++fv_it, ++idx) {
                        if (field == POSITION) {
                            Eigen::RowVector3f pos(mesh->point(*fv_it).data());
                            vertex_data.block(idx, begin, 1, end-begin) = pos;
                        }
                        if (field == NORMAL) {
                            Eigen::RowVector3f nrm(mesh->normal(*fv_it).data());
                            vertex_data.block(idx, begin, 1, end-begin) = nrm.normalized();
                        }
                        if (field == COLOR) {
                            OpenMesh::Vec4uc openmesh_color = color_caster.cast(mesh->color(*fv_it));
                            Eigen::Matrix<uint8_t, 1, 4> uc_col(openmesh_color.data());
                            uint32_t* casted_ptr = (uint32_t*)(&(vertex_data(idx, begin)));
                            *casted_ptr = (uint32_t(uc_col[0]) << 16) | (uint32_t(uc_col[1]) << 8) | uint32_t(uc_col[2]) | (uint32_t(uc_col[3]) << 24);
                        }
                    }
                }
                if (field == NORMAL) {
                    auto fv_it = mesh->cfv_iter(*it);
                    Eigen::Vector3f p0(mesh->point(*(fv_it++)).data());
                    Eigen::Vector3f p1(mesh->point(*(fv_it++)).data());
                    Eigen::Vector3f p2(mesh->point(*fv_it).data());
                    Eigen::Vector3f nrm = (p1-p0).cross(p2-p0).normalized();
                    vertex_data.block(idx++, begin, 1, end-begin) = nrm.transpose();
                    vertex_data.block(idx++, begin, 1, end-begin) = nrm.transpose();
                    vertex_data.block(idx++, begin, 1, end-begin) = nrm.transpose();
                }
            }

            begin = end;
        }
        for (uint32_t i = 0; i < indices.size(); ++i) {
            indices[i] = i;
        }
    }
}


#define INSTANTIATE_OPENMESH_COLOR(type)         \
    template struct mesh_traits<tri_mesh<type>>;
#include <openmesh_colors.def>


} // harmont


#endif // USE_OPENMESH
