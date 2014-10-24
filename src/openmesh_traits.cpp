#ifdef USE_OPENMESH

#include <openmesh_traits.hpp>


namespace harmont {


template <>
void mesh_traits<tri_mesh>::buffer_data(const tri_mesh& mesh, const fields_t& fields, Eigen::MatrixXf& vertex_data, Eigen::Matrix<uint32_t, Eigen::Dynamic, 1>& indices, bool shared_vertices) {
    uint32_t columns = 0;
    for (const auto& field : fields) {
        columns += field == COLOR ? 1 : 3;
    }
    uint32_t rows = shared_vertices ? mesh.n_vertices() : 3 * mesh.n_faces();
    uint32_t num_faces = mesh.n_faces();
    vertex_data.resize(rows, columns);
    indices.resize(3*num_faces);

    if (shared_vertices) {
        uint32_t begin = 0, end;
        for (const auto& field : fields) {
            end = begin + (field == COLOR ? 1 : 3);

            uint32_t idx = 0;
            for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it, ++idx) {
                if (field == POSITION) {
                    Eigen::RowVector3f pos(mesh.point(*it).data());
                    vertex_data.block(idx, begin, 1, end-begin) = pos;
                }
                if (field == NORMAL) {
                    Eigen::RowVector3f nrm(mesh.normal(*it).data());
                    vertex_data.block(idx, begin, 1, end-begin) = nrm;
                }
                if (field == COLOR) {
                    Eigen::Matrix<uint8_t, 1, 4> uc_col(mesh.color(*it).data());
                    uint32_t* casted_ptr = (uint32_t*)(&(vertex_data(idx, begin)));
                    *casted_ptr = (uint32_t(uc_col[0]) << 16) | (uint32_t(uc_col[1]) << 8) | uint32_t(uc_col[2]) | (uint32_t(uc_col[3]) << 24);
                }
            }

            begin = end;
        }

        uint32_t idx = 0;
        for (auto it = mesh.faces_begin(); it != mesh.faces_end(); ++it) {
            for (auto fvIt = mesh.cfv_iter(*it); fvIt.is_valid(); ++fvIt) {
                indices[idx++] = fvIt->idx();
            }
        }
    } else {
        throw std::runtime_error("Not implemented yet"+SPOT);
    }
}

//template <>
//void mesh_traits<poly_mesh>::buffer_data(const poly_mesh& mesh, const fields_t& fields, Eigen::MatrixXf& vertex_data, Eigen::VectorXi& indices, bool shared_vertices) {
    //poly_mesh copy_mesh(mesh);
    //copy_mesh.triangulate();

	//OpenMesh::IO::Options opt;
	//opt += OpenMesh::IO::Options::FaceNormal;
	//opt += OpenMesh::IO::Options::VertexNormal;
	//opt += OpenMesh::IO::Options::VertexColor;

    //if (!opt.face_has_normal()) {
        //copy_mesh.update_face_normals();
    //}
    //// If no vertex normals were loaded, estimate them.
    //// Note that OpenMesh requires face normals to be available for this.
    //if (!opt.vertex_has_normal()) {
        //copy_mesh.update_normals();
    //}
    //// If no vertex colors were loaded, set a default value for all vertices.
    //if (!opt.vertex_has_color()) {
        //for (auto it = copy_mesh.vertices_begin(); it != copy_mesh.vertices_end(); ++it) {
            //copy_mesh.set_color(*it, internal_openmesh_traits::Color(255, 255, 255, 255));
        //}
    //}
    //tri_mesh t_mesh(copy_mesh);
    //mesh_traits::buffer_data(tri_mesh, fields, vertex_data, indices, shared_vertices);
//}

} // harmont

#endif // USE_OPENMESH
