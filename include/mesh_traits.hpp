#ifndef HARMONT_MESH_TRAITS_HPP_
#define HARMONT_MESH_TRAITS_HPP_

#include <vector>

#include "common.hpp"


namespace harmont {

typedef enum {POSITION, NORMAL, COLOR} data_field_t;

template <typename T>
struct mesh_traits {
    typedef T mesh_type_t;
    typedef std::vector<data_field_t> fields_t;

    static void buffer_data(const T& mesh, const fields_t& fields, Eigen::MatrixXf& vertex_data, Eigen::Matrix<uint32_t, Eigen::Dynamic, 1>& indices, bool shared_vertices);
};


} // harmont

#endif /* HARMONT_MESH_TRAITS_HPP_ */
