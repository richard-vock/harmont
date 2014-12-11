#ifndef _HARMONT_POINTCLOUD_TRAITS_HPP_
#define _HARMONT_POINTCLOUD_TRAITS_HPP_

#include <vector>

#include "common.hpp"


namespace harmont {


template <typename T>
struct pointcloud_traits {
    typedef T cloud_type_t;
    typedef std::vector<data_field_t> fields_t;

    static void buffer_data(const T& cloud, const fields_t& fields, Eigen::MatrixXf& point_data, Eigen::Matrix<uint32_t, Eigen::Dynamic, 1>& indices);
};


} // harmont

#endif /* _HARMONT_POINTCLOUD_TRAITS_HPP_ */
