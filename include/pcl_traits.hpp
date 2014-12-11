#ifndef _HARMONT_PCL_TRAITS_HPP_
#define _HARMONT_PCL_TRAITS_HPP_

#include "pointcloud_traits.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace harmont {

template <typename PointType>
using cloud = ::pcl::PointCloud<PointType>;

} // harmont

#endif /* _HARMONT_PCL_TRAITS_HPP_ */
