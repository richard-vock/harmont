#ifdef USE_PCL

#include <pcl_traits.hpp>


namespace harmont {

template <typename PointType>
struct pointcloud_traits<cloud<PointType>> {
    typedef cloud<PointType> cloud_t;
    typedef std::vector<data_field_t> fields_t;

    static void buffer_data(const cloud_t& cloud, const fields_t& fields, Eigen::MatrixXf& point_data, Eigen::Matrix<uint32_t, Eigen::Dynamic, 1>& indices);
};


template <typename PointType>
void pointcloud_traits<cloud<PointType>>::buffer_data(const cloud_t& cloud, const fields_t& fields, Eigen::MatrixXf& point_data, Eigen::Matrix<uint32_t, Eigen::Dynamic, 1>& indices) {
    uint32_t columns = 0;
    for (const auto& field : fields) {
        columns += field == COLOR ? 1 : 3;
    }
    uint32_t rows = cloud.size();
    point_data.resize(rows, columns);
    indices.resize(cloud.size());
    for (uint32_t i = 0; i < indices.size(); ++i) {
        indices[i] = i;
    }

    uint32_t begin = 0, end;
    for (const auto& field : fields) {
        end = begin + (field == COLOR ? 1 : 3);

        uint32_t idx = 0;
        for (const auto& p : cloud) {
            if (field == POSITION) {
                point_data.block(idx, begin, 1, end-begin) = p.getVector3fMap().transpose();
            }
            if (field == NORMAL) {
                point_data.block(idx, begin, 1, end-begin) = p.getNormalVector3fMap().normalized().transpose();
            }
            if (field == COLOR) {
                point_data(idx, begin) = p.rgb;
            }
            ++idx;
        }
        begin = end;
    }
}


#define INSTANTIATE_PCL_POINT(type) \
    template struct pointcloud_traits<cloud<type>>;
#include <pcl_points.def>


} // harmont


#endif // USE_PCL
