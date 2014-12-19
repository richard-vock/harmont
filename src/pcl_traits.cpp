#ifdef USE_PCL

#include <pcl_traits.hpp>
#include <pcl/io/pcd_io.h>

#include <boost/shared_ptr.hpp>


namespace harmont {

template <typename PointType, template <typename> class PtrT>
struct pointcloud_traits<cloud<PointType>, PtrT> {
    typedef cloud<PointType> cloud_t;
    typedef std::vector<data_field_t> fields_t;

	static PtrT<cloud_t> load_from_file(const std::string& path);
	static bbox_t bounding_box(PtrT<const cloud_t> cloud, const Eigen::Matrix4f& transformation);
	static void buffer_data(PtrT<const cloud_t> cloud, const fields_t& fields, renderable::vertex_data_t& vertex_data, renderable::index_data_t& indices, const Eigen::Vector4f& default_color = Eigen::Vector4f::Ones());
};

template <typename PointT>
struct pcl_color_traits {
    static float get_rgb(const PointT&, const Eigen::Vector4f& default_color) {
        pcl::PointXYZRGBNormal dummy;
        dummy.r = static_cast<uint8_t>(default_color[0] * 255.f);
        dummy.g = static_cast<uint8_t>(default_color[1] * 255.f);
        dummy.b = static_cast<uint8_t>(default_color[2] * 255.f);
        dummy.a = static_cast<uint8_t>(default_color[3] * 255.f);
        return dummy.rgb;
    }
};

//template <>
//struct pcl_color_traits<pcl::PointXYZRGBNormal> {
    //static float get_rgb(const pcl::PointXYZRGBNormal& point, const Eigen::Vector4f&) {
        //return point.rgb;
    //}
//};

template <typename PointType, template <typename> class PtrT>
PtrT<typename pointcloud_traits<cloud<PointType>, PtrT>::cloud_t> pointcloud_traits<cloud<PointType>, PtrT>::load_from_file(const std::string& path) {
    PtrT<cloud_t> cloud = PtrT<cloud_t>(new cloud_t());
    if (pcl::io::loadPCDFile(path, *cloud) != 0) {
        throw std::runtime_error("Unable to read pcd file"+SPOT);
    }
    return cloud;
}

template <typename PointType, template <typename> class PtrT>
bbox_t pointcloud_traits<cloud<PointType>, PtrT>::bounding_box(PtrT<const cloud_t> cloud, const Eigen::Matrix4f& transformation) {
    bbox_t bbox;
    for (const auto& p : *cloud) {
        bbox.extend(p.getVector3fMap());
    }
    return bbox;
}

template <typename PointType, template <typename> class PtrT>
void pointcloud_traits<cloud<PointType>, PtrT>::buffer_data(PtrT<const cloud_t> cloud, const fields_t& fields, Eigen::MatrixXf& point_data, Eigen::Matrix<uint32_t, Eigen::Dynamic, 1>& indices, const Eigen::Vector4f& default_color) {
    uint32_t columns = 0;
    for (const auto& field : fields) {
        columns += field == COLOR ? 1 : (field == TEXCOORDS ? 2 : 3);
    }
    uint32_t rows = cloud->size();
    point_data.resize(rows, columns);
    indices.resize(cloud->size());
    for (uint32_t i = 0; i < indices.size(); ++i) {
        indices[i] = i;
    }

    uint32_t begin = 0, end;
    for (const auto& field : fields) {
        end = begin + (field == COLOR ? 1 : (field == TEXCOORDS ? 2 : 3));

        uint32_t idx = 0;
        for (const auto& p : *cloud) {
            if (field == POSITION) {
                point_data.block(idx, begin, 1, end-begin) = p.getVector3fMap().transpose();
            }
            if (field == NORMAL) {
                point_data.block(idx, begin, 1, end-begin) = p.getNormalVector3fMap().normalized().transpose();
            }
            if (field == COLOR) {
                float rgb = pcl_color_traits<PointType>::get_rgb(p, default_color);
                point_data(idx, begin) = rgb;
            }
            if (field == TEXCOORDS) {
                point_data.block(idx, begin, 1, end-begin) = Eigen::RowVector2f::Zero();
            }
            ++idx;
        }
        begin = end;
    }
}


#define INSTANTIATE_PCL_POINT(type) \
    template struct pointcloud_traits<cloud<type>, std::shared_ptr>; \
    template struct pointcloud_traits<cloud<type>, boost::shared_ptr>;
#include <pcl_points.def>


} // harmont


#endif // USE_PCL
