#include <filesystem>
namespace fs = std::filesystem;

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
typedef pcl::PointNormal   point_t;
typedef pcl::PointCloud<point_t> cloud_t;

#include <nanoflann.hpp>
using namespace nanoflann;
typedef KDTreeEigenMatrixAdaptor<Eigen::MatrixXf> kdtree_t;
constexpr std::size_t k = 20;

int main (int argc, char const* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: compute_normals <input_file> <output_file>\n";
        return 1;
    }

    const fs::path input_file(argv[1]);
    const fs::path output_file(argv[2]);

    if (!fs::exists(input_file)) {
        std::cerr << "Unable to read input file \"" << input_file << "\"\n";
    }

    // load cloud
    auto cloud = std::make_shared<cloud_t>();
    pcl::io::loadPCDFile(input_file.string(), *cloud);

    // initialize kdtree
    Eigen::MatrixXf mat(cloud->size(), 3);
    for (uint32_t i = 0; i < cloud->size(); ++i) {
        mat.row(i) = cloud->points[i].getVector3fMap().transpose();
    }
    kdtree_t tree(3, std::cref(mat));
    tree.index->buildIndex();

    // compute normals
    std::array<int64_t, k+1> indices;
    std::array<float, k+1> dists;
    for (auto& pnt : *cloud) {
        size_t found = tree.index->knnSearch(pnt.data, k+1, indices.data(), dists.data());
        if (found < 7) continue;

        // PCA, i.e.
        // given the mean shifted scatter matrix X and its SVD X = USV'
        // the normal is given by the third right singular vector
        Eigen::RowVector3f centroid = Eigen::RowVector3f::Zero();
        Eigen::MatrixXf scatter(found-1, 3);
        for (size_t i = 1; i < found; ++i) {
            scatter.row(i-1) = mat.row(indices[i]);
            centroid += (scatter.row(i-1) - centroid) / i;
        }
        scatter.rowwise() -= centroid;
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(scatter, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // last (3rd) column of V matrix is normal
        pnt.getNormalVector3fMap() = svd.matrixV().col(2);
    }

    pcl::io::savePCDFileBinary(output_file.string(), *cloud);
}
