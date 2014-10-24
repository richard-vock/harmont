#define USE_OPENMESH
#include <harmont/openmesh_traits.hpp>
using namespace harmont;

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>


int main (int argc, char const* argv[]) {
    std::string path(argv[1]);

    tri_mesh mesh;
    if (!OpenMesh::IO::read_mesh(mesh, path)) {
        std::cerr << "read error\n";
        exit(1);
    }

    Eigen::MatrixXf vertex_data;
    Eigen::VectorXi indices;
    mesh_traits<tri_mesh>::buffer_data(mesh, {mesh_traits<tri_mesh>::POSITION, mesh_traits<tri_mesh>::NORMAL, mesh_traits<tri_mesh>::COLOR}, vertex_data, indices, true);
    std::cout << indices.size() << "\n";
}
