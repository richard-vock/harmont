#ifndef HARMONT_OPENMESH_TRAITS_HPP_
#define HARMONT_OPENMESH_TRAITS_HPP_

#include "mesh_traits.hpp"
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/Traits.hh>
#include <OpenMesh/Core/Utils/color_cast.hh>

namespace harmont {

struct internal_openmesh_traits : public OpenMesh::DefaultTraits {
	typedef OpenMesh::Vec4uc Color;

	VertexAttributes( OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color );
	FaceAttributes( OpenMesh::Attributes::Normal );
};

typedef ::OpenMesh::TriMesh_ArrayKernelT<internal_openmesh_traits>   tri_mesh;
typedef ::OpenMesh::PolyMesh_ArrayKernelT<internal_openmesh_traits>  poly_mesh;

} // harmont

#endif // HARMONT_OPENMESH_TRAITS_HPP_
