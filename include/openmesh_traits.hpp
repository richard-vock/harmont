#ifndef HARMONT_OPENMESH_TRAITS_HPP_
#define HARMONT_OPENMESH_TRAITS_HPP_

#include "mesh_object.hpp"
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/Traits.hh>
#include <OpenMesh/Core/Utils/color_cast.hh>

namespace harmont {

template <typename ColorType>
struct internal_openmesh_traits : public OpenMesh::DefaultTraits {
	typedef ColorType Color;

	VertexAttributes( OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color );
	FaceAttributes( OpenMesh::Attributes::Normal );
};

template <typename ColorType>
using tri_mesh = ::OpenMesh::TriMesh_ArrayKernelT<internal_openmesh_traits<ColorType>>;

template <typename ColorType>
using poly_mesh = ::OpenMesh::PolyMesh_ArrayKernelT<internal_openmesh_traits<ColorType>>;

} // harmont

#endif // HARMONT_OPENMESH_TRAITS_HPP_
