#include <mesh_object.hpp>


template <typename MeshT>
inline mesh_object<MeshT>::mesh_object(std::string path, bool smooth, bool casts_shadows) : renderable(casts_shadows) {
    mesh_ = mesh_traits<MeshT>::load_from_file(path);
    mesh_traits<MeshT>::buffer_data(mesh_, {POSITION, COLOR, NORMAL}, vertex_data_, index_data_, smooth);
}

template <typename MeshT>
inline mesh_object<MeshT>::~mesh_object() {
}

template <typename MeshT>
inline void mesh_object<MeshT>::init() {
    renderable::init(vertex_data_, index_data_);
}

template <typename MeshT>
inline typename mesh_object<MeshT>::element_type_t mesh_object<MeshT>::element_type() const {
    return VERTS;
}

template <typename MeshT>
inline bool mesh_object<MeshT>::transparent() const {
    return false;
}

template <typename MeshT>
inline void mesh_object<MeshT>::compute_bounding_box_() {
    bbox_ = mesh_traits<MeshT>::bounding_box(mesh_, transform_);
}

template <typename MeshT>
inline GLenum mesh_object<MeshT>::gl_element_mode_() const {
    return GL_TRIANGLES;
}
