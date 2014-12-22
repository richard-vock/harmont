#include <z_plane_object.hpp>

namespace harmont {


z_plane_object::z_plane_object(float z, float x_size, float y_size, bool casts_shadows) : renderable(casts_shadows), z_(z), x_size_(x_size), y_size_(y_size) {
    compute_geometry_();
}

z_plane_object::~z_plane_object() {
}

void z_plane_object::init() {
    renderable::init(vertex_data_, index_data_);
}

typename z_plane_object::element_type_t z_plane_object::element_type() const {
    return VERTS;
}

bool z_plane_object::transparent() const {
    return false;
}

const float& z_plane_object::z() const {
    return z_;
}

void z_plane_object::set_z(const float& z) {
    z_ = z;
    compute_geometry_();
    init();
}

const float& z_plane_object::x_size() const {
    return x_size_;
}

void z_plane_object::set_x_size(const float& x_size) {
    x_size_ = x_size;
    compute_geometry_();
    init();
}

const float& z_plane_object::y_size() const {
    return y_size_;
}

void z_plane_object::set_y_size(const float& y_size) {
    y_size_ = y_size;
    compute_geometry_();
}

void z_plane_object::compute_geometry_() {
    //float col = renderable::color_to_rgba(Eigen::Vector4f(0.5f, 0.5f, 0.5f, 1.f));
    float col = renderable::color_to_rgba(Eigen::Vector4f::Ones());

    vertex_data_ = vertex_data_t(4, 9);
    vertex_data_ <<
        -0.5f * x_size_, -0.5f * y_size_, z_, col, 0.f, 0.f, 1.f, 0.f, 0.f,
         0.5f * x_size_, -0.5f * y_size_, z_, col, 0.f, 0.f, 1.f, 1.f, 0.f,
         0.5f * x_size_,  0.5f * y_size_, z_, col, 0.f, 0.f, 1.f, 1.f, 1.f,
        -0.5f * x_size_,  0.5f * y_size_, z_, col, 0.f, 0.f, 1.f, 0.f, 1.f;
    index_data_ = index_data_t(6, 1);
    index_data_ << 0, 1, 2, 0, 2, 3;
}

void z_plane_object::compute_bounding_box_() {
    bbox_ = bbox_t();
    bbox_.extend(Eigen::Vector3f(-0.5f * x_size_, -0.5f * y_size_, -0.01));
    bbox_.extend(Eigen::Vector3f( 0.5f * x_size_,  0.5f * y_size_,  0.01));
}

GLenum z_plane_object::gl_element_mode_() const {
    return GL_TRIANGLES;
}


} // harmont
