#include <camera_model.hpp>

namespace harmont {


camera_model::~camera_model() {
}

float camera_model::scale() const {
    return scale_;
}

camera_model::vec3_t camera_model::forward() const {
    return opengl_to_math_.block<3,3>(0, 0) * trans_.block<1,3>(2, 0).transpose().normalized();
}

camera_model::vec3_t camera_model::up() const {
    return opengl_to_math_.block<3,3>(0, 0) * trans_.block<1,3>(1, 0).transpose().normalized();
}

camera_model::vec3_t camera_model::right() const {
    return opengl_to_math_.block<3,3>(0, 0) * trans_.block<1,3>(0, 0).transpose().normalized();
}

camera_model::mat4_t camera_model::view_matrix() const {
    mat4_t scale = mat4_t::Identity();
    scale.block<3,3>(0,0) *= scale_;
    return trans_ * scale * math_to_opengl_;
}

camera_model::mat3_t camera_model::view_normal_matrix() const {
    return normal_ * math_to_opengl_.block<3,3>(0,0);
}

void camera_model::update(vec3_t translational, vec3_t rotational, bool ortho_projection) {
    update_(translational, rotational, ortho_projection);
    normal_ = trans_.topLeftCorner<3,3>();
    //if (fabs(1.f - scale_) > 0.0001) {
        //normal_ = normal_.inverse();
        //normal_.transposeInPlace();
    //}
}

camera_model::camera_model() {
    math_to_opengl_ <<  1.f, 0.f, 0.f, 0.f,
                        0.f, 0.f, 1.f, 0.f,
                        0.f, -1.f, 0.f, 0.f,
                        0.f, 0.f, 0.f, 1.f;
    opengl_to_math_ <<  1.f, 0.f, 0.f, 0.f,
                        0.f, 0.f, -1.f, 0.f,
                        0.f, 1.f, 0.f, 0.f,
                        0.f, 0.f, 0.f, 1.f;
}


} // harmont
