#include <camera_model.hpp>

namespace harmont {


camera_model::~camera_model() {
}

camera_model::vec3_t camera_model::position() const {
    return position_;
}

camera_model::vec3_t camera_model::look_at() const {
    return look_at_;
}

camera_model::vec3_t camera_model::forward() const {
    return trans_.block<1,3>(2, 0).transpose().normalized();
}

camera_model::vec3_t camera_model::up() const {
    return trans_.block<1,3>(1, 0).transpose().normalized();
}

camera_model::vec3_t camera_model::right() const {
    return trans_.block<1,3>(0, 0).transpose().normalized();
}

const camera_model::mat4_t& camera_model::view_matrix() const {
    return trans_;
}

const camera_model::mat3_t& camera_model::view_normal_matrix() const {
    return normal_;
}

void camera_model::set_position(const vec3_t& position) {
    set_parameters(position, look_at_, up());
    position_ = position;
}

void camera_model::set_look_at(const vec3_t& look_at) {
    set_parameters(position_, look_at, up());
    look_at_ = look_at;
}

void camera_model::set_forward(const vec3_t& forward) {
    vec3_t look_at = position_ + forward.normalized();
    set_parameters(position_, look_at, up());
    look_at_ = look_at;
}

void camera_model::set_up(const vec3_t& up) {
    set_parameters(position_, look_at_, up.normalized());
}

void camera_model::set_right(const vec3_t& right) {
    set_parameters(position_, look_at_, forward().cross(right.normalized()));
}

void camera_model::update(vec3_t translational, vec3_t rotational, bool ortho_projection) {
    update_(translational, rotational, ortho_projection);
    normal_ = trans_.topLeftCorner<3,3>();
    float scale = (trans_ * vec3_t::UnitZ().homogeneous()).head(3).norm();
    if (fabs(1.f - scale) > 0.0001) {
        normal_ = normal_.inverse();
        normal_.transposeInPlace();
    }
}

camera_model::camera_model() {
}

} // harmont
