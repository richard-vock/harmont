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
    return opengl_to_math_.block<3,3>(0, 0) * trans_.block<1,3>(2, 0).transpose().normalized();
}

camera_model::vec3_t camera_model::up() const {
    return opengl_to_math_.block<3,3>(0, 0) * trans_.block<1,3>(1, 0).transpose().normalized();
}

camera_model::vec3_t camera_model::right() const {
    return opengl_to_math_.block<3,3>(0, 0) * trans_.block<1,3>(0, 0).transpose().normalized();
}

const camera_model::mat4_t& camera_model::view_matrix() const {
    return trans_;
}

const camera_model::mat3_t& camera_model::view_normal_matrix() const {
    return normal_;
}

void camera_model::set_position(const vec3_t& position) {
    position_ = position;
    update(vec3_t::Zero(), vec3_t::Zero(), false);
}

void camera_model::set_look_at(const vec3_t& look_at) {
    look_at_ = look_at;
    update(vec3_t::Zero(), vec3_t::Zero(), false);
}

void camera_model::set_forward(const vec3_t& forward) {
    vec3_t look_at = position_ + forward.normalized();
    look_at_ = look_at;
    update(vec3_t::Zero(), vec3_t::Zero(), false);
}

void camera_model::set_up(const vec3_t& up) {
    up_ = up.normalized();
    update(vec3_t::Zero(), vec3_t::Zero(), false);
}

void camera_model::set_right(const vec3_t& right) {
    vec3_t forward = (look_at_ - position_).normalized();
    up_ = forward.cross(right).normalized();
    update(vec3_t::Zero(), vec3_t::Zero(), false);
}

void camera_model::update(vec3_t translational, vec3_t rotational, bool ortho_projection) {
    update_(translational, rotational, ortho_projection);
    float scale = (trans_ * vec3_t::UnitZ().homogeneous()).head(3).norm();
    normal_ = trans_.topLeftCorner<3,3>();
    if (fabs(1.f - scale) > 0.0001) {
        normal_ = normal_.inverse();
        normal_.transposeInPlace();
    }
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
