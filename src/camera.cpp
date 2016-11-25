#include <camera.hpp>

#include <view.hpp>

namespace harmont {

camera::camera(camera_model::ptr model, int width, int height, float fov, float near, float far, bool ortho) : model_(model), width_(width), height_(height), fov_(fov), near_(near), far_(far), ortho_(ortho) {
    float aspect = static_cast<float>(std::max(width, height)) / std::min(width, height);
    frustum_width_ = 2.f * near * aspect / fov;
    frustum_height_ = 2.f * near / fov;
    reshape(width_, height_);
}

camera::~camera() {
}

camera_model::ptr camera::model() {
    return model_;
}

camera_model::const_ptr camera::model() const {
    return model_;
}

void camera::set_model(camera_model::ptr model) {
    model->move_to(model_);
    model_ = model;
}

camera::vec3_t camera::position() const {
    return model_->position();
}

camera::vec3_t camera::look_at() const {
    return model_->look_at();
}

camera::vec3_t camera::forward() const {
    return model_->forward();
}

camera::vec3_t camera::up() const {
    return model_->up();
}

camera::vec3_t camera::right() const {
    return model_->right();
}

int camera::width() const {
    return width_;
}

int camera::height() const {
    return height_;
}

float camera::near() const {
    return near_;
}

float camera::far() const {
    return far_;
}

float camera::fov() const {
    return fov_;
}

float camera::frustum_width() const {
    return frustum_width_;
}

float camera::frustum_height() const {
    return frustum_height_;
}

camera::mat4_t camera::view_matrix() const {
    return model_->view_matrix();
}

camera::mat3_t camera::view_normal_matrix() const {
    return model_->view_normal_matrix();
}

const camera::mat4_t& camera::projection_matrix() const {
    return projection_;
}

camera::mat4_t camera::inverse_view_projection_matrix() const {
    return (projection_ * model_->view_matrix()).inverse();
}

void camera::set_near(float near) {
    near_ = near;
    reshape(width_, height_);
}

void camera::set_far(float far) {
    far_ = far;
    reshape(width_, height_);
}

void camera::set_near_far(float near, float far) {
    near_ = near;
    far_ = far;
    reshape(width_, height_);
}

void camera::set_fov(float fov) {
    fov_ = fov;
    reshape(width_, height_);
}

bool camera::ortho() const {
    return ortho_;
}

void camera::set_ortho(bool state) {
    ortho_ = state;
    reshape(width_, height_);
}

void camera::toggle_ortho() {
    ortho_ = !ortho_;
    reshape(width_, height_);
}

std::vector<camera::vec3_t> camera::frustum_corners() const {
    mat4_t ivp = inverse_view_projection_matrix();

    Eigen::Matrix<float, 4, 8> c;
    c << -1.f,  1.f,  1.f, -1.f, -1.f,  1.f,  1.f, -1.f,
         -1.f, -1.f,  1.f,  1.f, -1.f, -1.f,  1.f,  1.f,
         -1.f, -1.f, -1.f, -1.f,  1.f,  1.f,  1.f,  1.f,
          1.f,  1.f,  1.f,  1.f,  1.f,  1.f,  1.f,  1.f;
    Eigen::Matrix<float, 4, 8> p = ivp * c;

    std::vector<camera::vec3_t> result(8);
    for (uint32_t i = 0; i < result.size(); ++i) {
        result[i] = p.block<3,1>(0, i) / p(3, i);
    }

    return result;
}

camera::ray_t camera::pick_ray(int x, int y) const {
    mat4_t view_mat = model_->view_matrix();
    Eigen::Matrix<int,4,1> viewport(0, 0, width_, height_);
    vec3_t origin = unproject(vec3_t(static_cast<float>(x), static_cast<float>(y), 0.f), view_mat, projection_, viewport);
    vec3_t direction = unproject(vec3_t(static_cast<float>(x), static_cast<float>(y), 1.f), view_mat, projection_, viewport) - origin;
    direction.normalize();
    return {origin, direction};
}

void camera::reshape(int width, int height) {
    width_ = width;
    height_ = height;

    float aspect = static_cast<float>(std::max(width, height)) / std::min(width, height);
	projection_ = perspective(fov_, aspect, near_, far_);

    float f = 1.0 / tan(fov_ * M_PI / 360.f);
    frustum_width_ = 2.f*near_*aspect / f;
    frustum_height_ = 2.f*near_ / f;

	if (ortho_) {
        mat4_t view_mat = model_->view_matrix();
		Eigen::Vector4f look_at;
		look_at << model_->look_at(), 1.f;
		look_at = view_mat * look_at;
		Eigen::Vector4f right = look_at;
		right[0] += 1.f;

		float factor = static_cast<float>(tan(fov_ * M_PI / 180.f) * 20.0);
		mat4_t ortho_mat = harmont::ortho(-aspect*factor, aspect*factor, -factor, factor, near_, far_);
		Eigen::Vector4f p0 = projection_*look_at, p1 = projection_*right, o0 = ortho_mat*look_at, o1 = ortho_mat*right;
		p0.head(3) /= p0[3]; p1.head(3) /= p1[3];
		float scale = (p1.head(3)-p0.head(3)).norm() / (o1.head(3)-o0.head(3)).norm();
		mat4_t scale_mat = mat4_t::Identity();
		scale_mat.block<3,3>(0,0) *= scale;
		projection_ = ortho_mat * scale_mat;
	}
}

void camera::update(vec3_t translational, vec3_t rotational) {
    model_->update(translational, rotational, ortho_);
}

} // harmont
