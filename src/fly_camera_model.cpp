#include <fly_camera_model.hpp>


namespace harmont {

fly_camera_model::ptr fly_camera_model::from_looking_at(const vec3_t& position, const vec3_t& look_at) {
    ptr model = std::make_shared<fly_camera_model>();
    model->scale_ = 1.f;
    model->pos_ = model->math_to_opengl_.block<3,3>(0,0) * position;

    vec3_t forward = (look_at - position).normalized();
    std::tie(model->phi_, model->theta_, model->psi_) = model->angles_from_forward_(forward);

    return model;
}

fly_camera_model::fly_camera_model() : camera_model() {
    update(vec3_t::Zero(), vec3_t::Zero(), false);
}

fly_camera_model::~fly_camera_model() {
}

fly_camera_model::vec3_t fly_camera_model::position() const {
    return opengl_to_math_.block<3,3>(0, 0) * pos_;
}

fly_camera_model::vec3_t fly_camera_model::look_at() const {
    return opengl_to_math_.block<3,3>(0,0) * pos_ + forward();
}

void fly_camera_model::move_to(fly_camera_model::ptr other) {
    scale_    = other->scale();
    pos_      = other->pos_;
    phi_      = other->phi_;
    theta_    = other->theta_;
    psi_      = other->psi_;
}

void fly_camera_model::move_to(camera_model::ptr other) {
    scale_    = other->scale();
    pos_      = math_to_opengl_.block<3,3>(0,0) * other->position();
    vec3_t forward = (other->look_at() - other->position()).normalized();

    std::tie(phi_, theta_, psi_) = angles_from_forward_(forward);
}

void fly_camera_model::update_(vec3_t translational, vec3_t rotational, bool ortho_projection) {
    if (translational.head(2).squaredNorm() > eps<float>()) {
        pan_(vec2_t(-translational[0], translational[1]));
    }

    if (fabs(translational[2]) > eps<float>()) {
        zoom_(translational[2], ortho_projection);
    }

    if (rotational.squaredNorm() > eps<float>()) {
        rot_(rotational);
    }

    determine_matrix_();
}

void fly_camera_model::pan_(vec2_t delta) {
    vec3_t offset = trans_.topLeftCorner<3,3>().transpose() * vec3_t(delta[0], delta[1], 0.f);
    pos_ += offset;
}

void fly_camera_model::zoom_(float delta, bool ortho) {
    if (ortho) {
        scale_ += delta;
    } else {
        pos_ += -delta * trans_.block<1,3>(2, 0).transpose();
    }
}

void fly_camera_model::rot_(vec3_t delta) {
	float pi2 = static_cast<float>(2.f * M_PI);
	float pdiv2 = static_cast<float>(0.5f * M_PI);

    phi_   -= delta[0];
    theta_ -= delta[1];
    psi_   -= delta[2];

    // correctly bound values
    while (phi_ <  M_PI) phi_ += pi2;
    while (phi_ >= M_PI) phi_ -= pi2;
    if (theta_ < -pdiv2) theta_ = -pdiv2;
    if (theta_ >  pdiv2) theta_ =  pdiv2;
    if (psi_ < -pdiv2) psi_ = -pdiv2;
    if (psi_ >  pdiv2) psi_ =  pdiv2;
}

void fly_camera_model::determine_matrix_() {
    Eigen::Affine3f t;
    // note that this is the inverted matrix
    t = Eigen::AngleAxisf(-theta_, vec3_t::UnitX())
      * Eigen::AngleAxisf(-phi_,   vec3_t::UnitY())
      * Eigen::Translation<float,3>(-pos_);
	trans_ = t.matrix();
}

std::tuple<float, float, float> fly_camera_model::angles_from_forward_(const vec3_t& forward) {
    vec3_t f = math_to_opengl_.block<3,3>(0,0) * forward;
    float theta = asin(f[1]);//acos(f[1]) - 0.5f * M_PI;
    float phi = atan2(-f[0], -f[2]);
    // TODO
    float psi = 0.f;
    return std::make_tuple(phi, theta, psi);
}


} // harmont
