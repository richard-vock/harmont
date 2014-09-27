#include <orbit_camera_model.hpp>


namespace harmont {

orbit_camera_model::ptr orbit_camera_model::from_looking_at(const vec3_t& position, const vec3_t& look_at) {
    ptr model = std::make_shared<orbit_camera_model>();
    model->scale_ = 1.f;
    model->look_at_ = model->math_to_opengl_.block<3,3>(0,0) * look_at;

    vec3_t pos = model->math_to_opengl_.block<3,3>(0,0) * (position - look_at);
    model->radius_ = pos.norm();
    pos.normalize();
    model->theta_ = acos(pos[2]);
    pos[1] = 0.f;
    float len = pos.norm();
    if (len < Eigen::NumTraits<float>::dummy_precision()) {
        model->phi_ = 0.f;
    } else {
        pos /= len;
        model->phi_ = atan2(pos[0], pos[2]);
        if (model->phi_ < 0.f) model->phi_ += 2.f * static_cast<float>(M_PI);
    }

    // TODO: Implement
    model->psi_ = 0.f;

    return model;
}

orbit_camera_model::orbit_camera_model() : camera_model() {
    update(vec3_t::Zero(), vec3_t::Zero(), false);
}

orbit_camera_model::~orbit_camera_model() {
}

orbit_camera_model::vec3_t orbit_camera_model::position() const {
    Eigen::Affine3f t;
	t = Eigen::Translation<float,3>(look_at_)
	  //* Eigen::AngleAxisf(-psi_  , vec3_t::UnitZ())
      * Eigen::AngleAxisf(-phi_  , vec3_t::UnitY())
      * Eigen::AngleAxisf(-theta_, vec3_t::UnitX())
      * Eigen::Translation<float,3>(-radius_*vec3_t::UnitZ());
    return opengl_to_math_.block<3,3>(0,0) * (t * vec4_t::UnitW()).head(3);
}

orbit_camera_model::vec3_t orbit_camera_model::look_at() const {
    return opengl_to_math_.block<3,3>(0,0) * look_at_;
}

void orbit_camera_model::move_to(orbit_camera_model::ptr other) {
    scale_    = other->scale();
    phi_      = other->phi_;
    theta_    = other->theta_;
    psi_      = other->psi_;
    radius_   = other->radius_;
}

void orbit_camera_model::move_to(camera_model::ptr other) {
    scale_    = other->scale();
    look_at_ = other->look_at();
    vec3_t pos = (math_to_opengl_.block<3,3>(0,0) * other->position()) - look_at_;
    radius_ = pos.norm();
    pos.normalize();
    theta_ = acos(pos[2]);
    pos[1] = 0.f;
    float len = pos.norm();
    if (len < Eigen::NumTraits<float>::dummy_precision()) {
        phi_ = 0.f;
    } else {
        pos /= len;
        phi_ = atan2(pos[0], pos[2]);
        if (phi_ < 0.f) phi_ += 2.f * static_cast<float>(M_PI);
    }

    // TODO: Implement
    psi_ = 0.f;
}

void orbit_camera_model::update_(vec3_t translational, vec3_t rotational, bool ortho_projection) {
    if (translational.head(2).squaredNorm() > tiny<float>()) {
        pan_(vec2_t(translational[0], -translational[1]));
    }

    if (fabs(translational[2]) > tiny<float>()) {
        zoom_(translational[2], ortho_projection);
    }

    if (rotational.squaredNorm() > tiny<float>()) {
        rot_(rotational);
    }

    determine_matrix_();
}

void orbit_camera_model::pan_(vec2_t delta) {
    vec3_t offset = trans_.topLeftCorner<3,3>().transpose() * vec3_t(delta[0], delta[1], 0.f);
    look_at_ += offset;
}

void orbit_camera_model::zoom_(float delta, bool ortho) {
    if (ortho) {
        scale_ += delta;
    } else {
        radius_ = std::max((radius_ - delta), tiny<float>());
    }
}

void orbit_camera_model::rot_(vec3_t delta) {
	float pi2 = static_cast<float>(2.f * M_PI);
	float pdiv2 = static_cast<float>(0.5f * M_PI);

    phi_   -= delta[0];
    theta_ -= delta[1];
    psi_   -= delta[2];

    // correctly bound values
    while (phi_ <    0) phi_ += pi2;
    while (phi_ >= pi2) phi_ -= pi2;
    if (theta_ < -pdiv2) theta_ = -pdiv2;
    if (theta_ >  pdiv2) theta_ =  pdiv2;
    if (psi_ < -pdiv2) psi_ = -pdiv2;
    if (psi_ >  pdiv2) psi_ =  pdiv2;
}

void orbit_camera_model::determine_matrix_() {
    Eigen::Affine3f t;
	t = Eigen::Translation<float,3>(-radius_*vec3_t::UnitZ())
	  * Eigen::AngleAxisf(-theta_, vec3_t::UnitX())
	  * Eigen::AngleAxisf(-phi_  , vec3_t::UnitY())
	  //* Eigen::AngleAxisf(-psi_  , vec3_t::UnitZ())
	  * Eigen::Translation<float,3>(-look_at_);
	trans_ = t.matrix();
}


} // harmont
