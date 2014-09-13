#include <orbit_camera_model.hpp>


namespace harmont {

orbit_camera_model::orbit_camera_model() : camera_model() {
    position_ = -20.f * vec3_t::UnitY();
    update(vec3_t::Zero(), vec3_t::Zero(), false);
}

orbit_camera_model::~orbit_camera_model() {
}

void orbit_camera_model::update_(vec3_t translational, vec3_t rotational, bool ortho_projection) {
    if (translational.head(2).squaredNorm() > tiny<float>()) {
        pan_(translational.head(2));
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
    position_ += offset;
    look_at_ += offset;
}

void orbit_camera_model::zoom_(float delta, bool ortho) {
    if (ortho) {
        scale_ += delta;
    } else {
        vec3_t forward = position_ - look_at_;
        float radius = forward.norm();
        forward.normalize();
        radius = std::min((radius - delta), tiny<float>());
        position_ = look_at_ - radius * forward;
    }
}

void orbit_camera_model::rot_(vec3_t delta) {
	float pi2 = static_cast<float>(2.f * M_PI);
	float pdiv2 = static_cast<float>(0.5f * M_PI);

    phi_   += delta[0];
    theta_ += delta[1];
    psi_   += delta[2];

    // correctly bound values
    while (phi_ <    0) phi_ += pi2;
    while (phi_ >= pi2) phi_ -= pi2;
    if (theta_ < -pdiv2) theta_ = -pdiv2;
    if (theta_ >  pdiv2) theta_ =  pdiv2;
    if (psi_ < -pdiv2) psi_ = -pdiv2;
    if (psi_ >  pdiv2) psi_ =  pdiv2;

    // update position
    float radius = (look_at_ - position_).norm();
	position_ = opengl_to_math_.block<3,3>(0,0)
		  * ((Eigen::AngleAxisf(psi_  , Eigen::Vector3f::UnitZ())
		  *   Eigen::AngleAxisf(phi_  , Eigen::Vector3f::UnitY())
	      *   Eigen::AngleAxisf(theta_, Eigen::Vector3f::UnitX()))
	      *   Eigen::Vector3f(0.f, 0.f, radius)) + look_at_;
}

void orbit_camera_model::determine_matrix_() {
    Eigen::Affine3f t;
    float radius = (look_at_ - position_).norm();
	t = Eigen::Translation<float,3>(-radius*Eigen::Vector3f::UnitZ())
	  * Eigen::AngleAxisf(-theta_, Eigen::Vector3f::UnitX())
	  * Eigen::AngleAxisf(-phi_  , Eigen::Vector3f::UnitY())
	  * Eigen::AngleAxisf(-psi_  , Eigen::Vector3f::UnitZ())
	  * Eigen::Translation<float,3>(math_to_opengl_.block<3,3>(0,0) * (-look_at_));
    mat4_t scale = mat4_t::Identity();
    scale.block<3,3>(0,0) *= scale_;
	trans_ = t.matrix() * scale;
}


} // harmont
