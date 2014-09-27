#ifndef ORBIT_CAMERA_MODEL_HPP_
#define ORBIT_CAMERA_MODEL_HPP_

#include "camera_model.hpp"

namespace harmont {

class orbit_camera_model : public camera_model {
    public:
        typedef std::shared_ptr<orbit_camera_model>       ptr;
        typedef std::weak_ptr<orbit_camera_model>         wptr;
        typedef std::shared_ptr<const orbit_camera_model> const_ptr;
        typedef std::weak_ptr<const orbit_camera_model>   const_wptr;

    public:
        static ptr from_looking_at(const vec3_t& position, const vec3_t& look_at);
        orbit_camera_model();
        virtual ~orbit_camera_model();

        vec3_t position() const;
        vec3_t look_at() const;

        void move_to(camera_model::ptr other);
        void move_to(orbit_camera_model::ptr other);

    protected:
        void update_(vec3_t translational, vec3_t rotational, bool ortho_projection);

        void pan_(vec2_t delta);
        void zoom_(float delta, bool ortho);
        void rot_(vec3_t delta);

        void determine_matrix_();

    protected:
        vec3_t look_at_  = vec3_t::Zero();
        float  phi_      = 0.f;
        float  theta_    = 0.f;
        float  psi_      = 0.f;
        float  radius_   = 20.f;
};


} // harmont

#endif /* ORBIT_CAMERA_MODEL_HPP_ */
