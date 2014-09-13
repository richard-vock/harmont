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
        orbit_camera_model();
        virtual ~orbit_camera_model();

    protected:
        void update_(vec3_t translational, vec3_t rotational, bool ortho_projection);

        void pan_(vec2_t delta);
        void zoom_(float delta, bool ortho);
        void rot_(vec3_t delta);

        void determine_matrix_();

    protected:
        float scale_  = 1.f;
        float phi_    = 0.f;
        float theta_  = 0.f;
        float psi_    = 0.f;
};


} // harmont

#endif /* ORBIT_CAMERA_MODEL_HPP_ */
