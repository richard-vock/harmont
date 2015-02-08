#ifndef FLY_CAMERA_MODEL_HPP_
#define FLY_CAMERA_MODEL_HPP_

#include "camera_model.hpp"

namespace harmont {

class fly_camera_model : public camera_model {
    public:
        typedef std::shared_ptr<fly_camera_model>       ptr;
        typedef std::weak_ptr<fly_camera_model>         wptr;
        typedef std::shared_ptr<const fly_camera_model> const_ptr;
        typedef std::weak_ptr<const fly_camera_model>   const_wptr;

    public:
        static ptr from_looking_at(const vec3_t& position, const vec3_t& look_at);
        fly_camera_model();
        virtual ~fly_camera_model();

        vec3_t position() const;
        vec3_t look_at() const;

        void move_to(camera_model::ptr other);
        void move_to(fly_camera_model::ptr other);

    protected:
        void update_(vec3_t translational, vec3_t rotational, bool ortho_projection);

        void pan_(vec2_t delta);
        void zoom_(float delta, bool ortho);
        void rot_(vec3_t delta);

        void determine_matrix_();

        std::tuple<float, float, float> angles_from_forward_(const vec3_t& forward);

    protected:
        vec3_t pos_      = vec3_t::Zero();
        float  phi_      = 0.f;
        float  theta_    = 0.f;
        float  psi_      = 0.f;
};


} // harmont

#endif /* FLY_CAMERA_MODEL_HPP_ */
