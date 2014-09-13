#ifndef CAMERA_MODEL_HPP_
#define CAMERA_MODEL_HPP_

#include "common.hpp"

namespace harmont {

class camera_model {
    public:
        typedef std::shared_ptr<camera_model>       ptr;
        typedef std::weak_ptr<camera_model>         wptr;
        typedef std::shared_ptr<const camera_model> const_ptr;
        typedef std::weak_ptr<const camera_model>   const_wptr;
        typedef Eigen::Vector3f                     vec2_t;
        typedef Eigen::Vector3f                     vec3_t;
        typedef Eigen::Matrix3f                     mat3_t;
        typedef Eigen::Matrix4f                     mat4_t;

    public:
        template <class Derived>
        static ptr looking_at(const vec3_t& position, const vec3_t& look_at = vec3_t::Zero());
        template <class Derived>
        static ptr looking_towards(const vec3_t& position, const vec3_t& forward);
        virtual ~camera_model();

        vec3_t position() const;
        vec3_t look_at() const;
        vec3_t forward() const;
        vec3_t up() const;
        vec3_t right() const;

        const mat4_t& view_matrix() const;
        const mat3_t& view_normal_matrix() const;

        void set_position(const vec3_t& position);
        void set_look_at(const vec3_t& look_at);
        void set_forward(const vec3_t& forward);
        void set_up(const vec3_t& up);
        void set_right(const vec3_t& right);

        void update(vec3_t translational, vec3_t rotational, bool ortho_projection);

    protected:
        camera_model();

        virtual void set_parameters(const vec3_t& position, const vec3_t& look_at, const vec3_t& up) = 0;
        virtual void update_(vec3_t translational, vec3_t rotational, bool ortho_projection) = 0;

    protected:
        vec3_t position_ = -vec3_t::UnitZ();
        vec3_t look_at_  = vec3_t::Zero();
        vec3_t up_       = vec3_t::UnitY();
        mat4_t trans_    = mat4_t::Identity();
        mat3_t normal_   = mat3_t::Identity();
        mat4_t math_to_opengl_;
        mat4_t opengl_to_math_;
};

#include "camera_model.ipp"

} // harmont

#endif /* CAMERA_MODEL_HPP_ */
