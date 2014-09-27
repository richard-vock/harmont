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
        typedef Eigen::Vector2f                     vec2_t;
        typedef Eigen::Vector3f                     vec3_t;
        typedef Eigen::Vector4f                     vec4_t;
        typedef Eigen::Matrix3f                     mat3_t;
        typedef Eigen::Matrix4f                     mat4_t;

    public:
        template <class Derived>
        static ptr looking_at(const vec3_t& position, const vec3_t& look_at = vec3_t::Zero());
        template <class Derived>
        static ptr looking_towards(const vec3_t& position, const vec3_t& forward);
        virtual ~camera_model();

        virtual vec3_t position() const = 0;
        virtual vec3_t look_at() const = 0;
        float  scale() const;
        vec3_t forward() const;
        vec3_t up() const;
        vec3_t right() const;

        mat4_t view_matrix() const;
        mat3_t view_normal_matrix() const;

        void update(vec3_t translational, vec3_t rotational, bool ortho_projection);

        virtual void move_to(camera_model::ptr other) = 0;

    protected:
        camera_model();

        virtual void update_(vec3_t translational, vec3_t rotational, bool ortho_projection) = 0;

    protected:
        float  scale_    = 1.f;
        mat4_t trans_    = mat4_t::Identity();
        mat3_t normal_   = mat3_t::Identity();
        mat4_t math_to_opengl_;
        mat4_t opengl_to_math_;
};

#include "camera_model.ipp"

} // harmont

#endif /* CAMERA_MODEL_HPP_ */
