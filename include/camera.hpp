#ifndef CAMERA_HPP_
#define CAMERA_HPP_

#include "common.hpp"
#include "camera_model.hpp"

namespace harmont {


class camera {
    public:
        typedef std::shared_ptr<camera>       ptr;
        typedef std::weak_ptr<camera>         wptr;
        typedef std::shared_ptr<const camera> const_ptr;
        typedef std::weak_ptr<const camera>   const_wptr;
        typedef camera_model::vec2_t          vec2_t;
        typedef camera_model::vec3_t          vec3_t;
        typedef camera_model::mat3_t          mat3_t;
        typedef camera_model::mat4_t          mat4_t;
        typedef std::pair<vec3_t, vec3_t>     ray_t;

    public:
        camera(camera_model::ptr model, int width, int height, float fov, float near, float far, bool ortho = false);
        virtual ~camera();

        camera_model::ptr model();
        camera_model::const_ptr model() const;
        void set_model(camera_model::ptr model);

        vec3_t position() const;
        vec3_t look_at() const;
        vec3_t forward() const;
        vec3_t up() const;
        vec3_t right() const;
        int    width() const;
        int    height() const;
        float  near() const;
        float  far() const;
        float  fov() const;
        float  frustum_width() const;
        float  frustum_height() const;

        mat4_t view_matrix() const;
        mat3_t view_normal_matrix() const;
        const mat4_t& projection_matrix() const;

        void set_position(const vec3_t& position);
        void set_look_at(const vec3_t& look_at);
        void set_forward(const vec3_t& forward);
        void set_up(const vec3_t& up);
        void set_right(const vec3_t& right);
        void set_near(float near);
        void set_far(float far);
        void set_near_far(float near, float far);
        void set_fov(float fov);

        bool ortho() const;
        void set_ortho(bool state);
        void toggle_ortho();

        ray_t pick_ray(int x, int y) const;

        void reshape(int width, int height);
        void update(vec3_t translational, vec3_t rotational);

    protected:
        camera_model::ptr model_;
        int               width_;
        int               height_;
        float             fov_;
        float             near_;
        float             far_;
        float             frustum_width_;
        float             frustum_height_;
        bool              ortho_;
        mat4_t            projection_;
};


} // harmont

#endif /* CAMERA_HPP_ */
