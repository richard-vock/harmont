#ifndef APPLICATION_HPP_
#define APPLICATION_HPP_

#include "common.hpp"
#include "camera.hpp"

namespace harmont {

class application {
    public:
        typedef std::shared_ptr<application>       ptr;
        typedef std::weak_ptr<application>         wptr;
        typedef std::shared_ptr<const application> const_ptr;
        typedef std::weak_ptr<const application>   const_wptr;
        template <typename... Args>
        using callback_t = std::function<void (Args...)>;

    public:
        application(int width, int height, camera::ptr cam, callback_t<camera::ptr> render_callback, callback_t<camera::ptr> reshape_callback = nullptr);
        virtual ~application();

        void init(int argc, char* argv[], std::string title, callback_t<> init_callback);
        virtual void run() = 0;

        virtual void update() = 0;

        int width() const;
        int height() const;
        camera::ptr current_camera();
        camera::const_ptr current_camera() const;

    protected:
        virtual void init_(int argc, char* argv[], std::string title) = 0;

        void render_();
        void reshape_(int width, int height);

        template <class CameraModel>
        static camera::ptr default_camera_(int width, int height);

    protected:
        int                      width_;
        int                      height_;
        callback_t<camera::ptr>  cb_render_;
        callback_t<camera::ptr>  cb_reshape_;
        camera::ptr              camera_;
};

#include "application.ipp"

} // harmont

#endif /* APPLICATION_HPP_ */
