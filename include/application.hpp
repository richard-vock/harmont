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
        typedef Eigen::Vector2i                    screen_pos_t;
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

        void on_mouse_move(callback_t<screen_pos_t, screen_pos_t> callback);
        void on_click_left(callback_t<screen_pos_t> callback);
        void on_click_right(callback_t<screen_pos_t> callback);
        void on_click_middle(callback_t<screen_pos_t> callback);
        void on_drag_start_left(callback_t<screen_pos_t> callback);
        void on_drag_start_right(callback_t<screen_pos_t> callback);
        void on_drag_start_middle(callback_t<screen_pos_t> callback);
        void on_drag_stop_left(callback_t<screen_pos_t, screen_pos_t> callback);
        void on_drag_stop_right(callback_t<screen_pos_t, screen_pos_t> callback);
        void on_drag_stop_middle(callback_t<screen_pos_t, screen_pos_t> callback);
        void on_drag_left(callback_t<screen_pos_t, screen_pos_t> callback);
        void on_drag_right(callback_t<screen_pos_t, screen_pos_t> callback);
        void on_drag_middle(callback_t<screen_pos_t, screen_pos_t> callback);
        void on_scroll(callback_t<int> callback);
        void on_char(callback_t<unsigned char> callback);

    protected:
        virtual void init_(int argc, char* argv[], std::string title) = 0;

        void render_();
        void reshape_(int width, int height);

        void mouse_move_(screen_pos_t pos, screen_pos_t delta);
        void click_left_(screen_pos_t pos);
        void click_right_(screen_pos_t pos);
        void click_middle_(screen_pos_t pos);
        void drag_start_left_(screen_pos_t pos);
        void drag_start_right_(screen_pos_t pos);
        void drag_start_middle_(screen_pos_t pos);
        void drag_stop_left_(screen_pos_t start, screen_pos_t pos);
        void drag_stop_right_(screen_pos_t start, screen_pos_t pos);
        void drag_stop_middle_(screen_pos_t start, screen_pos_t pos);
        void drag_left_(screen_pos_t pos, screen_pos_t delta);
        void drag_right_(screen_pos_t pos, screen_pos_t delta);
        void drag_middle_(screen_pos_t pos, screen_pos_t delta);
        void scroll_(int delta);
        void char_(unsigned char key);

        template <class CameraModel>
        static camera::ptr default_camera_(int width, int height);

    protected:
        int                      width_;
        int                      height_;
        camera::ptr              camera_;
        callback_t<camera::ptr>  cb_render_;
        callback_t<camera::ptr>  cb_reshape_;

        callback_t<screen_pos_t, screen_pos_t> cb_mouse_move_;
        callback_t<screen_pos_t>               cb_click_left_;
        callback_t<screen_pos_t>               cb_click_right_;
        callback_t<screen_pos_t>               cb_click_middle_;
        callback_t<screen_pos_t>               cb_drag_start_left_;
        callback_t<screen_pos_t>               cb_drag_start_right_;
        callback_t<screen_pos_t>               cb_drag_start_middle_;
        callback_t<screen_pos_t, screen_pos_t> cb_drag_stop_left_;
        callback_t<screen_pos_t, screen_pos_t> cb_drag_stop_right_;
        callback_t<screen_pos_t, screen_pos_t> cb_drag_stop_middle_;
        callback_t<screen_pos_t, screen_pos_t> cb_drag_left_;
        callback_t<screen_pos_t, screen_pos_t> cb_drag_right_;
        callback_t<screen_pos_t, screen_pos_t> cb_drag_middle_;
        callback_t<int>                        cb_scroll_;
        callback_t<unsigned char>              cb_char_;
};

#include "application.ipp"

} // harmont

#endif /* APPLICATION_HPP_ */
