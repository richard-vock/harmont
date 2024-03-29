#ifndef FREEGLUT_APPLICATION_HPP_
#define FREEGLUT_APPLICATION_HPP_

#include "application.hpp"

namespace harmont {

class freeglut_application : public application {
    public:
        typedef std::shared_ptr<freeglut_application>       ptr;
        typedef std::weak_ptr<freeglut_application>         wptr;
        typedef std::shared_ptr<const freeglut_application> const_ptr;
        typedef std::weak_ptr<const freeglut_application>   const_wptr;
        //template <typename... Args>
        //using callback_t = application::callback_t<Args...>;


    public:
        freeglut_application(int width, int height, camera::ptr cam, callback_t<camera::ptr> render_callback, callback_t<camera::ptr> reshape_callback = nullptr);
        template <class CameraModel>
        static ptr create(int width, int height, callback_t<camera::ptr> render_callback, callback_t<camera::ptr> reshape_callback = nullptr);
        virtual ~freeglut_application();

        void run();
        void update();

    protected:
        void init_(int argc, char* argv[], std::string title);
        void display_();
        void mouse_(int button, int state, int x, int y);
        void motion_(int x, int y);

        static void glut_display_(freeglut_application* self);
        static void glut_reshape_(freeglut_application* self, int width, int height);
        static void glut_mouse_(freeglut_application* self, int button, int state, int x, int y);
        static void glut_mouse_move_(freeglut_application* self, int x, int y);
        static void glut_keyboard_(freeglut_application* self, unsigned char key);

    protected:
        screen_pos_t  last_pos_ = screen_pos_t(0, 0);
        screen_pos_t  drag_start_ = screen_pos_t(0, 0);
        bool          dragging_ = false;
        bool          left_down_ = false;
        bool          right_down_ = false;
        bool          middle_down_ = false;
};

#include "freeglut_application.ipp"

} // harmont

#endif /* FREEGLUT_APPLICATION_HPP_ */
