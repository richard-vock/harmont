#include <freeglut_application.hpp>

#include <GL/glew.h>
#include <GL/freeglut.h>

namespace harmont {

freeglut_application* application_instance = nullptr;

freeglut_application::freeglut_application(int width, int height, camera::ptr cam, callback_t<camera::ptr> render_callback, callback_t<camera::ptr> reshape_callback) : application(width, height, cam, std::move(render_callback), std::move(reshape_callback)) {
    application_instance = this;
}

freeglut_application::~freeglut_application() {
}

void freeglut_application::run() {
    glutDisplayFunc([] () { freeglut_application::glut_display_(application_instance); });
    glutReshapeFunc([] (int width, int height) { freeglut_application::glut_reshape_(application_instance, width, height); });
    glutMouseFunc([] (int button, int state, int x, int y) { freeglut_application::glut_mouse_(application_instance, button, state, x, y); });
    glutMotionFunc([] (int x, int y) { freeglut_application::glut_mouse_move_(application_instance, x, y); });
    glutPassiveMotionFunc([] (int x, int y) { freeglut_application::glut_mouse_move_(application_instance, x, y); });
    glutKeyboardFunc([] (unsigned char key, int, int) { freeglut_application::glut_keyboard_(application_instance, key); });
    glutMainLoop();
}

void freeglut_application::update() {
    glutPostRedisplay();
}

void freeglut_application::init_(int argc, char* argv[], std::string title) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(width_, height_);
    glutInitWindowPosition(100, 100);
    glutCreateWindow(title.c_str());
    glewInit();
}

void freeglut_application::display_() {
    render_();
    glutSwapBuffers();
}

void freeglut_application::glut_display_(freeglut_application* self) {
    self->display_();
}

void freeglut_application::glut_reshape_(freeglut_application* self, int width, int height) {
    self->reshape_(width, height);
}

void freeglut_application::mouse_(int button, int state, int x, int y) {
    screen_pos_t crt(x, y);
    if (state == GLUT_DOWN) {
        switch (button) {
            case GLUT_LEFT_BUTTON:    drag_start_ = crt; left_down_ = true; break;
            case GLUT_RIGHT_BUTTON:   drag_start_ = crt; right_down_ = true; break;
            case GLUT_MIDDLE_BUTTON:  drag_start_ = crt; middle_down_ = true; break;
            case 3: scroll_(1); break;
            case 4: scroll_(-1); break;
        }
    } else {
        if (button == GLUT_LEFT_BUTTON) {
            left_down_ = false;
            if (dragging_) {
                drag_stop_left_(drag_start_, crt);
                dragging_ = false;
            } else {
                click_left_(drag_start_);
            }
        } else if (button == GLUT_RIGHT_BUTTON) {
            right_down_ = false;
            if (dragging_) {
                drag_stop_right_(drag_start_, crt);
                dragging_ = false;
            } else {
                click_right_(drag_start_);
            }
        } else {
            middle_down_ = false;
            if (dragging_) {
                drag_stop_middle_(drag_start_, crt);
                dragging_ = false;
            } else {
                click_middle_(drag_start_);
            }
        }
    }
}

void freeglut_application::motion_(int x, int y) {
    screen_pos_t crt(x, y);
    if ((left_down_ || right_down_ || middle_down_) && ((crt - drag_start_).template lpNorm<1>() > 2)) {
        if (!dragging_) {
            if (left_down_) drag_start_left_(drag_start_);
            if (right_down_) drag_start_right_(drag_start_);
            if (middle_down_) drag_start_middle_(drag_start_);
        }
        dragging_ = true;
    }

    if (dragging_) {
        if (left_down_) drag_left_(crt, crt - last_pos_);
        if (right_down_) drag_right_(crt, crt - last_pos_);
        if (middle_down_) drag_middle_(crt, crt - last_pos_);
    } else {
        mouse_move_(crt, crt - last_pos_);
    }

    last_pos_ = crt;
}

void freeglut_application::glut_mouse_(freeglut_application* self, int button, int state, int x, int y) {
    self->mouse_(button, state, x, y);
}

void freeglut_application::glut_mouse_move_(freeglut_application* self, int x, int y) {
    self->motion_(x, y);
}

void freeglut_application::glut_keyboard_(freeglut_application* self, unsigned char key) {
    self->char_(key);
}


} // harmont
