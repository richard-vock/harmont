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


} // harmont
