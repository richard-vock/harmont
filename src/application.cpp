#include <application.hpp>

namespace harmont {

application::application(int width, int height, camera::ptr cam, callback_t<camera::ptr> render_callback, callback_t<camera::ptr> reshape_callback) :
    width_(width),
    height_(height),
    cb_render_(std::move(render_callback)),
    cb_reshape_(std::move(reshape_callback)),
    camera_(cam) {
}

application::~application() {
}

void application::init(int argc, char* argv[], std::string title, callback_t<> init_callback) {
    init_(argc, argv, title);
    if (init_callback) init_callback();
}

int application::width() const {
    return width_;
}

int application::height() const {
    return height_;
}

camera::ptr application::current_camera() {
    return camera_;
}

camera::const_ptr application::current_camera() const {
    return camera_;
}

void application::render_() {
    if (cb_render_) cb_render_(camera_);
}

void application::reshape_(int width, int height) {
    width_ = width;
    height_ = height;
    glViewport(0, 0, width, height);
    camera_->reshape(width, height);
    if (cb_reshape_) cb_reshape_(camera_);
}


} // harmont
