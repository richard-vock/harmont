#include <application.hpp>

namespace harmont {

application::application(int width, int height, camera::ptr cam, callback_t<camera::ptr> render_callback, callback_t<camera::ptr> reshape_callback) :
    width_(width),
    height_(height),
    camera_(cam),
    cb_render_(std::move(render_callback)),
    cb_reshape_(std::move(reshape_callback)) {
}

application::~application() {
}

void application::init(int argc, char* argv[], std::string title, callback_t<> init_callback) {
    init_(argc, argv, title);
    if (init_callback) init_callback();
    reshape_(width_, height_);
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

void application::on_mouse_move(callback_t<screen_pos_t, screen_pos_t> callback) {
    cb_mouse_move_ = std::move(callback);
}

void application::on_click_left(callback_t<screen_pos_t> callback) {
    cb_click_left_ = std::move(callback);
}

void application::on_click_right(callback_t<screen_pos_t> callback) {
    cb_click_right_ = std::move(callback);
}

void application::on_click_middle(callback_t<screen_pos_t> callback) {
    cb_click_middle_ = std::move(callback);
}

void application::on_drag_start_left(callback_t<screen_pos_t> callback) {
    cb_drag_start_left_ = std::move(callback);
}

void application::on_drag_start_right(callback_t<screen_pos_t> callback) {
    cb_drag_start_right_ = std::move(callback);
}

void application::on_drag_start_middle(callback_t<screen_pos_t> callback) {
    cb_drag_start_middle_ = std::move(callback);
}

void application::on_drag_stop_left(callback_t<screen_pos_t, screen_pos_t> callback) {
    cb_drag_stop_left_ = std::move(callback);
}

void application::on_drag_stop_right(callback_t<screen_pos_t, screen_pos_t> callback) {
    cb_drag_stop_right_ = std::move(callback);
}

void application::on_drag_stop_middle(callback_t<screen_pos_t, screen_pos_t> callback) {
    cb_drag_stop_middle_ = std::move(callback);
}

void application::on_drag_left(callback_t<screen_pos_t, screen_pos_t> callback) {
    cb_drag_left_ = std::move(callback);
}

void application::on_drag_right(callback_t<screen_pos_t, screen_pos_t> callback) {
    cb_drag_right_ = std::move(callback);
}

void application::on_drag_middle(callback_t<screen_pos_t, screen_pos_t> callback) {
    cb_drag_middle_ = std::move(callback);
}

void application::on_scroll(callback_t<int> callback) {
    cb_scroll_ = std::move(callback);
}

void application::on_char(callback_t<unsigned char> callback) {
    cb_char_ = std::move(callback);
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

void application::mouse_move_(screen_pos_t pos, screen_pos_t delta) {
    if (cb_mouse_move_) cb_mouse_move_(pos, delta);
}

void application::click_left_(screen_pos_t pos) {
    if (cb_click_left_) cb_click_left_(pos);
}

void application::click_right_(screen_pos_t pos) {
    if (cb_click_right_) cb_click_right_(pos);
}

void application::click_middle_(screen_pos_t pos) {
    if (cb_click_middle_) cb_click_middle_(pos);
}

void application::drag_start_left_(screen_pos_t pos) {
    if (cb_drag_start_left_) cb_drag_start_left_(pos);
}

void application::drag_start_right_(screen_pos_t pos) {
    if (cb_drag_start_right_) cb_drag_start_right_(pos);
}

void application::drag_start_middle_(screen_pos_t pos) {
    if (cb_drag_start_middle_) cb_drag_start_middle_(pos);
}

void application::drag_stop_left_(screen_pos_t start, screen_pos_t pos) {
    if (cb_drag_stop_left_) cb_drag_stop_left_(start, pos);
}

void application::drag_stop_right_(screen_pos_t start, screen_pos_t pos) {
    if (cb_drag_stop_right_) cb_drag_stop_right_(start, pos);
}

void application::drag_stop_middle_(screen_pos_t start, screen_pos_t pos) {
    if (cb_drag_stop_middle_) cb_drag_stop_middle_(start, pos);
}

void application::drag_left_(screen_pos_t pos, screen_pos_t delta) {
    if (cb_drag_left_) cb_drag_left_(pos, delta);
}

void application::drag_right_(screen_pos_t pos, screen_pos_t delta) {
    camera_->update(camera::vec3_t::Zero(), 0.01f * camera::vec3_t(delta[0], delta[1], 0.0));
    if (cb_drag_right_) cb_drag_right_(pos, delta);
    update();
}

void application::drag_middle_(screen_pos_t pos, screen_pos_t delta) {
    camera_->update(0.01f * camera::vec3_t(-delta[0], -delta[1], 0.0), camera::vec3_t::Zero());
    if (cb_drag_middle_) cb_drag_middle_(pos, delta);
    update();
}

void application::scroll_(int delta) {
    camera_->update(-1.f * camera::vec3_t(0.0, 0.0, delta), camera::vec3_t::Zero());
    if (cb_scroll_) cb_scroll_(delta);
    update();
}

void application::char_(unsigned char key) {
    if (cb_char_) cb_char_(key);
}


} // harmont
