template <class CameraModel>
inline freeglut_application::ptr freeglut_application::create(int width, int height, callback_t<camera::ptr> render_callback, callback_t<camera::ptr> reshape_callback) {
    return ptr(new freeglut_application(width, height, default_camera_<CameraModel>(width, height), render_callback, reshape_callback));
}
