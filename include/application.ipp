template <class CameraModel>
inline camera::ptr application::default_camera_(int width, int height) {
    auto model = camera_model::looking_at<CameraModel>(camera_model::vec3_t(0.0, -20.0, 0.0));
    return std::make_shared<camera>(model, width, height, 40.0, 0.01, 200.0);
}
