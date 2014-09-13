template <class Derived>
inline camera_model::ptr camera_model::looking_at(const vec3_t& position, const vec3_t& look_at) {
    ptr model(new Derived());
    vec3_t up = vec3_t::UnitZ();
    if (fabs(up.dot((look_at - position).normalized())) > 0.99) {
        up = vec3_t::UnitX();
    }
    model->set_parameters(position, look_at, up);
    return model;
}

template <class Derived>
inline camera_model::ptr camera_model::looking_towards(const vec3_t& position, const vec3_t& forward) {
    return looking_at<Derived>(position, position + forward.normalized());
}
