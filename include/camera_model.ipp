template <class Derived>
inline camera_model::ptr camera_model::looking_at(const vec3_t& position, const vec3_t& look_at) {
    return Derived::from_looking_at(position, look_at);
}

template <class Derived>
inline camera_model::ptr camera_model::looking_towards(const vec3_t& position, const vec3_t& forward) {
    return looking_at<Derived>(position, position + forward.normalized());
}
