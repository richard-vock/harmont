template <class RNG>
inline Eigen::Vector2f shadow_pass::gen_point_(RNG& rng, float radius, float offset, const Eigen::Vector2f& base) {
    float r = rng() * radius + offset;
    float a = rng() * 2.f * static_cast<float>(M_PI);
    return base + r * Eigen::Vector2f(cos(a), sin(a));
}
