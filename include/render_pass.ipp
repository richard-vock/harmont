template <typename T>
inline void render_pass::set_uniform(const std::string& name, T&& value) {
    bool release = !(program_->bound());
    program_->bind();

    ((*this)[name]).set(std::forward<T>(value));

    if (release) {
        program_->release();
    }
}
