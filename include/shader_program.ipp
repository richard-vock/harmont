template <typename T>
inline void shader_program::variable_t::set(T&& value) const {
    if (var_type_ == ATTRIBUTE) {
        throw std::runtime_error("shader_program::variable_t::set(): Vertex attribute variable values cannot be set. Use vertex buffer objects instead"+SPOT);
    }

    uniform_dispatch::set(data_type_, size_, location_, std::forward<T>(value));
}

template <typename T>
inline void shader_program::variable_t::operator=(T&& value) const {
    set(std::forward<T>(value));
}
