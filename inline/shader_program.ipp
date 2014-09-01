template <typename T>
inline void shader_program::variable_t::set(T&& value) const {
    if (var_type_ == ATTRIBUTE) {
        throw std::runtime_error("shader_program::variable_t::set(): Vertex attribute variable values cannot be set. Use vertex buffer objects instead"+SPOT);
    }
    switch (data_type_) {
        case GL_FLOAT: uniform_dispatch<GL_FLOAT>::set(std::forward<T>(value), location_); break;
        case GL_FLOAT_VEC2: uniform_dispatch<GL_FLOAT_VEC2>::set(std::forward<T>(value), location_); break;
        case GL_FLOAT_VEC3: uniform_dispatch<GL_FLOAT_VEC3>::set(std::forward<T>(value), location_); break;
        case GL_FLOAT_VEC4: uniform_dispatch<GL_FLOAT_VEC4>::set(std::forward<T>(value), location_); break;
        case GL_INT: uniform_dispatch<GL_INT>::set(std::forward<T>(value), location_); break;
        case GL_INT_VEC2: uniform_dispatch<GL_INT_VEC2>::set(std::forward<T>(value), location_); break;
        case GL_INT_VEC3: uniform_dispatch<GL_INT_VEC3>::set(std::forward<T>(value), location_); break;
        case GL_INT_VEC4: uniform_dispatch<GL_INT_VEC4>::set(std::forward<T>(value), location_); break;
        case GL_UNSIGNED_INT: uniform_dispatch<GL_UNSIGNED_INT>::set(std::forward<T>(value), location_); break;
        case GL_UNSIGNED_INT_VEC2: uniform_dispatch<GL_UNSIGNED_INT_VEC2>::set(std::forward<T>(value), location_); break;
        case GL_UNSIGNED_INT_VEC3: uniform_dispatch<GL_UNSIGNED_INT_VEC3>::set(std::forward<T>(value), location_); break;
        case GL_UNSIGNED_INT_VEC4: uniform_dispatch<GL_UNSIGNED_INT_VEC4>::set(std::forward<T>(value), location_); break;
        case GL_UNSIGNED_INT_ATOMIC_COUNTER: uniform_dispatch<GL_UNSIGNED_INT_ATOMIC_COUNTER>::set(std::forward<T>(value), location_); break;
        case GL_FLOAT_MAT2: uniform_dispatch<GL_FLOAT_MAT2>::set(std::forward<T>(value), location_); break;
        case GL_FLOAT_MAT3: uniform_dispatch<GL_FLOAT_MAT3>::set(std::forward<T>(value), location_); break;
        case GL_FLOAT_MAT4: uniform_dispatch<GL_FLOAT_MAT4>::set(std::forward<T>(value), location_); break;
        case GL_FLOAT_MAT2x3: uniform_dispatch<GL_FLOAT_MAT2x3>::set(std::forward<T>(value), location_); break;
        case GL_FLOAT_MAT2x4: uniform_dispatch<GL_FLOAT_MAT2x4>::set(std::forward<T>(value), location_); break;
        case GL_FLOAT_MAT3x2: uniform_dispatch<GL_FLOAT_MAT3x2>::set(std::forward<T>(value), location_); break;
        case GL_FLOAT_MAT3x4: uniform_dispatch<GL_FLOAT_MAT3x4>::set(std::forward<T>(value), location_); break;
        case GL_FLOAT_MAT4x2: uniform_dispatch<GL_FLOAT_MAT4x2>::set(std::forward<T>(value), location_); break;
        case GL_FLOAT_MAT4x3: uniform_dispatch<GL_FLOAT_MAT4x3>::set(std::forward<T>(value), location_); break;
        default: break;
    }
}

template <typename T>
inline void shader_program::variable_t::operator=(T&& value) const {
    set(std::forward<T>(value));
}
