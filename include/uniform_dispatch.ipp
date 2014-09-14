template <typename T>
void uniform_dispatch::set(GLenum data_type, GLint size, GLint location, T&& value) {
    set_(data_type, size, location, std::forward<T>(value));
}

template <typename T>
inline require<T, std::is_floating_point> uniform_dispatch::set_(GLenum data_type, GLint, GLint location, T&& value) {
    if (data_type != GL_FLOAT) {
        throw std::runtime_error("shader_program::set(): Invalid data type. Uniform variable is of type " + gl_type_name(data_type) + SPOT);
    }
    float v = static_cast<float>(std::forward<T>(value));
    glUniform1fv(location, 1, &v);
}

template <template <typename, typename> class C, template <typename> class A>
inline void uniform_dispatch::set_(GLenum data_type, GLint size, GLint location, const C<float,A<float>>& seq) {
    if (data_type != GL_FLOAT && data_type != GL_FLOAT_VEC2 && data_type != GL_FLOAT_VEC3 && data_type != GL_FLOAT_VEC4) {
        throw std::runtime_error("shader_program::set(): Invalid data type. Uniform variable is of type " + gl_type_name(data_type) + SPOT);
    }
    GLuint element_size = element_size_(data_type);
    if (seq.size() != element_size * size) {
        throw std::runtime_error("shader_program::set(): Invalid data size. Expected " + std::to_string(size) + " element(s) of size " + std::to_string(element_size) + ". Data has size " + std::to_string(seq.size()) + SPOT);
    }
    switch (data_type) {
        case GL_FLOAT: glUniform1fv(location, size, seq.data()); break;
        case GL_FLOAT_VEC2: glUniform2fv(location, size, seq.data()); break;
        case GL_FLOAT_VEC3: glUniform3fv(location, size, seq.data()); break;
        default: glUniform4fv(location, size, seq.data()); break;
    }
}

template <template <typename, typename> class C, typename T, template <typename> class A>
inline require<T, std::is_floating_point> uniform_dispatch::set_(GLenum data_type, GLint size, GLint location, const C<T,A<T>>& seq) {
    std::vector<float> s(seq.begin(), seq.end());
    set_(data_type, size, location, s);
}

template <int Rows, int Cols, int Options>
inline void uniform_dispatch::set_(GLenum data_type, GLint size, GLint location, const Eigen::Matrix<float, Rows, Cols, Options>& matrix) {
    if (data_type == GL_FLOAT || data_type == GL_FLOAT_VEC2 || data_type == GL_FLOAT_VEC3 || data_type == GL_FLOAT_VEC4) {
        GLuint element_size = element_size_(data_type);
        GLuint size = matrix.rows() * matrix.cols();
        if (size != element_size * size) {
            throw std::runtime_error("shader_program::set(): Invalid data size. Expected matrix of size " + std::to_string(element_size * size) + ". Data has size " + std::to_string(size) + SPOT);
        }
    } else if (is_matrix_type_(data_type)) {
        int r, c;
        std::tie(r, c) = matrix_size_(data_type);
        if (matrix.rows() != r || matrix.cols() != c) {
            throw std::runtime_error("shader_program::set(): Invalid data size. Expected matrix of size (" + std::to_string(r) + "x" + std::to_string(c) + "). Data has size (" + std::to_string(matrix.rows()) + "x" + std::to_string(matrix.cols()) + ")" + SPOT);
        }
    } else {
        throw std::runtime_error("shader_program::set(): Invalid data type. Uniform variable is of type " + gl_type_name(data_type) + SPOT);
    }
    set_matrix_(data_type, size, location, matrix);
}

template <typename T, int Rows, int Cols, int Options>
inline require<T, std::is_floating_point> uniform_dispatch::set_(GLenum data_type, GLint size, GLint location, const Eigen::Matrix<T, Rows, Cols, Options>& matrix) {
    set_(data_type, size, location, matrix.template cast<float>());
}

template <typename T>
inline require<T, std::is_integral, std::is_signed> uniform_dispatch::set_(GLenum data_type, GLint size, GLint location, T&& value) {
    if (data_type != GL_INT) {
        throw std::runtime_error("shader_program::set(): Invalid data type. Uniform variable is of type " + gl_type_name(data_type) + SPOT);
    }
    int v = static_cast<int>(std::forward<T>(value));
    glUniform1iv(location, 1, &v);
}

template <template <typename, typename> class C, template <typename> class A>
inline void uniform_dispatch::set_(GLenum data_type, GLint size, GLint location, const C<int,A<int>>& seq) {
    if (data_type != GL_INT && data_type != GL_INT_VEC2 && data_type != GL_INT_VEC3 && data_type != GL_INT_VEC4) {
        throw std::runtime_error("shader_program::set(): Invalid data type. Uniform variable is of type " + gl_type_name(data_type) + SPOT);
    }
    GLuint element_size = element_size_(data_type);
    if (seq.size() != element_size * size) {
        throw std::runtime_error("shader_program::set(): Invalid data size. Expected " + std::to_string(size) + " element(s) of size " + std::to_string(element_size) + ". Data has size " + std::to_string(seq.size()) + SPOT);
    }
    switch (data_type) {
        case GL_INT: glUniform1iv(location, size, seq.data()); break;
        case GL_INT_VEC2: glUniform2iv(location, size, seq.data()); break;
        case GL_INT_VEC3: glUniform3iv(location, size, seq.data()); break;
        default: glUniform4iv(location, size, seq.data()); break;
    }
}

template <template <typename, typename> class C, typename T, template <typename> class A>
inline require<T, std::is_integral, std::is_signed> uniform_dispatch::set_(GLenum data_type, GLint size, GLint location, const C<T,A<T>>& seq) {
    std::vector<int> s(seq.begin(), seq.end());
    set_(data_type, size, location, s);
}

template <int Rows, int Cols, int Options>
inline void uniform_dispatch::set_(GLenum data_type, GLint size, GLint location, const Eigen::Matrix<int, Rows, Cols, Options>& matrix) {
    if (data_type == GL_INT || data_type == GL_INT_VEC2 || data_type == GL_INT_VEC3 || data_type == GL_INT_VEC4) {
        GLuint element_size = element_size_(data_type);
        GLuint size = matrix.rows() * matrix.cols();
        if (size != element_size * size) {
            throw std::runtime_error("shader_program::set(): Invalid data size. Expected matrix of size " + std::to_string(element_size * size) + ". Data has size " + std::to_string(size) + SPOT);
        }
    } else {
        throw std::runtime_error("shader_program::set(): Invalid data type. Uniform variable is of type " + gl_type_name(data_type) + SPOT);
    }
    set_matrix_(data_type, size, location, matrix);
}

template <typename T, int Rows, int Cols, int Options>
inline require<T, std::is_integral, std::is_signed> uniform_dispatch::set_(GLenum data_type, GLint size, GLint location, const Eigen::Matrix<T, Rows, Cols, Options>& matrix) {
    set_(data_type, size, location, matrix.template cast<int>());
}

template <typename T>
inline require<T, std::is_integral, std::is_unsigned> uniform_dispatch::set_(GLenum data_type, GLint size, GLint location, T&& value) {
    if (data_type != GL_UNSIGNED_INT) {
        throw std::runtime_error("shader_program::set(): Invalid data type. Uniform variable is of type " + gl_type_name(data_type) + SPOT);
    }
    unsigned int v = static_cast<unsigned int>(std::forward<T>(value));
    glUniform1uiv(location, 1, &v);
}

template <template <typename, typename> class C, template <typename> class A>
inline void uniform_dispatch::set_(GLenum data_type, GLint size, GLint location, const C<unsigned int,A<unsigned int>>& seq) {
    if (data_type != GL_UNSIGNED_INT && data_type != GL_UNSIGNED_INT_VEC2 && data_type != GL_UNSIGNED_INT_VEC3 && data_type != GL_UNSIGNED_INT_VEC4) {
        throw std::runtime_error("shader_program::set(): Invalid data type. Uniform variable is of type " + gl_type_name(data_type) + SPOT);
    }
    GLuint element_size = element_size_(data_type);
    if (seq.size() != element_size * size) {
        throw std::runtime_error("shader_program::set(): Invalid data size. Expected " + std::to_string(size) + " element(s) of size " + std::to_string(element_size) + ". Data has size " + std::to_string(seq.size()) + SPOT);
    }
    switch (data_type) {
        case GL_UNSIGNED_INT: glUniform1uiv(location, size, seq.data()); break;
        case GL_UNSIGNED_INT_VEC2: glUniform2uiv(location, size, seq.data()); break;
        case GL_UNSIGNED_INT_VEC3: glUniform3uiv(location, size, seq.data()); break;
        default: glUniform4uiv(location, size, seq.data()); break;
    }
}

template <template <typename, typename> class C, typename T, template <typename> class A>
inline require<T, std::is_integral, std::is_unsigned> uniform_dispatch::set_(GLenum data_type, GLint size, GLint location, const C<T,A<T>>& seq) {
    std::vector<unsigned int> s(seq.begin(), seq.end());
    set_(data_type, size, location, s);
}

template <int Rows, int Cols, int Options>
inline void uniform_dispatch::set_(GLenum data_type, GLint size, GLint location, const Eigen::Matrix<unsigned int, Rows, Cols, Options>& matrix) {
    if (data_type == GL_UNSIGNED_INT || data_type == GL_UNSIGNED_INT_VEC2 || data_type == GL_UNSIGNED_INT_VEC3 || data_type == GL_UNSIGNED_INT_VEC4) {
        GLuint element_size = element_size_(data_type);
        GLuint size = matrix.rows() * matrix.cols();
        if (size != element_size * size) {
            throw std::runtime_error("shader_program::set(): Invalid data size. Expected matrix of size " + std::to_string(element_size * size) + ". Data has size " + std::to_string(size) + SPOT);
        }
    } else {
        throw std::runtime_error("shader_program::set(): Invalid data type. Uniform variable is of type " + gl_type_name(data_type) + SPOT);
    }
    set_matrix_(data_type, size, location, matrix);
}

template <typename T, int Rows, int Cols, int Options>
inline require<T, std::is_integral, std::is_unsigned> uniform_dispatch::set_(GLenum data_type, GLint size, GLint location, const Eigen::Matrix<T, Rows, Cols, Options>& matrix) {
    set_(data_type, size, location, matrix.template cast<unsigned int>());
}

template <typename T, int Rows, int Cols, int Options>
inline void uniform_dispatch::set_matrix_(GLenum data_type, GLint size, GLint location, const Eigen::Matrix<T, Rows, Cols, Options>& matrix) {
    if (Options & Eigen::RowMajor && (
        data_type == GL_FLOAT_MAT2 ||
        data_type == GL_FLOAT_MAT3 ||
        data_type == GL_FLOAT_MAT4 ||
        data_type == GL_FLOAT_MAT2x3 ||
        data_type == GL_FLOAT_MAT2x4 ||
        data_type == GL_FLOAT_MAT3x2 ||
        data_type == GL_FLOAT_MAT3x4 ||
        data_type == GL_FLOAT_MAT4x2 ||
        data_type == GL_FLOAT_MAT4x3)) {
        Eigen::Matrix<T, Rows, Cols, (Options ^ Eigen::RowMajor) | Eigen::ColMajor> new_matrix = matrix;
        switch (data_type) {
            case GL_FLOAT: glUniform1fv(location, size, reinterpret_cast<const float*>(new_matrix.data())); break;
            case GL_FLOAT_VEC2: glUniform2fv(location, size, reinterpret_cast<const float*>(new_matrix.data())); break;
            case GL_FLOAT_VEC3: glUniform3fv(location, size, reinterpret_cast<const float*>(new_matrix.data())); break;
            case GL_FLOAT_VEC4: glUniform4fv(location, size, reinterpret_cast<const float*>(new_matrix.data())); break;
            case GL_INT: glUniform1iv(location, size, reinterpret_cast<const int*>(new_matrix.data())); break;
            case GL_INT_VEC2: glUniform2iv(location, size, reinterpret_cast<const int*>(new_matrix.data())); break;
            case GL_INT_VEC3: glUniform3iv(location, size, reinterpret_cast<const int*>(new_matrix.data())); break;
            case GL_INT_VEC4: glUniform4iv(location, size, reinterpret_cast<const int*>(new_matrix.data())); break;
            case GL_UNSIGNED_INT: glUniform1uiv(location, size, reinterpret_cast<const unsigned int*>(new_matrix.data())); break;
            case GL_UNSIGNED_INT_VEC2: glUniform2uiv(location, size, reinterpret_cast<const unsigned int*>(new_matrix.data())); break;
            case GL_UNSIGNED_INT_VEC3: glUniform3uiv(location, size, reinterpret_cast<const unsigned int*>(new_matrix.data())); break;
            case GL_UNSIGNED_INT_VEC4: glUniform4uiv(location, size, reinterpret_cast<const unsigned int*>(new_matrix.data())); break;
            case GL_FLOAT_MAT2: glUniformMatrix2fv(location, size, GL_FALSE, reinterpret_cast<const float*>(new_matrix.data())); break;
            case GL_FLOAT_MAT3: glUniformMatrix3fv(location, size, GL_FALSE, reinterpret_cast<const float*>(new_matrix.data())); break;
            case GL_FLOAT_MAT4: glUniformMatrix4fv(location, size, GL_FALSE, reinterpret_cast<const float*>(new_matrix.data())); break;
            case GL_FLOAT_MAT2x3: glUniformMatrix2x3fv(location, size, GL_FALSE, reinterpret_cast<const float*>(new_matrix.data())); break;
            case GL_FLOAT_MAT2x4: glUniformMatrix2x4fv(location, size, GL_FALSE, reinterpret_cast<const float*>(new_matrix.data())); break;
            case GL_FLOAT_MAT3x2: glUniformMatrix3x2fv(location, size, GL_FALSE, reinterpret_cast<const float*>(new_matrix.data())); break;
            case GL_FLOAT_MAT3x4: glUniformMatrix3x4fv(location, size, GL_FALSE, reinterpret_cast<const float*>(new_matrix.data())); break;
            case GL_FLOAT_MAT4x2: glUniformMatrix4x2fv(location, size, GL_FALSE, reinterpret_cast<const float*>(new_matrix.data())); break;
            case GL_FLOAT_MAT4x3: glUniformMatrix4x3fv(location, size, GL_FALSE, reinterpret_cast<const float*>(new_matrix.data())); break;
            default: break;
        }
    } else {
        switch (data_type) {
            case GL_FLOAT: glUniform1fv(location, size, reinterpret_cast<const float*>(matrix.data())); break;
            case GL_FLOAT_VEC2: glUniform2fv(location, size, reinterpret_cast<const float*>(matrix.data())); break;
            case GL_FLOAT_VEC3: glUniform3fv(location, size, reinterpret_cast<const float*>(matrix.data())); break;
            case GL_FLOAT_VEC4: glUniform4fv(location, size, reinterpret_cast<const float*>(matrix.data())); break;
            case GL_INT: glUniform1iv(location, size, reinterpret_cast<const int*>(matrix.data())); break;
            case GL_INT_VEC2: glUniform2iv(location, size, reinterpret_cast<const int*>(matrix.data())); break;
            case GL_INT_VEC3: glUniform3iv(location, size, reinterpret_cast<const int*>(matrix.data())); break;
            case GL_INT_VEC4: glUniform4iv(location, size, reinterpret_cast<const int*>(matrix.data())); break;
            case GL_UNSIGNED_INT: glUniform1uiv(location, size, reinterpret_cast<const unsigned int*>(matrix.data())); break;
            case GL_UNSIGNED_INT_VEC2: glUniform2uiv(location, size, reinterpret_cast<const unsigned int*>(matrix.data())); break;
            case GL_UNSIGNED_INT_VEC3: glUniform3uiv(location, size, reinterpret_cast<const unsigned int*>(matrix.data())); break;
            case GL_UNSIGNED_INT_VEC4: glUniform4uiv(location, size, reinterpret_cast<const unsigned int*>(matrix.data())); break;
            case GL_FLOAT_MAT2: glUniformMatrix2fv(location, size, GL_FALSE, reinterpret_cast<const float*>(matrix.data())); break;
            case GL_FLOAT_MAT3: glUniformMatrix3fv(location, size, GL_FALSE, reinterpret_cast<const float*>(matrix.data())); break;
            case GL_FLOAT_MAT4: glUniformMatrix4fv(location, size, GL_FALSE, reinterpret_cast<const float*>(matrix.data())); break;
            case GL_FLOAT_MAT2x3: glUniformMatrix2x3fv(location, size, GL_FALSE, reinterpret_cast<const float*>(matrix.data())); break;
            case GL_FLOAT_MAT2x4: glUniformMatrix2x4fv(location, size, GL_FALSE, reinterpret_cast<const float*>(matrix.data())); break;
            case GL_FLOAT_MAT3x2: glUniformMatrix3x2fv(location, size, GL_FALSE, reinterpret_cast<const float*>(matrix.data())); break;
            case GL_FLOAT_MAT3x4: glUniformMatrix3x4fv(location, size, GL_FALSE, reinterpret_cast<const float*>(matrix.data())); break;
            case GL_FLOAT_MAT4x2: glUniformMatrix4x2fv(location, size, GL_FALSE, reinterpret_cast<const float*>(matrix.data())); break;
            case GL_FLOAT_MAT4x3: glUniformMatrix4x3fv(location, size, GL_FALSE, reinterpret_cast<const float*>(matrix.data())); break;
            default: break;
        }
    }
}

inline GLint uniform_dispatch::element_size_(GLenum data_type) {
    switch (data_type) {
        case GL_FLOAT: return 1; break;
        case GL_FLOAT_VEC2: return 2; break;
        case GL_FLOAT_VEC3: return 3; break;
        case GL_FLOAT_VEC4: return 4; break;
        case GL_INT: return 1; break;
        case GL_INT_VEC2: return 2; break;
        case GL_INT_VEC3: return 3; break;
        case GL_INT_VEC4: return 4; break;
        case GL_UNSIGNED_INT: return 1; break;
        case GL_UNSIGNED_INT_VEC2: return 2; break;
        case GL_UNSIGNED_INT_VEC3: return 3; break;
        case GL_UNSIGNED_INT_VEC4: return 4; break;
        case GL_UNSIGNED_INT_ATOMIC_COUNTER: return 1; break;
        case GL_FLOAT_MAT2: return 4; break;
        case GL_FLOAT_MAT3: return 9; break;
        case GL_FLOAT_MAT4: return 16; break;
        case GL_FLOAT_MAT2x3: return 6; break;
        case GL_FLOAT_MAT2x4: return 8; break;
        case GL_FLOAT_MAT3x2: return 6; break;
        case GL_FLOAT_MAT3x4: return 12; break;
        case GL_FLOAT_MAT4x2: return 8; break;
        case GL_FLOAT_MAT4x3: return 12; break;
        default: break;
    }
    return 0;
}

inline std::pair<GLint, GLint> uniform_dispatch::matrix_size_(GLenum data_type) {
    switch (data_type) {
        case GL_FLOAT_MAT2: return {2, 2}; break;
        case GL_FLOAT_MAT3: return {3, 3}; break;
        case GL_FLOAT_MAT4: return {4, 4}; break;
        case GL_FLOAT_MAT2x3: return {2, 3}; break;
        case GL_FLOAT_MAT2x4: return {2, 4}; break;
        case GL_FLOAT_MAT3x2: return {3, 2}; break;
        case GL_FLOAT_MAT3x4: return {3, 4}; break;
        case GL_FLOAT_MAT4x2: return {4, 2}; break;
        case GL_FLOAT_MAT4x3: return {4, 3}; break;
        default: break;
    }
    return {0, 0};
}

inline bool uniform_dispatch::is_matrix_type_(GLenum data_type) {
    int r, c;
    std::tie(r, c) = matrix_size_(data_type);
    return r > 0 && c > 0;
}
