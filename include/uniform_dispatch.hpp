#ifndef UNIFORM_DISPATCH_HPP_
#define UNIFORM_DISPATCH_HPP_

#include "common.hpp"

namespace Eigen {
typedef Eigen::Matrix<unsigned int, 2, 1> Vector2ui;
typedef Eigen::Matrix<unsigned int, 3, 1> Vector3ui;
typedef Eigen::Matrix<unsigned int, 4, 1> Vector4ui;
} // Eigen

namespace harmont {


template <int V>
struct uniform_dispatch;

template <>
struct uniform_dispatch<GL_FLOAT> {
    template <typename T>
    static void set(T value, GLint location) {
        float v = static_cast<float>(value);
        glUniform1fv(location, 1, &v);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<float, A<float>>& values, GLint location) {
        glUniform1fv(location, values.size(), values.data());
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<T, A<T>>& values, GLint location) {
        std::vector<float> v(values.begin(), values.end());
        set(v, location);
    }
};

template <>
struct uniform_dispatch<GL_FLOAT_VEC2> {
    static constexpr int dim = 2;
    template <typename T>
    static void set(const Eigen::Matrix<T, dim, 1>& value, GLint location) {
        Eigen::Vector2f v = value.template cast<float>(value);
        glUniform2fv(location, dim, v.data());
    }
    template <typename T>
    static void set(const Eigen::Matrix<T, 1, dim>& value, GLint location) {
        Eigen::Vector2f v = value.template cast<float>(value);
        glUniform2fv(location, dim, v.data());
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<float, A<float>>& values, GLint location) {
        if (values.size() % dim != 0) {
            throw std::runtime_error("uniform_dispatch::set(): Value count must be multiple of 2" + SPOT);
        }
        glUniform2fv(location, values.size(), values.data());
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<T, A<T>>& values, GLint location) {
        std::vector<float> v(values.begin(), values.end());
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Vector2f, A<Eigen::Vector2f>>& values, GLint location) {
        std::vector<float> v(dim*values.size());
        for (uint32_t i = 0; i < values.size(); ++i) {
            for (uint32_t j = 0; j < dim; ++j) {
                v[dim*i+j] = values[i][j];
            }
        }
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Matrix<T, dim, 1>, A<Eigen::Matrix<T, dim, 1>>>& values, GLint location) {
        std::vector<Eigen::Vector2f> v(values.begin(), values.end());
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Matrix<T, 1, dim>, A<Eigen::Matrix<T, 1, dim>>>& values, GLint location) {
        std::vector<Eigen::Vector2f> v(values.begin(), values.end());
        set(v, location);
    }
};

template <>
struct uniform_dispatch<GL_FLOAT_VEC3> {
    static constexpr int dim = 3;
    template <typename T>
    static void set(const Eigen::Matrix<T, dim, 1>& value, GLint location) {
        Eigen::Vector2f v = value.template cast<float>(value);
        glUniform3fv(location, dim, v.data());
    }
    template <typename T>
    static void set(const Eigen::Matrix<T, 1, dim>& value, GLint location) {
        Eigen::Vector2f v = value.template cast<float>(value);
        glUniform3fv(location, dim, v.data());
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<float, A<float>>& values, GLint location) {
        if (values.size() % dim != 0) {
            throw std::runtime_error("uniform_dispatch::set(): Value count must be multiple of 2" + SPOT);
        }
        glUniform3fv(location, values.size(), values.data());
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<T, A<T>>& values, GLint location) {
        std::vector<float> v(values.begin(), values.end());
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Vector2f, A<Eigen::Vector2f>>& values, GLint location) {
        std::vector<float> v(dim*values.size());
        for (uint32_t i = 0; i < values.size(); ++i) {
            for (uint32_t j = 0; j < dim; ++j) {
                v[dim*i+j] = values[i][j];
            }
        }
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Matrix<T, dim, 1>, A<Eigen::Matrix<T, dim, 1>>>& values, GLint location) {
        std::vector<Eigen::Vector2f> v(values.begin(), values.end());
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Matrix<T, 1, dim>, A<Eigen::Matrix<T, 1, dim>>>& values, GLint location) {
        std::vector<Eigen::Vector2f> v(values.begin(), values.end());
        set(v, location);
    }
};

template <>
struct uniform_dispatch<GL_FLOAT_VEC4> {
    static constexpr int dim = 4;
    template <typename T>
    static void set(const Eigen::Matrix<T, dim, 1>& value, GLint location) {
        Eigen::Vector2f v = value.template cast<float>(value);
        glUniform4fv(location, dim, v.data());
    }
    template <typename T>
    static void set(const Eigen::Matrix<T, 1, dim>& value, GLint location) {
        Eigen::Vector2f v = value.template cast<float>(value);
        glUniform4fv(location, dim, v.data());
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<float, A<float>>& values, GLint location) {
        if (values.size() % dim != 0) {
            throw std::runtime_error("uniform_dispatch::set(): Value count must be multiple of 2" + SPOT);
        }
        glUniform4fv(location, values.size(), values.data());
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<T, A<T>>& values, GLint location) {
        std::vector<float> v(values.begin(), values.end());
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Vector2f, A<Eigen::Vector2f>>& values, GLint location) {
        std::vector<float> v(dim*values.size());
        for (uint32_t i = 0; i < values.size(); ++i) {
            for (uint32_t j = 0; j < dim; ++j) {
                v[dim*i+j] = values[i][j];
            }
        }
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Matrix<T, dim, 1>, A<Eigen::Matrix<T, dim, 1>>>& values, GLint location) {
        std::vector<Eigen::Vector2f> v(values.begin(), values.end());
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Matrix<T, 1, dim>, A<Eigen::Matrix<T, 1, dim>>>& values, GLint location) {
        std::vector<Eigen::Vector2f> v(values.begin(), values.end());
        set(v, location);
    }
};

template <>
struct uniform_dispatch<GL_INT> {
    template <typename T>
    static void set(T value, GLint location) {
        int v = static_cast<int>(value);
        glUniform1iv(location, 1, &v);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<int, A<int>>& values, GLint location) {
        glUniform1iv(location, values.size(), values.data());
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<T, A<T>>& values, GLint location) {
        std::vector<int> v(values.begin(), values.end());
        set(v, location);
    }
};

template <>
struct uniform_dispatch<GL_INT_VEC2> {
    static constexpr int dim = 2;
    template <typename T>
    static void set(const Eigen::Matrix<T, dim, 1>& value, GLint location) {
        Eigen::Vector2i v = value.template cast<int>(value);
        glUniform2iv(location, dim, v.data());
    }
    template <typename T>
    static void set(const Eigen::Matrix<T, 1, dim>& value, GLint location) {
        Eigen::Vector2i v = value.template cast<int>(value);
        glUniform2iv(location, dim, v.data());
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<int, A<int>>& values, GLint location) {
        if (values.size() % dim != 0) {
            throw std::runtime_error("uniform_dispatch::set(): Value count must be multiple of 2" + SPOT);
        }
        glUniform2iv(location, values.size(), values.data());
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<T, A<T>>& values, GLint location) {
        std::vector<int> v(values.begin(), values.end());
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Vector2i, A<Eigen::Vector2i>>& values, GLint location) {
        std::vector<int> v(dim*values.size());
        for (uint32_t i = 0; i < values.size(); ++i) {
            for (uint32_t j = 0; j < dim; ++j) {
                v[dim*i+j] = values[i][j];
            }
        }
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Matrix<T, dim, 1>, A<Eigen::Matrix<T, dim, 1>>>& values, GLint location) {
        std::vector<Eigen::Vector2i> v(values.begin(), values.end());
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Matrix<T, 1, dim>, A<Eigen::Matrix<T, 1, dim>>>& values, GLint location) {
        std::vector<Eigen::Vector2i> v(values.begin(), values.end());
        set(v, location);
    }
};

template <>
struct uniform_dispatch<GL_INT_VEC3> {
    static constexpr int dim = 3;
    template <typename T>
    static void set(const Eigen::Matrix<T, dim, 1>& value, GLint location) {
        Eigen::Vector3i v = value.template cast<int>(value);
        glUniform3iv(location, dim, v.data());
    }
    template <typename T>
    static void set(const Eigen::Matrix<T, 1, dim>& value, GLint location) {
        Eigen::Vector3i v = value.template cast<int>(value);
        glUniform3iv(location, dim, v.data());
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<int, A<int>>& values, GLint location) {
        if (values.size() % dim != 0) {
            throw std::runtime_error("uniform_dispatch::set(): Value count must be multiple of 3" + SPOT);
        }
        glUniform3iv(location, values.size(), values.data());
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<T, A<T>>& values, GLint location) {
        std::vector<int> v(values.begin(), values.end());
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Vector3i, A<Eigen::Vector3i>>& values, GLint location) {
        std::vector<int> v(dim*values.size());
        for (uint32_t i = 0; i < values.size(); ++i) {
            for (uint32_t j = 0; j < dim; ++j) {
                v[dim*i+j] = values[i][j];
            }
        }
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Matrix<T, dim, 1>, A<Eigen::Matrix<T, dim, 1>>>& values, GLint location) {
        std::vector<Eigen::Vector3i> v(values.begin(), values.end());
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Matrix<T, 1, dim>, A<Eigen::Matrix<T, 1, dim>>>& values, GLint location) {
        std::vector<Eigen::Vector3i> v(values.begin(), values.end());
        set(v, location);
    }
};

template <>
struct uniform_dispatch<GL_INT_VEC4> {
    static constexpr int dim = 4;
    template <typename T>
    static void set(const Eigen::Matrix<T, dim, 1>& value, GLint location) {
        Eigen::Vector4i v = value.template cast<int>(value);
        glUniform4iv(location, dim, v.data());
    }
    template <typename T>
    static void set(const Eigen::Matrix<T, 1, dim>& value, GLint location) {
        Eigen::Vector4i v = value.template cast<int>(value);
        glUniform4iv(location, dim, v.data());
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<int, A<int>>& values, GLint location) {
        if (values.size() % dim != 0) {
            throw std::runtime_error("uniform_dispatch::set(): Value count must be multiple of 4" + SPOT);
        }
        glUniform4iv(location, values.size(), values.data());
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<T, A<T>>& values, GLint location) {
        std::vector<int> v(values.begin(), values.end());
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Vector4i, A<Eigen::Vector4i>>& values, GLint location) {
        std::vector<int> v(dim*values.size());
        for (uint32_t i = 0; i < values.size(); ++i) {
            for (uint32_t j = 0; j < dim; ++j) {
                v[dim*i+j] = values[i][j];
            }
        }
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Matrix<T, dim, 1>, A<Eigen::Matrix<T, dim, 1>>>& values, GLint location) {
        std::vector<Eigen::Vector4i> v(values.begin(), values.end());
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Matrix<T, 1, dim>, A<Eigen::Matrix<T, 1, dim>>>& values, GLint location) {
        std::vector<Eigen::Vector4i> v(values.begin(), values.end());
        set(v, location);
    }
};

template <>
struct uniform_dispatch<GL_UNSIGNED_INT> {
    template <typename T>
    static void set(T value, GLint location) {
        unsigned int v = static_cast<unsigned int>(value);
        glUniform1uiv(location, 1, &v);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<unsigned int, A<unsigned int>>& values, GLint location) {
        glUniform1uiv(location, values.size(), values.data());
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<T, A<T>>& values, GLint location) {
        std::vector<unsigned int> v(values.begin(), values.end());
        set(v, location);
    }
};

template <>
struct uniform_dispatch<GL_UNSIGNED_INT_VEC2> {
    static constexpr int dim = 2;
    template <typename T>
    static void set(const Eigen::Matrix<T, dim, 1>& value, GLint location) {
        Eigen::Vector2ui v = value.template cast<unsigned int>(value);
        glUniform2uiv(location, dim, v.data());
    }
    template <typename T>
    static void set(const Eigen::Matrix<T, 1, dim>& value, GLint location) {
        Eigen::Vector2ui v = value.template cast<unsigned int>(value);
        glUniform2uiv(location, dim, v.data());
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<unsigned int, A<unsigned int>>& values, GLint location) {
        if (values.size() % dim != 0) {
            throw std::runtime_error("uniform_dispatch::set(): Value count must be multiple of 2" + SPOT);
        }
        glUniform2uiv(location, values.size(), values.data());
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<T, A<T>>& values, GLint location) {
        std::vector<unsigned int> v(values.begin(), values.end());
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Vector2ui, A<Eigen::Vector2ui>>& values, GLint location) {
        std::vector<unsigned int> v(dim*values.size());
        for (uint32_t i = 0; i < values.size(); ++i) {
            for (uint32_t j = 0; j < dim; ++j) {
                v[dim*i+j] = values[i][j];
            }
        }
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Matrix<T, dim, 1>, A<Eigen::Matrix<T, dim, 1>>>& values, GLint location) {
        std::vector<Eigen::Vector2ui> v(values.begin(), values.end());
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Matrix<T, 1, dim>, A<Eigen::Matrix<T, 1, dim>>>& values, GLint location) {
        std::vector<Eigen::Vector2ui> v(values.begin(), values.end());
        set(v, location);
    }
};

template <>
struct uniform_dispatch<GL_UNSIGNED_INT_VEC3> {
    static constexpr int dim = 3;
    template <typename T>
    static void set(const Eigen::Matrix<T, dim, 1>& value, GLint location) {
        Eigen::Vector3ui v = value.template cast<unsigned int>(value);
        glUniform3uiv(location, dim, v.data());
    }
    template <typename T>
    static void set(const Eigen::Matrix<T, 1, dim>& value, GLint location) {
        Eigen::Vector3ui v = value.template cast<unsigned int>(value);
        glUniform3uiv(location, dim, v.data());
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<unsigned int, A<unsigned int>>& values, GLint location) {
        if (values.size() % dim != 0) {
            throw std::runtime_error("uniform_dispatch::set(): Value count must be multiple of 3" + SPOT);
        }
        glUniform3uiv(location, values.size(), values.data());
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<T, A<T>>& values, GLint location) {
        std::vector<unsigned int> v(values.begin(), values.end());
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Vector3ui, A<Eigen::Vector3ui>>& values, GLint location) {
        std::vector<unsigned int> v(dim*values.size());
        for (uint32_t i = 0; i < values.size(); ++i) {
            for (uint32_t j = 0; j < dim; ++j) {
                v[dim*i+j] = values[i][j];
            }
        }
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Matrix<T, dim, 1>, A<Eigen::Matrix<T, dim, 1>>>& values, GLint location) {
        std::vector<Eigen::Vector3ui> v(values.begin(), values.end());
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Matrix<T, 1, dim>, A<Eigen::Matrix<T, 1, dim>>>& values, GLint location) {
        std::vector<Eigen::Vector3ui> v(values.begin(), values.end());
        set(v, location);
    }
};

template <>
struct uniform_dispatch<GL_UNSIGNED_INT_VEC4> {
    static constexpr int dim = 4;
    template <typename T>
    static void set(const Eigen::Matrix<T, dim, 1>& value, GLint location) {
        Eigen::Vector4ui v = value.template cast<unsigned int>(value);
        glUniform4uiv(location, dim, v.data());
    }
    template <typename T>
    static void set(const Eigen::Matrix<T, 1, dim>& value, GLint location) {
        Eigen::Vector4ui v = value.template cast<unsigned int>(value);
        glUniform4uiv(location, dim, v.data());
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<unsigned int, A<unsigned int>>& values, GLint location) {
        if (values.size() % dim != 0) {
            throw std::runtime_error("uniform_dispatch::set(): Value count must be multiple of 4" + SPOT);
        }
        glUniform4uiv(location, values.size(), values.data());
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<T, A<T>>& values, GLint location) {
        std::vector<unsigned int> v(values.begin(), values.end());
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Vector4ui, A<Eigen::Vector4ui>>& values, GLint location) {
        std::vector<unsigned int> v(dim*values.size());
        for (uint32_t i = 0; i < values.size(); ++i) {
            for (uint32_t j = 0; j < dim; ++j) {
                v[dim*i+j] = values[i][j];
            }
        }
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Matrix<T, dim, 1>, A<Eigen::Matrix<T, dim, 1>>>& values, GLint location) {
        std::vector<Eigen::Vector4ui> v(values.begin(), values.end());
        set(v, location);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<Eigen::Matrix<T, 1, dim>, A<Eigen::Matrix<T, 1, dim>>>& values, GLint location) {
        std::vector<Eigen::Vector4ui> v(values.begin(), values.end());
        set(v, location);
    }
};

template <>
struct uniform_dispatch<GL_UNSIGNED_INT_ATOMIC_COUNTER> {
    template <typename T>
    static void set(T value, GLint location) {
        unsigned int v = static_cast<unsigned int>(value);
        glUniform1uiv(location, 1, &v);
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<unsigned int, A<unsigned int>>& values, GLint location) {
        glUniform1uiv(location, values.size(), values.data());
    }
    template <template <typename, typename> class C, typename T, template <typename> class A>
    static void set(const C<T, A<T>>& values, GLint location) {
        std::vector<unsigned int> v(values.begin(), values.end());
        set(v, location);
    }
};

template <>
struct uniform_dispatch<GL_FLOAT_MAT2> {
    static void set(const float* values, GLint location) {
        glUniformMatrix2fv(location, 4, GL_FALSE, values);
    }
    template <typename T, int Rows, int Cols, int Options>
    static void set(const Eigen::Matrix<T, Rows, Cols, Options>& matrix, GLint location) {
        if (matrix.rows() != 2 || matrix.cols() != 2) {
            throw std::runtime_error("uniform_dispatch::set(): Matrix dimensions used in variable set do not match. Must be a 2x2 matrix"+SPOT);
        }
        if (Options & Eigen::ColMajor) {
            set(matrix.template cast<float>().data(), location);
            return;
        }
        Eigen::Matrix<float, Rows, Cols, (Options ^ Eigen::RowMajor) | Eigen::ColMajor> copy = matrix.template cast<float>();
        set(copy.data(), location);
    }
};

template <>
struct uniform_dispatch<GL_FLOAT_MAT3> {
    static void set(const float* values, GLint location) {
        glUniformMatrix3fv(location, 9, GL_FALSE, values);
    }
    template <typename T, int Rows, int Cols, int Options>
    static void set(const Eigen::Matrix<T, Rows, Cols, Options>& matrix, GLint location) {
        if (matrix.rows() != 3 || matrix.cols() != 3) {
            throw std::runtime_error("uniform_dispatch::set(): Matrix dimensions used in variable set do not match. Must be a 3x3 matrix"+SPOT);
        }
        if (Options & Eigen::ColMajor) {
            set(matrix.template cast<float>().data(), location);
            return;
        }
        Eigen::Matrix<float, Rows, Cols, (Options ^ Eigen::RowMajor) | Eigen::ColMajor> copy = matrix.template cast<float>();
        set(copy.data(), location);
    }
};

template <>
struct uniform_dispatch<GL_FLOAT_MAT4> {
    static void set(const float* values, GLint location) {
        glUniformMatrix4fv(location, 16, GL_FALSE, values);
    }
    template <typename T, int Rows, int Cols, int Options>
    static void set(const Eigen::Matrix<T, Rows, Cols, Options>& matrix, GLint location) {
        if (matrix.rows() != 4 || matrix.cols() != 4) {
            throw std::runtime_error("uniform_dispatch::set(): Matrix dimensions used in variable set do not match. Must be a 4x4 matrix"+SPOT);
        }
        if (Options & Eigen::ColMajor) {
            set(matrix.template cast<float>().data(), location);
            return;
        }
        Eigen::Matrix<float, Rows, Cols, (Options ^ Eigen::RowMajor) | Eigen::ColMajor> copy = matrix.template cast<float>();
        set(copy.data(), location);
    }
};

template <>
struct uniform_dispatch<GL_FLOAT_MAT2x3> {
    static void set(const float* values, GLint location) {
        glUniformMatrix2x3fv(location, 6, GL_FALSE, values);
    }
    template <typename T, int Rows, int Cols, int Options>
    static void set(const Eigen::Matrix<T, Rows, Cols, Options>& matrix, GLint location) {
        if (matrix.rows() != 2 || matrix.cols() != 3) {
            throw std::runtime_error("uniform_dispatch::set(): Matrix dimensions used in variable set do not match. Must be a 2x3 matrix"+SPOT);
        }
        if (Options & Eigen::ColMajor) {
            set(matrix.template cast<float>().data(), location);
            return;
        }
        Eigen::Matrix<float, Rows, Cols, (Options ^ Eigen::RowMajor) | Eigen::ColMajor> copy = matrix.template cast<float>();
        set(copy.data(), location);
    }
};

template <>
struct uniform_dispatch<GL_FLOAT_MAT2x4> {
    static void set(const float* values, GLint location) {
        glUniformMatrix2x4fv(location, 8, GL_FALSE, values);
    }
    template <typename T, int Rows, int Cols, int Options>
    static void set(const Eigen::Matrix<T, Rows, Cols, Options>& matrix, GLint location) {
        if (matrix.rows() != 2 || matrix.cols() != 4) {
            throw std::runtime_error("uniform_dispatch::set(): Matrix dimensions used in variable set do not match. Must be a 2x4 matrix"+SPOT);
        }
        if (Options & Eigen::ColMajor) {
            set(matrix.template cast<float>().data(), location);
            return;
        }
        Eigen::Matrix<float, Rows, Cols, (Options ^ Eigen::RowMajor) | Eigen::ColMajor> copy = matrix.template cast<float>();
        set(copy.data(), location);
    }
};

template <>
struct uniform_dispatch<GL_FLOAT_MAT3x2> {
    static void set(const float* values, GLint location) {
        glUniformMatrix3x2fv(location, 6, GL_FALSE, values);
    }
    template <typename T, int Rows, int Cols, int Options>
    static void set(const Eigen::Matrix<T, Rows, Cols, Options>& matrix, GLint location) {
        if (matrix.rows() != 3 || matrix.cols() != 2) {
            throw std::runtime_error("uniform_dispatch::set(): Matrix dimensions used in variable set do not match. Must be a 3x2 matrix"+SPOT);
        }
        if (Options & Eigen::ColMajor) {
            set(matrix.template cast<float>().data(), location);
            return;
        }
        Eigen::Matrix<float, Rows, Cols, (Options ^ Eigen::RowMajor) | Eigen::ColMajor> copy = matrix.template cast<float>();
        set(copy.data(), location);
    }
};

template <>
struct uniform_dispatch<GL_FLOAT_MAT3x4> {
    static void set(const float* values, GLint location) {
        glUniformMatrix3x4fv(location, 12, GL_FALSE, values);
    }
    template <typename T, int Rows, int Cols, int Options>
    static void set(const Eigen::Matrix<T, Rows, Cols, Options>& matrix, GLint location) {
        if (matrix.rows() != 3 || matrix.cols() != 4) {
            throw std::runtime_error("uniform_dispatch::set(): Matrix dimensions used in variable set do not match. Must be a 3x4 matrix"+SPOT);
        }
        if (Options & Eigen::ColMajor) {
            set(matrix.template cast<float>().data(), location);
            return;
        }
        Eigen::Matrix<float, Rows, Cols, (Options ^ Eigen::RowMajor) | Eigen::ColMajor> copy = matrix.template cast<float>();
        set(copy.data(), location);
    }
};

template <>
struct uniform_dispatch<GL_FLOAT_MAT4x2> {
    static void set(const float* values, GLint location) {
        glUniformMatrix4x2fv(location, 8, GL_FALSE, values);
    }
    template <typename T, int Rows, int Cols, int Options>
    static void set(const Eigen::Matrix<T, Rows, Cols, Options>& matrix, GLint location) {
        if (matrix.rows() != 4 || matrix.cols() != 2) {
            throw std::runtime_error("uniform_dispatch::set(): Matrix dimensions used in variable set do not match. Must be a 4x2 matrix"+SPOT);
        }
        if (Options & Eigen::ColMajor) {
            set(matrix.template cast<float>().data(), location);
            return;
        }
        Eigen::Matrix<float, Rows, Cols, (Options ^ Eigen::RowMajor) | Eigen::ColMajor> copy = matrix.template cast<float>();
        set(copy.data(), location);
    }
};

template <>
struct uniform_dispatch<GL_FLOAT_MAT4x3> {
    static void set(const float* values, GLint location) {
        glUniformMatrix4x3fv(location, 12, GL_FALSE, values);
    }
    template <typename T, int Rows, int Cols, int Options>
    static void set(const Eigen::Matrix<T, Rows, Cols, Options>& matrix, GLint location) {
        if (matrix.rows() != 4 || matrix.cols() != 3) {
            throw std::runtime_error("uniform_dispatch::set(): Matrix dimensions used in variable set do not match. Must be a 4x3 matrix"+SPOT);
        }
        if (Options & Eigen::ColMajor) {
            set(matrix.template cast<float>().data(), location);
            return;
        }
        Eigen::Matrix<float, Rows, Cols, (Options ^ Eigen::RowMajor) | Eigen::ColMajor> copy = matrix.template cast<float>();
        set(copy.data(), location);
    }
};


} // harmont


#endif /* UNIFORM_DISPATCH_HPP_ */
