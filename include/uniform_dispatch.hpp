#ifndef UNIFORM_DISPATCH_HPP_
#define UNIFORM_DISPATCH_HPP_

#include <vector>

namespace harmont {

struct uniform_dispatch {
    template <typename T>
    static void set(GLenum data_type, GLint size, GLint location, T&& value);

    template <typename T>
    static require<T, std::is_floating_point> set_(GLenum data_type, GLint size, GLint location, T&& value);

    template <template <typename, typename> class C, template <typename> class A>
    static void set_(GLenum data_type, GLint size, GLint location, const C<float,A<float>>& seq);

    template <template <typename, typename> class C, typename T, template <typename> class A>
    static require<T, std::is_floating_point> set_(GLenum data_type, GLint size, GLint location, const C<T,A<T>>& seq);

    template <int Rows, int Cols, int Options>
    static void set_(GLenum data_type, GLint size, GLint location, const Eigen::Matrix<float, Rows, Cols, Options>& matrix);

    template <typename T, int Rows, int Cols, int Options>
    static require<T, std::is_floating_point> set_(GLenum data_type, GLint size, GLint location, const Eigen::Matrix<T, Rows, Cols, Options>& matrix);

    template <typename T>
    static require<T, std::is_integral, std::is_signed> set_(GLenum data_type, GLint size, GLint location, T&& value);

    template <template <typename, typename> class C, template <typename> class A>
    static void set_(GLenum data_type, GLint size, GLint location, const C<int,A<int>>& seq);

    template <template <typename, typename> class C, typename T, template <typename> class A>
    static require<T, std::is_integral, std::is_signed> set_(GLenum data_type, GLint size, GLint location, const C<T,A<T>>& seq);

    template <int Rows, int Cols, int Options>
    static void set_(GLenum data_type, GLint size, GLint location, const Eigen::Matrix<int, Rows, Cols, Options>& matrix);

    template <typename T, int Rows, int Cols, int Options>
    static require<T, std::is_integral, std::is_signed> set_(GLenum data_type, GLint size, GLint location, const Eigen::Matrix<T, Rows, Cols, Options>& matrix);

    template <typename T>
    static require<T, std::is_integral, std::is_unsigned> set_(GLenum data_type, GLint size, GLint location, T&& value);

    template <template <typename, typename> class C, template <typename> class A>
    static void set_(GLenum data_type, GLint size, GLint location, const C<unsigned int,A<unsigned int>>& seq);

    template <template <typename, typename> class C, typename T, template <typename> class A>
    static require<T, std::is_integral, std::is_unsigned> set_(GLenum data_type, GLint size, GLint location, const C<T,A<T>>& seq);

    template <int Rows, int Cols, int Options>
    static void set_(GLenum data_type, GLint size, GLint location, const Eigen::Matrix<unsigned int, Rows, Cols, Options>& matrix);

    template <typename T, int Rows, int Cols, int Options>
    static require<T, std::is_integral, std::is_unsigned> set_(GLenum data_type, GLint size, GLint location, const Eigen::Matrix<T, Rows, Cols, Options>& matrix);

    template <typename T, int Rows, int Cols, int Options>
    static void set_matrix_(GLenum data_type, GLint size, GLint location, const Eigen::Matrix<T, Rows, Cols, Options>& matrix);

    static GLint element_size_(GLenum data_type);

    static std::pair<GLint, GLint> matrix_size_(GLenum data_type);

    static bool is_matrix_type_(GLenum data_type);

};

#include "uniform_dispatch.ipp"

} // harmont

#endif /* UNIFORM_DISPATCH_HPP_ */
