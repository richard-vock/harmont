/*
 * harmont - c++ opengl wrapper library
 *
 * Written in 2014 by Richard Vock
 *
 * To the extent possible under law, the author(s) have dedicated all copyright and related and neighboring rights to this software to the public domain worldwide.
 * This software is distributed without any warranty.
 *
 * You should have received a copy of the CC0 Public Domain Dedication along with this software.
 * If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
 *
 */

#ifndef HARMONT_VERTEX_BUFFER_H_
#define HARMONT_VERTEX_BUFFER_H_

#include "render_pass.hpp"

namespace harmont {

template <typename Scalar, GLenum Target = GL_ARRAY_BUFFER>
class vertex_buffer {
	public:
		typedef std::shared_ptr<vertex_buffer>        ptr;
		typedef std::weak_ptr<vertex_buffer>          wptr;
		typedef std::shared_ptr<const vertex_buffer>  const_ptr;
		typedef std::weak_ptr<const vertex_buffer>    const_wptr;
        typedef std::pair<std::string, uint32_t>      layout_element_t;
        typedef std::vector<layout_element_t>         layout_t;
        template <typename S, int Dim>
        using eigen_vector = Eigen::Matrix<S, Dim, 1>;
        static constexpr std::size_t scalar_size = sizeof(Scalar);

	public:
		vertex_buffer(uint32_t element_count, GLenum usage = GL_STATIC_DRAW);
		virtual ~vertex_buffer();

        static ptr from_layout(const layout_t& layout);

        static ptr from_data(const Scalar* data, uint32_t element_count, GLenum usage = GL_STATIC_DRAW);
        template <template <typename, typename> class C, template <typename> class A>
        static ptr from_data(const C<Scalar, A<Scalar>>& sequence, GLenum usage = GL_STATIC_DRAW);
        template <template <typename, typename> class C, typename S, template <typename> class A>
        static ptr from_data(const C<S, A<S>>& sequence, GLenum usage = GL_STATIC_DRAW);
        template <int Dim, template <typename, typename> class C, template <typename> class A>
        static ptr from_data(const C<eigen_vector<Scalar, Dim>, A<eigen_vector<Scalar, Dim>>>& vector_sequence, GLenum usage = GL_STATIC_DRAW);
        template <int Dim, template <typename, typename> class C, typename S, template <typename> class A>
        static ptr from_data(const C<eigen_vector<S, Dim>, A<eigen_vector<S, Dim>>>& vector_sequence, GLenum usage = GL_STATIC_DRAW);
        template <int Rows, int Cols, int Options>
        static ptr from_data(const Eigen::Matrix<Scalar, Rows, Cols, Options>& matrix, GLenum usage = GL_STATIC_DRAW);
        template <typename S, int Rows, int Cols, int Options>
        static ptr from_data(const Eigen::Matrix<S, Rows, Cols, Options>& matrix, GLenum usage = GL_STATIC_DRAW);

        GLuint handle() const;
        static constexpr GLenum target() { return Target; }
        GLenum usage() const;
        uint32_t element_count() const;
        uint32_t data_size() const;
        bool bound() const;

        static GLuint bound_buffer();

        void bind();
        void release();

        void bind_to_array(const layout_t& layout, shader_program::ptr program);
        void bind_to_array(const layout_t& layout, render_pass::ptr pass);

        void set_data(const Scalar* data, uint32_t element_count);
        template <template <typename, typename> class C, template <typename> class A>
        void set_data(const C<Scalar, A<Scalar>>& sequence);
        template <template <typename, typename> class C, typename S, template <typename> class A>
        void set_data(const C<S, A<S>>& sequence);
        template <int Dim, template <typename, typename> class C, template <typename> class A>
        void set_data(const C<eigen_vector<Scalar, Dim>, A<eigen_vector<Scalar, Dim>>>& vector_sequence);
        template <int Dim, template <typename, typename> class C, typename S, template <typename> class A>
        void set_data(const C<eigen_vector<S, Dim>, A<eigen_vector<S, Dim>>>& vector_sequence);
        template <int Rows, int Cols, int Options>
        void set_data(const Eigen::Matrix<Scalar, Rows, Cols, Options>& matrix);
        template <typename S, int Rows, int Cols, int Options>
        void set_data(const Eigen::Matrix<S, Rows, Cols, Options>& matrix);

        void get_data(Scalar* data);
        template <template <typename, typename> class C, typename S, template <typename> class A>
        void get_data(C<S, A<S>>& sequence);
        template <int Dim, template <typename, typename> class C, typename S, template <typename> class A>
        void get_data(C<eigen_vector<S, Dim>, A<eigen_vector<S, Dim>>>& vector_sequence);
        template <typename S, int Rows, int Cols, int Options>
        void get_data(Eigen::Matrix<S, Rows, Cols, Options>& matrix);


    protected:
        void allocate_();

    protected:
        uint32_t element_count_;
        GLenum   usage_;
        uint32_t data_size_;
        GLuint   handle_;
};

#include <vertex_buffer.ipp>

template <typename Scalar>
using index_buffer = vertex_buffer<Scalar, GL_ELEMENT_ARRAY_BUFFER>;

template <typename Scalar>
using texture_buffer = vertex_buffer<Scalar, GL_TEXTURE_BUFFER>;


} // harmont

#endif /* HARMONT_VERTEX_BUFFER_H_ */
