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

#ifndef HARMONT_TEXTURE_HPP
#define HARMONT_TEXTURE_HPP

#include "common.hpp"


namespace harmont {


class texture {
	public:
		typedef std::shared_ptr<texture>        ptr;
		typedef std::weak_ptr<texture>          wptr;
		typedef std::shared_ptr<const texture>  const_ptr;
		typedef std::weak_ptr<const texture>    const_wptr;


	public:
		template <typename Scalar>
		static ptr texture_1d(int width, int channels, const Scalar* data = nullptr, GLenum min_filter = GL_NEAREST, GLenum mag_filter = GL_NEAREST, GLenum wrap_s = GL_CLAMP_TO_EDGE, GLenum wrap_t = GL_CLAMP_TO_EDGE);

		template <typename Scalar, int Dim, int Options>
		static ptr texture_1d(const Eigen::Matrix<Scalar, Dim, 1, Options>& vector, const Scalar* data = nullptr, GLenum min_filter = GL_NEAREST, GLenum mag_filter = GL_NEAREST, GLenum wrap_s = GL_CLAMP_TO_EDGE, GLenum wrap_t = GL_CLAMP_TO_EDGE);

		template <typename Scalar>
		static ptr texture_2d(int width, int height, int channels, const Scalar* data = nullptr, GLenum min_filter = GL_NEAREST, GLenum mag_filter = GL_NEAREST, GLenum wrap_s = GL_CLAMP_TO_EDGE, GLenum wrap_t = GL_CLAMP_TO_EDGE);

		template <typename Scalar, int Rows, int Cols, int Options>
		static ptr texture_2d(const Eigen::Matrix<Scalar, Rows, Cols, Options>& matrix, GLenum min_filter = GL_NEAREST, GLenum mag_filter = GL_NEAREST, GLenum wrap_s = GL_CLAMP_TO_EDGE, GLenum wrap_t = GL_CLAMP_TO_EDGE);

		template <typename Scalar>
		static ptr texture_3d(int width, int height, int depth, int channels, const Scalar* data = nullptr, GLenum min_filter = GL_NEAREST, GLenum mag_filter = GL_NEAREST, GLenum wrap_s = GL_CLAMP_TO_EDGE, GLenum wrap_t = GL_CLAMP_TO_EDGE);

		template <typename Scalar>
		static ptr depth_texture(int width, int height);

		~texture();

		GLuint handle() const;

		int width() const;
		int height() const;
		int depth() const;
		int dim() const;
		int size() const;
		bool is_depth_attachment() const;

		void resize(int width, int height = 0, int depth = 0)

		void set_min_filter(GLenum filter);
		void set_mag_filter(GLenum filter);
		void set_filter(GLenum filter);
		void set_wrap_s(GLenum mode);
		void set_wrap_t(GLenum mode);
		void set_wrap(GLenum mode);

		template <typename Scalar>
		void set_data(const Scalar* data);

		template <typename Scalar, int Rows, int Cols, int Options>
		void set_data(const Eigen::Matrix<Scalar, Rows, Cols, Options>& matrix);

		void bind();
		static void release();

		static GLint  active_unit();
		static GLuint active_texture(GLenum target, GLenum unit);

	protected:
		struct parameters_t_ {
			GLenum min_filter = GL_NEAREST;
			GLenum mag_filter = GL_NEAREST;
			GLenum wrap_s = GL_CLAMP_TO_EDGE;
			GLenum wrap_t = GL_CLAMP_TO_EDGE;
		};
		template <typename Scalar>
		texture(GLenum scalar_type, GLenum internal_format, int width, int height = 0, int depth = 0, const Scalar* data = nullptr, parameters_t_ params = parameters_t_(), bool is_depth_attachment = false);

		void allocate_();

	protected:
		GLuint        handle_;
		int           width_;
		int           height_;
		int           depth_;
		GLenum        scalar_type_;
		GLenum        internal_format_;
		GLenum        target_;
		GLenum        dims_;
		parameters_t_ params_;
		bool          is_depth_attachment_;
};

#include "texture.ipp"

} // harmont


#endif // HARMONT_TEXTURE_HPP
