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

#include "data_buffer.hpp"
#include <Eigen/Dense>

#include <damogran/colors.hpp>

namespace harmont {

template <class Type, int Dim>
class vertex_buffer : public data_buffer<Type> {
	public:
		typedef Eigen::Matrix<Type,Dim,1> vec_t;

	public:
		vertex_buffer();
		virtual ~vertex_buffer();

		void set(const std::vector<vec_t>& data);
		void add(const std::vector<vec_t>& data);

		void set(const std::vector<damogran::RGBA<Type>>& colors);
		void add(const std::vector<damogran::RGBA<Type>>& colors);

		void upload(access_t access);

		void bind();
		void release();
};


} // harmont

#endif /* HARMONT_VERTEX_BUFFER_H_ */
