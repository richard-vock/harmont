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

#ifndef HARMONT_DATA_BUFFER_H_
#define HARMONT_DATA_BUFFER_H_

#include "common.hpp"
#include <iostream>
#include <cstring>
#include <vector>

namespace harmont {

typedef enum {STATIC_ACCESS = GL_STATIC_DRAW, DYNAMIC_ACCESS = GL_DYNAMIC_DRAW} access_t;

template <class Type>
class data_buffer {
	public:
		data_buffer();
		virtual ~data_buffer();

		void init();

		virtual void set(std::vector<Type> data);

		virtual void upload(access_t access) = 0;

		virtual void bind() = 0;
		virtual void release() = 0;

		GLuint id() const;
		uint32_t data_size() const;

	protected:
		static constexpr GLuint type_id_();
		void alloc_(uint32_t count);
		uint32_t alloc_additional_(uint32_t count);
		void upload_data_(int target, access_t access);

	protected:
		Type*    data_;
		GLuint   id_;
		uint32_t data_size_;
};


} // harmont

#endif /* HARMONT_DATA_BUFFER_H_ */
