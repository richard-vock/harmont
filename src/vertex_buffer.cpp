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

#include <vertex_buffer.hpp>

namespace harmont {


template <typename Type, int Dim>
vertex_buffer<Type,Dim>::vertex_buffer() : data_buffer<Type>() {}

template <typename Type, int Dim>
vertex_buffer<Type,Dim>::~vertex_buffer() {}

template <typename Type, int Dim>
void vertex_buffer<Type,Dim>::set(const std::vector<vec_t>& data) {
	unsigned int data_size = static_cast<unsigned int>(Dim * data.size());
	if (data_size != this->data_size_) this->alloc_(data_size);
	for (unsigned int i=0; i < data.size(); ++i) {
		for (unsigned int d=0; d < Dim; ++d) {
			this->data_[i*Dim + d] = data[i][d];
		}
	}
}

template <typename Type, int Dim>
void vertex_buffer<Type,Dim>::add(const std::vector<vec_t>& data) {
	unsigned int data_size = static_cast<unsigned int>(Dim * data.size());
	unsigned int start_index = this->alloc_additional_(data_size);
	for (unsigned int i=0; i < data.size(); ++i) {
		for (unsigned int d=0; d < Dim; ++d) {
			this->data_[start_index + i*Dim + d] = data[i][d];
		}
	}
}

template <typename Type, int Dim>
void vertex_buffer<Type,Dim>::upload(access_t access) {
	this->upload_data_(GL_ARRAY_BUFFER, access);
}

template <typename Type, int Dim>
void vertex_buffer<Type,Dim>::bind() {
	glBindBuffer(GL_ARRAY_BUFFER, this->id_);
}

template <typename Type, int Dim>
void vertex_buffer<Type,Dim>::release() {
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}


} // harmont


// instantiate
#define SUPPORTED_TYPES \
	X(float)     \
	X(double)    \
	X(int)       \
	X(uint32_t)

#define X(type) \
	template class harmont::vertex_buffer<type, 2>; \
	template class harmont::vertex_buffer<type, 3>; \
	template class harmont::vertex_buffer<type, 4>;
SUPPORTED_TYPES
#undef X
#undef SUPPORTED_TYPES
