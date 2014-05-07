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

#include <data_buffer.hpp>

namespace harmont {

template <typename Type>
struct type_id;
template <> struct type_id<float>    { static constexpr GLuint value = GL_FLOAT;  };
template <> struct type_id<double>   { static constexpr GLuint value = GL_DOUBLE; };
template <> struct type_id<int>      { static constexpr GLuint value = GL_INT; };
template <> struct type_id<uint32_t> { static constexpr GLuint value = GL_UNSIGNED_INT; };


template <typename Type>
data_buffer<Type>::data_buffer() : data_(NULL), id_(0), data_size_(0) {
}

template <typename Type>
data_buffer<Type>::~data_buffer() {
	if (id_) glDeleteBuffers(1, &id_);
	delete [] data_;
	data_ = NULL;
	id_ = 0;
}

template <typename Type>
void data_buffer<Type>::init() {
	glGenBuffers(1, &id_);
}

template <typename Type>
void data_buffer<Type>::set(std::vector<Type> data) {
	if (!data_) alloc_(static_cast<uint32_t>(data.size()));
	for (uint32_t i=0; i < data.size(); ++i) { data_[i] = data[i]; }
}

template <typename Type>
GLuint data_buffer<Type>::id() const {return id_;}

template <typename Type>
uint32_t data_buffer<Type>::data_size() const {return data_size_;}

template <typename Type>
constexpr GLuint data_buffer<Type>::type_id_() {
	return type_id<Type>::value;
}

template <typename Type>
void data_buffer<Type>::alloc_(uint32_t count) {
	data_ = new Type[count];
	data_size_ = count;
}

template <typename Type>
uint32_t data_buffer<Type>::alloc_additional_(uint32_t count) {
	Type* new_array = new Type[count+data_size_];
	memcpy((void*)new_array, (const void*)data_, data_size_*sizeof(Type));
	delete [] data_;
	data_ = new_array;
	uint32_t start_index = data_size_;
	data_size_ += count;
	return start_index;
}

template <typename Type>
void data_buffer<Type>::upload_data_(int target, access_t access) {
	glBindBuffer(target, id_);
	glBufferData(target, sizeof(Type)*data_size_, data_, access);
}


} // harmont


// instantiations
template class harmont::data_buffer<float>;
template class harmont::data_buffer<double>;
template class harmont::data_buffer<int>;
template class harmont::data_buffer<uint32_t>;
