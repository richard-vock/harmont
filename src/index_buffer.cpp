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

#include <index_buffer.hpp>

namespace harmont {


index_buffer::index_buffer() : data_buffer<uint32_t>() {
}

index_buffer::~index_buffer() {
}

void index_buffer::upload(access_t access) {
	this->upload_data_(GL_ELEMENT_ARRAY_BUFFER, access);
}

void index_buffer::bind() {
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->id_);
}

void index_buffer::release() {
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}


} // harmont
