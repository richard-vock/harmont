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

#ifndef HARMONT_INDEX_BUFFER_H_
#define HARMONT_INDEX_BUFFER_H_

#include "data_buffer.hpp"

namespace harmont {

class index_buffer : public data_buffer<uint32_t> {
	public:
		index_buffer();
		virtual ~index_buffer();

		void upload(access_t access);
		void bind();
		void release();
};

} // harmont

#endif /* HARMONT_INDEX_BUFFER_H_ */
