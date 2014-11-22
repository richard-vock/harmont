#ifndef GEOMETRIC_TRAITS_HPP_
#define GEOMETRIC_TRAITS_HPP_

#include <tuple>
#include "vertex_buffer.hpp"

namespace harmont {

template <typename T>
struct geometric_traits {
    typedef vertex_buffer<float> vbo_t;
    typedef index_buffer<GLuint> ibo_t;
    // vbo_t::ptr get_buffers(const T& geometry);
    // std::tuple<vbo_t::ptr, ibo_t::ptr> get_indexed_buffers(const T& geometry);
};

} // harmont

#endif /* GEOMETRIC_TRAITS_HPP_ */
