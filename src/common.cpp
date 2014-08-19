#include <common.hpp>

#include <functional>

namespace harmont {

template <>
struct gl_type_enum<int8_t> {
	constexpr GLenum value = GL_BYTE;
};

template <>
struct gl_type_enum<uint8_t> {
	constexpr GLenum value = GL_UNSIGNED_BYTE;
};

template <>
struct gl_type_enum<int16_t> {
	constexpr GLenum value = GL_SHORT;
};

template <>
struct gl_type_enum<uint16_t> {
	constexpr GLenum value = GL_UNSIGNED_SHORT;
};

template <>
struct gl_type_enum<int32_t> {
	constexpr GLenum value = GL_INT;
};

template <>
struct gl_type_enum<uint32_t> {
	constexpr GLenum value = GL_UNSIGNED_INT;
};

template <>
struct gl_type_enum<float> {
	constexpr GLenum value = GL_FLOAT;
};

template <>
struct gl_type_enum<double> {
	constexpr GLenum value = GL_DOUBLE;
};

template <>
struct gl_attrib_func<int8_t> {
    static gl_attrib_func_t get_functor() {
        return std::bind(&glVertexAttribIPointer,
                          std::placeholders::_1,
                          std::placeholders::_2,
                          GL_BYTE,
                          std::placeholders::_3,
                          std::placeholders::_4
        );
    }
};

template <>
struct gl_attrib_func<uint8_t> {
    static gl_attrib_func_t get_functor() {
        return std::bind(&glVertexAttribIPointer,
                          std::placeholders::_1,
                          std::placeholders::_2,
                          GL_UNSIGNED_BYTE,
                          std::placeholders::_3,
                          std::placeholders::_4
        );
    }
};

template <>
struct gl_attrib_func<int16_t> {
    static gl_attrib_func_t get_functor() {
        return std::bind(&glVertexAttribIPointer,
                          std::placeholders::_1,
                          std::placeholders::_2,
                          GL_SHORT,
                          std::placeholders::_3,
                          std::placeholders::_4
        );
    }
};

template <>
struct gl_attrib_func<uint16_t> {
    static gl_attrib_func_t get_functor() {
        return std::bind(&glVertexAttribIPointer,
                          std::placeholders::_1,
                          std::placeholders::_2,
                          GL_UNSIGNED_SHORT,
                          std::placeholders::_3,
                          std::placeholders::_4
        );
    }
};

template <>
struct gl_attrib_func<int32_t> {
    static gl_attrib_func_t get_functor() {
        return std::bind(&glVertexAttribIPointer,
                          std::placeholders::_1,
                          std::placeholders::_2,
                          GL_INT,
                          std::placeholders::_3,
                          std::placeholders::_4
        );
    }
};

template <>
struct gl_attrib_func<uint32_t> {
    static gl_attrib_func_t get_functor() {
        return std::bind(&glVertexAttribIPointer,
                          std::placeholders::_1,
                          std::placeholders::_2,
                          GL_UNSIGNED_INT,
                          std::placeholders::_3,
                          std::placeholders::_4
        );
    }
};

template <>
struct gl_attrib_func<float> {
    static gl_attrib_func_t get_functor() {
        return std::bind(&glVertexAttribPointer,
                          std::placeholders::_1,
                          std::placeholders::_2,
                          GL_FLOAT,
                          false,
                          std::placeholders::_3,
                          std::placeholders::_4
        );
    }
};

template <>
struct gl_attrib_func<double> {
    static gl_attrib_func_t get_functor() {
        return std::bind(&glVertexAttribPointer,
                          std::placeholders::_1,
                          std::placeholders::_2,
                          GL_DOUBLE,
                          false,
                          std::placeholders::_3,
                          std::placeholders::_4
        );
    }
};

} // harmont
