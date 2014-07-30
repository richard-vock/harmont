#include <common.hpp>


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

} // harmont
