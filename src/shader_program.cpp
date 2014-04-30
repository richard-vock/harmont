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

#include <shader_program.hpp>

#include <Eigen/Dense>
using namespace Eigen;

using boost::optional;
using boost::none;


namespace harmont {


shader_program::shader_program() : ref_(0), link_status_(0) {
}

template <int Stage>
void shader_program::add_shader(typename shader<Stage>::ptr shader) {
	switch (Stage) {
		case GL_VERTEX_SHADER:   v_shader_ = std::dynamic_pointer_cast<vertex_shader>(shader); break;
		case GL_FRAGMENT_SHADER: f_shader_ = std::dynamic_pointer_cast<fragment_shader>(shader); break;
		case GL_GEOMETRY_SHADER: g_shader_ = std::dynamic_pointer_cast<geometry_shader>(shader); break;
		default: throw std::runtime_error("shader_program::add_shader: Unknown shader type"+SPOT);
	}
}

void shader_program::add_shaders(std::string v_shader, std::string f_shader, std::string g_shader) {
	this->add_shader<GL_VERTEX_SHADER>(vertex_shader::load(v_shader));
	this->add_shader<GL_FRAGMENT_SHADER>(fragment_shader::load(f_shader));
	if (g_shader != "") this->add_shader<GL_GEOMETRY_SHADER>(geometry_shader::load(g_shader));
}

void shader_program::link(optional<std::map<int, std::string>> output_map) {
	// create program
	ref_ = glCreateProgram();
	// attach shaders
	glAttachShader(ref_, v_shader_->ref());
	glAttachShader(ref_, f_shader_->ref());
	if (g_shader_) glAttachShader(ref_, g_shader_->ref());
	// link
	if (output_map) {
		for (const auto& p : output_map.get()) {
			glBindFragDataLocation(ref_, p.first, p.second.c_str());
		}
	}
	glLinkProgram(ref_);
	glGetProgramiv(ref_, GL_LINK_STATUS, &link_status_);
	print_log_();

	if (!link_status_) {
		throw std::runtime_error("shader_program::link: Unable to compile/link program"+SPOT);
	}
}

void shader_program::use() {
	glUseProgram(ref_);
}

void shader_program::bind_attrib(GLuint pos, const GLchar* name) {
	glBindAttribLocation(ref_, pos, name);
}

template <>
void shader_program::set_uniform<float>(const GLchar* name, const float& value) {
	GLint location = uniform_location_(name);
	glUniform1f(location, value);
}

template <>
void shader_program::set_uniform<int>(const GLchar* name, const int& value) {
	GLint location = uniform_location_(name);
	glUniform1i(location, value);
}

template <>
void shader_program::set_uniform<bool>(const GLchar* name, const bool& value) {
	this->set_uniform<int>(name, static_cast<int>(value));
}

template <>
void shader_program::set_uniform<Vector2f>(const GLchar* name, const Vector2f& value) {
	GLint location = uniform_location_(name);
	glUniform2fv(location, 1, value.data());
}

template <>
void shader_program::set_uniform<Vector2i>(const GLchar* name, const Vector2i& value) {
	GLint location = uniform_location_(name);
	glUniform2iv(location, 1, value.data());
}

template <>
void shader_program::set_uniform<Vector3f>(const GLchar* name, const Vector3f& value) {
	GLint location = uniform_location_(name);
	glUniform3fv(location, 1, value.data());
}

template <>
void shader_program::set_uniform<Vector3i>(const GLchar* name, const Vector3i& value) {
	GLint location = uniform_location_(name);
	glUniform3iv(location, 1, value.data());
}

template <>
void shader_program::set_uniform<Vector4f>(const GLchar* name, const Vector4f& value) {
	GLint location = uniform_location_(name);
	glUniform4fv(location, 1, value.data());
}

template <>
void shader_program::set_uniform<Vector4i>(const GLchar* name, const Vector4i& value) {
	GLint location = uniform_location_(name);
	glUniform4iv(location, 1, value.data());
}

template <>
void shader_program::set_uniform<Matrix2f>(const GLchar* name, const Matrix2f& value) {
	GLint location = uniform_location_(name);
	glUniformMatrix2fv(location, 1, false, value.data());
}

template <>
void shader_program::set_uniform<Matrix3f>(const GLchar* name, const Matrix3f& value) {
	GLint location = uniform_location_(name);
	glUniformMatrix3fv(location, 1, false, value.data());
}

template <>
void shader_program::set_uniform<Matrix4f>(const GLchar* name, const Matrix4f& value) {
	GLint location = uniform_location_(name);
	glUniformMatrix4fv(location, 1, false, value.data());
}

template <>
void shader_program::set_uniform<float>(const GLchar* name, GLsizei count, const float* values) {
	GLint location = uniform_location_(name);
	glUniform1fv(location, count, values);
}

template <>
void shader_program::set_uniform<int>(const GLchar* name, GLsizei count, const int* values) {
	GLint location = uniform_location_(name);
	glUniform1iv(location, count, values);
}

void shader_program::set_texture(const GLchar* name, int unit) {
	this->set_uniform<int>(name, unit);
}

bool shader_program::linked() const {
	return link_status_ != 0;
}

GLint shader_program::uniform_location_(const GLchar* name) {
	GLint loc;

	loc = glGetUniformLocation(ref_, name);

	if (loc == -1) {
		throw std::runtime_error("shader_program::uniform_location_: No uniform named "+std::string(name)+" in shader code"+SPOT);
	}

	return loc;
}

void shader_program::print_log_() {
	int infologLength = 0;
	int charsWritten  = 0;
	GLchar *infoLog;

	glGetProgramiv(ref_, GL_INFO_LOG_LENGTH, &infologLength);

	if (infologLength > 1) {
		infoLog = (GLchar *)malloc(infologLength);
		if (infoLog == NULL) {
			std::cout << "Could not allocate InfoLog buffer" << std::endl;
			return;
		}
		glGetProgramInfoLog(ref_, infologLength, &charsWritten, infoLog);
		std::cout << "Program InfoLog:\n"+std::string(infoLog) << std::endl;
		free(infoLog);
	}
}


} // harmont


template void harmont::shader_program::add_shader<GL_VERTEX_SHADER>(typename vertex_shader::ptr shader);
template void harmont::shader_program::add_shader<GL_FRAGMENT_SHADER>(typename fragment_shader::ptr shader);
template void harmont::shader_program::add_shader<GL_GEOMETRY_SHADER>(typename geometry_shader::ptr shader);
