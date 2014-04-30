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

#include <shader.hpp>

#include <iostream>
#include <fstream>
using std::ifstream;

#include <cstdio>
#include <cstdlib>

#include <stdexcept>

namespace harmont {

template <int Stage>
typename shader<Stage>::ptr shader<Stage>::load(std::string filename) {
	if (Stage != GL_VERTEX_SHADER && Stage != GL_FRAGMENT_SHADER && Stage != GL_GEOMETRY_SHADER) {
		return ptr();
	}
	ptr result(new shader());

	result->read_code_(filename);
	result->create_();
	result->compile_();

	return result;
}

template <int Stage>
shader<Stage>::~shader() {
	if (code_) delete [] code_;
}

template <int Stage>
inline GLenum shader<Stage>::stage() {
	return Stage;
}
template <int Stage>
inline GLuint shader<Stage>::ref() {
	return ref_;
}

template <int Stage>
shader<Stage>::shader() : ref_(0), code_(NULL), compile_status_(0), link_status_(0) {
}

template <int Stage>
void shader<Stage>::read_code_(std::string filename) {
	std::ifstream f;
	f.open(filename.c_str(), std::ios::binary);
	if(!f.is_open() || !f.good()) {
		throw std::runtime_error("shader::read_code_: Could not open file for reading."+SPOT);
		return;
	}
	std::ifstream::pos_type begin_pos = f.tellg();
	f.seekg(0, std::ios_base::end);
	int vertFileSize = static_cast<int>(f.tellg() - begin_pos);
	f.seekg(0, std::ios_base::beg);
	code_ = new GLchar[vertFileSize + 1];
	code_[vertFileSize] = 0;
	f.read(code_, vertFileSize);
	if (!code_) {
		throw std::runtime_error("shader::read_code_: No shader code read."+SPOT);
	}
}

template <int Stage>
void shader<Stage>::create_() {
	ref_ = glCreateShader(Stage);
	const GLchar* code = code_;
	glShaderSource(ref_, 1, &code, NULL);
}

template <int Stage>
void shader<Stage>::compile_() {
	glCompileShader(ref_);
	glGetShaderiv(ref_, GL_COMPILE_STATUS, &compile_status_);
	print_log_();

	if (!compile_status_) {
		throw std::runtime_error("shader::compile_: Could not open compile shader."+SPOT);
	}
}

template <int Stage>
void shader<Stage>::print_log_() {
	int infologLength = 0;
	int charsWritten  = 0;
	GLchar *infoLog;

	glGetShaderiv(ref_, GL_INFO_LOG_LENGTH, &infologLength);

	if (infologLength > 1) {
		infoLog = (GLchar *)malloc(infologLength);
		if (infoLog == NULL) {
			throw std::runtime_error("shader::print_log_: Could not allocate info log buffer."+SPOT);
		}
		glGetShaderInfoLog(ref_, infologLength, &charsWritten, infoLog);
		switch (Stage) {
			case GL_VERTEX_SHADER:   std::cout << "Vertex Shader InfoLog:\n"+std::string(infoLog) << std::endl;   break;
			case GL_FRAGMENT_SHADER: std::cout << "Fragment Shader InfoLog:\n"+std::string(infoLog) << std::endl; break;
			case GL_GEOMETRY_SHADER: std::cout << "Geometry Shader InfoLog:\n"+std::string(infoLog) << std::endl; break;
			default: std::cout << "Unknown shader type" << std::endl;
		}
		free(infoLog);
	}
}

} // harmont

template class harmont::shader<GL_VERTEX_SHADER>;
template class harmont::shader<GL_FRAGMENT_SHADER>;
template class harmont::shader<GL_GEOMETRY_SHADER>;
