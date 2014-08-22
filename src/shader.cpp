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
#include <streambuf>

#include <cstdio>
#include <cstdlib>

#include <stdexcept>

#ifdef USE_PLUSTACHE
#include <plustache/plustache_types.hpp>
#include <plustache/template.hpp>
#endif // USE_PLUSTACHE


namespace harmont {

template <int Stage>
typename shader<Stage>::ptr shader<Stage>::from_file(std::string filename, bool compile_now) {
    return from_source(load_file_(filename, compile_now));
}

template <int Stage>
typename shader<Stage>::ptr shader<Stage>::from_source(std::string source, bool compile_now) {
    return std::make_shared<shader<Stage>>(source, compile_now);
}

#ifdef USE_PLUSTACHE
template <int Stage>
typename shader<Stage>::ptr shader<Stage>::from_file(std::string filename, const parameters_t& params, bool compile_now) {
    return from_source(load_file_(filename), params, compile_now);
}

template <int Stage>
typename shader<Stage>::ptr shader<Stage>::from_source(std::string source, const parameters_t& params, bool compile_now) {
    return from_source(render_source_(source, params), compile_now);
}
#endif // USE_PLUSTACHE

template <int Stage>
shader<Stage>::~shader() {
}

template <int Stage>
inline GLuint shader<Stage>::handle() {
	return handle_;
}

template <int Stage>
shader<Stage>::shader(std::string source, bool compile_now) : handle_(0), source_(source), compile_status_(0), link_status_(0) {
    if (compile_now) compile_();
}

template <int Stage>
void shader<Stage>::compile_() {
	handle_ = glCreateShader(Stage);
	glShaderSource(handle_, 1, source_.c_str(), NULL);
	glCompileShader(handle_);
	glGetShaderiv(handle_, GL_COMPILE_STATUS, &compile_status_);
	print_log_();

	if (!compile_status_) {
		throw std::runtime_error("shader::compile_: Could not compile shader."+SPOT);
	}
}

template <int Stage>
void shader<Stage>::print_log_() {
	int infologLength = 0;
	int charsWritten  = 0;
	GLchar *infoLog;

	glGetShaderiv(handle_, GL_INFO_LOG_LENGTH, &infologLength);

	if (infologLength > 1) {
		infoLog = (GLchar *)malloc(infologLength);
		if (infoLog == NULL) {
			throw std::runtime_error("shader::print_log_: Could not allocate info log buffer."+SPOT);
		}
		glGetShaderInfoLog(handle_, infologLength, &charsWritten, infoLog);
		switch (Stage) {
			case GL_VERTEX_SHADER:   std::cout << "Vertex Shader InfoLog:\n"+std::string(infoLog) << std::endl;   break;
			case GL_FRAGMENT_SHADER: std::cout << "Fragment Shader InfoLog:\n"+std::string(infoLog) << std::endl; break;
			case GL_GEOMETRY_SHADER: std::cout << "Geometry Shader InfoLog:\n"+std::string(infoLog) << std::endl; break;
			default: std::cout << "Unknown shader type" << std::endl;
		}
		free(infoLog);
	}
}

template <int Stage>
std::string shader<Stage>::load_file_(const std::string& filename) {
    std::ifstream in(filename.c_str());
    if (!in.good()) {
        throw std::runtime_error("shader::load_file_: Unable to open file \"" + filename + "\" for reading" + SPOT);
    }
    return str((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}


} // harmont


template class harmont::shader<GL_VERTEX_SHADER>;
template class harmont::shader<GL_FRAGMENT_SHADER>;
template class harmont::shader<GL_GEOMETRY_SHADER>;
