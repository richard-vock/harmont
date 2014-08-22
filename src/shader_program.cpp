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

#include <tuple>
#include <vector>
#include <Eigen/Dense>
using namespace Eigen;


using boost::optional;
using boost::none;


namespace harmont {


shader_program::variable::description_t variable_for_index(GLuint program, GLuint index, variable_type type) {
    GLchar c_name[64];
    GLsizei length, buf_size = 64;
    GLenum type;
    GLint size;
    switch (type) {
        case ATTRIBUTE: glGetActiveAttrib(program, index, buf_size, length, size, type, name); break;
        default: glGetActiveUniform(program, index, buf_size, &length, &size, &type, c_name);
    }
    return std::make_tuple(std::string(c_name, c_name+length), type, size);
}

std::vector<shader_program::variable::description_t> program_variables(GLuint program, variable_type type) {
    GLint num_variables;
    glGetProgramiv(program, type == ATTRIBUTE ? GL_ACTIVE_ATTRIBUTES : GL_ACTIVE_UNIFORMS, &num_variables);
    std::vector<shader_program::variable::description_t> variables;
    for (GLint i = 0; i < num_variables; ++i) {
        variables.push_back(variable_for_index(program, i, type));
    }
    return variables;
}

shader_program::variable::description_t variable_for_name(GLuint program, std::string name, variable_type type) {
    auto variables = program_variables(program, type);
    for (const auto& v : variables) {
        if (std::get<0>(v) == name) return v;
    }
    throw std::runtime_error("no " + (type == ATTRIBUTE ? "attribute" : "uniform") + " named \"" + name + "\"");
}

shader_program::shader_program(vertex_shader::ptr vs, fragment_shader::ptr fs, bool link_now) : shader_program(vs, fs, nullptr, link_now) {
}

shader_program::shader_program(vertex_shader::ptr vs, fragment_shader::ptr fs, geometry_shader::ptr gs, bool link_now) : handle_(0), vs_(vs), fs_(fs), gs_(gs), link_status_(0) {
    handle_ = glCreateProgram();

    if (vs_) attach_shader_(vs_);
    if (fs_) attach_shader_(fs_);
    if (gs_) attach_shader_(gs_);

    if (link_now) link();
}

shader_program::~shader_program() {
}

GLuint shader_program::handle() const {
    return handle_;
}

void shader_program::link() {
	glLinkProgram(handle_);
	glGetProgramiv(handle_, GL_LINK_STATUS, &link_status_);
	print_log_();

	if (!link_status_) {
		throw std::runtime_error("shader_program::link: Unable to compile/link program"+SPOT);
	}

    auto descs = program_variables(handle_, UNIFORM);
    for (const auto& d : descs) {
        uniforms_[std::get<0>(d)] = std::make_shared<variable>(handle_, d, UNIFORM);
    }
    descs = program_variables(handle_, ATTRIBUTE);
    for (const auto& d : descs) {
        attributes_[std::get<0>(d)] = std::make_shared<variable>(handle_, d, ATTRIBUTE);
    }
}

bool shader_program::linked() const {
    return link_status_ != 0;
}

void shader_program::bind() {
    glUseProgram(handle_);
}

void shader_program::release() {
    glUseProgram(0);
}

bool shader_program::bound() const {
    GLuint crt;
    glGetIntegerv(GL_CURRENT_PROGRAM, &crt);
    return crt == handle_;
}

shader_program::variable_ptr shader_program::operator[](std::string name) {
    auto found = variables_.find(name);
    if (found == variables_.end()) {
        throw std::runtime_error("shader_program::operator[](): No variable named \""+name+"\""+SPOT);
    }
    return *found;
}

template <int Stage>
void attach_shader_(shader<Stage>::ptr shader) {
    if (!shader) return;
    glAttachShader(handle_, shader->handle());
    print_log_();
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


template void harmont::shader_program::attach_shader_<GL_VERTEX_SHADER>(typename shader<GL_VERTEX_SHADER>::ptr shader);
template void harmont::shader_program::attach_shader_<GL_FRAGMENT_SHADER>(typename shader<GL_FRAGMENT_SHADER>::ptr shader);
template void harmont::shader_program::attach_shader_<GL_GEOMETRY_SHADER>(typename shader<GL_GEOMETRY_SHADER>::ptr shader);
