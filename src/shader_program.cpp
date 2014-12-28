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

#include <vector>

namespace harmont {


shader_program::variable_t::description_t variable_for_index(GLuint program, GLuint index, shader_program::variable_type type) {
    GLchar c_name[64];
    GLsizei length, buf_size = 64;
    GLenum var_type;
    GLint size;
    switch (type) {
        case shader_program::ATTRIBUTE: glGetActiveAttrib(program, index, buf_size, &length, &size, &var_type, c_name); break;
        default: glGetActiveUniform(program, index, buf_size, &length, &size, &var_type, c_name);
    }
    return std::make_tuple(std::string(c_name, c_name+length), var_type, size);
}

std::vector<shader_program::variable_t::description_t> program_variables(GLuint program, shader_program::variable_type type) {
    GLint num_variables;
    glGetProgramiv(program, type == shader_program::ATTRIBUTE ? GL_ACTIVE_ATTRIBUTES : GL_ACTIVE_UNIFORMS, &num_variables);
    std::vector<shader_program::variable_t::description_t> variables;
    for (GLint i = 0; i < num_variables; ++i) {
        variables.push_back(variable_for_index(program, i, type));
    }
    return variables;
}

shader_program::variable_t::description_t variable_for_name(GLuint program, std::string name, shader_program::variable_type type) {
    auto variables = program_variables(program, type);
    for (const auto& v : variables) {
        if (std::get<0>(v) == name) return v;
    }
    throw std::runtime_error(std::string("no ") + (type == shader_program::ATTRIBUTE ? "attribute" : "uniform") + " named \"" + name + "\"");
}

shader_program::shader_program(vertex_shader::ptr vs, fragment_shader::ptr fs, bool link_now) : shader_program(vs, fs, nullptr, link_now) {
}

shader_program::shader_program(const std::vector<vertex_shader::ptr>& vs, const std::vector<fragment_shader::ptr>& fs, bool link_now) : shader_program(vs, fs, std::vector<geometry_shader::ptr>(), link_now) {
}

shader_program::shader_program(vertex_shader::ptr vs, fragment_shader::ptr fs, geometry_shader::ptr gs, bool link_now) : handle_(0), link_status_(0) {
    handle_ = glCreateProgram();

    if (vs) attach_shader_<GL_VERTEX_SHADER>(vs);
    if (fs) attach_shader_<GL_FRAGMENT_SHADER>(fs);
    if (gs) attach_shader_<GL_GEOMETRY_SHADER>(gs);

    if (link_now) link();
}

shader_program::shader_program(const std::vector<vertex_shader::ptr>& vs, const std::vector<fragment_shader::ptr>& fs, const std::vector<geometry_shader::ptr>& gs, bool link_now) : handle_(0), link_status_(0) {
    handle_ = glCreateProgram();

    for (auto s : vs) if (s) attach_shader_<GL_VERTEX_SHADER>(s);
    for (auto s : fs) if (s) attach_shader_<GL_FRAGMENT_SHADER>(s);
    for (auto s : gs) if (s) attach_shader_<GL_GEOMETRY_SHADER>(s);

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
        uniforms_[std::get<0>(d)] = variable_ptr(new variable_t(handle_, d, UNIFORM));
    }
    descs = program_variables(handle_, ATTRIBUTE);
    for (const auto& d : descs) {
        attributes_[std::get<0>(d)] = variable_ptr(new variable_t(handle_, d, ATTRIBUTE));
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
    GLint crt;
    glGetIntegerv(GL_CURRENT_PROGRAM, &crt);
    return crt == static_cast<GLint>(handle_);
}

const shader_program::variable_t& shader_program::operator[](std::string name) const {
    return variable(name);
}

const shader_program::variable_t& shader_program::variable(std::string name) const {
    auto found = uniforms_.find(name);
    if (found != uniforms_.end()) return *(found->second);
    found = attributes_.find(name);
    if (found != attributes_.end()) return *(found->second);
    throw std::runtime_error("shader_program::variable_t(): No variable named \""+name+"\""+SPOT);
}

template <int Stage>
void shader_program::attach_shader_(typename shader<Stage>::ptr shader) {
    if (!shader) return;
    glAttachShader(handle_, shader->handle());
    print_log_();
}

void shader_program::print_log_() {
	int infologLength = 0;
	int charsWritten  = 0;
	GLchar *infoLog;

	glGetProgramiv(handle_, GL_INFO_LOG_LENGTH, &infologLength);

	if (infologLength > 1) {
		infoLog = (GLchar *)malloc(infologLength);
		if (infoLog == NULL) {
			std::cout << "Could not allocate InfoLog buffer" << std::endl;
			return;
		}
		glGetProgramInfoLog(handle_, infologLength, &charsWritten, infoLog);
		std::cout << "Program InfoLog:\n"+std::string(infoLog) << std::endl;
		free(infoLog);
	}
}


shader_program::variable_t::variable_t(GLuint program, const description_t& desc, variable_type var_type) : program_(program), desc_(desc), var_type_(var_type), name_(std::get<0>(desc)), data_type_(std::get<1>(desc)), size_(std::get<2>(desc)) {
    if (var_type == UNIFORM) {
        location_ = glGetUniformLocation(program_, name_.c_str());
    } else {
        location_ = glGetAttribLocation(program_, name_.c_str());
    }
}

shader_program::variable_t::variable_t(const variable_t& other) {
    program_ = other.program_;
    desc_ = other.desc_;
    var_type_ = other.var_type_;
    name_ = other.name_;
    data_type_ = other.data_type_;
    size_ = other.size_;
    location_ = other.location_;
}

shader_program::variable_t& shader_program::variable_t::operator=(const variable_t& other) {
    program_ = other.program_;
    desc_ = other.desc_;
    var_type_ = other.var_type_;
    name_ = other.name_;
    data_type_ = other.data_type_;
    size_ = other.size_;
    location_ = other.location_;
    return *this;
}

shader_program::variable_t& shader_program::variable_t::operator=(variable_t&& other) {
    program_ = other.program_;
    desc_ = other.desc_;
    var_type_ = other.var_type_;
    name_ = other.name_;
    data_type_ = other.data_type_;
    size_ = other.size_;
    location_ = other.location_;
    return *this;
}

shader_program::variable_t::~variable_t() {
}

shader_program::variable_type shader_program::variable_t::type() const {
    return var_type_;
}

const std::string& shader_program::variable_t::name() const {
    return name_;
}

GLenum shader_program::variable_t::data_type() const {
    return data_type_;
}

GLint shader_program::variable_t::size() const {
    return size_;
}

bool shader_program::variable_t::is_uniform() const {
    return var_type_ == UNIFORM;
}

bool shader_program::variable_t::is_attribute() const {
    return var_type_ == ATTRIBUTE;
}

GLuint shader_program::variable_t::location() const {
    return location_;
}


} // harmont

template void harmont::shader_program::attach_shader_<GL_VERTEX_SHADER>(typename shader<GL_VERTEX_SHADER>::ptr shader);
template void harmont::shader_program::attach_shader_<GL_FRAGMENT_SHADER>(typename shader<GL_FRAGMENT_SHADER>::ptr shader);
template void harmont::shader_program::attach_shader_<GL_GEOMETRY_SHADER>(typename shader<GL_GEOMETRY_SHADER>::ptr shader);
