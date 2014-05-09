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

#include <geometry.hpp>

using damogran::RGBA;

namespace harmont {


geometry::geometry() :
	has_vertices_(false),
	has_normals_(false),
	has_colors_(false),
	has_tex_coords_(false),
	has_indices_(false),
   init_vertices_(false),
	init_normals_(false),
	init_colors_(false),
	init_tex_coords_(false),
	init_indices_(false),
   has_data_vertices_(false),
	has_data_normals_(false),
	has_data_colors_(false),
	has_data_tex_coords_(false),
	has_data_indices_(false)	{
}

geometry::~geometry() {
}

void geometry::init() {
	vao_.init();
}

void geometry::enable_vertices() {
	has_vertices_ = true;
}

void geometry::enable_normals() {
	has_normals_ = true;
}

void geometry::enable_colors() {
	has_colors_ = true;
}

void geometry::enable_tex_coords() {
	has_tex_coords_ = true;
}

void geometry::enable_indices() {
	has_indices_ = true;
}

void geometry::disable_vertices() {
	has_vertices_ = false;
}

void geometry::disable_normals() {
	has_normals_ = false;
}

void geometry::disable_colors() {
	has_colors_ = false;
}

void geometry::disable_tex_coords() {
	has_tex_coords_ = false;
}

void geometry::disable_indices() {
	has_indices_ = false;
}

void geometry::bind_vertices(shader_program& program, const GLchar* var_name) {
	check_init_vertices_();
	int idx = get_buffer_index_(VERTICES);
	if (idx < 0) return;
	program.bind_attrib((GLuint)idx, var_name);
}

void geometry::bind_normals(shader_program& program, const GLchar* var_name) {
	check_init_normals_();
	int idx = get_buffer_index_(NORMALS);
	if (idx < 0) return;
	program.bind_attrib((GLuint)idx, var_name);
}

void geometry::bind_colors(shader_program& program, const GLchar* var_name) {
	check_init_colors_();
	int idx = get_buffer_index_(COLORS);
	if (idx < 0) return;
	program.bind_attrib((GLuint)idx, var_name);
}

void geometry::bind_tex_coords(shader_program& program, const GLchar* var_name) {
	check_init_tex_coords_();
	int idx = get_buffer_index_(TEXCOORDS);
	if (idx < 0) return;
	program.bind_attrib((GLuint)idx, var_name);
}

bool geometry::has_vertices() const {
	return has_vertices_;
}

bool geometry::has_normals() const {
	return has_normals_;
}

bool geometry::has_colors() const {
	return has_colors_;
}

bool geometry::has_tex_coords() const {
	return has_tex_coords_;
}

bool geometry::has_indices() const {
	return has_indices_;
}

void geometry::set_vertices(const std::vector<Eigen::Vector3f>& vertices) {
	check_init_vertices_();
	vbo_.set(vertices);
	has_data_vertices_ = true;
}

void geometry::set_normals(const std::vector<Eigen::Vector3f>& normals) {
	check_init_normals_();
	nbo_.set(normals);
	has_data_normals_ = true;
}

void geometry::set_colors(const std::vector<RGBA<float>>& colors) {
	std::vector<Eigen::Vector4f> data(colors.size());
	std::transform(colors.begin(), colors.end(), data.begin(), [&] (auto col) { return col.head(4); });

	check_init_colors_();
	cbo_.set(data);
	has_data_colors_ = true;
}

void geometry::set_tex_coords(const std::vector<Eigen::Vector2f>& tex_coords) {
	check_init_tex_coords_();
	tbo_.set(tex_coords);
	has_data_tex_coords_ = true;
}

void geometry::set_indices(const std::vector<GLuint>& indices) {
	check_init_indices_();
	ibo_.set(indices);
	has_data_indices_ = true;
}

void geometry::add_vertices(const std::vector<Eigen::Vector3f>& vertices) {
	if (!has_data_vertices_) {
		set_vertices(vertices);
		return;
	}
	vbo_.add(vertices);
}

void geometry::add_normals(const std::vector<Eigen::Vector3f>& normals) {
	if (!has_data_normals_) {
		set_normals(normals);
		return;
	}
	nbo_.add(normals);
}

void geometry::add_colors(const std::vector<RGBA<float>>& colors) {
	if (!has_data_colors_) {
		set_colors(colors);
		return;
	}

	std::vector<Eigen::Vector4f> data(colors.size());
	std::transform(colors.begin(), colors.end(), data.begin(), [&] (auto col) { return col.head(4); });
	cbo_.add(data);
}

void geometry::upload(std::set<buffers_t> dynamic_access, std::set<buffers_t> stream_access) {
	std::set<buffers_t> dyn(dynamic_access), strm(stream_access);
	auto get_type = [&] (buffers_t buf) {
		if (stream_access.find(buf) == stream_access.end()) {
			if (dynamic_access.find(buf) == dynamic_access.end()) {
				return STATIC_ACCESS;
			} else {
				return DYNAMIC_ACCESS;
			}
		}
		return STREAM_ACCESS;
	};
	if (has_data_vertices_) {
		upload_vertices(get_type(VERTICES));
	}
	if (has_data_normals_) {
		upload_normals(get_type(NORMALS));
	}
	if (has_data_colors_) {
		upload_colors(get_type(COLORS));
	}
	if (has_tex_coords_) {
		upload_tex_coords(get_type(TEXCOORDS));
	}
	if (has_data_indices_) {
		upload_indices(get_type(INDICES));
	}
}

void geometry::upload_vertices(access_t access) {
	if (!has_data_vertices_) {
		std::cout << "Trying to upload vertices without data! Skipping." << std::endl;
		return;
	}
	vbo_.upload(access);
	vao_.bind();
	if (buffer_index_map_.find(VERTICES) == buffer_index_map_.end()) {
		unsigned int idx = static_cast<unsigned int>(buffer_index_map_.size());
		buffer_index_map_[VERTICES] = idx;
	}
	vao_.set(buffer_index_map_[VERTICES],3);
	vao_.release();
}

void geometry::upload_normals(access_t access) {
	if (!has_data_normals_) {
		std::cout << "Trying to upload normals without data! Skipping." << std::endl;
		return;
	}
	nbo_.upload(access);
	vao_.bind();
	if (buffer_index_map_.find(NORMALS) == buffer_index_map_.end()) {
		unsigned int idx = static_cast<unsigned int>(buffer_index_map_.size());
		buffer_index_map_[NORMALS] = idx;
	}
	vao_.set(buffer_index_map_[NORMALS],3);
	vao_.release();
}

void geometry::upload_colors(access_t access) {
	if (!has_data_colors_) {
		std::cout << "Trying to upload colors without data! Skipping." << std::endl;
		return;
	}
	cbo_.upload(access);
	vao_.bind();
	if (buffer_index_map_.find(COLORS) == buffer_index_map_.end()) {
		unsigned int idx = static_cast<unsigned int>(buffer_index_map_.size());
		buffer_index_map_[COLORS] = idx;
	}
	vao_.set(buffer_index_map_[COLORS],4);
	vao_.release();
}

void geometry::upload_tex_coords(access_t access) {
	if (!has_data_tex_coords_) {
		std::cout << "Trying to upload tex coords without data! Skipping." << std::endl;
		return;
	}
	tbo_.upload(access);
	vao_.bind();
	if (buffer_index_map_.find(TEXCOORDS) == buffer_index_map_.end()) {
		unsigned int idx = static_cast<unsigned int>(buffer_index_map_.size());
		buffer_index_map_[TEXCOORDS] = idx;
	}
	vao_.set(buffer_index_map_[TEXCOORDS],2);
	vao_.release();
}

void geometry::upload_indices(access_t access) {
	if (!has_data_indices_) {
		std::cout << "Trying to upload indices without data! Skipping." << std::endl;
		return;
	}
	ibo_.upload(access);
}

GLuint geometry::get_vbo_id() const {
	return vbo_.id();
}

GLuint geometry::get_nbo_id() const {
	return nbo_.id();
}

GLuint geometry::get_cbo_id() const {
	return cbo_.id();
}

GLuint geometry::get_tbo_id() const {
	return tbo_.id();
}

GLuint geometry::get_ibo_id() const {
	return ibo_.id();
}

unsigned int geometry::get_vbo_size() const {
	return vbo_.data_size();
}

unsigned int geometry::get_nbo_size() const {
	return nbo_.data_size();
}

unsigned int geometry::get_cbo_size() const {
	return cbo_.data_size();
}

unsigned int geometry::get_tbo_size() const {
	return tbo_.data_size();
}

unsigned int geometry::get_ibo_size() const {
	return ibo_.data_size();
}

void geometry::bind() {
	vao_.bind();
	if (has_vertices_) vao_.enable(get_buffer_index_(VERTICES));
	if (has_normals_) vao_.enable(get_buffer_index_(NORMALS));
	if (has_colors_) vao_.enable(get_buffer_index_(COLORS));
	if (has_tex_coords_) vao_.enable(get_buffer_index_(TEXCOORDS));
	if (has_indices_) ibo_.bind();
}

void geometry::release() {
	if (has_vertices_) vao_.disable(get_buffer_index_(VERTICES));
	if (has_normals_) vao_.disable(get_buffer_index_(NORMALS));
	if (has_colors_) vao_.disable(get_buffer_index_(COLORS));
	if (has_tex_coords_) vao_.disable(get_buffer_index_(TEXCOORDS));
	if (has_indices_) ibo_.release();
	vao_.release();
}

void geometry::check_init_vertices_() {
	if (init_vertices_) return;
	vbo_.init();
	init_vertices_ = true;
}

void geometry::check_init_normals_() {
	if (init_normals_) return;
	nbo_.init();
	init_normals_ = true;
}

void geometry::check_init_colors_() {
	if (init_colors_) return;
	cbo_.init();
	init_colors_ = true;
}

void geometry::check_init_tex_coords_() {
	if (init_tex_coords_) return;
	tbo_.init();
	init_tex_coords_ = true;
}

void geometry::check_init_indices_() {
	if (init_indices_) return;
	ibo_.init();
	init_indices_ = true;
}

int geometry::get_buffer_index_(buffers_t buffer) {
	if (buffer_index_map_.find(buffer) == buffer_index_map_.end()) {
		std::cout << "Trying to get buffer index for uninitialized buffer." << std::endl;
		return -1;
	}
	return (int)buffer_index_map_[buffer];
}


} // harmont
