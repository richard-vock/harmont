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

#ifndef HARMONT_GEOMETRY_H_
#define HARMONT_GEOMETRY_H_

#include <map>
#include <set>

#include <damogran/colors.hpp>

#include "common.hpp"
#include "shader_program.hpp"
#include "vertex_buffer.hpp"
#include "index_buffer.hpp"
#include "vertex_array.hpp"

namespace harmont {


class geometry {
	public:
		typedef std::shared_ptr<geometry>        ptr;
		typedef std::weak_ptr<geometry>          wptr;
		typedef std::shared_ptr<const geometry>  const_ptr;
		typedef std::weak_ptr<const geometry>    const_wptr;

	public:
		typedef vertex_buffer<GLfloat, 2>                             vbo2_t;
		typedef vertex_buffer<GLfloat, 3>                             vbo3_t;
		typedef vertex_buffer<GLfloat, 4>                             vbo4_t;
		typedef enum {VERTICES, NORMALS, COLORS, TEXCOORDS, INDICES}  buffers_t;

	public:
		geometry();
		virtual ~geometry();

		void init();

		void enable_vertices();
		void enable_normals();
		void enable_colors();
		void enable_tex_coords();
		void enable_indices();

		void disable_vertices();
		void disable_normals();
		void disable_colors();
		void disable_tex_coords();
		void disable_indices();

		void bind_vertices(shader_program& program, const GLchar* var_name);
		void bind_normals(shader_program& program, const GLchar* var_name);
		void bind_colors(shader_program& program, const GLchar* var_name);
		void bind_tex_coords(shader_program& program, const GLchar* var_name);

		bool has_vertices()  const;
		bool has_normals()   const;
		bool has_colors()    const;
		bool has_tex_coords() const;
		bool has_indices()   const;

		void set_vertices (const std::vector<Eigen::Vector3f>& vertices);
		void set_normals  (const std::vector<Eigen::Vector3f>& normals);
		void set_colors   (const std::vector<damogran::RGBA<float>>& colors);
		void set_tex_coords(const std::vector<Eigen::Vector2f>& tex_coords);
		void set_indices  (const std::vector<GLuint>& indices);

		void add_vertices (const std::vector<Eigen::Vector3f>& vertices);
		void add_normals  (const std::vector<Eigen::Vector3f>& normals);
		void add_colors   (const std::vector<damogran::RGBA<float>>& colors);

		void upload(std::set<buffers_t> dynamic_access = std::set<buffers_t>(), std::set<buffers_t> stream_access = std::set<buffers_t>());
		void upload_vertices(access_t access = STATIC_ACCESS);
		void upload_normals(access_t access = STATIC_ACCESS);
		void upload_colors(access_t access = STATIC_ACCESS);
		void upload_tex_coords(access_t access = STATIC_ACCESS);
		void upload_indices(access_t access = STATIC_ACCESS);

		GLuint get_vbo_id() const;
		GLuint get_nbo_id() const;
		GLuint get_cbo_id() const;
		GLuint get_tbo_id() const;
		GLuint get_ibo_id() const;

		unsigned int get_vbo_size() const;
		unsigned int get_nbo_size() const;
		unsigned int get_cbo_size() const;
		unsigned int get_tbo_size() const;
		unsigned int get_ibo_size() const;

		void bind();
		void release();

	protected:
		void check_init_vertices_();
		void check_init_normals_();
		void check_init_colors_();
		void check_init_tex_coords_();
		void check_init_indices_();

		int get_buffer_index_(buffers_t buffer);

	protected:
		vertex_array  vao_;
		vbo3_t        vbo_;
		vbo3_t        nbo_;
		vbo4_t        cbo_;
		vbo2_t        tbo_;
		index_buffer  ibo_;

		bool  has_vertices_;
		bool  has_normals_;
		bool  has_colors_;
		bool  has_tex_coords_;
		bool  has_indices_;

		bool  init_vertices_;
		bool  init_normals_;
		bool  init_colors_;
		bool  init_tex_coords_;
		bool  init_indices_;

		bool  has_data_vertices_;
		bool  has_data_normals_;
		bool  has_data_colors_;
		bool  has_data_tex_coords_;
		bool  has_data_indices_;

		std::map<buffers_t, GLuint>  buffer_index_map_;
};


} // harmont

#endif /* HARMONT_GEOMETRY_H_ */
