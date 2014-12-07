#include <iostream>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <harmont/harmont.hpp>
#include <harmont/openmesh_traits.hpp>
#include <boost/lexical_cast.hpp>
extern "C" {
#include "rgbe.h"
}
#include "deferred_renderer.hpp"
using namespace harmont;

typedef OpenMesh::Vec4f   color_t;
typedef tri_mesh<color_t> mesh_t;

freeglut_application::ptr   app_g;
deferred_renderer::ptr_t    renderer_g;

mesh_t                      mesh_g;
Eigen::AlignedBox<float, 3> bb_g;

vertex_array::ptr           vao_g;
vertex_buffer<float>::ptr   vbo_g;
index_buffer<uint32_t>::ptr ibo_g;
uint32_t                    num_indices_g;


void init() {
    Eigen::Vector3f light_dir = Eigen::Vector3f(1.f, 1.f, 1.f).normalized();

    deferred_renderer::render_parameters_t r_params {
        light_dir,
        0.8f,
        0.05f,
        false,
        "mesh.vert",
        "hdr.frag",
        "pisa_diffuse.hdr"
    };
    deferred_renderer::shadow_parameters_t s_params {
        2048,
        16,
        "shadow.vert",
        "shadow.frag",
    };
    renderer_g = std::make_shared<deferred_renderer>(r_params, s_params, bb_g, app_g->current_camera()->width(), app_g->current_camera()->height());

    vao_g = std::make_shared<vertex_array>();
    vao_g->bind();

    Eigen::MatrixXf vbo_data;
    Eigen::Matrix<uint32_t, Eigen::Dynamic, 1> ibo_data;
    mesh_traits<mesh_t>::buffer_data(mesh_g, {POSITION, COLOR, NORMAL}, vbo_data, ibo_data, false);
    num_indices_g = ibo_data.rows();

    vertex_buffer<float>::layout_t vbo_layout = {{"position", 3}, {"color", 1}, {"normal", 3}};
    vbo_g = vertex_buffer<float>::from_data(vbo_data);
    vbo_g->bind_to_array(vbo_layout, renderer_g->geometry_pass());

    ibo_g = index_buffer<uint32_t>::from_data(ibo_data);

    vao_g->release();
}

void render(shader_program::ptr program) {
    vao_g->bind();
    ibo_g->bind();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDrawElements(GL_TRIANGLES, num_indices_g, GL_UNSIGNED_INT, nullptr);
    ibo_g->release();
    vao_g->release();
}

void display(camera::ptr cam) {
    renderer_g->render(&render, cam, bb_g);
}

void reshape(camera::ptr cam) {
    renderer_g->reshape(cam);
}


int main (int argc, char* argv[]) {
    omerr().disable();
	OpenMesh::IO::Options opt;
	opt += OpenMesh::IO::Options::FaceNormal;
	opt += OpenMesh::IO::Options::VertexNormal;
	opt += OpenMesh::IO::Options::VertexColor;

	mesh_g.request_vertex_normals();
	mesh_g.request_face_normals();
    if (!OpenMesh::IO::read_mesh(mesh_g, argv[1], opt)) {
        std::cerr << "Unable to read mesh file. Aborting.\n";
        exit(1);
    }
    mesh_g.triangulate();
    if (!opt.face_has_normal()) {
        mesh_g.update_face_normals();
    }
    mesh_g.update_normals();
    // let us not expect materials to be set
    for (auto it = mesh_g.vertices_begin(); it != mesh_g.vertices_end(); ++it) {
        mesh_g.set_color(*it, color_t(1.f, 1.f, 1.f, 1.f));
        bb_g.extend(Eigen::Vector3f(mesh_g.point(*it).data()));
    }

    app_g = freeglut_application::create<orbit_camera_model>(800, 600, &display, &reshape);
    app_g->init(argc, argv, "Render Mesh", &init);
    app_g->on_drag_left([&] (Eigen::Vector2i pos, Eigen::Vector2i delta) { renderer_g->delta_clipping_height(-delta[1] * 0.01f); app_g->update(); });
    app_g->on_click_left([&] (Eigen::Vector2i pos) { Eigen::Vector3f dir = app_g->current_camera()->forward().normalized(); renderer_g->set_light_dir(dir, bb_g); app_g->update(); });
    app_g->on_click_right([&] (Eigen::Vector2i pos) { renderer_g->set_debug_position(pos); app_g->update(); });
    app_g->on_char([&] (unsigned char key) {
        if (key == 'b') renderer_g->delta_shadow_bias(-0.001);
        if (key == 'B') renderer_g->delta_shadow_bias(0.001);
        if (key == 'e') renderer_g->delta_exposure(-0.001f);
        if (key == 'E') renderer_g->delta_exposure(0.001f);
        if (key == 'r') renderer_g->delta_ssao_radius(-0.01f);
        if (key == 'R') renderer_g->delta_ssao_radius(0.01f);
        if (key == 'c' || key == 'C') renderer_g->toggle_clipping();
        app_g->update();
    });

    app_g->run();
}
