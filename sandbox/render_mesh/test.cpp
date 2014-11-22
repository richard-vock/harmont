#include <iostream>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <harmont/harmont.hpp>
#include <harmont/openmesh_traits.hpp>
using namespace harmont;

typedef OpenMesh::Vec4f   color_t;
typedef tri_mesh<color_t> mesh_t;

mesh_t mesh_g;
vertex_array::ptr vao_g;
vertex_buffer<float>::ptr vbo_g;
index_buffer<uint32_t>::ptr ibo_g;
render_pass::ptr geom_pass_g;
uint32_t num_indices;


void init() {
    // pass
    geom_pass_g = std::make_shared<render_pass>(vertex_shader::from_file("mesh.vert"), fragment_shader::from_file("mesh.frag"));

    // data
    vao_g = std::make_shared<vertex_array>();
    vao_g->bind();

    Eigen::MatrixXf vbo_data;
    Eigen::Matrix<uint32_t, Eigen::Dynamic, 1> ibo_data;
    mesh_traits<mesh_t>::buffer_data(mesh_g, {POSITION, COLOR, NORMAL}, vbo_data, ibo_data, true);
    num_indices = ibo_data.rows();

    vertex_buffer<float>::layout_t vbo_layout = {{"position", 3}, {"color", 1}, {"normal", 3}};
    vbo_g = vertex_buffer<float>::from_data(vbo_data);
    vbo_g->bind_to_array(vbo_layout, geom_pass_g);

    ibo_g = index_buffer<uint32_t>::from_data(ibo_data);

    vao_g->release();
}

void render(shader_program::ptr program) {
    vao_g->bind();
    ibo_g->bind();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDrawElements(GL_TRIANGLES, num_indices, GL_UNSIGNED_INT, nullptr);
    ibo_g->release();
    vao_g->release();
}

void display(camera::ptr cam) {
    geom_pass_g->set_uniform("modelview_matrix", cam->view_matrix());
    geom_pass_g->render(&render);
}

void reshape(camera::ptr cam) {
    geom_pass_g->set_uniform("projection_matrix", cam->projection_matrix());
}


int main (int argc, char* argv[]) {
    if (!OpenMesh::IO::read_mesh(mesh_g, argv[1])) {
        std::cerr << "Unable to read mesh file. Aborting.\n";
        exit(1);
    }
    // let us not expect materials to be set
    for (auto it = mesh_g.vertices_begin(); it != mesh_g.vertices_end(); ++it) {
        mesh_g.set_color(*it, color_t(1.f, 1.f, 1.f, 1.f));
    }

    auto app = freeglut_application::create<orbit_camera_model>(800, 600, &display, &reshape);
    app->init(argc, argv, "Render Mesh", &init);

    app->run();
}
