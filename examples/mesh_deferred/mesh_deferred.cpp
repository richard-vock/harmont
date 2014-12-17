#include <iostream>

#include <harmont/harmont.hpp>
#include <harmont/openmesh_traits.hpp>
#include <harmont/deferred_renderer.hpp>
#include <boost/lexical_cast.hpp>
using namespace harmont;

typedef OpenMesh::Vec4f      om_color_t;
typedef tri_mesh<om_color_t> mesh_t;
typedef mesh_object<mesh_t>  mesh_obj_t;

freeglut_application::ptr   app_g;
deferred_renderer::ptr_t    renderer_g;
mesh_obj_t::ptr_t           mesh_g;


void init() {
    Eigen::Vector3f light_dir = Eigen::Vector3f(1.f, 1.f, 1.f).normalized();

    deferred_renderer::render_parameters_t r_params {
        light_dir,
        Eigen::Vector3f(0.5f, 0.5f, 0.5f),
        0.8f,
        0.003f,
        true,
        "pisa_diffuse.hdr"
    };
    deferred_renderer::shadow_parameters_t s_params {
        2048,
        32
    };
    renderer_g = std::make_shared<deferred_renderer>(r_params, s_params, app_g->current_camera()->width(), app_g->current_camera()->height());
    mesh_g->init();
    renderer_g->add_object("mesh", mesh_g);
}

void display(camera::ptr cam) {
    renderer_g->render(cam);
}

void reshape(camera::ptr cam) {
    renderer_g->reshape(cam);
}

int main (int argc, char* argv[]) {
    mesh_g = std::make_shared<mesh_obj_t>(argv[1], false);

    app_g = freeglut_application::create<orbit_camera_model>(800, 600, &display, &reshape);
    app_g->init(argc, argv, "Render Mesh", &init);
    app_g->on_drag_left([&] (Eigen::Vector2i pos, Eigen::Vector2i delta) { mesh_g->delta_clipping_height(-delta[1] * 0.01f); app_g->update(); });
    app_g->on_click_left([&] (Eigen::Vector2i pos) { Eigen::Vector3f dir = app_g->current_camera()->forward().normalized(); renderer_g->set_light_dir(dir); app_g->update(); });
    app_g->on_char([&] (unsigned char key) {
        if (key == 'b') renderer_g->delta_shadow_bias(-0.001);
        if (key == 'B') renderer_g->delta_shadow_bias(0.001);
        if (key == 'e') renderer_g->delta_exposure(-0.001f);
        if (key == 'E') renderer_g->delta_exposure(0.001f);
        if (key == 'r') renderer_g->delta_ssdo_radius(-0.01f);
        if (key == 'R') renderer_g->delta_ssdo_radius(0.01f);
        if (key == 'a') renderer_g->delta_ssdo_reflective_albedo(-0.1f);
        if (key == 'A') renderer_g->delta_ssdo_reflective_albedo(0.1f);
        if (key == 'c' || key == 'C') mesh_g->toggle_clipping();
        if (key == 's' || key == 'S') mesh_g->toggle_casts_shadows();
        app_g->update();
    });

    app_g->run();
}
