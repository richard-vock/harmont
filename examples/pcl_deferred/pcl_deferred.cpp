#include <iostream>

#include <harmont/harmont.hpp>
#include <harmont/pcl_traits.hpp>
#include <harmont/deferred_renderer.hpp>
#include <boost/lexical_cast.hpp>
using namespace harmont;

typedef pcl::PointXYZRGBNormal point_t;
typedef cloud<point_t>         cloud_t;
typedef pointcloud_object<cloud_t> cloud_obj_t;

freeglut_application::ptr   app_g;
deferred_renderer::ptr_t    renderer_g;
cloud_obj_t::ptr_t          cloud_g;



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
    cloud_g->init();
    renderer_g->add_object("cloud", cloud_g);
}

void display(camera::ptr cam) {
    renderer_g->render(cam);
}

void reshape(camera::ptr cam) {
    renderer_g->reshape(cam);
}

int main (int argc, char* argv[]) {
    cloud_g = std::make_shared<cloud_obj_t>(argv[1]);

    app_g = freeglut_application::create<orbit_camera_model>(800, 600, &display, &reshape);
    app_g->init(argc, argv, "Render Mesh", &init);
    app_g->on_drag_left([&] (Eigen::Vector2i pos, Eigen::Vector2i delta) { cloud_g->delta_clipping_height(-delta[1] * 0.01f); app_g->update(); });
    app_g->on_click_left([&] (Eigen::Vector2i pos) { Eigen::Vector3f dir = app_g->current_camera()->forward().normalized(); renderer_g->set_light_dir(dir); app_g->update(); });
    app_g->on_char([&] (unsigned char key) {
        if (key == 'b') renderer_g->delta_shadow_bias(-0.001);
        if (key == 'B') renderer_g->delta_shadow_bias(0.001);
        if (key == 'e') renderer_g->delta_exposure(-0.001f);
        if (key == 'E') renderer_g->delta_exposure(0.001f);
        if (key == 'r') renderer_g->delta_ssdo_radius(-0.1f);
        if (key == 'R') renderer_g->delta_ssdo_radius(0.1f);
        if (key == 's') renderer_g->delta_ssdo_exponent(-0.1f);
        if (key == 'S') renderer_g->delta_ssdo_exponent(0.1f);
        if (key == 'a') renderer_g->delta_ssdo_reflective_albedo(-0.1f);
        if (key == 'A') renderer_g->delta_ssdo_reflective_albedo(0.1f);
        if (key == '-') renderer_g->delta_point_size(-0.01f);
        if (key == '+') renderer_g->delta_point_size(0.01f);
        if (key == 'c' || key == 'C') cloud_g->toggle_clipping();
        app_g->update();
    });

    app_g->run();
}
