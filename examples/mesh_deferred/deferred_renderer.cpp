#include "deferred_renderer.hpp"

#include <limits>

extern "C" {
#include "rgbe.h"
}

namespace harmont {


deferred_renderer::deferred_renderer(const render_parameters_t& render_parameters, const shadow_parameters_t& shadow_parameters, const bounding_box_t& bbox, int width, int height) {
    exposure_ = render_parameters.exposure;
    light_dir_ = render_parameters.light_dir;

    //shadow_pass_ = std::make_shared<shadow_pass>(
        //shadow_parameters.resolution,
        //shadow_parameters.sample_count,
        //shadow_parameters.vertex_shader,
        //shadow_parameters.fragment_shader
    //);
    //shadow_pass_->update(bbox, light_dir_);

    load_hdr_map_(render_parameters.hdr_map);

    //vertex_shader::parameters_t params = {{"sample_count", std::to_string(shadow_parameters.sample_count)}, {"shadow_res", std::to_string(shadow_parameters.resolution)}};
    //vertex_shader::ptr vs = vertex_shader::from_file(render_parameters.vertex_shader, params);
    //fragment_shader::ptr fs = fragment_shader::from_file(render_parameters.fragment_shader, params);
    //geom_pass_ = std::make_shared<render_pass>(vs, fs);
    //geom_pass_->set_uniform("poisson_disk[0]", shadow_pass_->poisson_disk());
    //std::vector<float> light_dir_vec(light_dir_.data(), light_dir_.data()+3);
    //geom_pass_->set_uniform("light_dir", light_dir_vec);
    //geom_pass_->set_uniform("two_sided", 1);

    depth_tex_ = texture::depth_texture<float>(width, height);
    gbuffer_tex_ = texture::texture_2d<float>(width, height, 3);

    vertex_shader::ptr full_quad_vert = vertex_shader::from_file("full_quad.vert");
    vertex_shader::ptr   gbuffer_vert = vertex_shader::from_file("gbuffer.vert");
    fragment_shader::ptr clear_frag   = fragment_shader::from_file("clear.frag");
    fragment_shader::ptr gbuffer_frag = fragment_shader::from_file("gbuffer.frag");
    fragment_shader::ptr debug_gbuffer_frag = fragment_shader::from_file("debug_gbuffer.frag");

    clear_pass_ = std::make_shared<render_pass_2d>(full_quad_vert, clear_frag, render_pass::textures({gbuffer_tex_}));
    geom_pass_ = std::make_shared<render_pass>(gbuffer_vert, gbuffer_frag, render_pass::textures({gbuffer_tex_}), depth_tex_);
    debug_pass_ = std::make_shared<render_pass_2d>(full_quad_vert, debug_gbuffer_frag);
}

deferred_renderer::~deferred_renderer() {
}

void deferred_renderer::set_light_dir(const Eigen::Vector3f& light_dir, const bounding_box_t& bbox) {
    light_dir_ = light_dir;
    //shadow_pass_->update(bbox, light_dir_);
}

float deferred_renderer::exposure() const {
    return exposure_;
}

void deferred_renderer::set_exposure(float exposure) {
    exposure_ = exposure;
    if (exposure_ < 0.0001f) exposure_ = 0.0001f;
    if (exposure_ > 1.f) exposure_ = 1.f;
}

void deferred_renderer::delta_exposure(float delta) {
    set_exposure(exposure_ + delta);
}

render_pass::ptr deferred_renderer::geometry_pass() {
    return geom_pass_;
}

render_pass::const_ptr deferred_renderer::geometry_pass() const {
    return geom_pass_;
}

void deferred_renderer::render(const render_callback_t& render_callback, camera::ptr cam, const bounding_box_t& bbox) {
    glEnable(GL_DEPTH_TEST);

    // update near/far values
    float near, far;
    std::tie(near, far) = get_near_far(cam, bbox);
    cam->set_near_far(near, far);

    // render shadow texture
    //shadow_pass_->render(render_callback, cam->width(), cam->height());

    // setup geometry pass variables
    //auto eye = cam->forward().normalized();
    //std::vector<float> eye_dir = { eye[0], eye[1], eye[2] };
    geom_pass_->set_uniform("projection_matrix", cam->projection_matrix());
    geom_pass_->set_uniform("modelview_matrix", cam->view_matrix());
    //geom_pass_->set_uniform("shadow_matrix", shadow_pass_->transform());
    std::vector<float> light_dir_vec(light_dir_.data(), light_dir_.data()+3);
    //geom_pass_->set_uniform("light_dir", light_dir_vec);
    //geom_pass_->set_uniform("l_white", 1.f / exposure_ - 1.f);
    //geom_pass_->set_uniform("eye_dir", eye_dir);

    // render geometry pass
    //geom_pass_->render(render_callback, {{diff_tex_, "map_diffuse"}, {shadow_pass_->shadow_texture(), "map_shadow"}});
    clear_pass_->render([&] (shader_program::ptr) { });
    geom_pass_->render(render_callback);
    debug_pass_->render([&] (shader_program::ptr) { }, {{gbuffer_tex_, "map_gbuffer"}});
    //debug_pass_->render([&] (shader_program::ptr) { });
}

void deferred_renderer::reshape(camera::ptr cam) {
    geom_pass_->set_uniform("projection_matrix", cam->projection_matrix());
    depth_tex_->resize(cam->width(), cam->height());
    gbuffer_tex_->resize(cam->width(), cam->height());
}

void deferred_renderer::load_hdr_map_(std::string filename) {
	FILE *f;

	f = fopen(filename.c_str(), "rb");
	if (f == NULL) {
        std::cout << "Cannot open hdr file \"" << filename << "\" for reading." << "\n";
        exit(1);
	}

	rgbe_header_info header;
    int width, height;
	RGBE_ReadHeader(f, &width, &height, &header);
    float* data = new float[width*height*3];
	RGBE_ReadPixels_RLE(f, data, width, height);
	fclose(f);
    diff_tex_ = texture::texture_2d(width, height, 3, data, GL_LINEAR, GL_LINEAR);
    delete [] data;
}

std::pair<float, float> deferred_renderer::get_near_far(camera::const_ptr cam, const bounding_box_t& bbox) {
    Eigen::Vector3f bb_min = bbox.min(), bb_max = bbox.max();
    float near = std::numeric_limits<float>::max(), far = 0.f;
    Eigen::Matrix4f vm = cam->view_matrix();
    for (uint32_t c=0; c<8; ++c) {
        // compute corner
        Eigen::Vector3f corner;
        for (uint32_t i = 0; i < 3; ++i) {
            corner[i] = (c & 1 << i) ? bb_max[i] : bb_min[i];
        }
        // compute depth
        Eigen::Vector3f proj = (vm * corner.homogeneous()).head(3);
        float depth = fabs(proj[2]);
        if (depth < near) near = depth;
        if (depth > far)  far = depth;
    }
    near = std::max(near, 0.01f);

    return {near, far};
}


} // harmont
