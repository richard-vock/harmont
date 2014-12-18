#include <deferred_renderer.hpp>


#ifdef BUILD_DEFERRED_RENDERER

#include <limits>

extern "C" {
#include <rgbe/rgbe.h>
}

namespace harmont {


deferred_renderer::deferred_renderer(const render_parameters_t& render_parameters, const shadow_parameters_t& shadow_parameters, int width, int height) : point_size_(1.f) {
    exposure_ = render_parameters.exposure;
    two_sided_ = render_parameters.two_sided;
    light_dir_ = render_parameters.light_dir;
    background_color_ = render_parameters.background_color;
    shadow_bias_ = render_parameters.shadow_bias;

    shadow_pass_ = std::make_shared<shadow_pass>(
        shadow_parameters.resolution,
        shadow_parameters.sample_count
    );

    ssdo_pass_ = std::make_shared<ssdo>(4, 40, 2.1f);
    ssdo_pass_->init(width, height);

    load_hdr_map_(render_parameters.hdr_map);

    depth_tex_ = texture::depth_texture<float>(width, height);
    gbuffer_tex_ = texture::texture_2d<unsigned int>(width, height, 3);

    vertex_shader::parameters_t params = {{"sample_count", std::to_string(shadow_parameters.sample_count)}, {"shadow_res", std::to_string(shadow_parameters.resolution)}};
    vertex_shader::ptr full_quad_vert = vertex_shader::from_file(std::string(GLSL_PREFIX)+"full_quad.vert");
    vertex_shader::ptr   gbuffer_vert = vertex_shader::from_file(std::string(GLSL_PREFIX)+"gbuffer.vert");
    fragment_shader::ptr clear_frag   = fragment_shader::from_file(std::string(GLSL_PREFIX)+"clear.frag");
    fragment_shader::ptr gbuffer_frag = fragment_shader::from_file(std::string(GLSL_PREFIX)+"gbuffer.frag");
    fragment_shader::ptr compose_frag = fragment_shader::from_file(std::string(GLSL_PREFIX)+"compose.frag", params);

    auto ssdo_clear_textures = ssdo_pass_->clear_textures();
    render_pass::textures clear_textures({gbuffer_tex_});
    clear_textures.insert(clear_textures.end(), ssdo_clear_textures.begin(), ssdo_clear_textures.end());
    clear_pass_ = std::make_shared<render_pass_2d>(full_quad_vert, clear_frag, clear_textures);
    geom_pass_ = std::make_shared<render_pass>(gbuffer_vert, gbuffer_frag, render_pass::textures({gbuffer_tex_}), depth_tex_);
    compose_pass_ = std::make_shared<render_pass_2d>(full_quad_vert, compose_frag);
}

deferred_renderer::~deferred_renderer() {
}

void deferred_renderer::set_light_dir(const Eigen::Vector3f& light_dir) {
    light_dir_ = light_dir;
}

void deferred_renderer::set_background_color(const Eigen::Vector3f& color) {
    background_color_ = color;
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

float deferred_renderer::shadow_bias() const {
    return shadow_bias_;
}

void deferred_renderer::set_shadow_bias(float bias) {
    shadow_bias_ = bias;
}

void deferred_renderer::delta_shadow_bias(float delta) {
    shadow_bias_ += delta;
    if (shadow_bias_ < 0.f) shadow_bias_ = 0.f;
}

bool deferred_renderer::two_sided() const {
    return two_sided_;
}

void deferred_renderer::set_two_sided(bool two_sided) {
    two_sided_ = two_sided;
}

void deferred_renderer::toggle_two_sided() {
    two_sided_ = !!two_sided_;
}

float deferred_renderer::ssdo_radius() const {
    return ssdo_pass_->radius();
}

void deferred_renderer::set_ssdo_radius(float radius) {
    ssdo_pass_->set_radius(radius);
}

void deferred_renderer::delta_ssdo_radius(float delta) {
    ssdo_pass_->delta_radius(delta);
}

float deferred_renderer::ssdo_exponent() const {
    return ssdo_pass_->exponent();
}

void deferred_renderer::set_ssdo_exponent(float exponent) {
    ssdo_pass_->set_exponent(exponent);
}

void deferred_renderer::delta_ssdo_exponent(float delta) {
    ssdo_pass_->delta_exponent(delta);
}

float deferred_renderer::ssdo_reflective_albedo() const {
    return ssdo_pass_->reflective_albedo();
}

void deferred_renderer::set_ssdo_reflective_albedo(float reflective_albedo) {
    ssdo_pass_->set_reflective_albedo(reflective_albedo);
}

void deferred_renderer::delta_ssdo_reflective_albedo(float delta) {
    ssdo_pass_->delta_reflective_albedo(delta);
}

float deferred_renderer::point_size() const {
    return point_size_;
}

void deferred_renderer::set_point_size(float point_size) {
    point_size_ = point_size;
    if (point_size_ < 0.f) point_size_ = 0.f;
}

void deferred_renderer::delta_point_size(float delta) {
    set_point_size(point_size_ + delta);
}

render_pass::ptr deferred_renderer::geometry_pass() {
    return geom_pass_;
}

render_pass::const_ptr deferred_renderer::geometry_pass() const {
    return geom_pass_;
}

renderable::ptr_t deferred_renderer::object(std::string identifier) {
    auto find_it = objects_.find(identifier);
    if (find_it == objects_.end()) throw std::runtime_error("deferred_renderer::object(): Object \""+identifier+"\" does not exist"+SPOT);
    return find_it->second;
}

renderable::const_ptr_t deferred_renderer::object(std::string identifier) const {
    auto find_it = objects_.find(identifier);
    if (find_it == objects_.end()) throw std::runtime_error("deferred_renderer::object(): Object \""+identifier+"\" does not exist"+SPOT);
    return find_it->second;
}

const deferred_renderer::object_map_t& deferred_renderer::objects() const {
    return objects_;
}

void deferred_renderer::add_object(std::string identifier, renderable::ptr_t object) {
    if (objects_.find(identifier) != objects_.end()) {
        throw std::runtime_error("deferred_renderer::add_object(): Trying to add already existing object \""+identifier+"\""+SPOT);
    }

    objects_[identifier] = object;

    vertex_buffer<float>::layout_t vbo_layout;

    // shadow pass
    vbo_layout = {{"position", 3}};
    object->shadow_vertex_array()->bind();
    object->shadow_vertex_buffer()->bind_to_array(vbo_layout, shadow_pass_->program());
    object->shadow_vertex_array()->release();

    // display pass
    vbo_layout = {{"position", 3}, {"color", 1}, {"normal", 3}};
    object->display_vertex_array()->bind();
    object->display_vertex_buffer()->bind_to_array(vbo_layout, geom_pass_->program());
    object->display_vertex_array()->release();
}

void deferred_renderer::remove_object(std::string identifier) {
    auto find_iter = objects_.find(identifier);
    if (find_iter == objects_.end()) {
        throw std::runtime_error("deferred_renderer::remove_object: Trying to remove non-existent object \""+identifier+"\""+SPOT);
    }
    objects_.erase(find_iter);
}

void deferred_renderer::render(camera::ptr cam) {
    if (!objects_.size()) {
        glClearColor(background_color_[0], background_color_[1], background_color_[2], 1.0);
        glClear(GL_COLOR_BUFFER_BIT);
        return;
    }
    glEnable(GL_DEPTH_TEST);

    // compute bounding box
    bbox_ = bbox_t();
    for (const auto& obj : objects_) {
        bbox_.extend(obj.second->bounding_box());
    }
    shadow_pass_->update(bbox_, light_dir_);

    // update near/far values
    float near, far;
    std::tie(near, far) = get_near_far(cam, bbox_);
    cam->set_near_far(near, far);

    geometry_callback_t geom_callback = [&] (shader_program::ptr program, pass_type_t type) { render_geometry(program, type); };

    // update clear pass
    //clear_pass_->set_uniform("far", shadow_pass_->far());

    // update geometry pass
    geom_pass_->set_uniform("projection_matrix", cam->projection_matrix());
    geom_pass_->set_uniform("view_matrix", cam->view_matrix());
    geom_pass_->set_uniform("normal_matrix", cam->view_normal_matrix());
    geom_pass_->set_uniform("two_sided", static_cast<int>(two_sided_));
    float vp_ratio = -2.f * point_size_ * cam->near() * cam->height() / (cam->frustum_height() * 100.f);
    geom_pass_->set_uniform("vp_ratio", vp_ratio);

    // update compose pass
    auto eye = cam->forward().normalized();
    std::vector<float> eye_dir = { eye[0], eye[1], eye[2] };
    std::vector<float> light_dir_vec(light_dir_.data(), light_dir_.data()+3);
    std::vector<float> bg_col_vec(background_color_.data(), background_color_.data()+3);
    compose_pass_->set_uniform("light_dir", light_dir_vec);
    compose_pass_->set_uniform("eye_dir", eye_dir);
    compose_pass_->set_uniform("l_white", 1.f / exposure_ - 1.f);
    compose_pass_->set_uniform("shadow_matrix", shadow_pass_->transform());
    compose_pass_->set_uniform("inv_view_proj_matrix", cam->inverse_view_projection_matrix());
	compose_pass_->set_uniform("shadow_bias", shadow_bias_);
	compose_pass_->set_uniform("background_color", bg_col_vec);
	compose_pass_->set_uniform("poisson_disk[0]", shadow_pass_->poisson_disk());

    clear_pass_->render([&] (shader_program::ptr) { });

    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
    glEnable(GL_PROGRAM_POINT_SIZE);
    shadow_pass_->render(geom_callback, cam->width(), cam->height(), vp_ratio);

    geom_pass_->render([&] (shader_program::ptr program) { geom_callback(program, DISPLAY_GEOMETRY); });
    glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
    glDisable(GL_PROGRAM_POINT_SIZE);

    ssdo_pass_->compute(gbuffer_tex_, diff_tex_, cam, 1);
    compose_pass_->render([&] (shader_program::ptr) { }, {{gbuffer_tex_, "map_gbuffer"}, {shadow_pass_->shadow_texture(), "map_shadow"}, {ssdo_pass_->ssdo_texture(), "map_ssdo"}});
}

void deferred_renderer::reshape(camera::ptr cam) {
    int width = cam->width();
    int height = cam->height();
    geom_pass_->set_uniform("projection_matrix", cam->projection_matrix());
    depth_tex_->resize(width, height);
    gbuffer_tex_->resize(width, height);
    ssdo_pass_->reshape(width, height);
}

void deferred_renderer::render_geometry(shader_program::ptr program, pass_type_t type) {
    for (const auto& obj : objects_) {
        if (type != SHADOW_GEOMETRY || obj.second->casts_shadows()) {
            obj.second->render(program, type, bbox_);
        }
    }
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
    diff_tex_ = texture::texture_2d(width, height, 3, data, GL_RGB, GL_LINEAR, GL_LINEAR);
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

#endif // BUILD_DEFERRED_RENDERER
