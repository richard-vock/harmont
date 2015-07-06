#include <ssdo.hpp>

#ifdef BUILD_DEFERRED_RENDERER

#include <random>
#include <chrono>

namespace harmont {


ssdo::ssdo(uint32_t variation, uint32_t num_samples, float radius) : variation_(variation), num_samples_(num_samples), radius_(radius), exponent_(0.8f), first_pass_(true), refl_albedo_(1.0) {
}

ssdo::~ssdo() {
}

void ssdo::init(int width, int height) {
    tex_noise_ = texture::texture_2d<float>(variation_, variation_, 2, nullptr, GL_RG32F, GL_NEAREST, GL_NEAREST, GL_REPEAT, GL_REPEAT);
    init_samples_();
    init_noise_();
    tex_ssdo_ = texture::texture_2d<float>(width, height, 3);

    float* data = new float[width * height * 3];
    std::fill(data, data+(width*height*3), 0.f);
    tex_last_ = texture::texture_2d<float>(width, height, 3, data);
    delete [] data;

    tex_work_ = texture::texture_2d<float>(width, height, 3);
    auto vs_quad = vertex_shader::from_file(std::string(GLSL_PREFIX)+"full_quad.vert");
    auto fs_sample = fragment_shader::from_file(std::string(GLSL_PREFIX)+"ssdo.frag");
    auto fs_blur_h = fragment_shader::from_file(std::string(GLSL_PREFIX)+"blur_h.frag");
    auto fs_blur_v = fragment_shader::from_file(std::string(GLSL_PREFIX)+"blur_v.frag");
    auto fs_gbuffer = fragment_shader::from_file(std::string(GLSL_PREFIX)+"gbuffer.glsl");
    auto fs_projection = fragment_shader::from_file(std::string(GLSL_PREFIX)+"projection.glsl");
    auto fs_util = fragment_shader::from_file(std::string(GLSL_PREFIX)+"utility.glsl");
    auto fs_shading = fragment_shader::from_file(std::string(GLSL_PREFIX)+"shading.glsl");
    pass_sample_ = render_pass_2d::ptr(new render_pass_2d({vs_quad}, {fs_sample, fs_gbuffer, fs_projection, fs_util, fs_shading}, render_pass::textures({tex_ssdo_})));
    pass_blur_h_ = std::make_shared<render_pass_2d>(vs_quad, fs_blur_h, render_pass::textures({tex_work_}));
    pass_blur_v_ = std::make_shared<render_pass_2d>(vs_quad, fs_blur_v, render_pass::textures({tex_ssdo_, tex_last_}));
    pass_blur_h_->set_uniform("dimension", 0);
    pass_blur_v_->set_uniform("dimension", 1);
}

void ssdo::reshape(int width, int height) {
    tex_ssdo_->resize(width, height);
    tex_last_->resize(width, height);
    tex_work_->resize(width, height);
}

void ssdo::compute(texture::ptr gbuffer, texture::ptr env_map, camera::ptr cam, uint32_t num_blur_passes) {
    pass_sample_->set_uniform("view_matrix", cam->view_matrix());
    pass_sample_->set_uniform("projection_matrix", cam->projection_matrix());
    pass_sample_->set_uniform("inv_view_proj_matrix", cam->inverse_view_projection_matrix());
    pass_sample_->set_uniform("exponent", exponent_);
    pass_sample_->set_uniform("radius", radius_);
    pass_sample_->set_uniform("near", cam->near());
    pass_sample_->set_uniform("far", cam->far());
    pass_sample_->set_uniform("reflective_albedo", refl_albedo_);
    pass_sample_->set_uniform("is_first_frame", first_pass_ ? 1 : 0);
    pass_sample_->render([&] (shader_program::ptr) {}, {{tex_samples_, "map_samples"}, {tex_noise_, "map_noise"}, {gbuffer, "map_gbuffer"}, {env_map, "map_env"}, {tex_last_, "map_last"}});
    //pass_sample_->render([&] (shader_program::ptr) {}, {{tex_samples_, "map_samples"}, {tex_noise_, "map_noise"}, {gbuffer, "map_gbuffer"}, {env_map, "map_env"}});
    //pass_sample_->render([&] (shader_program::ptr) {}, {{tex_samples_, "map_samples"}, {gbuffer, "map_gbuffer"}});

    // blur
    for (uint32_t i = 0; i < num_blur_passes; ++i) {
        if (i == 0) {
            int width = cam->width();
            int height = cam->height();
            pass_blur_h_->set_uniform("width", width);
            pass_blur_h_->set_uniform("height", height);
            pass_blur_v_->set_uniform("width", width);
            pass_blur_v_->set_uniform("height", height);
        }
        pass_blur_h_->render([&] (shader_program::ptr) {}, {{tex_ssdo_, "map_input"}, {gbuffer, "map_gbuffer"}});
        pass_blur_v_->render([&] (shader_program::ptr) {}, {{tex_work_, "map_input"}, {gbuffer, "map_gbuffer"}});
    }

    first_pass_ = false;
}

texture::ptr ssdo::ssdo_texture() {
    return tex_ssdo_;
}

texture::ptr ssdo::sample_texture() {
    return tex_samples_;
}

texture::ptr ssdo::noise_texture() {
    return tex_noise_;
}

render_pass::textures ssdo::clear_textures() {
    return render_pass::textures({tex_ssdo_, tex_last_});
}

uint32_t ssdo::variation() const {
    return variation_;
}

void ssdo::set_variation(uint32_t variation) {
    variation_ = variation;
    if (variation_ < 1) variation_ = 1;
    init_samples_();
    init_noise_();
}

uint32_t ssdo::num_samples() const {
    return num_samples_;
}

void ssdo::set_num_samples(uint32_t num_samples) {
    num_samples_ = num_samples;
    if (num_samples_ < 1) num_samples_ = 1;
    init_samples_();
}

float ssdo::radius() const {
    return radius_;
}

void ssdo::set_radius(float radius) {
    radius_ = radius;
    if (radius_ < 0.f) radius_ = 0.f;
}

void ssdo::delta_radius(float delta) {
    set_radius(radius_ + delta);
}

float ssdo::exponent() const {
    return exponent_;
}

void ssdo::set_exponent(float exponent) {
    exponent_ = exponent;
    if (exponent_ < 0.01f) exponent_ = 0.01f;
}

void ssdo::delta_exponent(float delta) {
    set_exponent(exponent_ + delta);
}

float ssdo::reflective_albedo() const {
    return refl_albedo_;
}

void ssdo::set_reflective_albedo(float reflective_albedo) {
    refl_albedo_ = reflective_albedo;
    if (refl_albedo_ < 0.f) refl_albedo_ = 0.f;
    if (refl_albedo_ > 1.f) refl_albedo_ = 1.f;
}

void ssdo::delta_reflective_albedo(float delta) {
    set_reflective_albedo(refl_albedo_ + delta);
}

void ssdo::init_samples_() {
    typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> mat_t;
    mat_t data = 0.5f * (mat_t::Random(num_samples_, 4) + mat_t::Ones(num_samples_, 4));
    float factor = 1.f / (sqrt(2.f * M_PI));
    for (uint32_t i = 0; i < num_samples_; ++i) {
        data(i, 3) = fabs(factor * exp(-0.5 * data(i, 3) * data(i,3)));
    }
    tex_samples_ = texture::texture_1d<float>(num_samples_, 4, data.data(), GL_RGBA32F);

    //float* samples = new float[variation_ * num_samples_ * 3];
    //for (uint32_t i = 0; i < variation_; ++i) {
        //for (uint32_t j = 0; j < num_samples_; ++j) {
            //Eigen::Vector3f sample = Eigen::Vector3f::Random();
            //sample[2] = 0.5f * (sample[2] + 1.f);
            //sample = sample.normalized();
            //samples[i*num_samples_*3 + j*3 + 0] = sample[0];
            //samples[i*num_samples_*3 + j*3 + 1] = sample[1];
            //samples[i*num_samples_*3 + j*3 + 2] = sample[2];
        //}
    //}
    //tex_samples_->set_data(samples);
    //delete [] samples;
}

void ssdo::init_noise_() {
    unsigned long seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<float> distribution(0, 1);
    float* data = new float[variation_ * variation_ * 2];
    for (uint32_t i = 0; i < variation_; ++i) {
        for (uint32_t j = 0; j < variation_; ++j) {
            data[i*variation_*2 + j*2 + 0] = distribution(generator);
            data[i*variation_*2 + j*2 + 1] = distribution(generator);
            data[i*variation_*2 + j*2 + 2] = distribution(generator);
        }
    }
    tex_noise_->set_data(data);
    delete [] data;
}


} // harmont

#endif // BUILD_DEFERRED_RENDERER
