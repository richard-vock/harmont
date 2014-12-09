#include "ssao.hpp"

#include <random>
#include <chrono>

namespace harmont {


ssao::ssao(uint32_t variation, uint32_t num_samples, float radius) : variation_(variation), num_samples_(num_samples), radius_(radius), exponent_(1.f) {
}

ssao::~ssao() {
}

void ssao::init(int width, int height) {
    tex_noise_ = texture::texture_2d<float>(variation_, variation_, 2, nullptr, GL_RG32F, GL_NEAREST, GL_NEAREST, GL_REPEAT, GL_REPEAT);
    init_samples_();
    init_noise_();
    tex_ssao_ = texture::texture_2d<float>(width, height, 1);
    tex_work_ = texture::texture_2d<float>(width, height, 1);
    auto vs_quad = vertex_shader::from_file("full_quad.vert");
    auto fs_sample = fragment_shader::from_file("ssao.frag");
    auto fs_blur = fragment_shader::from_file("blur.frag");
    pass_sample_ = std::make_shared<render_pass_2d>(vs_quad, fs_sample, render_pass::textures({tex_ssao_}));
    pass_blur_h_ = std::make_shared<render_pass_2d>(vs_quad, fs_blur, render_pass::textures({tex_work_}));
    pass_blur_v_ = std::make_shared<render_pass_2d>(vs_quad, fs_blur, render_pass::textures({tex_ssao_}));
    pass_blur_h_->set_uniform("dimension", 0);
    pass_blur_v_->set_uniform("dimension", 1);
}

void ssao::reshape(int width, int height) {
    tex_ssao_->resize(width, height);
    tex_work_->resize(width, height);
}

void ssao::compute(texture::ptr gbuffer, camera::ptr cam, uint32_t num_blur_passes) {
    pass_sample_->set_uniform("modelview_matrix", cam->view_matrix());
    pass_sample_->set_uniform("projection_matrix", cam->projection_matrix());
    pass_sample_->set_uniform("inv_view_proj_matrix", cam->inverse_view_projection_matrix());
    pass_sample_->set_uniform("exponent", exponent_);
    pass_sample_->set_uniform("radius", radius_);
    pass_sample_->set_uniform("near", cam->near());
    pass_sample_->set_uniform("far", cam->far());
    pass_sample_->render([&] (shader_program::ptr) {}, {{tex_samples_, "map_samples"}, {tex_noise_, "map_noise"}, {gbuffer, "map_gbuffer"}});
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
        pass_blur_h_->render([&] (shader_program::ptr) {}, {{tex_ssao_, "map_input"}, {gbuffer, "map_gbuffer"}});
        pass_blur_v_->render([&] (shader_program::ptr) {}, {{tex_work_, "map_input"}, {gbuffer, "map_gbuffer"}});
    }
}

texture::ptr ssao::ssao_texture() {
    return tex_ssao_;
}

texture::ptr ssao::sample_texture() {
    return tex_samples_;
}

texture::ptr ssao::noise_texture() {
    return tex_noise_;
}

uint32_t ssao::variation() const {
    return variation_;
}

void ssao::set_variation(uint32_t variation) {
    variation_ = variation;
    if (variation_ < 1) variation_ = 1;
    init_samples_();
    init_noise_();
}

uint32_t ssao::num_samples() const {
    return num_samples_;
}

void ssao::set_num_samples(uint32_t num_samples) {
    num_samples_ = num_samples;
    if (num_samples_ < 1) num_samples_ = 1;
    init_samples_();
}

float ssao::radius() const {
    return radius_;
}

void ssao::set_radius(float radius) {
    radius_ = radius;
    if (radius_ < 0.f) radius_ = 0.f;
}

void ssao::delta_radius(float delta) {
    set_radius(radius_ + delta);
}

float ssao::exponent() const {
    return exponent_;
}

void ssao::set_exponent(float exponent) {
    exponent_ = exponent;
    if (exponent_ < 0.01f) exponent_ = 0.01f;
}

void ssao::delta_exponent(float delta) {
    set_exponent(exponent_ + delta);
}

void ssao::init_samples_() {
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

void ssao::init_noise_() {
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
