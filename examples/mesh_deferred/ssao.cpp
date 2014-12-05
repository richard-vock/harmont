#include "ssao.hpp"

#include <random>
#include <chrono>

namespace harmont {


ssao::ssao(uint32_t variation, uint32_t num_samples, float radius) : variation_(variation), num_samples_(num_samples), radius_(radius) {
}

ssao::~ssao() {
}

void ssao::init(int width, int height) {
    tex_noise_ = texture::texture_2d<unsigned int>(4, 4, 1, nullptr, GL_RED, GL_NEAREST, GL_NEAREST, GL_REPEAT, GL_REPEAT);
    init_samples_();
    init_noise_();
    tex_ssao_ = texture::texture_2d<float>(width, height, 1);
    auto vs_sample = vertex_shader::from_file("full_quad.vert");
    auto fs_sample = fragment_shader::from_file("ssao.frag");
    pass_sample_ = std::make_shared<render_pass_2d>(vs_sample, fs_sample, render_pass::textures({tex_ssao_}));
}

void ssao::reshape(int width, int height) {
    tex_ssao_->resize(width, height);
}

void ssao::compute(texture::ptr gbuffer, camera::ptr cam) {
    pass_sample_->set_uniform("modelview_matrix", cam->view_matrix());
    pass_sample_->set_uniform("projection_matrix", cam->projection_matrix());
    pass_sample_->set_uniform("inv_view_proj_matrix", cam->inverse_view_projection_matrix());
    //pass_sample_->set_uniform("exponent", 0.5);
    pass_sample_->set_uniform("radius", radius_);
    //pass_sample_->set_uniform("near", cam->near());
    //pass_sample_->set_uniform("far", cam->far());
    //pass_sample_->render([&] (shader_program::ptr) {}, {{tex_samples_, "map_samples"}, {tex_noise_, "map_noise"}, {gbuffer, "map_gbuffer"}});
    //pass_sample_->render([&] (shader_program::ptr) {}, {{tex_samples_, "map_samples"}, {gbuffer, "map_gbuffer"}});
    pass_sample_->render([&] (shader_program::ptr) {}, {{gbuffer, "map_gbuffer"}});
    //pass_sample_->render([&] (shader_program::ptr) {}, {{gbuffer, "map_gbuffer"}});
}

texture::ptr ssao::ssao_texture() {
    return tex_ssao_;
}

texture::ptr ssao::sample_texture() {
    return tex_samples_;
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
    radius_ += delta;
    if (radius_ < 0.f) radius_ = 0.f;
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
    std::uniform_int_distribution<int> distribution(0, variation_ - 1);
    Eigen::Matrix<unsigned int, 4, 4> noise;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            noise(i, j) = distribution(generator);
        }
    }
    //tex_noise_->set_data(noise);
}


} // harmont
