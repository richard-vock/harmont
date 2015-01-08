#include <renderable.hpp>

namespace harmont {

renderable::renderable(bool casts_shadows) : active_(true), transparent_(false), bbox_valid_(false), casts_shadows_(casts_shadows), clipping_(false), clipping_height_(0.5f), clipping_normal_(0.f, 0.f, 1.f), invert_clipping_(false), num_elements_(0), transform_(transformation_t::Identity()) {
}

renderable::~renderable() {
}

void renderable::init(const vertex_data_t& vertex_data, const index_data_t& index_data) {
    shadow_array_ = std::make_shared<vertex_array>();
    display_array_ = std::make_shared<vertex_array>();
    vertex_data_t pos_data = vertex_data.block(0, 0, vertex_data.rows(), 3);
    vertex_data_t col_data = vertex_data.col(3);
    transparent_ = is_transparent(col_data);
    initial_color_data_ = vertex_data.block(0, 3, vertex_data.rows(), 1);
    shadow_buffer_ = vbo_t::from_data(pos_data);
    display_buffer_ = vbo_t::from_data(vertex_data);
    index_buffer_ = ibo_t::from_data(index_data);
    num_elements_ = index_data.rows();
}

void renderable::update_geometry(const vertex_data_t& vertex_data) {
    if (vertex_data.rows() != num_elements_) throw std::runtime_error("renderable::update_geometry(): Vertex data size does not match prior element count"+SPOT);
    vertex_data_t pos_data = vertex_data.block(0, 0, vertex_data.rows(), 3);
    initial_color_data_ = vertex_data.block(0, 3, vertex_data.rows(), 1);
    shadow_buffer_->set_data(vertex_data);
    display_buffer_->set_data(vertex_data);
}

void renderable::render(shader_program::ptr program, pass_type_t type, const bbox_t& bbox) {
    if (!initialized()) return;

    pre_render(program, type);

    if (clipping_) {
        float min_z = bbox_.min()[2];
        float max_z = bbox_.max()[2];
        float clip_z = min_z + clipping_height_ * (max_z - min_z);
        float factor = invert_clipping_ ? -1.f : 1.f;
        Eigen::Vector3f clip_n = factor * clipping_normal_;
        ((*program)["clip_normal"]).set(std::vector<float>(clip_n.data(), clip_n.data()+3));
        ((*program)["clip_distance"]).set(factor * clip_z);
        glEnable(GL_CLIP_DISTANCE0);
    }

    if (type == DISPLAY_GEOMETRY) {
        ((*program)["has_texture"]).set(static_cast<int>(static_cast<bool>(tex_)));
        if (tex_) {
            auto location = ((*program)["map_tex"]).location();
            glUniform1i(location, 0);
            glActiveTexture(GL_TEXTURE0);
            tex_->bind();
        }
    }

    ((*program))["model_matrix"].set(transform_);

    if (type == SHADOW_GEOMETRY) {
        shadow_array_->bind();
    } else {
        display_array_->bind();
    }
    index_buffer_->bind();

    glDrawElements(gl_element_mode_(), num_elements_, GL_UNSIGNED_INT, nullptr);

    if (tex_) {
        tex_->release();
    }

    if (type == SHADOW_GEOMETRY) {
        shadow_array_->release();
    } else {
        display_array_->release();
    }

    if (clipping_) {
        glDisable(GL_CLIP_DISTANCE0);
    }

    post_render(program, type);
}

void renderable::pre_render(shader_program::ptr program, pass_type_t type) {
}

void renderable::post_render(shader_program::ptr program, pass_type_t type) {
}

void renderable::set_colors(const std::vector<uint32_t>& indices, const std::vector<color_t>& colors) {
    if (indices.size() != colors.size()) throw std::runtime_error("renderable::set_colors: Index count must match colors count"+SPOT);
    vertex_data_t color_data = initial_color_data_;
    for (uint32_t i = 0; i < indices.size(); ++i) {
        uint32_t idx = indices[i];
        if (idx < color_data.rows()) color_data(idx, 0) = color_to_rgba(colors[i]);
    }
    set_color_data_(color_data);
}

void renderable::set_colors(const std::vector<uint32_t>& indices, const color_t& color) {
    std::vector<color_t> color_data(indices.size(), color);
    set_colors(indices, color_data);
}

void renderable::set_colors(const std::vector<color_t>& colors) {
    if (colors.size() != num_elements_) throw std::runtime_error("renderable::set_colors: Number of colors must match number of elements"+SPOT);
    std::vector<uint32_t> indices(colors.size());
    std::iota(indices.begin(), indices.end(), 0);
    set_colors(indices, colors);
}

void renderable::set_colors(const color_t& color) {
    std::vector<color_t> colors(num_elements_, color);
    set_colors(colors);
}

void renderable::reset_colors() {
    set_color_data_(initial_color_data_);
}

bool renderable::transparent() const {
    return transparent_;
}

bool renderable::initialized() const {
    return num_elements_ > 0;
}

uint32_t renderable::num_elements() const {
    return num_elements_;
}

renderable::vbo_t::ptr renderable::shadow_vertex_buffer() {
    return shadow_buffer_;
}

renderable::vbo_t::const_ptr renderable::shadow_vertex_buffer() const {
    return shadow_buffer_;
}

renderable::vbo_t::ptr renderable::display_vertex_buffer() {
    return display_buffer_;
}

renderable::vbo_t::const_ptr renderable::display_vertex_buffer() const {
    return display_buffer_;
}

renderable::vao_t::ptr renderable::shadow_vertex_array() {
    return shadow_array_;
}

renderable::vao_t::const_ptr renderable::shadow_vertex_array() const {
    return shadow_array_;
}

renderable::vao_t::ptr renderable::display_vertex_array() {
    return display_array_;
}

renderable::vao_t::const_ptr renderable::display_vertex_array() const {
    return display_array_;
}

renderable::ibo_t::ptr renderable::element_index_buffer() {
    return index_buffer_;
}

renderable::ibo_t::const_ptr renderable::element_index_buffer() const {
    return index_buffer_;
}

bool renderable::active() const {
    return active_;
}

void renderable::set_active(const bool& active) {
    active_ = active;
}

void renderable::toggle_active() {
    active_ = !active_;
}

bool renderable::casts_shadows() const {
    return casts_shadows_;
}

void renderable::set_casts_shadows(bool casts_shadows) {
    casts_shadows_ = casts_shadows;
}

void renderable::toggle_casts_shadows() {
    casts_shadows_ = !casts_shadows_;
}

bool renderable::clipping() const {
    return clipping_;
}

void renderable::set_clipping(bool clipping) {
    clipping_ = clipping;
}

void renderable::toggle_clipping() {
    clipping_ = !clipping_;
}

float renderable::clipping_height() const {
    return clipping_height_;
}

void renderable::set_clipping_height(float height) {
    clipping_height_ = height;
    clamp(clipping_height_, 0.f, 1.f);
}

void renderable::delta_clipping_height(float delta) {
    set_clipping_height(clipping_height_ + delta);
}

const Eigen::Vector3f& renderable::clipping_normal() const {
    return clipping_normal_;
}

void renderable::set_clipping_normal(const Eigen::Vector3f& clipping_normal) {
    clipping_normal_ = clipping_normal;
}

bool renderable::invert_clipping() const {
    return invert_clipping_;
}

void renderable::set_invert_clipping(const bool& invert_clipping) {
    invert_clipping_ = invert_clipping;
}

void renderable::toggle_invert_clipping() {
    invert_clipping_ = !invert_clipping_;
}

const renderable::transformation_t& renderable::transformation() const {
    return transform_;
}

void renderable::set_transformation(const transformation_t& transformation) {
    transform_ = transformation;
    bbox_valid_ = false;
}

void renderable::move(const transformation_t& transformation) {
    set_transformation(transformation * transform_);
}

void renderable::set_texture(texture::ptr tex) {
    tex_ = tex;
}

void renderable::unset_texture() {
    tex_.reset();
}

bbox_t renderable::bounding_box() {
    if (!bbox_valid_) {
        compute_bounding_box_();
        bbox_valid_ = true;
    }
    return bbox_;
}

float renderable::color_to_rgba(Eigen::Vector4f col) {
    renderable::color_t clamped = col;
    clamp(clamped, 0.f, 1.f);
    internal_color_t ic;
    ic.r = static_cast<uint8_t>(col[0] * 255.f);
    ic.g = static_cast<uint8_t>(col[1] * 255.f);
    ic.b = static_cast<uint8_t>(col[2] * 255.f);
    ic.a = static_cast<uint8_t>(col[3] * 255.f);
    return ic.rgba;
}

float renderable::alpha_from_rgba(float rgba) {
    internal_color_t ic;
    ic.rgba = rgba;
    return static_cast<float>(ic.a) / 255.f;
}

void renderable::set_color_data_(const vertex_data_t& color_data) {
    auto mapped = display_buffer_->eigen_map<Eigen::RowMajor>(color_data.rows(), 9);
    mapped.col(3) = color_data;
    display_buffer_->unmap();
    transparent_ = is_transparent(color_data);
}

bool renderable::is_transparent(const vertex_data_t& color_data) {
    bool transparent = false;
    for (uint32_t i = 0; i < color_data.rows(); ++i) {
        if (alpha_from_rgba(color_data(i,0)) < 1.f) {
            transparent = true;
            break;
        }
    }
    return transparent;
}

} // harmont
