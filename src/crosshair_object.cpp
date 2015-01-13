#include <crosshair_object.hpp>

namespace harmont {


crosshair_object::crosshair_object(const Eigen::Vector3f& vertex, Eigen::Vector4f color, float size, float line_width, bool casts_shadows) : crosshair_object(vertices_t(1, vertex), color, size, line_width, casts_shadows) {
}

crosshair_object::crosshair_object(const vertices_t& vertices, Eigen::Vector4f color, float size, float line_width, bool casts_shadows) : crosshair_object(vertices, colors_t(vertices.size(), color), size, line_width, casts_shadows) {
}

crosshair_object::crosshair_object(const vertices_t& vertices, const colors_t& colors, float size, float line_width, bool casts_shadows) : renderable(casts_shadows), vertices_(vertices), colors_(colors), size_(size), line_width_(line_width) {
    if (colors.size() != vertices.size()) {
        throw std::runtime_error("crosshair_object::crosshair_object(): Number of colors must equal number of vertices"+SPOT);
    }
}

crosshair_object::~crosshair_object() {
}

void crosshair_object::compute_vertex_data() {
    std::vector<float> cols(colors_.size());
    std::transform(colors_.begin(), colors_.end(), cols.begin(), [&] (const Eigen::Vector4f& c) { return renderable::color_to_rgba(c); });

    vertex_data_ = vertex_data_t::Zero(6 * cols.size(), 9);
    index_data_ = index_data_t(6 * cols.size(), 1);
    for (uint32_t i = 0; i < cols.size(); ++i) {
        Eigen::RowVector3f origin = vertices_[i].transpose();
        vertex_data_.block(i*6 + 0, 0, 1, 3) = origin - size_ * Eigen::RowVector3f::UnitX();
        vertex_data_.block(i*6 + 1, 0, 1, 3) = origin + size_ * Eigen::RowVector3f::UnitX();
        vertex_data_.block(i*6 + 2, 0, 1, 3) = origin - size_ * Eigen::RowVector3f::UnitY();
        vertex_data_.block(i*6 + 3, 0, 1, 3) = origin + size_ * Eigen::RowVector3f::UnitY();
        vertex_data_.block(i*6 + 4, 0, 1, 3) = origin - size_ * Eigen::RowVector3f::UnitZ();
        vertex_data_.block(i*6 + 5, 0, 1, 3) = origin + size_ * Eigen::RowVector3f::UnitZ();
        for (uint32_t j = 0; j < 6; ++j) {
            index_data_(i*6 + j, 0) = i*6 + j;
            vertex_data_(i*6 + j, 3) = cols[i];
        }
    }
    compute_bounding_box_();
}

float crosshair_object::size() const {
    return size_;
}

void crosshair_object::set_size(const float& size) {
    size_ = size;
    compute_vertex_data();
    update_geometry(vertex_data_);
}

float crosshair_object::line_width() const {
    return line_width_;
}

void crosshair_object::set_line_width(const float& line_width) {
    line_width_ = line_width;
}

void crosshair_object::pre_render(shader_program::ptr program, pass_type_t type) {
    glEnable(GL_LINE_SMOOTH);
    glLineWidth(line_width_);
}

void crosshair_object::post_render(shader_program::ptr program, pass_type_t type) {
    glLineWidth(1.f);
    glDisable(GL_LINE_SMOOTH);
}

typename crosshair_object::element_type_t crosshair_object::element_type() const {
    return VERTS;
}

bool crosshair_object::transparent() const {
    return false;
}

void crosshair_object::compute_bounding_box_() {
    bbox_ = bbox_t();
    for (uint32_t i = 0; i < vertices_.size(); ++i) {
        bbox_.extend(vertices_[i]);
    }
    bbox_.extend(bbox_.max() + size_ * Eigen::Vector3f::Ones());
    bbox_.extend(bbox_.min() - size_ * Eigen::Vector3f::Ones());
}

GLenum crosshair_object::gl_element_mode_() const {
    return GL_LINES;
}


} // harmont
