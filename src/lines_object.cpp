#include <lines_object.hpp>

namespace harmont {


lines_object::lines_object(const vertices_t& vertices, Eigen::Vector4f color, float line_width, mode_t mode, bool casts_shadows) : lines_object(vertices, colors_t(1, color), line_width, mode, casts_shadows) {
}

lines_object::lines_object(const vertices_t& vertices, const colors_t& colors, float line_width, mode_t mode, bool casts_shadows) : renderable(casts_shadows), vertices_(vertices), line_width_(line_width), mode_(mode) {
    if (colors.size() != vertices.size()) {
        throw std::runtime_error("lines_object::lines_object(): Number of colors must equal number of vertices"+SPOT);
    }
    compute_geometry_(colors);
}

lines_object::~lines_object() {
}

float lines_object::line_width() const {
    return line_width_;
}

void lines_object::set_line_width(const float& line_width) {
    line_width_ = line_width;
}

void lines_object::init() {
    renderable::init(vertex_data_, index_data_);
}

void lines_object::pre_render(shader_program::ptr program, pass_type_t type) {
    glEnable(GL_LINE_SMOOTH);
    glLineWidth(line_width_);
}

void lines_object::post_render(shader_program::ptr program, pass_type_t type) {
    glLineWidth(1.f);
    glDisable(GL_LINE_SMOOTH);
}

typename lines_object::element_type_t lines_object::element_type() const {
    return VERTS;
}

bool lines_object::transparent() const {
    return false;
}

void lines_object::compute_geometry_(const colors_t& colors) {
    std::vector<float> cols(colors.size());
    std::transform(colors.begin(), colors.end(), cols.begin(), [&] (const Eigen::Vector4f& c) { return renderable::color_to_rgba(c); });

    vertex_data_ = vertex_data_t::Zero(cols.size(), 9);
    index_data_ = index_data_t(cols.size(), 1);
    for (uint32_t i = 0; i < cols.size(); ++i) {
        vertex_data_.block(i, 0, 1, 3) = vertices_[i].transpose();
        //vertex_data_.block(i, 4, 1, 3) = Eigen::RowVector3f::UnitZ();
        vertex_data_(i, 3) = cols[i];
        index_data_(i, 0) = i;
    }
}

void lines_object::compute_bounding_box_() {
    bbox_ = bbox_t();
    for (uint32_t i = 0; i < vertices_.size(); ++i) {
        bbox_.extend(vertices_[i]);
    }
}

GLenum lines_object::gl_element_mode_() const {
    return mode_ == SEPARATE ? GL_LINES : GL_LINE_STRIP;
}


} // harmont
