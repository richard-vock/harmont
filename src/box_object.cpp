#include <box_object.hpp>

namespace harmont {

box_object::box_object(const vertices_t& vertices, Eigen::Vector4f color, bool as_lines, float line_width, bool casts_shadows) : renderable(casts_shadows), as_lines_(as_lines), line_width_(line_width), colors_(1, color) {
    vertices_ = vertices;
    setup_from_corners_();
}

box_object::box_object(const Eigen::Vector3f& min, const Eigen::Vector3f max, Eigen::Vector4f color, bool as_lines, float line_width, bool casts_shadows) : renderable(casts_shadows), as_lines_(as_lines), line_width_(line_width), colors_(1, color) {
    vertices_.resize(8);

    vertices_[0] = Eigen::Vector3f(min[0], min[1], min[2]);
    vertices_[1] = Eigen::Vector3f(max[0], min[1], min[2]);
    vertices_[2] = Eigen::Vector3f(max[0], min[1], max[2]);
    vertices_[3] = Eigen::Vector3f(min[0], min[1], max[2]);

    vertices_[4] = Eigen::Vector3f(min[0], max[1], min[2]);
    vertices_[5] = Eigen::Vector3f(max[0], max[1], min[2]);
    vertices_[6] = Eigen::Vector3f(max[0], max[1], max[2]);
    vertices_[7] = Eigen::Vector3f(min[0], max[1], max[2]);

    setup_from_corners_();
}

box_object::box_object(const Eigen::Matrix3f& base, const Eigen::Vector3f& size, const Eigen::Vector3f& center, Eigen::Vector4f color, bool as_lines, float line_width, bool casts_shadows) : renderable(casts_shadows), as_lines_(as_lines), line_width_(line_width), colors_(1, color) {
    vertices_.resize(8);

    Eigen::Vector3f off = 0.5f * size;

    vertices_[0] = center + base * Eigen::Vector3f(-off[0], -off[1], -off[2]);
    vertices_[1] = center + base * Eigen::Vector3f(+off[0], -off[1], -off[2]);
    vertices_[2] = center + base * Eigen::Vector3f(+off[0], -off[1], +off[2]);
    vertices_[3] = center + base * Eigen::Vector3f(-off[0], -off[1], +off[2]);

    vertices_[4] = center + base * Eigen::Vector3f(-off[0], +off[1], -off[2]);
    vertices_[5] = center + base * Eigen::Vector3f(+off[0], +off[1], -off[2]);
    vertices_[6] = center + base * Eigen::Vector3f(+off[0], +off[1], +off[2]);
    vertices_[7] = center + base * Eigen::Vector3f(-off[0], +off[1], +off[2]);

    setup_from_corners_();
}

box_object::~box_object() {
}

float box_object::line_width() const {
    return line_width_;
}

void box_object::set_line_width(const float& line_width) {
    line_width_ = line_width;
}

void box_object::compute_vertex_data() {
    std::vector<float> cols(colors_.size());
    std::transform(colors_.begin(), colors_.end(), cols.begin(), [&] (const Eigen::Vector4f& c) { return renderable::color_to_rgba(c); });

    vertex_data_ = vertex_data_t::Zero(cols.size(), 9);
    index_data_ = index_data_t(cols.size(), 1);
    for (uint32_t i = 0; i < cols.size(); ++i) {
        vertex_data_.block(i, 0, 1, 3) = vertices_[i].transpose();
        if (!as_lines_) {
            vertex_data_.block(i, 4, 1, 3) = normals_[i].transpose();
        }
        //vertex_data_.block(i, 4, 1, 3) = Eigen::RowVector3f::UnitZ();
        vertex_data_(i, 3) = cols[i];
        index_data_(i, 0) = i;
    }
    colors_.clear();
}

void box_object::pre_render(shader_program::ptr program, pass_type_t type) {
    if (as_lines_) {
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(line_width_);
    }
}

void box_object::post_render(shader_program::ptr program, pass_type_t type) {
    if (as_lines_) {
        glLineWidth(1.f);
        glDisable(GL_LINE_SMOOTH);
    }
}

typename box_object::element_type_t box_object::element_type() const {
    return VERTS;
}

bool box_object::transparent() const {
    return false;
}

void box_object::compute_bounding_box_() {
    bbox_ = bbox_t();
    for (uint32_t i = 0; i < vertices_.size(); ++i) {
        bbox_.extend((transform_ * vertices_[i].homogeneous()).head(3));
    }
    Eigen::Vector3f range = bbox_.max() - bbox_.min();
    for (uint32_t i = 0; i < 3; ++i) {
        if (range[i] < 1.f) {
            bbox_.min()[i] -= 5.f;
            bbox_.max()[i] += 5.f;
        }
    }
}

GLenum box_object::gl_element_mode_() const {
    return as_lines_ ? GL_LINES : GL_TRIANGLES;
}

void box_object::setup_from_corners_() {
    std::vector<Eigen::Vector3f> vs(as_lines_ ? 24 : 36);
    std::vector<Eigen::Vector3f> ns;
    if (!as_lines_) ns.resize(36);
    if (as_lines_) {
        vs[ 0] = vertices_[0];
        vs[ 1] = vertices_[1];
        vs[ 2] = vertices_[1];
        vs[ 3] = vertices_[2];
        vs[ 4] = vertices_[2];
        vs[ 5] = vertices_[3];
        vs[ 6] = vertices_[3];
        vs[ 7] = vertices_[0];
        vs[ 8] = vertices_[0];
        vs[ 9] = vertices_[4];
        vs[10] = vertices_[1];
        vs[11] = vertices_[5];
        vs[12] = vertices_[2];
        vs[13] = vertices_[6];
        vs[14] = vertices_[3];
        vs[15] = vertices_[7];
        vs[16] = vertices_[4];
        vs[17] = vertices_[5];
        vs[18] = vertices_[5];
        vs[19] = vertices_[6];
        vs[20] = vertices_[6];
        vs[21] = vertices_[7];
        vs[22] = vertices_[7];
        vs[23] = vertices_[4];
    } else {
        vs[ 0] = vertices_[0];
        vs[ 1] = vertices_[1];
        vs[ 2] = vertices_[2];
        vs[ 3] = vertices_[0];
        vs[ 4] = vertices_[2];
        vs[ 5] = vertices_[3];
        vs[ 6] = vertices_[1];
        vs[ 7] = vertices_[5];
        vs[ 8] = vertices_[6];
        vs[ 9] = vertices_[1];
        vs[10] = vertices_[6];
        vs[11] = vertices_[2];
        vs[12] = vertices_[2];
        vs[13] = vertices_[6];
        vs[14] = vertices_[7];
        vs[15] = vertices_[2];
        vs[16] = vertices_[7];
        vs[17] = vertices_[3];
        vs[18] = vertices_[3];
        vs[19] = vertices_[7];
        vs[20] = vertices_[4];
        vs[21] = vertices_[3];
        vs[22] = vertices_[4];
        vs[23] = vertices_[0];
        vs[24] = vertices_[0];
        vs[25] = vertices_[4];
        vs[26] = vertices_[1];
        vs[27] = vertices_[1];
        vs[28] = vertices_[4];
        vs[29] = vertices_[5];
        vs[30] = vertices_[4];
        vs[31] = vertices_[6];
        vs[32] = vertices_[5];
        vs[33] = vertices_[4];
        vs[34] = vertices_[7];
        vs[35] = vertices_[6];

        ns[ 0] = -Eigen::Vector3f::UnitY();
        ns[ 1] = -Eigen::Vector3f::UnitY();
        ns[ 2] = -Eigen::Vector3f::UnitY();
        ns[ 3] = -Eigen::Vector3f::UnitY();
        ns[ 4] = -Eigen::Vector3f::UnitY();
        ns[ 5] = -Eigen::Vector3f::UnitY();
        ns[ 6] =  Eigen::Vector3f::UnitX();
        ns[ 7] =  Eigen::Vector3f::UnitX();
        ns[ 8] =  Eigen::Vector3f::UnitX();
        ns[ 9] =  Eigen::Vector3f::UnitX();
        ns[10] =  Eigen::Vector3f::UnitX();
        ns[11] =  Eigen::Vector3f::UnitX();
        ns[12] =  Eigen::Vector3f::UnitZ();
        ns[13] =  Eigen::Vector3f::UnitZ();
        ns[14] =  Eigen::Vector3f::UnitZ();
        ns[15] =  Eigen::Vector3f::UnitZ();
        ns[16] =  Eigen::Vector3f::UnitZ();
        ns[17] =  Eigen::Vector3f::UnitZ();
        ns[18] = -Eigen::Vector3f::UnitX();
        ns[19] = -Eigen::Vector3f::UnitX();
        ns[20] = -Eigen::Vector3f::UnitX();
        ns[21] = -Eigen::Vector3f::UnitX();
        ns[22] = -Eigen::Vector3f::UnitX();
        ns[23] = -Eigen::Vector3f::UnitX();
        ns[24] = -Eigen::Vector3f::UnitZ();
        ns[25] = -Eigen::Vector3f::UnitZ();
        ns[26] = -Eigen::Vector3f::UnitZ();
        ns[27] = -Eigen::Vector3f::UnitZ();
        ns[28] = -Eigen::Vector3f::UnitZ();
        ns[29] = -Eigen::Vector3f::UnitZ();
        ns[30] =  Eigen::Vector3f::UnitY();
        ns[31] =  Eigen::Vector3f::UnitY();
        ns[32] =  Eigen::Vector3f::UnitY();
        ns[33] =  Eigen::Vector3f::UnitY();
        ns[34] =  Eigen::Vector3f::UnitY();
        ns[35] =  Eigen::Vector3f::UnitY();
    }

    vertices_ = vs;
    normals_ = ns;
    auto col = colors_[0];
    colors_ = colors_t(vs.size(), col);
}


} // harmont
