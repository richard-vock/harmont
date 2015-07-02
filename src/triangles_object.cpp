#include <triangles_object.hpp>

namespace harmont {


triangles_object::triangles_object(const vertices_t& vertices, Eigen::Vector4f color, bool casts_shadows) : triangles_object(vertices, colors_t(vertices.size(), color), casts_shadows) {
}

triangles_object::triangles_object(const vertices_t& vertices, const colors_t& colors, bool casts_shadows) : renderable(casts_shadows), vertices_(vertices), colors_(colors) {
    if (colors.size() != vertices.size()) {
        throw std::runtime_error("triangles_object::triangles_object(): Number of colors must equal number of vertices"+SPOT);
    }
    if (vertices.size() % 3 != 0) {
        throw std::runtime_error("triangles_object::triangles_object(): Number of vertices must be a multiple of 3"+SPOT);
    }

    normals_.resize(vertices.size());
    uint32_t num_faces = normals_.size() / 3;
    for (uint32_t f = 0; f < num_faces; ++f) {
        Eigen::Vector3f normal = (vertices[f*3+1] - vertices[f*3]).cross(vertices[f*3+2] - vertices[f*3]).normalized();
        normals_[f*3+0] = normal;
        normals_[f*3+1] = normal;
        normals_[f*3+2] = normal;
    }
}

triangles_object::~triangles_object() {
}

void triangles_object::compute_vertex_data() {
    std::vector<float> cols(colors_.size());
    std::transform(colors_.begin(), colors_.end(), cols.begin(), [&] (const Eigen::Vector4f& c) { return renderable::color_to_rgba(c); });

    vertex_data_ = vertex_data_t::Zero(cols.size(), 9);
    index_data_ = index_data_t(cols.size(), 1);
    for (uint32_t i = 0; i < cols.size(); ++i) {
        vertex_data_.block(i, 0, 1, 3) = vertices_[i].transpose();
        vertex_data_.block(i, 4, 1, 3) = normals_[i].transpose();
        vertex_data_(i, 3) = cols[i];
        index_data_(i, 0) = i;
    }
    colors_.clear();
}

void triangles_object::pre_render(shader_program::ptr program, pass_type_t type) {
}

void triangles_object::post_render(shader_program::ptr program, pass_type_t type) {
}

typename triangles_object::element_type_t triangles_object::element_type() const {
    return VERTS;
}

bool triangles_object::transparent() const {
    return false;
}

void triangles_object::compute_bounding_box_() {
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

GLenum triangles_object::gl_element_mode_() const {
    return GL_TRIANGLES;
}


} // harmont
