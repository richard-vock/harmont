#include <renderable_group.hpp>

namespace harmont {


renderable_group::renderable_group(const std::vector<renderable::ptr_t>& objects) : objects_(objects), clipping_height_(0.5f) {
    set_clipping_height(0.5f);
}

renderable_group::~renderable_group() {
}

void renderable_group::init() {
    for (auto& o : objects_) {
        o->init();
    }
}

std::vector<renderable::ptr_t>& renderable_group::objects() {
    return objects_;
}

const std::vector<renderable::ptr_t>& renderable_group::objects() const {
    return objects_;
}

bool renderable_group::all_active() const {
    bool active = true;
    for (const auto& o : objects_) {
        if (!o->active()) {
            active = false;
            break;
        }
    }
    return active;
}

bool renderable_group::any_active() const {
    bool active = false;
    for (const auto& o : objects_) {
        if (o->active()) {
            active = true;
            break;
        }
    }
    return active;
}

void renderable_group::set_active(const bool& active) {
    for (auto& o : objects_) {
        o->set_active(active);
    }
}

void renderable_group::toggle_active() {
    for (auto& o : objects_) {
        o->toggle_active();
    }
}

bool renderable_group::all_casts_shadows() const {
    bool casts = true;
    for (const auto& o : objects_) {
        if (!o->casts_shadows()) {
            casts = false;
            break;
        }
    }
    return casts;
}

bool renderable_group::any_casts_shadows() const {
    bool casts = false;
    for (const auto& o : objects_) {
        if (o->casts_shadows()) {
            casts = true;
            break;
        }
    }
    return casts;
}

void renderable_group::set_casts_shadows(bool casts_shadows) {
    for (auto& o : objects_) {
        o->set_casts_shadows(casts_shadows);
    }
}

void renderable_group::toggle_casts_shadows() {
    for (auto& o : objects_) {
        o->toggle_casts_shadows();
    }
}

bool renderable_group::all_clipping() const {
    bool clip = true;
    for (const auto& o : objects_) {
        if (!o->clipping()) {
            clip = false;
            break;
        }
    }
    return clip;
}

bool renderable_group::any_clipping() const {
    bool clip = false;
    for (const auto& o : objects_) {
        if (o->clipping()) {
            clip = true;
            break;
        }
    }
    return clip;
}

void renderable_group::set_clipping(bool clipping) {
    for (auto& o : objects_) {
        o->set_clipping(clipping);
    }
}

void renderable_group::toggle_clipping() {
    for (auto& o : objects_) {
        o->toggle_clipping();
    }
}

float renderable_group::clipping_height() const {
    return clipping_height_;
}

void renderable_group::set_clipping_height(float height) {
    clipping_height_ = height;
    clamp(clipping_height_, 0.f, 1.f);
    bbox_t bbox = bounding_box();
    float global_height = bbox.min()[2] + clipping_height_ * (bbox.max()[2] - bbox.min()[2]);
    for (auto& o : objects_) {
        bbox_t bb = o->bounding_box();
        float ch = (global_height - bb.min()[2]) / (bb.max()[2] - bb.min()[2]);
        o->set_clipping_height(ch);
    }
}

void renderable_group::delta_clipping_height(float delta) {
    set_clipping_height(clipping_height_ + delta);
}

bool renderable_group::all_invert_clipping() const {
    bool clip = true;
    for (const auto& o : objects_) {
        if (!o->invert_clipping()) {
            clip = false;
            break;
        }
    }
    return clip;
}

bool renderable_group::any_invert_clipping() const {
    bool clip = false;
    for (const auto& o : objects_) {
        if (o->invert_clipping()) {
            clip = true;
            break;
        }
    }
    return clip;
}

void renderable_group::set_invert_clipping(const bool& invert_clipping) {
    for (auto& o : objects_) {
        o->set_invert_clipping(invert_clipping);
    }
}

void renderable_group::toggle_invert_clipping() {
    for (auto& o : objects_) {
        o->toggle_invert_clipping();
    }
}

const renderable::transformation_t& renderable_group::transformation() const {
    return objects_[0]->transformation();
}

void renderable_group::set_transformation(const renderable::transformation_t& transformation) {
    for (auto& o : objects_) {
        o->set_transformation(transformation);
    }
}

void renderable_group::move(const renderable::transformation_t& transformation) {
    for (auto& o : objects_) {
        o->move(transformation);
    }
}

void renderable_group::set_object_color(uint32_t object_index, const Eigen::Vector4f& color) {
    if (object_index > objects_.size()) throw std::out_of_range("renderable_group::set_object_color(): Out-of-bound index"+SPOT);
    objects_[object_index]->set_colors(color);
}

void renderable_group::reset_colors() {
    for (auto& o : objects_) {
        o->reset_colors();
    }
}

bbox_t renderable_group::bounding_box() {
    bbox_t bbox;
    for (const auto& o : objects_) {
        bbox.extend(o->bounding_box());
    }
    return bbox;
}


} // harmont
