#include <iostream>
#include <algorithm>
#include <cctype>
#include <locale>
#include <fstream>

#include <harmont.hpp>
#include <deferred_renderer.hpp>
#include <boost/lexical_cast.hpp>
using namespace harmont;

freeglut_application::ptr   app_g;
deferred_renderer::ptr_t    renderer_g;

// trim from start (in place)
static inline void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
        return !std::isspace(ch);
    }));
}

// trim from end (in place)
static inline void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

// trim from both ends (in place)
static inline void trim(std::string &s) {
    ltrim(s);
    rtrim(s);
}

namespace harmont {

class obj_object : public renderable {
	public:
		obj_object(const std::string& input_file) : renderable(true) {
            Eigen::Matrix3f correct;
            correct <<
                1.f, 0.f, 0.f,
                0.f, 0.f, 1.f,
                0.f, -1.f, 0.f;
            Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
            std::ifstream in(input_file);
            if (in.good()) {
                std::string line;
                float max_radius = 0.f;
                float min_radius = 100000.f;
                while (std::getline(in, line)) {
                    trim(line);
                    if (line == "") continue;
                    if (line[0] == '#') continue;
                    Eigen::Vector3f pos, nrm, rgb;
                    float radius;
                    sscanf(line.c_str(), "v %f %f %f %f %f %f %f %f %f %f", &pos[0], &pos[1], &pos[2], &rgb[0], &rgb[1], &rgb[2], &radius, &nrm[0], &nrm[1], &nrm[2]);
                    if (radius < 0) continue;
                    pos_.push_back(correct * pos);
                    centroid += (pos_.back() - centroid) / pos_.size();
                    nrm_.push_back(correct * nrm);
                    col_.push_back(rgb);
                    radii_.push_back(radius);
                    min_radius = std::min(min_radius, radius);
                    max_radius = std::max(max_radius, radius);
                }
            } else {
                throw std::runtime_error("Unable to open file \"" + input_file + "\" for reading.");
            }

            for (auto& pos : pos_) {
                pos -= centroid;
            }

            point_count_ = pos_.size();
        }

		void compute_vertex_data() {
            vertex_data_ = renderable::vertex_data_t(point_count_, 10);

            // indices
            index_data_ = index_data_t(point_count_, 1);
            for (uint32_t i = 0; i < point_count_; ++i) {
                index_data_[i] = i;
            }

            // vertex data
            for (uint32_t i = 0; i < point_count_; ++i) {
                vertex_data_.block(i, 0, 1, 3) = pos_[i].transpose();
                vertex_data_(i, 3) = renderable::color_to_rgba(col_[i].homogeneous());
                vertex_data_.block(i, 4, 1, 3) = nrm_[i].transpose();
                vertex_data_(i, 9) = radii_[i];
            }
        }

		element_type_t element_type() const {
            return SPLATS;
        }

        uint32_t
        point_count(uint32_t ) const {
            return point_count_;
        }

	protected:
		void compute_bounding_box_() {
            bbox_.setEmpty();
            for (uint32_t i = 1; i < point_count_; ++i) {
                bbox_.extend(vertex_data_.block<1,3>(i,0).transpose());
            }
        }
		GLenum gl_element_mode_() const {
            return GL_POINTS;
        }

	protected:
        std::vector<Eigen::Vector3f> pos_;
        std::vector<Eigen::Vector3f> nrm_;
        std::vector<Eigen::Vector3f> col_;
        std::vector<float> radii_;
        uint32_t point_count_;
};

}

std::shared_ptr<harmont::obj_object> cloud_g;

void init() {
    Eigen::Vector3f light_dir = Eigen::Vector3f(1.f, 1.f, 1.f).normalized();

    deferred_renderer::render_parameters_t r_params {
        light_dir,
        Eigen::Vector3f(0.5f, 0.5f, 0.5f), // background color
        0.8f,
        0.003f,
        true
    };
    deferred_renderer::shadow_parameters_t s_params {
        2048,
        32
    };
    renderer_g = std::make_shared<deferred_renderer>(r_params, s_params, app_g->current_camera()->width(), app_g->current_camera()->height());
    cloud_g->init();
    renderer_g->add_object("cloud", cloud_g);
}

void display(camera::ptr cam) {
    renderer_g->render(cam);
}

void reshape(camera::ptr cam) {
    renderer_g->reshape(cam);
}

int main (int argc, char* argv[]) {
    cloud_g = std::make_shared<obj_object>(argv[1]);

    app_g = freeglut_application::create<orbit_camera_model>(800, 600, &display, &reshape);
    app_g->init(argc, argv, "Cloud Renderer", &init);
    app_g->on_click_left([&] (Eigen::Vector2i pos) { Eigen::Vector3f dir = -app_g->current_camera()->forward().normalized(); renderer_g->set_light_dir(dir); app_g->update(); });
    app_g->on_char([&] (unsigned char key) {
        if (key == 'b') renderer_g->delta_shadow_bias(-0.001);
        if (key == 'B') renderer_g->delta_shadow_bias(0.001);
        if (key == 'e') renderer_g->delta_exposure(-0.001f);
        if (key == 'E') renderer_g->delta_exposure(0.001f);
        if (key == '-') renderer_g->delta_point_size(-0.01f);
        if (key == '+') renderer_g->delta_point_size(0.01f);
        app_g->update();
    });

    app_g->run();
}
