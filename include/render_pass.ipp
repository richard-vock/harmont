template <typename Func>
inline void render_pass::render(Func&& draw_call, const named_textures& inputs, bool clear_depth_buffer) {
    program_->bind();
    fbo_->bind();

    framebuffer::named_textures textures(inputs.size());
    std::transform( inputs.begin(),
                    inputs.end(),
                    textures.begin(),
                    [&] (auto tex) {
                        return std::make_pair(tex.first, (*this)[tex.second]->location());
                    }
    );
    fbo_->bind(textures);
    if (clear_depth_buffer) {
        glClear(GL_DEPTH_BUFFER_BIT);
    }
    draw_call(program_);
    fbo_->release();
    program_->release();
}

template <typename... Args>
inline void render_pass::set_uniforms(named_uniform<Args>... uniforms) {
    bool release = !(program_->bound());
    program_->bind();

    set_uniforms_(uniforms...);

    if (release) {
        program_->release();
    }
}


template <typename Arg, typename... Args>
inline void render_pass::set_uniforms_(named_uniform<Arg> uniform, named_uniform<Args>... uniforms) {
    ((*this)[uniform.first])->set(uniform.second);
    set_uniforms_(uniforms...);
}

inline void render_pass::set_uniforms_() {
}
