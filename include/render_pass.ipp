template <typename Func>
inline void render_pass::render(Func&& draw_call, const named_textures& inputs, bool clear_depth_buffer) {
    program_->bind();
    fbo_->bind();

    framebuffer::named_textures textures(inputs.size());
    std::transform( inputs.begin(),
                    inputs.end(),
                    textures.begin(),
                    [&] (const named_texture& tex) {
                        return std::make_pair(tex.first, (*this)[tex.second].location());
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

template <typename T>
inline void render_pass::set_uniform(const std::string& name, T&& value) {
    bool release = !(program_->bound());
    program_->bind();

    ((*this)[name]).set(std::forward<T>(value));

    if (release) {
        program_->release();
    }
}
