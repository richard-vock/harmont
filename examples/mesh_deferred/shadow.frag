#version 430

layout(location = 0) in vec4 in_position;

layout(location = 0) out vec4 out_color;

void main() {
    out_color.r = 0.5 * (in_position.z + 1.0);
}
