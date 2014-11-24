#version 430

layout(location = 0) in vec4 in_position;

layout(location = 0) out vec4 out_color;

void main() {
    out_color.r = in_position.z / in_position.w;
}
