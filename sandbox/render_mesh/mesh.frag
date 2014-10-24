#version 430


layout(location = 0) in vec3 in_position;
layout(location = 1) in vec4 in_color;
layout(location = 2) in vec3 in_normal;

out vec4 out_color;

void main() {
    out_color = in_color;
}
