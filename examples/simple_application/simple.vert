#version 430

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 color;

layout(location = 0) uniform mat4 modelview_matrix;
layout(location = 1) uniform mat4 projection_matrix;

layout(location = 0) out vec3 out_color;

void main() {
	gl_Position = projection_matrix * modelview_matrix * vec4(position, 1.0);
    out_color = color;
}
