#version 430
// Vertex program
in vec3  position;

layout(location = 0) uniform mat4  modelview_matrix;
layout(location = 1) uniform mat4  projection_matrix;

void main() {
	gl_Position = projection_matrix * modelview_matrix * vec4(position, 1.0);
}
