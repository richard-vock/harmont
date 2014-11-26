#version 430
// Vertex program
in vec3  position;

layout(location = 0) uniform mat4 shadow_matrix;

layout(location = 0) out vec3 out_position;

void main() {
    out_position = position;
	gl_Position = shadow_matrix * vec4(out_position, 1.0);
}
