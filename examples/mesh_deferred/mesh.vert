#version 430
// Vertex program
in vec3  position;
in float color;
in vec3  normal;

layout(location = 0) uniform mat4 modelview_matrix;
layout(location = 1) uniform mat4 projection_matrix;
layout(location = 2) uniform mat3 normal_matrix;
layout(location = 3) uniform vec3 light_dir;
layout(location = 9) uniform mat4 shadow_matrix;

layout(location = 0) out vec3 out_position;
layout(location = 1) out vec4 out_color;
layout(location = 2) out vec3 out_normal;
layout(location = 3) out vec4 out_shadow_pos;

void main() {
    out_position = position;
	gl_Position = projection_matrix * modelview_matrix * vec4(out_position, 1.0);
    out_shadow_pos = shadow_matrix * vec4(out_position, 1.0);
    out_normal = normalize(normal);
    uint casted = floatBitsToUint(color);
    uint r = (casted >> 16) & 255;
    uint g = (casted >>  8) & 255;
    uint b = casted & 255;
    uint a = (casted >> 24) & 255;
    out_color = vec4(float(r) / 255.0, float(g) / 255.0, float(b) / 255.0, float(a) / 255.0);
}
