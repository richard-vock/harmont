#version 430
// Vertex program
in vec3  position;
in float color;
in vec3  normal;
in vec2  tex_coords;

layout(location = 0) uniform mat4  model_matrix;
layout(location = 1) uniform mat4  view_matrix;
layout(location = 2) uniform mat4  projection_matrix;
layout(location = 3) uniform vec3  clip_normal;
layout(location = 4) uniform float clip_distance;
layout(location = 5) uniform float vp_ratio;

layout(location = 0) out vec3 out_position;
layout(location = 1) out vec4 out_color;
layout(location = 2) out vec3 out_normal;
layout(location = 3) out vec2 out_tex_coords;

void main() {
    out_position = (view_matrix * model_matrix * vec4(position, 1.0)).xyz;
	gl_Position = projection_matrix * vec4(out_position, 1.0);

    if (dot(normal, normal) < 0.2) {
        out_normal = vec3(0.0, 0.0, 0.0);
    } else {
        out_normal = normalize(normal);
    }

    uint casted = floatBitsToUint(color);
    uint r = (casted >> 16) & 255;
    uint g = (casted >>  8) & 255;
    uint b = casted & 255;
    uint a = (casted >> 24) & 255;

    out_color = vec4(float(r) / 255.0, float(g) / 255.0, float(b) / 255.0, float(a) / 255.0);
    out_tex_coords = tex_coords;

    gl_ClipDistance[0] = -dot((model_matrix * vec4(position, 1.0)).xyz, clip_normal) + clip_distance;
    gl_PointSize = max(1.0, vp_ratio / out_position.z);
}
