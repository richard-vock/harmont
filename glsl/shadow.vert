#version 430
// Vertex program
in vec3  position;
in float color;
in vec3  normal;

layout(location = 0) uniform mat4 model_matrix;
layout(location = 1) uniform mat4 shadow_view;
layout(location = 2) uniform mat4 shadow_proj;
layout(location = 3) uniform vec3  clip_normal;
layout(location = 4) uniform float clip_distance;
layout(location = 5) uniform float vp_ratio;

layout(location = 0) out vec4 out_position;

void main() {
	out_position = shadow_view * model_matrix * vec4(position, 1.0);
	gl_Position = shadow_proj * out_position;

    gl_ClipDistance[0] = -dot((model_matrix * vec4(position, 1.0)).xyz, clip_normal) + clip_distance;
    gl_PointSize = max(0.1, vp_ratio / out_position.z);
    out_position = gl_Position;
}
