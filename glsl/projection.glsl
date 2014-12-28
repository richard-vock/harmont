#version 400

vec3 view_project(in vec3 pos, in mat4 projection_matrix, in mat4 view_matrix) {
    vec4 proj = projection_matrix * view_matrix * vec4(pos, 1.0);
    return proj.xyz / proj.w;
}

vec3 inverse_view_project(in vec3 pos, in mat4 inv_view_proj_matrix) {
    vec4 proj = inv_view_proj_matrix * vec4(pos, 1.0);
    return proj.xyz / proj.w;
}

float linearize_depth(in float z, in float near, in float far) {
    return (2.0 * near) / (far + near - z * (far-near));
}
