#version 430

precision highp float;

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec4 in_color;
layout(location = 2) in vec3 in_normal;

layout(location = 4) uniform mat3 normal_matrix;
layout(location = 5) uniform bool two_sided;

// material parameters
vec3  mat_diffuse = vec3(0.8, 0.8, 0.8);
vec3  mat_specular = vec3(0.1, 0.1, 0.1);
float mat_roughness  = 0.7;

out vec3 out_color;

vec3 rgb_to_ycbcr(vec3 rgb);


void main() {
    vec3 normal = in_normal;
    vec3 eye_dir = normalize(in_position);
    if (two_sided && dot(normalize(normal_matrix * normal), eye_dir) > 0.0) {
        normal *= -1.0;
    }

    vec3 albedo_ycbcr = rgb_to_ycbcr(mat_diffuse);
    vec3 spec_ycbcr = rgb_to_ycbcr(mat_specular);

    float final_r = 2.0 * gl_FragCoord.z - 1.0;
    float final_g = float(packSnorm4x8(vec4(normalize(normal_matrix * normal), mat_roughness)));
    /*float final_b = float(packSnorm4x8(vec4(albedo_ycbcr, spec_ycbcr.x)));*/
    float final_b = float(packSnorm4x8(vec4(eye_dir, 0.0)));
    out_color = vec3(final_r, final_g, final_b);
}

vec3 rgb_to_ycbcr(vec3 rgb) {
    vec3 ycbcr;
    ycbcr.r = dot(rgb, vec3(0.299, 0.587, 0.114));
    ycbcr.g = dot(rgb, vec3(-0.168, -0.331, 0.5)) + 0.5;
    ycbcr.b = dot(rgb, vec3(0.5, -0.418, -0.081)) + 0.5;
    return ycbcr;
}
