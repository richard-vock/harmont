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

out uvec3 out_color;

vec3 rgb_to_ycbcr(vec3 rgb);


void main() {
    vec3 normal = in_normal;
    vec3 eye_dir = normalize(in_position);
    if (two_sided && dot(normalize(normal_matrix * normal), eye_dir) > 0.0) {
        normal *= -1.0;
    }

    vec3 albedo_ycbcr = rgb_to_ycbcr(in_color.rgb);
    vec3 spec_ycbcr = rgb_to_ycbcr(mat_specular);

    uint final_r = floatBitsToUint(gl_FragCoord.z);//uint(clamp(gl_FragCoord.z * 255.0, 0.0, 255.0));
    uint final_g = packSnorm4x8(vec4(normal, mat_roughness));
    uint final_b = packSnorm4x8(vec4(in_color.rgb, spec_ycbcr.x));
    out_color = uvec3(final_r, final_g, final_b);
}

vec3 rgb_to_ycbcr(vec3 rgb) {
    vec3 ycbcr;
    ycbcr.r = dot(rgb, vec3(0.299, 0.587, 0.114));
    ycbcr.g = dot(rgb, vec3(-0.168, -0.331, 0.5)) + 0.5;
    ycbcr.b = dot(rgb, vec3(0.5, -0.418, -0.081)) + 0.5;
    return ycbcr;
}
