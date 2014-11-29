#version 430

precision highp float;

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec4 in_color;
layout(location = 2) in vec3 in_normal;

layout(location = 2) uniform mat3 normal_matrix;
layout(location = 3) uniform bool two_sided;

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

    float n_denom = 1.0 - normal.z;
    vec2 n_xy;
    if (n_denom < 0.00001) {
        n_xy = vec2(normal.x, normal.y);
    } else {
        n_xy = vec2(normal.x / (1.0 - normal.z), normal.y / (1.0 - normal.z));
    }
    vec3 albedo_ycbcr = rgb_to_ycbcr(mat_diffuse);
    vec3 spec_ycbcr = rgb_to_ycbcr(mat_specular);

    float final_r = 2.0 * gl_FragCoord.z - 1.0;;
    float final_g = float(packSnorm4x8(vec4(n_xy, mat_roughness * sign(normal.z), 0.0)));
    float final_b = float(packSnorm4x8(vec4(albedo_ycbcr, spec_ycbcr.x)));
    /*[>uint final_g = uint(255 * n_xy.x) << 3*8<]*/
                 /*[>+ uint(255 * n_xy.y) << 2*8<]*/
                 /*[>+ uint(255 * rough)  << 1*8;<]*/
    /*uint final_b = uint(255 * albedo_ycbcr.r) << 3*8*/
                 /*+ uint(255 * albedo_ycbcr.g) << 2*8*/
                 /*+ uint(255 * albedo_ycbcr.b) << 1*8*/
                 /*+ uint(128 * spec_ycbcr.r + 128);*/
    /*float g = encode_rgba(vec4(0.0, 1.0, 0.5, 1.0));*/
    /*out_color = vec3(uintBitsToFloat(final_r), g, uintBitsToFloat(final_b));*/
    /*vec3 rgb = vec3(0.0, 0.0, 1.0);*/
    /*uint packed_rgb = packSnorm4x8(vec4(rgb, 0.0));*/
    /*float pfloat = float(packed_rgb);*/
    /*proj.xyz /= proj.w;*/
    /*out_color = proj.xyz;*/
    out_color = vec3(final_r, final_g, final_b);
}

vec3 rgb_to_ycbcr(vec3 rgb) {
    vec3 ycbcr;
    ycbcr.r = dot(rgb, vec3(0.299, 0.587, 0.114));
    ycbcr.g = dot(rgb, vec3(-0.168, -0.331, 0.5)) + 0.5;
    ycbcr.b = dot(rgb, vec3(0.5, -0.418, -0.081)) + 0.5;
    return ycbcr;
}
