#version 430

precision highp float;

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec4 in_color;
layout(location = 2) in vec3 in_normal;

// material parameters
float k_ambient = 0.1;
float k_diffuse = 0.8;
/*float k_specular = 0.01;*/
/*float k_rough  = 0.5;*/
float k_specular = 0.1;
float k_rough  = 0.7;

out vec3 out_color;

vec3 rgb_to_ycbcr(vec3 rgb);


void main() {
    vec2 n_xy = vec2(in_normal.x / (1.0 - in_normal.z), in_normal.y / (1.0 - in_normal.z));
    float rough = k_rough * sign(in_normal.z);
    vec3 albedo_rgb = k_diffuse * vec3(1.0, 1.0, 1.0);
    vec3 spec_rgb = k_specular * vec3(1.0, 1.0, 1.0);
    vec3 albedo_ycbcr = rgb_to_ycbcr(albedo_rgb);
    vec3 spec_ycbcr = rgb_to_ycbcr(spec_rgb);

    /*uint final_r = uint(255 * in_position.z);*/
    uint final_r = 0;
    uint final_g = packSnorm4x8(vec4(n_xy, rough, 0.0));
    uint final_b = 0;
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
    out_color = vec3(float(final_r), float(final_g), float(final_b));
}

vec3 rgb_to_ycbcr(vec3 rgb) {
    vec3 ycbcr;
    ycbcr.r = dot(rgb, vec3(0.299, 0.587, 0.114));
    ycbcr.g = dot(rgb, vec3(-0.168, -0.331, 0.5)) + 0.5;
    ycbcr.b = dot(rgb, vec3(0.5, -0.418, -0.081)) + 0.5;
    return ycbcr;
}
