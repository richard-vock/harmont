#version 400

vec3 ycbcr_to_rgb(vec3 ycbcr) {
    vec3 off = ycbcr - vec3(0.0, 0.5, 0.5);
    vec3 rgb;
    rgb.r = dot(vec3(1.0, 0.0, -1.402), off);
    rgb.g = dot(vec3(1.0, -0.344, -0.714), off);
    rgb.b = dot(vec3(1.0, 0.0, -1.772), off);
    return rgb;
}

vec3 unpack_normal(uvec3 gbuffer, out float roughness) {
    vec4 g_part = unpackSnorm4x8(gbuffer.g);
    vec3 normal = g_part.xyz;
    roughness = g_part.w;
    return normal;
}

void unpack_colors(uvec3 gbuffer, out vec3 mat_diffuse, out vec3 mat_specular) {
    vec4 b_part = unpackSnorm4x8(gbuffer.b);
    mat_diffuse = b_part.xyz; //ycbcr_to_rgb(b_part.xyz);
    mat_specular = ycbcr_to_rgb(vec3(b_part.w, 0.5, 0.5));
}

float unpack_depth(uvec3 gbuffer) {
    // returns z between 0 and 1
    /*return clamp(float(gbuffer.r) / 255.0, 0.0, 1.0);*/
    return uintBitsToFloat(gbuffer.r);
}
