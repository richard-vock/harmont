#version 430

layout(location = 0) in vec2 tc;

layout(location = 0) uniform sampler2D map_gbuffer;
layout(location = 1) uniform sampler1D map_samples;
layout(location = 2) uniform ivec2 pixel;
layout(location = 3) uniform mat4 inv_view_proj_matrix;
layout(location = 4) uniform float radius;
layout(location = 5) uniform int width;
layout(location = 6) uniform int height;

out vec3 sample_pos;


vec3 sample_point(mat3 local, vec3 pos, vec3 eye) {
    int sample_idx = int(floor(gl_FragCoord.x));
    if (sample_idx == 0) {
        return local[2];
    }
    if (sample_idx == 1) {
        return eye;
    }
    vec4 local_sample = texelFetch(map_samples, sample_idx, 0);
    local_sample.xy = 2.0 * local_sample.xy - 1.0;
    local_sample.xyz = radius * local_sample.a * normalize(local_sample.xyz) + 0.02 * vec3(0.0, 0.0, 1.0);
    return local_sample.xyz * local + pos;
}

vec3 inverse_view_project(vec3 pos) {
    vec4 proj = inv_view_proj_matrix * vec4(pos, 1.0);
    return proj.xyz / proj.w;
}

vec3 unpack_normal(vec3 gbuffer, out float roughness) {
    vec4 g_part = unpackSnorm4x8(uint(gbuffer.g));
    vec3 normal = g_part.xyz;
    roughness = g_part.w;
    return normal;
}

void main(void) {
    ivec2 px = pixel;
    px.y = height - px.y - 1;
    vec2 fpx = vec2(px) / vec2(width, height);
    vec3 gbuffer = texture(map_gbuffer, fpx).rgb;
    if (gbuffer.r == 0.0 && gbuffer.g == 0.0 && gbuffer.b == 0.0) {
        sample_pos = vec3(0.0, 0.0, 0.0);
        return;
    }

    // get normal
    float roughness;
    vec3 normal = unpack_normal(gbuffer, roughness);
    vec3 eye = unpackSnorm4x8(uint(gbuffer.b)).xyz;

    // get world position
    vec3 world_pos = inverse_view_project(vec3(fpx * 2.0 - 1.0, gbuffer.r));

    // compute local coordinate system
    vec3 tangent = abs(normal.x) > 0.99 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
    tangent = normalize(tangent - dot(tangent, normal) * normal);
    mat3 local = mat3(tangent, cross(normal, tangent), normal);

	sample_pos = sample_point(local, world_pos, eye);
}
