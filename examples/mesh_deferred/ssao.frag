#version 430

layout(location = 0) uniform mat4 modelview_matrix;
layout(location = 1) uniform mat4 projection_matrix;
layout(location = 2) uniform float exponent;
layout(location = 3) uniform float radius;
layout(location = 4) uniform sampler2D map_gbuffer;
layout(location = 5) uniform sampler1D map_samples;
layout(location = 6) uniform sampler2D map_noise;
layout(location = 7) uniform mat4 inv_view_proj_matrix;
layout(location = 8) uniform float near;
layout(location = 9) uniform float far;

in  vec2 tc;
out float light;

const float pi = 3.14159265358979;

float ssao(mat3 local, vec3 pos);
vec3 unpack_normal(vec3 gbuffer, out float roughness);
vec3 view_project(vec3 pos);
vec3 inverse_view_project(vec3 pos);
float linearize_depth(float z);

void main (void) {
    vec3 gbuffer = texture2D(map_gbuffer, tc).rgb;
    if (gbuffer.r == 0.0 && gbuffer.g == 0.0 && gbuffer.b == 0.0) {
        light = 1.0;
        return;
    }

    // get normal
    float roughness;
    vec3 normal = unpack_normal(gbuffer, roughness);

    // get world position
    vec3 world_pos = inverse_view_project(vec3(tc * 2.0 - 1.0, gbuffer.r));

    // compute local coordinate system
    vec3 tangent = abs(normal.x) > 0.99 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
    tangent = normalize(tangent - dot(tangent, normal) * normal);
    mat3 local = mat3(tangent, cross(normal, tangent), normal);

	light = ssao(local, world_pos);
}

float ssao(mat3 local, vec3 pos) {
    float occlusion = 0.0;
    vec3 global_sample;
    vec4 local_sample;
    int num_samples = 20; //textureSize(map_samples, 0).x;
    /*vec2 tc_noise = tc * (vec2(textureSize(map_gbuffer, 0)) / vec2(textureSize(map_noise, 0)));*/
    /*int variation = int(texture(map_noise, tc_noise).r);*/
    int used_samples = 0;
    for (int i=0; i<num_samples; ++i) {
        local_sample = texelFetch(map_samples, i, 0);
        local_sample.xy = 2.0 * local_sample.xy - 1.0;
        local_sample.xyz = radius * local_sample.a * normalize(local_sample.xyz) + 0.02 * vec3(0.0, 0.0, 1.0);
        /*local_sample = texture(map_samples, vec2(0.0, 0.0)).xyz;*/
        global_sample = local_sample.xyz * local + pos;

        vec3 proj_sample = view_project(global_sample);

        vec3 sample_g = texture2D(map_gbuffer, 0.5 * (proj_sample.xy + vec2(1.0))).rgb;
        if (sample_g.r == 0.0 && sample_g.g == 0.0 && sample_g.b == 0.0) {
            continue;
        }

        float this_z = 0.5 * (proj_sample.z + 1.0);
        float ref_z = 0.5 * (sample_g.r + 1.0);
        /*float this_z = linearize_depth(0.5 * (proj_sample.z + 1.0));*/
        /*float ref_z = linearize_depth(0.5 * (sample_g.r + 1.0));*/
        /*continue;*/
        vec3 world_sample = inverse_view_project(vec3(proj_sample.xy, sample_g.r));

        float range_check = 1.0; //length(world_sample - pos) > radius ? 0.0 : 1.0;
        used_samples += int(range_check);
        occlusion += range_check * (this_z > ref_z ? 1.0 : 0.0);
    }
    if (used_samples > 1.0) {
        occlusion /= float(used_samples);
    } else {
        occlusion = 0.0;
    }
    return occlusion;
    /*return 1.0 - pow(occlusion, exponent);*/
    /*return pow(occlusion, exponent);*/
}

vec3 unpack_normal(vec3 gbuffer, out float roughness) {
    vec4 g_part = unpackSnorm4x8(uint(gbuffer.g));
    vec3 normal = g_part.xyz;
    roughness = g_part.w;
    return normal;
}

vec3 view_project(vec3 pos) {
    vec4 proj = projection_matrix * modelview_matrix * vec4(pos, 1.0);
    return proj.xyz / proj.w;
}

vec3 inverse_view_project(vec3 pos) {
    vec4 proj = inv_view_proj_matrix * vec4(pos, 1.0);
    return proj.xyz / proj.w;
}

float linearize_depth(float z) {
    return (2.0 * near) / (far + near - z * (far-near));
}
