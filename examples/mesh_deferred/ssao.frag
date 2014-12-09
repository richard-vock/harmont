#version 430

layout(location = 0) uniform mat4 modelview_matrix;
layout(location = 1) uniform mat4 projection_matrix;
layout(location = 2) uniform float exponent;
layout(location = 3) uniform float radius;
layout(location = 4) uniform usampler2D map_gbuffer;
layout(location = 5) uniform sampler1D map_samples;
layout(location = 6) uniform sampler2D map_noise;
layout(location = 7) uniform mat4 inv_view_proj_matrix;
layout(location = 8) uniform float near;
layout(location = 9) uniform float far;

in  vec2 tc;
out float light;

const float pi = 3.14159265358979;

float ssao(mat3 local, vec3 pos);
vec3 unpack_normal(uvec3 gbuffer, out float roughness);
float unpack_depth(uvec3 gbuffer);
vec3 view_project(vec3 pos);
vec3 inverse_view_project(vec3 pos);
float linearize_depth(float z);
int argmax(vec3 vec);

void main (void) {
    uvec3 gbuffer = texture(map_gbuffer, tc).rgb;
    if (gbuffer.r == 0 && gbuffer.g == 0 && gbuffer.b == 0) {
        light = 0.0;
        return;
    }

    // get normal
    float roughness;
    vec3 normal = normalize(unpack_normal(gbuffer, roughness));

    // get world position
    float world_z = 2.0 * unpack_depth(gbuffer) - 1.0;
    vec3 world_pos = inverse_view_project(vec3(tc * 2.0 - 1.0, world_z));

    vec2 tc_noise = tc * (vec2(textureSize(map_gbuffer, 0)) / vec2(textureSize(map_noise, 0)));
    vec2 noise = 0.5 * (texture(map_noise, tc_noise).rg - vec2(1.0, 1.0));

    // compute local coordinate system
    /*vec3 tangent = abs(normal.x) > 0.999 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);*/
    int amax = argmax(normal);
    vec3 tangent = vec3(0.0, 0.0, 0.0);
    tangent[amax] = 1.0;
    tangent[(amax + 1) % 3] = noise.x;
    tangent[(amax + 2) % 3] = noise.y;
    tangent = normalize(tangent);
    tangent = normalize(tangent - dot(tangent, normal) * normal);
    mat3 local = mat3(tangent, cross(normal, tangent), normal);

	light = ssao(local, world_pos);
}

float ssao(mat3 local, vec3 pos) {
    float occlusion = 0.0;
    vec3 global_sample;
    vec4 local_sample;
    int num_samples = textureSize(map_samples, 0).x;
    int used_samples = 0;
    for (int i=0; i<num_samples; ++i) {
        local_sample = texelFetch(map_samples, i, 0);
        local_sample.xy = 2.0 * local_sample.xy - 1.0;
        local_sample.xyz = radius * local_sample.w * normalize(local_sample.xyz) + 0.002 * vec3(0.0, 0.0, 1.0);
        /*local_sample = texture(map_samples, vec2(0.0, 0.0)).xyz;*/
        global_sample = local * local_sample.xyz + pos;

        vec3 proj_sample = view_project(global_sample);

        uvec3 sample_g = texture(map_gbuffer, 0.5 * (proj_sample.xy + vec2(1.0))).rgb;
        if (sample_g.r != 0 || sample_g.g != 0 || sample_g.b != 0) {
            float sample_z = unpack_depth(sample_g);
            float this_z = linearize_depth(0.5 * (proj_sample.z + 1.0));
            float ref_z = linearize_depth(sample_z);
            vec3 world_sample = inverse_view_project(vec3(proj_sample.xy, 2.0 * sample_z - 1.0));

            float range_check = length(world_sample - pos) > radius ? 0.0 : 1.0;
            used_samples += int(range_check);
            occlusion += range_check * (this_z > ref_z ? 1.0 : 0.0);
        }
    }
    /*return occlusion / num_samples;*/
    if (used_samples > 1) {
        occlusion /= float(used_samples);
    } else {
        occlusion = 0.0;
    }
    /*return occlusion;*/
    return 1.0 - pow(occlusion, exponent);
    /*return pow(occlusion, exponent);*/
}

vec3 unpack_normal(uvec3 gbuffer, out float roughness) {
    vec4 g_part = unpackSnorm4x8(gbuffer.g);
    vec3 normal = g_part.xyz;
    roughness = g_part.w;
    return normal;
}

float unpack_depth(uvec3 gbuffer) {
    // returns z between 0 and 1
    /*return clamp(float(gbuffer.r) / 255.0, 0.0, 1.0);*/
    return uintBitsToFloat(gbuffer.r);
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

int argmax(vec3 vec) {
    float crt_max = 0.0;
    int result = 0;
    float value;
    for (int i=0; i<3; ++i) {
        value = abs(vec[i]);
        if (value <= crt_max) continue;
        result = i;
        crt_max = value;
    }
    return result;
}
