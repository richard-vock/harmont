#version 430

const float pi = 3.14159265358979;

layout(location = 0) uniform mat4 view_matrix;
layout(location = 1) uniform mat4 projection_matrix;
layout(location = 2) uniform float exponent;
layout(location = 3) uniform float radius;
layout(location = 4) uniform usampler2D map_gbuffer;
layout(location = 5) uniform sampler1D map_samples;
layout(location = 6) uniform sampler2D map_noise;
layout(location = 7) uniform mat4 inv_view_proj_matrix;
layout(location = 8) uniform float near;
layout(location = 9) uniform float far;
layout(location = 10) uniform sampler2D map_env;
layout(location = 11) uniform sampler2D map_last;
layout(location = 12) uniform float reflective_albedo;
layout(location = 13) uniform bool is_first_frame;

in  vec2 tc;
out vec3 light;

// imported functions
vec3 unpack_normal(uvec3 gbuffer, out float roughness);
void unpack_colors(uvec3 gbuffer, out vec3 mat_diffuse, out vec3 mat_specular);
float unpack_depth(uvec3 gbuffer);
vec3 view_project(in vec3 pos, in mat4 projection_matrix, in mat4 view_matrix);
vec3 inverse_view_project(in vec3 pos, in mat4 inv_view_proj_matrix);
float linearize_depth(in float z, in float near, in float far);
int argmax(vec3 vec);
vec3 sample_hdr(in vec3 dir, in sampler2D map_environment);

// compute indirect lighting term given a position, the local frame and diffuse material properties
vec3 ssdo(mat3 local, vec3 pos, vec3 kd);

void main (void) {
    uvec3 gbuffer = texture(map_gbuffer, tc).rgb;
    if (gbuffer.r == 0 && gbuffer.g == 0 && gbuffer.b == 0) {
        light = vec3(0.0, 0.0, 0.0);
        return;
    }

    // get normal
    float roughness;
    vec3 normal = normalize(unpack_normal(gbuffer, roughness));

    // get colors
    vec3 mat_diffuse, mat_specular;
    unpack_colors(gbuffer, mat_diffuse, mat_specular);
    if (dot(normal, normal) < 0.2) {
        light = mat_diffuse;
    } else {
        // get world position
        float world_z = 2.0 * unpack_depth(gbuffer) - 1.0;
        vec3 world_pos = inverse_view_project(vec3(tc * 2.0 - 1.0, world_z), inv_view_proj_matrix);

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

        light = ssdo(local, world_pos, mat_diffuse);
    }
}

vec3 ssdo(mat3 local, vec3 pos, vec3 kd) {
    vec3 light = vec3(0.0, 0.0, 0.0);
    vec3 global_sample;
    vec4 local_sample;
    int num_samples = textureSize(map_samples, 0).x;
    /*int used_samples = 0;*/
    for (int i=0; i<num_samples; ++i) {
        local_sample = texelFetch(map_samples, i, 0);
        local_sample.xy = 2.0 * local_sample.xy - 1.0;
        local_sample.xyz = radius * local_sample.w * normalize(local_sample.xyz) + 0.002 * vec3(0.0, 0.0, 1.0);
        global_sample = local * local_sample.xyz + pos;

        vec3 proj_sample = view_project(global_sample, projection_matrix, view_matrix);

        vec2 sample_tc = 0.5 * (proj_sample.xy + vec2(1.0));
        uvec3 sample_g = texture(map_gbuffer, sample_tc).rgb;
        vec3 nlocal = normalize(local_sample.xyz);
        float cos_theta = clamp(nlocal.z, 0.0, 1.0);
        if (sample_g.r != 0 || sample_g.g != 0 || sample_g.b != 0) {
            float sample_z = unpack_depth(sample_g);
            float this_z = linearize_depth(0.5 * (proj_sample.z + 1.0), near, far);
            float ref_z = linearize_depth(sample_z, near, far);
            vec3 world_sample = inverse_view_project(vec3(proj_sample.xy, 2.0 * sample_z - 1.0), inv_view_proj_matrix);
            float dist = length(world_sample - pos);

            vec3 dir = normalize(global_sample - pos);
            if (dist < radius && this_z > ref_z && !is_first_frame) {
                // indirect light
                float dummy;
                vec3 other_normal = normalize(unpack_normal(sample_g, dummy));
                float other_cos_theta = clamp(dot(-dir, other_normal), 0.0, 1.0);
                vec3 last_color = texture(map_last, sample_tc).rgb;
                float di = dist;
                if (di < 1.0) di = 1.0;
                light += reflective_albedo * cos_theta * other_cos_theta * (1.0 / (di*di)) * last_color * kd;
            } else {
                // direct light
                light += 0.8 * pi * cos_theta * sample_hdr(dir, map_env) * kd;
            }
        } else {
            vec3 dir = normalize(global_sample - pos);
            vec3 l = 0.8 * pi * cos_theta * sample_hdr(dir, map_env) * kd;
            light += l;
        }
    }
    light /= float(num_samples);
    /*if (used_samples > 1) {*/
        /*light /= float(used_samples);*/
    /*} else {*/
        /*light = vec3(0.0, 0.0, 0.0);*/
    /*}*/
    light.x = pow(light.x, exponent);
    light.y = pow(light.y, exponent);
    light.z = pow(light.z, exponent);
    return light;
}
