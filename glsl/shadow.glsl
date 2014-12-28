#version 130

#define PCF_SAMPLES        {{sample_count}}
#define SHADOW_RES         {{shadow_res}}
#define SHADOW_RES_INV     (1.0 / SHADOW_RES)

const float sampling_spread = 1.0;
const float distance_weight = 0.0;

float take_shadow_sample(in sampler2D map_shadow, in vec2 xy, in vec2 o, in float z, inout float n) {
    float l = length(o * SHADOW_RES);
    float f = 1.0 / (1.0 + pow(l, distance_weight));
    n += f;
    return f * float(texture2D(map_shadow, xy + o).r < z);
}

float sample_shadow(in sampler2D map_shadow, in vec2 poisson_disk[PCF_SAMPLES], in vec2 xy, in float z) {
    float h = SHADOW_RES_INV * sampling_spread;
    float ret = 0.0;
    float n = 0.0;

    for (int i=0; i < PCF_SAMPLES; ++i) {
        vec2 o = poisson_disk[i] * h;
        ret += take_shadow_sample(map_shadow, xy, o, z, n);
    }
    return ret / n;
}

float in_shadow(in sampler2D map_shadow, in vec2 poisson_disk[PCF_SAMPLES], in vec3 light_dir, in float shadow_bias, in vec3 normal, in vec4 in_shadow_pos) {
    if (dot(normal, light_dir) < 0.0) {
        return 1.0;
    }
    vec3 real_shadow = in_shadow_pos.xyz / in_shadow_pos.w;
    return sample_shadow(map_shadow, poisson_disk, 0.5 * (real_shadow.xy + vec2(1.0, 1.0)), 0.5 * (real_shadow.z + 1.0) - shadow_bias);
}
