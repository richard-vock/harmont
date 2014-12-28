#version 430

#define PCF_SAMPLES        {{sample_count}}

in vec2 tc;

layout(location = 0) uniform usampler2D map_gbuffer;
layout(location = 1) uniform sampler2D map_diffuse;
layout(location = 2) uniform sampler2D map_shadow;
layout(location = 3) uniform vec3 light_dir;
layout(location = 4) uniform vec3 eye_dir;
layout(location = 5) uniform float l_white;
layout(location = 6) uniform mat4 shadow_matrix;
layout(location = 7) uniform mat4 inv_view_proj_matrix;
layout(location = 8) uniform float shadow_bias;
layout(location = 9) uniform sampler2D map_ssdo;
layout(location = 10) uniform vec3 background_color;
layout(location = 11) uniform vec2 poisson_disk[PCF_SAMPLES];

out vec4 out_color;


// constants
const float pi = 3.14159265358979;
const float min_diffuse = 0.2;

/*vec3 mat_specular = vec3(0.1, 0.1, 0.1);*/

// light parameters
const vec3 light_emission = vec3(1.0, 1.0, 1.0);

vec3 unpack_normal(uvec3 gbuffer, out float roughness);
void unpack_colors(uvec3 gbuffer, out vec3 mat_diffuse, out vec3 mat_specular);
float unpack_depth(uvec3 gbuffer);
vec3 sample_hdr(in vec3 dir, in sampler2D map_environment);
vec3 tone_map(in vec3 col, in float l_white);
vec3 diffuse(vec3 n, vec3 l, vec3 kd);
vec3 specular(float roughness, vec3 ks, vec3 n, vec3 v, vec3 l);
float in_shadow(in sampler2D map_shadow, in vec2 poisson_disk[PCF_SAMPLES], in vec3 light_dir, in float shadow_bias, in vec3 normal, in vec4 in_shadow_pos);

void main(void) {
    uvec3 gbuffer = texture(map_gbuffer, tc).rgb;
    if (gbuffer.r == 0 && gbuffer.g == 0 && gbuffer.b == 0) {
        out_color = vec4(background_color, 1.0);
        return;
    }

    float roughness;
    vec3 normal = unpack_normal(gbuffer, roughness);

    vec3 mat_diffuse, mat_specular;
    unpack_colors(gbuffer, mat_diffuse, mat_specular);

    if (dot(normal, normal) < 0.2) {
        out_color = vec4(mat_diffuse, 1.0);
        return;
    } else {
        vec3 mat_ambient = 0.03 * mat_diffuse;

        float window_z = unpack_depth(gbuffer);
        vec4 window_pos = vec4(tc * 2.0 - 1.0, 2.0 * window_z - 1.0, 1.0);
        vec4 world_pos = inv_view_proj_matrix * window_pos;
        world_pos /= world_pos.w;
        vec4 in_shadow_pos = shadow_matrix * world_pos;

        vec3 env_col = sample_hdr(normal, map_diffuse) * vec3(1.0, 1.0, 1.0);
        float visibility = 1.0 - in_shadow(map_shadow, poisson_disk, light_dir, shadow_bias, normal, in_shadow_pos);

        vec3 ssdo = texture(map_ssdo, tc).rgb;

        vec3 ambient = mat_ambient * env_col;// * ssdo;
        vec3 diffuse = ssdo; // env_col;// * ssdo;
        vec3 hdr_color = vec3(0.0, 0.0, 0.0); //ambient;
        hdr_color += clamp(visibility, min_diffuse, 1.0) * diffuse;
        if (visibility > 0.0) {
            vec3 specular = specular(roughness, mat_specular, normal, eye_dir, light_dir) * light_emission;
            hdr_color += visibility * specular;
        }

        out_color = vec4(tone_map(hdr_color, l_white), 1.0);
    }
}

float spec_d(float roughness, vec3 n, vec3 h) {
    // computes ndf as suggested by disney (GGX/Trowbridge-Reitz)
    float nh = dot(n, h);
    float tmp = nh * nh * (roughness - 1) + 1;
    return roughness / (pi * tmp * tmp);
}

float spec_g(float roughness, vec3 n, vec3 v, vec3 l) {
    // modified schlick model
    float k = (roughness + 1) * (roughness + 1) / 8;
    float nv = dot(n,v);
    float nl = dot(n,l);
    float gl = nl / (nl * (1-k) + k);
    float gv = nv / (nv * (1-k) + k);
    return gl * gv;
}

vec3 spec_f(vec3 ks, vec3 v, vec3 h) {
    // simplified schlick approximation
    float vh = dot(v,h);
    if (vh < 0.0) return vec3(0.0, 0.0, 0.0);
    return ks + (vec3(1.0, 1.0, 1.0) - ks) * pow(2.0, (-5.55473*vh - 6.98316) * vh);
    /*return (1-ks) * pow(2.0, (-5.55473*vh - 6.98316) * vh);*/
}

vec3 specular(float roughness, vec3 ks, vec3 n, vec3 v, vec3 l) {
    float nl = dot(n, l);
    float nv = dot(n, v);
    if (nl < 0.0 || nv < 0.0) return vec3(0.0, 0.0, 0.0);
    vec3 h = normalize(l + v);
    float sd = spec_d(roughness, n, h);
    vec3 sf = spec_f(ks, v, h);
    float sg = spec_g(roughness, n, v, l);

    return sd * sf * sg / (4.0 * nl * nv);
}
