#version 430

in vec2 tc;

layout(location = 0) uniform sampler2D map_gbuffer;
layout(location = 1) uniform sampler2D map_diffuse;
/*layout(location = 2) uniform sampler2D map_shadow;*/
layout(location = 3) uniform vec3 light_dir;
layout(location = 4) uniform vec3 eye_dir;
layout(location = 5) uniform float l_white;
/*layout(location = 6) uniform vec2 poisson_disk[PCF_SAMPLES];*/

out vec4 out_color;


// constants
const float pi = 3.14159265358979;
const float piInv = 0.318309886183791;
const float fudge = 0.001;

// tonemapping parameters
const float shoulder_s = 0.22;
const float linear_s = 0.3;
const float linear_a = 0.1;
const float toe_s = 0.2;
const float toe_n = 0.01;
const float toe_d = 0.3;

/*vec3 mat_specular = vec3(0.1, 0.1, 0.1);*/

// light parameters
const vec3 light_emission = vec3(1.0, 1.0, 1.0);

// shadow parameters
const float shadow_bias = 0.05;
const float sampling_spread = 1.0;
const float distance_weight = 0.0;

vec3 unpack_normal(vec3 gbuffer, out float roughness);
void unpack_colors(vec3 gbuffer, out vec3 mat_diffuse, out vec3 mat_specular);
vec3 ycbcr_to_rgb(vec3 ycbcr);
vec2 dirToUV(in vec3 dir);
vec3 tone_map(in vec3 col);
vec3 diffuse(vec3 n, vec3 l, vec3 kd);
vec3 specular(float roughness, vec3 ks, vec3 n, vec3 v, vec3 l);

void main(void) {
    vec3 gbuffer = texture2D(map_gbuffer, tc).rgb;
    if (gbuffer.r == 0.0 && gbuffer.g == 0.0 && gbuffer.b == 0.0) {
        out_color = vec4(1.0, 1.0, 1.0, 0.0);
        return;
    }

    float roughness;
    vec3 normal = unpack_normal(gbuffer, roughness);

    vec3 mat_diffuse, mat_specular;
    unpack_colors(gbuffer, mat_diffuse, mat_specular);
    vec3 mat_ambient = 0.1 * mat_diffuse;

    vec3 env_col = texture2D(map_diffuse, dirToUV(normal)).rgb * vec3(1.0, 1.0, 1.0);
    float visibility = 1.0; // 1.0 - in_shadow(normal);

    vec3 ambient = mat_ambient * env_col;
    vec3 diffuse = diffuse(normal, light_dir, mat_diffuse) * env_col;
    vec3 hdr_color = ambient;
    hdr_color += clamp(visibility, 0.05, 1.0) * diffuse;
    if (visibility > 0.0) {
        vec3 specular = specular(roughness, mat_specular, normal, eye_dir, light_dir) * light_emission;
        hdr_color += visibility * specular;
    }

    out_color = vec4(tone_map(hdr_color), 1.0);
}

vec3 unpack_normal(vec3 gbuffer, out float roughness) {
    vec4 g_part = unpackSnorm4x8(uint(gbuffer.g));
    vec2 n_xy = g_part.xy;
    float z_sign = sign(g_part.z);
    roughness = g_part.z * z_sign;
    float denom = 1.0 + n_xy.x*n_xy.x + n_xy.y*n_xy.y;
    vec3 normal = vec3(2.0*n_xy.x / denom, 2.0*n_xy.y / denom, (-1.0 + n_xy.x*n_xy.x + n_xy.y*n_xy.y) / denom);
    normal.z *= z_sign * sign(normal.z);
    return normal;
}

void unpack_colors(vec3 gbuffer, out vec3 mat_diffuse, out vec3 mat_specular) {
    vec4 b_part = unpackSnorm4x8(uint(gbuffer.b));
    mat_diffuse = ycbcr_to_rgb(b_part.xyz);
    mat_specular = ycbcr_to_rgb(vec3(b_part.w, 0.5, 0.5));
}

vec3 ycbcr_to_rgb(vec3 ycbcr) {
    vec3 off = ycbcr - vec3(0.0, 0.5, 0.5);
    vec3 rgb;
    rgb.r = dot(vec3(1.0, 0.0, -1.402), off);
    rgb.g = dot(vec3(1.0, -0.344, -0.714), off);
    rgb.b = dot(vec3(1.0, 0.0, -1.772), off);
    return rgb;
}

vec2 dirToUV(in vec3 dir) {
	if (dot(dir, vec3(0,0,1)) >=  0.9999999) return vec2(1.0, 0.0);
	if (dot(dir, vec3(0,0,1)) <= -0.9999999) return vec2(1.0, 1.0);
	return clamp(vec2(0.5f*(1.f + atan(dir.x,-dir.y) / pi), acos(dir.z) / pi), 0.f, 1.f);
}

vec3 filmic_map(vec3 x) {
    float a = shoulder_s;
    float b = linear_s;
    float c = linear_a;
    float d = toe_s;
    float e = toe_n;
    float f = toe_d;
    return ((x*(a*x + c*b) + d*e) / (x * (a*x + b) + d*f)) - e / f;
}

float filmic_map(float x) {
    float a = shoulder_s;
    float b = linear_s;
    float c = linear_a;
    float d = toe_s;
    float e = toe_n;
    float f = toe_d;
    return ((x*(a*x + c*b) + d*e) / (x * (a*x + b) + d*f)) - e / f;
}

vec3 tone_map(in vec3 col) {
    // filmic tonemapping as done in uncharted 2 (plus gamma correction)
    vec3 mapped = filmic_map(col.rgb) / filmic_map(l_white);
    mapped.r = pow(mapped.r, 1.0 / 2.2);
    mapped.g = pow(mapped.g, 1.0 / 2.2);
    mapped.b = pow(mapped.b, 1.0 / 2.2);
	return mapped;
}

vec3 diffuse(vec3 n, vec3 l, vec3 kd) {
    float nl = dot(n, l);
    if (nl < 0.0) return vec3(0.0, 0.0, 0.0);
    return kd * max(0, dot(n,l)) / pi;
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
