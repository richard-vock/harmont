#version 430


layout(location = 0) in vec3 in_position;
layout(location = 1) in vec4 in_color;
layout(location = 2) in vec3 in_normal;

layout(location = 3) uniform vec3 light_dir;
layout(location = 4) uniform sampler2D map_diffuse;
layout(location = 5) uniform vec3 eye_dir;
layout(location = 6) uniform float l_white;


const float pi = 3.14159265358979;
const float piInv = 0.318309886183791;
const float fudge = 0.001;
const vec3 ambient_color = vec3(0.1, 0.1, 0.1);

const float shoulder_s = 0.22;
const float linear_s = 0.3;
const float linear_a = 0.1;
const float toe_s = 0.2;
const float toe_n = 0.01;
const float toe_d = 0.3;

const float albedo = 1.0;
const vec3  light_emission = vec3(1.0, 1.0, 1.0);
const float sigma = 0.1;
const float sigma2 = sigma * sigma;

out vec4 out_color;

vec2 dirToUV(in vec3 dir);
vec4 tone_map(in vec4 col);
mat3 local_base(in vec3 normal);
vec2 to_local_angles(in mat3 local_mat, in vec3 v);

void main() {
    vec4 ambient = 0.6 * texture2D(map_diffuse, dirToUV(in_normal)) * in_color;

    mat3 local_mat = local_base(in_normal);
    vec2 local_i = to_local_angles(local_mat, light_dir);
    vec2 local_o = to_local_angles(local_mat, eye_dir);

    float A = 1.0 - 0.5*(sigma2 / (sigma2+0.33));
    float B = 0.45 * (sigma2 / (sigma2 + 0.09));
    float a = max(local_i.x, local_o.x);
    float b = min(local_i.x, local_o.x);
    //vec3 oren_nayar = (albedo / pi) * cos(local_i.x) * (A + (B * max(0.0, cos(local_i.y - local_o.y)) * sin(a) * tan(b))) * light_emission;
    float cos_theta_i = dot(light_dir, in_normal);
    //vec3 oren_nayar = (albedo / pi) * max(0.0, cos(local_i.x)) * light_emission;
    vec3 oren_nayar = (albedo / pi) * cos_theta_i * light_emission;
    vec4 diffuse = vec4(oren_nayar, 1.0) * in_color;

    out_color = tone_map(ambient);
    out_color.a = in_color.a;
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

vec4 tone_map(in vec4 col) {
	col.rgb = filmic_map(col.rgb) / filmic_map(l_white);

	return col;
}

mat3 local_base(in vec3 n) {
    mat3 local_mat;
    vec3 x = vec3(1.0, 0.0, 0.0);
    if (abs(dot(x, n)) > 0.9) x = vec3(0.0, 1.0, 0.0);
    x = normalize(x - dot(x, n) * n);
    local_mat[0] = x;
    local_mat[1] = normalize(cross(n, x));
    local_mat[2] = n;

    return transpose(local_mat);
}

vec2 to_local_angles(in mat3 local_mat, in vec3 v) {
    vec3 local = local_mat * v;
    return vec2(acos(v.z), atan(v.y / v.x));
}
