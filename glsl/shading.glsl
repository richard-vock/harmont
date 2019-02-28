#version 400

const float pi = 3.14159265358979;
const float shoulder_s = 0.22;
const float linear_s = 0.3;
const float linear_a = 0.1;
const float toe_s = 0.2;
const float toe_n = 0.01;
const float toe_d = 0.3;


float spec_d(float roughness, vec3 n, vec3 h) {
    // computes ndf as suggested by disney (GGX/Trowbridge-Reitz)
    float nh = dot(n, h);
    if (nh < 0.001) return 0.0;
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
    if (vh < 0.001) return vec3(0.0, 0.0, 0.0);
    return ks + (vec3(1.0, 1.0, 1.0) - ks) * pow(2.0, (-5.55473*vh - 6.98316) * vh);
    /*return (1-ks) * pow(2.0, (-5.55473*vh - 6.98316) * vh);*/
}

vec3 specular(float roughness, vec3 ks, vec3 n, vec3 v, vec3 l) {
    // modified cook-torrance as suggested by crytek/disney
    float nl = dot(n, l);
    float nv = dot(n, v);
    if (nl < 0.001 || nv < 0.001) return vec3(0.0, 0.0, 0.0);
    vec3 h = normalize(l + v);
    float sd = spec_d(roughness, n, h);
    vec3 sf = spec_f(ks, v, h);
    float sg = spec_g(roughness, n, v, l);

    return sd * sf * sg / (4.0 * nl * nv);
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

vec3 tone_map(in vec3 col, in float l_white) {
    // filmic tonemapping as done in uncharted 2 (plus gamma correction)
    vec3 mapped = filmic_map(col.rgb) / filmic_map(l_white);
    mapped.r = pow(mapped.r, 1.0 / 2.2);
    mapped.g = pow(mapped.g, 1.0 / 2.2);
    mapped.b = pow(mapped.b, 1.0 / 2.2);
	return mapped;
}
