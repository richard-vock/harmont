#version 130

const float pi = 3.14159265358979;

const float shoulder_s = 0.22;
const float linear_s = 0.3;
const float linear_a = 0.1;
const float toe_s = 0.2;
const float toe_n = 0.01;
const float toe_d = 0.3;

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

vec3 sample_hdr(in vec3 dir, in sampler2D map_environment) {
    vec2 uv;
	if (dot(dir, vec3(0,0,1)) >=  0.9999999) uv = vec2(1.0, 0.0);
    else if (dot(dir, vec3(0,0,1)) <= -0.9999999) uv = vec2(1.0, 1.0);
    else uv = clamp(vec2(0.5f*(1.f + atan(dir.x,-dir.y) / pi), acos(dir.z) / pi), 0.f, 1.f);
    return texture(map_environment, uv).rgb;
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
