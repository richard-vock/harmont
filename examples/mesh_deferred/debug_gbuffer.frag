#version 430

layout(location = 0) uniform sampler2D map_gbuffer;

const float pi = 3.14159265358979;
const float piInv = 0.318309886183791;

in vec2 tc;
out vec4 frag_color;

vec3 hsv2rgb(float h, float s, float v) {
	float a = 2.0 * pi * h / (pi / 3.f);
	float c = floor(a);
	float f = a - c;
	float p = v * (1-s);
	float q = v * (1-s*f);
	float t = v * (1-s*(1-f));

	float r, g, b;

	if (c == 1) {
		r = q; g = v; b = p;
	} else if (c == 2) {
		r = p; g = v; b = t;
	} else if (c == 3) {
		r = p; g = q; b = v;
	} else if (c == 4) {
		r = t; g = p; b = v;
	} else if (c == 5) {
		r = v; g = p; b = q;
	} else {
		r = v; g = t; b = p;
	}

	return vec3(r, g, b);
}

void main(void) {
    vec3 gbuffer = texture2D(map_gbuffer, tc).rgb;
    if (gbuffer.r == 0.0 && gbuffer.g == 0.0 && gbuffer.b == 0.0) {
        frag_color = vec4(0.0, 0.0, 0.0, 0.0);
        return;
    }

    vec4 g_part = unpackSnorm4x8(uint(gbuffer.g));
    vec2 n_xy = g_part.xy;
    float z_sign = sign(g_part.z);
    float denom = 1.0 + n_xy.x*n_xy.x + n_xy.y*n_xy.y;
    vec3 normal = vec3(2.0*n_xy.x / denom, 2.0*n_xy.y / denom, (-1.0 + n_xy.x*n_xy.x + n_xy.y*n_xy.y) / denom);
    normal.z *= z_sign;
    /*vec4 g_part = decode_rgba(gbuffer.g);*/
    /*vec3 normal = g_part.xyz * 2.0 - 1.0;*/
    /*float z_sign = sign(float((g_part >> 2*8) & 255) / 128.0 - 1.0);*/
    /*vec3 normal = g_part.rgb;*/

    /*float denom = 1.0 + nx*nx + ny*ny;*/
    /*vec3 normal = vec3(2.0*nx / denom, 2.0*ny / denom, (-1.0 + nx*nx + ny*ny) / denom);*/
    /*normal.z *= z_sign;*/

    float phi = (atan(normal.z, normal.x) + pi) / (2.0 * pi);
    float theta = 1.0 - acos(normal.y) / pi;
    //float angle = atan(normal.z / normal.x) / (2.0 * pi);
	//normal_color = (hsv2rgb(angle, 1.0, 1.0), 1.0)
	//vec3 normal_color = vec3(red_part, 0.0, 1.0 - red_part);
    frag_color = vec4(hsv2rgb(phi, 1.0, theta), 1.0);
    /*frag_color = vec4(vec3(g_part.z, 0.0, 0.0), 1.0);*/
    /*uint packed_rgb = uint(gbuffer.r);*/
    /*vec3 rgb = unpackSnorm4x8(packed_rgb).rgb;*/
    /*frag_color = vec4(rgb, 1.0);*/
}
