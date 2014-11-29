#version 430

layout(location = 0) uniform sampler2D map_gbuffer;
layout(location = 1) uniform int       width;
layout(location = 2) uniform int       height;
layout(location = 3) uniform float     near;
layout(location = 4) uniform float     far;
layout(location = 5) uniform float     frustum_width;
layout(location = 6) uniform float     frustum_height;
layout(location = 7) uniform mat4      inv_view_proj_matrix;

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

vec3 unpack_normal(vec3 gbuffer);

void main(void) {
    vec3 gbuffer = texture2D(map_gbuffer, tc).rgb;
    if (gbuffer.r == 0.0 && gbuffer.g == 0.0 && gbuffer.b == 0.0) {
        frag_color = vec4(1.0, 0.0, 0.0, 0.0);
        return;
    }

    vec4 b_part = unpackSnorm4x8(uint(gbuffer.b));
    vec3 real_pos = b_part.rgb;
    /*real_pos.xyz /= real_pos.w;*/


    vec3 pos = gbuffer;
    vec4 window_pos = vec4(tc * 2.0 - 1.0, pos.z, 1.0);
    vec4 world_pos = inv_view_proj_matrix * window_pos;
    world_pos.xyz /= world_pos.w;
    float error = length(pos.x - world_pos.z);
    /*float z = pos.z;*/
    /*vec3 clip = vec3((gl_FragCoord.x/width - 0.5) * 2.0, (-gl_FragCoord.y/height + 0.5) * 2.0 / (width / height), z);*/
    /*vec3 clip = vec3((gl_FragCoord.x/width - 0.5) * 2.0 / (10.0 * frustum_width), (-gl_FragCoord.y/height + 0.5) * 2.0 / (width / height), z);*/
    /*float norm_x = (pos.x - near) / (far-near);*/
    /*vec3 clip = vec3((gl_FragCoord.x/width - 0.5) * 2.0, (-gl_FragCoord.y/height + 0.5) * 2.0, z);*/
    /*float l = 0.5 * (clip.y + 1.0);*/
    /*clip.x *= clip.z;*/
    /*clip.y *= -clip.z;*/
    /*vec3 pos = inv_view_mat * vec4(clip, 1.0);*/
    /*pos.xyz /= pos.w;*/

    /*z = real_pos.z;*/
    /*float ndc_z = (-z - near) / (far - near);//length(clip - real_pos);*/
    /*float real_z = (-z - near) / (far - near);//length(clip - real_pos);*/
    /*float real_z = 0.5 * (real_pos.z + 1.0);*/
    /*z = real_pos.z;*/
    /*float real_z = (-z - near) / (far - near);*/
    /*float ndc_y = pos.y / (frustum_height * float(height));*/
    float v = error;//abs(clip.x - pos.x);
    frag_color = vec4(v, v, v, 1.0);

    /*vec3 normal = unpack_normal(gbuffer);*/

    /*float phi = (atan(normal.z, normal.x) + pi) / (2.0 * pi);*/
    /*float theta = 1.0 - acos(normal.y) / pi;*/
    /*frag_color = vec4(hsv2rgb(phi, 1.0, theta), 1.0);*/
}

vec3 unpack_normal(vec3 gbuffer) {
    vec4 g_part = unpackSnorm4x8(uint(gbuffer.g));
    vec2 n_xy = g_part.xy;
    float z_sign = sign(g_part.z);
    float denom = 1.0 + n_xy.x*n_xy.x + n_xy.y*n_xy.y;
    vec3 normal = vec3(2.0*n_xy.x / denom, 2.0*n_xy.y / denom, (-1.0 + n_xy.x*n_xy.x + n_xy.y*n_xy.y) / denom);
    normal.z *= z_sign * sign(normal.z);
    return normal;
}
