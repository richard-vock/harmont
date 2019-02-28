#version 430

precision highp float;

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec4 in_color;
layout(location = 2) in vec3 in_normal;
layout(location = 3) in vec2 in_tex_coords;

layout(location = 6) uniform mat3 normal_matrix;
layout(location = 7) uniform bool two_sided;
layout(location = 8) uniform sampler2D map_tex;
layout(location = 9) uniform vec3 eye_dir;
layout(location = 10) uniform vec3 light_dir;

vec3  mat_specular = vec3(0.2, 0.2, 0.2);
float mat_roughness  = 0.7;
const vec3 light_emission = vec3(1.0, 1.0, 1.0);

out vec4 out_accum;
out float out_count;


vec3 specular(float roughness, vec3 ks, vec3 n, vec3 v, vec3 l);


void main() {
    vec3 normal = in_normal;
    vec3 es_eye_dir = normalize(in_position);
    if (dot(normal, normal) < 0.2) {
        out_accum = vec4(in_color.rgb * in_color.a, in_color.a);
        out_count = 1.0;
        return;
    } else if (two_sided && dot(normalize(normal_matrix * normal), es_eye_dir) > 0.0) {
        normal *= -1.0;
    }

    vec4 frag_color = in_color;
    frag_color.rgb *= frag_color.a;
    out_accum = frag_color;
    out_count = 1.0;
}
