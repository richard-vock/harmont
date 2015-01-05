#version 430

precision highp float;

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec4 in_color;
layout(location = 2) in vec3 in_normal;
layout(location = 3) in vec2 in_tex_coords;

layout(location = 6) uniform mat3 normal_matrix;
layout(location = 7) uniform bool two_sided;
layout(location = 8) uniform bool has_texture;
layout(location = 9) uniform sampler2D map_tex;
layout(location = 10) uniform sampler2D map_hdr;
layout(location = 11) uniform vec3 eye_dir;
layout(location = 12) uniform vec3 light_dir;

vec3  mat_specular = vec3(0.2, 0.2, 0.2);
float mat_roughness  = 0.7;
const vec3 light_emission = vec3(1.0, 1.0, 1.0);

out vec4 out_accum;
out float out_count;


vec3 sample_hdr(in vec3 dir, in sampler2D map_environment);
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

    vec3 frag_color = in_color.rgb;
    if (has_texture) {
        frag_color *= texture(map_tex, in_tex_coords).rgb;
    }

    vec3 diff = 0.6 * frag_color * sample_hdr(normal, map_hdr);
    vec3 spec = specular(mat_roughness, mat_specular, normal, eye_dir, light_dir) * light_emission;
    float alpha = in_color.a;
    out_accum = vec4(alpha * (diff + spec), alpha);
    out_count = 1.0;
}
