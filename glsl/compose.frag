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
const float min_diffuse = 0.2;

/*vec3 mat_specular = vec3(0.1, 0.1, 0.1);*/

// light parameters
const vec3 light_emission = vec3(1.0, 1.0, 1.0);

vec3 unpack_normal(uvec3 gbuffer, out float roughness);
void unpack_colors(uvec3 gbuffer, out vec3 mat_diffuse, out vec3 mat_specular);
float unpack_depth(uvec3 gbuffer);
vec3 tone_map(in vec3 col, in float l_white);
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

    out_color = vec4(mat_diffuse, 1.0);
}
