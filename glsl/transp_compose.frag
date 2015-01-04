#version 430

in vec2 tc;

layout(location = 0) uniform sampler2D  map_accum;
layout(location = 1) uniform sampler2D  map_count;
layout(location = 2) uniform float      l_white;
layout(location = 3) uniform vec3       background_color;

out vec4 out_color;

vec3 tone_map(in vec3 col, in float l_white);

void main(void) {
    vec4 accum = texture(map_accum, tc).rgba;
    float count = texture(map_count, tc).r;

    if (count == 0.0) {
        out_color = vec4(background_color, 1.0);
        return;
    }

    vec3 avg_rgb = accum.rgb / accum.a;
    float avg_a  = accum.a / count;

    float T = pow(1.0 - avg_a, count);
    out_color = vec4((1.0 - T) * tone_map(avg_rgb, l_white) + T * background_color, 1.0);
}
