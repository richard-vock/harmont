#version 430

layout(location = 0) uniform sampler2D map_ssdo;


in vec2 tc;
out vec4 frag_color;

void main(void) {
    vec3 col = texture(map_ssdo, tc).rgb;
    frag_color = vec4(col, 1.0);
}
