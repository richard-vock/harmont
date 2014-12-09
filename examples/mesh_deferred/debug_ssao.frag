#version 430

layout(location = 0) uniform sampler2D map_ssao;


in vec2 tc;
out vec4 frag_color;

void main(void) {
    float l = texture(map_ssao, tc).r;
    frag_color = vec4(l, l, l, 1.0);
}
