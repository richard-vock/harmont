#version 430

layout(location = 0) uniform float far;

in vec2 tc;
out float frag_shadow;

void main(void) {
    frag_shadow = 1.0;
}
