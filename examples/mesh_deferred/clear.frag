#version 430

in vec2 tc;
out vec3 frag_gbuffer;
out float frag_ssao;

void main(void) {
    frag_gbuffer = vec3(0.0, 0.0, 0.0);
    frag_ssao = 1.0;
}
