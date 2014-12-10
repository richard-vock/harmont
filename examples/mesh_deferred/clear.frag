#version 430

in vec2 tc;
out vec3 frag_gbuffer;
out vec3 frag_ssao;
out vec3 frag_ssao_last;

void main(void) {
    frag_gbuffer = vec3(0.0, 0.0, 0.0);
    frag_ssao = vec3(0.0, 0.0, 0.0);
    frag_ssao_last = vec3(0.0, 0.0, 0.0);
}
