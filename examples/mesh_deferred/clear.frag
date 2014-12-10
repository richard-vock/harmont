#version 430

in vec2 tc;
out vec3 frag_gbuffer;
out vec3 frag_ssdo;
out vec3 frag_ssdo_last;

void main(void) {
    frag_gbuffer = vec3(0.0, 0.0, 0.0);
    frag_ssdo = vec3(0.0, 0.0, 0.0);
    frag_ssdo_last = vec3(0.0, 0.0, 0.0);
}
