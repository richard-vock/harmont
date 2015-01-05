#version 430

layout(location = 0) uniform float far;

in vec2 tc;
out vec3 frag_gbuffer;
out vec4 frag_transp_accum;
out float frag_transp_count;
out vec3 frag_ssdo;
out vec3 frag_ssdo_last;

void main(void) {
    frag_gbuffer = vec3(0.0);
    frag_transp_accum = vec4(0.0);
    frag_transp_count = 0.0;
    frag_ssdo = vec3(0.0);
    frag_ssdo_last = vec3(0.0);
}
