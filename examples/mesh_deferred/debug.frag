#version 430
layout(location = 0) in vec3 in_position;

layout(location = 1) uniform sampler2D map_shadow;

layout(location = 0) out vec4 out_color;
void main() {
    float lum = texture2D(map_shadow, 0.5 * (in_position.xy + 1.0));
    out_color = vec4(lum, lum, lum, 1.0);
}
