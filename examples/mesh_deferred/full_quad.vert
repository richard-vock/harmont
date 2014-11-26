#version 330

in vec3 position;

out vec2 tc;

void main(void) {
	gl_Position  = vec4(position.xyz, 1.0);
	tc = clamp(0.5 * (position.xy + 1.0), 0.0, 1.0);
}
