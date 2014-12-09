#version 430

float offset[3] = float[](0.0, 1.3846153846, 3.2307692308);
float weight[3] = float[](0.2270270270, 0.3162162162, 0.0702702703);

layout(location = 0) uniform  sampler2D map_input;
layout(location = 1) uniform usampler2D map_gbuffer;
layout(location = 2) uniform int dimension;
layout(location = 3) uniform int width;
layout(location = 4) uniform int height;

in  vec2 tc;
out float blurred;

bool is_background(vec2 tex_coord) {
    uvec3 gbuffer = texture(map_gbuffer, tex_coord).rgb;
    return gbuffer.r == 0 && gbuffer.g == 0 && gbuffer.b == 0;
}

void main (void) {
    if (is_background(tc)) {
		blurred = texture(map_input, tc).r;
		return;
    }
	vec2 unit = vec2(0.0, 0.0);
	float scale = 1.0 / float(width);
	if (dimension == 0) unit.x = 1.0;
	if (dimension == 1) {
		unit.y = 1.0;
		scale = 1.0 / float(height);
	}

	float sum_weights = 0.0;
	blurred = texture(map_input, tc).r * weight[0];
	sum_weights += weight[0];
	for (int i=1; i<3; ++i) {
		vec2 tc_off = tc + unit * offset[i] * scale;
		if (!is_background(tc_off)) {
			blurred += texture(map_input, tc_off) * weight[i];
			sum_weights += weight[i];
		}
		tc_off = tc - unit * offset[i] * scale;
		if (!is_background(tc_off)) {
			blurred += texture(map_input, tc_off) * weight[i];
			sum_weights += weight[i];
		}
	}
	blurred *= 1.0 / sum_weights;
}
