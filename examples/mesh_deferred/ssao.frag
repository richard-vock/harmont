#version 430

layout(location = 0) uniform mat4 modelview_matrix;
layout(location = 1) uniform mat4 projection_matrix;
layout(location = 2) uniform float exponent;
layout(location = 3) uniform float radius;
layout(location = 4) uniform sampler2D map_gbuffer;
layout(location = 5) uniform sampler2D map_samples;
layout(location = 6) uniform sampler2D map_noise;
layout(location = 7) uniform mat4 inv_view_proj_matrix;


in  vec2 tc;
out float light;

float ssao(mat3 local, vec3 pos);
vec3 unpack_normal(vec3 gbuffer, out float roughness);
vec3 view_project(vec3 pos);

void main (void) {
    vec3 gbuffer = texture2D(map_gbuffer, tc).rgb;
    if (gbuffer.r == 0.0 && gbuffer.g == 0.0 && gbuffer.b == 0.0) {
        light = 1.0;
        return;
    }

    // get normal
    float roughness;
    vec3 normal = unpack_normal(gbuffer, roughness);

    // get world position
    vec4 window_pos = vec4(tc * 2.0 - 1.0, gbuffer.r, 1.0);
    vec4 world_pos = inv_view_proj_matrix * window_pos;
    vec3 pos = world_pos.xyz / world_pos.w;

    // compute local coordinate system
    vec3 tangent = abs(normal.x) > 0.99 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
    tangent = normalize(tangent - dot(tangent, normal) * normal);
    mat3 local = transpose(mat3(tangent, cross(normal, tangent), normal));

	light = ssao(local, pos);
}

float ssao(mat3 local, vec3 pos) {
	float occlusion = 0.0;
	vec3 global_sample, local_sample;
	int num_samples = textureSize(map_samples, 0).y;
    /*vec2 tc_noise = tc * (vec2(textureSize(map_gbuffer, 0)) / vec2(textureSize(map_noise, 0)));*/
    /*int variation = int(texture(map_noise, tc_noise).r);*/
    int variation = 0;

	int used_samples = 0;
	for (int i=0; i<num_samples; ++i) {
		local_sample = texelFetch(map_samples, ivec2(i, variation), 0).xyz;
		global_sample = local * local_sample + pos;

		vec3 s = view_project(global_sample);

        vec3 gbuffer_test = texture2D(map_gbuffer, 0.5 * (s.xy + 1.0)).rgb;
        if (gbuffer_test.r == 0.0 && gbuffer_test.g == 0.0 && gbuffer_test.b == 0.0) {
            continue;
        }
        vec4 s_world = inv_view_proj_matrix * vec4(s, 1.0);
        vec3 s_pos = s_world.xyz / s_world.w;
		vec3 v_sample = view_project(s_pos);

		float range_check = length(s_pos - pos) > radius ? 0.0 : 1.0;
		used_samples += int(range_check);
		occlusion += range_check * (v_sample.z < s.z ? 1.0 : 0.0);
	}
	if (used_samples > 1.0) {
		occlusion /= float(used_samples);
	}
	return 1.0 - pow(occlusion, exponent);
}

vec3 unpack_normal(vec3 gbuffer, out float roughness) {
    vec4 g_part = unpackSnorm4x8(uint(gbuffer.g));
    vec2 n_xy = g_part.xy;
    float z_sign = sign(g_part.z);
    roughness = g_part.z * z_sign;
    float denom = 1.0 + n_xy.x*n_xy.x + n_xy.y*n_xy.y;
    vec3 normal = vec3(2.0*n_xy.x / denom, 2.0*n_xy.y / denom, (-1.0 + n_xy.x*n_xy.x + n_xy.y*n_xy.y) / denom);
    normal.z *= z_sign * sign(normal.z);
    return normal;
}

vec3 view_project(vec3 pos) {
    vec4 proj = projection_matrix * modelview_matrix * vec4(pos, 1.0);
    return proj.xyz / proj.w;
}
