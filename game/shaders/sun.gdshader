shader_type spatial;
// Solar disk with limb darkening (linear law, u = 0.6), emissive for glow.
render_mode unshaded, cull_back;

uniform float intensity = 30.0;

varying vec3 local_normal;

void vertex() {
	local_normal = NORMAL;
}

void fragment() {
	float mu = clamp(dot(normalize(NORMAL), vec3(0.0, 0.0, 1.0)), 0.0, 1.0);  // view space: towards camera
	float limb = 1.0 - 0.6 * (1.0 - mu);
	ALBEDO = vec3(1.0, 0.97, 0.92) * limb * intensity;
}
