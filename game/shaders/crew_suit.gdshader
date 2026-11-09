shader_type spatial;
render_mode ambient_light_disabled;
// The crew suit (Task 8b, docs/subsystems/cockpit.md "Crew body and suit"): the cabin look of cabin.gdshader (Godot PBR under the cabin lamps
// and the Sun, aerial perspective as FOG, no sky ambient; the walls' bounced light as the emission term `fill` and their reflection
// `fill_surround` through the split-sum BRDF, both set by CockpitView each frame) with the suit's baked maps (tools/structures/suit_textures.py):
// - suit_albedo: the suit's surface detail on the UV atlas (sRGB);
// - suit_wrinkle: the joint folds at full bend: RG = slopes dh/du, dh/dv (0.5 + slope / slope_range, m of height per m of atlas), B = fold
//   cavity. A fold zone belongs to one joint: UV2.x = its slot + 1 (0 = none), UV2.y = the zone mask; the joint's wrinkle weight (0 at the
//   straight joint, 1 at full bend: FigureDeform) comes per figure as instance uniforms wrinkle_l / wrinkle_r = (shoulder, elbow, hip, knee).
// The fold normal perturbs NORMAL through the surface gradient of the height (Schueler 2013, "Normal mapping without precomputed tangents":
// the gradients of u and v from screen-space derivatives), so the mesh needs no tangents.

#include "res://shaders/atmosphere.gdshaderinc"

uniform vec4 albedo : source_color = vec4(1.0);
uniform sampler2D suit_albedo : source_color, filter_linear_mipmap_anisotropic, repeat_disable;
uniform sampler2D suit_wrinkle : hint_default_black, filter_linear_mipmap, repeat_disable;
uniform vec2 atlas_m = vec2(2.56, 1.60);  // the atlas in metres of suit surface (UV 1 = this many metres)
uniform float slope_range = 1.6;  // figure_layout.WRINKLE_SLOPE_RANGE
uniform float cavity_shade = 0.25;  // albedo darkening in a fold's valley at full weight (fix round 1: 0.35 read as hard creases)
uniform float roughness : hint_range(0.0, 1.0) = 0.65;
uniform float metallic : hint_range(0.0, 1.0) = 0.0;
uniform float specular : hint_range(0.0, 1.0) = 0.4;
uniform float fill = 0.0;
uniform float fill_surround = 0.0;
instance uniform vec4 wrinkle_l = vec4(0.0);  // (shoulder, elbow, hip, knee) of the crew's left side, slots 0..3
instance uniform vec4 wrinkle_r = vec4(0.0);  // slots 4..7

varying vec3 world_offset;

void vertex() {
	world_offset = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

vec3 env_brdf(vec3 f0, float rough, float n_dot_v) {
	const vec4 c0 = vec4(-1.0, -0.0275, -0.572, 0.022);
	const vec4 c1 = vec4(1.0, 0.0425, 1.04, -0.04);
	vec4 r = rough * c0 + c1;
	float a004 = min(r.x * r.x, exp2(-9.28 * n_dot_v)) * r.x + r.y;
	vec2 ab = vec2(-1.04, 1.04) * a004 + r.zw;
	return f0 * ab.x + ab.y;
}

float joint_weight(float slot_plus_one) {
	int slot = int(slot_plus_one + 0.5) - 1;
	if (slot < 0) {
		return 0.0;
	}
	vec4 side = slot < 4 ? wrinkle_l : wrinkle_r;
	int k = slot % 4;
	return k == 0 ? side.x : (k == 1 ? side.y : (k == 2 ? side.z : side.w));
}

void fragment() {
	vec4 fog;
	vec3 colour_factor;
	atm_fog(world_offset, fog, colour_factor);
	FOG = fog;
	vec3 base = texture(suit_albedo, UV).rgb * albedo.rgb;
	float w = clamp(UV2.y, 0.0, 1.0) * joint_weight(UV2.x);
	vec3 n = normalize(NORMAL);
	// screen-space derivatives in uniform control flow (inside the branch they are undefined at its edges: speckles)
	vec3 dp1 = dFdx(VERTEX);
	vec3 dp2 = dFdy(VERTEX);
	vec2 duv1 = dFdx(UV);
	vec2 duv2 = dFdy(UV);
	vec4 fold = texture(suit_wrinkle, UV);
	if (w > 0.001) {
		vec2 slope = (fold.rg - 0.5) * slope_range;
		vec3 dp2perp = cross(dp2, n);
		vec3 dp1perp = cross(n, dp1);
		float det = dot(dp1, dp2perp);
		if (abs(det) > 1.0e-12) {
			vec3 grad_u = (dp2perp * duv1.x + dp1perp * duv2.x) / det;
			vec3 grad_v = (dp2perp * duv1.y + dp1perp * duv2.y) / det;
			vec3 grad_h = grad_u * slope.x * atlas_m.x + grad_v * slope.y * atlas_m.y;
			n = normalize(n - grad_h * w);
			NORMAL = n;
		}
		base *= 1.0 - cavity_shade * fold.b * w;
	}
	ALBEDO = base * colour_factor;
	ROUGHNESS = roughness;
	METALLIC = metallic;
	SPECULAR = specular;
	vec3 f0 = mix(vec3(0.16 * specular * specular), base, metallic);
	float n_dot_v = clamp(dot(n, normalize(VIEW)), 1.0e-3, 1.0);
	EMISSION = (base * (1.0 - metallic) * fill + env_brdf(f0, roughness, n_dot_v) * fill_surround) * colour_factor;
}
