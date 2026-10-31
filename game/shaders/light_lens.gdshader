shader_type spatial;
render_mode unshaded, blend_add, cull_disabled, depth_draw_never, fog_disabled;
// A lamp's lens as a camera-facing disc, shown at the luminance its luminous intensity gives: L = I / A_projected
// (cd / m^2), x atmosphere_scene_per_cd_m2 (the one exposure of the game, incl. the eye adaptation's gain). Far away
// the disc stops shrinking at a minimum angle (the far emissive point) and its luminance falls by the area ratio, so
// the flux the eye receives stays I / d^2 (a point source's). A faint halo (bloom of the optics) round the core.

global uniform float atmosphere_scene_per_cd_m2;

uniform float intensity_cd = 0.0;       // luminous intensity toward the viewer, 0 = off
uniform vec3 tint : source_color = vec3(1.0);  // linear colour, max channel 1
uniform float lens_m = 0.3;             // lens diameter
uniform float min_angle_rad = 0.0006;   // smallest apparent diameter
const float HALO_SCALE = 6.0;           // quad width / core width
const float HALO_SHARE = 0.03;          // [E] cosmetic: share of the core's peak in the halo's centre

varying float core_luminance_cd;

void vertex() {
	MODELVIEW_MATRIX = VIEW_MATRIX * mat4(INV_VIEW_MATRIX[0], INV_VIEW_MATRIX[1], INV_VIEW_MATRIX[2], MODEL_MATRIX[3]);
	float dist = length(MODELVIEW_MATRIX[3].xyz);
	float core_m = max(lens_m, dist * min_angle_rad);
	VERTEX.xy *= core_m * HALO_SCALE;  // the quad mesh is 1 x 1
	core_luminance_cd = intensity_cd / (0.7853982 * core_m * core_m);
}

void fragment() {
	float r = length(UV - vec2(0.5)) * 2.0;  // 0 centre .. 1 the quad's edge
	float core = 1.0 - smoothstep(0.9 / HALO_SCALE, 1.15 / HALO_SCALE, r);
	float halo = HALO_SHARE * pow(max(1.0 - r, 0.0), 3.0) / (1.0 + 90.0 * r * r);
	ALBEDO = tint * core_luminance_cd * atmosphere_scene_per_cd_m2 * (core + halo);
	ALPHA = 1.0;
}
