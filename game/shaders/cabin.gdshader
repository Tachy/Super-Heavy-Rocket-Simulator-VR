shader_type spatial;
render_mode ambient_light_disabled;
// Matte / painted cabin surfaces of the Crew Dragon (CockpitView): lit_aerial's look (Godot PBR under the Sun's light, aerial
// perspective as FOG) plus the cabin terms. (1) Closed cabin: no sky ambient light and no sky radiance (ambient_light_disabled);
// only the Sun through the windows and the cabin lamps light it directly. The bounced light of the white walls is an emission term:
// diffuse albedo x fill, and the walls' reflection on glossy parts: a uniform surround of luminance fill_surround weighted with the
// split-sum environment BRDF (fix round 5: AO 0.03 on the sky's radiance still left the sky - far brighter than the cabin the eye
// adaptation meters - in every reflection: black seats, brackets and the lower visor read mid grey). (2) Panel seams: thin dark
// lines on a grid in the part's own coordinates (surface detail by shader, not geometry), strength seam_depth (0 = none). Further
// surface detail by uniforms (3)-(7): band, two-tone visor, helmet panel lines, key labels, markings.

#include "res://shaders/atmosphere.gdshaderinc"

uniform vec4 albedo : source_color = vec4(1.0);
uniform float roughness : hint_range(0.0, 1.0) = 0.85;
uniform float metallic : hint_range(0.0, 1.0) = 0.0;
uniform float specular : hint_range(0.0, 1.0) = 0.4;
uniform float fill = 0.0;  // scene value of the cabin's bounced light (E / pi x scene_per_cd_m2), set by CockpitView each frame
uniform float fill_surround = 0.0;  // scene luminance of the walls the glossy parts reflect (fill x wall albedo), set by CockpitView each frame
uniform float seam_depth : hint_range(0.0, 1.0) = 0.0;
uniform vec2 seam_pitch_m = vec2(0.42, 0.30);
uniform float seam_width_m = 0.0015;
// (3) Band: a dark band on one face of a part (the vent across the console's silver back, crew1_1): fragments whose local normal is within
// ~25 deg of band_face_normal and whose position lies in [band_up_range] along band_up (from band_origin) and within band_half_width along Z.
uniform float band_on = 0.0;
uniform vec3 band_origin = vec3(0.0);
uniform vec3 band_up = vec3(1.0, 0.0, 0.0);
uniform vec3 band_face_normal = vec3(0.0, 1.0, 0.0);
uniform vec2 band_up_range = vec2(0.0, 0.0);
uniform float band_half_width = 0.0;
uniform vec4 band_albedo : source_color = vec4(0.0, 0.0, 0.0, 1.0);
// (4) Two-tone surface by UV.y (the visor: copper-tinted top, dark clear glass below; UV.y = 0 at its top edge): tint_split < 0 = off.
uniform float tint_split = -1.0;
uniform vec4 albedo_low : source_color = vec4(0.0, 0.0, 0.0, 1.0);
uniform float roughness_low = 0.1;
uniform float metallic_low = 0.0;
// (5) Panel lines on the helmet (Task 5b, spacex-anzug: the recessed groove from the crown down the side): up to two planes in the part's
// own frame; a line where |n . p - offset| < groove_width / 2 and limit . p < limit_value and |p - groove_centre| < groove_radius
// (the helmet material also covers the suit cuffs). groove_plane[i] = (n, offset), groove_limit[i] = (limit vector, limit value).
uniform int groove_count = 0;
uniform vec4 groove_plane[2];
uniform vec4 groove_limit[2];
uniform vec3 groove_centre = vec3(0.0);
uniform float groove_radius = 0.0;
uniform float groove_width = 0.004;
uniform vec4 groove_albedo : source_color = vec4(0.08, 0.08, 0.085, 1.0);
// (6) Key label (Task 5b): the face of a key (local normal within ~25 deg of label_normal) shows the atlas cell label_cell (u0, v0, u1, v1)
// stretched over the face rectangle label_size about label_centre (axes label_right, label_up); the atlas is LINEAR coverage, the label colour
// label_albedo mixed in by it.
uniform float label_on = 0.0;
uniform sampler2D label_atlas : filter_linear_mipmap, repeat_disable;
uniform vec4 label_cell = vec4(0.0, 0.0, 1.0, 1.0);
uniform vec3 label_centre = vec3(0.0);
uniform vec3 label_right = vec3(0.0, 0.0, 1.0);
uniform vec3 label_up = vec3(1.0, 0.0, 0.0);
uniform vec3 label_normal = vec3(0.0, 1.0, 0.0);
uniform vec2 label_size = vec2(0.024, 0.016);
uniform vec4 label_albedo : source_color = vec4(0.85, 0.85, 0.85, 1.0);
// (7) Marking (Task 5b): a rectangle on the faces whose local normal is near mark_normal, [mark_lat] along mark_right and [mark_up] along
// mark_up from mark_origin: albedo mark_albedo and an emitted scene value mark_emission (the red light strips under the shelf modules,
// the red end of the handle housing). mark_emission is set every frame (scene-referred like the lamps).
uniform float mark_on = 0.0;
uniform vec3 mark_origin = vec3(0.0);
uniform vec3 mark_right = vec3(0.0, 0.0, 1.0);
uniform vec3 mark_up = vec3(1.0, 0.0, 0.0);
uniform vec3 mark_normal = vec3(0.0, 1.0, 0.0);
uniform vec2 mark_lat = vec2(0.0);
uniform vec2 mark_up_range = vec2(0.0);
uniform vec4 mark_albedo : source_color = vec4(0.5, 0.02, 0.02, 1.0);
uniform vec3 mark_emission = vec3(0.0);

varying vec3 world_offset;  // fragment - the render origin (= the atmosphere's camera), world axes
varying vec3 local_position;
varying vec3 local_normal;

void vertex() {
	world_offset = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	local_position = VERTEX;
	local_normal = NORMAL;
}

// Split-sum environment BRDF, analytic fit (Karis 2014, "Physically Based Shading on Mobile"): the fraction of a uniform
// surround a surface with specular colour f0 and this roughness reflects toward the viewer.
vec3 env_brdf(vec3 f0, float rough, float n_dot_v) {
	const vec4 c0 = vec4(-1.0, -0.0275, -0.572, 0.022);
	const vec4 c1 = vec4(1.0, 0.0425, 1.04, -0.04);
	vec4 r = rough * c0 + c1;
	float a004 = min(r.x * r.x, exp2(-9.28 * n_dot_v)) * r.x + r.y;
	vec2 ab = vec2(-1.04, 1.04) * a004 + r.zw;
	return f0 * ab.x + ab.y;
}

float seam(vec3 p, vec3 n) {
	// Two grids along the two axes the surface is not facing.
	vec3 a = abs(n);
	vec2 q = a.x > 0.7 ? p.yz : (a.y > 0.7 ? p.xz : p.xy);
	vec2 cell = q / seam_pitch_m;
	vec2 d = abs(fract(cell + 0.5) - 0.5) * seam_pitch_m;
	float w = max(seam_width_m, 1.0e-5);
	vec2 line = 1.0 - smoothstep(vec2(0.5 * w), vec2(1.5 * w), d);
	return max(line.x, line.y);
}

void fragment() {
	vec4 fog;
	vec3 colour_factor;
	atm_fog(world_offset, fog, colour_factor);
	FOG = fog;
	float line = seam_depth > 0.0 ? seam(local_position, normalize(local_normal)) : 0.0;
	line *= smoothstep(0.12, 0.45, abs(dot(normalize(NORMAL), normalize(VIEW))));  // the 1.5 mm seams alias to moire at grazing angles (pilot_eye ceiling): fade them out there
	vec3 base = albedo.rgb;
	float rough = roughness;
	float metal = metallic;
	if (band_on > 0.0 && dot(normalize(local_normal), band_face_normal) > 0.9) {
		vec3 d = local_position - band_origin;
		float u = dot(d, band_up);
		float aa = max(fwidth(u), 1.0e-4);
		float in_up = smoothstep(band_up_range.x - aa, band_up_range.x + aa, u) * (1.0 - smoothstep(band_up_range.y - aa, band_up_range.y + aa, u));
		float in_lat = 1.0 - smoothstep(band_half_width - aa, band_half_width + aa, abs(d.z));
		base = mix(base, band_albedo.rgb, in_up * in_lat);
		rough = mix(rough, 0.9, in_up * in_lat);  // the vent is a matte dark slot, no metal sheen
		metal = mix(metal, 0.0, in_up * in_lat);
	}
	if (tint_split >= 0.0) {
		float w = fwidth(UV.y) + 0.02;
		float low = smoothstep(tint_split - w, tint_split + w, UV.y);
		base = mix(base, albedo_low.rgb, low);
		rough = mix(rough, roughness_low, low);
		metal = mix(metal, metallic_low, low);
	}
	vec3 glow = vec3(0.0);
	if (groove_count > 0 && length(local_position - groove_centre) < groove_radius) {
		float g = 0.0;
		for (int i = 0; i < min(groove_count, 2); i++) {
			float s = dot(groove_plane[i].xyz, local_position) - groove_plane[i].w;
			float aa = max(fwidth(s), 1.0e-5);
			float in_line = 1.0 - smoothstep(0.5 * groove_width - aa, 0.5 * groove_width + aa, abs(s));
			float lim = dot(groove_limit[i].xyz, local_position) - groove_limit[i].w;
			in_line *= 1.0 - smoothstep(-0.004, 0.004, lim);
			g = max(g, in_line);
		}
		base = mix(base, groove_albedo.rgb, g);
		rough = mix(rough, 0.8, g);
	}
	if (label_on > 0.0 && dot(normalize(local_normal), label_normal) > 0.9) {
		vec3 d = local_position - label_centre;
		vec2 f = vec2(dot(d, label_right) / label_size.x + 0.5, 0.5 - dot(d, label_up) / label_size.y);
		if (all(greaterThanEqual(f, vec2(0.0))) && all(lessThanEqual(f, vec2(1.0)))) {
			float cover = texture(label_atlas, mix(label_cell.xy, label_cell.zw, f)).r;
			base = mix(base, label_albedo.rgb, cover);
		}
	}
	if (mark_on > 0.0 && dot(normalize(local_normal), mark_normal) > 0.9) {
		vec3 d = local_position - mark_origin;
		float u = dot(d, mark_up);
		float l = dot(d, mark_right);
		float aa = max(fwidth(u), 1.0e-4);
		float inside = smoothstep(mark_up_range.x - aa, mark_up_range.x + aa, u) * (1.0 - smoothstep(mark_up_range.y - aa, mark_up_range.y + aa, u))
				* step(mark_lat.x, l) * step(l, mark_lat.y);
		base = mix(base, mark_albedo.rgb, inside);
		metal = mix(metal, 0.0, inside);
		glow = mark_emission * inside;
	}
	ALBEDO = base * (1.0 - seam_depth * 0.55 * line) * colour_factor;
	ROUGHNESS = rough;
	METALLIC = metal;
	SPECULAR = specular;
	vec3 f0 = mix(vec3(0.16 * specular * specular), base, metal);  // Godot's dielectric F0 from SPECULAR
	float n_dot_v = clamp(dot(normalize(NORMAL), normalize(VIEW)), 1.0e-3, 1.0);
	EMISSION = (base * (1.0 - metal) * fill + env_brdf(f0, rough, n_dot_v) * fill_surround + glow) * colour_factor;
}
