shader_type spatial;
render_mode cull_disabled;
// Far range of the Pad B tower (LatticeBuilder, beyond MID_END_M): one box per section whose faces carry the lattice as
// an analytic pattern of the detail file's measured numbers - bright corner columns, a dark deck at every brace row, the
// dark column bundle in the middle and the chevron / X braces. Every member's coverage is the box-filtered share of the
// pixel footprint it covers (fwidth of the face coordinates), so a member thinner than a pixel dims towards its mean
// density instead of flickering; the coverage is the alpha (blended: stable under a head turn, a hashed alpha flickered
// 20-60 % more than the geometry it replaces). Seen through the atmosphere like the other structures (atm_fog, as lit_aerial.gdshader).

#include "res://shaders/atmosphere.gdshaderinc"

uniform float face_width_m = 12.0;
uniform float height_m = 12.0;
uniform float panel_m = 4.0;   // brace row pitch
uniform float half_m = 4.9;    // centre line of legs and braces
uniform float leg_m = 2.2;
uniform float brace_m = 0.5;
uniform float deck_half_m = 2.4;  // half width of the deck slabs
uniform float deck_m = 0.8;    // visible thickness of a deck seen through the lattice
uniform float core_m = 3.6;
uniform float blur = 2.0;  // [E] filter width over the pixel footprint (tower_handover_check: shimmer)
uniform float alpha_scale = 1.0;  // coverage multiplier (1: the measured pattern as is)
uniform bool crossed = false;  // X braces instead of chevrons
uniform vec4 steel_colour : source_color = vec4(0.55, 0.58, 0.63, 1.0);
uniform vec4 leg_colour : source_color = vec4(0.69, 0.72, 0.77, 1.0);
uniform vec4 dark_colour : source_color = vec4(0.06, 0.065, 0.07, 1.0);

varying vec3 local_position;
varying vec3 local_normal;
varying vec3 world_offset;  // fragment - the render origin (= the atmosphere's camera), world axes

void vertex() {
	local_position = VERTEX;
	local_normal = NORMAL;
	world_offset = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

// Share of a pixel footprint of width fw (metres) around a line's centre that a line of width w covers; d: distance.
float line_cover(float d, float w, float fw) {
	float lo = max(d - 0.5 * fw, -0.5 * w);
	float hi = min(d + 0.5 * fw, 0.5 * w);
	return clamp((hi - lo) / fw, 0.0, 1.0);
}

void fragment() {
	vec4 fog;
	vec3 colour_factor;
	atm_fog(world_offset, fog, colour_factor);
	FOG = fog;
	float u = abs(local_normal.x) > 0.5 ? local_position.z : local_position.x;
	float y = local_position.y + 0.5 * height_m;
	// Pixel footprint on the face (metres): the derivatives of the face coordinates, widened by blur (a wider filter keeps
	// the mean coverage and calms the head-turn flicker of sub-pixel members).
	float fw = blur * max(length(vec2(dFdx(u), dFdx(y))), length(vec2(dFdy(u), dFdy(y)))) + 1.0e-4;
	if (abs(local_normal.y) > 0.5) {  // top and bottom of the box: open
		ALPHA = 0.0;
		ALBEDO = vec3(0.0);
	} else {
		float au = abs(u);
		// Corner columns.
		float leg = clamp((au + 0.5 * fw - (0.5 * face_width_m - leg_m)) / fw, 0.0, 1.0);
		// Decks: a dark line at every row, the mean share once a pixel spans half a row or more.
		float row = y - round(y / panel_m) * panel_m;
		float to_mean = smoothstep(0.3 * panel_m, 0.8 * panel_m, fw);
		float deck = mix(line_cover(row, deck_m, fw), deck_m / panel_m, to_mean);
		// Braces between the columns.
		float g = y - floor(y / panel_m) * panel_m;
		float brace;
		float mean_brace;
		if (crossed) {
			float slope = panel_m / (2.0 * half_m);
			float k = 1.0 / sqrt(1.0 + slope * slope);
			brace = max(line_cover((g - slope * (u + half_m)) * k, brace_m, fw),
					line_cover((g - slope * (half_m - u)) * k, brace_m, fw));
			mean_brace = 2.0 * length(vec2(2.0 * half_m, panel_m)) * brace_m / (2.0 * half_m * panel_m);
		} else {
			float slope = panel_m / half_m;
			float k = 1.0 / sqrt(1.0 + slope * slope);
			brace = line_cover((g - panel_m * (1.0 - au / half_m)) * k, brace_m, fw);
			mean_brace = length(vec2(half_m, panel_m)) * brace_m / (half_m * panel_m);
		}
		brace = mix(brace, mean_brace, to_mean) * step(au, half_m);
		float core = 0.85 * clamp((0.5 * core_m + 0.5 * fw - au) / fw, 0.0, 1.0);
		float dark = max(deck * step(au, deck_half_m), core);
		float w_leg = leg;
		float w_dark = dark * (1.0 - w_leg);
		float w_brace = brace * (1.0 - w_leg) * (1.0 - w_dark);
		vec3 colour = (leg_colour.rgb * w_leg + dark_colour.rgb * w_dark + steel_colour.rgb * w_brace)
				/ max(w_leg + w_dark + w_brace, 1.0e-4);
		ALBEDO = colour * colour_factor;
		ALPHA = alpha_scale * max(max(leg, dark), brace);
	}
	METALLIC = 0.5;
	ROUGHNESS = 0.55;
}
