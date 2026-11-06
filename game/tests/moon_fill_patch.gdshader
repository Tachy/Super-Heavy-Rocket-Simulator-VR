shader_type spatial;
// Probe of the production fill light and rock lighting functions (moon procedural surface plan Task 13, review m2 of the
// Task 12 re-review 2) for moon_fill_check.gd: one value per mode on a flat unshaded quad (HDR viewport). Local frame:
// up = +Y, east = +X, north = -Z; the eye at the zenith. MoonDetailShading.configure sets the detail uniforms; the check
// sets the horizon map (a constant wall elevation in every sector).
// mode 0: the fill on a horizontal receiver that sees no Sun itself (V = 0: a floor shadowed by the walls) - only the
//         walls' term remains, each wall at its own Sun visibility (Task 12 fix 2, N2);
// mode 1: the same receiver's direct Sun reflectance if it were lit (moon_hapke_norm x I/F), the scale for mode 0;
// mode 2 / 3: moon_rock_lit with the eye below the rock's local horizontal / at the 1 deg floor (Task 12 fix 2, N1).
render_mode unshaded, cull_disabled;

#include "res://shaders/moon_regolith.gdshaderinc"
#include "res://shaders/moon_detail.gdshaderinc"
#include "res://shaders/moon_horizon.gdshaderinc"
#include "res://shaders/moon_rock_light.gdshaderinc"
#include "res://shaders/moon_near_shadow.gdshaderinc"
#include "res://shaders/moon_fill.gdshaderinc"

uniform int mode = 0;
uniform vec3 sun_local = vec3(0.0, 1.0, 0.0);  // unit direction to the Sun in the local frame
uniform float patch_spacing_m = 0.0102;

void fragment() {
	vec3 up = vec3(0.0, 1.0, 0.0);
	vec3 east = vec3(1.0, 0.0, 0.0);
	vec3 north = vec3(0.0, 0.0, -1.0);
	vec3 to_sun = normalize(sun_local);
	float sun_elevation = asin(clamp(dot(to_sun, up), -1.0, 1.0));
	float value = 0.0;
	if (mode == 0) {
		float ground_lit = moon_fill_emitter(1.0, patch_spacing_m, detail_horizon_spacing_m, sun_elevation) * 0.0;
		value = moon_fill_reflectance(up, up, to_sun, 1.0, ground_lit, 0.0, 1.0, true, east, north, vec2(0.5));
	} else if (mode == 1) {
		value = moon_hapke_norm * hapke_reflectance(moon_hapke_albedo_phase, moon_hapke_opposition, dot(up, to_sun), 1.0,
				dot(up, to_sun), moon_hapke_theta_bar_rad);
	} else if (mode == 2) {
		value = moon_rock_lit(sun_elevation, patch_spacing_m, detail_horizon_spacing_m, vec2(-0.2, 0.5));
	} else {
		value = moon_rock_lit(sun_elevation, patch_spacing_m, detail_horizon_spacing_m, vec2(detail_rock_shape.w, 0.5));
	}
	ALBEDO = vec3(value);
}
