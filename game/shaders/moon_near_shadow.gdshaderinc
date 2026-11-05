// Near-field Sun shadows on the Moon (moon procedural surface plan Task 12; MoonNearShadow, docs/subsystems/
// moon-surface.md "Near-field shadows"): the Sun's DirectionalLight3D casts Godot's directional shadow map (PSSM, PCSS
// softened by the solar disc's angular diameter) from the drawn terrain, rocks, vehicles and structures around the eye;
// terrain_moon.gdshader and moon_rock.gdshader receive it through their light() function. The materials are lit
// (not unshaded) only for that: everything they shade goes to EMISSION as before, and the part of the Sun's radiance the
// shadow map gates goes to light() as a fragment-to-light varying, so the near-field shadow multiplies exactly that part:
//     Sun = base x [w x f_stat x V + (1 - w) x f_near x min(V, S)],
// base the Sun's radiance on the fragment without any shadow, V the horizon map's visible fraction of the disc, S the
// shadow map's lit fraction, w = Godot's own fade of the shadow over the range's last part (smoothstep of the view depth
// from fade_from to fade_to, the light's fade_start x max distance to its max distance; MoonNearShadow sets both), f_stat
// the statistical light factor (Smith's G1 of the drawn relief, the drawn rocks' statistical shadows) and f_near the same
// without what the map draws (moon_detail.gdshaderinc). Godot's ATTENUATION is the faded shadow w + (1 - w) S, hence
// (1 - w) min(V, S) = min((1 - w) V, ATTENUATION - w). The earthshine has no shadow map: unchanged.
// Off (moon_near_shadow.x 0, the fidelity setting moon_near_shadows or away from the Moon's ground): w = 1, nothing goes
// to light(), the materials' expressions are the former ones (EMISSION = the former ALBEDO; the lit pipeline adds no
// ambient, specular or light term: ambient_light_disabled, specular_disabled, SPECULAR 0).

// (on: 1 / 0, fade_from (m of view depth), fade_to (m), fill light: 1 / 0) - MoonNearShadow, per frame.
global uniform vec4 moon_near_shadow;
// The eye in the render world (the camera; in VR the head centre): the rocks' size selection in the shadow pass, whose
// MODELVIEW_MATRIX is the light's (MoonNearShadow).
global uniform vec3 moon_eye_world;
// The shadow map's texel (m) in each of the 4 splits, their far view depths (m): MoonNearShadow from Godot's split
// spheres. The relief the map resolves starts at 2 texels (the sampling's Nyquist scale).
global uniform vec4 moon_near_shadow_texel;
global uniform vec4 moon_near_shadow_split;

varying float near_sun_radiance;  // base x f_near, the part the shadow map gates (0: nothing)
varying float near_fade;  // w
varying float near_horizon;  // V
varying vec3 near_sun_view;  // the Sun's direction in view space (identifies the Sun's light in light())

// w at view depth depth_m (1 with the near shadows off).
float moon_near_fade(float depth_m) {
	if (moon_near_shadow.x < 0.5) {
		return 1.0;
	}
	return smoothstep(moon_near_shadow.y, moon_near_shadow.z, depth_m);
}

// The smallest relief wavelength (m) the map resolves at view depth depth_m: 2 texels of the split holding it.
float moon_near_resolved_m(float depth_m) {
	float texel = depth_m < moon_near_shadow_split.x ? moon_near_shadow_texel.x
			: (depth_m < moon_near_shadow_split.y ? moon_near_shadow_texel.y
			: (depth_m < moon_near_shadow_split.z ? moon_near_shadow_texel.z : moon_near_shadow_texel.w));
	return 2.0 * texel;
}

// The receivers' light() (varyings passed in: custom functions do not read them): the Sun's gated part.
float moon_near_light(float radiance, float fade, float horizon, vec3 sun_view, vec3 light_direction, bool directional,
		float attenuation) {
	if (radiance > 0.0 && directional && dot(light_direction, sun_view) > 0.9999) {
		return radiance * min((1.0 - fade) * horizon, max(attenuation - fade, 0.0));
	}
	return 0.0;
}
