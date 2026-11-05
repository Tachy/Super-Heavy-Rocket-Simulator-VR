// Near-field Sun shadows on the Earth (moon procedural surface plan Task 16; EarthNearShadow, docs/subsystems/
// terrain-rendering.md "Near-field Sun shadows"): the Sun's DirectionalLight3D casts Godot's directional shadow map
// (PSSM, PCSS softened by the solar disc's angular diameter) from the pad structures and vehicles around the eye;
// terrain_earth.gdshader receives it through its light() function, as the Moon's terrain does (moon_near_shadow.gdshaderinc).
// The material is lit (not unshaded) only for that: everything it shaded before goes to EMISSION, except the direct Sun's
// part inside the map's range, which goes to light() as a fragment-to-light varying, so the shadow multiplies exactly the
// direct Sun (after the atmosphere's and the clouds' transmittance and the aerial perspective's transmittance to the eye),
// never the sky light, the Moon or the in-scattered light:
//     direct Sun seen = D x [w + (1 - w) S] = D w (EMISSION) + D (ATTENUATION - w) (light()),
// D the unshadowed direct Sun radiance at the eye, S the map's lit fraction, w Godot's own fade of the shadow over the
// range's last part (smoothstep of the view depth from fade_from to fade_to; EarthNearShadow sets both) and Godot's
// ATTENUATION = w + (1 - w) S.
// Off (earth_near_shadow.x 0: the fidelity setting, the Moon's shadows active, the eye high, the Sun down): w = 1,
// nothing goes to light(); the lit pipeline adds no ambient, specular or light term (ambient_light_disabled,
// specular_disabled, SPECULAR 0, ALBEDO 1 only as light()'s gate): the image is the unshaded shader's.

// (on: 1 / 0, fade_from (m of view depth), fade_to (m), unused) - EarthNearShadow, per frame.
global uniform vec4 earth_near_shadow;

varying vec3 earth_near_radiance;  // D x (1 - w): the direct Sun's part the map gates, already seen at the eye
varying float earth_near_fade_w;  // w
varying vec3 earth_near_sun_view;  // the Sun's direction in view space (identifies the Sun's light in light())

// w at view depth depth_m (1 with the near shadows off).
float earth_near_fade(float depth_m) {
	if (earth_near_shadow.x < 0.5) {
		return 1.0;
	}
	return smoothstep(earth_near_shadow.y, earth_near_shadow.z, depth_m);
}

// The receivers' light() (varyings passed in): the Sun's gated part, D (ATTENUATION - w) = radiance x (ATT - w) / (1 - w).
vec3 earth_near_light(vec3 radiance, float fade, vec3 sun_view, vec3 light_direction, bool directional,
		float attenuation) {
	if (fade < 1.0 && directional && dot(light_direction, sun_view) > 0.9999) {
		return radiance * clamp((attenuation - fade) / (1.0 - fade), 0.0, 1.0);
	}
	return vec3(0.0);
}
