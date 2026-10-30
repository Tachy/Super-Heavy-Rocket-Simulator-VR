// Clouds in the light of the Earth's surfaces (M8 weather and clouds, Task 9): the direct Sun through the cloud shadow
// maps, the Sun's light diffused through the column and the sky's diffuse transmittance at a ground point. Include
// atmosphere.gdshaderinc and clouds.gdshaderinc first. Surfaces and the cloud pass's march (the cloud shadows on the
// air, cloud_pass.gdshaderinc cloud_air_shadow_loss); never the shadow kernel (it writes the maps sampled here).
//
// Shadow maps (CloudShadowEffect, before the opaque pass; cloud_baked.gdshaderinc CLOUD_SHADOW_BAKED_FILL): orthographic along the
// Sun, a plane perpendicular to it through an anchor near the ground under the camera (Earth-fixed between re-anchors:
// no swimming), axes e1, e2. A texel holds, along its line toward the Sun (parameter t from the plane, t = (p - A) . s),
// the first and last t with cloud, the optical depth between and the cover C (the beam cover of the largest cloud fraction on the line;
// 1 for the shapes): a receiver at t_r gets 1 - C + C exp(-od / C x clamp((t_back - t_r) / (t_back - t_front))) -
// exact below the clouds (the ground), linear inside them. Both maps (the near one +-16 km at 62.5 m texels, the far
// one +-200 km) hold the baked volume's Sun transmittance S - the shapes it draws (C = 1), the field's slanted column
// where a line's floor point lies beyond its levels; beyond the far map the column slanted to the Sun,
// 1 - C + C exp(-tau / (C mu0)). Distance rule (horizontal distance of
// the receiver from the camera; CloudLayer.SHADOW_*): near -> far between 9 and 11 km, far -> column between 120 and
// 140 km, smooth weights: no pop, and a ground shadow near the camera matches the cloud above it. Both maps are world
// space: the same for both VR eyes.

global uniform sampler2D cloud_shadow_near : filter_nearest, repeat_disable;
global uniform sampler2D cloud_shadow_far : filter_nearest, repeat_disable;
global uniform bool cloud_shadow_valid;          // the maps hold this frame's frame (CloudShadowEffect ran)
global uniform vec3 cloud_shadow_near_anchor;    // anchor - camera, Earth-local render axes (m)
global uniform vec3 cloud_shadow_far_anchor;
global uniform vec3 cloud_shadow_e1;             // map axes (unit, perpendicular to the Sun)
global uniform vec3 cloud_shadow_e2;
global uniform vec3 cloud_shadow_sun;            // the Sun direction the maps were made for (unit)
// (near half extent m, far half extent m, near fade start m, near fade end m)
global uniform vec4 cloud_shadow_extent;
global uniform vec2 cloud_shadow_far_fade;       // (far fade start m, far fade end m)

// The transmittance of one map at the receiver (offset from the camera): the four texels' transmittances, bilinear
// (the exponential filtered, not the depths).
float cloud_shadow_map_transmittance(sampler2D map, vec3 anchor, float half_extent, vec3 offset_local) {
	vec3 d = offset_local - anchor;
	vec2 size = vec2(textureSize(map, 0));
	vec2 xy = (vec2(dot(d, cloud_shadow_e1), dot(d, cloud_shadow_e2)) / half_extent * 0.5 + 0.5) * size - 0.5;
	xy = clamp(xy, vec2(0.0), size - 1.001);
	vec2 i0 = floor(xy);
	vec2 f = xy - i0;
	float t_r = dot(d, cloud_shadow_sun);
	float t[4];
	for (int k = 0; k < 4; k++) {
		vec4 texel = texelFetch(map, ivec2(i0) + ivec2(k & 1, k >> 1), 0);
		float fraction = clamp((texel.g - t_r) / max(texel.g - texel.r, 1.0e-3), 0.0, 1.0);
		float cover = max(texel.a, CLOUD_MIN_FRACTION);
		t[k] = 1.0 - cover + cover * exp(-texel.b * fraction / cover);
	}
	return mix(mix(t[0], t[1], f.x), mix(t[2], t[3], f.x), f.y);
}

// The direct Sun's cloud transmittance at the receiver (offset from the camera; column: its column depth and cover,
// mu0: the Sun's cosine to its vertical) by the distance rule above.
float cloud_shadow_transmittance(vec3 offset_local, vec2 column, float mu0) {
	float analytic = cloud_column_direct(column, mu0);
	if (!cloud_shadow_valid) {
		return analytic;
	}
	vec3 up_c = normalize((atmosphere_camera_hi + atmosphere_camera_lo) * ATM_STRETCH);
	float horizontal = length(offset_local - up_c * dot(offset_local, up_c));
	float w_near = 1.0 - smoothstep(cloud_shadow_extent.z, cloud_shadow_extent.w, horizontal);
	float w_far = 1.0 - smoothstep(cloud_shadow_far_fade.x, cloud_shadow_far_fade.y, horizontal);
	float result = analytic;
	if (w_near < 1.0 && w_far > 0.0) {
		float far = cloud_shadow_map_transmittance(cloud_shadow_far, cloud_shadow_far_anchor, cloud_shadow_extent.y,
				offset_local);
		result = mix(analytic, far, w_far);
	}
	if (w_near > 0.0) {
		float near = cloud_shadow_map_transmittance(cloud_shadow_near, cloud_shadow_near_anchor, cloud_shadow_extent.x,
				offset_local);
		result = mix(result, near, w_near);
	}
	return result;
}

// The cloud light at a ground point (offset from the camera, Earth-local render axes) for the light from light_dir
// (unit): x = the direct beam's transmittance (shadow maps for the Sun, sun_maps true; the column for the Moon),
// y = the light's diffuse part through the column per unit of its horizontal irradiance (isotropic downward),
// z = the sky's diffuse transmittance; w = the light's cosine to the geodetic vertical.
vec4 cloud_ground_light(vec3 offset_local, vec3 light_dir, bool sun_maps) {
	vec3 p = atmosphere_camera_hi + atmosphere_camera_lo + offset_local;
	vec3 up = cloud_geodetic_up(p);
	float mu0 = dot(up, light_dir);
	vec2 column = cloud_column_mean_depth(cloud_lat_lon(p), cloud_height_of(p));
	float direct = sun_maps ? cloud_shadow_transmittance(offset_local, column, mu0) : cloud_column_direct(column, mu0);
	return vec4(direct, cloud_sun_diffuse(column, mu0), cloud_sky_factor(column), mu0);
}
