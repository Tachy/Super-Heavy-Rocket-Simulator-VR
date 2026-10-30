// The cloud pass's compute kernels (M8 weather and clouds, Task 8): GLSL 450 only - compiled through CloudShaderPort,
// never included by a Godot shader. CLOUD_MARCH (with atmosphere.gdshaderinc, clouds.gdshaderinc, cloud_far and
// cloud_light before this file - their `global uniform`s translated into set 0 - and cloud_baked.gdshaderinc after
// it, CLOUD_BAKED_MARCH; CloudBaked builds these kernels) or CLOUD_UPSAMPLE (CloudEffect) is defined per kernel;
// CLOUD_CENTRE marks the VR centre view's march.
//
// The march runs once per frame in the centre view (VR: the head, a frustum covering both eyes, CloudCentreView,
// uniform in angle per axis; the desktop: the view itself), the upsample once per eye (VR performance part 2):
// 1. CLOUD_MARCH at reduced resolution (the quality preset's divisor): one ray per low-resolution pixel through the
//    centre of its block of full-resolution pixels; the scene depth of the block in a checkerboard: the nearest of the
//    block on even pixels, the farthest on odd ones (both depths are represented for the upsampling) - unless the
//    block's distances lie within CLOUD_ONE_SURFACE_SPREAD of each other (one surface: the ground seen from far, a
//    slope): then the mean, the centre ray's own distance (Task 10). The ray is shaded through the baked volume
//    (cloud_baked_shade): fixed sample points, an unchanged view gives a bit-identical image. Out: premultiplied
//    colour + the background's transmittance T = 1 - alpha (every image of the pass holds T in .a: its half float
//    resolves the direct beam of the Sun's disk to T ~ 1e-7, where 1 - T had lost it below 2.4e-4; every stage is
//    linear in it), and (march distance used, transmittance-weighted cloud distance).
// 2. CLOUD_UPSAMPLE to full resolution, per eye: a 4 x 4 Catmull-Rom filter of the low-resolution pixels x a depth
//    weight exp(-CLOUD_DEPTH_SHARPNESS |ln(D / D_i)|) (D the full-resolution scene distance, D_i the sample's march
//    distance): a cloud in front of a mountain does not bleed over it and the sky between peaks keeps its cloud. The
//    desktop's view gathers on its own grid; a VR eye reprojects its ray into the centre image - at its scene point,
//    then at the taps' cloud distance (the disparity of the cloud). Out: the full-resolution image the composite quad
//    (cloud_layer.gdshader) blends over the scene.

#if defined(CLOUD_MARCH) || defined(CLOUD_UPSAMPLE)

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

// The view's parameters (CloudEffect._view_block).
layout(set = 1, binding = 0, std140) uniform CloudView {
	mat4 inv_projection;       // clip (the renderer's: reversed Z in [0, 1], 1 = near; y down) -> view space
	mat4 view_to_world;        // the eye's transform (render world; the floating origin is the camera)
	mat4 world_to_earth;       // render world axes -> Earth-local axes (rotation)
	vec4 sizes;                // full width, full height, low width, low height
	vec4 eye_earth;            // xyz: the eye's offset from the head (camera), Earth-local axes (m); 0 on the desktop
	vec4 params;               // x: 1 = the VR centre image is equiangular (CloudCentreView), w: march pixel angle [rad]
} cloud_view;

// Distance [m] from the camera (the head) of a pixel's depth (reversed Z: 1 near, 0 far = no geometry) through a
// projection's inverse and its image size.
float cloud_scene_distance_in(mat4 inv_projection, vec2 size, vec2 pixel_centre, float depth) {
	if (depth <= 0.0) {
		return 1.0e30;
	}
	vec2 ndc = pixel_centre / size * 2.0 - 1.0;  // the renderer's projection: y down (flipped)
	vec4 v = inv_projection * vec4(ndc, depth, 1.0);
	return length(v.xyz / v.w);
}

// Distance [m] from the camera of a full-resolution pixel's depth of this view.
float cloud_scene_distance(vec2 pixel_centre, float depth) {
	return cloud_scene_distance_in(cloud_view.inv_projection, cloud_view.sizes.xy, pixel_centre, depth);
}

// The head-space unit direction through a clip-space ndc of this view's projection.
vec3 cloud_ray_view_ndc(vec2 ndc) {
	// The renderer's per-eye projection already contains the eye offset (inverse: clip -> head space), so one unprojected
	// point is not on the eye's ray. Two depths of the same pixel are: their difference is the eye's ray direction.
	vec4 a = cloud_view.inv_projection * vec4(ndc, 0.5, 1.0);
	vec4 b = cloud_view.inv_projection * vec4(ndc, 1.0, 1.0);
	return normalize(a.xyz / a.w - b.xyz / b.w);
}

// The equiangular centre image (CloudCentreView, cloud_view.params.x = 1): the image coordinate u in
// [-1, 1] per axis is uniform in the angle atan(t) of the head-space tangent t = c + h ndc, which the projection makes
// linear in its ndc; returns that ndc. c, h from the projection's corner rays (CloudCentreView.warp: the eyes' upsample
// inverts it with the same values).
vec2 cloud_equiangular_ndc(vec2 u) {
	vec3 rp = cloud_ray_view_ndc(vec2(1.0));
	vec3 rm = cloud_ray_view_ndc(vec2(-1.0));
	vec2 tp = rp.xy / max(-rp.z, 1.0e-6);
	vec2 tm = rm.xy / max(-rm.z, 1.0e-6);
	vec2 c = 0.5 * (tp + tm);
	vec2 h = 0.5 * (tp - tm);
	vec2 angle = mix(atan(c - h), atan(c + h), 0.5 * (u + 1.0));
	return (tan(angle) - c) / h;
}

// Head-space (view) unit direction of the ray through a full-resolution position.
vec3 cloud_ray_view(vec2 pixel_centre) {
	vec2 ndc = pixel_centre / cloud_view.sizes.xy * 2.0 - 1.0;  // the renderer's projection: y down (flipped)
	if (cloud_view.params.x > 0.5) {
		ndc = cloud_equiangular_ndc(ndc);
	}
	return cloud_ray_view_ndc(ndc);
}

// Earth-local unit direction of the ray through a full-resolution position.
vec3 cloud_ray_earth(vec2 pixel_centre) {
	vec3 dir_world = normalize(mat3(cloud_view.view_to_world) * cloud_ray_view(pixel_centre));
	return normalize(mat3(cloud_view.world_to_earth) * dir_world);
}

#endif

// =====================================================================================================================
#ifdef CLOUD_MARCH

layout(set = 1, binding = 1) uniform sampler2D scene_depth;  // full resolution, nearest
layout(rgba16f, set = 1, binding = 2) uniform writeonly image2D out_color;
layout(rg32f, set = 1, binding = 3) uniform writeonly image2D out_depth;

const float CLOUD_MIN_SCALED_TRANSMITTANCE = 0.01;
// [E] The Sun's disk is seen through the cloud at its physical radiance (its centre 8.67e4 on the scene scale, the
// mean 6.9e4: 3e5 times a lit ground's), so the background transmittance of rays toward it is marched on past the
// scaled criterion (which leaves exp(-tau) ~ 5e-4 of the disk) down to CLOUD_SUN_MIN_TRANSMITTANCE (the disk x that is
// < 1e-4 of a lit ground) - the transmittance alone, without light samples. Those rays: within CLOUD_SUN_RAY_ANGLE of
// the Sun (3 x the disk's radius 4.65e-3: the march pixel and the upsample).
const float CLOUD_SUN_MIN_TRANSMITTANCE = 1.0e-9;
const float CLOUD_SUN_RAY_ANGLE = 0.014;
// [E] A block whose nearest and farthest scene distances differ by less than this (ln of their ratio) shows one
// surface: its centre ray is marched to the block's mean distance. The checkerboard's nearest depth cut that ray short
// by up to the block's depth extent - from 8 600 km a slanted 2 x 2 block of ground spans several km, so every other
// pixel lost the low clouds in front of the ground (Task 10). An occluder's edge (a mountain against the sky or the
// far terrain) spans far more and keeps the checkerboard. 0.05: the upsampling's depth weight
// exp(-CLOUD_DEPTH_SHARPNESS x 0.05) = 0.82 cannot tell such depths apart either.
const float CLOUD_ONE_SURFACE_SPREAD = 0.05;
// [E] The eclipse's factor on the sky light (atm_sky_eclipse_factor: costly - evaluated at every light refresh it took
// the descent's p99 from 14.5 to 25 ms): once per 8 x 8 work group at two points of the group's centre ray, 5 km and
// 150 km away, linear in the distance between them (held beyond). The penumbra spans ~3500 km from first contact to
// totality (~3 % of obscuration per 100 km), the umbra ~200 km: neighbouring pixels share the factor.
const float CLOUD_ECLIPSE_NEAR_M = 5000.0;
const float CLOUD_ECLIPSE_FAR_M = 150000.0;
shared vec3 cloud_group_eclipse_near;
shared vec3 cloud_group_eclipse_far;

const float CLOUD_FAR_COVER_FOOTPRINT_SCALE = 2.0;
const float CLOUD_FAR_COVER_UNALIGNED_SCALE = 1.37;
const mat3 CLOUD_FAR_COVER_UNALIGNED_ROTATION = mat3(0.8130186879, 0.5112918472, -0.2785341274, -0.4537591358,
		0.8561682215, 0.2471408976, 0.3648331945, -0.0745427634, 0.9280841107);
const float CLOUD_FAR_COVER_ALONG_MAX_M = 32000.0;

// The ray dir (Earth-local unit) from a point offset from the camera (Earth-local, m): the camera's ray (cloud_ray) -
// its origin is an eye at most a few centimetres from the head (cloud_vr_stereo_test), the head's height and up.
CloudRay cloud_ray_from(vec3 dir_local, vec3 origin_offset) {
	CloudRay ray = cloud_ray(dir_local);
	return ray;
}

// ---- Cloud shadows on the air (static clouds Task 9b, ruling 2026-10-27 "Wolkenschatten auf die Luft"). The air
// light the tables give (atmosphere.gdshaderinc) is the clear sky's: every point of a view ray is lit by the Sun's
// direct beam. Under and beside clouds that beam is cut: a backlit cumulus is seen through the shadowed haze in front
// of it, and the gaps of a broken deck at a low Sun send crepuscular rays across the sky. Physics: of the air light
// only the single scattering of the direct beam (single Rayleigh: atmosphere_single_rayleigh, single Mie:
// atmosphere_single_mie) is weighted by the beam's cloud transmittance T_c at the scattering point; the multiple
// scattering (the sky's diffuse light, the deck's glow) is unchanged. The loss along the view ray,
//     L = sum_i (1 - T_c(p_i)) x [T(x, s_i-1) S1(s_i-1) - T(x, s_i) S1(s_i)] x V(s_i)
// in Bruneton's difference form per segment (as the eclipse's atm_segmented_scattering: unweighted it telescopes to
// the plain single-scattering lookup), S1 = S1_R x P_R + S1_M x P_M x the scene's Mie fade (smoothstep(0, 0.01, mu_s)
// at the CAMERA, as atm_to_point_model and atm_sky: the unweighted sum is then exactly the scene's single
// scattering), V = 1 in front of the clouds' weighted distance and the background's transmittance behind it (the
// composite dims the scene's air behind a cloud by T and adds the air in front of it at weight 1: the march's
// alpha x atm_to_point_offset to the clouds' distance; the segment holding that distance split linearly). T_c from
// the baked volume's S (cloud_baked_air_sun: the shapes it draws, +-410 km; the slanted column beyond), averaged over
// fixed points of the segment (CLOUD_AIR_SHADOW_SUBSAMPLES): world space, the same for both VR eyes, no jitter
// (static). The pass's colour carries -L (negative where the air is shadowed); the composite adds it to the scene.
// Segments: CLOUD_AIR_SHADOW_SEGMENTS over the part of the ray below the highest cloud top of the loaded weather
// (cloud_air_shadow_top_m: environment::cloud_render_top_m; no cloud above it shadows the air: the Sun above the
// point's horizon looks up), at d0 + (d1 - d0) (i / n)^2 - denser near the camera, where the air is densest along a
// low ray.
global uniform sampler3D atmosphere_single_rayleigh : filter_linear, repeat_disable;
// Fidelity flag (CloudLayer.air_shadow): false keeps the clear sky's air light in front of and between the clouds.
global uniform bool cloud_air_shadow;
// The highest cloud top of the bound snapshots (CloudLayer._bind_slots; 0: no cloud), metres above the ellipsoid.
global uniform float cloud_air_shadow_top_m;
// [E] 16 segments x 2 points: 32 x 3 cost ~5 ms of the ~8 ms march at divisor 2 in the stereo mock (2026-10-29). T_c
// per segment is the mean of CLOUD_AIR_SHADOW_SUBSAMPLES points spread evenly over it (fixed per pixel): one midpoint
// aliased the shadows into regular stripes across the sky (the segment spacing against the shadows' spacing).
const int CLOUD_AIR_SHADOW_SEGMENTS = 16;
const int CLOUD_AIR_SHADOW_SUBSAMPLES = 2;

// The single scattering (unphased Rayleigh, unphased Mie) at (r, mu, mu_s, nu) (atm_combined_scattering's lookup).
void cloud_single_scattering(float r, float rho, float mu, float mu_s, float nu, bool ground, out vec3 rayleigh,
		out vec3 mie) {
	vec4 uvwz = atm_scattering_uvwz(r, rho, mu, mu_s, nu, ground);
	float nu_size = atmosphere_scattering_size.x;
	float tex_coord_x = uvwz.x * (nu_size - 1.0);
	float tex_x = floor(tex_coord_x);
	float lerp_nu = tex_coord_x - tex_x;
	vec3 uvw0 = vec3((tex_x + uvwz.y) / nu_size, uvwz.z, uvwz.w);
	vec3 uvw1 = vec3((tex_x + 1.0 + uvwz.y) / nu_size, uvwz.z, uvwz.w);
	rayleigh = mix(textureLod(atmosphere_single_rayleigh, uvw0, 0.0).rgb,
			textureLod(atmosphere_single_rayleigh, uvw1, 0.0).rgb, lerp_nu);
	mie = mix(textureLod(atmosphere_single_mie, uvw0, 0.0).rgb, textureLod(atmosphere_single_mie, uvw1, 0.0).rgb,
			lerp_nu);
}

// The baked volume's own Sun transmittance (cloud_baked.gdshaderinc S): false beyond its levels. Its lookup footprint
// (m at the band's floor), set per air sample by cloud_air_shadow_parts: the samples' spacing along the ray (times the
// rate its Sun line moves at the floor) and the pixel's footprint there, the latter over the Sun's sine (a pixel step moves the line's floor point by that much
// more under a low Sun) - the shapes' shadows prefiltered at it, not aliased into stripes between pixel rows.
float cloud_baked_air_footprint_m = 0.0;
#ifdef CLOUD_BAKED_DIAG_AIR_SHARP
const float CLOUD_BAKED_AIR_MIN_SUN_SINE = 1.0;  // (diagnostics: the pixel's footprint alone, no low-Sun growth)
#else
const float CLOUD_BAKED_AIR_MIN_SUN_SINE = 0.1;  // [E] the footprint's growth stops below a ~6 deg Sun
#endif
bool cloud_baked_air_sun(vec3 offset_local, float height_m, vec3 sun_dir, out float t);

// The direct Sun's cloud transmittance at an air point (offset from the camera, Earth-local render axes): the baked
// volume's S within its levels (the shapes it draws, +-410 km), the field's slanted column beyond.
float cloud_air_sun_transmittance(vec3 offset_local, vec3 sun_dir) {
	vec3 p = atmosphere_camera_hi + atmosphere_camera_lo + offset_local;
	float height_m = cloud_height_of(p);
	if (height_m > CLOUD_TOP_M) {
		return 1.0;
	}
	float baked_t;
	if (cloud_baked_air_sun(offset_local, height_m, sun_dir, baked_t)) {
		return baked_t;
	}
	return cloud_column_direct(cloud_column_mean_depth(cloud_lat_lon(p), height_m), dot(cloud_geodetic_up(p), sun_dir));
}

// The view ray's geometry for the air shadow: the stretch of the ray below the highest cloud top up to end_m (render
// metres: the scene or the ray's end), the camera's model-space point and angles. False: nothing to shadow - no cloud,
// the ray misses the layer, or the camera lies above the atmosphere's top (r > top radius: the tables' parametrisation
// needs the ray moved to its top-of-atmosphere entry, as atm_sky does; not done - an open point: the loss switches
// off above ~100 km).
struct CloudAirRay {
	float from_m;
	float to_m;
	float scale;  // model metres per render metre along the ray
	float r;
	float rho;
	float mu;
	float mu_s;
	float nu;
	bool ground;
	float phase_r;
	float phase_m;  // times the scene's Mie fade at the camera
};

bool cloud_air_ray(CloudRay ray, vec3 dir, vec3 eye, float end_m, out CloudAirRay a) {
	a.from_m = 0.0;
	a.to_m = 0.0;
	float top_m = min(cloud_air_shadow_top_m, CLOUD_TOP_M);
	if (!cloud_air_shadow || !atmosphere_enabled || top_m <= 0.0) {
		return false;
	}
	vec2 top = cloud_sphere_crossing(ray, top_m);
	if (top.x > top.y || top.y <= 0.0) {
		return false;
	}
	a.from_m = max(top.x, 0.0);
	a.to_m = min(top.y, end_m);
	if (a.to_m <= a.from_m) {
		return false;
	}
	vec3 up;
	atm_point_model(eye, up, a.r, a.rho);
	if (a.r > atmosphere_top_radius_m) {
		return false;
	}
	vec3 v_stretched = dir * ATM_STRETCH;
	a.scale = length(v_stretched);
	vec3 v = v_stretched / a.scale;
	vec3 sun = normalize(atmosphere_sun_direction * ATM_STRETCH);
	a.mu = atm_clamp_cosine(dot(up, v));
	a.mu_s = atm_clamp_cosine(dot(up, sun));
	a.nu = atm_clamp_cosine(dot(v, sun));
	a.ground = atm_ray_intersects_ground(a.r, a.rho, a.mu);
	a.phase_r = atm_rayleigh_phase(a.nu);
	// Bruneton's fade of the single Mie term with the Sun below the horizon, at the camera (atm_to_point_model).
	a.phase_m = atm_mie_phase(atmosphere_mie_phase_g, a.nu) * smoothstep(0.0, 0.01, a.mu_s);
	return true;
}

// T(x, s) S1(s) at the distance at_m (render metres) along the ray: the phased single scattering seen from the camera.
vec3 cloud_air_single_at(CloudAirRay a, float at_m) {
	float R = atmosphere_bottom_radius_m;
	float d = at_m * a.scale;
	float h = clamp(atm_height_along(a.r, a.rho, a.mu, d), 0.0, atmosphere_top_radius_m - R);
	float r_d = R + h;
	float mu_d = atm_clamp_cosine((a.r * a.mu + d) / r_d);
	float mu_s_d = atm_clamp_cosine((a.r * a.mu_s + d * a.nu) / r_d);
	vec3 t_d = d > 0.0 ? atm_transmittance(a.r, a.rho, a.mu, d, a.ground) : vec3(1.0);
	vec3 rayleigh;
	vec3 mie;
	cloud_single_scattering(r_d, atm_rho_from_height(h), mu_d, mu_s_d, a.nu, a.ground, rayleigh, mie);
	return t_d * (rayleigh * a.phase_r + mie * a.phase_m);
}

// The air light the clouds' shadows take away along the ray dir from the eye (offset from the camera) up to end_m,
// scene units, in two parts: `front` in front of the clouds' weighted distance front_m (seen directly) and `behind`
// beyond it (unweighted: the caller multiplies it by the background's transmittance T). Loss = front + T x behind.
// Zero without clouds in the Sun's way.
void cloud_air_shadow_parts(CloudRay ray, vec3 dir, vec3 eye, float end_m, float front_m, out vec3 front,
		out vec3 behind) {
	front = vec3(0.0);
	behind = vec3(0.0);
	CloudAirRay a;
	if (!cloud_air_ray(ray, dir, eye, end_m, a)) {
		return;
	}
	vec3 sun_dir = normalize(atmosphere_sun_direction);
	vec3 up_c = normalize((atmosphere_camera_hi + atmosphere_camera_lo) * ATM_STRETCH);
	vec3 s_prev = vec3(0.0);
	float prev_m = a.from_m;
	float inv_n = 1.0 / float(CLOUD_AIR_SHADOW_SEGMENTS);
	// How fast the air point's Sun line moves at the floor per metre along the ray: dir less its part along the Sun
	// line (dir - sun (dir . up) / mu_s, horizontal) - looking along the Sun's azimuth the far samples share lines.
	float floor_rate = length(dir - sun_dir * dot(dir, up_c) / max(dot(sun_dir, up_c), CLOUD_BAKED_AIR_MIN_SUN_SINE));
	for (int i = 0; i <= CLOUD_AIR_SHADOW_SEGMENTS; i++) {
		float f = float(i) * inv_n;
		float at_m = a.from_m + (a.to_m - a.from_m) * f * f;
		vec3 s_d = cloud_air_single_at(a, at_m);
		if (i > 0) {
			float shadow = 0.0;
			for (int k = 0; k < CLOUD_AIR_SHADOW_SUBSAMPLES; k++) {
				float at_k = mix(prev_m, at_m, (float(k) + 0.5) / float(CLOUD_AIR_SHADOW_SUBSAMPLES));
				cloud_baked_air_footprint_m = max((at_m - prev_m) / float(CLOUD_AIR_SHADOW_SUBSAMPLES) * floor_rate,
						at_k * cloud_view.params.w / max(a.mu_s, CLOUD_BAKED_AIR_MIN_SUN_SINE));
#ifdef CLOUD_BAKED_DIAG_AIR_NO_LOD
				cloud_baked_air_footprint_m = 0.0;  // (diagnostics: the finest level at every air sample)
#endif
				shadow += 1.0 - cloud_air_sun_transmittance(eye + dir * at_k, sun_dir);
			}
			shadow /= float(CLOUD_AIR_SHADOW_SUBSAMPLES);
			if (shadow > 0.0) {
				if (atmosphere_eclipse_segments > 0) {
					shadow *= atm_sunlit(atmosphere_camera_hi + atmosphere_camera_lo + eye
							+ dir * (0.5 * (prev_m + at_m)));
				}
				vec3 piece = shadow * (s_prev - s_d);
				// The segment's part in front of the clouds' distance seen directly, the rest through T: linear in
				// front_m (a step at the midpoint stair-stepped the loss along the clouds' distance contours).
				float seen = clamp((front_m - prev_m) / max(at_m - prev_m, 1.0e-3), 0.0, 1.0);
				front += seen * piece;
				behind += (1.0 - seen) * piece;
			}
		}
		s_prev = s_d;
		prev_m = at_m;
	}
	// No clamp per segment: a piece the tables' interpolation makes negative (long near-horizon segments, the setting
	// Sun) belongs to the telescoping sum - clamped, the sum exceeded the scene's single scattering by up to 8 % (Sun
	// 4 deg, view 1 deg up) and 10-50 % at a 0.3-deg Sun (cloud_check section 14). The parts as a whole are >= 0.
	front = atm_to_scene(max(front, vec3(0.0)));
	behind = atm_to_scene(max(behind, vec3(0.0)));
}

// The loss with the background's transmittance behind_t (see cloud_air_shadow_parts).
vec3 cloud_air_shadow_loss(CloudRay ray, vec3 dir, vec3 eye, float end_m, float front_m, float behind_t) {
	vec3 front;
	vec3 behind;
	cloud_air_shadow_parts(ray, dir, eye, end_m, front_m, front, behind);
	return front + behind_t * behind;
}

// The work group's eclipse factors on the sky light (CLOUD_ECLIPSE_NEAR_M / _FAR_M) along its centre ray dir from
// origin (offset from the camera): probe points above the ground (a ray that meets it is probed short of it).
void cloud_eclipse_factors(vec3 dir, vec3 origin, out vec3 near_factor, out vec3 far_factor) {
	near_factor = vec3(1.0);
	far_factor = vec3(1.0);
	if (cloud_enabled && atmosphere_enabled && atmosphere_eclipse_segments > 0) {
		vec3 camera_point = atmosphere_camera_hi + atmosphere_camera_lo + origin;
		vec2 ground = cloud_sphere_crossing(cloud_ray_from(dir, origin), 0.0);
		float limit_m = ground.x <= ground.y && ground.x > 0.0 ? ground.x : CLOUD_ECLIPSE_FAR_M;
		near_factor = atm_sky_eclipse_factor(camera_point + dir * min(CLOUD_ECLIPSE_NEAR_M, limit_m));
		far_factor = atm_sky_eclipse_factor(camera_point + dir * min(CLOUD_ECLIPSE_FAR_M, limit_m));
	}
}

// What the images store of a ray's colour and transmittance: the half-float range (a lit cloud seen through a
// dark-adapted eye; negative: the air shadow's loss, cloud_air_shadow_loss), no NaN into the history, and in .a the
// background's transmittance T, not alpha: a half float resolves T relatively (normal down to 6.1e-5, subnormal to
// 6e-8), where 1 - T near 1 had a spacing of 4.9e-4 - the Sun's disk vanished behind a slant tau of 8.
vec4 cloud_stored_color(vec3 color, float transmittance) {
	color = clamp(color, vec3(-65000.0), vec3(65000.0));
	if (any(isnan(color)) || isnan(transmittance)) {
		color = vec3(0.0);
		transmittance = 1.0;
	}
	return vec4(color, clamp(transmittance, 0.0, 1.0));
}

// The baked volume's ray (cloud_baked.gdshaderinc, compiled after this file).
void cloud_baked_shade(vec3 dir, vec3 eye, float scene_m, out vec4 color_t, out float cloud_m);

#ifdef CLOUD_CENTRE
// The centre view has no depth buffer of its own: its rays take the scene distance from an eye's (VR performance
// part 2).
layout(set = 1, binding = 4) uniform sampler2D scene_depth_right;  // `scene_depth` is the left eye's
layout(set = 1, binding = 5, std140) uniform CloudEyes {
	mat4 clip_from_head[2];  // the eyes' projections (the renderer's: eye offset and rotation inside)
	mat4 inv_projection[2];
	vec4 eye_sizes;          // xy: the eyes' full-resolution size
} cloud_eyes;

// [E] Where a centre ray is projected into an eye to find its depth texels: far, so the texel is the eye's look along
// the same direction; nearer geometry is then read with a parallax of at most IPD/2 / D (the distance from the head
// differs by at most IPD/2: cloud_vr_stereo_test), which the eyes' own upsample (their depth) absorbs at full
// resolution.
const float CLOUD_CENTRE_PROBE_M = 1.0e6;
// [E] Samples per axis of an eye's depth texels under one centre pixel (strided beyond).
const int CLOUD_CENTRE_GATHER_MAX = 6;

// The ndc of a head-space direction's far point in an eye; w <= 0 (behind the eye): far outside.
vec2 cloud_eye_ndc(int eye, vec3 dir_view) {
	vec4 c = cloud_eyes.clip_from_head[eye] * vec4(dir_view * CLOUD_CENTRE_PROBE_M, 1.0);
	return c.w > 0.0 ? c.xy / c.w : vec2(1.0e9);
}

// Nearest, farthest, sum and count of the scene distances under a low-resolution centre pixel: read in the eye whose
// ndc holds the pixel's centre ray (the left one first), over the eye texels its corners project to.
void cloud_block_distances(ivec2 pixel, vec2 scale, inout float near_m, inout float far_m, inout float sum_m,
		inout float count) {
	vec3 dir = cloud_ray_view(min((vec2(pixel) + 0.5) * scale, cloud_view.sizes.xy));
	int eye = all(lessThanEqual(abs(cloud_eye_ndc(0, dir)), vec2(1.0))) ? 0 : 1;
	vec2 size = cloud_eyes.eye_sizes.xy;
	vec2 lo = vec2(1.0e30);
	vec2 hi = vec2(-1.0e30);
	for (int j = 0; j <= 1; j++) {
		for (int i = 0; i <= 1; i++) {
			vec2 corner = min(vec2(pixel + ivec2(i, j)) * scale, cloud_view.sizes.xy);
			vec2 px = (clamp(cloud_eye_ndc(eye, cloud_ray_view(corner)), vec2(-1.0), vec2(1.0)) * 0.5 + 0.5) * size;
			lo = min(lo, px);
			hi = max(hi, px);
		}
	}
	ivec2 full = ivec2(size);
	ivec2 first = clamp(ivec2(floor(lo)), ivec2(0), full - 1);
	ivec2 last = clamp(ivec2(ceil(hi)) - 1, first, full - 1);
	ivec2 stride = max((last - first + CLOUD_CENTRE_GATHER_MAX) / CLOUD_CENTRE_GATHER_MAX, ivec2(1));
	for (int y = first.y; y <= last.y; y += stride.y) {
		for (int x = first.x; x <= last.x; x += stride.x) {
			float depth = eye == 0 ? texelFetch(scene_depth, ivec2(x, y), 0).r
					: texelFetch(scene_depth_right, ivec2(x, y), 0).r;
			float dist = cloud_scene_distance_in(cloud_eyes.inv_projection[eye], size, vec2(x, y) + 0.5, depth);
			near_m = min(near_m, dist);
			far_m = max(far_m, dist);
			sum_m += dist;
			count += 1.0;
		}
	}
}
#else
// Nearest, farthest, sum and count of the scene distances of the block of full-resolution pixels this pixel covers.
void cloud_block_distances(ivec2 pixel, vec2 scale, inout float near_m, inout float far_m, inout float sum_m,
		inout float count) {
	ivec2 full = ivec2(cloud_view.sizes.xy);
	ivec2 first = ivec2(vec2(pixel) * scale);
	ivec2 last = min(ivec2(vec2(pixel + 1) * scale) - 1, full - 1);
	for (int y = first.y; y <= last.y; y++) {
		for (int x = first.x; x <= last.x; x++) {
			float dist = cloud_scene_distance(vec2(x, y) + 0.5, texelFetch(scene_depth, ivec2(x, y), 0).r);
			near_m = min(near_m, dist);
			far_m = max(far_m, dist);
			sum_m += dist;
			count += 1.0;
		}
	}
}
#endif

void main() {
	ivec2 low = ivec2(cloud_view.sizes.zw);
	vec2 scale = cloud_view.sizes.xy / cloud_view.sizes.zw;
	ivec2 pixel = ivec2(gl_GlobalInvocationID.xy);
	// The work group's eclipse factors on the sky light (cloud_eclipse_factors), before any invocation leaves.
	if (gl_LocalInvocationIndex == 0u) {
		vec2 group_centre = (vec2(gl_WorkGroupID.xy * gl_WorkGroupSize.xy) + 0.5 * vec2(gl_WorkGroupSize.xy)) * scale;
		vec3 group_dir = cloud_ray_earth(min(group_centre, cloud_view.sizes.xy - 0.5));
		cloud_eclipse_factors(group_dir, cloud_view.eye_earth.xyz, cloud_group_eclipse_near, cloud_group_eclipse_far);
	}
	barrier();
	if (pixel.x >= low.x || pixel.y >= low.y) {
		return;
	}
	vec2 centre = (vec2(pixel) + 0.5) * scale;
	bool nearest = ((pixel.x + pixel.y) & 1) == 0;
	float near_m = 1.0e30;
	float far_m = 0.0;
	float sum_m = 0.0;
	float count = 0.0;
	cloud_block_distances(pixel, scale, near_m, far_m, sum_m, count);
	float scene_m = nearest ? near_m : far_m;
	if (far_m < 1.0e29 && far_m <= near_m * exp(CLOUD_ONE_SURFACE_SPREAD)) {
		scene_m = sum_m / count;  // one surface: the centre ray's distance
	}
	// One ray through the pixel's centre from the eye (the centre view: the head).
	vec4 color_t;
	float cloud_m;
	cloud_baked_shade(cloud_ray_earth(centre), cloud_view.eye_earth.xyz, scene_m, color_t, cloud_m);  // baked volume
	imageStore(out_color, pixel, color_t);
	imageStore(out_depth, pixel, vec4(scene_m, cloud_m, 0.0, 0.0));
}

#endif

// =====================================================================================================================
#ifdef CLOUD_UPSAMPLE

layout(set = 1, binding = 1) uniform sampler2D scene_depth;    // full resolution, nearest
layout(set = 1, binding = 2) uniform sampler2D low_color;      // the march's colour (of the centre view), nearest
layout(set = 1, binding = 3) uniform sampler2D low_depth;      // the march distances (of the centre view), nearest
layout(rgba16f, set = 1, binding = 4) uniform writeonly image2D out_image;
// The centre view the march image belongs to (CloudEffect): its transform from Earth-local offsets of the head to its
// clip space, its sizes (virtual full, low) and params: x the mode (1 = an eye reads the centre (VR), 0 = the view is
// the centre: the desktop's own pixel grid).
layout(set = 1, binding = 5, std140) uniform CloudCentre {
	mat4 clip_from_earth;
	vec4 sizes;
	vec4 params;
	vec4 warp;  // the equiangular centre image: (c_x, h_x, c_y, h_y), tangent = c + h ndc (CloudCentreView.warp); 0: off
} cloud_centre;

// [E] Sharpness of the depth weight: a factor e^-1 per 1/4 of a e-fold distance ratio (|ln(D / D_i)| = 0.25: a
// sample 28 % nearer or farther weighs 37 %); equal distances (the sky, a far cloud deck over the sea) weigh 1.
const float CLOUD_DEPTH_SHARPNESS = 4.0;
// [E] The tent radius [centre pixels] of an eye's first gather (its taps' cloud distance: where it reads the centre).
const float CLOUD_TENT_RADIUS_MAX = 2.0;
// [E] Taps whose depth weights differ by more than this ratio (one below it times the largest) show two surfaces: the
// gather keeps only its kernel's positive part (static clouds Task 8 fix round 1, the dotted horizon row).
const float CLOUD_DEPTH_MIXED_WEIGHT = 0.5;
// [E] The upsample's colour floor per T (cloud_tent_gather): a tap's T below this counts as this (a body tap with a
// negative colour - none in theory - does not make the floor unbounded).
const float CLOUD_FLOOR_MIN_T = 1.0e-3;
// Distance where an eye pixel without scene geometry is reprojected into the centre (the far sky: rotation only).
const float CLOUD_UPSAMPLE_SKY_M = 1.0e7;

// Catmull-Rom's 1-D weight at distance |x| [low pixels] (Keys 1981, a = -0.5): interpolating (1 at 0, 0 at the other
// texel centres), C1, support 2 - a march pixel's binary edge becomes a ramp of about one march pixel with smooth
// corners instead of bilinear's stair steps; its negative lobes are clamped in cloud_tent_gather.
float cloud_catmull_rom(float x) {
	x = abs(x);
	return x < 1.0 ? (1.5 * x - 2.5) * x * x + 1.0 : (x < 2.0 ? ((-0.5 * x + 2.5) * x - 4.0) * x + 2.0 : 0.0);
}

// The centre's low-resolution position (texel units, texel centres at +0.5) of an Earth-local offset from the head,
// clamped to the image (an eye's edge pixel outside the centre reads its edge).
vec2 cloud_centre_position(vec3 offset) {
	vec4 clip = cloud_centre.clip_from_earth * vec4(offset, 1.0);
	vec2 ndc = clip.w > 0.0 ? clip.xy / clip.w : vec2(0.0);
	if (cloud_centre.warp.y != 0.0) {
		// the equiangular centre image (cloud_equiangular_ndc's inverse)
		vec2 c = cloud_centre.warp.xz;
		vec2 h = cloud_centre.warp.yw;
		vec2 lo = atan(c - h);
		ndc = 2.0 * (atan(c + h * ndc) - lo) / (atan(c + h) - lo) - 1.0;
	}
	vec2 low = cloud_centre.sizes.zw;
	return clamp((ndc * 0.5 + 0.5) * low, vec2(0.5), low - 0.5);
}

// The gather at a centre position: premultiplied colour + T weighted by the kernel (radius > 0: a tent of that radius
// per axis; 0: Catmull-Rom) and the depth weight; `cloud` gets the alpha-weighted cloud distance of the taps (0
// without cloud).
vec4 cloud_tent_gather(vec2 position, float scene_m, float radius, out float cloud) {
	ivec2 low = ivec2(cloud_centre.sizes.zw);
	ivec2 base = ivec2(floor(position - 0.5)) - 1;
	vec4 sum = vec4(0.0);
	float weights = 0.0;
	vec4 sum_positive = vec4(0.0);  // the kernel's positive part alone (CLOUD_DEPTH_MIXED_WEIGHT)
	// The alpha 1 - T too: the Catmull-Rom's weights cancel (sum w << sum |w|), so the normalised sum of T = 1 taps
	// came out up to ~5e-4 below 1 where the scene depth moves (the sea's waves: the clear sky flickered by one
	// half-float step), while a sum of alpha = 0 stays exactly 0. T below 0.5 is taken from the sum of T (the Sun's disk
	// behind a deck: relative precision), above from 1 - the sum of alpha.
	float sum_alpha = 0.0;
	float sum_alpha_positive = 0.0;
	float weights_positive = 0.0;
	float depth_weight_min = 1.0;
	float depth_weight_max = 0.0;
	float cloud_sum = 0.0;
	float cloud_weights = 0.0;
	vec4 best = vec4(0.0, 0.0, 0.0, 1.0);  // no cloud (T 1)
	float best_error = 1.0e30;
	// The taps' colours and depth weights: the lobes' undershoot floor (below)
	vec4 tap_color[16];
	float tap_depth_weight[16];
	for (int j = 0; j <= 3; j++) {
		for (int i = 0; i <= 3; i++) {
			ivec2 q = clamp(base + ivec2(i, j), ivec2(0), low - 1);
			vec2 d = abs(vec2(base + ivec2(i, j)) + 0.5 - position);
			float tent = radius > 0.0 ? max(0.0, 1.0 - d.x / radius) * max(0.0, 1.0 - d.y / radius)
					: cloud_catmull_rom(d.x) * cloud_catmull_rom(d.y);  // radius 0: Catmull-Rom
			vec4 c = texelFetch(low_color, q, 0);
			vec2 depths = texelFetch(low_depth, q, 0).rg;
			float march_m = min(depths.r, 1.0e20);
			float error = abs(log(max(scene_m, 1.0) / max(march_m, 1.0)));
			float depth_weight = exp(-CLOUD_DEPTH_SHARPNESS * error);
			float w = tent * depth_weight;
			sum += w * c;
			sum_alpha += w * (1.0 - c.a);
			weights += w;
			tap_color[j * 4 + i] = c;
			tap_depth_weight[j * 4 + i] = tent > 0.0 ? depth_weight : -1.0;
			if (tent > 0.0) {
				sum_positive += w * c;
				sum_alpha_positive += w * (1.0 - c.a);
				weights_positive += w;
			}
			if (tent != 0.0) {
				depth_weight_min = min(depth_weight_min, depth_weight);
				depth_weight_max = max(depth_weight_max, depth_weight);
			}
			if (depths.g > 0.0 && w > 0.0) {
				cloud_sum += w * (1.0 - c.a) * depths.g;  // .a: T
				cloud_weights += w * (1.0 - c.a);
			}
			if (tent > 0.0 && error < best_error) {
				best_error = error;
				best = c;
			}
		}
	}
	cloud = cloud_weights > 1.0e-6 ? cloud_sum / cloud_weights : 0.0;
	// Taps of two depths (the march's nearest / farthest checkerboard along a depth edge - the horizon seen from
	// altitude): the Catmull-Rom's negative lobes, renormalised over the taps of one depth, alternate with the
	// checkerboard and dotted the edge's row; its positive part alone (the nearest texels) is used there.
	if (depth_weight_min < CLOUD_DEPTH_MIXED_WEIGHT * depth_weight_max) {
		sum = sum_positive;
		sum_alpha = sum_alpha_positive;
		weights = weights_positive;
	}
	vec4 result = weights > 1.0e-4 ? sum / weights : best;
	if (weights > 1.0e-4 && result.a >= 0.5) {
		result.a = 1.0 - sum_alpha / weights;
	}
	result.a = clamp(result.a, 0.0, 1.0);
	// The Catmull-Rom lobes can over- / undershoot: premultiplied colour >= the lowest tap's (0 without air shadow:
	// the air shadow's loss is negative), T in [0, 1]. The floor from the taps of this pixel's depth only (depth
	// weight within CLOUD_DEPTH_MIXED_WEIGHT of the best): a near object's silhouette against the shadowed sky took
	// the sky's larger loss as its floor. And a tap's negative colour belongs to its T x scene (the loss of the air
	// the scene holds: the sky's -L at T 1, a cloud body's colour >= 0 at T 0), so the floor is also this pixel's T
	// times the taps' lowest colour per T: a body pixel next to the shadowed sky took the sky's -L through the filter
	// (final T x scene + colour < 0 at T ~ 0; cloud_check section 14). Both from the inner 2 x 2 taps (the texels
	// around the pixel): the outer ring's negative colour let the lobes' undershoot next to a bright cloud darken the
	// shadowed sky by 4-12 % of its loss, where without air shadow the floor 0 clamped it.
	vec3 lowest = vec3(0.0);
	vec3 lowest_per_t = vec3(0.0);
	for (int k = 0; k < 16; k++) {
		bool inner = (k & 3) == 1 || (k & 3) == 2;
		inner = inner && (k >> 2 == 1 || k >> 2 == 2);
		if (inner && tap_depth_weight[k] >= CLOUD_DEPTH_MIXED_WEIGHT * depth_weight_max) {
			lowest = min(lowest, tap_color[k].rgb);
			lowest_per_t = min(lowest_per_t, tap_color[k].rgb / max(tap_color[k].a, CLOUD_FLOOR_MIN_T));
		}
	}
	return vec4(max(result.rgb, max(lowest, result.a * lowest_per_t)), result.a);
}

void main() {
	ivec2 pixel = ivec2(gl_GlobalInvocationID.xy);
	ivec2 full = ivec2(cloud_view.sizes.xy);
	if (pixel.x >= full.x || pixel.y >= full.y) {
		return;
	}
	float scene_m = min(cloud_scene_distance(vec2(pixel) + 0.5, texelFetch(scene_depth, pixel, 0).r), 1.0e20);
	if (cloud_centre.params.x > 0.5) {
		// An eye reads the shared centre image at its own ray: first at its scene point, then again at the taps' cloud
		// distance (the cloud's own disparity; cloud_vr_stereo_test).
		vec3 dir = cloud_ray_earth(vec2(pixel) + 0.5);
		vec3 eye = cloud_view.eye_earth.xyz;
		float cloud_m;
		cloud_tent_gather(cloud_centre_position(eye + dir * min(scene_m, CLOUD_UPSAMPLE_SKY_M)), scene_m,
				CLOUD_TENT_RADIUS_MAX, cloud_m);
		float at_m = cloud_m > 0.0 ? min(cloud_m, scene_m) : min(scene_m, CLOUD_UPSAMPLE_SKY_M);
		imageStore(out_image, pixel, cloud_tent_gather(cloud_centre_position(eye + dir * at_m), scene_m, 0.0,
				cloud_m));
		return;
	}
	// The desktop: the gather on the view's own grid (the centre is the view).
	vec2 position = (vec2(pixel) + 0.5) * cloud_view.sizes.zw / cloud_view.sizes.xy;
	float cloud_m;
	imageStore(out_image, pixel, cloud_tent_gather(position, scene_m, 0.0, cloud_m));
}

#endif
