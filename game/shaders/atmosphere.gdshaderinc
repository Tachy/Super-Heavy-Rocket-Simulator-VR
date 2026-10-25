// Precomputed atmospheric scattering of the Earth at render time (M8 atmosphere): lookups into the core's tables
// (StarsimCore.load_atmosphere, uploaded by game/world/atmosphere.gd) and Bruneton's runtime functions GetSkyRadiance,
// GetSkyRadianceToPoint and GetSunAndSkyIrradiance in his precomputed-luminance mode. A mirror of the CPU reference
// core/src/environment/atmosphere_scattering.cpp and of the parameterisation in atmosphere_tables.cpp: change them
// together (game/tests/atmosphere_check.gd compares this include with the CPU at sample pixels).
//
// Frames: every *_local vector is in the Earth-local render frame (ITRF in render axes (x, y, z) -> (y, z, x): polar
// axis +Y), metres from the Earth's centre, unscaled (scaled space does not affect it). The WGS84 ellipsoid maps onto
// the model's sphere of radius a by stretching the polar axis by a / b (positions and rays by M = diag(1, a/b, 1) in
// render axes, normals by M^-T), as in the core.
// Units: results are scene values = luminance (cd/m^2) x atmosphere_luminance_scale (one exposure for sky, ground and
// vehicles: day_exposure x pi / E_v,TOA x 1 / d_AU^2 x the eye adaptation's exposure gain, game/world/atmosphere.gd).
// Numerics (float32 on the GPU, floating origin): the camera's height comes from atmosphere_camera_altitude_m
// (differenced in float64 on the CPU); the near-ground terms rho = sqrt(r^2 - R^2) and the ray discriminants are formed
// from heights and (r + R), never from differences of large squares, so the pad's horizon neither jitters nor bands.
// Cameras far outside the atmosphere enter it through the ray's closest-approach distance (a cross product, accurate
// to the camera position's float32 rounding), so the limb seen from the Moon stays in place.
//
// Ported from Eric Bruneton's "Precomputed Atmospheric Scattering: a New Implementation" (2017,
// atmosphere/functions.glsl, https://github.com/ebruneton/precomputed_atmospheric_scattering), distributed under the
// BSD 3-clause licence; his notice follows.
//
// Copyright (c) 2017 Eric Bruneton
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
// following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
//    disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
//    following disclaimer in the documentation and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
//    products derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// ---- Tables (RGBA float, linear filtering, clamp to edge as the CPU's GPU-style sampling).
global uniform sampler2D atmosphere_transmittance : filter_linear, repeat_disable;
global uniform sampler3D atmosphere_scattering : filter_linear, repeat_disable;
global uniform sampler3D atmosphere_single_mie : filter_linear, repeat_disable;
global uniform sampler2D atmosphere_irradiance : filter_linear, repeat_disable;

// ---- Per-frame state (Atmosphere.update).
global uniform vec3 atmosphere_camera_hi;           // camera, Earth-local render frame (m): float32 part
global uniform vec3 atmosphere_camera_lo;           // and the float64 remainder
global uniform float atmosphere_camera_altitude_m;  // above the model ground in the stretched space (float64-derived)
global uniform mat3 atmosphere_world_to_earth;      // render world axes -> Earth-local axes
global uniform vec3 atmosphere_sun_direction;       // Earth-local unit vector to the apparent Sun (unstretched)
global uniform float atmosphere_luminance_scale;    // table luminance (cd/m^2, Sun at 1 AU) -> scene; incl. 1/d_AU^2
global uniform float atmosphere_scene_per_cd_m2;    // a true luminance (cd/m^2) -> scene value (no 1/d_AU^2)
// Eye adaptation's exposure gain (game/world/eye_adaptation.gd; 1 = the day exposure), already contained in the two
// scales above; for what is not lit through the tables (Moon, Sun disc, plumes).
global uniform float atmosphere_exposure_gain;
global uniform bool atmosphere_enabled;             // false until the tables are uploaded

// ---- Model parameters (AtmosphereParameters of the loaded tables, set once on upload).
global uniform float atmosphere_bottom_radius_m;    // model ground (a + ground_altitude_m)
global uniform float atmosphere_top_radius_m;       // a + top_altitude_m
global uniform float atmosphere_mu_s_min;
global uniform float atmosphere_mie_phase_g;
global uniform float atmosphere_sun_angular_radius_rad;
global uniform vec3 atmosphere_sky_radiance_to_luminance;  // table units -> cd/m^2
global uniform vec3 atmosphere_sun_illuminance_lux;  // TOA solar illuminance at 1 AU per channel (solar x sun factor)
global uniform vec4 atmosphere_scattering_size;     // (nu, mu_s, mu, r) cells of the scattering tables
global uniform vec4 atmosphere_table_sizes;         // (transmittance mu, transmittance r, irradiance mu_s, irradiance r)

// a / b of WGS84 (1 / (1 - f), f = 1 / 298.257223563): the polar stretch of the model space (core:
// atmosphere_scattering.cpp polar_stretch).
const float ATM_POLAR_STRETCH = 1.0033640898209764;
const vec3 ATM_STRETCH = vec3(1.0, ATM_POLAR_STRETCH, 1.0);
const float ATM_PI = 3.14159265358979;

float atm_clamp_cosine(float mu) { return clamp(mu, -1.0, 1.0); }
float atm_safe_sqrt(float a) { return sqrt(max(a, 0.0)); }

// The camera, Earth-local (hi + lo; in float32 this is the rounded position - heights use the altitude uniform).
vec3 atm_camera_local() { return atmosphere_camera_hi + atmosphere_camera_lo; }

// Distance from the model ground to the top boundary along the horizontal ray (Bruneton's H).
float atm_horizon_h() {
	float R = atmosphere_bottom_radius_m, T = atmosphere_top_radius_m;
	return sqrt((T - R) * (T + R));
}
// rho = sqrt(r^2 - R^2) from the height h = r - R above the model ground.
float atm_rho_from_height(float h) {
	return sqrt(max(h, 0.0) * (2.0 * atmosphere_bottom_radius_m + max(h, 0.0)));
}

// ---- Ray geometry (Bruneton functions.glsl), r and its exact rho / height passed in.
float atm_distance_to_top(float r, float mu) {
	float T = atmosphere_top_radius_m;
	float rmu = r * mu;
	return max(-rmu + atm_safe_sqrt(rmu * rmu + (T - r) * (T + r)), 0.0);
}
bool atm_ray_intersects_ground(float r, float rho, float mu) {
	float rmu = r * mu;
	return mu < 0.0 && rmu * rmu - rho * rho >= 0.0;
}
float atm_distance_to_bottom(float r, float rho, float mu) {
	float rmu = r * mu;
	return max(-rmu - atm_safe_sqrt(rmu * rmu - rho * rho), 0.0);
}

float atm_texture_coord_from_unit_range(float x, float size) { return 0.5 / size + x * (1.0 - 1.0 / size); }

// ---- Transmittance to the top boundary (GetTransmittanceToTopAtmosphereBoundary).
vec3 atm_transmittance_to_top(float r, float rho, float mu) {
	float H = atm_horizon_h();
	float d = atm_distance_to_top(r, mu);
	float d_min = atmosphere_top_radius_m - r;
	float d_max = rho + H;
	float x_mu = (d - d_min) / (d_max - d_min);
	float x_r = rho / H;
	vec2 uv = vec2(atm_texture_coord_from_unit_range(x_mu, atmosphere_table_sizes.x),
			atm_texture_coord_from_unit_range(x_r, atmosphere_table_sizes.y));
	return textureLod(atmosphere_transmittance, uv, 0.0).rgb;
}

float atm_rho_of(float r) {
	float R = atmosphere_bottom_radius_m;
	return atm_safe_sqrt((r - R) * (r + R));
}

// Height above the model ground of the point at distance d along the ray (r, mu) from a point at height
// rho^2 / (r + R): r_d^2 - r^2 = d (d + 2 r mu) is taken over (r_d + r), never r_d - R of two large float32 radii, so
// points near the ground keep their height to float32 precision of the height itself (aerial perspective near the pad).
float atm_height_along(float r, float rho, float mu, float d) {
	float q = d * (d + 2.0 * r * mu);
	float r_d = sqrt(max(r * r + q, 0.0));
	return rho * rho / (r + atmosphere_bottom_radius_m) + q / (r_d + r);
}

// GetTransmittance: between the point at (r, mu) and the point at distance d along the ray.
vec3 atm_transmittance(float r, float rho, float mu, float d, bool ground) {
	float h_d = clamp(atm_height_along(r, rho, mu, d), 0.0, atmosphere_top_radius_m - atmosphere_bottom_radius_m);
	float r_d = atmosphere_bottom_radius_m + h_d;
	float rho_d = atm_rho_from_height(h_d);
	float mu_d = atm_clamp_cosine((r * mu + d) / r_d);
	vec3 num;
	vec3 den;
	if (ground) {
		num = atm_transmittance_to_top(r_d, rho_d, -mu_d);
		den = atm_transmittance_to_top(r, rho, -mu);
	} else {
		num = atm_transmittance_to_top(r, rho, mu);
		den = atm_transmittance_to_top(r_d, rho_d, mu_d);
	}
	vec3 out_t;
	for (int k = 0; k < 3; k++) {
		out_t[k] = den[k] > 0.0 ? min(num[k] / den[k], 1.0) : 0.0;
	}
	return out_t;
}

// GetTransmittanceToSun: the transmittance times the fraction of the Sun disc above the model horizon.
vec3 atm_transmittance_to_sun(float r, float rho, float mu_s) {
	float sin_theta_h = atmosphere_bottom_radius_m / r;
	float cos_theta_h = -rho / r;
	float alpha = atmosphere_sun_angular_radius_rad;
	float visible = smoothstep(-sin_theta_h * alpha, sin_theta_h * alpha, mu_s - cos_theta_h);
	return atm_transmittance_to_top(r, rho, mu_s) * visible;
}

// ---- Scattering lookups (GetScatteringTextureUvwzFromRMuMuSNu + GetCombinedScattering, separate Mie table).
float atm_u_mu_s(float mu_s) {
	float R = atmosphere_bottom_radius_m;
	float H = atm_horizon_h();
	float d_min = atmosphere_top_radius_m - R;
	float d_max = H;
	float d = atm_distance_to_top(R, mu_s);
	float a = (d - d_min) / (d_max - d_min);
	float D = atm_distance_to_top(R, atmosphere_mu_s_min);
	float A = (D - d_min) / (d_max - d_min);
	return atm_texture_coord_from_unit_range(max(1.0 - a / A, 0.0) / (1.0 + a), atmosphere_scattering_size.y);
}

vec4 atm_scattering_uvwz(float r, float rho, float mu, float mu_s, float nu, bool ground) {
	float R = atmosphere_bottom_radius_m;
	float H = atm_horizon_h();
	float half_mu = atmosphere_scattering_size.z * 0.5;
	float u_r = atm_texture_coord_from_unit_range(rho / H, atmosphere_scattering_size.w);
	float rmu = r * mu;
	float discriminant = rmu * rmu - rho * rho;  // r^2 mu^2 - r^2 + R^2
	float u_mu;
	if (ground) {
		float d = -rmu - atm_safe_sqrt(discriminant);
		float d_min = rho * rho / (r + R);  // r - R
		float d_max = rho;
		u_mu = 0.5 - 0.5 * atm_texture_coord_from_unit_range(d_max == d_min ? 0.0 : (d - d_min) / (d_max - d_min),
				half_mu);
	} else {
		float d = -rmu + atm_safe_sqrt(discriminant + H * H);
		float d_min = atmosphere_top_radius_m - r;
		float d_max = rho + H;
		u_mu = 0.5 + 0.5 * atm_texture_coord_from_unit_range((d - d_min) / (d_max - d_min), half_mu);
	}
	return vec4((nu + 1.0) * 0.5, atm_u_mu_s(mu_s), u_mu, u_r);
}

// Scattering (RGB: Rayleigh + multiple, without the phase) and single Mie (RGB) at (r, mu, mu_s, nu): two trilinear
// samples in the nu slices around u_nu, lerped (Bruneton's GetScattering).
void atm_combined_scattering(float r, float rho, float mu, float mu_s, float nu, bool ground,
		out vec3 scattering, out vec3 single_mie) {
	vec4 uvwz = atm_scattering_uvwz(r, rho, mu, mu_s, nu, ground);
	float nu_size = atmosphere_scattering_size.x;
	float tex_coord_x = uvwz.x * (nu_size - 1.0);
	float tex_x = floor(tex_coord_x);
	float lerp_nu = tex_coord_x - tex_x;
	vec3 uvw0 = vec3((tex_x + uvwz.y) / nu_size, uvwz.z, uvwz.w);
	vec3 uvw1 = vec3((tex_x + 1.0 + uvwz.y) / nu_size, uvwz.z, uvwz.w);
	scattering = mix(textureLod(atmosphere_scattering, uvw0, 0.0).rgb, textureLod(atmosphere_scattering, uvw1, 0.0).rgb,
			lerp_nu);
	single_mie = mix(textureLod(atmosphere_single_mie, uvw0, 0.0).rgb, textureLod(atmosphere_single_mie, uvw1, 0.0).rgb,
			lerp_nu);
}

// GetIrradiance (the sky irradiance table), from the height above the model ground.
vec3 atm_irradiance(float h, float mu_s) {
	float x_r = h / (atmosphere_top_radius_m - atmosphere_bottom_radius_m);
	float x_mu_s = mu_s * 0.5 + 0.5;
	vec2 uv = vec2(atm_texture_coord_from_unit_range(x_mu_s, atmosphere_table_sizes.z),
			atm_texture_coord_from_unit_range(x_r, atmosphere_table_sizes.w));
	return textureLod(atmosphere_irradiance, uv, 0.0).rgb;
}

float atm_rayleigh_phase(float nu) { return 3.0 / (16.0 * ATM_PI) * (1.0 + nu * nu); }
float atm_mie_phase(float g, float nu) {
	float k = 3.0 / (8.0 * ATM_PI) * (1.0 - g * g) / (2.0 + g * g);
	float x = 1.0 + g * g - 2.0 * g * nu;
	return k * (1.0 + nu * nu) / (x * sqrt(x));
}

vec3 atm_to_scene(vec3 radiance) {
	return radiance * atmosphere_sky_radiance_to_luminance * atmosphere_luminance_scale;
}

// The camera in model space: its direction (unit) and its distance r from the centre with rho, from the altitude
// uniform (clamped onto the model ground from below, as the core's clamp_to_ground).
void atm_camera_model(vec3 camera_local, out vec3 direction, out float r, out float rho) {
	direction = normalize(camera_local * ATM_STRETCH);
	float h = max(atmosphere_camera_altitude_m, 0.0);
	r = atmosphere_bottom_radius_m + h;
	rho = atm_rho_from_height(h);
}

// A camera above the top boundary looking along v: enters at the top boundary if the ray meets the atmosphere.
// Returns false for a ray that misses it (or leaves it behind); else the entry point's direction, mu and its
// distance from the camera along v.
bool atm_enter_from_space(vec3 camera_model, vec3 v, out vec3 entry_direction, out float mu, out float entry_distance) {
	float T = atmosphere_top_radius_m;
	float along = dot(camera_model, v);
	float b = length(cross(camera_model, v));  // closest approach of the ray to the centre
	if (along >= 0.0 || b >= T) {
		return false;
	}
	float s = sqrt((T - b) * (T + b));
	vec3 entry = (camera_model - v * along) - v * s;
	entry_direction = entry / T;
	mu = -s / T;
	entry_distance = -along - s;
	return true;
}

// ---- Sky radiance along the model-space unit ray v from a point with up direction `up`, radius r and rho (inside or
// above the atmosphere), scene units; the transmittance of the ray to the atmosphere's edge (0 into the ground).
vec3 atm_sky_from(vec3 up, float r, float rho, vec3 v, out vec3 transmittance) {
	vec3 sun = normalize(atmosphere_sun_direction * ATM_STRETCH);
	float mu = atm_clamp_cosine(dot(up, v));
	if (r > atmosphere_top_radius_m) {
		vec3 entry_up;
		float entry_distance;
		if (!atm_enter_from_space(up * r, v, entry_up, mu, entry_distance)) {
			transmittance = vec3(1.0);
			return vec3(0.0);
		}
		up = entry_up;
		r = atmosphere_top_radius_m;
		rho = atm_horizon_h();
	}
	float mu_s = atm_clamp_cosine(dot(up, sun));
	float nu = atm_clamp_cosine(dot(v, sun));
	bool ground = atm_ray_intersects_ground(r, rho, mu);
	transmittance = ground ? vec3(0.0) : atm_transmittance_to_top(r, rho, mu);
	vec3 scattering;
	vec3 single_mie;
	atm_combined_scattering(r, rho, mu, mu_s, nu, ground, scattering, single_mie);
	vec3 radiance = scattering * atm_rayleigh_phase(nu) + single_mie * atm_mie_phase(atmosphere_mie_phase_g, nu);
	return atm_to_scene(radiance);
}

// ---- Sky radiance seen from the camera along a view direction (GetSkyRadiance, shadow_length 0), scene units; the
// transmittance of the view ray to the atmosphere's edge (0 for a ray into the ground). camera_local is the camera
// (atm_camera_local()); its height is atmosphere_camera_altitude_m.
vec3 atm_sky(vec3 camera_local, vec3 view_dir_local, out vec3 transmittance) {
	vec3 up;
	float r;
	float rho;
	atm_camera_model(camera_local, up, r, rho);
	return atm_sky_from(up, r, rho, normalize(view_dir_local * ATM_STRETCH), transmittance);
}

// The point at camera + offset (offset_local: Earth-local render axes, metres, camera-relative) in model space: its up
// direction, radius and rho, the height from the camera's altitude plus the offset (atm_height_along), so a point
// near the camera keeps its height to float32 precision of the offset.
void atm_point_model(vec3 offset_local, out vec3 up_p, out float r_p, out float rho_p) {
	vec3 up;
	float r;
	float rho;
	atm_camera_model(atm_camera_local(), up, r, rho);
	vec3 o = offset_local * ATM_STRETCH;
	float d = length(o);
	float h = d > 0.0 ? atm_height_along(r, rho, atm_clamp_cosine(dot(up, o / d)), d)
			: rho * rho / (r + atmosphere_bottom_radius_m);
	h = max(h, 0.0);
	up_p = normalize(up * r + o);
	r_p = atmosphere_bottom_radius_m + h;
	rho_p = atm_rho_from_height(h);
}

// Sky radiance along view_dir_local seen from the point at camera + offset_local (camera-relative, Earth-local render
// metres), scene units; the transmittance of that ray (the sea's reflected sky).
vec3 atm_sky_at_offset(vec3 offset_local, vec3 view_dir_local, out vec3 transmittance) {
	vec3 up_p;
	float r_p;
	float rho_p;
	atm_point_model(offset_local, up_p, r_p, rho_p);
	return atm_sky_from(up_p, r_p, rho_p, normalize(view_dir_local * ATM_STRETCH), transmittance);
}

// An object far along view_dir_local (the Moon, its terrain, the Sun, the stars) seen from the camera: the sky in front
// of it and the transmittance (atm_sky), unless the camera is outside the atmosphere and the object lies before the
// ray's entry (a camera at the Moon looking at the lunar ground with the Earth behind it): then nothing (T = 1).
// distance_m: the object's true distance from the camera.
vec3 atm_sky_to_far_object(vec3 camera_local, vec3 view_dir_local, float distance_m, out vec3 transmittance) {
	vec3 up;
	float r;
	float rho;
	atm_camera_model(camera_local, up, r, rho);
	vec3 v = normalize(view_dir_local * ATM_STRETCH);
	if (r > atmosphere_top_radius_m) {
		vec3 entry_up;
		float mu;
		float entry_distance;
		if (!atm_enter_from_space(up * r, v, entry_up, mu, entry_distance) || entry_distance >= distance_m) {
			transmittance = vec3(1.0);
			return vec3(0.0);
		}
	}
	return atm_sky_from(up, r, rho, v, transmittance);
}

// ---- In-scattered light between the camera and a point (aerial perspective, GetSkyRadianceToPoint, shadow_length
// 0), scene units; the transmittance between them. offset_model: point - camera in the stretched model space,
// camera-relative (callers difference near points before they reach the shader: terrain_earth's camera_offset; the
// stretch is linear, so stretch(p) - stretch(c) = stretch(p - c)). A point below the model ground is moved onto it.
vec3 atm_to_point_model(vec3 offset_model, out vec3 transmittance) {
	float R = atmosphere_bottom_radius_m;
	vec3 sun = normalize(atmosphere_sun_direction * ATM_STRETCH);
	vec3 up;
	float r;
	float rho;
	atm_camera_model(atm_camera_local(), up, r, rho);
	vec3 camera_model = up * r;
	float d = length(offset_model);
	if (d == 0.0) {
		transmittance = vec3(1.0);
		return vec3(0.0);
	}
	vec3 v = offset_model / d;
	if (atm_height_along(r, rho, atm_clamp_cosine(dot(up, v)), d) < 0.0) {  // below the model ground (rare): onto it
		vec3 point_model = normalize(camera_model + offset_model) * R;
		offset_model = point_model - camera_model;
		d = length(offset_model);
		v = offset_model / d;
	}
	float mu = atm_clamp_cosine(dot(up, v));
	if (r > atmosphere_top_radius_m) {
		vec3 entry_up;
		float entry_distance;
		// No entry, or the point lies before it: the segment is outside the atmosphere.
		if (!atm_enter_from_space(camera_model, v, entry_up, mu, entry_distance) || d <= entry_distance) {
			transmittance = vec3(1.0);
			return vec3(0.0);
		}
		up = entry_up;
		r = atmosphere_top_radius_m;
		rho = atm_horizon_h();
		d -= entry_distance;
	}
	float mu_s = atm_clamp_cosine(dot(up, sun));
	float nu = atm_clamp_cosine(dot(v, sun));
	bool ground = atm_ray_intersects_ground(r, rho, mu);
	vec3 tr = atm_transmittance(r, rho, mu, d, ground);
	transmittance = tr;
	vec3 s;
	vec3 m;
	atm_combined_scattering(r, rho, mu, mu_s, nu, ground, s, m);

	float h_p = clamp(atm_height_along(r, rho, mu, d), 0.0, atmosphere_top_radius_m - R);
	float r_p = R + h_p;
	float rho_p = atm_rho_from_height(h_p);
	float mu_p = atm_clamp_cosine((r * mu + d) / r_p);
	float mu_s_p = atm_clamp_cosine((r * mu_s + d * nu) / r_p);
	vec3 s_p;
	vec3 m_p;
	atm_combined_scattering(r_p, rho_p, mu_p, mu_s_p, nu, ground, s_p, m_p);

	// Bruneton's hack against artefacts with the Sun below the horizon: fade the single Mie term out.
	float mie_fade = smoothstep(0.0, 0.01, mu_s);
	vec3 scattering = s - tr * s_p;
	vec3 single_mie = (m - tr * m_p) * mie_fade;
	vec3 radiance = scattering * atm_rayleigh_phase(nu) + single_mie * atm_mie_phase(atmosphere_mie_phase_g, nu);
	return atm_to_scene(radiance);
}

// Aerial perspective to a point given camera-relative (offset_local = point - camera, Earth-local render metres): the
// form for surfaces near the camera (terrain, sea, vehicles), whose offset is exact in float32.
vec3 atm_to_point_offset(vec3 offset_local, out vec3 transmittance) {
	return atm_to_point_model(offset_local * ATM_STRETCH, transmittance);
}

// Aerial perspective to an absolute Earth-local point: only for points far from the camera (the difference of two
// absolute float32 positions carries metre-level error); near points use atm_to_point_offset. camera_local is
// atm_camera_local().
vec3 atm_to_point(vec3 camera_local, vec3 point_local, out vec3 transmittance) {
	return atm_to_point_model((point_local - atmosphere_camera_hi - atmosphere_camera_lo) * ATM_STRETCH, transmittance);
}

// ---- Light on a surface element (GetSunAndSkyIrradiance), as the scene value of a white Lambertian surface
// (illuminance x atmosphere_luminance_scale / pi): multiply by the albedo. Points above the top boundary take the
// top's values.
// The direct Sun transmitted to the point, at normal incidence (no cosine): the sea's glint needs it uncosined.
vec3 atm_sun_at(vec3 point_local) {
	vec3 p = point_local * ATM_STRETCH;
	float r = clamp(length(p), atmosphere_bottom_radius_m, atmosphere_top_radius_m);
	float mu_s = atm_clamp_cosine(dot(normalize(p), normalize(atmosphere_sun_direction * ATM_STRETCH)));
	return atmosphere_sun_illuminance_lux * atm_transmittance_to_sun(r, atm_rho_of(r), mu_s)
			* (atmosphere_luminance_scale / ATM_PI);
}

// The sky light on a surface with the (geodetic-frame) normal_local: the horizontal sky irradiance tilted by
// (1 + n.up) / 2 in the model space.
vec3 atm_sky_light(vec3 point_local, vec3 normal_local) {
	vec3 p = point_local * ATM_STRETCH;
	float r = clamp(length(p), atmosphere_bottom_radius_m, atmosphere_top_radius_m);
	vec3 up = normalize(p);
	vec3 normal = normalize(normal_local / ATM_STRETCH);
	float mu_s = atm_clamp_cosine(dot(up, normalize(atmosphere_sun_direction * ATM_STRETCH)));
	return atm_irradiance(r - atmosphere_bottom_radius_m, mu_s) * (1.0 + dot(normal, up)) * 0.5
			* atmosphere_sky_radiance_to_luminance * (atmosphere_luminance_scale / ATM_PI);
}

// Both: returns the transmitted direct Sun with the cosine of the true, unstretched geometry; the sky term in `sky`.
vec3 atm_sun_and_sky(vec3 point_local, vec3 normal_local, out vec3 sky) {
	sky = atm_sky_light(point_local, normal_local);
	float cosine = max(dot(normalize(normal_local), normalize(atmosphere_sun_direction)), 0.0);
	return atm_sun_at(point_local) * cosine;
}

// ---- Aerial perspective for Godot's lit materials (vehicles, tower, pads, drone ship: lit_aerial.gdshader,
// lattice.gdshader). Godot blends the FOG output after the lighting with a scalar weight: final = lit x (1 - FOG.a) +
// FOG.rgb x FOG.a. The per-channel transmittance T is split into the scalar t = max(T), carried by FOG.a = 1 - t, and
// the colour factor T / t, which the caller applies to ALBEDO and EMISSION before the lighting: diffuse light, metallic
// reflections and emission then end at lit x T + in-scatter exactly; only the colourless dielectric specular (F0 ~ 0.04)
// is attenuated by t instead of T (a little whiter in the haze than it should be).
// offset_world: fragment - the atmosphere's camera, render world axes, metres. The camera is the render origin (the
// floating origin: world.gd places everything relative to it), so a fragment's world position is its offset.
void atm_fog(vec3 offset_world, out vec4 fog, out vec3 colour_factor) {
	fog = vec4(0.0);
	colour_factor = vec3(1.0);
	if (!atmosphere_enabled) {
		return;
	}
	vec3 transmittance;
	vec3 in_scatter = atm_to_point_offset(atmosphere_world_to_earth * offset_world, transmittance);
	float t = max(transmittance.r, max(transmittance.g, transmittance.b));
	colour_factor = t > 0.0 ? transmittance / t : vec3(0.0);
	float a = 1.0 - t;
	// a = 0 (no air between): the in-scatter is 0 as well; FOG.rgb is weighted by a, so it is S / a.
	fog = a > 1.0e-6 ? vec4(in_scatter / a, a) : vec4(0.0);
}
