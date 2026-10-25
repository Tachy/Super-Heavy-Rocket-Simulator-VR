// Clouds from the weather field (M8 weather and clouds, Task 8): the field lookup with the time blend, the height
// layers, the coverage-preserving sub-grid shapes, the extinction and the scattering source - the GPU port of
// core/include/starsim/environment/cloud_optics.hpp, cloud_noise.hpp, cloud_render_texels.hpp and weather_snapshot.hpp
// (change them together). Include atmosphere.gdshaderinc first (the floating-origin globals atmosphere_camera_hi/lo).
//
// Frames and numerics: positions are Earth-local render axes (ITRF (x, y, z) -> (y, z, x), polar axis +Y), metres.
// Points near the camera are passed camera-relative (offset_local = point - camera), exact in float32; heights come from
// the camera's height cloud_camera_height_m (float64 on the CPU) plus the offset in the stretched model space (the
// WGS84 ellipsoid -> the sphere of radius a, as atmosphere.gdshaderinc): height = |M p| - a, within 0.34 % of the
// geodetic height (68 m at 20 km against 250 m / 1 km layers). Latitude: geodetic from tan(phi) = (a/b)^2 z / s (exact
// on the ellipsoid, < 1e-5 rad off at 20 km); longitude atan2.
//
// Field (CloudLayer uploads the GPU form of two snapshots, cloud_render_texels.hpp): x = longitude column (wraps),
// y = latitude row (+90 deg at row 0, -90 deg at the last row, both inclusive), z = height layer; RGBA8: R cloud
// fraction, G/B the 16-bit linear grid-mean extinction, A the log-coded in-cloud optical depth from 20 km down to the
// layer centre. Values belong to texel centres (layer centres in height), hardware trilinear between them = the core's
// WeatherSnapshot::at(): texture w = layer coordinate / 32 clamped to the centres of layers 0 and 31 (the core holds the
// end values), zero above 20 km. The two snapshots may differ in size (the 1-deg analog beside a 0.25-deg ERA5 hour).
//
// Sub-grid shapes (cloud_noise.hpp): cloud where the shape noise's R channel, sampled TRILINEARLY WITHOUT MIPMAPS (what
// the coverage table assumes), exceeds coverage_threshold(cloud fraction), so the covered area fraction is the field's;
// in-cloud extinction = grid mean / cloud fraction; the detail noise modulates it by 1 + a (d - mean(d)) (mean 1, so
// the coverage-weighted mean stays the field's) with the amplitude a fading linearly to 0 at cloud_detail_distance_m:
// beyond it the density is exactly field mean / fraction inside the shapes - the faded detail integrates to the same
// optical depth (no pop). Shape coordinates are anisotropic (CLOUD_SHAPE_TILE_H_M across, CLOUD_SHAPE_TILE_V_M up:
// q = p / S_H + up(p) h(p) (1 / S_V - 1 / S_H), up = p / |p|), detail isotropic; both Earth-fixed, so every eye and
// every frame sees the same shapes. The camera's coordinates arrive wrapped to [0, 1) from the CPU (float64).

// ---- Globals (project.godot [shader_globals], CloudLayer).
global uniform sampler3D cloud_field_before : filter_linear, repeat_enable;
global uniform sampler3D cloud_field_after : filter_linear, repeat_enable;
global uniform vec3 cloud_field_before_size;  // (columns, rows, layers)
global uniform vec3 cloud_field_after_size;
global uniform float cloud_blend;  // weight of cloud_field_after
global uniform sampler3D cloud_noise_shape : filter_linear, repeat_enable;
global uniform sampler3D cloud_noise_detail : filter_linear, repeat_enable;
global uniform sampler2D cloud_coverage_threshold : filter_linear, repeat_disable;  // 256 x 1, R = t(code / 255)
global uniform bool cloud_enabled;
global uniform float cloud_camera_height_m;  // |M camera| - a, float64 on the CPU
global uniform vec3 cloud_noise_origin;      // shape coordinates of the camera, wrapped to [0, 1)
global uniform vec3 cloud_detail_origin;     // detail coordinates of the camera, wrapped
global uniform float cloud_detail_mean;      // mean of the detail value over the texture
global uniform float cloud_detail_amplitude; // a at the camera (quality preset; 0 = no detail)
global uniform float cloud_detail_distance_m;
global uniform vec4 cloud_phase;             // Jendersie & d'Eon: (hg_g, draine_g, draine_alpha, draine_weight)
global uniform float cloud_asymmetry_g;      // g of that phase function (closed form)
global uniform float cloud_extinction_step_per_m;  // per unit of the 16-bit G/B number
global uniform vec2 cloud_depth_code;        // (min, max) of the A channel's log code
// Checks only (cloud_check.gd; 0 in the game): 1 = the background transmittance T_bg, 2 = the Sun's scattered radiance
// per unit Sun irradiance without the air and the sky (both opaque, both of the GRID-MEAN field without the sub-grid
// shapes: the core's CPU references have no shapes); 3 = the coverage (1 cloud, 0 clear) where the ray crosses the height
// cloud_debug_height_m (the GPU's filtered coverage against the core's table).
global uniform int cloud_debug_output;
global uniform float cloud_debug_height_m;

// [S] WGS84 equatorial radius (core constants.hpp wgs84_a_m); the stretch is ATM_STRETCH (atmosphere.gdshaderinc).
const float CLOUD_A_M = 6378137.0;
// (a / b)^2 of WGS84: geodetic latitude from the position.
const float CLOUD_AB2 = 1.0067394967422765;
const float CLOUD_PI = 3.14159265358979;
// Height layers (core weather_snapshot.hpp): 16 x 250 m from 0 to 4 km, 16 x 1 km from 4 to 20 km.
const float CLOUD_FINE_LAYER_M = 250.0;
const float CLOUD_COARSE_LAYER_M = 1000.0;
const float CLOUD_FINE_TOP_M = 4000.0;
const float CLOUD_TOP_M = 20000.0;
const float CLOUD_LAYERS = 32.0;
// [E] Sub-grid shape scales: the shape tile 32 km across (Perlin octaves of 4 km to 500 m, Worley cells of 8 km) and
// 16 km high (towers stay coherent over a deck's depth: the cloudy parts of the layers overlap, the maximum-overlap
// column of cloud_render_texels.hpp); the detail tile 2 km (Worley cells 1 km to 250 m).
const float CLOUD_SHAPE_TILE_H_M = 32000.0;
const float CLOUD_SHAPE_TILE_V_M = 16000.0;
const float CLOUD_DETAIL_TILE_M = 2000.0;
// A cloud fraction below half a code step counts as none (no division by ~0).
const float CLOUD_MIN_FRACTION = 0.5 / 255.0;

// ---- Height layers.
float cloud_layer_coordinate(float height_m) {
	float s = height_m < CLOUD_FINE_TOP_M ? height_m / CLOUD_FINE_LAYER_M
			: 16.0 + (height_m - CLOUD_FINE_TOP_M) / CLOUD_COARSE_LAYER_M;
	return clamp(s, 0.0, CLOUD_LAYERS);
}

float cloud_layer_centre_m(float layer) {
	return layer < 16.0 ? (layer + 0.5) * CLOUD_FINE_LAYER_M
			: CLOUD_FINE_TOP_M + (layer - 16.0 + 0.5) * CLOUD_COARSE_LAYER_M;
}

// ---- Geometry: (latitude, longitude) of an Earth-local point; its height from the camera-relative offset.
vec2 cloud_lat_lon(vec3 p_local) {
	float s = length(p_local.xz);
	return vec2(atan(CLOUD_AB2 * p_local.y, s), atan(p_local.x, p_local.z));
}

// The camera in model space: radius r_c = a + h_c, its up and the model-space ray (unit) with the length factor of
// the stretch (model distance = distance x scale).
struct CloudRay {
	vec3 dir_local;   // unit, Earth-local render axes
	float scale;      // |M dir|
	float mu;         // cosine of the model ray to the camera's model up
	float r_c;
};

CloudRay cloud_ray(vec3 dir_local) {
	CloudRay ray;
	ray.dir_local = dir_local;
	vec3 vm = dir_local * ATM_STRETCH;
	ray.scale = length(vm);
	vec3 up_c = normalize((atmosphere_camera_hi + atmosphere_camera_lo) * ATM_STRETCH);
	ray.mu = dot(up_c, vm / ray.scale);
	ray.r_c = CLOUD_A_M + cloud_camera_height_m;
	return ray;
}

// Height above the ellipsoid (model space) at distance d along the ray: h + d'(d' + 2 r mu) / (r_d + r), never the
// difference of two large radii.
float cloud_height_along(CloudRay ray, float d) {
	float dm = d * ray.scale;
	float r_d = sqrt(max(ray.r_c * ray.r_c + dm * dm + 2.0 * ray.r_c * ray.mu * dm, 0.0));
	return cloud_camera_height_m + dm * (dm + 2.0 * ray.r_c * ray.mu) / (r_d + ray.r_c);
}

// d(height) / d(distance) along the ray at distance d (the sine of the local elevation).
float cloud_height_slope(CloudRay ray, float d) {
	float dm = d * ray.scale;
	float r_d = sqrt(max(ray.r_c * ray.r_c + dm * dm + 2.0 * ray.r_c * ray.mu * dm, 1.0));
	return ray.scale * (dm + ray.r_c * ray.mu) / r_d;
}

// Distances (true metres) where the ray crosses the sphere of height h_s: x = entry, y = exit; x > y: no crossing.
vec2 cloud_sphere_crossing(CloudRay ray, float h_s) {
	float r_s = CLOUD_A_M + h_s;
	float b = ray.r_c * ray.mu;
	float disc = b * b - (cloud_camera_height_m - h_s) * (ray.r_c + r_s);
	if (disc < 0.0) {
		return vec2(1.0, -1.0);
	}
	float q = sqrt(disc);
	return vec2(-b - q, -b + q) / ray.scale;
}

// ---- Field lookup.
struct CloudCell {
	float fraction;     // cloud fraction
	float extinction;   // grid-mean extinction [1/m]
};

vec3 cloud_field_uvw(vec2 lat_lon, float w, vec3 size) {
	float u = lat_lon.y / (2.0 * CLOUD_PI) + 0.5 / size.x;  // column c's centre at longitude c x 360 / columns
	float y = clamp((0.5 * CLOUD_PI - lat_lon.x) / (CLOUD_PI / (size.y - 1.0)), 0.0, size.y - 1.0);
	return vec3(u, (y + 0.5) / size.y, w);
}

CloudCell cloud_decode(vec4 t) {
	CloudCell c;
	c.fraction = t.r;
	c.extinction = (t.g * 65280.0 + t.b * 255.0) * cloud_extinction_step_per_m;  // (256 G + B) codes, UNORM8 = code/255
	return c;
}

// The blended field at (lat, lon, height): cloud fraction and grid-mean extinction, 0 above 20 km.
CloudCell cloud_field(vec2 lat_lon, float height_m) {
	CloudCell c;
	c.fraction = 0.0;
	c.extinction = 0.0;
	if (height_m > CLOUD_TOP_M) {
		return c;
	}
	float w = clamp(cloud_layer_coordinate(height_m), 0.5, CLOUD_LAYERS - 0.5) / CLOUD_LAYERS;
	if (cloud_blend < 1.0) {
		CloudCell b = cloud_decode(texture(cloud_field_before, cloud_field_uvw(lat_lon, w, cloud_field_before_size)));
		c.fraction += (1.0 - cloud_blend) * b.fraction;
		c.extinction += (1.0 - cloud_blend) * b.extinction;
	}
	if (cloud_blend > 0.0) {
		CloudCell a = cloud_decode(texture(cloud_field_after, cloud_field_uvw(lat_lon, w, cloud_field_after_size)));
		c.fraction += cloud_blend * a.fraction;
		c.extinction += cloud_blend * a.extinction;
	}
	return c;
}

float cloud_decode_depth(float a) {
	float code = floor(a * 255.0 + 0.5);
	return code < 0.5 ? 0.0 : cloud_depth_code.x * pow(cloud_depth_code.y / cloud_depth_code.x, (code - 1.0) / 254.0);
}

// In-cloud extinction and in-cloud depth from the top at one layer centre (w exactly on it: only the horizontal
// bilinear filter acts), blended. The depth code is decoded after the horizontal filter (between columns of different
// codes geometric; the vertical part, the important one, is exact).
vec2 cloud_centre_column(vec2 lat_lon, float layer) {
	float w = (layer + 0.5) / CLOUD_LAYERS;
	vec2 result = vec2(0.0);
	if (cloud_blend < 1.0) {
		vec4 t = texture(cloud_field_before, cloud_field_uvw(lat_lon, w, cloud_field_before_size));
		CloudCell c = cloud_decode(t);
		float in_cloud = c.fraction >= CLOUD_MIN_FRACTION ? c.extinction / c.fraction : 0.0;
		result += (1.0 - cloud_blend) * vec2(in_cloud, cloud_decode_depth(t.a));
	}
	if (cloud_blend > 0.0) {
		vec4 t = texture(cloud_field_after, cloud_field_uvw(lat_lon, w, cloud_field_after_size));
		CloudCell c = cloud_decode(t);
		float in_cloud = c.fraction >= CLOUD_MIN_FRACTION ? c.extinction / c.fraction : 0.0;
		result += cloud_blend * vec2(in_cloud, cloud_decode_depth(t.a));
	}
	return result;
}

// The column of the scattering source at a point with in-cloud extinction beta_point: x = in-cloud optical depth from
// the column top (20 km) down to the point, y = the whole column (to the bottom of layer 0) - cloud_render_texels.hpp.
vec2 cloud_column_depths(vec2 lat_lon, float height_m, float beta_point) {
	float s = cloud_layer_coordinate(height_m) - 0.5;  // centre coordinate
	float above = clamp(floor(s) + 1.0, 0.0, CLOUD_LAYERS - 1.0);
	vec2 depth;
	if (s >= CLOUD_LAYERS - 1.0) {
		depth.x = beta_point * max(CLOUD_TOP_M - height_m, 0.0);  // held from the top centre to 20 km
	} else {
		vec2 centre = cloud_centre_column(lat_lon, above);
		depth.x = centre.y + 0.5 * (beta_point + centre.x) * max(cloud_layer_centre_m(above) - height_m, 0.0);
	}
	vec2 bottom = cloud_centre_column(lat_lon, 0.0);
	depth.y = max(bottom.y + bottom.x * cloud_layer_centre_m(0.0), depth.x);
	return depth;
}

// ---- Sub-grid shapes.
// Shape coordinates of the point at camera + offset (height_m its height): see the header.
vec3 cloud_shape_coordinates(vec3 offset_local, float height_m) {
	vec3 camera = atmosphere_camera_hi + atmosphere_camera_lo;
	vec3 up_p = normalize(camera + offset_local);
	vec3 up_c = normalize(camera);
	float k = 1.0 / CLOUD_SHAPE_TILE_V_M - 1.0 / CLOUD_SHAPE_TILE_H_M;
	return cloud_noise_origin + offset_local / CLOUD_SHAPE_TILE_H_M
			+ k * (up_p * height_m - up_c * cloud_camera_height_m);
}

// Coverage: 1 where the shape is cloud for the cloud fraction, else 0 (cloud_noise.hpp coverage rule).
float cloud_coverage(vec3 shape_q, float fraction) {
	float threshold = texture(cloud_coverage_threshold, vec2((fraction * 255.0 + 0.5) / 256.0, 0.5)).r;
	float shape = texture(cloud_noise_shape, shape_q).r;
	return shape > threshold ? 1.0 : 0.0;
}

// The detail factor (mean 1) with the amplitude faded by the distance from the camera.
float cloud_detail_factor(vec3 offset_local, float distance_m) {
	float a = cloud_detail_amplitude * clamp(1.0 - distance_m / max(cloud_detail_distance_m, 1.0), 0.0, 1.0);
	if (a <= 0.0) {
		return 1.0;
	}
	vec3 d = texture(cloud_noise_detail, cloud_detail_origin + offset_local / CLOUD_DETAIL_TILE_M).rgb;
	return max(1.0 + a * (dot(d, vec3(0.625, 0.25, 0.125)) - cloud_detail_mean), 0.0);
}

// In-cloud extinction [1/m] (unscaled) at camera + offset_local with its height; with_detail adds the faded detail.
// Also returns the cell's in-cloud extinction (without detail) for the column terms.
// From the field cell already looked up at that point (cloud_extinction_at below looks it up).
float cloud_extinction_in_cell(CloudCell cell, vec3 offset_local, float height_m, float distance_m, bool with_detail,
		out float in_cloud) {
	in_cloud = 0.0;
	if (cloud_debug_output == 1 || cloud_debug_output == 2) {  // checks: the grid-mean field
		in_cloud = cell.extinction;
		return cell.extinction;
	}
	if (cell.fraction < CLOUD_MIN_FRACTION || cell.extinction <= 0.0) {
		return 0.0;
	}
	if (cloud_coverage(cloud_shape_coordinates(offset_local, height_m), cell.fraction) <= 0.0) {
		return 0.0;
	}
	in_cloud = cell.extinction / cell.fraction;
	return with_detail ? in_cloud * cloud_detail_factor(offset_local, distance_m) : in_cloud;
}

float cloud_extinction_at(vec3 offset_local, float height_m, float distance_m, bool with_detail,
		out vec2 lat_lon, out float in_cloud) {
	lat_lon = cloud_lat_lon(atmosphere_camera_hi + atmosphere_camera_lo + offset_local);
	return cloud_extinction_in_cell(cloud_field(lat_lon, height_m), offset_local, height_m, distance_m, with_detail,
			in_cloud);
}

// Height of the absolute Earth-local point p (Task 9 / 10 callers; far points only: float32 absolute positions).
float cloud_height_of(vec3 p_earth_m) {
	return length(p_earth_m * ATM_STRETCH) - CLOUD_A_M;
}

// Extinction [1/m] at an absolute Earth-local point (Tasks 9 / 10): the shapes, detail faded with the distance from
// the camera.
float cloud_density(vec3 p_earth_m) {
	vec3 offset = p_earth_m - atmosphere_camera_hi - atmosphere_camera_lo;
	vec2 lat_lon;
	float in_cloud;
	return cloud_extinction_at(offset, cloud_height_of(p_earth_m), length(offset), true, lat_lon, in_cloud);
}

// Unscaled optical depth from camera + offset_local toward direction dir_local over length_m in n samples (spacing
// growing quadratically, the first sample offset by jitter in [0, 1)); shapes, no detail. Stops above 20 km.
float cloud_optical_depth_toward(vec3 offset_local, float height_m, vec3 dir_local, float length_m, int n,
		float jitter) {
	vec3 p = atmosphere_camera_hi + atmosphere_camera_lo + offset_local;
	vec3 up = normalize(p * ATM_STRETCH);
	float climb = dot(up, normalize(dir_local * ATM_STRETCH));
	float tau = 0.0;
	float previous = 0.0;
	float count = float(n);
	for (int i = 0; i < n; i++) {
		float x1 = (float(i) + 1.0) / count;
		float end = length_m * x1 * x1;
		float xs = (float(i) + jitter) / count;
		float at = length_m * xs * xs;
		// Height along the straight line (curvature: + at^2 / 2r, enough over a few km).
		float h = height_m + at * climb + at * at / (2.0 * CLOUD_A_M);
		if (h > CLOUD_TOP_M && climb >= 0.0) {
			break;
		}
		vec2 lat_lon;
		float in_cloud;
		tau += cloud_extinction_at(offset_local + dir_local * at, h, 0.0, false, lat_lon, in_cloud) * (end - previous);
		previous = end;
	}
	return tau;
}

// Transmittance of the direct Sun (Beer-Lambert, unscaled extinction, as the core's cloud_sun_transmittance) from the
// absolute Earth-local point toward sun_dir: through the layers to 20 km (or 300 km of path), 12 samples.
const float CLOUD_SUN_PATH_MAX_M = 300000.0;
float cloud_sun_transmittance(vec3 p_earth_m, vec3 sun_dir) {
	float h = cloud_height_of(p_earth_m);
	if (h > CLOUD_TOP_M && dot(normalize(p_earth_m), sun_dir) >= 0.0) {
		return 1.0;
	}
	vec3 up = normalize(p_earth_m * ATM_STRETCH);
	float climb = dot(up, normalize(sun_dir * ATM_STRETCH));
	float to_top = climb > 1e-3 ? max(CLOUD_TOP_M - h, 0.0) / climb : CLOUD_SUN_PATH_MAX_M;
	vec3 offset = p_earth_m - atmosphere_camera_hi - atmosphere_camera_lo;
	return exp(-cloud_optical_depth_toward(offset, h, sun_dir, min(to_top, CLOUD_SUN_PATH_MAX_M), 12, 0.5));
}

// ---- Optics (cloud_optics.hpp).
float cloud_henyey_greenstein(float one_minus_mu, float g) {
	// 1 + g^2 - 2 g mu written as (1 - g)^2 + 2 g (1 - mu): no cancellation in the forward peak (g ~0.995).
	float denom = (1.0 - g) * (1.0 - g) + 2.0 * g * one_minus_mu;
	return (1.0 - g * g) / (4.0 * CLOUD_PI * denom * sqrt(denom));
}

// p(mu) of Jendersie & d'Eon 2023 (HG + Draine), per steradian; one_minus_mu = 1 - cos theta (passed separately for
// the forward peak's precision).
float cloud_phase_function(float mu, float one_minus_mu) {
	float g_hg = cloud_phase.x;
	float g_d = cloud_phase.y;
	float alpha = cloud_phase.z;
	float w = cloud_phase.w;
	float draine = cloud_henyey_greenstein(one_minus_mu, g_d) * (1.0 + alpha * mu * mu)
			/ (1.0 + alpha * (1.0 + 2.0 * g_d * g_d) / 3.0);
	return (1.0 - w) * cloud_henyey_greenstein(one_minus_mu, g_hg) + w * draine;
}

// Delta-Eddington constants of the phase function: x = f, y = g', z = gamma1.
vec3 cloud_delta_eddington() {
	float g = cloud_asymmetry_g;
	float g_scaled = g / (1.0 + g);
	return vec3(g * g, g_scaled, 0.75 * (1.0 - g_scaled));
}

float cloud_transport_scale() {
	return 1.0 - cloud_asymmetry_g * cloud_asymmetry_g;
}

// The Sun's scattering source J (cloud_scattering_source: TMS on the delta-Eddington source function) per unit Sun
// irradiance and unit unscaled optical depth. cos_theta: Sun propagation . view propagation (1 = looking into the Sun),
// one_minus_cos = 1 - cos_theta; view_up_cos: the view propagation direction (toward the camera) . up; depths unscaled.
float cloud_scattering_source(float cos_theta, float one_minus_cos, float view_up_cos, float sun_cos_zenith,
		float sun_optical_depth, float depth_from_top, float column_optical_depth) {
	if (sun_cos_zenith <= 0.0) {
		return 0.0;
	}
	vec3 de = cloud_delta_eddington();
	float k = 1.0 - de.x;
	float mu0 = min(1.0, sun_cos_zenith);
	float column = k * max(0.0, column_optical_depth);
	float t = clamp(k * depth_from_top, 0.0, column);
	float gamma1 = de.z;
	float gamma3 = (2.0 - 3.0 * de.y * mu0) / 4.0;
	float r = (gamma1 * column + (gamma3 - gamma1 * mu0) * (1.0 - exp(-column / mu0))) / (1.0 + gamma1 * column);
	float a0 = mu0 * (r + 2.0 * gamma1 * mu0 + 1.0 - 2.0 * gamma3);
	float a1 = 2.0 * gamma1 * mu0 * (r - 1.0);
	float a2 = -mu0 * (2.0 * gamma1 * mu0 + 1.0 - 2.0 * gamma3);
	float b0 = mu0 * (r - 1.0);
	float b1 = mu0;
	float e = exp(-t / mu0);
	float sigma = a0 + a1 * t + a2 * e;
	float net = b0 + b1 * e;
	float i0 = sigma / (2.0 * CLOUD_PI);
	float i1 = 3.0 * net / (4.0 * CLOUD_PI);
	return k * (i0 + de.y * view_up_cos * i1)
			+ cloud_phase_function(cos_theta, one_minus_cos) * exp(-k * max(0.0, sun_optical_depth));
}

// The sky's source (cloud_sky_scattering_source) per unit horizontal sky irradiance on the column top.
float cloud_sky_scattering_source(float view_up_cos, float depth_from_top, float column_optical_depth) {
	vec3 de = cloud_delta_eddington();
	float k = 1.0 - de.x;
	float column = k * max(0.0, column_optical_depth);
	float t = clamp(k * depth_from_top, 0.0, column);
	float transmittance = 1.0 / (1.0 + de.z * column);
	float sigma = 2.0 - transmittance - 2.0 * de.z * transmittance * t;
	float i0 = sigma / (2.0 * CLOUD_PI);
	float i1 = -3.0 * transmittance / (4.0 * CLOUD_PI);
	return k * (i0 + de.y * view_up_cos * i1);
}
