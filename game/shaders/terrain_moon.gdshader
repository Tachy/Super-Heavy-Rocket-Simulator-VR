shader_type spatial;
// Moon terrain node (TerrainQuadtree): the model of moon.gdshader (the regolith reflectance of moon_regolith.gdshaderinc:
// Hapke by default, the former Lommel-Seeliger law with moon_brdf 0; Earth shadow with the
// reddened umbra glow, physical earthshine) on the relief normal, with geomorphing. Albedo: the global colour map
// (moon_color, equirectangular) through CUSTOM1's global UV; the Moon has no imagery crops and no water.
// Terrain shadows: the core's baked horizon (16 azimuths per grid vertex, TerrainNodeBuilder._horizon_image) hides the
// Sun and the earthshine behind crater rims and mountains: the visible fraction of each light's disc above the horizon
// in its azimuth (linear between the two neighbouring sectors, bilinear between the vertices). Not modelled: the
// shadows of vehicles and of structures below the grid's resolution, the disc's azimuthal extent (the horizon is
// taken in the disc centre's azimuth), light scattered into a shadow.
// Positions are in the Moon's local render frame (metres, unscaled): vertex_offset restores them from the node centre.
// Seen through the Earth's atmosphere like moon.gdshader (a lunar ground view never crosses it).
render_mode unshaded, cull_back;

#include "res://shaders/eclipse.gdshaderinc"
#include "res://shaders/atmosphere.gdshaderinc"
#include "res://shaders/terrain_morph.gdshaderinc"
#include "res://shaders/moon_regolith.gdshaderinc"
#include "res://shaders/moon_detail.gdshaderinc"
#include "res://shaders/moon_horizon.gdshaderinc"

// CUSTOM1.u runs past 1 on nodes across the antimeridian (continuous inside the node), hence repeat.
uniform sampler2D albedo_texture : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
global uniform vec3 moon_sun_position_local;    // apparent, from the Moon's centre (World)
global uniform vec3 moon_earth_position_local;  // from the Moon's centre (World)
// The camera's Moon-local render position, float32 hi + lo parts (TerrainQuadtree.camera_global, once per frame).
global uniform vec3 moon_terrain_camera_hi;
global uniform vec3 moon_terrain_camera_lo;
// Set from GDScript (World: the Sun's and the Earth's radius).
uniform float sun_radius_m;
uniform float earth_radius_m;
// The constants of moon.gdshader (same look as the body mesh from scaled space).
uniform vec3 umbra_glow = vec3(0.020, 0.005, 0.0015);
// Diagnostic tint (render_check --moon_diag): 0 off; 1 red where the interpolated normal faces away from the eye
// (dot(n, to_eye) < REGOLITH_MU_MIN: regolith_reflectance's limb replacement, Lommel-Seeliger at its 2x maximum);
// 2 red where the shading normal faces away from the Sun (self-shadow by the normal), blue where the horizon map hides
// more than half the Sun (cast shadow), magenta both; 3 the drawn triangle's own (facet) normal for the shading.
uniform int diag_mode = 0;
// Hapke's mean slope override (rad; tests, diagnostics); negative: with detail shading the residual of the node's
// effective spacing (moon_detail.gdshaderinc), without it the globe's (moon_hapke_theta_bar_rad).
uniform float hapke_theta_bar_rad = -1.0;

varying vec3 local_position;
varying vec3 local_normal;
varying vec2 horizon_uv;  // the horizon map's texture coordinates, from CUSTOM2 (the position in the vertex grid, 0 .. 64)
varying vec2 global_uv;  // equirectangular u (continuous in the node), v
varying vec3 camera_offset;  // fragment - camera (node-local render axes, metres; exact near the camera)
varying float morph_factor;  // the vertex morph (0: this level, 1: the parent's surface)
varying vec2 grid_position;  // CUSTOM2: the vertex grid position (0 .. 64), the detail textures' coordinate

void vertex() {
	vec3 camera_local = terrain_camera_local(moon_terrain_camera_hi, moon_terrain_camera_lo);
	float morph = terrain_morph_factor(VERTEX, camera_local);
	morph_factor = morph;
	grid_position = CUSTOM2.xy;
	VERTEX += CUSTOM0.xyz * morph;
	NORMAL = normalize(mix(NORMAL, CUSTOM3.xyz, morph));
	local_position = VERTEX + vertex_offset;
	local_normal = NORMAL;
	global_uv = CUSTOM1.xy;
	// Texel centres are the (owner's) grid vertices.
	horizon_uv = ((CUSTOM2.xy * horizon_grid_scale + horizon_grid_offset) / horizon_stride + 0.5) / float(textureSize(horizon_map, 0).x);
	camera_offset = VERTEX - camera_local;
}


void fragment() {
	vec3 n = normalize(local_normal);
	vec3 to_sun = moon_sun_position_local - local_position;
	vec3 to_earth = moon_earth_position_local - local_position;
	vec3 to_eye = normalize(-camera_offset);
	// The drawn triangle's own (facet) normal, from camera_offset (exact near the camera; local_position's float32
	// steps are ~0.1 m), in uniform control flow for the derivatives.
	vec3 facet = cross(dFdx(camera_offset), dFdy(camera_offset));
	bool normal_away = dot(n, to_eye) < REGOLITH_MU_MIN;  // the diagnostic's set (before the replacement)
	if (diag_mode == 3 && dot(facet, facet) > 0.0) {
		vec3 f3 = normalize(facet);
		n = dot(f3, to_eye) < 0.0 ? -f3 : f3;
	}
	if (moon_brdf == 1 && normal_away && dot(facet, facet) > 0.0) {
		// Hapke only (the Lommel-Seeliger law stays bit-identical): the interpolated vertex normal faces away from the
		// eye on a triangle that faces it (crests seen at grazing view: the vertex normals are central differences over
		// two cells and lean beyond the crest). The limb replacement then lit thin bright lines along the crests (Task 7
		// review M1); the triangle's own orientation is what the eye sees there.
		vec3 f = normalize(facet);
		n = dot(f, to_eye) < 0.0 ? -f : f;
	}
	// Detail shading (moon_detail.gdshaderinc): the effective vertex spacing (the morph slides to the parent's 2 x),
	// the texture relief's height gradient (derivatives taken in uniform control flow).
	float spacing_eff = detail_cell_m * exp2(morph_factor);
	vec3 detail_gradient = detail_texture_gradient(grid_position, camera_offset, n, spacing_eff);
	float theta_bar = hapke_theta_bar_rad >= 0.0 ? hapke_theta_bar_rad
			: (detail_shading ? detail_theta_bar(spacing_eff) : moon_hapke_theta_bar_rad);
	float mu0_raw = dot(n, normalize(to_sun));
	float mu0 = max(mu0_raw, 0.0);
	float mu = max(dot(n, to_eye), 1e-3);
	float lommel_seeliger = 2.0 * mu0 / (mu0 + mu);
	float sunlit = 1.0 - occulted_fraction(to_sun, sun_radius_m, to_earth, earth_radius_m);
	float sun_visible = 1.0;  // above the terrain horizon
	// The Sun's elevation over the horizon in its azimuth (the detail statistics' macroscopic elevation; over the local
	// horizontal where no horizon map is read).
	float sun_margin = asin(clamp(dot(normalize(to_sun), normalize(local_position)), -1.0, 1.0));
	float earth_visible = 1.0;
	// The same for the Earth (the earthshine's statistics; terrain_visible_fraction refines it over the horizon).
	float earth_margin = asin(clamp(dot(normalize(to_earth), normalize(local_position)), -1.0, 1.0));
	if (horizon_enabled) {
		// A light that cannot reach the fragment anyway (below the relief's horizon, or the Sun with no umbra
		// glow) skips its horizon; the silhouette's normal replacement (regolith_reflectance) is kept clear.
		bool silhouette = dot(n, to_eye) < REGOLITH_MU_MIN;
		bool sun_matters = mu0_raw > 0.0 || sunlit < 1.0 || silhouette;
		bool earth_matters = dot(n, normalize(to_earth)) > 0.0 || silhouette;
		// Local axes at the fragment in the Moon-fixed render frame (the pole is +Y; at a pole any east will do).
		vec3 up = normalize(local_position);
		vec3 east = vec3(up.z, 0.0, -up.x);
		east = length(east) > POLE_EAST_MIN ? normalize(east) : vec3(0.0, 0.0, 1.0);
		vec3 north = cross(up, east);
		if (sun_matters) {
			sun_visible = terrain_visible_fraction(to_sun, sun_radius_m, east, north, up, horizon_uv, sun_margin);
		}
		if (earth_matters || detail_shading) {
			earth_visible = terrain_visible_fraction(to_earth, earth_radius_m, east, north, up, horizon_uv,
					earth_margin);
		}
	}
	vec2 detail_factors = vec2(1.0);
	float sun_modulation = 1.0;
	float earth_modulation = 1.0;
	if (detail_shading) {
		float abundance = texture(detail_rock_abundance, global_uv).r;
		// Rocks above the threshold diameter at this distance are geometry (Task 11, MoonRockLayer; the same rule as
		// moon_rock.gdshader), the rest statistical.
		float geometry_min_m = detail_rock_threshold_m(length(camera_offset), rock_node_min_m, abundance);
		detail_factors = detail_light_factors(abundance, geometry_min_m, spacing_eff, detail_horizon_spacing_m,
				sun_margin, earth_margin);
		sun_modulation = detail_modulation(detail_gradient, n, normalize(to_sun), to_eye);
		earth_modulation = detail_modulation(detail_gradient, n, normalize(to_earth), to_eye);
	}
	// The Sun and the earthshine through the one regolith reflectance (night-vision fix round I3).
	float sun_reflectance = regolith_reflectance_rough(n, normalize(to_sun), to_eye, theta_bar);
	float earth_reflectance = regolith_reflectance_rough(n, normalize(to_earth), to_eye, theta_bar);
	float light;
	if (detail_shading) {
		light = moon_regolith_scale * (sun_reflectance * sunlit * sun_visible * moon_sun_scene * detail_factors.x
				* sun_modulation + earth_reflectance * earth_visible * moon_earthshine * detail_factors.y
				* earth_modulation);
	} else {  // Task 9's expression as it was (detail off stays pixel-identical)
		light = moon_regolith_scale * (sun_reflectance * sunlit * sun_visible * moon_sun_scene + earth_reflectance
				* earth_visible * moon_earthshine);
	}
	// v stays half a texel inside the map (repeat would bleed the opposite pole into the polar rows).
	float half_texel_v = 0.5 / float(textureSize(albedo_texture, 0).y);
	vec3 albedo = texture(albedo_texture, vec2(global_uv.x, clamp(global_uv.y, half_texel_v, 1.0 - half_texel_v))).rgb;
	vec3 eclipse_light = umbra_glow * (1.0 - sunlit) * sun_visible * lommel_seeliger;
	vec3 color = albedo * (light + eclipse_light);
	if (atmosphere_enabled) {
		// Through the Earth's atmosphere only if the view ray crosses it before the ground (a camera near the Moon:
		// never; atm_sky_to_far_object).
		vec3 direction_world = normalize((INV_VIEW_MATRIX * vec4(VERTEX, 0.0)).xyz);
		vec3 transmittance;
		vec3 sky = atm_sky_to_far_object(atm_camera_local(), atmosphere_world_to_earth * direction_world,
				length(camera_offset), transmittance);
		color = color * atmosphere_exposure_gain * transmittance + sky;  // the eye adaptation's exposure gain on the lit Moon
	}
	if (diag_mode == 1 && normal_away) {
		color = vec3(1.0, 0.0, 0.0);
	}
	if (diag_mode == 2 && (mu0_raw <= 0.0 || sun_visible < 0.5)) {
		color = vec3(mu0_raw <= 0.0 ? 1.0 : 0.0, 0.0, sun_visible < 0.5 ? 1.0 : 0.0);
	}
	ALBEDO = color;
}
