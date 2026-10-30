shader_type spatial;
// Moon terrain node (TerrainQuadtree): the model of moon.gdshader (the regolith reflectance of moon_regolith.gdshaderinc, Earth shadow with the
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

// CUSTOM1.u runs past 1 on nodes across the antimeridian (continuous inside the node), hence repeat.
uniform sampler2D albedo_texture : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
global uniform vec3 moon_sun_position_local;    // apparent, from the Moon's centre (World)
global uniform vec3 moon_earth_position_local;  // from the Moon's centre (World)
// The camera's Moon-local render position, float32 hi + lo parts (TerrainQuadtree.camera_global, once per frame).
global uniform vec3 moon_terrain_camera_hi;
global uniform vec3 moon_terrain_camera_lo;
// The node's horizon map: RGBAH, texel (a * 4 + c, b) = vertex (a, b), azimuths 4 c .. 4 c + 3 (elevation, rad).
uniform sampler2D horizon_map : filter_nearest;
uniform bool horizon_enabled = false;  // a node without a horizon (no terrain data) lights without terrain shadows
uniform float sun_radius_m = 695700000.0;
uniform float earth_radius_m = 6371000.0;
// The constants of moon.gdshader (same look as the body mesh from scaled space).
uniform vec3 umbra_glow = vec3(0.020, 0.005, 0.0015);

varying vec3 local_position;
varying vec3 local_normal;
varying vec2 horizon_grid;  // CUSTOM2: the position in the node's vertex grid (0 .. 64), the horizon map's texel
varying vec2 global_uv;  // equirectangular u (continuous in the node), v
varying vec3 camera_offset;  // fragment - camera (node-local render axes, metres; exact near the camera)

void vertex() {
	vec3 camera_local = terrain_camera_local(moon_terrain_camera_hi, moon_terrain_camera_lo);
	float morph = terrain_morph_factor(VERTEX, camera_local);
	VERTEX += CUSTOM0.xyz * morph;
	NORMAL = normalize(mix(NORMAL, CUSTOM3.xyz, morph));
	local_position = VERTEX + vertex_offset;
	local_normal = NORMAL;
	global_uv = CUSTOM1.xy;
	horizon_grid = CUSTOM2.xy;
	camera_offset = VERTEX - camera_local;
}

// The four azimuths 4 block .. 4 block + 3 of the horizon (rad), bilinear between the four vertices around the fragment.
vec4 horizon_block(ivec2 cell, vec2 weight, int block) {
	int x = cell.x * 4 + block;
	vec4 h00 = texelFetch(horizon_map, ivec2(x, cell.y), 0);
	vec4 h10 = texelFetch(horizon_map, ivec2(x + 4, cell.y), 0);
	vec4 h01 = texelFetch(horizon_map, ivec2(x, cell.y + 1), 0);
	vec4 h11 = texelFetch(horizon_map, ivec2(x + 4, cell.y + 1), 0);
	return mix(mix(h00, h10, weight.x), mix(h01, h11, weight.x), weight.y);
}

// Fraction of the disc of radius `radius_m` at `to_light` above the terrain horizon: the horizon elevation in the
// light's azimuth against the disc's elevation, the circular segment above a straight horizon (exact: d = signed
// height of the centre over the horizon in disc radii, area = 1/2 + (d sqrt(1 - d^2) + asin d) / pi).
// east / north: the fragment's local axes, up: its radial direction.
float terrain_visible_fraction(vec3 to_light, float radius_m, vec3 east, vec3 north, vec3 up, ivec2 cell,
		vec2 weight) {
	float distance_m = length(to_light);
	vec3 direction = to_light / distance_m;
	float angular_radius = radius_m / distance_m;  // asin(x) = x to 3.5e-6 (Sun) / 4.6e-5 (Earth) relative
	float elevation = asin(clamp(dot(direction, up), -1.0, 1.0));
	float azimuth = atan(dot(direction, east), dot(direction, north));
	float sector = (azimuth < 0.0 ? azimuth + TAU : azimuth) * (16.0 / TAU);
	int k0 = int(floor(sector)) & 15;
	int c0 = k0 & 3;
	vec4 block = horizon_block(cell, weight, k0 >> 2);
	// The next sector is in the same texel block except after the block's last azimuth.
	float next = c0 < 3 ? block[(c0 + 1) & 3] : horizon_block(cell, weight, ((k0 >> 2) + 1) & 3).x;
	float horizon = mix(block[c0], next, fract(sector));
	float d = clamp((elevation - horizon) / angular_radius, -1.0, 1.0);
	return clamp(0.5 + (d * sqrt(1.0 - d * d) + asin(d)) / PI, 0.0, 1.0);
}

void fragment() {
	vec3 n = normalize(local_normal);
	vec3 to_sun = moon_sun_position_local - local_position;
	vec3 to_earth = moon_earth_position_local - local_position;
	vec3 to_eye = normalize(-camera_offset);
	float mu0 = max(dot(n, normalize(to_sun)), 0.0);
	float mu = max(dot(n, to_eye), 1e-3);
	float lommel_seeliger = 2.0 * mu0 / (mu0 + mu);
	float sunlit = 1.0 - occulted_fraction(to_sun, sun_radius_m, to_earth, earth_radius_m);
	float sun_visible = 1.0;  // above the terrain horizon
	float earth_visible = 1.0;
	if (horizon_enabled) {
		// Local axes at the fragment in the Moon-fixed render frame (the pole is +Y; at a pole any east will do).
		vec3 up = normalize(local_position);
		vec3 east = vec3(up.z, 0.0, -up.x);
		east = length(east) > 1e-6 ? normalize(east) : vec3(0.0, 0.0, 1.0);
		vec3 north = cross(up, east);
		ivec2 cell = ivec2(min(floor(horizon_grid), vec2(63.0)));
		vec2 weight = horizon_grid - vec2(cell);
		sun_visible = terrain_visible_fraction(to_sun, sun_radius_m, east, north, up, cell, weight);
		earth_visible = terrain_visible_fraction(to_earth, earth_radius_m, east, north, up, cell, weight);
	}
	// The Sun and the earthshine through the one regolith reflectance (night-vision fix round I3).
	float light = moon_regolith_scale * (regolith_reflectance(n, normalize(to_sun), to_eye) * sunlit * sun_visible
			* moon_sun_scene + regolith_reflectance(n, normalize(to_earth), to_eye) * earth_visible * moon_earthshine);
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
	ALBEDO = color;
}
