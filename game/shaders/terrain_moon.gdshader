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
// The node's horizon map: layer c (RGBAH, row x row) texel (i, j) = grid vertex (i, j) * horizon_stride, azimuths
// 4 c .. 4 c + 3 (elevation, rad); texel centres are the vertices, so the filter interpolates between them.
uniform float horizon_stride = 1.0;  // grid vertices per texel (TerrainNodeBuilder.HORIZON_STRIDE, set by the quadtree)
uniform sampler2DArray horizon_map : filter_linear, repeat_disable;
uniform float horizon_max_rad = 0.0;  // the highest elevation in the map (with margin): a light above it is not hidden
uniform bool horizon_enabled = false;  // a node without a horizon (no terrain data) lights without terrain shadows
// [E] |east| of the unnormalised east (cos latitude) below which the fragment counts as at a pole (within ~1e-6 rad).
const float POLE_EAST_MIN = 1e-6;
// Set from GDScript (World: the Sun's and the Earth's radius).
uniform float sun_radius_m;
uniform float earth_radius_m;
// The constants of moon.gdshader (same look as the body mesh from scaled space).
uniform vec3 umbra_glow = vec3(0.020, 0.005, 0.0015);

varying vec3 local_position;
varying vec3 local_normal;
varying vec2 horizon_uv;  // the horizon map's texture coordinates, from CUSTOM2 (the position in the vertex grid, 0 .. 64)
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
	// Texel centres are the grid vertices.
	horizon_uv = (CUSTOM2.xy / horizon_stride + 0.5) / float(textureSize(horizon_map, 0).x);
	camera_offset = VERTEX - camera_local;
}

// Fraction of the disc of radius `radius_m` at `to_light` above the terrain horizon: the horizon elevation in the
// light's azimuth against the disc's elevation, the circular segment above a straight horizon (exact: d = signed
// height of the centre over the horizon in disc radii, area = 1/2 + (d sqrt(1 - d^2) + asin d) / pi).
// east / north: the fragment's local axes, up: its radial direction; uv: the horizon map's texture coordinates.
float terrain_visible_fraction(vec3 to_light, float radius_m, vec3 east, vec3 north, vec3 up, vec2 uv) {
	float distance_m = length(to_light);
	vec3 direction = to_light / distance_m;
	float angular_radius = radius_m / distance_m;  // asin(x) = x to 3.5e-6 (Sun) / 4.6e-5 (Earth) relative
	float elevation = asin(clamp(dot(direction, up), -1.0, 1.0));
	if (elevation - angular_radius > horizon_max_rad) {
		return 1.0;  // the whole disc above the node's highest horizon: no fetch
	}
	float azimuth = atan(dot(direction, east), dot(direction, north));
	float sector = (azimuth < 0.0 ? azimuth + TAU : azimuth) * (16.0 / TAU);
	int k0 = int(floor(sector)) & 15;
	int c0 = k0 & 3;
	int layer = k0 >> 2;
	vec4 block = textureLod(horizon_map, vec3(uv, float(layer)), 0.0);
	// The next sector is in the same layer except after the layer's last azimuth.
	float next = c0 < 3 ? block[(c0 + 1) & 3] : textureLod(horizon_map, vec3(uv, float((layer + 1) & 3)), 0.0).x;
	float horizon = mix(block[c0], next, fract(sector));
	float d = clamp((elevation - horizon) / angular_radius, -1.0, 1.0);
	return clamp(0.5 + (d * sqrt(1.0 - d * d) + asin(d)) / PI, 0.0, 1.0);
}

void fragment() {
	vec3 n = normalize(local_normal);
	vec3 to_sun = moon_sun_position_local - local_position;
	vec3 to_earth = moon_earth_position_local - local_position;
	vec3 to_eye = normalize(-camera_offset);
	float mu0_raw = dot(n, normalize(to_sun));
	float mu0 = max(mu0_raw, 0.0);
	float mu = max(dot(n, to_eye), 1e-3);
	float lommel_seeliger = 2.0 * mu0 / (mu0 + mu);
	float sunlit = 1.0 - occulted_fraction(to_sun, sun_radius_m, to_earth, earth_radius_m);
	float sun_visible = 1.0;  // above the terrain horizon
	float earth_visible = 1.0;
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
			sun_visible = terrain_visible_fraction(to_sun, sun_radius_m, east, north, up, horizon_uv);
		}
		if (earth_matters) {
			earth_visible = terrain_visible_fraction(to_earth, earth_radius_m, east, north, up, horizon_uv);
		}
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
