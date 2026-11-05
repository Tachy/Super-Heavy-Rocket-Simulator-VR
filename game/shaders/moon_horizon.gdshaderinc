// The Moon terrain's baked horizon map and the visible fraction of a light's disc above it (terrain_moon.gdshader; the
// drawn rocks of moon_rock.gdshader read their node's map at the rock's base, Task 11). The quadtree sets the uniforms
// per node (TerrainQuadtree._upload).
// The node's horizon map: layer c (RGBAH, row x row) texel (i, j) = grid vertex (i, j) * horizon_stride, azimuths
// 4 c .. 4 c + 3 (elevation, rad); texel centres are the vertices, so the filter interpolates between them.
uniform float horizon_stride = 1.0;  // grid vertices per texel (TerrainNodeBuilder.HORIZON_STRIDE, set by the quadtree)
// A node below the measured data's level draws with its ancestor's map (the map's owner): its grid position maps to
// the owner's grid as CUSTOM2 * scale + offset (1, 0: the node's own map; set by the quadtree).
uniform float horizon_grid_scale = 1.0;
uniform vec2 horizon_grid_offset = vec2(0.0);
uniform sampler2DArray horizon_map : filter_linear, repeat_disable;
uniform float horizon_max_rad = 0.0;  // the highest elevation in the map (with margin): a light above it is not hidden
uniform bool horizon_enabled = false;  // a node without a horizon (no terrain data) lights without terrain shadows
// [E] |east| of the unnormalised east (cos latitude) below which the fragment counts as at a pole (within ~1e-6 rad).
const float POLE_EAST_MIN = 1e-6;

// Fraction of the disc of radius `radius_m` at `to_light` above the terrain horizon: the horizon elevation in the
// light's azimuth against the disc's elevation, the circular segment above a straight horizon (exact: d = signed
// height of the centre over the horizon in disc radii, area = 1/2 + (d sqrt(1 - d^2) + asin d) / pi).
// east / north: the fragment's local axes, up: its radial direction; uv: the horizon map's texture coordinates.
// margin: the disc centre's elevation over the horizon in its azimuth (rad; its elevation when no fetch is needed).
float terrain_visible_fraction(vec3 to_light, float radius_m, vec3 east, vec3 north, vec3 up, vec2 uv,
		inout float margin) {
	float distance_m = length(to_light);
	vec3 direction = to_light / distance_m;
	float angular_radius = radius_m / distance_m;  // asin(x) = x to 3.5e-6 (Sun) / 4.6e-5 (Earth) relative
	float elevation = asin(clamp(dot(direction, up), -1.0, 1.0));
	margin = elevation;
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
	margin = elevation - horizon;
	float d = clamp((elevation - horizon) / angular_radius, -1.0, 1.0);
	return clamp(0.5 + (d * sqrt(1.0 - d * d) + asin(d)) / PI, 0.0, 1.0);
}
