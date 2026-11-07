// The active landing site's albedo tile (lunar sites plan Task 11), shared by terrain_moon.gdshader and moon_rock.gdshader.
// The tile is a RELATIVE albedo (1.0 = the global map's value there, tile mean 1, an illumination-normalised WAC mosaic at
// ~100 m, LunarSiteService.read_albedo): it multiplies the global albedo input only, the Hapke photometry stays the one
// lighting law (no baked shading). Over `site_albedo_blend_m` at the tile edge the factor fades to 1.0 (no seam).
//
// Mapping (uniforms set by LunarSiteService.albedo_uniforms; grid coordinates have the pixel centres at integers, the
// texture's texel k covers [k - 0.5, k + 0.5]):
//   site_albedo_grid.w = 1 simple cylindrical: line = grid.x - grid.z * lat_deg, sample = grid.y + grid.z * lon_deg
//                        (lon in 0 .. 360, taken at the copy nearest to the tile; grid.z = pixels per degree);
//   site_albedo_grid.w = 2 / 3 polar stereographic north / south (core PolarStereographicTile::position_of_direction):
//                        line = grid.x + sign * k * x / grid.z, sample = grid.y + k * y / grid.z, k = 2 R / (1 + sign z),
//                        (x, y, z) the MOON_ME unit vector, grid.z the pixel size in metres;
//   site_albedo_grid.w = 0: no site albedo (factor 1).
// site_albedo_bounds = (latitude min, latitude max, longitude min, longitude max) in rad of the tile's extent (the
// longitudes of a polar tile span everything): a cheap reject before the mapping.

// Mipmapped (LunarSiteService.apply_albedo): from orbit / a far horizon the ~100 m texture is minified; the level comes
// from the screen footprint of the terrain uv (derivatives taken before any branch), so it neither aliases nor shimmers.
uniform sampler2D site_albedo_texture : filter_linear_mipmap, repeat_disable;
uniform vec4 site_albedo_bounds = vec4(0.0);
uniform vec4 site_albedo_grid = vec4(0.0);
uniform float site_albedo_blend_m = 5000.0;

const float SITE_ALBEDO_MOON_RADIUS_M = 1737400.0;  // the lunar sphere (constants.hpp moon_radius_m)
const float SITE_ALBEDO_PI = 3.14159265358979;

// The factor at the equirectangular (u, v) of the terrain (u = (lon + 180) / 360, possibly outside 0 .. 1 across the
// antimeridian; v = (90 - lat) / 180).
// The albedo with the site factor applied, the product kept in the physical range: a relative factor above 1 on an already
// bright global texel must not push the albedo past 1 (or past what the global map held).
vec3 site_albedo_apply(vec3 albedo, float factor) {
	return clamp(albedo * factor, vec3(0.0), max(albedo, vec3(1.0)));
}

float site_albedo_factor(vec2 uv) {
	// Screen footprint of the uv in metres on the sphere (v spans 180 deg), before any branch (uniform control flow).
	float footprint_m = max(length(dFdx(uv)), length(dFdy(uv))) * SITE_ALBEDO_PI * SITE_ALBEDO_MOON_RADIUS_M;
	float mode = site_albedo_grid.w;
	if (mode < 0.5) {
		return 1.0;
	}
	float lat = (0.5 - uv.y) * SITE_ALBEDO_PI;
	if (lat < site_albedo_bounds.x || lat > site_albedo_bounds.y) {
		return 1.0;
	}
	float lon = (uv.x - 0.5) * 2.0 * SITE_ALBEDO_PI;
	vec2 size = vec2(textureSize(site_albedo_texture, 0));
	float line;
	float sample_px;
	float pixel_m;  // the tile pixel on the sphere, metres
	float edge_m;  // true distance to the nearest tile edge, metres (negative outside)
	if (mode < 1.5) {
		float ppd = site_albedo_grid.z;
		float period = 360.0 * ppd;
		float s0 = site_albedo_grid.y + ppd * degrees(lon);
		sample_px = s0 - period * floor((s0 - 0.5 * size.x) / period + 0.5);
		line = site_albedo_grid.x - ppd * degrees(lat);
		float metres_per_px = SITE_ALBEDO_MOON_RADIUS_M * SITE_ALBEDO_PI / (180.0 * ppd);
		pixel_m = metres_per_px;
		float d_lat = min(line + 0.5, size.y - 0.5 - line) * metres_per_px;
		float d_lon = min(sample_px + 0.5, size.x - 0.5 - sample_px) * metres_per_px * cos(lat);
		edge_m = min(d_lat, d_lon);
	} else {
		float sign_z = mode < 2.5 ? 1.0 : -1.0;
		float denominator = 1.0 + sign_z * sin(lat);
		if (denominator <= 1.0e-4) {
			return 1.0;
		}
		pixel_m = site_albedo_grid.z * 0.5 * denominator;  // conformal: true size = projected x denominator / 2
		float k = 2.0 * SITE_ALBEDO_MOON_RADIUS_M / denominator;
		float x = cos(lat) * cos(lon);
		float y = cos(lat) * sin(lon);
		line = site_albedo_grid.x + sign_z * k * x / site_albedo_grid.z;
		sample_px = site_albedo_grid.y + k * y / site_albedo_grid.z;
		float edge_px = min(min(line + 0.5, size.y - 0.5 - line), min(sample_px + 0.5, size.x - 0.5 - sample_px));
		// The projection is conformal with the scale 2 / denominator: a true distance is the projected one x denominator / 2.
		edge_m = edge_px * site_albedo_grid.z * 0.5 * denominator;
	}
	if (edge_m <= 0.0) {
		return 1.0;
	}
	float lod = max(log2(max(footprint_m / max(pixel_m, 1.0e-3), 1.0)), 0.0);
	float relative = textureLod(site_albedo_texture, (vec2(sample_px, line) + 0.5) / size, lod).r;
	float weight = smoothstep(0.0, 1.0, clamp(edge_m / max(site_albedo_blend_m, 1.0), 0.0, 1.0));
	return mix(1.0, relative, weight);
}
