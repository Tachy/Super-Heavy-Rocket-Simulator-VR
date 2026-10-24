class_name HeightSource
extends RefCounted
## Surface heights and positions of a body for the terrain renderer (worker-thread safe reads).
## With the core's terrain loaded (StarsimCore.load_terrain) the heights are the physical surface (relief, site
## plateaus, the sea on the geoid); without it the Earth uses the flat rule (a launch site's height within
## FLAT_RULE_SPAN_DEG of the site, else the ellipsoid, no water) and the Moon its mean sphere.

const WGS84_A_M := 6378137.0  # [S] WGS84 semi-major axis
const WGS84_F := 1.0 / 298.257223563  # [S] WGS84 flattening
const MOON_RADIUS_M := 1737400.0  # [S] IAU mean lunar radius (CLAUDE.md: Moon sphere R = 1737.4 km)
## [E] Flat rule: a sample within this many degrees (latitude and longitude) of a site gets the site's height
## (the span of the former ground patch).
const FLAT_RULE_SPAN_DEG := 2.0

## "earth" or "moon".
var body := "earth"
## StarsimCore (surface_samples / surface_resolution_m); null: the flat rule.
var core: Object = null
## Launch sites ({latitude_deg, longitude_deg, height_m}) for the flat rule.
var sites: Array = []


## Heights above the ellipsoid / sphere (m) and water flags at the given points: {height_m: PackedFloat64Array,
## water: PackedByteArray (non-zero = water)}.
func sample(lats_deg: PackedFloat64Array, lons_deg: PackedFloat64Array) -> Dictionary:
	if core != null:
		var samples: Dictionary = core.surface_samples(body, lats_deg, lons_deg)
		if bool(samples["present"]):
			return {"height_m": samples["height_m"], "water": samples["water"]}
	var heights := PackedFloat64Array()
	heights.resize(lats_deg.size())
	heights.fill(0.0)
	var water := PackedByteArray()
	water.resize(lats_deg.size())
	water.fill(0)
	if body == "earth" and not lats_deg.is_empty():
		var lat_min := lats_deg[0]
		var lat_max := lats_deg[0]
		for lat in lats_deg:
			lat_min = minf(lat_min, lat)
			lat_max = maxf(lat_max, lat)
		for site: Dictionary in sites:
			var site_lat: float = site["latitude_deg"]
			if site_lat <= lat_min - FLAT_RULE_SPAN_DEG or site_lat >= lat_max + FLAT_RULE_SPAN_DEG:
				continue  # no point near this site (most nodes: skips the per-point loop)
			var site_lon: float = site["longitude_deg"]
			var site_height: float = site["height_m"]
			for k in lats_deg.size():
				if absf(lats_deg[k] - site_lat) < FLAT_RULE_SPAN_DEG \
						and absf(wrapf(lons_deg[k] - site_lon, -180.0, 180.0)) < FLAT_RULE_SPAN_DEG:
					heights[k] = site_height
	return {"height_m": heights, "water": water}


## Horizontal resolution of the surface data at a point (m); INF without terrain data.
func resolution_m(lat_deg: float, lon_deg: float) -> float:
	if core == null:
		return INF
	return core.surface_resolution_m(body, lat_deg, lon_deg)


## Body-fixed position (m) of a point at height_m above the reference surface (Earth: WGS84, Moon: sphere).
func position_fixed(lat_deg: float, lon_deg: float, height_m: float) -> Vector3:
	if body == "moon":
		var la := deg_to_rad(lat_deg)
		var lo := deg_to_rad(lon_deg)
		return Vector3(cos(la) * cos(lo), cos(la) * sin(lo), sin(la)) * (MOON_RADIUS_M + height_m)
	return geodetic_to_fixed(lat_deg, lon_deg, height_m)


## Body-fixed positions (m) of points given by their unit geodetic normals (CubeSphere directions) and heights: the
## same surface as position_fixed without trigonometry (Earth: N (x, y, (1 - e^2) z) + h n, N = a / sqrt(1 - e^2 z^2)).
func positions_fixed(normals: PackedVector3Array, heights_m: PackedFloat64Array) -> PackedVector3Array:
	var result := PackedVector3Array()
	result.resize(normals.size())
	if body == "moon":
		for k in normals.size():
			result[k] = normals[k] * (MOON_RADIUS_M + heights_m[k])
		return result
	var e2 := WGS84_F * (2.0 - WGS84_F)
	for k in normals.size():
		var d := normals[k]
		var n := WGS84_A_M / sqrt(1.0 - e2 * d.z * d.z)
		var h := heights_m[k]
		result[k] = Vector3((n + h) * d.x, (n + h) * d.y, (n * (1.0 - e2) + h) * d.z)
	return result


## WGS84 geodetic coordinates -> Earth-fixed position (m).
static func geodetic_to_fixed(lat_deg: float, lon_deg: float, height_m := 0.0) -> Vector3:
	var e2 := WGS84_F * (2.0 - WGS84_F)
	var la := deg_to_rad(lat_deg)
	var lo := deg_to_rad(lon_deg)
	var n := WGS84_A_M / sqrt(1.0 - e2 * sin(la) * sin(la))
	return Vector3((n + height_m) * cos(la) * cos(lo), (n + height_m) * cos(la) * sin(lo),
			(n * (1.0 - e2) + height_m) * sin(la))
