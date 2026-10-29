class_name SurfaceClearance
extends RefCounted
## Keeps a camera at least MIN_CLEARANCE_M above the terrain surface of the Earth and the Moon (user 2026-10-29: no
## camera may come closer than 1 m or pierce the surface). The surface is the terrain renderer's height source
## (HeightSource: the core's terrain / sea on the geoid, else the flat rule / mean sphere); a camera below it is moved
## out along the local vertical to the surface + the clearance.

const MIN_CLEARANCE_M := 1.0
const _WGS84_E2 := 6.69437999014e-3  ## [S: NIMA TR8350.2] first eccentricity squared


## The body-fixed position `p_fixed_m` (m), moved out to MIN_CLEARANCE_M above `heights`' surface if it is closer.
static func clamp_fixed(p_fixed_m: Vector3, heights: HeightSource) -> Vector3:
	if heights == null or p_fixed_m.length() < 1.0:
		return p_fixed_m
	var geo := _geodetic(p_fixed_m, heights.body == "earth")  # (lat_deg, lon_deg, height above the reference surface)
	var surface: Dictionary = heights.sample(PackedFloat64Array([geo.x]), PackedFloat64Array([geo.y]))
	var floor_m: float = (surface["height_m"] as PackedFloat64Array)[0] + MIN_CLEARANCE_M
	if geo.z >= floor_m:
		return p_fixed_m
	return heights.position_fixed(geo.x, geo.y, floor_m)


## Geodetic latitude / longitude (deg) and height (m) of a body-fixed point: Earth WGS84 (Bowring, two iterations:
## sub-millimetre near the surface), Moon the mean sphere.
static func _geodetic(p: Vector3, earth: bool) -> Vector3:
	var lon := atan2(p.y, p.x)
	var rho := sqrt(p.x * p.x + p.y * p.y)
	if not earth:
		return Vector3(rad_to_deg(atan2(p.z, rho)), rad_to_deg(lon), p.length() - World.MOON_RADIUS_M)
	var a := Atmosphere.WGS84_A_M
	var lat := atan2(p.z, rho * (1.0 - _WGS84_E2))
	var n := a
	for i in 2:
		n = a / sqrt(1.0 - _WGS84_E2 * sin(lat) * sin(lat))
		lat = atan2(p.z + _WGS84_E2 * n * sin(lat), rho)
	n = a / sqrt(1.0 - _WGS84_E2 * sin(lat) * sin(lat))
	var h := rho / cos(lat) - n if absf(cos(lat)) > 1e-6 else absf(p.z) - n * (1.0 - _WGS84_E2)
	return Vector3(rad_to_deg(lat), rad_to_deg(lon), h)
