class_name AerosolRegions
extends RefCounted
## The observer's maritime aerosol weight (look-fixes T9b): the atmosphere has two aerosol table sets (continental and
## maritime, config/atmosphere/earth_optics.toml [aerosol.*]); every lookup blends them by one weight w in [0, 1]
## (core AtmosphereScattering, shaders/atmosphere.gdshaderinc). w is the water share of the ground around the eye's
## sub-point, averaged over a footprint that grows with the height (the air the eye looks through):
##   R = max(FOOTPRINT_MIN_M, FOOTPRINT_HORIZON_SHARE x the distance to the horizon sqrt(2 a h + h^2)).
## The map is data/textures/earth_water.png (the renderer's sea rule: 1 - the ETOPO 2022 land share above the EGM2008
## geoid, tools/data_pipeline/earth_albedo.py water_fraction), reduced to a pyramid of 2x2 means; w is the bilinear value
## at the sub-point, blended between the two pyramid levels around the footprint (continuous in place and height).
## Limits [call]: one weight for the whole view (a coast seen from the beach takes the mixed value both ways; from orbit
## the whole Earth takes the footprint's mix); no wind or season (sea salt inland, dust or smoke plumes are not modelled).

## [E] Smallest footprint radius: sea salt dominates the coastal aerosol over the first tens of km inland and the near
## haze an observer sees spans ~50 km at AOD 0.1 (visibility ~3.9 / extinction).
const FOOTPRINT_MIN_M := 50000.0
## [call] Footprint radius as a share of the distance to the horizon above the minimum (the half the haze sits in).
const FOOTPRINT_HORIZON_SHARE := 0.5
const EARTH_A_M := 6378137.0
## Pyramid base: 1024 x 512 (0.35 deg, 39 km at the equator).
const BASE_WIDTH := 1024
## Without the map: the continental set alone.
const FALLBACK_WEIGHT := 0.0

var _levels: Array[Image] = []


## The pyramid of the water map at path, or an empty one (weight FALLBACK_WEIGHT) if the file is missing.
static func from_file(path: String) -> AerosolRegions:
	var r := AerosolRegions.new()
	if not FileAccess.file_exists(path):
		push_warning("AerosolRegions: %s missing: continental aerosol everywhere" % path)
		return r
	var image := Image.load_from_file(path)
	if image == null or image.is_empty():
		push_warning("AerosolRegions: %s unreadable: continental aerosol everywhere" % path)
		return r
	r.set_map(image)
	return r


## Builds the pyramid from a water-fraction image (equirectangular, north up, longitude -180 at the left edge).
func set_map(image: Image) -> void:
	var base := image.duplicate() as Image
	base.convert(Image.FORMAT_RF)
	# Trilinear (mipmap) reduction: area means of the source.
	base.resize(BASE_WIDTH, BASE_WIDTH / 2, Image.INTERPOLATE_TRILINEAR)
	_levels = [base]
	while _levels[-1].get_width() > 4:
		var next := _levels[-1].duplicate() as Image
		next.resize(next.get_width() / 2, next.get_height() / 2, Image.INTERPOLATE_BILINEAR)  # 2 x 2 means
		_levels.append(next)


func has_map() -> bool:
	return not _levels.is_empty()


## Footprint radius (m) for an eye at height_m above the ellipsoid.
static func footprint_radius_m(height_m: float) -> float:
	var h := maxf(height_m, 0.0)
	return maxf(FOOTPRINT_MIN_M, FOOTPRINT_HORIZON_SHARE * sqrt(2.0 * EARTH_A_M * h + h * h))


## The maritime weight at geodetic latitude / longitude (deg) and height (m).
func weight(latitude_deg: float, longitude_deg: float, height_m: float) -> float:
	if _levels.is_empty():
		return FALLBACK_WEIGHT
	var texel_m := TAU * EARTH_A_M / BASE_WIDTH  # level 0 texel at the equator
	var level := clampf(log(footprint_radius_m(height_m) / texel_m) / log(2.0), 0.0, _levels.size() - 1.0)
	var l0 := int(floor(level))
	var l1 := mini(l0 + 1, _levels.size() - 1)
	var a := _sample(_levels[l0], latitude_deg, longitude_deg)
	var b := _sample(_levels[l1], latitude_deg, longitude_deg)
	return clampf(lerpf(a, b, level - l0), 0.0, 1.0)


## Bilinear at texel centres, wrapping in longitude, clamped at the poles.
static func _sample(image: Image, latitude_deg: float, longitude_deg: float) -> float:
	var w := image.get_width()
	var h := image.get_height()
	var x := (wrapf(longitude_deg + 180.0, 0.0, 360.0) / 360.0) * w - 0.5
	var y := clampf((90.0 - latitude_deg) / 180.0 * h - 0.5, 0.0, h - 1.0)
	var x0 := int(floor(x))
	var y0 := int(floor(y))
	var fx := x - x0
	var fy := y - y0
	var y1 := mini(y0 + 1, h - 1)
	var xa := posmod(x0, w)
	var xb := posmod(x0 + 1, w)
	var top := lerpf(image.get_pixel(xa, y0).r, image.get_pixel(xb, y0).r, fx)
	var bottom := lerpf(image.get_pixel(xa, y1).r, image.get_pixel(xb, y1).r, fx)
	return lerpf(top, bottom, fy)
