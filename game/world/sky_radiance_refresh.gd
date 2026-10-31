class_name SkyRadianceRefresh
extends RefCounted
## Decides when the sky's radiance cubemap (Godot's ambient light and reflections of every lit object) must be drawn
## anew. Godot marks a sky dirty only on a write to its material: sky.gdshader reads global uniforms, so without one
## the cube keeps its first state (black, before the tables). World makes that write when `due` says the sky really
## changed. Rendering schedule only: nothing here is physics.

## [E] Sun about the eye (Earth-local) and the eye's zenith in the cube's axes. In the deepest twilight the sky falls by
## a decade per ~4 deg of the Sun, so 0.1 deg is a ~6 % step at worst, under 1 % by day; 24 s at the Earth's rotation,
## 1.5 s in low orbit, each a few frames of 128 px.
const ANGLE_RAD := deg_to_rad(0.1)
## [E] The eye's height: 200 m is 2.4 % of the air column above it (scale height ~8.5 km); higher up 2 % of the height
## moves the horizon (0.25 deg at 400 km) and the body's view factor by about as little.
const HEIGHT_MIN_M := 200.0
const HEIGHT_FRACTION := 0.02
## [E] The cube holds scene values, i.e. times the eye's gain: 2 % steps of a light level do not show.
const SCALE_LOG := 0.02
## [E] The part of the solar disc the Moon leaves at the eye (an eclipse's sky).
const SUN_FRACTION := 0.01

var _drawn := false
var _ready := false
var _earth_zenith := Vector3.ZERO
var _moon_zenith := Vector3.ZERO
var _sun_local := Vector3.ZERO
var _earth_height_m := 0.0
var _moon_height_m := 0.0
var _luminance_scale := 0.0
var _moon_light := 0.0
var _sun_fraction := 1.0


## Whether the state differs enough from the state of the last refresh; if so it becomes the new reference.
## tables_ready: Atmosphere.ready. earth_zenith / moon_zenith: unit vectors in the render world's axes, from the body's
## centre to the eye. moon_height_m: < 0 while the eye is not near the Moon. moon_light: atmosphere_moon_light's value.
func due(tables_ready: bool, earth_zenith: Vector3, earth_height_m: float, moon_zenith: Vector3, moon_height_m: float,
		sun_local: Vector3, luminance_scale: float, moon_light: float, sun_fraction: float) -> bool:
	if _drawn and tables_ready == _ready and not _sky_changed(earth_zenith, earth_height_m, moon_zenith, moon_height_m,
			sun_local, luminance_scale, moon_light, sun_fraction):
		return false
	_drawn = true
	_ready = tables_ready
	_earth_zenith = earth_zenith
	_earth_height_m = earth_height_m
	_moon_zenith = moon_zenith
	_moon_height_m = moon_height_m
	_sun_local = sun_local
	_luminance_scale = luminance_scale
	_moon_light = moon_light
	_sun_fraction = sun_fraction
	return true


func _sky_changed(earth_zenith: Vector3, earth_height_m: float, moon_zenith: Vector3, moon_height_m: float,
		sun_local: Vector3, luminance_scale: float, moon_light: float, sun_fraction: float) -> bool:
	return _turned(_earth_zenith, earth_zenith) or _turned(_sun_local, sun_local) \
			or _moved(_earth_height_m, earth_height_m) \
			or (moon_height_m < 0.0) != (_moon_height_m < 0.0) \
			or (moon_height_m >= 0.0 and (_turned(_moon_zenith, moon_zenith) or _moved(_moon_height_m, moon_height_m))) \
			or _scaled(_luminance_scale, luminance_scale) or _scaled(_moon_light, moon_light) \
			or absf(sun_fraction - _sun_fraction) > SUN_FRACTION


static func _turned(a: Vector3, b: Vector3) -> bool:
	return a.cross(b).length() > sin(ANGLE_RAD) or a.dot(b) < 0.0


static func _moved(a_m: float, b_m: float) -> bool:
	return absf(b_m - a_m) > maxf(HEIGHT_MIN_M, HEIGHT_FRACTION * absf(a_m))


## Relative change of a non-negative level; a level appearing from or falling to zero counts.
static func _scaled(a: float, b: float) -> bool:
	if a <= 0.0 or b <= 0.0:
		return (a <= 0.0) != (b <= 0.0)
	return absf(log(b / a)) > SCALE_LOG
