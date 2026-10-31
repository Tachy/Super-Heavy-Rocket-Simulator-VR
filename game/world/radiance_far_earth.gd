class_name RadianceFarEarth
extends RefCounted
## What the sky's radiance cube (sky.gdshader) shows of the far Earth and the lunar ground for an eye near the Moon: how the
## small Earth is drawn so that a coarse cube samples it steadily, and how much of each light the ground gets. Vectors from
## the eye, one frame, m.

## [E] The Earth's disc (with its air) is drawn at least this large in the cube: from the Moon it is about one texel of the
## octahedral 128 px cube (radius 0.95 deg, texel ~1.6 deg), so point sampling would give the earthshine 0 to 3 texels,
## jumping as the Earth moves. Five texels across sample it steadily.
const MIN_RADIUS_RAD := 0.07
## [E] The eye's own shadows stand for the ground around it up to GROUND_SHADOW_FULL_M above the ground: 90 % of a plate's
## view factor lies within 3 x its height, inside a polar crater's shadow (tens of km) up there. By GROUND_SHADOW_NONE_M the
## view covers more than the shadow: the eye's shadow says nothing about the ground (an orbiter in the Moon's shadow still
## sees the sunlit ground ahead; the terminator comes from the regolith's own term).
const GROUND_SHADOW_FULL_M := 1000.0
const GROUND_SHADOW_NONE_M := 10000.0


## How far (rad) the widened disc must rise to stand clear of the lunar ground sphere of the cube (0: it does).
static func lift_rad(to_earth_m: Vector3, to_moon_m: Vector3, moon_radius_m: float) -> float:
	var horizon_rad := asin(minf(moon_radius_m / to_moon_m.length(), 1.0))
	return maxf(horizon_rad + MIN_RADIUS_RAD - to_earth_m.angle_to(to_moon_m), 0.0)


## The direction of the disc after that rise: away from the Moon's centre, in the plane of the Earth and the Moon.
static func lifted_direction(to_earth_m: Vector3, to_moon_m: Vector3, lift: float) -> Vector3:
	var earth := to_earth_m.normalized()
	var axis := to_moon_m.cross(earth)
	if axis.length_squared() <= 0.0:
		return earth
	return earth.rotated(axis.normalized(), lift)


## The part of a light the ground in view gets, from the part of its disc the eye sees and the eye's height over the ground.
static func ground_fraction(eye_fraction: float, height_m: float) -> float:
	var weight := clampf((GROUND_SHADOW_NONE_M - height_m) / (GROUND_SHADOW_NONE_M - GROUND_SHADOW_FULL_M), 0.0, 1.0)
	return lerpf(1.0, eye_fraction, weight)
