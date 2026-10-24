class_name WaterWaves
extends RefCounted
## The calm sea's wave set (shading only: water.gdshaderinc perturbs the normal, the geometry stays at the geoid).
## Four directional Gerstner octaves with plane wave vectors in a tangent frame anchored near the camera. Their phases
## at the camera are computed here in double (k . x reaches 1e7 rad on the Earth, omega t 1e9 rad: float32 would lose
## both); the shader adds k . (fragment - camera), which is small and exact near the camera. The pattern is therefore
## fixed on the Earth while the camera moves and travels with the simulation time (time warp speeds it). The frame is
## re-anchored (vectors horizontal again; the pattern jumps) when the camera has moved REANCHOR_ANGLE_RAD away while it is
## higher than WAVE_FADE_END_M above the ellipsoid (no wave visible, the jump unseen); lower down only beyond
## LOW_REANCHOR_ANGLE_RAD, where the tilt of the anchored frame would start to show.

## [E] Wavelengths of the four octaves (m): swell to ripples.
const WAVELENGTHS_M: Array[float] = [60.0, 23.0, 9.0, 3.5]
## [E] Travel directions relative to WIND_HEADING_DEG (deg): spread so that no octave pair aligns.
const DIRECTION_OFFSETS_DEG: Array[float] = [0.0, 31.0, -37.0, 67.0]
## [E] Compass heading the waves travel towards (deg, from north through east).
const WIND_HEADING_DEG := 60.0
## [E] Slope k A of every octave: gentle (Cox-Munk adds the unresolved slopes in the glint).
const SLOPE := 0.04
## [E] Gerstner steepness Q_i of every octave (lift Q_i k_i A_i); <= 1, the deep-water trochoid limit
## [S: GPU Gems 1, ch. 1 (Finch 2004): Q_i = 1 gives the sharpest crests, beyond it loops] (shader global
## water_wave_steepness).
const STEEPNESS := 0.5
## [S] Standard gravity (BIPM), deep-water dispersion omega^2 = g k.
const STANDARD_GRAVITY_MPS2 := 9.80665
## [E] Re-anchoring distance of the tangent frame above WAVE_FADE_END_M (1 deg ~ 111 km).
const REANCHOR_ANGLE_RAD := PI / 180.0
## [E] Re-anchoring distance while the waves are visible (the frame tilts by this angle: sin 10 deg = 17 % of k
## vertical at most; a low flight of 1100 km).
const LOW_REANCHOR_ANGLE_RAD := 10.0 * PI / 180.0
## Mirror of water.gdshaderinc WAVE_FADE_END_M: beyond this camera distance the wave normals are flat.
const WAVE_FADE_END_M := 20000.0

## Columns: wave vector (rad/m, body-local render axes) in xyz, slope k A in w (shader global water_wave_vectors).
var vectors := Projection()
## Phase of each wave at the camera (rad, [0, 2 pi)) at the update's instant (shader global water_wave_phases).
var phases := Vector4.ZERO
var _anchor_up := Vector3.ZERO  # Earth-fixed unit vector of the frame's anchor; zero: not anchored yet


## Recomputes the wave vectors (on re-anchoring) and the phases for a camera at camera_fixed_m (Earth-fixed metres)
## at simulation time t_s (TDB seconds).
func update(camera_fixed_m: Vector3, t_s: float) -> void:
	var up := camera_fixed_m.normalized()
	if _anchor_up == Vector3.ZERO or _anchor_up.angle_to(up) > reanchor_angle_rad(camera_fixed_m):
		_anchor(up)
	var camera := Frames.to_render(camera_fixed_m)
	for i in WAVELENGTHS_M.size():
		phases[i] = fposmod(wave_vector(i).dot(camera) - angular_frequency(i) * t_s, TAU)


## Wave vector of octave i (rad/m, body-local render axes).
func wave_vector(i: int) -> Vector3:
	var column: Vector4 = vectors[i]
	return Vector3(column.x, column.y, column.z)


## Slope variance the octaves resolve at full strength (sum (k A)^2 / 2 of sinusoids); the shader takes it (faded with
## the octaves) out of the glint's Cox-Munk variance.
static func resolved_slope_variance() -> float:
	return WAVELENGTHS_M.size() * 0.5 * SLOPE * SLOPE


## Angle the camera may move from the anchor before the frame re-anchors (rad): small while no wave is visible (the
## camera higher than WAVE_FADE_END_M above the WGS84 ellipsoid), large below.
static func reanchor_angle_rad(camera_fixed_m: Vector3) -> float:
	var r := camera_fixed_m.length()
	var sin_lat := camera_fixed_m.z / r if r > 0.0 else 0.0
	# first order in the flattening (tens of metres: irrelevant against the 20 km threshold)
	var height_m := r - HeightSource.WGS84_A_M * (1.0 - HeightSource.WGS84_F * sin_lat * sin_lat)
	return REANCHOR_ANGLE_RAD if height_m > WAVE_FADE_END_M else LOW_REANCHOR_ANGLE_RAD


## Angular frequency of octave i (rad/s), deep-water dispersion.
static func angular_frequency(i: int) -> float:
	return sqrt(STANDARD_GRAVITY_MPS2 * TAU / WAVELENGTHS_M[i])


func _anchor(up: Vector3) -> void:
	_anchor_up = up
	var axis := Vector3(0, 0, 1) if absf(up.z) < 0.99 else Vector3(1, 0, 0)  # the pole: any other axis
	var east := axis.cross(up).normalized()
	var north := up.cross(east)
	for i in WAVELENGTHS_M.size():
		var heading := deg_to_rad(WIND_HEADING_DEG + DIRECTION_OFFSETS_DEG[i])
		var k := Frames.to_render((north * cos(heading) + east * sin(heading)) * (TAU / WAVELENGTHS_M[i]))
		vectors[i] = Vector4(k.x, k.y, k.z, SLOPE)
