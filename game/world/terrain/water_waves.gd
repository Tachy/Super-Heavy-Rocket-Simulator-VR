class_name WaterWaves
extends RefCounted
## The sea's wave set (shading only: water.gdshaderinc perturbs the normal, the geometry stays at the geoid).
## Four directional Gerstner octaves with plane wave vectors in a tangent frame anchored near the camera. Their phases
## at the camera are computed here in double (k . x reaches 1e7 rad on the Earth, omega t 1e9 rad: float32 would lose
## both); the shader adds k . (fragment - camera), which is small and exact near the camera. The pattern is therefore
## fixed on the Earth while the camera moves and travels with the simulation time (time warp speeds it). The frame is
## re-anchored (vectors horizontal again; the pattern jumps) when the camera has moved REANCHOR_ANGLE_RAD away while it is
## higher than WAVE_FADE_END_M above the ellipsoid (no wave visible, the jump unseen); lower down only beyond
## LOW_REANCHOR_ANGLE_RAD, where the tilt of the anchored frame would start to show.
## A mission's sea state (set_sea_state) rescales the set: the first octave at the dominant deep-water wavelength
## g Tp^2 / (2 pi), the others at the calm set's ratios, all towards the waves' heading, slopes from Hs. Shading only:
## the pattern matches the sea's scale and direction, not the phases of the core's deck motion (spec non-goal).

## [E] Wavelengths of the four octaves of the calm set (m): swell to ripples.
const WAVELENGTHS_M: Array[float] = [60.0, 23.0, 9.0, 3.5]
## [E] Travel directions relative to the travel heading (WIND_HEADING_DEG or the sea state's, deg): spread so that
## no octave pair aligns.
const DIRECTION_OFFSETS_DEG: Array[float] = [0.0, 31.0, -37.0, 67.0]
## [E] Compass heading the waves travel towards (deg, from north through east).
const WIND_HEADING_DEG := 60.0
## [E] Slope k A of every octave of the calm set: gentle (Cox-Munk adds the unresolved slopes in the glint).
const SLOPE := 0.04
## [E] Upper bound of the slope k A of every octave in a sea state: the resolved variance 4 x 0.08^2 / 2 = 0.0128 stays
## below the glint's Cox-Munk variance at its 5 m/s wind (0.0286), so the glint keeps an unresolved part.
const MAX_SEA_SLOPE := 0.08
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
## Wavelengths (m), slope k A of every octave and travel heading (deg, clockwise from north) in use: the calm set or
## the sea state's.
var wavelengths_m: Array[float] = WAVELENGTHS_M.duplicate()
var slope := SLOPE
var heading_deg := WIND_HEADING_DEG
var _anchor_up := Vector3.ZERO  # Earth-fixed unit vector of the frame's anchor; zero: not anchored yet
var _sea := {}  # the sea state in use (set_sea_state); empty: the calm set


## Follows a sea state (FlightSim.landing_zone sea_state: hs_m, tp_s, heading_deg), or the calm set when it is empty.
## The first octave takes the dominant (peak) deep-water wavelength g Tp^2 / (2 pi), the others the calm set's ratios;
## every octave's slope is k_p Hs / 4, so that the resolved slope variance sum (k A)^2 / 2 = 4 (k_p Hs / 4)^2 / 2 equals
## (k_p a_s)^2 / 2 of the significant wave (amplitude a_s = Hs / 2): it follows Hs (and k_p), capped at MAX_SEA_SLOPE.
## (Hs 2.5 m, Tp 8 s: wavelength 99.9 m, slope 0.039 - close to the calm set's 0.04.) Re-anchors (the pattern jumps)
## only when the values change.
func set_sea_state(sea: Dictionary) -> void:
	if sea == _sea:
		return
	_sea = sea.duplicate()
	var hs := float(sea.get("hs_m", 0.0))
	var tp := float(sea.get("tp_s", 0.0))
	if hs <= 0.0 or tp <= 0.0:
		wavelengths_m = WAVELENGTHS_M.duplicate()
		slope = SLOPE
		heading_deg = WIND_HEADING_DEG
	else:
		var peak_wavelength := STANDARD_GRAVITY_MPS2 * tp * tp / TAU
		for i in WAVELENGTHS_M.size():
			wavelengths_m[i] = peak_wavelength * WAVELENGTHS_M[i] / WAVELENGTHS_M[0]
		slope = minf(TAU / peak_wavelength * hs / 4.0, MAX_SEA_SLOPE)
		heading_deg = float(sea.get("heading_deg", WIND_HEADING_DEG))
	if _anchor_up != Vector3.ZERO:
		_anchor(_anchor_up)


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
func resolved_slope_variance() -> float:
	return wavelengths_m.size() * 0.5 * slope * slope


## Angle the camera may move from the anchor before the frame re-anchors (rad): small while no wave is visible (the
## camera higher than WAVE_FADE_END_M above the WGS84 ellipsoid), large below.
static func reanchor_angle_rad(camera_fixed_m: Vector3) -> float:
	var r := camera_fixed_m.length()
	var sin_lat := camera_fixed_m.z / r if r > 0.0 else 0.0
	# first order in the flattening (tens of metres: irrelevant against the 20 km threshold)
	var height_m := r - HeightSource.WGS84_A_M * (1.0 - HeightSource.WGS84_F * sin_lat * sin_lat)
	return REANCHOR_ANGLE_RAD if height_m > WAVE_FADE_END_M else LOW_REANCHOR_ANGLE_RAD


## Angular frequency of octave i (rad/s), deep-water dispersion.
func angular_frequency(i: int) -> float:
	return sqrt(STANDARD_GRAVITY_MPS2 * TAU / wavelengths_m[i])


func _anchor(up: Vector3) -> void:
	_anchor_up = up
	var axis := Vector3(0, 0, 1) if absf(up.z) < 0.99 else Vector3(1, 0, 0)  # the pole: any other axis
	var east := axis.cross(up).normalized()
	var north := up.cross(east)
	for i in WAVELENGTHS_M.size():
		var heading := deg_to_rad(heading_deg + DIRECTION_OFFSETS_DEG[i])
		var k := Frames.to_render((north * cos(heading) + east * sin(heading)) * (TAU / wavelengths_m[i]))
		vectors[i] = Vector4(k.x, k.y, k.z, slope)
