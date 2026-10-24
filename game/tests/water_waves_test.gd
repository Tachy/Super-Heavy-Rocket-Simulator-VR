# Headless test of the sea's wave set (WaterWaves): horizontal wave vectors, deep-water dispersion, a wave pattern fixed
# on the Earth while the camera moves, travelling with the simulation time. No GPU.
# Run: godot --headless --path game --script res://tests/water_waves_test.gd
extends SceneTree

## Mirrors of water.gdshaderinc (Cox & Munk 1954 clean-surface fit, glint_wind_mps default).
const COX_MUNK_SIGMA2_0 := 0.003
const COX_MUNK_SIGMA2_PER_MPS := 0.00512
const GLINT_WIND_MPS := 5.0

var _failures := 0


func _initialize() -> void:
	var waves := WaterWaves.new()
	var camera_fixed := HeightSource.geodetic_to_fixed(32.5, -61.0, 10.0)
	var t0 := 8.1e8  # TDB s, a 2025 epoch: large enough to expose float32 time
	waves.update(camera_fixed, t0)
	var up := Frames.to_render(camera_fixed.normalized())

	# Wave vectors: horizontal at the camera, |k| = 2 pi / wavelength, slopes as configured.
	var vectors_ok := true
	for i in WaterWaves.WAVELENGTHS_M.size():
		var k := waves.wave_vector(i)
		vectors_ok = vectors_ok and absf(k.length() - TAU / WaterWaves.WAVELENGTHS_M[i]) < 1e-12 \
				and absf(k.normalized().dot(up)) < 1e-9 and waves.vectors[i].w == WaterWaves.SLOPE
	_check(vectors_ok, "wave vectors horizontal at the camera, |k| = 2 pi / wavelength, slope in w")

	# Deep-water dispersion: omega^2 = g k.
	var dispersion_ok := true
	for i in WaterWaves.WAVELENGTHS_M.size():
		var k := TAU / WaterWaves.WAVELENGTHS_M[i]
		dispersion_ok = dispersion_ok and absf(waves.angular_frequency(i) ** 2 - WaterWaves.STANDARD_GRAVITY_MPS2 * k) < 1e-9
	_check(dispersion_ok, "deep-water dispersion omega^2 = g k")

	# A point 3 km away: its phase (camera phase + k . offset) is the same seen from a camera moved 2 km (fixed pattern).
	var point := Frames.to_render(HeightSource.geodetic_to_fixed(32.52, -61.01, 0.0))
	var phases_a := _phases_at(waves, point, Frames.to_render(camera_fixed))
	var moved := HeightSource.geodetic_to_fixed(32.51, -60.985, 10.0)
	waves.update(moved, t0)
	var phases_b := _phases_at(waves, point, Frames.to_render(moved))
	var fixed_error := _max_phase_error(phases_a, phases_b, [0.0, 0.0, 0.0, 0.0])
	_check(fixed_error < 1e-6, "wave pattern fixed on the Earth while the camera moves (phase error %s rad)"
			% String.num_scientific(fixed_error))

	# Time: after dt each wave's phase at a fixed point drops by omega dt (waves travel along k).
	var dt := 0.37
	waves.update(moved, t0 + dt)
	var phases_c := _phases_at(waves, point, Frames.to_render(moved))
	var shifts := []
	for i in 4:
		shifts.append(-waves.angular_frequency(i) * dt)
	var time_error := _max_phase_error(phases_b, phases_c, shifts)
	_check(time_error < 1e-6, "waves travel with the simulation time (phase error %s rad)" % String.num_scientific(time_error))

	# Low over the sea (waves visible) the frame does not re-anchor 2 deg away: the pattern does not jump.
	var anchored_k := waves.wave_vector(0)
	var low_far := HeightSource.geodetic_to_fixed(34.5, -61.0, 500.0)
	var phases_d := _phases_at(waves, point, Frames.to_render(moved))
	waves.update(low_far, t0 + dt)
	var phases_e := _phases_at(waves, point, Frames.to_render(low_far))
	var low_error := _max_phase_error(phases_d, phases_e, [0.0, 0.0, 0.0, 0.0])
	_check(waves.wave_vector(0) == anchored_k and low_error < 1e-6,
			"no re-anchoring 2 deg away below the wave fade distance (pattern jump %s rad)" % String.num_scientific(low_error))

	# Above the fade distance (no wave visible) the frame re-anchors after 1 deg: vectors horizontal at the new camera.
	var high_far := HeightSource.geodetic_to_fixed(34.5, -61.0, WaterWaves.WAVE_FADE_END_M + 1000.0)
	waves.update(high_far, t0)
	var high_up := Frames.to_render(high_far.normalized())
	_check(absf(waves.wave_vector(0).normalized().dot(high_up)) < 1e-9,
			"the wave frame re-anchors under a distant camera above the fade distance")

	# Low but beyond the large angle the frame re-anchors as well (the tilt would show).
	var very_far := HeightSource.geodetic_to_fixed(46.0, -61.0, 10.0)
	waves.update(very_far, t0)
	var very_far_up := Frames.to_render(very_far.normalized())
	_check(absf(waves.wave_vector(0).normalized().dot(very_far_up)) < 1e-9,
			"the wave frame re-anchors low over the sea beyond %.0f deg" % rad_to_deg(WaterWaves.LOW_REANCHOR_ANGLE_RAD))
	_check(WaterWaves.reanchor_angle_rad(HeightSource.geodetic_to_fixed(80.0, 10.0, 19000.0)) == WaterWaves.LOW_REANCHOR_ANGLE_RAD
			and WaterWaves.reanchor_angle_rad(HeightSource.geodetic_to_fixed(80.0, 10.0, 21000.0)) == WaterWaves.REANCHOR_ANGLE_RAD,
			"re-anchoring angle switches at the wave fade distance (ellipsoid height, also near the pole)")

	# Gerstner steepness within the deep-water trochoid limit: Q_i <= 1, so each octave's lift Q_i k A <= k A.
	_check(WaterWaves.STEEPNESS > 0.0 and WaterWaves.STEEPNESS <= 1.0
			and WaterWaves.STEEPNESS * WaterWaves.SLOPE <= WaterWaves.SLOPE,
			"Gerstner steepness Q_i = %.2f <= 1 (lift per octave %.3f <= k A %.3f)"
			% [WaterWaves.STEEPNESS, WaterWaves.STEEPNESS * WaterWaves.SLOPE, WaterWaves.SLOPE])

	# Resolved slope variance: sum (k A)^2 / 2, and below the Cox-Munk total at the glint wind (5 m/s) so the glint
	# keeps a positive unresolved part.
	var cox_munk_sigma2 := COX_MUNK_SIGMA2_0 + COX_MUNK_SIGMA2_PER_MPS * GLINT_WIND_MPS
	var resolved := waves.resolved_slope_variance()
	_check(absf(resolved - 4.0 * 0.5 * 0.04 * 0.04) < 1e-15 and resolved < cox_munk_sigma2,
			"resolved slope variance %.4f below Cox-Munk sigma^2 %.4f (unresolved %.4f)"
			% [resolved, cox_munk_sigma2, cox_munk_sigma2 - resolved])

	# A mission's sea state (Hs 2.5 m, Tp 8 s, towards 120 deg): the first octave at g Tp^2 / (2 pi), the others at the
	# calm ratios, the octaves towards the heading, slope k_p Hs / 4 (resolved variance (k_p Hs / 2)^2 / 2); an empty
	# sea state restores the calm set; a steep sea is capped.
	waves.set_sea_state({"hs_m": 2.5, "tp_s": 8.0, "heading_deg": 120.0})
	var peak := WaterWaves.STANDARD_GRAVITY_MPS2 * 64.0 / TAU
	var k_p := TAU / peak
	var east := Frames.to_render(Vector3(0, 0, 1).cross(waves._anchor_up).normalized())
	var north := Frames.to_render(waves._anchor_up).cross(east)  # the anchored frame (WaterWaves._anchor)
	var sea_ok := absf(waves.resolved_slope_variance() - 0.5 * pow(k_p * 2.5 / 2.0, 2.0)) < 1e-15
	for i in WaterWaves.WAVELENGTHS_M.size():
		var k := waves.wave_vector(i)
		var heading := rad_to_deg(atan2(k.dot(east), k.dot(north)))
		sea_ok = sea_ok and absf(k.length() - k_p * WaterWaves.WAVELENGTHS_M[0] / WaterWaves.WAVELENGTHS_M[i]) < 1e-12 \
				and absf(fposmod(heading - 120.0 - WaterWaves.DIRECTION_OFFSETS_DEG[i] + 180.0, 360.0) - 180.0) < 1e-6 \
				and absf(waves.vectors[i].w - k_p * 2.5 / 4.0) < 1e-15
	_check(sea_ok, "sea state: peak wavelength %.1f m, octaves towards the heading, slope %.4f" % [peak, k_p * 2.5 / 4.0])
	waves.set_sea_state({"hs_m": 8.0, "tp_s": 6.0, "heading_deg": 0.0})
	_check(waves.slope == WaterWaves.MAX_SEA_SLOPE and waves.resolved_slope_variance() < cox_munk_sigma2,
			"a steep sea is capped at slope %.2f (resolved variance below Cox-Munk)" % WaterWaves.MAX_SEA_SLOPE)
	waves.set_sea_state({})
	_check(waves.slope == WaterWaves.SLOPE and absf(waves.wave_vector(0).length() - TAU / WaterWaves.WAVELENGTHS_M[0]) < 1e-12,
			"no sea state: the calm set again")

	print("WATER WAVES TEST ","PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


# Phase of every wave at a render-frame point as the shader computes it: camera phase + k . (point - camera).
static func _phases_at(waves: WaterWaves, point: Vector3, camera: Vector3) -> Array:
	var phases := []
	for i in 4:
		phases.append(waves.phases[i] + waves.wave_vector(i).dot(point - camera))
	return phases


static func _max_phase_error(a: Array, b: Array, shifts: Array) -> float:
	var error := 0.0
	for i in a.size():
		error = maxf(error, absf(wrapf(b[i] - a[i] - shifts[i], -PI, PI)))
	return error


func _check(condition: bool, label: String) -> void:
	print(("  ok    " if condition else "  FAIL  ") + label)
	if not condition:
		_failures += 1
