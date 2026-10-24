# Headless test of the sea's wave set (WaterWaves): horizontal wave vectors, deep-water dispersion, a wave pattern fixed
# on the Earth while the camera moves, travelling with the simulation time. No GPU.
# Run: godot --headless --path game --script res://tests/water_waves_test.gd
extends SceneTree

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

	# Far from the anchor the frame re-anchors: the vectors are horizontal again at the new camera.
	var far := HeightSource.geodetic_to_fixed(35.0, -58.0, 10.0)
	waves.update(far, t0)
	var far_up := Frames.to_render(far.normalized())
	_check(absf(waves.wave_vector(0).normalized().dot(far_up)) < 1e-9, "the wave frame re-anchors under a distant camera")

	print("WATER WAVES TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
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
