# CloudLayer's cross-fade (M8 weather and clouds, Task 8; controller ruling: no pop on a pair or source change, only two
# field textures resident). Headless: godot --headless --path game --script res://tests/cloud_layer_test.gd
# Small fake snapshots (2 x 3 cells x 32 layers) stand in for the service's dictionaries. Checked every frame:
# - the weight of slot 1 moves by at most dt / FADE_S (no jump of the visible mix);
# - a slot's content changes only while that slot has no weight;
# - the pair ends in its real blend and then tracks the service's blend; weather off fades out the same way.
extends SceneTree

const DT := 1.0 / 60.0

var _failures := 0
var _layer: CloudLayer


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	var core := StarsimCore.new()
	_layer = CloudLayer.new()
	root.add_child(_layer)
	_layer.setup(core)
	_layer.wait_until_ready()
	var a := _fake("a")
	var b := _fake("b")
	var c := _fake("c")
	var d := _fake("d")
	var camera := Vector3(6378137.0 + 100.0, 0.0, 0.0)

	# From nothing to the pair (a, b) at blend 0.3: fades in, ends at 0.3.
	_announce(a, b)
	_check(_drive(camera, 0.3, 8.0), "first pair: continuous")
	_check(_state_is_pair(["era5|a|2", "era5|b|2"], 0.3), "first pair resident at blend 0.3: %s" % _layer.state())
	# Tracking: the blend follows the service at once.
	_layer.update(camera, 0.35, DT)
	_check(_state_is_pair(["era5|a|2", "era5|b|2"], 0.35), "tracks the blend")

	# The hour rolls over: (b, c) at blend 0.02, b is resident - only c is uploaded.
	_announce(b, c)
	_check(_drive(camera, 0.02, 8.0), "next hour: continuous")
	_check(_state_is_pair(["era5|b|2", "era5|c|2"], 0.02), "next hour resident: %s" % _layer.state())

	# (b, c) -> (d, a): no snapshot shared; the dominant new one (a at blend 0.8) comes first.
	_announce(d, a)
	_check(_drive(camera, 0.8, 12.0), "source change: continuous")
	_check(_state_is_pair(["era5|d|2", "era5|a|2"], 0.8), "source change resident: %s" % _layer.state())

	# Again a pair sharing nothing (an analog -> ERA5 switch), now with the old pair's second snapshot visible.
	_announce(b, c)
	_check(_drive(camera, 0.6, 12.0), "disjoint pair: continuous")
	_check(_state_is_pair(["era5|b|2", "era5|c|2"], 0.6), "disjoint pair resident: %s" % _layer.state())

	# Weather off: both empty, faded out.
	_layer.set_target({}, {})
	_check(_drive(camera, 0.0, 8.0), "weather off: continuous")
	_check(_state_is(["", ""], -1.0), "weather off: nothing resident: %s" % _layer.state())

	_test_light_follows_fade(core, camera)
	_check(ShaderGlobals.value(&"cloud_column_before") == _layer._empty_column and _layer._empty_column.get_format() == Image.FORMAT_LA8,
			"an empty slot binds the zero LA8 column texture (cover 0, not an R8 read as 1)")

	_layer.shutdown()
	print("CLOUD LAYER TEST %s" % ("PASSED" if _failures == 0 else "FAILED (%d)" % _failures))
	quit(1 if _failures > 0 else 0)


## Final review I1: the CPU light (StarsimCore.weather_cloud_light) follows the visible cross-fade. A slab fades in from
## nothing and out again; at every frame the core's pair is the layer's (slot files, w1), the ground's Sun transmittance
## moves monotonically, and each pushed state gives exactly the light of the blended column depths (reference: the same
## weight on the real slab and an explicit empty-field snapshot).
func _test_light_follows_fade(core: StarsimCore, camera: Vector3) -> void:
	var slab_file := OS.get_temp_dir().path_join("starsim_layer_slab_%d.wthr" % OS.get_process_id())
	var clear_file := OS.get_temp_dir().path_join("starsim_layer_clear_%d.wthr" % OS.get_process_id())
	core.weather_write_slab_snapshot(slab_file, 2000.0, 3000.0, 3e-4, 1.0)
	core.weather_write_slab_snapshot(clear_file, 2000.0, 3000.0, 0.0, 0.0)
	var slab: Dictionary = core.weather_load_snapshot(slab_file)
	core.weather_load_snapshot(clear_file)
	var ground := Vector3(6378137.0, 0.0, 0.0)
	var sun := Vector3(1.0, 0.0, 0.0)
	var samples: Array = []  # [pushed w1, light dictionary] per distinct push
	var transmittance: Array = []
	for phase in range(2):
		_layer.set_target(slab if phase == 0 else {}, slab if phase == 0 else {})
		for i in range(int(4.0 / DT)):
			_layer.update(camera, 1.0, DT)
			var pushed: Dictionary = _layer._pushed_light
			var light: Dictionary = core.weather_cloud_light(ground, sun)
			transmittance.append(float(light["sun_transmittance"]))
			if phase == 0 and (samples.is_empty() or samples.back()[0] != pushed["w1"]) and slab_file in pushed["files"]:
				samples.append([float(pushed["w1"]), light, pushed["files"].duplicate()])
			if _layer.state()["uploading"]:
				OS.delay_msec(1)
	var rising := true
	var in_phase := true
	var turned := transmittance.find(transmittance.min())
	for i in range(1, turned + 1):
		in_phase = in_phase and transmittance[i] <= transmittance[i - 1] + 1e-12
	for i in range(turned + 1, transmittance.size()):
		rising = rising and transmittance[i] >= transmittance[i - 1] - 1e-12
	_check(in_phase and rising and transmittance[turned] < 0.01 and absf(float(transmittance.back()) - 1.0) < 1e-12,
			"CPU Sun transmittance falls monotonically while the slab fades in, rises back to 1 when it fades out (min %.4f)" % transmittance[turned])
	var mid_ok := not samples.is_empty()
	var mid_count := 0
	for sample: Array in samples:
		var files: Array = sample[2]  # the empty slot is an explicit snapshot without cloud in the reference
		core.weather_set_active(files[0] if files[0] != "" else clear_file, files[1] if files[1] != "" else clear_file, sample[0])
		var reference: Dictionary = core.weather_cloud_light(ground, sun)
		mid_ok = mid_ok and reference == sample[1]
		mid_count += 1 if sample[0] > 0.2 and sample[0] < 0.8 else 0
	_check(mid_ok and mid_count > 5, "CPU light at %d mid-fade states equals the blended state of slab and empty field" % mid_count)
	_check(_layer._pushed_light["files"] == ["", ""], "faded out: the core's pair is the empty slots")
	core.weather_set_active("", "", 0.0)
	DirAccess.remove_absolute(slab_file)
	DirAccess.remove_absolute(clear_file)


## What the service does: every announcement carries snapshot dictionaries with their texels (the layer strips them once
## uploaded; a snapshot announced again was loaded again).
func _announce(before: Dictionary, after: Dictionary) -> void:
	_layer.set_target(before.duplicate(), after.duplicate())


func _fake(key: String) -> Dictionary:
	var texels := PackedByteArray()
	texels.resize(2 * 3 * 32 * 4)
	return {"ok": true, "kind": "era5", "cache_key": key, "rows": 2, "columns": 3, "layers": 32,
			"cloud_texels": texels}


## Runs frames for up to `seconds`, waiting for each upload; false on a jump or a change of a weighted slot.
func _drive(camera: Vector3, blend: float, seconds: float) -> bool:
	var ok := true
	var frames := int(seconds / DT)
	for i in range(frames):
		var before: Dictionary = _layer.state()
		_layer.update(camera, blend, DT)
		var after: Dictionary = _layer.state()
		var w0 := float(before["w1"])
		var w1 := float(after["w1"])
		if absf(w1 - w0) > DT / CloudLayer.FADE_S + 1e-9 and not bool(before["tracking"]):
			print("  jump %.4f -> %.4f at frame %d" % [w0, w1, i])
			ok = false
		for s in range(2):
			var weight := w0 if s == 1 else 1.0 - w0
			if before["slots"][s] != after["slots"][s] and weight > 1e-9:
				print("  slot %d changed with weight %.4f at frame %d" % [s, weight, i])
				ok = false
		if after["uploading"]:
			OS.delay_msec(1)
	return ok


func _state_is(slots: Array, w1: float) -> bool:
	var s: Dictionary = _layer.state()
	return s["slots"] == slots and (w1 < 0.0 or is_equal_approx(float(s["w1"]), w1)) and bool(s["tracking"])


## The pair in either slot order with the visible blend (weight of the second snapshot).
func _state_is_pair(pair: Array, blend: float) -> bool:
	var s: Dictionary = _layer.state()
	if s["slots"] == pair:
		return is_equal_approx(float(s["w1"]), blend) and bool(s["tracking"])
	if s["slots"] == [pair[1], pair[0]]:
		return is_equal_approx(1.0 - float(s["w1"]), blend) and bool(s["tracking"])
	return false


func _check(ok: bool, label: String) -> void:
	if not ok:
		_failures += 1
	print("  %s  %s" % ["ok  " if ok else "FAIL", label])
