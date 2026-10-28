# CloudLayer's cross-fade (M8 weather and clouds, Task 8; controller ruling: no pop on a pair or source change, only two
# field textures resident). Headless: godot --headless --path game --script res://tests/cloud_layer_test.gd
# Small fake snapshots (2 x 3 cells x 32 layers) stand in for the service's dictionaries. Checked every frame:
# - the weight of slot 1 moves by at most dt / FADE_S (no jump of the visible mix);
# - a slot's content changes only while that slot has no weight;
# - the pair ends in its real blend and then tracks the service's blend; weather off fades out the same way.
# The view cache's frame loop (cloud view cache Task 4): a view jump and a clock jump restart the cache; a Sun step of
# 1 deg starts cross-fades, none completing in the frame it starts; the march covers the view (the batches' covering
# flag) from a restart and a layout re-creation on until the view has level 0 and its first fills have faded in.
extends SceneTree

const DT := 1.0 / 60.0
## The view cache case: a small cube (8 x 8 tiles of 64 per face) and the frames it may take to converge the view.
const VIEW_CACHE_FACE := 512
const VIEW_CACHE_MAX_FRAMES := 600

var _failures := 0
var _layer: CloudLayer


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	_test_presets()
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
	_check(ShaderGlobals.value(&"cloud_base_before") == _layer._empty_base
			and _layer._empty_base.get_format() == Image.FORMAT_RGF,
			"an empty slot binds the zero RGF base texture (no cloud base)")

	_test_view_cache(camera)
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


## The view cache's frame loop (cloud view cache Task 4; headless: this test takes the batches in place of the render
## thread and hands the layer a view): the view's tiles reach the full level; a view jump and a clock jump restart the
## cache (every tile back to nothing or the first level, the restart counted); a Sun step of 1 deg starts cross-fades
## (refreshes at the same level fade from their slot) and none completes in the frame it starts.
func _test_view_cache(camera: Vector3) -> void:
	var snapshot := _fake("view_cache")
	_layer.set_target(snapshot, snapshot)
	_drive(camera, 0.0, 4.0)
	var sun := Vector3(0.3, 0.8, 0.5).normalized()
	ShaderGlobals.set_value(&"atmosphere_sun_direction", sun)
	var forward := Vector3(0.0, 0.0, -1.0)
	var dirs: Array[Vector3] = []
	for ix in range(-2, 3):
		for iy in range(-2, 3):
			dirs.append(Vector3(0.15 * ix, 0.15 * iy, -1.0).normalized())
	_layer.effect.set_view_cache_view({"dirs": dirs, "sample_rad": 0.1, "face_texels": VIEW_CACHE_FACE})
	var planner := _layer.view_cache_planner
	var converged := _converge_view(camera, forward)
	_check(converged, "view cache: the view's tile reaches the full level (%d frames)" % _frames_run)
	var settle := 0
	while planner.last_covering and settle < VIEW_CACHE_MAX_FRAMES:
		_frame(camera)
		settle += 1
	_check(not planner.last_covering, "view cache: the view at level 0 and faded in, the march no longer covers (%d "
			% settle + "frames after the view's tile was full)")
	var resets := planner.resets
	_layer.notify_view_jump()
	var batch := _frame(camera)
	_check(planner.resets == resets + 1 and _highest_shown(planner) <= 0,
			"view cache: a view jump restarts it (restarts %d -> %d, highest level shown %d)"
			% [resets, planner.resets, _highest_shown(planner)])
	var covered := _covered_frames(camera, bool(batch.get("covering", false)), dirs)
	_check(covered.x > 0 and covered.y > 0.0,
			"view cache: after the view jump the march covers until level 0 is in view and faded in (%d frames)"
			% covered.x)
	_converge_view(camera, forward)
	resets = planner.resets
	_layer.notify_clock_jump()
	_frame(camera)
	_check(planner.resets == resets + 1 and _highest_shown(planner) <= 0,
			"view cache: a clock jump restarts it (restarts %d -> %d, highest level shown %d)"
			% [resets, planner.resets, _highest_shown(planner)])
	_converge_view(camera, forward)
	# A jump arriving while the worker runs a restart is not lost: it restarts at the next plan.
	resets = planner.resets
	_layer.notify_view_jump()
	_layer.update(camera, 0.0, DT)  # starts the run that restarts
	_layer.notify_view_jump()  # the main thread, while that run goes on
	planner.wait()
	_frame(camera)  # the next plan starts the second restart
	_frame(camera)
	_check(planner.resets == resets + 2,
			"view cache: a jump requested while the worker restarts survives to the next plan (restarts %d -> %d)"
			% [resets, planner.resets])
	_converge_view(camera, forward)
	# A layout re-creation (the eyes' density 1.5 x: a new face size, built on a worker): a restart the march covers.
	resets = planner.resets
	_layer.effect.set_view_cache_view({"dirs": dirs, "sample_rad": 0.1, "face_texels": VIEW_CACHE_FACE * 3 / 2})
	var first_covering := false
	for i in range(VIEW_CACHE_MAX_FRAMES):
		_frame(camera)
		if planner.resets != resets:
			first_covering = bool(_frame(camera).get("covering", false))  # the restarted run's batch
			break
	_check(planner.resets == resets + 1 and first_covering and planner.layout.face_texels > VIEW_CACHE_FACE,
			"view cache: a layout re-creation (face %d) restarts it with the march covering"
			% planner.layout.face_texels)
	_layer.effect.set_view_cache_view({"dirs": dirs, "sample_rad": 0.1, "face_texels": VIEW_CACHE_FACE})
	_converge_view(camera, forward)
	_converge_view(camera, forward)
	ShaderGlobals.set_value(&"atmosphere_sun_direction", sun.rotated(Vector3(1.0, 0.0, 0.0), deg_to_rad(1.0)))
	var started := 0
	var refreshes := 0
	var completed_at_once := 0
	for i in range(int(4.0 / DT)):
		_frame(camera)
		for tile in planner.last_fades:
			started += 1
			refreshes += 1 if planner.fade_source(tile) == 4 else 0
			completed_at_once += 1 if planner.fade_weight(tile, planner.now_s) >= 1.0 else 0
	_check(refreshes > 0 and completed_at_once == 0, "view cache: a 1 deg Sun step starts cross-fades (%d, %d of them "
			% [started, refreshes] + "refreshes from a fade slot), none complete in the frame they start (%d)"
			% completed_at_once)
	ShaderGlobals.set_value(&"atmosphere_sun_direction", sun)


var _frames_run := 0


## Frames (the batches taken) until the tile ahead shows the full level; false after VIEW_CACHE_MAX_FRAMES.
func _converge_view(camera: Vector3, forward: Vector3) -> bool:
	var planner := _layer.view_cache_planner
	_frames_run = 0
	while _frames_run < VIEW_CACHE_MAX_FRAMES:
		_frame(camera)
		_frames_run += 1
		if planner.layout != null and planner.shown_level(planner.layout.tile_of(forward)) \
				== CloudViewCacheSchedule.MAX_LEVEL:
			return true
	return false


## Frames from a restart (whose batch's covering flag is given) until a batch no longer covers, and whether then
## every sample ray's tile shows a level and no first fill is still fading in there: (frames or -1 when the restart's
## batch did not cover or the covering never ended, 1 / 0).
func _covered_frames(camera: Vector3, first: bool, dirs: Array[Vector3]) -> Vector2:
	var planner := _layer.view_cache_planner
	if not first:
		return Vector2(-1.0, 0.0)
	for i in range(VIEW_CACHE_MAX_FRAMES):
		var batch := _frame(camera)
		if not bool(batch.get("covering", true)):
			var shown := true
			for d in dirs:
				var tile := planner.layout.tile_of(d)
				shown = shown and planner.shown_level(tile) >= 0 \
						and (planner.fade_source(tile) != 0 or planner.fade_weight(tile, planner.now_s) >= 1.0)
			return Vector2(i + 1, 1.0 if shown else 0.0)
	return Vector2(-1.0, 0.0)


## One frame of the view cache's loop: the layer's update (the planner hands out the last run's batch and starts the
## next), the planner's run waited for (its state is read next), the batch taken as the render thread would.
func _frame(camera: Vector3) -> Dictionary:
	_layer.update(camera, 0.0, DT)
	_layer.view_cache_planner.wait()
	return _layer.effect.take_view_cache_batch()


static func _highest_shown(planner: CloudViewCachePlanner) -> int:
	var highest := -1
	for tile in range(planner.layout.tile_count()):
		highest = maxi(highest, planner.shown_level(tile))
	return highest


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


## VR flicker (user 2026-10-26): the VR history keeps longer than the desktop's and is clipped statistically (variance
## clip, clip_gamma > 0) instead of to the noisy neighbourhood's min / max; the desktop keeps its min / max (0).
func _test_presets() -> void:
	var vr: Dictionary = CloudLayer.PRESETS[CloudLayer.Quality.VR]
	var desktop: Dictionary = CloudLayer.PRESETS[CloudLayer.Quality.DESKTOP]
	_check(float(vr.get("clip_gamma", 0.0)) > 0.0 and float(desktop.get("clip_gamma", -1.0)) == 0.0,
			"presets: VR variance clip, desktop min / max")
	_check(float(vr["temporal_weight"]) < float(desktop["temporal_weight"]), "presets: the VR history keeps longer")


func _check(ok: bool, label: String) -> void:
	if not ok:
		_failures += 1
	print("  %s  %s" % ["ok  " if ok else "FAIL", label])
