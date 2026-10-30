# Renders the main scene at a fixed instant and camera, saves a PNG and quits (needs a GPU, no --headless).
# godot --path game --script res://tests/render_check.gd -- --utc=2024-04-08T18:42:00 --focus=earth
# Vehicle: --vehicle=starship_block3 --site=starbase_olp1 [--fly=40] (ignite, liftoff at 3 s, fly N s)
#          [--autopilot] (ascent autopilot instead of manual) [--circularize] (plan node afterwards)
#          [--watch_from_site[=2000]] (afterwards: seen from the ground 2000 m east of the site, aimed at the vehicle)
# Ground view: --lat= --lon= [--height=] [--body=moon] (selenographic, MOON_ME; --height defaults to the core's
#          surface there, the Earth's or the Moon's, --eye_height=2 above it) with --azimuth / --elevation / --fov
# Sea: --sea_lat=32.5 --sea_lon=-61 [--eye_height=10] ground view at sea, the eye above the geoid (with --azimuth etc.)
# Catch tower: [--tower_arms_deg=0] draws the arms at a fixed opening (0 = closed).
# Drone ship: a mission with a deck (--mission=falcon9_asds_sea --at_event=landed,20 --view=1) prints the viewed CG above
#          the drawn deck against its height above the feet.
# Webcast overlay: --webcast [--mission=starship_leo] (start the mission at --utc) [--t_plus=75] (advance to T+75 s)
#          or [--at_event=entry_interface,<offset s>[,<n>]] (to the n-th event of that kind in the mission timeline,
#          recorded first, else predicted; flies until it appears)
#          [--wait_on_pad] (a launch window's wait on the pad: T-) [--view=1] (Tab presses: switch the viewed object)
#          [--legacy_hud] (F1: the old HUD too)
# Weather: off in a render check unless --weather[=900 / =online] (online: waits for the downloads, at most 900 s or
#          that many), --weather=analog (the shipped analog of --utc), --weather=<file.wthr> (a cached snapshot) or
#          --weather=slab:<bottom m>,<top m>,<liquid kg/m^3>,<fraction>[,ice] (ice: the slab drawn as ice, cirrus; a
#          debug view); any other value exits with an error;
#          --cloud_quality=desktop|vr (the clouds' quality preset; default: the game's; --cloud_divisor=<n>: its march
#          resolution divisor instead, fractional allowed: 1.5),
#          the settle waits until the baked volume's first set is baked and shown;
#          --climb=<eye height m>,... [--climb_lon=<deg per step>] (a ground view: after the first settle
#          the eye moves to each height in turn, each step waiting until the baked volume has re-baked about the new eye;
#          a step that does not within CLIMB_MAX_FRAMES fails the run, exit code 1), --cloud_detail_levels=0..3 (the
#          detail levels), --cloud_scattering=table|closed (the multiple-scattering model: Monte Carlo table, default;
#          closed form).
# Eye adaptation: on as in the game (adapted to the final view); --fixed_exposure switches it off.
# Image identity: --deterministic[=64] once settled saves after exactly that many more frames (with --fixed_exposure,
#          for before / after comparisons; the HUD's frame-rate line hidden). --repeat_identical[=2] once settled compares that many consecutive frames
#          of the unchanged scene: any differing pixel fails (exit code 1; the HUD's frame-rate line hidden).
#          --no_air_shadow: the clouds' shadows on the air off (the flag CloudLayer.air_shadow).
# Refraction: on as in the game; --no_refraction draws Sun, Moon and stars at their geometric directions.
#          --look_at_sun aims a ground view at the (true) Sun; with --away_from_sun the opposite azimuth.
# Sun through a deck: --sun_through_deck=<vertical tau> (with --lat / --lon and --utc): a uniform slab 1-2 km of that
#          optical depth (liquid = tau / 150 000 kg/m^3: beta = 150 LWC) and the view aimed at the Sun (static clouds Task 9).
# GPU time: --gpu_time[=120] once settled, the root viewport's measured GPU render time over that many frames (median,
#          p99); --render_scale=<f> renders the 3D view at f x the window per axis (1.65: the VR pixel proxy);
#          --gpu_passes: also per GPU timestamp interval (the cloud pass's kernels, the bake, shadow maps), the baked
#          volume settled first.
# Shadow map dump: --shadow_dump=<file> writes the cloud shadow maps (near, far; raw RGBA32F) once settled and prints
#          their frame (baked shadow step; with STARSIM_BAKED_DEFINES=CLOUD_SHADOW_BAKED_DIAG the baked fill's check
#          values instead of the maps, cloud_baked.gdshaderinc cloud_shadow_fill_diag).
# Centre image dump: --centre_dump=<file> writes the cloud march's centre image once settled (raw RGBA16F, premultiplied
#          colour + T, after two int32 width / height) - offline cover statistics (baked prototype step 3).
# Stereo harness (world volume plan Task 1): --stereo=<json> renders through the multiview path with a stand-in headset
#          (StereoMockXr: the recorded per-eye projections, eye transforms and render target of the JSON that PerfLog
#          writes at a VR start, game/tests/data/pimax_dream_air_projections.json) with the VR cloud preset (unless
#          --cloud_quality; run without --xr-mode off, which switches the multiview shader variants off), the eye
#          images written as <out>_eye0.png / <out>_eye1.png; the log shows "stereo view_count 2". --cloud_view=<id>
#          (CloudViews: the gate views V1 .. V7, e.g. V3_era5) prepends that view's arguments; an argument given on the
#          command line wins.
#     [--yaw=0.6] [--pitch=-0.3] [--distance=2.2e7] [--azimuth=90] [--elevation=5] [--fov=50] --out=C:/tmp/x.png
extends SceneTree

const FRAMES_TO_SETTLE := 6
## --stereo: frames to wait for the eye images' read-back on the render thread.
const STEREO_CAPTURE_FRAMES := 120
## --sun_through_deck: the slab's heights (m) and the extinction per liquid water content of the renderer's droplets
## (beta = 3 LWC / (2 rho_w r_e) = 150 LWC [1/m], r_e 10 um; as cloud_check.gd THIN_LIQUID_KG_M3).
const SUN_DECK_BOTTOM_M := 1000
const SUN_DECK_TOP_M := 2000
const SUN_DECK_EXTINCTION_PER_LWC := 150.0
## Upper bound on the wait for the streamed terrain (frames).
const TERRAIN_MAX_FRAMES := 6000
## The flight session's fixed tick (200 Hz, sim::FlightSession): --t_plus aims a tick late (the grid need not hit T-0).
const SIM_TICK_S := 0.005
## --at_event: flight chunk and limit while the event is not yet in the mission timeline (s).
const EVENT_SEARCH_STEP_S := 3600.0
const EVENT_SEARCH_MAX_S := 20.0 * 86400.0
## --climb: frames a step may take to re-bake and settle.
const CLIMB_MAX_FRAMES := 1500
var _args := {}
var _climb_next := 0  ## --climb: the next step (0: none moved yet)
var _climb_move_frame := 0  ## --climb: the frame of the last move
var _climb_bakes := 0  ## --climb: CloudBaked.bakes_done at the last move
var _climb_failed := false
var _climb_finished := false  ## --climb: every step done (the image may take a few more frames)
var _stereo: StereoMockXr  ## --stereo: the stand-in headset
var _stereo_capture_frames := 0
var _main: Node
var _frames := 0
var _start_s := 0.0
var _terrain_idle := 0  ## consecutive frames without terrain loading
var _found_event_tdb_s := NAN  ## --at_event: the event's instant found by flying (_fly_until_event)
var _deterministic_left := -1  ## --deterministic: frames still to draw after the reset (-1: not yet reset)
var _cache_settled := 0  ## --repeat_identical: consecutive frames with the light cache settled
var _repeat_images: Array[Image] = []  ## --repeat_identical: the consecutive frames drawn so far
var _repeat_failed := false  ## --repeat_identical: the frames differ (exit code 1)
var _gpu_ms: Array[float] = []  ## --gpu_time: the measured frames' GPU render times
var _gpu_passes := {}  ## --gpu_passes: timestamp name -> Array of the measured frames' GPU ms
var _sweep_left: Array[float] = []  ## --divisor_sweep: the divisors still to measure
var _sweep_wait := -1  ## --divisor_sweep: frames left to settle at the current divisor (-1: not started)

func _initialize() -> void:
	var arguments := _with_cloud_view(OS.get_cmdline_user_args())
	if arguments.is_empty():
		return
	for arg in arguments:
		var parts := arg.trim_prefix("--").split("=", true, 1)
		_args[parts[0]] = parts[1] if parts.size() > 1 else ""
	if _args.has("sun_through_deck"):
		_args["weather"] = "slab:%d,%d,%s,1" % [SUN_DECK_BOTTOM_M, SUN_DECK_TOP_M,
				float(_args["sun_through_deck"]) / SUN_DECK_EXTINCTION_PER_LWC / (SUN_DECK_TOP_M - SUN_DECK_BOTTOM_M)]
		_args["look_at_sun"] = ""
	if _args.has("stereo") and not _start_stereo(String(_args["stereo"])):
		return
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)
	_start_s = Time.get_ticks_msec() / 1000.0
	if _args.has("render_scale"):
		root.scaling_3d_mode = Viewport.SCALING_3D_MODE_BILINEAR
		root.scaling_3d_scale = float(_args["render_scale"])
	if _args.has("gpu_time"):
		RenderingServer.viewport_set_measure_render_time(root.get_viewport_rid(), true)
	CloudEffect.capture_timestamps = _args.has("gpu_passes")


## The user arguments with --cloud_view=<id>'s arguments first (the command line's own win); [] and exit code 1 for an
## unknown view id.
func _with_cloud_view(user_args: PackedStringArray) -> PackedStringArray:
	var result := PackedStringArray()
	for arg in user_args:
		if arg.begins_with("--cloud_view="):
			var id := arg.trim_prefix("--cloud_view=")
			var view := CloudViews.args(id)
			if view.is_empty():
				_refuse("render_check: unknown --cloud_view=%s (%s)" % [id, ", ".join(CloudViews.ids())])
				return PackedStringArray()
			result.append_array(view)
	result.append_array(user_args)
	return result


## --stereo=<json>: the stand-in headset (StereoMockXr) on the XR server, before the scene tree starts (the XR manager then
## enters VR mode on it). False (exit code 1) when the JSON cannot be used.
func _start_stereo(path: String) -> bool:
	var mock := StereoMockXr.new()
	if not mock.load(path):
		_refuse("render_check: --stereo=%s cannot be loaded" % path)
		return false
	XRServer.add_interface(mock)
	if not mock.initialize():
		_refuse("render_check: the stand-in headset does not initialise")
		return false
	XRServer.primary_interface = mock
	# (XrManager._ready finds the stand-in as the primary interface and enters VR mode on it)
	_stereo = mock
	print("render_check: stereo harness, %s: %d views, render target %s per eye" % [path, mock._get_view_count(),
			mock.render_target_px])
	return true


func _finalize() -> void:
	if _stereo != null:  # the XR server must not keep the stand-in while the engine shuts down
		XRServer.primary_interface = null
		_stereo.uninitialize()
		XRServer.remove_interface(_stereo)


## The progress images (a --stereo run writes only the final eye images: the viewport texture is the multiview array).
func _save_viewport_png(path: String) -> void:
	if _stereo == null:
		root.get_viewport().get_texture().get_image().save_png(path)


## --stereo: the final frame's eye images <out>_eye<n>.png (read back on the render thread); true once written.
func _save_stereo_images(world: World, out: String) -> bool:
	if _stereo.captured.is_empty() and _stereo_capture_frames < STEREO_CAPTURE_FRAMES and _stereo.capture_error == "":
		_stereo.capture_requested = true
		_stereo_capture_frames += 1
		return false
	if _stereo.captured.is_empty():
		_refuse("render_check: no eye image captured (%s)" % _stereo.capture_error)
		return false
	DirAccess.make_dir_recursive_absolute(out.get_base_dir())
	print("render_check: stereo view_count %d (cloud pass %s), render target %s" % [_stereo.captured.size(),
			world.clouds.effect.stats.get("views", "?"), _stereo.render_target_px])
	print("render_check: eye images " + _stereo.capture_info)
	for view in range(_stereo.captured.size()):
		var path := out.get_basename() + "_eye%d." % view + out.get_extension()
		var error := _stereo.captured[view].save_png(path)
		print("render_check: saved %s (error %d)" % [path, error])
	return true


func _process(_delta: float) -> bool:
	var world: World = _main.get_node("World")
	world.clock.warp = 0.0
	# Weather downloads only on request (--weather): a render check must not fetch gigabytes of cloud data.
	world.weather.enabled = _weather_online()
	if _frames == 0:
		var quality: String = _args.get("cloud_quality", "vr" if _stereo != null else "")  # stereo: the VR preset
		if quality != "":
			world.clouds.quality = CloudLayer.Quality.VR if quality == "vr" else CloudLayer.Quality.DESKTOP
		_show_weather_file(world)
		if _args.has("cloud_divisor"):  # the march's resolution divisor (the preset's otherwise; tuning runs)
			world.clouds.effect.set_quality(float(_args["cloud_divisor"]))
		if _args.has("cloud_detail_levels"):  # the detail levels used (the preset's otherwise)
			ShaderGlobals.set_value(&"cloud_detail_levels", int(_args["cloud_detail_levels"]))
		if _args.has("no_air_shadow"):  # the fidelity flag (CloudLayer.air_shadow)
			world.clouds.air_shadow = false
		if _args.has("cloud_scattering"):  # the fidelity flag (CloudLayer.ScatteringModel)
			world.clouds.scattering_model = CloudLayer.ScatteringModel.CLOSED_FORM \
					if _args["cloud_scattering"] == "closed" else CloudLayer.ScatteringModel.TABLE
		if _args.has("repeat_identical") or _args.has("deterministic"):  # the HUD's frame-rate line changes per frame
			TimePanel.show_frame_rate = false
		world.clock.set_time(world.core.utc_to_tdb_s(_args.get("utc", "2024-04-08T18:42:00")))
		world.atmosphere.eye_adaptation_enabled = not _args.has("fixed_exposure")
		world.refraction.enabled = not _args.has("no_refraction")
		world.set_focus(_args.get("focus", World.FOCUS_EARTH))
		if _args.has("lat"):
			_view_from_ground(world)
		if _args.has("sea_lat"):
			_view_at_sea(world)
		var cam := world.camera
		if _args.has("yaw"): cam.yaw = float(_args["yaw"])
		if _args.has("pitch"): cam.pitch = float(_args["pitch"])
		if _args.has("distance"): cam.distance_m = float(_args["distance"])
		if _args.has("azimuth"): cam.azimuth = deg_to_rad(float(_args["azimuth"]))
		if _args.has("elevation"): cam.elevation = deg_to_rad(float(_args["elevation"]))
		if _args.has("fov"): cam.fov = float(_args["fov"])
		if _args.has("look_at_sun"):
			_aim_at_sun(world)
		if _args.has("look_at_moon"):
			_aim_at_moon(world)
		if _args.has("view_from"):
			_view_from(world, _args["view_from"])
		if _args.has("vehicle"):
			_fly_vehicle(world)
		if _args.has("webcast"):
			_fly_mission(world)
			cam = world.camera  # the mission's focus switch resets the camera: the requested view again
			if _args.has("yaw"): cam.yaw = float(_args["yaw"])
			if _args.has("pitch"): cam.pitch = float(_args["pitch"])
			if _args.has("distance"): cam.distance_m = float(_args["distance"])
		if _args.has("tower_arms_deg"):
			for view in world.towers:
				view.preview_opening_rad = deg_to_rad(float(_args["tower_arms_deg"]))
				view.update_state({})
		# The view is placed: the eye is adapted to it. _view_from and _aim_at_moon run update_world while placing the
		# camera, which used up set_focus's snap at the pre-aim pose (--view_from=moon: 22 000 km on the night side, gain
		# 25, then ~0.24 s of light adaptation over the settle frames: ~7.5 on a sunlit view). The world's own update this
		# frame (after this script's _process) takes the snap at the final pose.
		world.atmosphere.snap_adaptation()
		if _stereo != null:  # the VR panel (menu, overlay) is not part of the cloud views
			for child in _main.get_node("/root/XrManager").origin.get_children():
				if child is VrOverlayPanel:
					child.visible = false
	_frames += 1
	# The streamed terrain loads over many frames: wait until nothing is loading (or give up after TERRAIN_MAX_FRAMES).
	var loading: int = world.earth_terrain.stats["nodes_loading"] + world.moon_terrain.stats["nodes_loading"]
	_terrain_idle = _terrain_idle + 1 if loading == 0 and not world.atmosphere.loading else 0  # also the sky's tables
	# --weather[=max seconds]: also wait for the weather service (downloads, builds), at most that long (default 900 s).
	var weather_waiting: bool = _weather_online() and not world.weather.settled() \
			and Time.get_ticks_msec() / 1000.0 - _start_s < _weather_wait_s()
	# The clouds' noise (generated on a worker) and a running field upload / cross-fade (online weather: 2 s per unit).
	var clouds_waiting: bool = (_args.has("weather") and _args["weather"] != "off") \
			and (not world.clouds.noise_ready or world.clouds.state()["uploading"] or not world.clouds.state()["tracking"]) \
			and _frames < TERRAIN_MAX_FRAMES
	# The baked volume (clouds shown): until its first set is baked and shown, no bake running.
	var bake_waiting := _bake_waiting(world)
	var settled := _frames >= FRAMES_TO_SETTLE and not weather_waiting and not clouds_waiting and not bake_waiting \
			and (_terrain_idle >= FRAMES_TO_SETTLE or _frames >= TERRAIN_MAX_FRAMES)
	if _args.has("climb") and (settled or _climb_next > 0) and not _climb_done(world, settled):
		return false
	if settled:
		if _args.has("deterministic"):
			if _deterministic_left < 0:  # the same number of frames from here in every run
				_deterministic_left = int(_args["deterministic"]) if _args["deterministic"].is_valid_int() else 64
			if _deterministic_left > 0:
				_deterministic_left -= 1
				return false
		if _args.has("repeat_identical") and not _repeat_frames(world):
			return false
		if _args.has("gpu_time") and not _gpu_frames(world):
			return false
		print("render_check: ", world.weather.status_text())
		print("render_check: clouds %s, field textures %.1f MB, last upload %.1f ms" % [world.clouds.state(),
				world.clouds.field_texture_bytes / 1.0e6, world.clouds.last_upload_ms])
		print("render_check: terrain after %d frames: Earth %s, Moon %s" % [_frames, world.earth_terrain.stats,
				world.moon_terrain.stats])
		if _args.has("webcast"):
			var arc := (_main.get("overlay_host") as OverlayHost).overlay.timeline_arc
			print("render_check: timeline scale ", arc.time_scale, " now ", arc.now_tdb_s, " events ", arc.events.size())
		if world.flight.sim.is_active() and bool(world.flight.sim.lunar_site().get("present", false)):
			_print_lunar_placement(world)
		if world.drone_ship != null:
			_print_deck_placement(world)
		# The Sun's elevation at the eye (geodetic up on a ground view, else the geocentric vertical), for labels.
		var eye := world.camera.position_gcrf_m
		var up := world.camera.site_up_gcrf if world.camera.mode == OrbitCamera.Mode.GROUND else eye.normalized()
		var to_sun := (world.sun_apparent_from_earth_m - eye).normalized()
		print("render_check: Sun elevation at the eye %.2f deg (UTC %s), exposure gain %.3f, vehicle Sun light %.3f" % [
				rad_to_deg(asin(clampf(to_sun.dot(up), -1.0, 1.0))), world.core.tdb_s_to_utc(world.clock.tdb_s),
				world.atmosphere.exposure_gain, world.sun_light.light_energy])
		var a := world.atmosphere
		print("render_check: eye adapted to %s cd/m^2: Sun on a plate %s (visible %.3f), Earth in view %s, Moon in view %s, earthshine %s lux" % [
				a.adaptation_cd_m2, a.last_adaptation_down_cd_m2, a.last_adaptation_visible_fraction,
				a.last_adaptation_up_cd_m2, a.last_adaptation_moon_cd_m2, a.last_earthshine_lux])
		print("render_check: night vision: twilight glow %s cd/m^2, scotopic %s cd/m^2, rod scale %s, moonlight %s of the Sun (%s), vehicle Moon light %s, brightness %.4f, chroma %.4f" % [
				a.last_adaptation_glow_cd_m2, a.scotopic_cd_m2, a.rod_scale, a.moonlight_relative_value, "on" if a.moonlight_active else "off",
				world.moon_light.light_energy if world.moon_light.visible else 0.0, a.brightness, a.chroma])
		if _args.has("centre_dump"):
			_dump_centre(world, String(_args["centre_dump"]))
		if _args.has("shadow_dump"):
			_dump_shadow_maps(world, String(_args["shadow_dump"]))
		# Default: <repo>/screenshots/ (git-ignored; every test image goes there, user 2026-10-27).
		var out: String = _args.get("out", ProjectSettings.globalize_path("res://").path_join(
				"../screenshots/render_check.png").simplify_path())
		DirAccess.make_dir_recursive_absolute(out.get_base_dir())
		if _stereo != null:
			if not _save_stereo_images(world, out):
				return false
		else:
			var error := root.get_viewport().get_texture().get_image().save_png(out)
			print("render_check: saved %s (error %d)" % [out, error])
		if _repeat_failed:
			quit(1)
		return true  # quit
	return false


## --gpu_time[=120]: collects the root viewport's GPU render time of that many frames; true when done (printed).
## --gpu_passes (with --gpu_time): the frames are collected once the baked volume is settled (steady state), and per
## frame the GPU time between the RenderingDevice's captured timestamps of the previous frame, per name (the interval
## a timestamp starts; CloudEffect.capture_timestamps: cloud_pass_<kernel>, cloud_baked_bake, shadow_pass; their
## cloud_pass_* summed as cloud_pass), printed as median and p99 per name.
## --divisor_sweep=3,2,1.5,1 (with --gpu_time): the measurement repeated per cloud march
## divisor (CloudLayer.divisor_override), each after FRAMES_TO_SETTLE frames at it; the image is the last divisor's.
func _gpu_frames(world: World) -> bool:
	var wanted: int = int(_args["gpu_time"]) if _args["gpu_time"].is_valid_int() else 120
	if _args.has("divisor_sweep") and _sweep_wait < 0:
		for d in _args["divisor_sweep"].split(",", false):
			_sweep_left.append(float(d))
		_next_sweep_divisor(world)
	if _sweep_wait > 0:
		_sweep_wait -= 1
		return false
	if _args.has("gpu_passes"):
		var baked_ok := world.clouds.effect.baked != null and (world.clouds.effect.baked.settled()
				or (CloudBaked.rebake_frames > 0 and world.clouds.effect.baked.ready))  # forced re-bakes: measured
		_cache_settled = _cache_settled + 1 if baked_ok else 0
		if _cache_settled < FRAMES_TO_SETTLE and _frames < TERRAIN_MAX_FRAMES:
			return false
		_record_gpu_passes()
	_gpu_ms.append(RenderingServer.viewport_get_measured_render_time_gpu(root.get_viewport_rid()))
	if _gpu_ms.size() < wanted:
		return false
	var sorted := _gpu_ms.duplicate()
	sorted.sort()
	print("render_check: GPU render time over %d frames: median %.3f ms, p99 %.3f ms (render scale %.2f, %s)" % [
			wanted, sorted[wanted / 2], sorted[mini(wanted - 1, int(0.99 * wanted))], root.scaling_3d_scale,
			root.size])
	for name: String in _gpu_passes:
		var ms: Array = _gpu_passes[name]
		while ms.size() < wanted:  # frames without that interval
			ms.append(0.0)
		ms.sort()
		print("render_check: gpu pass %s median %.3f ms, p99 %.3f ms" % [name, ms[ms.size() / 2],
				ms[mini(ms.size() - 1, int(0.99 * ms.size()))]])
	if _args.has("divisor_sweep"):
		print("render_check: divisor_sweep divisor %s baked_shown %s (the lines above)" % [
				world.clouds.divisor(), world.clouds.effect.baked_shown])
		if not _sweep_left.is_empty():
			_next_sweep_divisor(world)
			return false
	return true


## --divisor_sweep: the next divisor, its settle frames, the measurement restarted.
func _next_sweep_divisor(world: World) -> void:
	world.clouds.divisor_override = _sweep_left.pop_front()
	_sweep_wait = FRAMES_TO_SETTLE
	_gpu_ms.clear()
	_gpu_passes.clear()


## True while the baked volume (the clouds shown) has no set shown yet or a bake / its
## cross-fade still runs (CloudBaked.settled; TERRAIN_MAX_FRAMES at most): the image shows the settled volume.
## --climb: the climb's state machine on a settled frame (or a step's wait): the last move's check (re-baked, settled,
## the shown set's anchor at the eye), then the next move. True once every step is done; a step that does not settle
## within CLIMB_MAX_FRAMES quits with exit code 1.
func _climb_done(world: World, settled: bool) -> bool:
	if _climb_finished:
		return true
	if _climb_failed:
		return false
	var heights: PackedFloat64Array = String(_args["climb"]).split_floats(",", false)
	var baked := world.clouds.effect.baked
	if _climb_next > 0:
		var rebaked := baked != null and baked.bakes_done > _climb_bakes
		if not (settled and rebaked):
			if _frames - _climb_move_frame > CLIMB_MAX_FRAMES:
				print("render_check: climb FAIL: the step to %.0f m did not re-bake within %d frames (bakes %d -> %d)"
						% [heights[_climb_next - 1], CLIMB_MAX_FRAMES, _climb_bakes,
						baked.bakes_done if baked != null else -1])
				_climb_failed = true
				quit(1)
			return false
		var camera: Vector3 = ShaderGlobals.value(&"atmosphere_camera_hi") + ShaderGlobals.value(&"atmosphere_camera_lo")
		print("render_check: climb step %d at %.0f m: %d frames, bakes %d -> %d, shown anchor %.0f m from the eye" % [
				_climb_next, heights[_climb_next - 1], _frames - _climb_move_frame, _climb_bakes, baked.bakes_done,
				baked.shown_anchor().distance_to(camera)])
	if _climb_next < heights.size():
		var site: Dictionary = world.ground_site.duplicate()
		site["eye_height_m"] = heights[_climb_next]
		site["longitude_deg"] = float(site["longitude_deg"]) + float(_args.get("climb_lon", "0"))
		world.set_ground_site(site)
		_climb_bakes = baked.bakes_done if baked != null else 0
		_climb_move_frame = _frames
		_climb_next += 1
		return false
	_climb_finished = true
	return true


func _bake_waiting(world: World) -> bool:
	var clouds := world.clouds
	if not bool(ShaderGlobals.value(&"cloud_enabled")) or _frames >= TERRAIN_MAX_FRAMES:
		return false
	var baked := clouds.effect.baked
	return baked == null or not baked.settled()


## --out with a suffix before its extension (the A / B images).
func _out_path(suffix: String) -> String:
	var out: String = _args.get("out", ProjectSettings.globalize_path("res://").path_join(
			"../screenshots/render_check.png").simplify_path())
	DirAccess.make_dir_recursive_absolute(out.get_base_dir())
	return out.get_basename() + suffix + "." + out.get_extension()


func _record_gpu_passes() -> void:
	var rd := RenderingServer.get_rendering_device()
	var frame := {}
	for i in range(1, rd.get_captured_timestamps_count()):
		var name := rd.get_captured_timestamp_name(i - 1)
		var ms := float(rd.get_captured_timestamp_gpu_time(i) - rd.get_captured_timestamp_gpu_time(i - 1)) / 1.0e6
		frame[name] = float(frame.get(name, 0.0)) + ms
		if name.begins_with("cloud_pass_"):
			frame["cloud_pass"] = float(frame.get("cloud_pass", 0.0)) + ms
	for name: String in frame:
		if not _gpu_passes.has(name):
			_gpu_passes[name] = []
		_gpu_passes[name].append(frame[name])


## --repeat_identical[=2]: once the baked volume is settled (no bake, no cross-fade), collects that many consecutive
## frames of the unchanged scene and compares them; true when all are collected (then the last one
## is saved). Frames that differ fail the check (exit code 1 after the save).
func _repeat_frames(world: World) -> bool:
	_cache_settled = _cache_settled + 1 if not _bake_waiting(world) else 0
	if _cache_settled < FRAMES_TO_SETTLE and _frames < TERRAIN_MAX_FRAMES:
		return false
	var wanted: int = maxi(int(_args["repeat_identical"]) if _args["repeat_identical"].is_valid_int() else 2, 2)
	_repeat_images.append(root.get_viewport().get_texture().get_image())
	if _repeat_images.size() < wanted:
		return false
	var first := _repeat_images[0]
	var worst := 0
	var box := Rect2i()  # the differing pixels' bounding box
	for k in range(1, _repeat_images.size()):
		var other := _repeat_images[k]
		var differing := 0
		for y in (0 if first.get_data() == other.get_data() else first.get_height()):
			for x in first.get_width():
				if first.get_pixel(x, y) != other.get_pixel(x, y):
					differing += 1
					box = Rect2i(x, y, 1, 1) if box.size == Vector2i.ZERO else box.expand(Vector2i(x, y))
		worst = maxi(worst, differing)
	print("render_check: repeat_identical %d frames: %s (most differing pixels %d of %d%s)" % [wanted,
			"identical" if worst == 0 else "DIFFER", worst, first.get_width() * first.get_height(),
			"" if worst == 0 else ", within %s" % box])
	if worst > 0:
		_repeat_failed = true
		if _args.has("out"):  # the first frame beside the saved last one (the difference is diagnosed from both)
			first.save_png(String(_args["out"]).get_basename() + "_first.png")
	return true


## --weather, --weather=online or --weather=<max seconds>: the weather service online (downloads); off / analog / a
## file / a slab: not.
func _weather_online() -> bool:
	if not _args.has("weather"):
		return false
	var value: String = _args["weather"]
	return value == "" or value == "online" or value.is_valid_float()


## --centre_dump=<file>: the centre view's march image (RGBA16F: premultiplied colour + T) as raw bytes after a header
## of two int32 (width, height) - offline cover statistics (cloud alpha = 1 - T per pixel).
func _dump_centre(world: World, path: String) -> void:
	var effect: CloudEffect = world.clouds.effect
	var data := effect.debug_centre_data("color")
	var low := effect.centre_low
	var file := FileAccess.open(path, FileAccess.WRITE)
	if file == null or data.size() < 8 * low.x * low.y:
		print("render_check: centre dump failed (%s)" % path)
		return
	file.store_32(low.x)
	file.store_32(low.y)
	file.store_buffer(data.slice(0, 8 * low.x * low.y))
	file.close()
	print("render_check: centre dump %dx%d -> %s" % [low.x, low.y, path])


## --shadow_dump=<file>: the cloud shadow maps (near, then far: RGBA32F each, CloudShadowEffect) as raw bytes after a
## header (int32 size, int32 baked fill 0 / 1), and their frame (anchors, axes, Sun, extents, the camera's up) printed.
func _dump_shadow_maps(world: World, path: String) -> void:
	var shadow: CloudShadowEffect = world.clouds.shadow_effect
	var near := shadow.debug_map_data(0)
	var far := shadow.debug_map_data(1)
	var size := int(round(sqrt(near.size() / 16.0)))
	var file := FileAccess.open(path, FileAccess.WRITE)
	if file == null or near.is_empty() or far.size() != near.size():
		print("render_check: shadow dump failed (%s)" % path)
		return
	file.store_32(size)
	file.store_32(1 if shadow.baked_fill else 0)
	file.store_buffer(near)
	file.store_buffer(far)
	file.close()
	var hi: Vector3 = ShaderGlobals.value(&"atmosphere_camera_hi")
	var lo: Vector3 = ShaderGlobals.value(&"atmosphere_camera_lo")
	var camera := hi + lo
	var up := (camera * Vector3(1.0, CloudLayer.POLAR_STRETCH, 1.0)).normalized()
	print("render_check: shadow dump %d (baked fill %s) -> %s" % [size, shadow.baked_fill, path])
	print("render_check: shadow frame %s" % JSON.stringify({"near_anchor": _v(ShaderGlobals.value(&"cloud_shadow_near_anchor")),
		"far_anchor": _v(ShaderGlobals.value(&"cloud_shadow_far_anchor")), "e1": _v(ShaderGlobals.value(&"cloud_shadow_e1")),
		"e2": _v(ShaderGlobals.value(&"cloud_shadow_e2")), "sun": _v(ShaderGlobals.value(&"cloud_shadow_sun")),
		"extent": [ShaderGlobals.value(&"cloud_shadow_extent").x, ShaderGlobals.value(&"cloud_shadow_extent").y],
		"up": _v(up), "camera": _v(camera)}))


static func _v(v: Vector3) -> Array:
	return [v.x, v.y, v.z]


## The online wait limit [s] of --weather (900 unless --weather=<seconds>).
func _weather_wait_s() -> float:
	var value: String = _args["weather"]
	return float(value) if value.is_valid_float() else 900.0


## --weather=<file.wthr> shows that snapshot; --weather=analog the shipped analog snapshot of --utc;
## --weather=slab:<bottom m>,<top m>,<liquid kg/m^3>,<cloud fraction> a synthetic uniform slab
## (StarsimCore.weather_write_slab_snapshot) - all pinned, without the service. Any other value is refused (exit 1).
func _show_weather_file(world: World) -> void:
	if not _args.has("weather") or _weather_online() or _args["weather"] == "off":
		return
	var value: String = _args["weather"]
	var file := ""
	if value == "analog":
		var unix := Time.get_unix_time_from_datetime_string(_args.get("utc", "2024-04-08T18:42:00"))
		file = world.weather.analog_file(unix / 3600.0 + WeatherService.HOURS_1900_TO_1970)
		if file == "":
			_refuse("render_check: no analog snapshot for --utc")
			return
	elif value.begins_with("slab:"):
		var p := value.trim_prefix("slab:").split(",")
		file = OS.get_temp_dir().path_join("starsim_render_check_slab_%d.wthr" % OS.get_process_id())
		if not world.core.weather_write_slab_snapshot(file, float(p[0]), float(p[1]), float(p[2]), float(p[3])):
			push_error("render_check: slab snapshot not written")
			return
	elif value.ends_with(".wthr"):
		file = value
	else:
		_refuse("render_check: unknown --weather=%s (online, <seconds>, off, analog, <file.wthr>, slab:...)" % value)
		return
	var snapshot: Dictionary = world.core.weather_load_snapshot(file)
	if not snapshot.get("ok", false):
		_refuse("render_check: " + str(snapshot.get("error", "")))
		return
	if value.begins_with("slab:") and value.trim_prefix("slab:").split(",").size() > 4 			and value.trim_prefix("slab:").split(",")[4] == "ice":
		# Debug (baked cirrus check): the slab's extinction marked as ice (ice share 1 everywhere) - the renderer's
		# cirrus look (shapes, bake taps); the extinction stays the given water's.
		var ice := PackedByteArray()
		ice.resize((snapshot["cloud_ice_texels"] as PackedByteArray).size())
		ice.fill(255)
		snapshot["cloud_ice_texels"] = ice
	world.clouds.wait_until_ready()
	world.clouds.show_now(snapshot, snapshot, 0.0)
	world.core.weather_set_active(file, file, 0.0)
	world.weather.pinned_label = WeatherService.snapshot_label(snapshot)
	print("render_check: weather from %s (%s %s)" % [file, snapshot["kind"], snapshot["cache_key"]])


## A bad option: the error, exit code 1 (no silent render without the asked-for weather).
func _refuse(message: String) -> void:
	push_error(message)
	print(message)
	quit(1)


# Orbit camera: look at the focus from the direction of the Sun or the Moon.
func _view_from(world: World, body: String) -> void:
	world.update_world()
	var direction := world.sun_apparent_from_earth_m if body == "sun" else world.moon_apparent_from_earth_m
	var u := Frames.to_render(direction).normalized()
	world.camera.pitch = -asin(u.y)
	world.camera.yaw = atan2(u.x, u.z)


# Launch a vehicle at the given instant, fly it with neutral controls (rate command holds attitude).
func _fly_vehicle(world: World) -> void:
	var site: Dictionary = world.sites[0]
	for candidate: Dictionary in world.sites:
		if candidate["id"] == _args.get("site", ""):
			site = candidate
	var t0 := world.clock.tdb_s
	world.flight.manual_control = true  # the script flies by its own commands, not the external view's autopilot
	world.launch_vehicle(_args["vehicle"], site)
	world.clock.warp = 0.0
	var sim := world.flight.sim
	sim.wait_idle(60.0)  # the launch runs on the simulation thread
	var fly := float(_args.get("fly", "0"))
	if fly > 0.0:
		if _args.has("autopilot"):
			sim.engage_ascent_autopilot(200.0, 200.0, 26.5, true)
		else:
			sim.ignite()
			_advance(sim, t0 + 3.0)
			sim.release_clamps()
		_advance(sim, t0 + 3.0 + fly)
		world.clock.set_time(t0 + 3.0 + fly)
		if _args.has("circularize"):
			sim.plan_circularization()
			sim.wait_idle(60.0)
		if _args.has("burn"):  # --burn=<prograde m/s>,<node in s>,<capture after s>
			var b := (_args["burn"] as String).split(",")
			sim.plan_burn(float(b[1]), float(b[0]), 0.0, 0.0)
			sim.wait_idle(60.0)
			var t_capture := t0 + 3.0 + fly + float(b[2])
			_advance(sim, t_capture)
			world.clock.set_time(t_capture)
	var cam := world.camera
	if _args.has("yaw"): cam.yaw = float(_args["yaw"])
	if _args.has("pitch"): cam.pitch = float(_args["pitch"])
	if _args.has("distance"): cam.distance_m = float(_args["distance"])
	print("render_check: telemetry ", sim.telemetry(sim.controlled_index()))
	if _args.has("watch_from_site"):
		_watch_from_site(world, site)


# --watch_from_site[=<east m>]: after the flight, a ground view from the launch site (the eye 2 m above it, moved
# <east m> east, default 2000 m) aimed at the controlled vehicle, --fov (default 50) - the rocket seen from the ground.
func _watch_from_site(world: World, site: Dictionary) -> void:
	var east_m := float(_args["watch_from_site"]) if _args["watch_from_site"] != "" else 2000.0
	var lat: float = site["latitude_deg"]
	var lon: float = site["longitude_deg"] + rad_to_deg(east_m / (World.EARTH_SEMI_MAJOR_AXIS_M * cos(deg_to_rad(lat))))
	world.set_ground_site({"id": "watch", "name": "watch", "latitude_deg": lat, "longitude_deg": lon,
			"height_m": site["height_m"]})
	world.update_world()
	var distance := _aim_at_vehicle(world)
	world.camera.fov = float(_args.get("fov", "50"))
	print("render_check: watching from %.4f, %.4f: vehicle %.1f km away at elevation %.2f deg" % [lat, lon,
			distance / 1000.0, rad_to_deg(world.camera.elevation)])


## Aims the ground view at the controlled vehicle; its distance (m).
func _aim_at_vehicle(world: World) -> float:
	var sim := world.flight.sim
	var cam := world.camera
	var d := (sim.body_position_gcrf_m(sim.controlled_index()) - cam.position_gcrf_m)
	var distance := d.length()
	d /= distance
	cam.elevation = asin(clampf(d.dot(cam.site_up_gcrf), -1.0, 1.0))
	cam.azimuth = atan2(d.dot(cam.site_east_gcrf), d.dot(cam.site_north_gcrf))
	return distance


# Starts a mission at the clock's instant and flies it to the requested mission time or timeline event.
func _fly_mission(world: World) -> void:
	var sim := world.flight.sim
	var id := sim.start_mission(_args.get("mission", "starship_leo"), world.clock.tdb_s, _args.has("wait_on_pad"))
	sim.wait_idle(600.0)
	print("render_check: mission start ", sim.result(id))
	world.flight.launch_site_id = sim.mission_site_id()
	world.set_focus(World.FOCUS_VEHICLE)
	world.clock.warp = 0.0
	if _args.has("at_event") and _fly_until_event(sim):
		# The search flew past the event (the sim cannot go back): the same mission again, flown to shortly before it.
		var target_late := _mission_target(sim)
		if sim.advance_to(target_late) > target_late - 600.0:
			id = sim.start_mission(_args.get("mission", "starship_leo"), world.clock.tdb_s, _args.has("wait_on_pad"))
			sim.wait_idle(600.0)
			print("render_check: mission restarted ", sim.result(id))
			world.set_focus(World.FOCUS_VEHICLE)
			world.clock.warp = 0.0
			_advance(sim, target_late - 600.0)
	var target := _mission_target(sim)
	if _args.has("at_event"):  # predicted events move while flying: approach, then aim again
		_advance(sim, target - 600.0)
		target = _mission_target(sim)
	_advance(sim, target)
	target = maxf(target, _mission_target(sim))  # T-0 is the recorded liftoff once it happened
	_advance(sim, target)
	world.clock.set_time(sim.advance_to(target))
	for i in int(_args.get("view", "0")):
		world.view_target.cycle(sim, 1)
	if _args.has("legacy_hud"):
		(_main.get("legacy_hud") as CanvasLayer).visible = true
	(_main.get("overlay_host") as OverlayHost).overlay.snap_timeline()
	var index := world.view_target.resolve(sim)
	var now := sim.advance_to(target)
	print("render_check: events (s from now) ", sim.mission_timeline().get("events", []).map(
			func(e): return "%s %.0f%s" % [e["kind"], float(e["tdb_s"]) - now, "?" if e["predicted"] else ""]))
	print("render_check: mission clock ", sim.mission_timeline().get("clock_s"), "  viewed ", index, "  telemetry ",
			sim.telemetry(index))


## Events of later mission phases (a lunar landing after the outbound coast) are neither recorded nor predicted at the
## start: fly in EVENT_SEARCH_STEP_S chunks until the --at_event kind appears in the timeline (or EVENT_SEARCH_MAX_S).
## Its instant is kept for _mission_target (a restarted mission has not reached it yet). True if it had to be searched.
func _fly_until_event(sim: FlightSim) -> bool:
	var spec := _event_spec()
	var kind: String = spec[0]
	var start := sim.mission_start_tdb_s()
	var t := start
	var wall0 := Time.get_ticks_msec()
	while t - start < EVENT_SEARCH_MAX_S and sim.is_active():
		var e := _find_event(sim.mission_timeline(), kind, spec[2])
		if not e.is_empty():
			print("render_check: event %s found after %.0f s of flight (%d ms, %s)" % [kind, t - start,
					Time.get_ticks_msec() - wall0, "predicted" if e["predicted"] else "recorded"])
			if not e["predicted"]:  # a predicted one moves (a resting lander's touchdown is always 'now')
				_found_event_tdb_s = float(e["tdb_s"])
			return t > start
		t += EVENT_SEARCH_STEP_S
		_advance(sim, t)
	return false


## The n-th (1-based) recorded event of a kind, else the n-th among recorded and predicted ones ({} if none).
static func _find_event(timeline: Dictionary, kind: String, occurrence := 1) -> Dictionary:
	var recorded: Array[Dictionary] = []
	var all: Array[Dictionary] = []
	for e: Dictionary in timeline.get("events", []):
		if e["kind"] == kind:
			all.append(e)
			if not e["predicted"]:
				recorded.append(e)
	if recorded.size() >= occurrence:
		return recorded[occurrence - 1]
	return all[occurrence - 1] if all.size() >= occurrence else {}


## (kind, offset s, occurrence) of --at_event=<kind>[,<offset s>[,<occurrence>]].
func _event_spec() -> Array:
	var spec := (_args["at_event"] as String).split(",")
	return [spec[0], float(spec[1]) if spec.size() > 1 else 0.0, int(spec[2]) if spec.size() > 2 else 1]


## TDB of --t_plus after T-0 (default 75 s), or of --at_event=<kind>[,<offset s>[,<n>]] (its n-th recorded occurrence,
## else the n-th among all; an event recorded by _fly_until_event in an earlier run of the same mission first).
func _mission_target(sim: FlightSim) -> float:
	var timeline: Dictionary = sim.mission_timeline()
	if _args.has("at_event"):
		var spec := _event_spec()
		var offset: float = spec[1]
		if not is_nan(_found_event_tdb_s):
			return _found_event_tdb_s + offset
		var e := _find_event(timeline, spec[0], spec[2])
		if not e.is_empty():
			return float(e["tdb_s"]) + offset
		print("render_check: no event %s in %s" % [spec[0], timeline.get("events", []).map(func(e): return e["kind"])])
	var t0 = timeline.get("t0_tdb_s")
	var zero := float(t0) if t0 != null else sim.mission_start_tdb_s()
	return zero + float(_args.get("t_plus", "75")) + SIM_TICK_S  # the sim stops on its tick grid, never just short


# The simulation thread advances in chunks; wait until it has reached the target.
func _advance(sim: FlightSim, target: float) -> void:
	for i in 1000:
		sim.advance_to(target)
		sim.wait_idle(5.0)
		if sim.advance_to(target) >= target - 0.01 or not sim.is_active():
			return


# Lunar missions: the viewed body's CG taken into the drawn Moon's frame (the game's moon.fixed_to_gcrf, the frame of
# the terrain mesh) against the landing site and the core's surface there; agreeing with the session's own
# altitude_terrain_m shows that the render places the site where the simulation has it.
func _print_deck_placement(world: World) -> void:
	var sim := world.flight.sim
	var index := world.view_target.resolve(sim)
	if index < 0:
		return
	# the viewed body's CG in the drawn deck's frame (x bow, y port, z deck normal; origin the deck centre on its top)
	var cg_render := Frames.to_render(sim.body_position_gcrf_m(index) - world.camera.position_gcrf_m)
	var cg_deck := Frames.from_render(world.drone_ship.deck_transform().affine_inverse() * cg_render)
	var legs: Dictionary = sim.body_legs(index)
	var expected := sim.body_cg_station_m(index) - float(legs.get("foot_station_m", NAN))
	print("render_check: drone ship - CG %.3f m above the drawn deck (CG above the feet %.3f m), %.2f m from its centre; "
			% [cg_deck.z, expected, Vector2(cg_deck.x, cg_deck.y).length()]
			+ "%s" % TimePanel.sea_state_line(sim.landing_zone()))


func _print_lunar_placement(world: World) -> void:
	var sim := world.flight.sim
	var index := world.view_target.resolve(sim)
	if index < 0:
		return
	var cg_fixed := world.moon.fixed_to_gcrf.transposed() * (world.view_target.cg_gcrf_m(sim, index) - world.moon_gcrf_m)
	var ll := CubeSphere.lat_lon_deg(cg_fixed.normalized())
	var surface: Dictionary = world.core.surface_samples("moon", PackedFloat64Array([ll.x]), PackedFloat64Array([ll.y]))
	var surface_m: float = surface["height_m"][0] if bool(surface["present"]) else 0.0
	var site: Dictionary = sim.lunar_site()
	var site_fixed: Vector3 = site["position_moon_me_m"]
	var horizontal_m := site_fixed.normalized().cross(cg_fixed.normalized()).length() * site_fixed.length()
	print("render_check: lunar CG at %.5f, %.5f: %.2f m above the drawn surface (session altitude_terrain_m %.2f), "
			% [ll.x, ll.y, cg_fixed.length() - World.MOON_RADIUS_M - surface_m, sim.telemetry(index)["altitude_terrain_m"]]
			+ "%.1f m from the site %s" % [horizontal_m, site["name"]])


# Ground view at --lat / --lon on the Earth or (--body=moon) the Moon; --height defaults to the core's surface there
# (StarsimCore.surface_samples: the terrain, else the ellipsoid / the mean sphere), the eye --eye_height above it.
func _view_from_ground(world: World) -> void:
	var lat := float(_args["lat"])
	var lon := float(_args["lon"])
	var body: String = _args.get("body", "earth")
	var height := float(_args.get("height", "0"))
	if not _args.has("height"):
		# The core's surface (the Earth terrain or the ellipsoid, the lunar terrain or the mean sphere): an eye at 0 m on
		# the ellipsoid stood ~150 m inside the terrain at Dallas (night-vision review I5).
		var samples: Dictionary = world.core.surface_samples(body, PackedFloat64Array([lat]), PackedFloat64Array([lon]))
		height = samples["height_m"][0] if bool(samples["present"]) else 0.0
		print("render_check: %s ground view at %.4f, %.4f: surface %.2f m" % [body, lat, lon, height])
	var site := {"id": "custom", "name": "custom", "body": body, "latitude_deg": lat, "longitude_deg": lon,
			"height_m": height}
	if _args.has("eye_height"):
		site["eye_height_m"] = float(_args["eye_height"])
	world.set_ground_site(site)


# Ground view at sea: the eye --eye_height (default 10 m) above the physics' surface at --sea_lat / --sea_lon (the
# geoid at sea; the ellipsoid without the Earth terrain).
func _view_at_sea(world: World) -> void:
	var lat := float(_args["sea_lat"])
	var lon := float(_args["sea_lon"])
	var samples: Dictionary = world.core.surface_samples("earth", PackedFloat64Array([lat]), PackedFloat64Array([lon]))
	var surface_m: float = samples["height_m"][0] if bool(samples["present"]) else 0.0
	var water: bool = bool(samples["present"]) and samples["water"][0] != 0
	print("render_check: sea view at %.4f, %.4f: surface %.2f m, water %s" % [lat, lon, surface_m, water])
	world.set_ground_site({"id": "sea", "name": "sea", "latitude_deg": lat, "longitude_deg": lon, "height_m": surface_m,
			"eye_height_m": float(_args.get("eye_height", "10"))})


## Aims a ground view at the true (unrefracted) Sun.
func _aim_at_sun(world: World) -> void:
	world.update_world()
	var cam := world.camera
	var d := (world.sun_apparent_from_earth_m - cam.position_gcrf_m).normalized()
	cam.elevation = asin(clampf(d.dot(cam.site_up_gcrf), -1.0, 1.0))
	cam.azimuth = atan2(d.dot(cam.site_east_gcrf), d.dot(cam.site_north_gcrf))
	if _args.has("away_from_sun"):  # the Sun behind the camera, the view at the same elevation
		cam.azimuth += PI


# Ground camera: point at the Moon.
func _aim_at_moon(world: World) -> void:
	world.update_world()
	var cam := world.camera
	var to_moon := world.moon_gcrf_m - cam.position_gcrf_m
	var up := cam.site_up_gcrf
	var north := cam.site_north_gcrf
	var east := cam.site_east_gcrf
	var d := to_moon.normalized()
	cam.elevation = asin(clampf(d.dot(up), -1.0, 1.0))
	cam.azimuth = atan2(d.dot(east), d.dot(north))
