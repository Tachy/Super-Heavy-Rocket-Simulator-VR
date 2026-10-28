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
#          --weather=slab:<bottom m>,<top m>,<liquid kg/m^3>,<fraction>; any other value exits with an error;
#          --cloud_quality=desktop|vr (the clouds' quality preset; default: the game's; --cloud_divisor=<n>: its march
#          resolution divisor instead), --cloud_debug=1..4 (the cloud
#          pass's debug output, clouds.gdshaderinc cloud_debug_output), --cloud_detail_levels=0..3 (the detail levels),
#          --cloud_scattering=table|closed (the multiple-scattering model: Monte Carlo table, default; closed form),
#          --no_light_cache (the light toward the Sun marched per sample: the light cache's reference),
#          --handover=<near m>,<far m> (the near / far hand-over band; 1e9,1e9: the march alone, 0,0: the far tier).
#          --far_fade=<start m>,<end m> (the far tier's shapes-to-column fade; 1e9,1e9: the shapes, 0,0: the column).
# Eye adaptation: on as in the game (adapted to the final view); --fixed_exposure switches it off.
# Image identity: --deterministic[=64] once settled resets the cloud history and the cloud pass's frame counter and
#          saves after exactly that many more frames (with --fixed_exposure, for before / after comparisons).
#          --repeat_identical[=2] once settled and the light cache complete compares that many consecutive frames of
#          the unchanged scene (no history reset): any differing pixel fails (exit code 1; the deterministic march,
#          static clouds Task 3; the HUD's frame-rate line hidden). --jittered_march: the jittered march with
#          temporal accumulation (the flag CloudLayer.deterministic_march off). --no_air_shadow: the clouds' shadows on the air off (the flag
#          CloudLayer.air_shadow).
# Cloud view cache (cloud view cache Task 4; on as in the game: settling waits until it has nothing left to fill and
#          prints the time to level 0 / the full level in view and to the whole cube): --no_view_cache shows the
#          per-frame march instead (the flag CloudLayer.view_cache). --ab_view_cache: the cache's cloud image against
#          the march at divisor 1, 2 x 2 blocks: median <= 1 %, p99 where no scene geometry is <= the march's own p99
#          against itself turned by 0.5 px (exit code 1); --shimmer_check: 16 frames yawed +-0.3 px, the cache's
#          frame-to-frame change p99 within 10 % of the march's at divisor 1 (exit code 1); --jump_check: after a view
#          jump and after a layout re-creation (the render scale changed) every frame's cloud coverage within 0.02 of
#          the per-frame march's (exit code 1).
#          --frames_every=0.5 [--frames_for=30] (with --vehicle and --fly; cloud view cache Task 5): once settled the
#          flight goes on in frames of 1/45 s (the VR frame) for that many seconds, the camera on board or (with
#          --watch_from_site) on the ground aimed at the vehicle; every 0.5 s of flight the cache's image (with the near
#          shell when a cloud is near) against the per-frame march's (the flag off, the preset's divisor): the mean alpha
#          over the annulus where the march's clouds lie at the near shell's radius (+-25 %) within 0.02 (no ring at
#          R_near; exit code 1), and the flight frames' GPU time (median, p99; with the VR preset p99 <= its frame target,
#          exit code 1). Images: --out with _fly_<t> for the samples' cache frames.
# Refraction: on as in the game; --no_refraction draws Sun, Moon and stars at their geometric directions.
#          --look_at_sun aims a ground view at the (true) Sun; with --away_from_sun the opposite azimuth.
# Sun through a deck: --sun_through_deck=<vertical tau> (with --lat / --lon and --utc): a uniform slab 1-2 km of that
#          optical depth (liquid = tau / 150 000 kg/m^3: beta = 150 LWC) and the view aimed at the Sun (static clouds Task 9).
# GPU time: --gpu_time[=120] once settled, the root viewport's measured GPU render time over that many frames (median,
#          p99); --render_scale=<f> renders the 3D view at f x the window per axis (1.65: the VR pixel proxy);
#          --gpu_passes: also per GPU timestamp interval (the cloud pass's kernels, fill, shadow maps), the light cache
#          settled first.
#     [--yaw=0.6] [--pitch=-0.3] [--distance=2.2e7] [--azimuth=90] [--elevation=5] [--fov=50] --out=C:/tmp/x.png
extends SceneTree

const FRAMES_TO_SETTLE := 6
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
## --ab_view_cache (cloud view cache Task 4, the controller's rulings for general views): the luminance error's floor
## (a fraction of the mean, as cloud_check CACHE_ERROR_FLOOR) and the median's bound; the p99 over the blocks without
## scene geometry is bounded by the same view's march against itself turned by AB_SHIFT_PX eye pixels (the max over four
## turns, +-x and +-y; two point samplings of the same hard edges, as the cache's texel grid is against the pixels).
const AB_ERROR_FLOOR := 0.05
const AB_MEDIAN_TOLERANCE := 0.01
const AB_SHIFT_PX := 0.5
const AB_TURNS := ["+x", "-x", "+y", "-y"]  ## the calibration's half-pixel turns (azimuth, elevation)
const AB_TURN_DIRECTIONS := [Vector2(1, 0), Vector2(-1, 0), Vector2(0, 1), Vector2(0, -1)]
## A pixel without scene geometry: the march's scene distance at least this (cloud_pass.gdshaderinc: 1e30 for depth 0).
const AB_SKY_M := 1.0e20
## --shimmer_check: frames, the head's yaw (eye pixels, alternating) and the allowed excess over the direct march.
const SHIMMER_FRAMES := 16
const SHIMMER_PX := 0.3
const SHIMMER_EXCESS := 0.1
## --jump_check (Task 4 fix round 1, controller ruling "no frame without clouds"): the cloud coverage (mean 1 - T over
## the frame, every JUMP_STRIDE-th pixel per axis) of every frame after a view jump and after a layout re-creation
## within JUMP_COVERAGE_TOLERANCE of the per-frame march's (the reference); the render scale that re-creates the layout
## (> the planner's 10 % hysteresis) and the frames recorded per phase at most.
const JUMP_COVERAGE_TOLERANCE := 0.02
const JUMP_STRIDE := 4
const JUMP_RENDER_SCALE := 1.3
const JUMP_MAX_FRAMES := 900
## --frames_every (cloud view cache Task 5): the flight's frame (s; the VR frame at 45 Hz), the annulus around the near
## shell's radius (the march's cloud distance within this factor of it), its pixel stride, the fewest pixels a sample
## counts with, and the bound on the mean alpha's difference there (the brief's: no ring at R_near).
const FLY_FRAME_S := 1.0 / 45.0
const FLY_ANNULUS_FACTOR := 1.25
const FLY_STRIDE := 2
const FLY_MIN_ANNULUS := 200
const FLY_ALPHA_TOLERANCE := 0.02

var _args := {}
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
## The cloud view cache's convergence (cloud view cache Task 4; the cache on): the wall clock its timing started at (the
## planner's last restart), when the view's tiles had level 0 / the full level and the whole cube was done (s, -1: not
## yet), frames without a filled tile, the frames' main-thread schedule time (FrameProfiler) and fill intervals.
var _cache_start_s := -1.0
var _cache_start_frame := 0
var _cache_resets := -1
var _cache_level0 := Vector2(-1.0, -1.0)  ## (s, frames)
var _cache_view_full := Vector2(-1.0, -1.0)
var _cache_full := Vector2(-1.0, -1.0)
var _cache_idle := 0
var _cache_done := false
var _cache_section_ms: Array[float] = []
var _cache_section_l0_ms: Array[float] = []  ## the schedule's time in frames after level 0 was everywhere in view
var _cache_fill_ms: Array[float] = []
var _cache_fills: Array[Vector3] = []  ## the planner's measured fills: (measured ms, predicted ms, budget ms)
var _cache_covered := Vector2(-1.0, -1.0)  ## when the planner stopped covering (s, frames)
var _check_failed := false  ## --ab_view_cache / --shimmer_check failed (exit code 1)
## --ab_view_cache: the cache's image, its ground mask, frames left before the march's image is read.
var _ab_cache: Image
var _ab_wait := -1
var _ab_anchor_m := 0.0  ## the camera's distance from the cache's anchor
var _ab_march: Image  ## the march's image (then the calibration against it turned by AB_SHIFT_PX)
var _ab_ground := PackedByteArray()
var _ab_result := {}  ## the cache against the march (judged after the calibration)
var _ab_turn := 0  ## the calibration turn being measured (AB_TURNS)
var _ab_calibration_p99 := 0.0  ## the max of the turns' p99 (no scene geometry)
var _ab_base_angles := Vector2.ZERO  ## the view's azimuth / elevation before the turns
## --shimmer_check: the phase (0 the cache, 1 the direct march), the base azimuth, frames of the phase, the cloud
## images of its frames, the cache's p99 of the change.
var _shimmer_phase := 0
var _shimmer_base := NAN
var _shimmer_k := 0
var _shimmer_images: Array[PackedFloat32Array] = []
var _shimmer_cache_p99 := -1.0
var _shimmer_wait := 0
## --jump_check: the phase (0 converged: the reference taken, 1 after the view jump, 2 after the layout re-creation),
## the reference coverage, the frames of the phase, the worst deviation per phase, the phase's frames with the march
## covering.
var _jump_phase := 0
var _jump_reference := -1.0
var _jump_cache_coverage := -1.0
var _jump_frames := 0
var _jump_worst := [0.0, 0.0, 0.0]
var _jump_covering := [0, 0, 0]
var _jump_idle := 0
## --frames_every (cloud view cache Task 5): the flight's instant (TDB s), the next sample's, the phase (0 flying, > 0
## frames left of the march's reference with the clock held), the sample's cache image, the samples (t, annulus pixels,
## cache alpha, march alpha), the flight frames' GPU ms and how many of them ran the near shell.
var _fly_t := NAN
var _fly_end := NAN
var _fly_next_sample := NAN
var _fly_march_wait := 0
var _fly_cache_image: Image
var _fly_samples: Array[Vector4] = []
var _fly_gpu_ms: Array[float] = []
var _fly_near_frames := 0
var _fly_near_at_sample := 0  ## CloudEffect.near_shell_frames when the sample's cache frame was taken
var _fly_near_sample := false  ## the near shell ran in the frames before the sample
var _fly_site := {}


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		var parts := arg.trim_prefix("--").split("=", true, 1)
		_args[parts[0]] = parts[1] if parts.size() > 1 else ""
	if _args.has("sun_through_deck"):
		_args["weather"] = "slab:%d,%d,%s,1" % [SUN_DECK_BOTTOM_M, SUN_DECK_TOP_M,
				float(_args["sun_through_deck"]) / SUN_DECK_EXTINCTION_PER_LWC / (SUN_DECK_TOP_M - SUN_DECK_BOTTOM_M)]
		_args["look_at_sun"] = ""
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)
	_start_s = Time.get_ticks_msec() / 1000.0
	if _args.has("render_scale"):
		root.scaling_3d_mode = Viewport.SCALING_3D_MODE_BILINEAR
		root.scaling_3d_scale = float(_args["render_scale"])
	if _args.has("gpu_time"):
		RenderingServer.viewport_set_measure_render_time(root.get_viewport_rid(), true)
	CloudEffect.capture_timestamps = _args.has("gpu_passes")
	FrameProfiler.enabled = true  # the view cache's main-thread schedule time (_cache_waiting)


func _process(_delta: float) -> bool:
	var world: World = _main.get_node("World")
	world.clock.warp = 0.0
	# Weather downloads only on request (--weather): a render check must not fetch gigabytes of cloud data.
	world.weather.enabled = _weather_online()
	if _frames == 0:
		if _args.has("cloud_quality"):
			world.clouds.quality = CloudLayer.Quality.VR if _args["cloud_quality"] == "vr" else CloudLayer.Quality.DESKTOP
		_show_weather_file(world)
		if _args.has("cloud_divisor"):  # the march's resolution divisor (the preset's otherwise; tuning runs)
			var preset: Dictionary = CloudLayer.PRESETS[world.clouds.quality]
			world.clouds.effect.set_quality(int(_args["cloud_divisor"]), float(preset["temporal_weight"]),
					float(preset["clip_gamma"]))
		if _args.has("cloud_detail_levels"):  # the detail levels used (the preset's otherwise)
			ShaderGlobals.set_value(&"cloud_detail_levels", int(_args["cloud_detail_levels"]))
		if _args.has("no_light_cache"):  # the fidelity flag (CloudLayer.light_cache_enabled)
			world.clouds.light_cache_enabled = false
		if _args.has("jittered_march"):  # the fidelity flag (CloudLayer.deterministic_march)
			world.clouds.deterministic_march = false
		if _args.has("no_view_cache"):  # the fidelity flag (CloudLayer.view_cache): the per-frame march
			world.clouds.view_cache = false
		if _args.has("no_air_shadow"):  # the fidelity flag (CloudLayer.air_shadow)
			world.clouds.air_shadow = false
		if _args.has("cloud_scattering"):  # the fidelity flag (CloudLayer.ScatteringModel)
			world.clouds.scattering_model = CloudLayer.ScatteringModel.CLOSED_FORM \
					if _args["cloud_scattering"] == "closed" else CloudLayer.ScatteringModel.TABLE
		if _args.has("handover"):  # the near / far hand-over band (CloudLayer.PRESETS handover_m): "<near m>,<far m>"
			var band: PackedStringArray = _args["handover"].split(",")
			ShaderGlobals.set_value(&"cloud_handover_near_m", float(band[0]))
			ShaderGlobals.set_value(&"cloud_handover_far_m", float(band[1]))
		if _args.has("far_fade"):  # the far tier's shapes-to-column fade (PRESETS far_fade_m): "<start m>,<end m>"
			var fade: PackedStringArray = _args["far_fade"].split(",")
			ShaderGlobals.set_value(&"cloud_far_pattern_fade_m", Vector2(float(fade[0]), float(fade[1])))
		if _args.has("repeat_identical"):  # the HUD's frame-rate line changes from frame to frame
			TimePanel.show_frame_rate = false
		if _args.has("cloud_debug"):  # the cloud pass's debug output (clouds.gdshaderinc cloud_debug_output)
			ShaderGlobals.set_value(&"cloud_debug_output", int(_args["cloud_debug"]))
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
	# The cloud view cache (on unless --no_view_cache): until nothing is left to fill (its timing printed below).
	var cache_waiting := _cache_waiting(world)
	if _frames >= FRAMES_TO_SETTLE and not weather_waiting and not clouds_waiting and not cache_waiting \
			and (_terrain_idle >= FRAMES_TO_SETTLE or _frames >= TERRAIN_MAX_FRAMES):
		if _args.has("ab_view_cache") and not _ab_frames(world):
			return false
		if _args.has("shimmer_check") and not _shimmer_frames(world):
			return false
		if _args.has("jump_check") and not _jump_frames_done(world):
			return false
		if _args.has("frames_every") and not _fly_frames_done(world):
			return false
		if _args.has("deterministic"):
			if _deterministic_left < 0:  # the same jitter sequence and history from here in every run
				var effect: CloudEffect = world.clouds.effect
				effect.reset_history()
				RenderingServer.call_on_render_thread(func() -> void: effect._frame = 0)
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
		if _cache_start_s >= 0.0:
			_print_cache_timing(world)
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
		# Default: <repo>/screenshots/ (git-ignored; every test image goes there, user 2026-10-27).
		var out: String = _args.get("out", ProjectSettings.globalize_path("res://").path_join(
				"../screenshots/render_check.png").simplify_path())
		DirAccess.make_dir_recursive_absolute(out.get_base_dir())
		var error := root.get_viewport().get_texture().get_image().save_png(out)
		print("render_check: saved %s (error %d)" % [out, error])
		if _repeat_failed or _check_failed:
			quit(1)
		return true  # quit
	return false


## --gpu_time[=120]: collects the root viewport's GPU render time of that many frames; true when done (printed).
## --gpu_passes (with --gpu_time): the frames are collected once the light cache is settled (steady state), and per
## frame the GPU time between the RenderingDevice's captured timestamps of the previous frame, per name (the interval
## a timestamp starts; CloudEffect.capture_timestamps: cloud_light_fill, cloud_pass_<kernel>, shadow_pass; their
## cloud_pass_* summed as cloud_pass), printed as median and p99 per name.
func _gpu_frames(world: World) -> bool:
	var wanted: int = int(_args["gpu_time"]) if _args["gpu_time"].is_valid_int() else 120
	if _args.has("gpu_passes"):
		_cache_settled = _cache_settled + 1 if world.clouds.light_cache.settled() else 0
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
	return true


## The cloud view cache's convergence (the cache on, the clouds shown): true while tiles are still filled (until
## FRAMES_TO_SETTLE frames without one; TERRAIN_MAX_FRAMES at most). Records from the planner's last restart the time
## to level 0 / the full level over the view's sample rays and to the whole cube, the main-thread schedule time per
## frame (FrameProfiler section cloud_view_cache) and the measured fill intervals.
func _cache_waiting(world: World) -> bool:
	var clouds := world.clouds
	var planner := clouds.view_cache_planner
	planner.wait()  # its state is read below (the run started last frame; normally done long ago)
	var frame_sections := FrameProfiler.take_frame()
	if _cache_done or not clouds.view_cache or not bool(ShaderGlobals.value(&"cloud_enabled")) \
			or planner.layout == null or _frames >= TERRAIN_MAX_FRAMES:
		return false
	var now := Time.get_ticks_msec() / 1000.0
	if planner.resets != _cache_resets:  # a restart: the timing starts again
		_cache_resets = planner.resets
		_cache_start_s = now
		_cache_start_frame = _frames
		_cache_level0 = Vector2(-1.0, -1.0)
		_cache_view_full = Vector2(-1.0, -1.0)
		_cache_covered = Vector2(-1.0, -1.0)
		_cache_section_ms.clear()
		_cache_section_l0_ms.clear()
		_cache_fill_ms.clear()
		_cache_fills.clear()
	var since := Vector2(now - _cache_start_s, _frames - _cache_start_frame)
	var section_ms := int(frame_sections.get(&"cloud_view_cache", 0)) / 1000.0
	_cache_section_ms.append(section_ms)
	if not planner.view_missing:  # level 0 everywhere in the view's cone (the planner's own test)
		_cache_section_l0_ms.append(section_ms)
	_cache_fill_ms.append(clouds.view_cache_fill_ms)
	_cache_fills.append_array(planner.take_fill_log())
	if _cache_covered.x < 0.0 and not planner.last_covering:
		_cache_covered = since
	var view: Dictionary = clouds.effect.view_cache_view()
	var lowest := CloudViewCacheSchedule.MAX_LEVEL
	for d: Vector3 in view.get("dirs", []):
		lowest = mini(lowest, planner.shown_level(planner.layout.tile_of(d)))
	if _cache_level0.x < 0.0 and lowest >= 0:
		_cache_level0 = since
	if _cache_view_full.x < 0.0 and lowest == CloudViewCacheSchedule.MAX_LEVEL:
		_cache_view_full = since
	# Idle: nothing filled and every cross-fade complete.
	var fading := planner.now_s - planner.last_fade_s < CloudViewCache.CROSSFADE_S
	_cache_idle = _cache_idle + 1 if planner.last_tiles == 0 and not fading else 0
	if _cache_idle >= FRAMES_TO_SETTLE:
		_cache_full = since
		_cache_done = true
		return false
	return true



## The view cache's convergence and costs since the last restart: the times to level 0 in view, the end of the march's
## covering, the full level in view and the whole cube; the main-thread schedule time (all frames, and the frames after
## level 0 was in view: the 1.0 ms bound); the measured fills against their budget (p99, max, the largest excess, the
## fills over budget + 2 ms) and against their prediction.
func _print_cache_timing(world: World) -> void:
	var clouds := world.clouds
	var layout := clouds.view_cache_planner.layout
	var sections := _stats(_cache_section_ms)
	var settled := _stats(_cache_section_l0_ms)
	var measured: Array[float] = []
	var excess: Array[float] = []
	var ratio: Array[float] = []
	var over := 0
	for f in _cache_fills:
		measured.append(f.x)
		excess.append(f.x - f.z)
		ratio.append(f.x / maxf(f.y, 1.0e-6))
		over += 1 if f.x > f.z + 2.0 else 0
	var fills := _stats(measured)
	print("render_check: view cache %d^2 texels per face: level 0 in view after %.2f s (%d frames), covering ended "
			% [layout.face_texels if layout != null else 0, _cache_level0.x, _cache_level0.y]
			+ "after %.2f s (%d frames), the full level in view after %.2f s (%d frames), the whole cube after %.2f s "
			% [_cache_covered.x, _cache_covered.y, _cache_view_full.x, _cache_view_full.y, _cache_full.x]
			+ "(%d frames); budget now %.2f ms" % [_cache_full.y, clouds.cache_budget_ms()])
	print("render_check: view cache schedule main thread: all %d frames median %.3f / p99 %.3f / max %.3f ms; after "
			% [sections["n"], sections["median"], sections["p99"], sections["max"]]
			+ "level 0 in view (%d frames) median %.3f / p99 %.3f / max %.3f ms" % [settled["n"], settled["median"],
			settled["p99"], settled["max"]])
	print("render_check: view cache fills: %d measured, GPU median %.3f / p99 %.3f / max %.3f ms; over the budget: max "
			% [fills["n"], fills["median"], fills["p99"], fills["max"]]
			+ "%.3f ms, %d fills over budget + 2 ms; measured / predicted median %.2f, p99 %.2f, max %.2f"
			% [_stats(excess)["max"], over, _stats(ratio)["median"], _stats(ratio)["p99"], _stats(ratio)["max"]])


## n, median, p99 and max of a list (0 when empty).
static func _stats(values: Array[float]) -> Dictionary:
	if values.is_empty():
		return {"n": 0, "median": 0.0, "p99": 0.0, "max": 0.0}
	var sorted := values.duplicate()
	sorted.sort()
	return {"n": sorted.size(), "median": sorted[sorted.size() / 2],
		"p99": sorted[mini(int(0.99 * sorted.size()), sorted.size() - 1)], "max": sorted.back()}


## --ab_view_cache (the cache full): the cloud image of the cache, then of the per-frame march at divisor 1 (the
## reference, cloud view cache controller rulings): both box-filtered over 2 x 2 pixels, the per-block luminance error
## |cache - march| / max(|march|, AB_ERROR_FLOOR x the mean) - median <= AB_MEDIAN_TOLERANCE over all blocks, and the
## p99 over the blocks whose rays miss the ground and every other scene geometry <= the calibration's: the march against
## itself with the view turned by AB_SHIFT_PX in four directions, the max over them (the same metric; the full
## image's p99 and T RMS printed). Images: the two frames and the error (x 10) next to --out. True when done (a
## failure: exit code 1).
func _ab_frames(world: World) -> bool:
	var effect := world.clouds.effect
	if _ab_wait < 0:
		_ab_cache = effect.output.get_layer_data(0)
		var camera: Vector3 = ShaderGlobals.value(&"atmosphere_camera_hi") \
				+ ShaderGlobals.value(&"atmosphere_camera_lo")
		_ab_anchor_m = camera.distance_to(world.clouds.view_cache_planner.anchor_earth_m)
		root.get_viewport().get_texture().get_image().save_png(_out_path("_cache"))
		world.clouds.view_cache = false
		var preset: Dictionary = CloudLayer.PRESETS[world.clouds.quality]
		effect.set_quality(1, float(preset["temporal_weight"]), float(preset["clip_gamma"]))
		_ab_wait = FRAMES_TO_SETTLE
		return false
	if _ab_wait > 0:
		_ab_wait -= 1
		return false
	if _ab_march != null:
		# The calibration: the march against itself with the view turned by AB_SHIFT_PX (two point samplings of the
		# same clouds, as the cache's texel grid is against the pixels) - the bound of the cache's p99 is the MAX over
		# the four turns (+-x azimuth, +-y elevation; controller ruling: one turn under-samples the march's variance).
		var shifted := _cloud_compare(effect.output.get_layer_data(0), _ab_march, _ab_ground)
		print("render_check: ab_view_cache calibration turn %d (%s): the march at divisor 1 against itself turned by "
				% [_ab_turn, AB_TURNS[_ab_turn]] + "%.1f px, 2 x 2 blocks: median %.3f %%, p99 %.3f %% without scene "
				% [AB_SHIFT_PX, shifted["median"] * 100.0, shifted["p99_sky"] * 100.0]
				+ "geometry (all: p99 %.3f %%)" % (shifted["p99_all"] * 100.0))
		_ab_calibration_p99 = maxf(_ab_calibration_p99, shifted["p99_sky"])
		_ab_turn += 1
		if _ab_turn < AB_TURNS.size():
			_ab_turn_view(world)
			return false
		var ok: bool = _ab_result["median"] <= AB_MEDIAN_TOLERANCE and _ab_result["p99_sky"] <= _ab_calibration_p99
		print("render_check: ab_view_cache %s: median %.3f %% (bound %.1f %%), p99 without scene geometry %.3f %% "
				% ["PASS" if ok else "FAIL", _ab_result["median"] * 100.0, AB_MEDIAN_TOLERANCE * 100.0,
				_ab_result["p99_sky"] * 100.0] + "(bound: the calibration's max over %d turns %.3f %%)"
				% [AB_TURNS.size(), _ab_calibration_p99 * 100.0])
		_check_failed = _check_failed or not ok
		return true
	var march := effect.output.get_layer_data(0)
	root.get_viewport().get_texture().get_image().save_png(_out_path("_march"))
	if march.get_size() != _ab_cache.get_size():
		print("render_check: ab_view_cache FAIL: image sizes differ (%s / %s)" % [_ab_cache.get_size(),
				march.get_size()])
		_check_failed = true
		return true
	var ground := _geometry_blocks(effect.debug_centre_data("depth"), march.get_size())
	if ground.is_empty():
		print("render_check: ab_view_cache FAIL: the march's scene distances are not the image's size (%s)"
				% effect.centre_low)
		_check_failed = true
		return true
	var r := _cloud_compare(_ab_cache, march, ground)
	_ab_result = r
	print("render_check: ab_view_cache: cache vs the march at divisor 1, 2 x 2 blocks: luminance error median %.3f %%, "
			% (r["median"] * 100.0) + "p99 %.3f %% over %d blocks without scene geometry (all: "
			% [r["p99_sky"] * 100.0, r["sky_blocks"]] + "p99 %.3f %%), worst %.1f %%, T RMS %.5f (mean luminance %.4f)"
			% [r["p99_all"] * 100.0, r["worst"] * 100.0, r["t_rms"], r["mean"]]
			+ "; the camera %.2f m from the anchor" % _ab_anchor_m)
	(r["picture"] as Image).save_png(_out_path("_ab_error"))
	_ab_march = march
	_ab_ground = ground
	_ab_base_angles = Vector2(world.camera.azimuth, world.camera.elevation)
	_ab_turn_view(world)
	return false


## Turns the view by AB_SHIFT_PX eye pixels in the direction of the current calibration turn (from the base view).
func _ab_turn_view(world: World) -> void:
	var cam := world.camera
	var shift_rad := AB_SHIFT_PX * 2.0 * tan(deg_to_rad(cam.fov) / 2.0) / float(_ab_march.get_height())
	var direction: Vector2 = AB_TURN_DIRECTIONS[_ab_turn]
	cam.azimuth = _ab_base_angles.x + direction.x * shift_rad
	cam.elevation = _ab_base_angles.y + direction.y * shift_rad
	_ab_wait = 2


## --jump_check (the cache full; controller ruling: no frame without clouds): the reference's cloud coverage - the
## per-frame march at the preset's divisor (the flag off for FRAMES_TO_SETTLE frames; the cache is kept) - then a view
## jump (CloudLayer.notify_view_jump: the cache restarts, the march covers) and, once the cache is idle again, a layout
## re-creation (the render scale changed to JUMP_RENDER_SCALE: a new face size) - every frame of both phases within
## JUMP_COVERAGE_TOLERANCE of the reference. The output read in a frame is the previous frame's (drawn after the jump
## from the first one on). True when done (a failure: exit code 1).
func _jump_frames_done(world: World) -> bool:
	var clouds := world.clouds
	var planner := clouds.view_cache_planner
	var coverage := _coverage(clouds.effect.output.get_layer_data(0))
	if _jump_phase == 0:
		if _jump_frames == 0:
			_jump_cache_coverage = coverage  # the converged cache's (printed)
			clouds.view_cache = false
		_jump_frames += 1
		if _jump_frames == FRAMES_TO_SETTLE:
			_jump_reference = coverage
			clouds.view_cache = true
		if _jump_frames < 2 * FRAMES_TO_SETTLE:
			return false
		print("render_check: jump_check reference: the march's coverage %.4f (the converged cache's %.4f)"
				% [_jump_reference, _jump_cache_coverage])
		_jump_phase = 1
		_jump_frames = 0
		clouds.notify_view_jump()
		return false
	_jump_frames += 1
	var phase := _jump_phase
	_jump_worst[phase] = maxf(_jump_worst[phase], absf(coverage - _jump_reference))
	_jump_covering[phase] += 1 if planner.last_covering else 0
	var fading := planner.now_s - planner.last_fade_s < CloudViewCache.CROSSFADE_S
	_jump_idle = _jump_idle + 1 if planner.last_tiles == 0 and not fading else 0
	if _jump_idle < FRAMES_TO_SETTLE and _jump_frames < JUMP_MAX_FRAMES:
		return false
	print("render_check: jump_check phase %s: %d frames (%d covering), coverage deviation max %.4f (reference %.4f)"
			% ["view jump" if phase == 1 else "layout re-creation", _jump_frames, _jump_covering[phase],
			_jump_worst[phase], _jump_reference])
	_jump_idle = 0
	_jump_frames = 0
	if phase == 1:
		_jump_phase = 2
		root.scaling_3d_mode = Viewport.SCALING_3D_MODE_BILINEAR
		root.scaling_3d_scale = JUMP_RENDER_SCALE * (float(_args["render_scale"]) if _args.has("render_scale") else 1.0)
		return false
	var ok: bool = _jump_worst[1] <= JUMP_COVERAGE_TOLERANCE and _jump_worst[2] <= JUMP_COVERAGE_TOLERANCE \
			and planner.resets >= 3
	print("render_check: jump_check %s: coverage within %.3f of the march's after the view jump (max %.4f) "
			% ["PASS" if ok else "FAIL", JUMP_COVERAGE_TOLERANCE, _jump_worst[1]]
			+ "and the layout re-creation (max %.4f; restarts %d)" % [_jump_worst[2], planner.resets])
	_check_failed = _check_failed or not ok
	return true


## --frames_every=<s> (cloud view cache Task 5; header): the flight on in FLY_FRAME_S frames, every <s> a sample of
## the cache's image against the per-frame march's over the near shell's boundary annulus. True when done (a failure:
## exit code 1).
func _fly_frames_done(world: World) -> bool:
	var clouds := world.clouds
	var sim := world.flight.sim
	var every := float(_args["frames_every"])
	if is_nan(_fly_end):
		_fly_end = _fly_t + float(_args.get("frames_for", "30"))
		_fly_next_sample = _fly_t
		_fly_near_frames = clouds.effect.near_shell_frames  # (the flight's count from here; march frames add none)
		RenderingServer.viewport_set_measure_render_time(root.get_viewport_rid(), true)
	if _fly_march_wait > 0:  # the march's reference of the sample: the clock held, the flag off
		_fly_march_wait -= 1
		if _fly_march_wait > 0:
			return false
		var march := clouds.effect.output.get_layer_data(0)
		root.get_viewport().get_texture().get_image().save_png(_out_path("_fly_%05.1f_march" % (_fly_t - _fly_end
				+ float(_args.get("frames_for", "30")))))
		var annulus := _near_annulus(clouds.effect.debug_centre_data("depth"), clouds.effect.centre_low,
				march.get_size())
		var cache_alpha := _mean_alpha(_fly_cache_image, annulus)
		var march_alpha := _mean_alpha(march, annulus)
		_fly_samples.append(Vector4(_fly_t, annulus.size(), cache_alpha, march_alpha))
		print("render_check: fly sample t %.1f s: annulus %d pixels, mean alpha cache %.4f, march %.4f (diff %.4f), "
				% [_fly_t - _fly_end + float(_args.get("frames_for", "30")), annulus.size(), cache_alpha, march_alpha,
				cache_alpha - march_alpha] + "near shell %s, nearest cloud %.0f m, camera %.0f m above the ellipsoid, "
				% [_fly_near_sample, clouds.nearest_cloud_m, ShaderGlobals.value(&"cloud_camera_height_m")]
				+ "re-anchors %d, evicted %d" % [clouds.view_cache_planner.reanchors,
				clouds.view_cache_planner.evicted_tiles])
		clouds.view_cache = true
		return false
	if _fly_t >= _fly_next_sample - 1.0e-6 and _fly_cache_image == null:
		# The sample: this frame's cache image (drawn last frame at this instant), then the march's at the same instant.
		_fly_cache_image = clouds.effect.output.get_layer_data(0)
		_fly_near_sample = clouds.effect.near_shell_frames > _fly_near_at_sample
		root.get_viewport().get_texture().get_image().save_png(_out_path("_fly_%05.1f" % (_fly_t - _fly_end
				+ float(_args.get("frames_for", "30")))))
		clouds.view_cache = false
		_fly_march_wait = FRAMES_TO_SETTLE
		return false
	if _fly_cache_image != null:
		_fly_cache_image = null
		_fly_next_sample += every
	if _fly_t >= _fly_end:
		_fly_near_frames = clouds.effect.near_shell_frames - _fly_near_frames
		return _fly_report()
	_fly_gpu_ms.append(RenderingServer.viewport_get_measured_render_time_gpu(root.get_viewport_rid()))
	_fly_near_at_sample = clouds.effect.near_shell_frames
	_fly_t += FLY_FRAME_S
	_advance(sim, _fly_t)
	world.clock.tdb_s = _fly_t  # the clock's own advance (set_time is a jump: the view cache would restart)
	if _args.has("watch_from_site"):
		_aim_at_vehicle(world)
	return false


## --frames_every's verdict: every sample with annulus pixels within FLY_ALPHA_TOLERANCE, and the flight frames' GPU
## p99 within the VR preset's frame target.
func _fly_report() -> bool:
	var worst := 0.0
	var counted := 0
	for sample in _fly_samples:
		if sample.y >= FLY_MIN_ANNULUS:
			counted += 1
			worst = maxf(worst, absf(sample.z - sample.w))
	var gpu := _stats(_fly_gpu_ms)
	var target := float(CloudLayer.PRESETS[(_main.get_node("World") as World).clouds.quality]["frame_target_ms"])
	var frame_ok: bool = target <= 0.0 or gpu["p99"] <= target
	var ok := counted > 0 and worst <= FLY_ALPHA_TOLERANCE and frame_ok
	print("render_check: frames_every %s: %d samples (%d with >= %d annulus pixels), mean alpha over the near shell's "
			% ["PASS" if ok else "FAIL", _fly_samples.size(), counted, FLY_MIN_ANNULUS] + "boundary within %.3f of the "
			% FLY_ALPHA_TOLERANCE + "march's (worst %.4f); flight frames %d (near shell in %d): GPU median %.2f / p99 "
			% [worst, gpu["n"], _fly_near_frames, gpu["median"]] + "%.2f / max %.2f ms (frame target %.1f ms)"
			% [gpu["p99"], gpu["max"], target])
	_check_failed = _check_failed or not ok
	return true


## The eye pixels (every FLY_STRIDE-th per axis) whose march cloud distance lies within FLY_ANNULUS_FACTOR of the near
## shell's radius, the cloud in front of the scene (the march's centre view = the view on the desktop).
static func _near_annulus(depth: PackedByteArray, low: Vector2i, size: Vector2i) -> PackedVector2Array:
	var pixels := PackedVector2Array()
	if depth.size() != low.x * low.y * 8:
		return pixels
	var r := CloudLayer.near_radius_m()
	for y in range(0, size.y, FLY_STRIDE):
		for x in range(0, size.x, FLY_STRIDE):
			var i := (mini(y * low.y / size.y, low.y - 1) * low.x + mini(x * low.x / size.x, low.x - 1)) * 8
			var scene_m := depth.decode_float(i)
			var cloud_m := depth.decode_float(i + 4)
			if cloud_m >= r / FLY_ANNULUS_FACTOR and cloud_m <= r * FLY_ANNULUS_FACTOR and scene_m > cloud_m:
				pixels.append(Vector2(x, y))
	return pixels


static func _mean_alpha(image: Image, pixels: PackedVector2Array) -> float:
	var sum := 0.0
	for p in pixels:
		sum += 1.0 - image.get_pixel(int(p.x), int(p.y)).a
	return sum / maxi(pixels.size(), 1)


## The cloud coverage of a cloud image (RGBA half: premultiplied colour + T): the mean of 1 - T over every
## JUMP_STRIDE-th pixel per axis.
static func _coverage(image: Image) -> float:
	var sum := 0.0
	var n := 0
	for y in range(0, image.get_height(), JUMP_STRIDE):
		for x in range(0, image.get_width(), JUMP_STRIDE):
			sum += 1.0 - image.get_pixel(x, y).a
			n += 1
	return sum / maxi(n, 1)


## --shimmer_check (the cache full): SHIMMER_FRAMES frames with the head yawed by +-SHIMMER_PX eye pixels alternating,
## the cloud image's per-pixel change from frame to frame (|luminance| + the mean luminance x |T|): its 99th percentile
## with the cache may exceed the full-resolution per-frame march's (divisor 1, the same frames) by SHIMMER_EXCESS at
## most. True when done (a failure: exit code 1).
func _shimmer_frames(world: World) -> bool:
	var cam := world.camera
	var effect := world.clouds.effect
	if is_nan(_shimmer_base):
		_shimmer_base = cam.azimuth
	if _shimmer_wait > 0:
		_shimmer_wait -= 1
		return false
	if _shimmer_k > 0:  # the frame drawn at the previous yaw
		_shimmer_images.append(_cloud_channels(effect.output.get_layer_data(0)))
	if _shimmer_images.size() == SHIMMER_FRAMES:
		var p99 := _change_p99(_shimmer_images)
		_shimmer_images.clear()
		_shimmer_k = 0
		cam.azimuth = _shimmer_base
		if _shimmer_phase == 0:
			_shimmer_cache_p99 = p99
			_shimmer_phase = 1
			world.clouds.view_cache = false
			var preset: Dictionary = CloudLayer.PRESETS[world.clouds.quality]
			effect.set_quality(1, float(preset["temporal_weight"]), float(preset["clip_gamma"]))
			_shimmer_wait = FRAMES_TO_SETTLE
			return false
		var ok := _shimmer_cache_p99 <= (1.0 + SHIMMER_EXCESS) * p99
		print("render_check: shimmer_check %s: frame-to-frame change p99 with the cache %.5f, the march at divisor 1 "
				% ["PASS" if ok else "FAIL", _shimmer_cache_p99]
				+ "%.5f (ratio %.3f, bound %.2f; %d frames, yaw +-%.2f px)"
				% [p99, _shimmer_cache_p99 / maxf(p99, 1.0e-12), 1.0 + SHIMMER_EXCESS, SHIMMER_FRAMES, SHIMMER_PX])
		_check_failed = _check_failed or not ok
		return true
	# One eye pixel at the view's centre (rad): the vertical field of view over the image height.
	var pixel_rad := 2.0 * tan(deg_to_rad(cam.fov) / 2.0) / float(effect.output.get_height())
	cam.azimuth = _shimmer_base + (1.0 if _shimmer_k % 2 == 0 else -1.0) * SHIMMER_PX * pixel_rad
	_shimmer_k += 1
	return false


## Per pixel of a cloud image (RGBA half: premultiplied colour + T): luminance and T, interleaved.
static func _cloud_channels(image: Image) -> PackedFloat32Array:
	var out := PackedFloat32Array()
	out.resize(2 * image.get_width() * image.get_height())
	var i := 0
	for y in range(image.get_height()):
		for x in range(image.get_width()):
			var c := image.get_pixel(x, y)
			out[i] = 0.2126 * c.r + 0.7152 * c.g + 0.0722 * c.b
			out[i + 1] = c.a
			i += 2
	return out


## The 99th percentile of the per-pixel change between consecutive images (|luminance| + the mean |luminance| x |T|).
static func _change_p99(images: Array[PackedFloat32Array]) -> float:
	var mean := 0.0
	var first := images[0]
	for i in range(0, first.size(), 2):
		mean += absf(first[i]) / (first.size() / 2)
	var changes := PackedFloat32Array()
	for k in range(1, images.size()):
		var a := images[k - 1]
		var b := images[k]
		for i in range(0, a.size(), 2):
			changes.append(absf(b[i] - a[i]) + mean * absf(b[i + 1] - a[i + 1]))
	changes.sort()
	return changes[int(0.99 * (changes.size() - 1))]


## Per 2 x 2 block of an image of `size`: 1 where a pixel shows scene geometry (the ground, the tower) - from the
## march's scene distances at divisor 1 (its centre view is the image: CloudEffect.debug_centre_data "depth", RG32F);
## empty when the sizes differ.
static func _geometry_blocks(depth: PackedByteArray, size: Vector2i) -> PackedByteArray:
	var mask := PackedByteArray()
	if depth.size() != size.x * size.y * 8:
		return mask
	var blocks := size / 2
	mask.resize(blocks.x * blocks.y)
	for by in range(blocks.y):
		for bx in range(blocks.x):
			var geometry := 0
			for j in range(2):
				for i in range(2):
					if depth.decode_float(((2 * by + j) * size.x + 2 * bx + i) * 8) < AB_SKY_M:
						geometry = 1
			mask[by * blocks.x + bx] = geometry
	return mask


## The A / B comparison of two cloud images (the blocks' means; ground: 1 per block whose ray meets the ground).
static func _cloud_compare(a: Image, b: Image, ground: PackedByteArray) -> Dictionary:
	var blocks := a.get_size() / 2
	var n := blocks.x * blocks.y
	var la := PackedFloat32Array()
	var lb := PackedFloat32Array()
	la.resize(n)
	lb.resize(n)
	var t_sq := 0.0
	var mean := 0.0
	for by in range(blocks.y):
		for bx in range(blocks.x):
			var ca := Color(0, 0, 0, 0)
			var cb := Color(0, 0, 0, 0)
			for j in range(2):
				for i in range(2):
					ca += a.get_pixel(2 * bx + i, 2 * by + j) * 0.25
					cb += b.get_pixel(2 * bx + i, 2 * by + j) * 0.25
			var k := by * blocks.x + bx
			la[k] = 0.2126 * ca.r + 0.7152 * ca.g + 0.0722 * ca.b
			lb[k] = 0.2126 * cb.r + 0.7152 * cb.g + 0.0722 * cb.b
			t_sq += (ca.a - cb.a) * (ca.a - cb.a)
			mean += absf(lb[k]) / n
	var errors := PackedFloat32Array()
	var sky := PackedFloat32Array()
	var picture := Image.create(blocks.x, blocks.y, false, Image.FORMAT_RGBF)
	for k in range(n):
		var e := absf(la[k] - lb[k]) / maxf(absf(lb[k]), AB_ERROR_FLOOR * mean)
		errors.append(e)
		if ground[k] == 0:
			sky.append(e)
		picture.set_pixel(k % blocks.x, k / blocks.x, Color(minf(10.0 * e, 1.0), 0.0, 0.0) if ground[k] == 0
				else Color(0.0, minf(10.0 * e, 1.0), 0.2))
	errors.sort()
	sky.sort()
	return {"median": errors[n / 2], "p99_all": errors[int(0.99 * (n - 1))], "worst": errors[n - 1],
		"p99_sky": sky[int(0.99 * (sky.size() - 1))] if sky.size() > 0 else 0.0, "sky_blocks": sky.size(),
		"t_rms": sqrt(t_sq / n), "mean": mean, "picture": picture}


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


## --repeat_identical[=2]: once the light cache is settled (no fill, no cross-fade), collects that many consecutive
## frames of the unchanged scene (no history reset) and compares them; true when all are collected (then the last one
## is saved). Frames that differ fail the check (exit code 1 after the save).
func _repeat_frames(world: World) -> bool:
	_cache_settled = _cache_settled + 1 if world.clouds.light_cache.settled() else 0
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
	return true


## --weather, --weather=online or --weather=<max seconds>: the weather service online (downloads); off / analog / a
## file / a slab: not.
func _weather_online() -> bool:
	if not _args.has("weather"):
		return false
	var value: String = _args["weather"]
	return value == "" or value == "online" or value.is_valid_float()


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
	_fly_t = t0 + 3.0 + fly
	_fly_site = site
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
