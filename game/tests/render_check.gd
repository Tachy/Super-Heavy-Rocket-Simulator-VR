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
#          resolution divisor instead, fractional allowed: 1.5),
#          --cloud_method=old|hybrid|hybrid_no_skip|hybrid_old_light|baked (the hybrid march / baked volume prototypes, CloudLayer.march_method: the view cache off, the world volume's upkeep on and settled
#          first), --cloud_debug=1..4 (the cloud
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
#          --watch_from_site) on the ground aimed at the vehicle; every 0.5 s of flight the cache's image (with the
#          near shell when a cloud is near) against the per-frame march's (the flag off, the preset's divisor): the
#          mean alpha over the annulus where the march's clouds lie at the near shell's radius (+-25 %) within 0.02
#          (no ring at R_near), 0.06 while the camera is within a cloud layer's vertical extent +- R_near (the field's
#          nearest cloud within R_near: the cache's accepted lag in a deck passage; exit code 1), and the flight
#          frames' GPU time (median, p99; with the VR preset p99 <= its frame target, exit code 1). Images: --out with
#          _fly_<t> for the samples' cache frames.
# World volume (world volume plan Task 5): --world_volume / --no_world_volume set the fidelity flag
#          CloudLayer.world_volume (default: the environment STARSIM_WORLD_VOLUME=1 or the user argument --world_volume;
#          off otherwise); on, the volume's upkeep runs (GPU intervals cloud_volume_upkeep: the bake, cloud_volume_light:
#          the light); the settle waits for its fill and light (Task 6). Task 7: with a level shown the frame is the
#          volume's march (CLOUD_VOLUME_FULL: one ray per eye pixel; GPU interval cloud_volume_full).
#          --volume_fill_scale=<x> fills the volume at x times the upkeep budget (checks of the march, not of the
#          upkeep: the cold start's upkeep verdict is then skipped).
#          --world_volume_unsplit forces CLOUD_VOLUME_FULL once Task 8's tiers exist (Task 7: the only path).
#          --light_entry_samples=<n> / --edge_curl=<a>: the fidelity flags (surface correction, curl distortion).
#          --ab_reference: the volume's cloud image against the reference path (world_volume and view_cache off,
#          divisor 1, 8 x steps): median <= 1 %, p99 without scene geometry <= the reference's own p99 turned by
#          0.5 px (the max over four turns; exit code 1). --edge_check: on the same frames, the edge pixels' (alpha
#          0.05-0.95) band width <= 1.5 x the reference's and alpha high-pass energy >= 0.7 x the reference's (no blur
#          regression, ruling R2; exit code 1); the reference's own change under the turns is printed.
#          --front_check=W,S: the clock at W x for S s in 1/45 s frames, once with the volume and
#          once with the reference path from the same instant; per frame the 16 x 16-block means of the cloud image's
#          frame-to-frame radiance change; the front step = the largest difference of (volume change - reference
#          change) between neighbouring blocks / the mean radiance, <= 1 % (and no frame whose mean alpha drops below
#          half the reference's); prints "front_check: max step <x> % lag <deg>" (exit code 1).
# Refraction: on as in the game; --no_refraction draws Sun, Moon and stars at their geometric directions.
#          --look_at_sun aims a ground view at the (true) Sun; with --away_from_sun the opposite azimuth.
# Sun through a deck: --sun_through_deck=<vertical tau> (with --lat / --lon and --utc): a uniform slab 1-2 km of that
#          optical depth (liquid = tau / 150 000 kg/m^3: beta = 150 LWC) and the view aimed at the Sun (static clouds Task 9).
# GPU time: --gpu_time[=120] once settled, the root viewport's measured GPU render time over that many frames (median,
#          p99); --render_scale=<f> renders the 3D view at f x the window per axis (1.65: the VR pixel proxy);
#          --gpu_passes: also per GPU timestamp interval (the cloud pass's kernels, fill, shadow maps), the light cache
#          settled first.
# March counters / ablations (world volume plan Task 0): --counters adds the march's counters (CloudEffect.counters)
#          and prints, once settled, "counter <name> median / p90 / p99 / max / sum" per march pixel of density_fetches,
#          light_fetches, occupied_steps (a step whose sample lies inside cloud: extinction > 0) and empty_steps, and
#          writes <out>_counter_<name>.png (normalised to the maximum); --ablate=<name,...> switches work off for timing
#          runs (CloudEffect.ABLATIONS: density, light, air, field, shape, detail, subsamples, outline, air_shadow,
#          far_tier; the environment variable STARSIM_CLOUD_ABLATE does the same).
# Spike S (world volume plan Task 2; throw-away, removed in Task 13): --spike=a|b runs CloudSpike's far tier alone
#          ([2 km, end] from the cyclops point; a: procedural density, b: baked) instead of the march, --spike=ref today's
#          march clipped to [2 km, end] (the sanity check's reference; with --handover=0,0); --spike_scale=0.5|1.0 its
#          resolution per axis. Prints the centre image's mean alpha / luminance and the spike's plan; with --counters
#          also tau_limited_steps, extinction_steps, structure_limited_steps (occupied_steps: steps in occupied space).
#          Round 2 (Task 2b): --spike_levers=1,2,4,s,t the step rule's levers, --spike_stop=<T> the termination
#          (_start_spike).
# Stereo harness (world volume plan Task 1): --stereo=<json> renders through the multiview path with a stand-in headset
#          (StereoMockXr: the recorded per-eye projections, eye transforms and render target of the JSON that PerfLog
#          writes at a VR start, game/tests/data/pimax_dream_air_projections.json) with the VR cloud preset (unless
#          --cloud_quality; run without --xr-mode off, which switches the multiview shader variants off), the eye
#          images written as <out>_eye0.png / <out>_eye1.png (the intermediate progress images
#          of --ab_view_cache / --frames_every are not written); the log shows "stereo view_count 2". --cloud_view=<id>
#          (CloudViews: the gate views V1 .. V7, e.g. V3_era5) prepends that view's arguments; an argument given on the
#          command line wins. --frames_every also works with --mission / --at_event (the booster catch V7).
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
## --ab_reference (world volume plan Task 7, ruling C23): the reference path's steps x this (cloud_grain_check.gd
## REFERENCE_STEP_FACTOR: the converged march along the ray).
const AB_REFERENCE_STEP_FACTOR := 8
## --edge_check (spec "feathery edges"; Task 7 ruling R2: no blur regression - the user wants feathery, detailed edges,
## appearance otherwise goes to the user as images): edge pixels have alpha in this range; the candidate's band width
## at most EDGE_WIDTH_MAX x the reference's, its high-pass energy at least EDGE_ENERGY_MIN x the reference's.
const EDGE_ALPHA := Vector2(0.05, 0.95)
const EDGE_WIDTH_MAX := 1.5
const EDGE_ENERGY_MIN := 0.7
## --front_check=W,S (Task 7, C13): the frame (s; the VR frame at 45 Hz), the blocks' size (pixels), the bound on the
## step of (volume change - reference change) between neighbouring blocks relative to the mean radiance; a frame is
## "black" when its mean alpha falls below FRONT_BLACK_SHARE of the reference's at the same instant.
const FRONT_FRAME_S := 1.0 / 45.0
const FRONT_BLOCK_PX := 16
const FRONT_TOLERANCE := 0.01
const FRONT_BLACK_SHARE := 0.5
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
## counts with, and the bound on the mean alpha's difference there (the brief's: no ring at R_near) - with the camera
## within a cloud layer's vertical extent +- R_near the controller's ruling's (Task 5: the cache's lag inside a deck
## passage is the spec's "briefly softer near clouds").
const FLY_FRAME_S := 1.0 / 45.0
const FLY_ANNULUS_FACTOR := 1.25
const FLY_STRIDE := 2
const FLY_MIN_ANNULUS := 200
const FLY_ALPHA_TOLERANCE := 0.02
const FLY_ALPHA_IN_LAYER_TOLERANCE := 0.06
## The flight frames after a sample's reference whose measured GPU time (the viewport's: the previous frame's) is still
## the reference march's (measured, Task 5 fix round 1: the first one shows its outline pass, 18 - 26 ms; the second
## the cache's) - not counted.
const FLY_GPU_LAG_FRAMES := 1
## [E] The flight's phases in the report (Task 5 fix round 2: the per-phase numbers): on the pad while the vehicle is
## within this of its altitude at the flight's start (m; the ascent's first second climbs more), then the camera in the
## layer (within a cloud layer's extent +- R_near, the alpha bound's rule) or outside it (above the deck).
const FLY_PAD_RISE_M := 20.0
const FLY_PHASES := ["pad", "in the layer", "outside the layer"]

var _args := {}
var _stereo: StereoMockXr  ## --stereo: the stand-in headset
var _stereo_capture_frames := 0
var _main: Node
var _frames := 0
var _start_s := 0.0
var _terrain_idle := 0  ## consecutive frames without terrain loading
var _found_event_tdb_s := NAN  ## --at_event: the event's instant found by flying (_fly_until_event)
var _deterministic_left := -1  ## --deterministic: frames still to draw after the reset (-1: not yet reset)
var _cache_settled := 0  ## --repeat_identical: consecutive frames with the light cache settled
var _volume_upkeeps: Array[float] = []  ## the world volume's measured upkeep (ms) per frame of its cold start
var _volume_seen := 0  ## measured frames of the world volume read so far
var _volume_reported := false
var _volume_release := -1  ## frames of the volume's march left before the measurement (-1: still held, 0: drawn)
var _repeat_images: Array[Image] = []  ## --repeat_identical: the consecutive frames drawn so far
var _repeat_failed := false  ## --repeat_identical: the frames differ (exit code 1)
var _gpu_ms: Array[float] = []  ## --gpu_time: the measured frames' GPU render times
var _gpu_passes := {}  ## --gpu_passes: timestamp name -> Array of the measured frames' GPU ms
var _sweep_left: Array[float] = []  ## --divisor_sweep: the divisors still to measure
var _sweep_wait := -1  ## --divisor_sweep: frames left to settle at the current divisor (-1: not started)
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
## --edge_check: the candidate's and the reference's edge metrics, the reference's largest change under the turns.
var _edge_candidate := {}
var _edge_reference := {}
var _edge_turn_change := {}
## --front_check: the phase (0 the volume's run, 1 the reference's), the frame, the start instant, the runs' per-frame
## block means and mean alpha, the Sun's elevation per frame.
var _front_phase := -1
var _front_frame := 0
var _front_t0 := 0.0
var _front_blocks := [[], []]
var _front_alpha := [[], []]
var _front_sun_deg: Array[float] = []
var _front_row_blocks := 1  ## blocks per row of the cloud image
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
var _fly_in_layer: Array[bool] = []  ## per sample: the camera within a cloud layer's extent +- R_near
var _fly_sample_gpu_from := 0  ## the first flight frame since the last sample (its frames' worst GPU time printed)
var _fly_gpu_skip := 0  ## flight frames whose measured GPU time is still a reference march frame's
var _fly_section_ms: Array[float] = []  ## the flight frames' main-thread section cloud_view_cache (FrameProfiler)
var _last_section_ms := 0.0  ## the last frame's section cloud_view_cache (ms)
var _fly_gpu_ms: Array[float] = []
var _fly_gpu_phase: Array[int] = []  ## per counted flight frame: its phase (FLY_PHASES)
var _fly_sample_phase: Array[int] = []  ## per sample: its phase
var _fly_start_altitude_m := NAN
var _fly_near_frames := 0
var _fly_near_at_sample := 0  ## CloudEffect.near_shell_frames when the sample's cache frame was taken
var _fly_near_sample := false  ## the near shell ran in the frames before the sample
var _fly_site := {}


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
	# The cloud pass's diagnostics builds (compiled when the CloudEffect is created, i.e. with the main scene).
	CloudEffect.counters = _args.has("counters")
	if _args.has("ablate"):
		CloudEffect.ablate = PackedStringArray(String(_args["ablate"]).split(",", false))
	if _args.has("spike") and not _start_spike():
		return
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
	FrameProfiler.enabled = true  # the view cache's main-thread schedule time (_cache_waiting)


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
			var preset: Dictionary = CloudLayer.PRESETS[world.clouds.quality]
			world.clouds.effect.set_quality(float(_args["cloud_divisor"]), float(preset["temporal_weight"]),
					float(preset["clip_gamma"]))
		if _args.has("cloud_detail_levels"):  # the detail levels used (the preset's otherwise)
			ShaderGlobals.set_value(&"cloud_detail_levels", int(_args["cloud_detail_levels"]))
		if _args.has("no_light_cache"):  # the fidelity flag (CloudLayer.light_cache_enabled)
			world.clouds.light_cache_enabled = false
		if _args.has("jittered_march"):  # the fidelity flag (CloudLayer.deterministic_march)
			world.clouds.deterministic_march = false
		if _args.has("no_view_cache"):  # the fidelity flag (CloudLayer.view_cache): the per-frame march
			world.clouds.view_cache = false
		if _args.has("world_volume"):  # the fidelity flag (CloudLayer.world_volume; Task 5: the upkeep alone)
			world.clouds.world_volume = true
		if _args.has("no_world_volume"):
			world.clouds.world_volume = false
		if _args.has("world_volume_unsplit"):  # Task 8's tiers off: the full-resolution march per eye
			CloudEffect.volume_unsplit = true
		if _args.has("light_entry_samples"):  # the fidelity flag (CloudLayer.light_entry_samples)
			world.clouds.light_entry_samples = int(_args["light_entry_samples"])
		if _args.has("edge_curl"):  # the fidelity flag (CloudLayer.edge_curl)
			world.clouds.edge_curl = float(_args["edge_curl"])
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
	# The cloud view cache (on unless --no_view_cache): until nothing is left to fill (its timing printed below).
	var cache_waiting := _cache_waiting(world)
	# The world volume (flag on): until its fill and its light are complete (world volume plan Task 6: the upkeep is
	# measured after the cold start).
	cache_waiting = cache_waiting or (world.clouds.volume_needed() and _volume_waiting(world) and _frames < TERRAIN_MAX_FRAMES)
	if CloudSpike.failed:
		_refuse("render_check: the spike's plan was refused (see the CloudSpike error)")
		return true
	cache_waiting = cache_waiting or (CloudSpike.mode != CloudSpike.Mode.OFF and not CloudSpike.ready)  # spike S's build
	var settled := _frames >= FRAMES_TO_SETTLE and not weather_waiting and not clouds_waiting and not cache_waiting \
			and (_terrain_idle >= FRAMES_TO_SETTLE or _frames >= TERRAIN_MAX_FRAMES)
	# The world volume's march draws the frame once the scene and the volume are settled (CloudEffect.volume_march_hold:
	# the settle runs at today's march's frame rate), then FRAMES_TO_SETTLE frames of it before anything is measured.
	if world.clouds.volume_needed() and _volume_release != 0:
		CloudEffect.volume_march_hold = not settled
		if settled:
			_volume_release = FRAMES_TO_SETTLE if _volume_release < 0 else _volume_release - 1
		return false
	if settled:
		if (_args.has("ab_view_cache") or _args.has("ab_reference") or _args.has("edge_check")) \
				and not _ab_frames(world):
			return false
		if _args.has("front_check") and not _front_frames(world):
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
		if _args.has("counters"):
			_print_counters(world)
		if _args.has("spike"):
			_print_centre_stats(world)
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
		if _repeat_failed or _check_failed:
			quit(1)
		return true  # quit
	return false


## --counters: the march counters of the last frame per march pixel (CloudEffect.debug_centre_data "counters", RGBA32UI:
## density fetches, light fetches, occupied steps, empty steps) - median, p90, p99, max and sum of each over all march
## pixels (clear-sky pixels count as 0), and <out>_counter_<name>.png, normalised to the maximum.
func _print_counters(world: World) -> void:
	var effect: CloudEffect = world.clouds.effect
	var data := effect.debug_centre_data("counters")
	var low := effect.counters_size
	if data.is_empty() or low.x * low.y == 0:
		print("render_check: counters: no data (no cloud pass ran)")
		return
	var values := data.to_int32_array()  # the texels' uints (counts stay far below 2^31)
	var names := ["density_fetches", "light_fetches", "occupied_steps", "empty_steps"]
	# Spike S (cloud_spike.gdshaderinc): channels 1-3 carry a second count in their high 16 bits.
	var spike := CloudSpike.mode != CloudSpike.Mode.OFF
	if spike:
		names.append_array(["tau_limited_steps", "extinction_steps", "structure_limited_steps"])
	var count := low.x * low.y
	for channel in range(names.size()):
		var column := PackedInt32Array()
		column.resize(count)
		var sum := 0
		var largest := 0
		for i in range(count):
			var v := values[4 * i + (channel if channel < 4 else channel - 3)]
			if spike and channel >= 1:
				v = (v >> 16) & 0xffff if channel >= 4 else v & 0xffff
			column[i] = v
			sum += v
			largest = maxi(largest, v)
		var sorted := column.duplicate()
		sorted.sort()
		print("render_check: counter %s median %d p90 %d p99 %d max %d sum %d" % [names[channel],
				sorted[count / 2], sorted[int(0.9 * (count - 1))], sorted[int(0.99 * (count - 1))], largest, sum])
		var bytes := PackedByteArray()
		bytes.resize(count)
		for i in range(count):
			bytes[i] = int(round(255.0 * column[i] / maxf(largest, 1.0)))
		var path := _out_path("_counter_" + names[channel])
		Image.create_from_data(low.x, low.y, false, Image.FORMAT_L8, bytes).save_png(path)
		print("render_check: saved %s" % path)


## --gpu_time[=120]: collects the root viewport's GPU render time of that many frames; true when done (printed).
## --gpu_passes (with --gpu_time): the frames are collected once the light cache is settled (steady state), and per
## frame the GPU time between the RenderingDevice's captured timestamps of the previous frame, per name (the interval
## a timestamp starts; CloudEffect.capture_timestamps: cloud_light_fill, cloud_pass_<kernel>, shadow_pass; their
## cloud_pass_* summed as cloud_pass), printed as median and p99 per name.
## --divisor_sweep=3,2,1.5,1 (with --gpu_time; the hybrid march prototype): the measurement repeated per cloud march
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
		var baked_ok := world.clouds.march_method != CloudEffect.MarchMethod.BAKED 				or (world.clouds.effect.baked != null and (world.clouds.effect.baked.settled()
				or (CloudBaked.rebake_frames > 0 and world.clouds.effect.baked.ready)))  # forced re-bakes: measured
		_cache_settled = _cache_settled + 1 if world.clouds.light_cache.settled() and baked_ok else 0
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
		print("render_check: divisor_sweep divisor %s method %s hybrid_shown %s (the lines above)" % [
				world.clouds.divisor(), world.clouds.march_method, world.clouds.effect.hybrid_shown])
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


## True while the world volume's fill or light is incomplete (CloudWorldVolumeSchedule.complete, light_stale); the
## cold start's measured upkeep (bake + light, GPU ms) collected and printed once it is done.
func _volume_waiting(world: World) -> bool:
	var stats := world.clouds.world_volume_stats()
	if int(stats["measured_frames"]) > _volume_seen:
		_volume_seen = int(stats["measured_frames"])
		_volume_upkeeps.append(float(stats["upkeep_ms"]))
	var schedule: CloudWorldVolumeSchedule = world.clouds.world_volume_schedule
	var fill_scale := float(_args.get("volume_fill_scale", "1"))
	var waiting := not schedule.complete()
	for k in CloudWorldVolumeLayout.LEVELS:
		waiting = waiting or schedule.light_stale(k)
	schedule.budget_scale = fill_scale if waiting else 1.0
	if not waiting and not _volume_reported:
		_volume_reported = true
		var ms := _volume_upkeeps.duplicate()
		ms.sort()
		print("render_check: world volume complete after %d frames; cold start upkeep (bake + light) median %.3f ms, "
				% [_frames, ms[ms.size() / 2] if not ms.is_empty() else -1.0] + "p99 %.3f ms, max %.3f ms" % [
				ms[mini(ms.size() - 1, int(0.99 * ms.size()))] if not ms.is_empty() else -1.0,
				ms.max() if not ms.is_empty() else -1.0])
		# Task 6 review round 1d: the cold start is a transition - its upkeep stays within 2 x the spike cap (the cap
		# itself binds in-flight load) in every measured frame.
		var transition_ms := 2.0 * CloudWorldVolumeSchedule.UPKEEP_SPIKE_MS
		if fill_scale != 1.0:
			print("render_check: world volume filled at %.0f x the budget (--volume_fill_scale): no upkeep verdict"
					% fill_scale)
		elif not ms.is_empty() and ms.max() > transition_ms:
			print("render_check: FAIL - world volume cold start upkeep max %.3f ms > transition cap %.1f ms" % [ms.max(),
					transition_ms])
			_check_failed = true
		var parts := PackedStringArray()
		var sums: Dictionary = stats["measured_levels"]
		for k: int in range(CloudWorldVolumeLayout.LEVELS - 1, -1, -1):
			if sums.has(k):
				var m: Array = sums[k]
				parts.append("L%d %s / %s" % [k, String.num_scientific(m[0] / maxf(m[1], 1e-9)),
						String.num_scientific(m[2] / maxf(m[3], 1e-9))])
		print("render_check: world volume cold start light rates (cells / ms, column cells / ms): %s" % ", ".join(parts))
		var bake := PackedStringArray()
		for k: int in range(CloudWorldVolumeLayout.LEVELS - 1, -1, -1):
			bake.append("L%d %s" % [k, String.num_scientific(schedule.bake_level_taps_per_ms[k])])
		print("render_check: world volume calibrated bake rates (taps / ms): %s" % ", ".join(bake))
		var cost := PackedStringArray()
		for k: int in range(CloudWorldVolumeLayout.LEVELS - 1, -1, -1):
			cost.append("L%d %.3f + %s / %s" % [k, schedule.light_latency_ms[k],
					String.num_scientific(schedule.light_cells_per_ms[k]),
					String.num_scientific(schedule.column_cells_per_ms[k])])
		print("render_check: world volume planned light cost (latency ms + cells / ms / column cells / ms): %s"
				% ", ".join(cost))
	return waiting


## The cloud view cache's convergence (the cache on, the clouds shown): true while tiles are still filled (until
## FRAMES_TO_SETTLE frames without one; TERRAIN_MAX_FRAMES at most). Records from the planner's last restart the time
## to level 0 / the full level over the view's sample rays and to the whole cube, the main-thread schedule time per
## frame (FrameProfiler section cloud_view_cache) and the measured fill intervals.
func _cache_waiting(world: World) -> bool:
	var clouds := world.clouds
	var planner := clouds.view_cache_planner
	planner.wait()  # its state is read below (the run started last frame; normally done long ago)
	var frame_sections := FrameProfiler.take_frame()
	_last_section_ms = int(frame_sections.get(&"cloud_view_cache", 0)) / 1000.0
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


## The A / B check shared by --ab_view_cache, --ab_reference and --edge_check (world volume plan Task 7, N5: the shared
## code moved out of --ab_view_cache, which Task 13 deletes): the candidate's cloud image (the current settings: the
## cache, or the world volume), then the reference's (_ab_to_reference: --ab_view_cache the per-frame march at divisor
## 1; --ab_reference / --edge_check the reference path - world_volume off, view_cache off, divisor 1, the grain check's
## REFERENCE_STEP_FACTOR x steps, ruling C23), both box-filtered over 2 x 2 pixels, the per-block luminance error
## |candidate - reference| / max(|reference|, AB_ERROR_FLOOR x the mean) - median <= AB_MEDIAN_TOLERANCE over all
## blocks, and the p99 over the blocks whose rays miss the ground and every other scene geometry <= the calibration's:
## the reference against itself with the view turned by AB_SHIFT_PX in four directions, the max over them (the same
## metric; the full image's p99 and T RMS printed). --edge_check (_edge_metrics): the edge pixels' (alpha 0.05-0.95)
## band width and alpha high-pass energy of the candidate against the reference's: at most EDGE_WIDTH_MAX / at least
## EDGE_ENERGY_MIN x the reference's (its own change under the turns printed). Images: the two frames and the error
## (x 10) next to --out. True when done (a failure: exit code 1).
func _ab_frames(world: World) -> bool:
	var effect := world.clouds.effect
	var label := "ab_view_cache" if _args.has("ab_view_cache") else "ab_reference"
	if _ab_wait < 0:
		_ab_cache = effect.output.get_layer_data(0)
		if _args.has("ab_view_cache"):
			var camera: Vector3 = ShaderGlobals.value(&"atmosphere_camera_hi") \
					+ ShaderGlobals.value(&"atmosphere_camera_lo")
			_ab_anchor_m = camera.distance_to(world.clouds.view_cache_planner.anchor_earth_m)
			_save_viewport_png(_out_path("_cache"))
		else:
			_save_viewport_png(_out_path("_volume"))
		_ab_to_reference(world)
		_ab_wait = FRAMES_TO_SETTLE
		return false
	if _ab_wait > 0:
		_ab_wait -= 1
		return false
	if not world.clouds.light_cache.settled() and _frames < TERRAIN_MAX_FRAMES:
		return false  # the reference's light cache (its Sun light) complete first
	if _ab_march != null:
		# The calibration: the reference against itself with the view turned by AB_SHIFT_PX (two point samplings of
		# the same clouds) - the bound of the candidate's p99 is the MAX over the four turns (+-x azimuth, +-y
		# elevation; controller ruling: one turn under-samples the reference's variance).
		var turned := effect.output.get_layer_data(0)
		var shifted := _cloud_compare(turned, _ab_march, _ab_ground)
		print("render_check: %s calibration turn %d (%s): the reference against itself turned by " % [label, _ab_turn,
				AB_TURNS[_ab_turn]] + "%.1f px, 2 x 2 blocks: median %.3f %%, p99 %.3f %% without scene "
				% [AB_SHIFT_PX, shifted["median"] * 100.0, shifted["p99_sky"] * 100.0]
				+ "geometry (all: p99 %.3f %%)" % (shifted["p99_all"] * 100.0))
		_ab_calibration_p99 = maxf(_ab_calibration_p99, shifted["p99_sky"])
		if _args.has("edge_check"):
			var e := _edge_metrics(turned)
			for key in ["width", "energy"]:
				_edge_turn_change[key] = maxf(float(_edge_turn_change.get(key, 0.0)),
						absf(e[key] - _edge_reference[key]) / maxf(_edge_reference[key], 1.0e-9))
		_ab_turn += 1
		if _ab_turn < AB_TURNS.size():
			_ab_turn_view(world)
			return false
		if _args.has("ab_view_cache") or _args.has("ab_reference"):
			var ok: bool = _ab_result["median"] <= AB_MEDIAN_TOLERANCE \
					and _ab_result["p99_sky"] <= _ab_calibration_p99
			print("render_check: %s %s: median %.3f %% (bound %.1f %%), p99 without scene geometry %.3f %% "
					% [label, "PASS" if ok else "FAIL", _ab_result["median"] * 100.0, AB_MEDIAN_TOLERANCE * 100.0,
					_ab_result["p99_sky"] * 100.0] + "(bound: the calibration's max over %d turns %.3f %%)"
					% [AB_TURNS.size(), _ab_calibration_p99 * 100.0])
			_check_failed = _check_failed or not ok
		if _args.has("edge_check"):
			_edge_verdict()
		return true
	var march := effect.output.get_layer_data(0)
	_save_viewport_png(_out_path("_march" if _args.has("ab_view_cache") else "_reference"))
	if march.get_size() != _ab_cache.get_size():
		print("render_check: %s FAIL: image sizes differ (%s / %s)" % [label, _ab_cache.get_size(),
				march.get_size()])
		_check_failed = true
		return true
	var ground := _geometry_blocks(effect.debug_centre_data("depth"), march.get_size())
	if ground.is_empty():
		print("render_check: %s FAIL: the reference's scene distances are not the image's size (%s)"
				% [label, effect.centre_low])
		_check_failed = true
		return true
	var r := _cloud_compare(_ab_cache, march, ground)
	_ab_result = r
	print("render_check: %s: %s vs the reference at divisor 1, 2 x 2 blocks: luminance error median %.3f %%, "
			% [label, "cache" if _args.has("ab_view_cache") else "world volume", r["median"] * 100.0]
			+ "p99 %.3f %% over %d blocks without scene geometry (all: " % [r["p99_sky"] * 100.0, r["sky_blocks"]]
			+ "p99 %.3f %%), worst %.1f %%, T RMS %.5f, mean T %.4f / %.4f (mean luminance %.4f)" % [r["p99_all"]
			* 100.0, r["worst"] * 100.0, r["t_rms"], r["t_mean_a"], r["t_mean_b"], r["mean"]]
			+ ("; the camera %.2f m from the anchor" % _ab_anchor_m if _args.has("ab_view_cache") else ""))
	(r["picture"] as Image).save_png(_out_path("_ab_error"))
	if _args.has("edge_check"):
		_edge_candidate = _edge_metrics(_ab_cache)
		_edge_reference = _edge_metrics(march)
	_ab_march = march
	_ab_ground = ground
	_ab_base_angles = Vector2(world.camera.azimuth, world.camera.elevation)
	_ab_turn_view(world)
	return false


## Switches the frame to the A / B reference (_ab_frames).
func _ab_to_reference(world: World) -> void:
	var preset: Dictionary = CloudLayer.PRESETS[world.clouds.quality]
	world.clouds.view_cache = false
	world.clouds.effect.set_quality(1, float(preset["temporal_weight"]), float(preset["clip_gamma"]))
	if _args.has("ab_view_cache"):
		return
	world.clouds.world_volume = false
	ShaderGlobals.set_value(&"cloud_steps", int(preset["steps"]) * AB_REFERENCE_STEP_FACTOR)
	ShaderGlobals.set_value(&"cloud_max_steps", int(preset["max_steps"]) * AB_REFERENCE_STEP_FACTOR)


## Turns the view by AB_SHIFT_PX eye pixels in the direction of the current calibration turn (from the base view).
func _ab_turn_view(world: World) -> void:
	var cam := world.camera
	var shift_rad := AB_SHIFT_PX * 2.0 * tan(deg_to_rad(cam.fov) / 2.0) / float(_ab_march.get_height())
	var direction: Vector2 = AB_TURN_DIRECTIONS[_ab_turn]
	cam.azimuth = _ab_base_angles.x + direction.x * shift_rad
	cam.elevation = _ab_base_angles.y + direction.y * shift_rad
	_ab_wait = 2


## --edge_check: the edge pixels of a cloud image (alpha = 1 - T in 0.05-0.95, EDGE_ALPHA) - their count per alpha 0.5
## crossing between horizontal and vertical neighbours (the edge band's mean width, pixels) and the mean |Laplacian| of
## alpha over them (the edges' high-pass energy).
static func _edge_metrics(image: Image) -> Dictionary:
	var w := image.get_width()
	var h := image.get_height()
	var alpha := PackedFloat32Array()
	alpha.resize(w * h)
	for y in range(h):
		for x in range(w):
			alpha[y * w + x] = 1.0 - clampf(image.get_pixel(x, y).a, 0.0, 1.0)
	var edges := 0
	var crossings := 0
	var energy := 0.0
	for y in range(1, h - 1):
		for x in range(1, w - 1):
			var a := alpha[y * w + x]
			if (a - 0.5) * (alpha[y * w + x + 1] - 0.5) < 0.0:
				crossings += 1
			if (a - 0.5) * (alpha[(y + 1) * w + x] - 0.5) < 0.0:
				crossings += 1
			if a > EDGE_ALPHA.x and a < EDGE_ALPHA.y:
				edges += 1
				energy += absf(4.0 * a - alpha[y * w + x - 1] - alpha[y * w + x + 1] - alpha[(y - 1) * w + x]
						- alpha[(y + 1) * w + x])
	return {"width": float(edges) / maxf(float(crossings), 1.0), "energy": energy / maxf(float(edges), 1.0),
		"edges": edges, "crossings": crossings}


## --edge_check's verdict (ruling R2): the candidate's band width <= EDGE_WIDTH_MAX x the reference's and its energy >=
## EDGE_ENERGY_MIN x the reference's (_ab_frames); the reference's own change under the turns for scale.
func _edge_verdict() -> void:
	var width_ratio: float = _edge_candidate["width"] / maxf(_edge_reference["width"], 1.0e-9)
	var energy_ratio: float = _edge_candidate["energy"] / maxf(_edge_reference["energy"], 1.0e-9)
	var ok := width_ratio <= EDGE_WIDTH_MAX and energy_ratio >= EDGE_ENERGY_MIN
	print(("render_check: edge_check %s: width %.4f vs %.4f px: %.3f x (bound <= %.2f x), energy %.4f vs %.4f: %.3f x "
			+ "(bound >= %.2f x); the reference's own change under the turns: width %.1f %%, energy %.1f %%; edge pixels "
			+ "%d / %d, crossings %d / %d") % ["PASS" if ok else "FAIL", _edge_candidate["width"],
			_edge_reference["width"], width_ratio, EDGE_WIDTH_MAX, _edge_candidate["energy"], _edge_reference["energy"],
			energy_ratio, EDGE_ENERGY_MIN, float(_edge_turn_change.get("width", 0.0)) * 100.0,
			float(_edge_turn_change.get("energy", 0.0)) * 100.0, _edge_candidate["edges"], _edge_reference["edges"],
			_edge_candidate["crossings"], _edge_reference["crossings"]])
	_check_failed = _check_failed or not ok


## --front_check=W,S (world volume plan Task 7, C13; header): the volume's run, then the reference path's from the same
## instant (the clock jumped back, its light cache settled first), each over S s of clock at W x in FRONT_FRAME_S frames
## (the clock advanced without a jump); per frame the FRONT_BLOCK_PX block means of the cloud image's luminance. The
## verdict compares the frame-to-frame changes (_front_report). True when done.
func _front_frames(world: World) -> bool:
	var parts: PackedStringArray = String(_args["front_check"]).split(",")
	var warp := float(parts[0])
	var frames := int(round(float(parts[1]) / FRONT_FRAME_S))
	if _front_phase < 0:
		_front_phase = 0
		_front_frame = 0
		_front_t0 = world.clock.tdb_s
	if _front_frame < 0:  # the reference waits for its light cache at the start instant
		_front_frame = _front_frame + 1 if world.clouds.light_cache.settled() else -FRAMES_TO_SETTLE
		return false
	if _front_frame > 0:  # the image of the instant set a frame before
		var image := world.clouds.effect.output.get_layer_data(0)
		_front_row_blocks = image.get_width() / FRONT_BLOCK_PX
		_front_blocks[_front_phase].append(_front_block_means(image))
		_front_alpha[_front_phase].append(1.0 - _mean_t(image))
		if _front_phase == 0:
			_front_sun_deg.append(_sun_elevation_deg(world))
	if _front_frame >= frames:
		if _front_phase == 1:
			return _front_report(warp)
		_front_phase = 1
		_front_frame = -FRAMES_TO_SETTLE
		world.clouds.world_volume = false
		world.clouds.view_cache = false
		world.clock.set_time(_front_t0)
		return false
	_front_frame += 1
	world.clock.tdb_s = _front_t0 + warp * FRONT_FRAME_S * _front_frame
	return false


## The block means (FRONT_BLOCK_PX square, luminance of the premultiplied cloud image) of an image.
static func _front_block_means(image: Image) -> PackedFloat32Array:
	var blocks := image.get_size() / FRONT_BLOCK_PX
	var means := PackedFloat32Array()
	means.resize(blocks.x * blocks.y)
	for by in range(blocks.y):
		for bx in range(blocks.x):
			var sum := 0.0
			for j in range(0, FRONT_BLOCK_PX, 2):  # every other pixel: the block's mean to 1/4 of its pixels
				for i in range(0, FRONT_BLOCK_PX, 2):
					var c := image.get_pixel(bx * FRONT_BLOCK_PX + i, by * FRONT_BLOCK_PX + j)
					sum += 0.2126 * c.r + 0.7152 * c.g + 0.0722 * c.b
			means[by * blocks.x + bx] = sum / float(FRONT_BLOCK_PX * FRONT_BLOCK_PX / 4)
	return means


## The mean of an image's .a (the cloud image's background transmittance T).
static func _mean_t(image: Image) -> float:
	var sum := 0.0
	var count := 0
	for y in range(0, image.get_height(), 4):
		for x in range(0, image.get_width(), 4):
			sum += image.get_pixel(x, y).a
			count += 1
	return sum / maxf(float(count), 1.0)


## The Sun's elevation at the eye (deg).
static func _sun_elevation_deg(world: World) -> float:
	var eye := world.camera.position_gcrf_m
	var up := world.camera.site_up_gcrf if world.camera.mode == OrbitCamera.Mode.GROUND else eye.normalized()
	var to_sun := (world.sun_apparent_from_earth_m - eye).normalized()
	return rad_to_deg(asin(clampf(to_sun.dot(up), -1.0, 1.0)))


## --front_check's verdict: per frame D = (volume change - reference change) per block; the step = the largest |D|
## difference between horizontally / vertically neighbouring blocks over the frame's mean radiance (the reference's),
## its max over the frames <= FRONT_TOLERANCE; no "black" frame (FRONT_BLACK_SHARE); the lag = the frame offset that
## best aligns the two runs' mean changes, as Sun angle. True (done).
func _front_report(warp: float) -> bool:
	var vol: Array = _front_blocks[0]
	var ref: Array = _front_blocks[1]
	var n := mini(vol.size(), ref.size())
	var worst := 0.0
	var worst_frame := -1
	var black := 0
	var mean_changes := [PackedFloat32Array(), PackedFloat32Array()]
	for f in range(1, n):
		var a0: PackedFloat32Array = vol[f - 1]
		var a1: PackedFloat32Array = vol[f]
		var b0: PackedFloat32Array = ref[f - 1]
		var b1: PackedFloat32Array = ref[f]
		var mean := 0.0
		for k in range(b1.size()):
			mean += absf(b1[k]) / b1.size()
		var d := PackedFloat32Array()
		d.resize(a1.size())
		var mv := 0.0
		var mr := 0.0
		for k in range(a1.size()):
			d[k] = (a1[k] - a0[k]) - (b1[k] - b0[k])
			mv += (a1[k] - a0[k]) / a1.size()
			mr += (b1[k] - b0[k]) / a1.size()
		mean_changes[0].append(mv)
		mean_changes[1].append(mr)
		var row := _front_row_blocks
		for k in range(d.size()):
			var x := k % row
			if x + 1 < row and k + 1 < d.size():
				var s := absf(d[k + 1] - d[k]) / maxf(mean, 1.0e-6)
				if s > worst:
					worst = s
					worst_frame = f
			if k + row < d.size():
				var s2 := absf(d[k + row] - d[k]) / maxf(mean, 1.0e-6)
				if s2 > worst:
					worst = s2
					worst_frame = f
		if float(_front_alpha[0][f]) < FRONT_BLACK_SHARE * float(_front_alpha[1][f]):
			black += 1
	# lag: the offset (frames, 0 .. 1 s) minimising the squared difference of the mean-change series
	var best := 0
	var best_err := INF
	for lag in range(0, mini(45, n / 2)):
		var err := 0.0
		for f in range(lag, mean_changes[0].size()):
			err += pow(mean_changes[0][f] - mean_changes[1][f - lag], 2.0)
		err /= maxf(float(mean_changes[0].size() - lag), 1.0)
		if err < best_err:
			best_err = err
			best = lag
	var sun_rate := (_front_sun_deg[_front_sun_deg.size() - 1] - _front_sun_deg[0]) \
			/ maxf(float(_front_sun_deg.size() - 1), 1.0)
	var ok := worst <= FRONT_TOLERANCE and black == 0
	print("front_check: max step %.3f %% lag %.4f deg (%s: %d frames at %.0f x, worst at frame %d, %d black frames, "
			% [worst * 100.0, absf(best * sun_rate), "PASS" if ok else "FAIL", n, warp, worst_frame, black]
			+ "Sun %.3f -> %.3f deg)" % [_front_sun_deg[0], _front_sun_deg[_front_sun_deg.size() - 1]])
	_check_failed = _check_failed or not ok
	return true


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
		_save_viewport_png(_out_path("_fly_%05.1f_march" % (_fly_t - _fly_end
				+ float(_args.get("frames_for", "30")))))
		var annulus := _near_annulus(clouds.effect.debug_centre_data("depth"), clouds.effect.centre_low,
				march.get_size())
		var cache_alpha := _mean_alpha(_fly_cache_image, annulus)
		var march_alpha := _mean_alpha(march, annulus)
		_fly_samples.append(Vector4(_fly_t, annulus.size(), cache_alpha, march_alpha))
		# The field's nearest cloud is a lower bound of the vertical gap to a cloudy layer (CloudLayer.nearest_cloud_m).
		var in_layer: bool = clouds.nearest_cloud_m <= CloudLayer.near_radius_m()
		_fly_in_layer.append(in_layer)
		_fly_sample_phase.append(_fly_phase(world))
		var since := _stats(_fly_gpu_ms.slice(_fly_sample_gpu_from))
		_fly_sample_gpu_from = _fly_gpu_ms.size()
		print("render_check: fly sample t %.1f s: annulus %d pixels, mean alpha cache %.4f, march %.4f (diff %.4f), "
				% [_fly_t - _fly_end + float(_args.get("frames_for", "30")), annulus.size(), cache_alpha, march_alpha,
				cache_alpha - march_alpha] + "near shell %s, nearest cloud %.0f m%s, camera %.0f m above the "
				% [_fly_near_sample, clouds.nearest_cloud_m, " (in layer)" if in_layer else "",
				ShaderGlobals.value(&"cloud_camera_height_m")] + "ellipsoid, re-anchors %d, evicted %d; frames since "
				% [clouds.view_cache_planner.reanchors, clouds.view_cache_planner.evicted_tiles]
				+ "GPU max %.2f ms, covering %s, "
				% [since["max"], clouds.view_cache_planner.last_covering]
				+ "budget %.2f ms (march %.2f ms predicted, rest median %.2f / max %.2f ms)"
				% [clouds.cache_budget_ms(), clouds.cache_march_ms(), _stats(clouds._view_cache_rest_ms)["median"],
				_stats(clouds._view_cache_rest_ms)["max"]])
		clouds.view_cache = true
		_fly_gpu_skip = FLY_GPU_LAG_FRAMES
		return false
	if _fly_t >= _fly_next_sample - 1.0e-6 and _fly_cache_image == null:
		# The sample: this frame's cache image (drawn last frame at this instant), then the march's at the same instant.
		_fly_cache_image = clouds.effect.output.get_layer_data(0)
		_fly_near_sample = clouds.effect.near_shell_frames > _fly_near_at_sample
		_save_viewport_png(_out_path("_fly_%05.1f" % (_fly_t - _fly_end
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
	_fly_section_ms.append(_last_section_ms)
	if _fly_gpu_skip > 0:
		_fly_gpu_skip -= 1  # the measured GPU time is the previous frame's: the sample's last reference march
	else:
		_fly_gpu_ms.append(RenderingServer.viewport_get_measured_render_time_gpu(root.get_viewport_rid()))
		_fly_gpu_phase.append(_fly_phase(world))
		var target_ms := float(CloudLayer.PRESETS[clouds.quality]["frame_target_ms"])
		if _args.has("gpu_passes") and _fly_gpu_ms.back() > target_ms:
			_print_slow_frame(_fly_gpu_ms.back())
	_fly_near_at_sample = clouds.effect.near_shell_frames
	_fly_t += FLY_FRAME_S
	_advance(sim, _fly_t)
	world.clock.tdb_s = _fly_t  # the clock's own advance (set_time is a jump: the view cache would restart)
	if _args.has("watch_from_site"):
		_aim_at_vehicle(world)
	return false


## --frames_every's verdict: every sample with annulus pixels within FLY_ALPHA_TOLERANCE (FLY_ALPHA_IN_LAYER_TOLERANCE
## with the camera within a cloud layer's extent +- R_near), and the flight frames' GPU p99 within the VR preset's
## frame target.
func _fly_report() -> bool:
	var worst := [0.0, 0.0]  # outside a layer's extent, within it
	var counted := [0, 0]
	for i in range(_fly_samples.size()):
		var sample := _fly_samples[i]
		if sample.y >= FLY_MIN_ANNULUS:
			var k := 1 if _fly_in_layer[i] else 0
			counted[k] += 1
			worst[k] = maxf(worst[k], absf(sample.z - sample.w))
	var gpu := _stats(_fly_gpu_ms)
	var target := float(CloudLayer.PRESETS[(_main.get_node("World") as World).clouds.quality]["frame_target_ms"])
	var frame_ok: bool = target <= 0.0 or gpu["p99"] <= target
	var ok: bool = counted[0] + counted[1] > 0 and worst[0] <= FLY_ALPHA_TOLERANCE \
			and worst[1] <= FLY_ALPHA_IN_LAYER_TOLERANCE and frame_ok
	print("render_check: frames_every %s: %d samples (%d with >= %d annulus pixels: %d outside a cloud layer's extent "
			% ["PASS" if ok else "FAIL", _fly_samples.size(), counted[0] + counted[1], FLY_MIN_ANNULUS, counted[0]]
			+ "+- R_near, %d within), mean alpha over the near shell's boundary within %.3f / %.3f of the march's " \
			% [counted[1], FLY_ALPHA_TOLERANCE, FLY_ALPHA_IN_LAYER_TOLERANCE] + "(worst %.4f / %.4f); flight frames %d "
			% [worst[0], worst[1], gpu["n"]] + "(near shell in %d): GPU median %.2f / p99 %.2f / max %.2f ms (frame "
			% [_fly_near_frames, gpu["median"], gpu["p99"], gpu["max"]] + "target %.1f ms)" % target)
	for phase in range(FLY_PHASES.size()):
		var phase_gpu: Array[float] = []
		for i in range(_fly_gpu_ms.size()):
			if _fly_gpu_phase[i] == phase:
				phase_gpu.append(_fly_gpu_ms[i])
		var phase_worst := 0.0
		var phase_counted := 0
		for i in range(_fly_samples.size()):
			if _fly_sample_phase[i] == phase and _fly_samples[i].y >= FLY_MIN_ANNULUS:
				phase_counted += 1
				phase_worst = maxf(phase_worst, absf(_fly_samples[i].z - _fly_samples[i].w))
		var phase_stats := _stats(phase_gpu)
		print("render_check: frames_every phase %s: %d frames, GPU median %.2f / p99 %.2f / max %.2f ms; %d samples "
				% [FLY_PHASES[phase], phase_stats["n"], phase_stats["median"], phase_stats["p99"], phase_stats["max"],
				phase_counted] + "with an annulus, worst alpha difference %.4f" % phase_worst)
	var section := _stats(_fly_section_ms)
	print("render_check: frames_every main thread (section cloud_view_cache: the nearest cloud, the plan's hand-over) "
			+ "over %d flight frames: median %.3f / p99 %.3f / max %.3f ms" % [section["n"], section["median"],
			section["p99"], section["max"]])
	_check_failed = _check_failed or not ok
	return true


## The flight's phase now (FLY_PHASES): on the pad, in a cloud layer's extent +- R_near, or outside it.
func _fly_phase(world: World) -> int:
	var sim := world.flight.sim
	var altitude_m: float = sim.telemetry(sim.controlled_index())["altitude_m"]
	if is_nan(_fly_start_altitude_m):
		_fly_start_altitude_m = altitude_m
	if altitude_m - _fly_start_altitude_m <= FLY_PAD_RISE_M:
		return 0
	return 1 if world.clouds.nearest_cloud_m <= CloudLayer.near_radius_m() else 2


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
	var t_a := 0.0
	var t_b := 0.0
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
			t_a += ca.a
			t_b += cb.a
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
		"t_mean_a": t_a / n, "t_mean_b": t_b / n,
		"p99_sky": sky[int(0.99 * (sky.size() - 1))] if sky.size() > 0 else 0.0, "sky_blocks": sky.size(),
		"t_rms": sqrt(t_sq / n), "mean": mean, "picture": picture}


## --out with a suffix before its extension (the A / B images).
func _out_path(suffix: String) -> String:
	var out: String = _args.get("out", ProjectSettings.globalize_path("res://").path_join(
			"../screenshots/render_check.png").simplify_path())
	DirAccess.make_dir_recursive_absolute(out.get_base_dir())
	return out.get_basename() + suffix + "." + out.get_extension()


## --frames_every with --gpu_passes: a flight frame over the frame target - its GPU timestamp intervals over 0.5 ms
## (the last frame whose timestamps came back), largest first.
func _print_slow_frame(gpu_ms: float) -> void:
	var rd := RenderingServer.get_rendering_device()
	var parts: Array[Vector2] = []
	var names := {}
	for i in range(1, rd.get_captured_timestamps_count()):
		var ms := float(rd.get_captured_timestamp_gpu_time(i) - rd.get_captured_timestamp_gpu_time(i - 1)) / 1.0e6
		if ms > 0.5:
			names[parts.size()] = rd.get_captured_timestamp_name(i - 1)
			parts.append(Vector2(ms, parts.size()))
	parts.sort()
	parts.reverse()
	var text := ""
	for p in parts:
		text += " %s %.2f" % [names[int(p.y)], p.x]
	print("render_check: slow flight frame t %.2f s: GPU %.2f ms;%s" % [_fly_t - _fly_end
			+ float(_args.get("frames_for", "30")), gpu_ms, text])


func _record_gpu_passes() -> void:
	var rd := RenderingServer.get_rendering_device()
	var frame := {}
	for i in range(1, rd.get_captured_timestamps_count()):
		var name := rd.get_captured_timestamp_name(i - 1)
		if name.begins_with(CloudWorldVolume.RUN_STAMP):
			continue  # the world volume's run id (an empty interval)
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


## --spike=a|b|ref (world volume plan Task 2, spike S; throw-away, removed in Task 13): CloudSpike's far tier (a: the
## procedural density, b: the bake) or the sanity check's reference (today's march clipped to [R_near, end]), the
## per-frame march (no view cache), the occupancy's band from a --weather=slab:<bottom>,<top>,..., --spike_scale=0.5|1.0
## the far tier's resolution per axis (the cloud pass's divisor 2 / 1). false (exit 1) on an unknown value.
func _start_spike() -> bool:
	var value: String = _args["spike"]
	if not value in ["a", "b", "ref"]:
		_refuse("render_check: unknown --spike=%s (a, b, ref)" % value)
		return false
	CloudSpike.mode = {"a": CloudSpike.Mode.PROCEDURAL, "b": CloudSpike.Mode.BAKED, "ref": CloudSpike.Mode.OFF}[value]
	CloudSpike.reference_clip = value == "ref"
	_args["no_view_cache"] = ""
	var scale: String = _args.get("spike_scale", "0.5")
	if not scale in ["0.5", "1.0"]:
		_refuse("render_check: unknown --spike_scale=%s (0.5, 1.0)" % scale)
		return false
	_args["cloud_divisor"] = "2" if scale == "0.5" else "1"
	# Round 2 (Task 2b): --spike_levers=<1,2,4,s,t> the step rule's levers (1 the bake cell's max bounds Delta tau,
	# 2 sphere tracing on the bake cells, 4 one analytic step per bake cell, s with 4: keep the structure term, t with
	# 4: the trilinear bake at the step's middle), --spike_stop=<T>
	# the termination's scaled transmittance (lever 3; default today's 0.01).
	var bits := {"1": 1, "2": 2, "4": 8, "s": 16, "t": 32}
	for lever: String in String(_args.get("spike_levers", "")).split(",", false):
		if not bits.has(lever):
			_refuse("render_check: unknown --spike_levers entry %s (1, 2, 4, s, t)" % lever)
			return false
		CloudSpike.levers |= bits[lever]
	CloudSpike.stop_transmittance = float(_args.get("spike_stop", "0.0"))
	var weather: String = _args.get("weather", "")
	if weather.begins_with("slab:"):
		var p := weather.trim_prefix("slab:").split(",")
		CloudSpike.band_m = Vector2(float(p[0]), float(p[1]))
	return true


## --spike: the centre view's march image (the far tier's colour + T) - mean alpha and mean luminance (Rec. 709) over
## all its pixels, the sanity check's quantities.
func _print_centre_stats(world: World) -> void:
	var effect: CloudEffect = world.clouds.effect
	var data := effect.debug_centre_data("color")
	var low := effect.centre_low
	var count := low.x * low.y
	if data.size() < 8 * count or count == 0:
		print("render_check: spike centre: no data")
		return
	var alpha := 0.0
	var luminance := 0.0
	for i in range(count):
		var o := 8 * i
		luminance += 0.2126 * data.decode_half(o) + 0.7152 * data.decode_half(o + 2) + 0.0722 * data.decode_half(o + 4)
		alpha += 1.0 - data.decode_half(o + 6)
	print("render_check: spike centre %dx%d mean alpha %.4f mean luminance %s (%s)" % [low.x, low.y, alpha / count,
			luminance / count, _args["spike"]])
	if CloudSpike.plan_report != "":
		print("render_check: " + CloudSpike.plan_report)


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
	_fly_t = world.clock.tdb_s  # --frames_every flies on from here
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
