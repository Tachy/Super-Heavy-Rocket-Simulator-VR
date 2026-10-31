# Hull shimmer check (Starship prototype plan, task 3 + final fix): the ship's tiled belly and its bare-steel lee side in a REAL
# flight session (ship return, entry state, in the vacuum of 60 km: no aerial perspective between camera and hull) seen from 5
# to 500 m (distance to the surface) with the VR focal length (FOCAL_PX 2900 px: FOV 17.6 deg on the 900 px window, scale 1;
# the eye's pixel grid of the Pimax at 5294 x 4250 px per eye), exposure fixed. Per surface and distance, over a physical patch
# of the surface (PATCH_M, capped at PATCH_MAX_PX), STEPS frames of a 0.3 px head motion (the pattern slides over the pixel
# grid), each rendered at scale 1 (A) and at scale 2 (B: the downsampled supersampled reference, the filtered truth):
#  - L: mean luminance of A (linear display values); tiles: ratio max / min of L over the distances <= RATIO_LIMIT (the detail
#    must fade to its mean, not brighten or darken with distance);
#  - temporal: mean per-pixel |dL| of A between consecutive motion frames / L; gated against the reference's own value
#    (genuine signal motion): temporal(A) <= TEMPORAL_FACTOR x temporal(B) + TEMPORAL_FLOOR (moire / crawling pixels);
#  - error: per-pixel RMS of (A - B) / L (the variance against the downsampled reference), gated <= ERROR_LIMIT.
# godot --path game --script res://tests/tile_shimmer_check.gd -- [--surface=tiles|steel] (one surface per run: each needs
# its own mission start; default tiles). Exit code 1 when a gate fails.
extends SceneTree

const DISTANCES_M := [5.0, 10.0, 20.0, 40.0, 80.0, 160.0, 320.0, 500.0]
## Surface -> [mission args, body index, station (m), camera direction in the body frame, sun_turn_deg (NAN: use the direction)].
## tiles: the ship's belly (-Z) at the return's entry state; steel: the booster barrel in the stack at T+100 s, seen from the
## Sun's side (lit; the ship's lee side at the entry state is in shade, where relative metrics only read noise).
const SURFACES := {
	"tiles": [{"mission": "starship_leo", "ship_return": "560", "t_plus": "5200", "utc": "2026-10-31T13:10:00"}, 1, 20.0,
			Vector3(0.0, 0.0, -1.0), NAN],
	"steel": [{"mission": "starship_leo", "t_plus": "100", "utc": "2026-10-31T19:00:00"}, 0, 30.0,
			Vector3(0.0, 0.0, -1.0), 0.0],
}
const WINDOW_HEIGHT_PX := 900.0
const FOCAL_PX := 2900.0
const PATCH_M := 2.0  ## > the ring pitch (1.8 m): every patch from 16 m on holds a weld bead
const PATCH_MIN_PX := 4
const PATCH_MAX_PX := 360
const STEPS := 8
const STEP_RAD := 0.3 / FOCAL_PX  ## 0.3 px of head motion per frame at any distance
const FRAMES_SETTLE := 6
const FRAMES_CAPTURE := 3
const REFERENCE_SCALE := 2.0
const RATIO_LIMIT := 1.10
# [E] Steel: a metal has no diffuse term, its mean follows what the sky's reflection and the dents show at each distance (the
# sky's gradient across the barrel). Measured 1.134 (2026-10-31); beyond this limit a mip level or the map changes brightness.
const STEEL_RATIO_LIMIT := 1.18
const TEMPORAL_FACTOR := 1.5
const TEMPORAL_FLOOR := 0.004
const ERROR_LIMIT := 0.15  ## [E] measured band of the filtered shader 0.00-0.13 (the old one 0.375); see vehicle-models.md
## The camera aims at the axis; the hull is its radius closer (the ship's barrel, config diameter 9 m).
const HULL_RADIUS_M := 4.5

var _args := {}
var _main: Node
var _world: World
var _rig: VehicleRig
var _sim: FlightSim
var _started := false
var _srgb_to_linear: Array[float] = []
var _surface := "tiles"
var _results: Array[Dictionary] = []


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		var parts := arg.trim_prefix("--").split("=", true, 1)
		_args[parts[0]] = parts[1] if parts.size() > 1 else ""
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)
	_surface = _args.get("surface", "tiles")
	root.scaling_3d_mode = Viewport.SCALING_3D_MODE_BILINEAR
	for i in 256:
		_srgb_to_linear.append(Color(i / 255.0, i / 255.0, i / 255.0).srgb_to_linear().r)


func _process(_delta: float) -> bool:
	_world = _main.get_node("World")
	_world.clock.warp = 0.0
	_world.weather.enabled = false
	if not _started and Engine.get_process_frames() > 10 and not _world.atmosphere.loading:
		_started = true
		TimePanel.show_frame_rate = false
		_world.atmosphere.eye_adaptation_enabled = false
		var rig_args: Dictionary = SURFACES[_surface][0].duplicate()
		rig_args.merge(_args, true)
		_rig = VehicleRig.new(_world, rig_args)
		_sim = _rig.start()
		_rig.fly_to_target(_sim)
		_rig.hide_ui(_main)
		_run()
	return false


func _n(count: int) -> void:
	for i in count:
		await process_frame


func _run() -> void:
	await _n(10)
	var fov_deg := rad_to_deg(2.0 * atan(0.5 * WINDOW_HEIGHT_PX / FOCAL_PX))
	var spec: Array = SURFACES[_surface]
	var body: int = spec[1]
	var station: float = spec[2]
	for distance: float in DISTANCES_M:
		var offset: Vector3 = spec[3] * (distance + HULL_RADIUS_M)
		_set_scale(1.0)
		_rig.place(_sim, body, station, offset, fov_deg, spec[4])
		await _n(FRAMES_SETTLE)
		var frames_a: Array[PackedFloat32Array] = []
		var frames_b: Array[PackedFloat32Array] = []
		for step in STEPS:
			_rig.place(_sim, body, station, offset, fov_deg, spec[4], step * STEP_RAD)
			_set_scale(1.0)
			await _n(FRAMES_CAPTURE)
			var image_a := root.get_viewport().get_texture().get_image()
			frames_a.append(_patch(image_a, distance))
			_set_scale(REFERENCE_SCALE)
			await _n(FRAMES_CAPTURE)
			frames_b.append(_patch(root.get_viewport().get_texture().get_image(), distance))
			if step == 0 and distance == DISTANCES_M[0]:
				image_a.save_png(ProjectSettings.globalize_path("res://").path_join(
						"../screenshots/vehicles/shimmer_%s_5m.png" % _surface))
		_results.append(_stats(_surface, distance, frames_a, frames_b))
	_set_scale(1.0)
	_report()


## The render scale; with it the hull shader's footprint (`footprint_scale`): the supersampled reference then evaluates the
## SAME filtered pattern as the 1x frame (a finer footprint would fade fewer details), only sampled more densely, so A - B
## is the sampling error alone (the shimmer).
func _set_scale(scale: float) -> void:
	root.scaling_3d_scale = scale
	for material: ShaderMaterial in VehicleModels.load_for(VehicleModels.PROTOTYPE_VEHICLE_ID)["materials"].values():
		material.set_shader_parameter("footprint_scale", scale)
		if _args.has("set"):  # diagnostics: --set=name:value sets a float shader parameter of the hull materials
			var kv := String(_args["set"]).split(":")
			material.set_shader_parameter(kv[0], float(kv[1]))


## Linear luminance of the central physical patch (row-major).
func _patch(image: Image, distance_m: float) -> PackedFloat32Array:
	image.convert(Image.FORMAT_RGBA8)
	var half := int(clampf(0.5 * PATCH_M * FOCAL_PX / distance_m, 0.5 * PATCH_MIN_PX, 0.5 * PATCH_MAX_PX))
	var width := image.get_width()
	var cx := width / 2
	var cy := image.get_height() / 2
	var data := image.get_data()
	var out := PackedFloat32Array()
	for y in range(cy - half, cy + half):
		var row := 4 * (y * width + cx - half)
		for x in 2 * half:
			var i := row + 4 * x
			out.append(0.2126 * _srgb_to_linear[data[i]] + 0.7152 * _srgb_to_linear[data[i + 1]]
					+ 0.0722 * _srgb_to_linear[data[i + 2]])
	return out


func _stats(surface: String, distance: float, frames_a: Array[PackedFloat32Array],
		frames_b: Array[PackedFloat32Array]) -> Dictionary:
	var count := frames_a[0].size()
	var lum := 0.0
	var temporal_a := 0.0
	var temporal_b := 0.0
	var error := 0.0
	for k in frames_a.size():
		var a := frames_a[k]
		var b := frames_b[k]
		for i in count:
			lum += a[i]
			error += (a[i] - b[i]) * (a[i] - b[i])
			if k > 0:
				temporal_a += absf(a[i] - frames_a[k - 1][i])
				temporal_b += absf(b[i] - frames_b[k - 1][i])
	var samples := float(count * frames_a.size())
	lum /= samples
	var pairs := float(count * (frames_a.size() - 1))
	return {"surface": surface, "distance_m": distance, "luminance": lum, "patch_px": int(sqrt(count)),
			"temporal": temporal_a / pairs / maxf(lum, 1e-9), "temporal_ref": temporal_b / pairs / maxf(lum, 1e-9),
			"error": sqrt(error / samples) / maxf(lum, 1e-9)}


func _report() -> void:
	var failed := false
	var lo := INF
	var hi := 0.0
	for r: Dictionary in _results:
		var temporal_ok: bool = r["temporal"] <= TEMPORAL_FACTOR * r["temporal_ref"] + TEMPORAL_FLOOR
		var error_ok: bool = r["error"] <= ERROR_LIMIT
		failed = failed or not temporal_ok or not error_ok
		print("tile_shimmer_check: %-5s %4.0f m  patch %3d px  L %.5f  temporal %.4f (ref %.4f) %s  error %.4f %s" % [
				r["surface"], r["distance_m"], r["patch_px"], r["luminance"], r["temporal"], r["temporal_ref"],
				"ok" if temporal_ok else "FAIL", r["error"], "ok" if error_ok else "FAIL"])
		lo = minf(lo, r["luminance"])
		hi = maxf(hi, r["luminance"])
	var ratio := hi / maxf(lo, 1e-9)
	var ratio_limit := RATIO_LIMIT if _surface == "tiles" else STEEL_RATIO_LIMIT
	var ratio_ok := ratio <= ratio_limit
	failed = failed or not ratio_ok
	print("tile_shimmer_check: %s luminance ratio max/min %.3f (limit %.2f) -> %s" % [_surface, ratio, ratio_limit,
			"PASS" if ratio_ok else "FAIL"])
	print("tile_shimmer_check: %s %s" % [_surface, "FAIL" if failed else "PASS"])
	quit(1 if failed else 0)
