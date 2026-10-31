# Distance hand-over of the Pad B tower (LatticeBuilder: members -> merged mesh -> lattice shader). Needs a GPU (no
# --headless) and data/models/starbase_olp2 (tools/structures/build_models.py).
# godot --path game --script res://tests/tower_handover_check.gd -- [--out=<dir>] [--steps=1.05] [--quick] [--tiers_only] [--dump_tiers]
# 1. Dolly: the camera backs away from the tower from 50 m to 5 km (ratio --steps per step, aimed at the tower's middle),
#    per step the mean luminance of the tower's rectangle (30 m .. 140 m, the lattice and what shows through it; the
#    table also lists the tower's own pixels, the frame with the tower differing from the one without it, and their
#    count). A hand-over band is judged by the second difference of that luminance, |L_i - (L_i-1 + L_i+1) / 2| / L_i:
#    the dolly's own smooth trend (size, haze) cancels, a pop does not. Pass: <= 2 % in the windows around both range
#    boundaries.
# 2. Shimmer: still camera, a head turn in 0.25 px steps; mean absolute frame-to-frame pixel change over the tower's
#    rectangle with the hand-over, divided by the same with the unreduced geometry (every member, no ranges) at the same
#    distances. Pass: <= 1.10 (the hand-over adds at most 10 % flicker to what the raw geometry has).
# 3. Member width at the start of its fade: >= 1 px (brace at the end of the near range, leg at the end of the mid range).
# Images and the table go to --out (default screenshots/pad-b/handover; git-ignored).
extends SceneTree

const SITE_LAT_DEG := 25.99686
const SITE_LON_DEG := -97.15394
const SITE_HEIGHT_M := -21.11
const CAMERA_AZIMUTH_DEG := 135.0  ## the camera lies this far from the pad (clockwise from north), looking back at the tower
const TOWER_AZIMUTH_DEG := 315.0
const EYE_HEIGHT_M := 15.0
const START_UTC := "2026-10-12T14:10:00"
const FOV_DEG := 60.0
const FIRST_M := 50.0
const LAST_M := 5000.0
const FRAMES_TO_SETTLE := 5
const TERRAIN_MAX_FRAMES := 600
const MASK_THRESHOLD := 0.02
const BAND_WINDOWS := [[0.8, 1.25, "near -> mid", LatticeBuilder.NEAR_END_M],
		[0.8, 1.25, "mid -> far", LatticeBuilder.MID_END_M]]
const MAX_JUMP := 0.02
const TIER_NAMES := ["members", "merged", "shader"]
## [tier a, tier b, distances] compared alone.
const TIER_PAIRS := [[0, 1, [250.0, 330.0, 380.0, 420.0, 450.0, 480.0]], [1, 2, [900.0, 1000.0, 1100.0, 1300.0]]]
const SHIMMER_DISTANCES := [280.0, 330.0, 600.0, 1000.0, 1500.0, 2500.0, 4000.0]
const SHIMMER_STEPS := 12
const SHIMMER_STEP_PX := 0.25
const MAX_SHIMMER_RATIO := 1.10
const IMAGE_DISTANCES := [100.0, 300.0, 420.0, 700.0, 1150.0, 1300.0, 2500.0, 5000.0]

var _main: Node
var _world: World
var _out := ""
var _failures := 0
var _dump := false
var _focal_px := 1.0


func _initialize() -> void:
	var args := {}
	for arg in OS.get_cmdline_user_args():
		var parts := arg.trim_prefix("--").split("=", true, 1)
		args[parts[0]] = parts[1] if parts.size() > 1 else ""
	_out = args.get("out", ProjectSettings.globalize_path("res://").path_join("../screenshots/pad-b/handover")
			.simplify_path())
	DirAccess.make_dir_recursive_absolute(_out)
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)
	_dump = args.has("dump_tiers")
	_run(float(args.get("steps", "1.05")), args.has("quick"), args.has("tiers_only"))


func _run(ratio: float, quick: bool, tiers_only: bool) -> void:
	await process_frame
	_world = _main.get_node("World")
	_world.clock.warp = 0.0
	_world.weather.enabled = false
	TimePanel.show_frame_rate = false
	_world.clock.set_time(_world.core.utc_to_tdb_s(START_UTC))
	_world.atmosphere.eye_adaptation_enabled = false
	_world.set_focus(World.FOCUS_EARTH)
	_world.camera.fov = FOV_DEG
	var tower := _world.towers[0] if not _world.towers.is_empty() else null
	if tower == null or tower.get_node_or_null("Lattice") == null:
		print("tower_handover_check: no Pad B lattice (models missing?): nothing to check")
		quit(1)
		return
	_focal_px = 0.5 * root.size.y / tan(deg_to_rad(0.5 * FOV_DEG))
	var lattice: Node3D = tower.get_node("Lattice")

	if tiers_only:
		await _report_tiers(tower, lattice)
		quit(0 if _failures == 0 else 1)
		return
	var distances: Array[float] = []
	var d := FIRST_M
	while d <= LAST_M * 1.001:
		distances.append(d)
		d *= (ratio if not quick else ratio * ratio)
	var luminance: Array[float] = []
	var csv := "distance_m,tower_pixels,tower_pixel_luminance,rectangle_luminance\n"
	for distance in distances:
		await _place(tower, distance)
		var rect := _tower_rect(tower, distance)
		var with_tower := await _grab()
		tower.visible = false
		await _frames(3)
		var without := await _grab()
		tower.visible = true
		var stats := _tower_pixels(with_tower.get_region(rect), without.get_region(rect))
		luminance.append(stats[2])
		csv += "%.1f,%d,%.5f,%.5f\n" % [distance, stats[1], stats[0], stats[2]]
		for shown: float in IMAGE_DISTANCES:
			if absf(distance / shown - 1.0) < 0.03:
				with_tower.save_png(_out.path_join("dolly_%04d.png" % int(round(distance))))
	FileAccess.open(_out.path_join("dolly.csv"), FileAccess.WRITE).store_string(csv)
	_report_dolly(distances, luminance)

	await _report_tiers(tower, lattice)
	await _report_shimmer(tower, lattice)
	_report_member_widths(lattice)
	print("TOWER HANDOVER CHECK ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


## Moves the camera `distance` m from the tower along the pad -> camera azimuth and aims at the tower's middle.
func _place(tower: Node3D, distance: float) -> void:
	var azimuth := deg_to_rad(CAMERA_AZIMUTH_DEG)
	var along_m := distance - 30.0  # the tower stands 30 m from the pad centre on the opposite side
	var lat := SITE_LAT_DEG + rad_to_deg(along_m * cos(azimuth) / World.EARTH_SEMI_MAJOR_AXIS_M)
	var lon := SITE_LON_DEG + rad_to_deg(along_m * sin(azimuth)
			/ (World.EARTH_SEMI_MAJOR_AXIS_M * cos(deg_to_rad(SITE_LAT_DEG))))
	var samples: Dictionary = _world.core.surface_samples("earth", PackedFloat64Array([lat]), PackedFloat64Array([lon]))
	var height: float = samples["height_m"][0] if bool(samples["present"]) else SITE_HEIGHT_M
	_world.set_ground_site({"id": "custom", "name": "custom", "body": "earth", "latitude_deg": lat,
			"longitude_deg": lon, "height_m": height, "eye_height_m": EYE_HEIGHT_M})
	_world.update_world()
	_world.camera.fov = FOV_DEG
	_world.camera.azimuth = deg_to_rad(TOWER_AZIMUTH_DEG)
	_world.camera.elevation = atan2(70.0 - EYE_HEIGHT_M, distance)
	_world.atmosphere.snap_adaptation()
	await _settle()


func _settle() -> void:
	var idle := 0
	var frames := 0
	while idle < FRAMES_TO_SETTLE and frames < TERRAIN_MAX_FRAMES:
		await process_frame
		frames += 1
		var loading: int = _world.earth_terrain.stats["nodes_loading"]
		idle = idle + 1 if loading == 0 and not _world.atmosphere.loading else 0


func _frames(count: int) -> void:
	for i in range(count):
		await process_frame


func _grab() -> Image:
	await _frames(2)
	return root.get_viewport().get_texture().get_image()


## [mean luminance of the pixels that differ between the frames with and without the tower, their number, mean
## luminance of the whole rectangle (tower and what shows through it)].
func _tower_pixels(with_tower: Image, without: Image) -> Array:
	var sum := 0.0
	var count := 0
	var rect_sum := 0.0
	for y in range(0, with_tower.get_height()):
		for x in range(0, with_tower.get_width()):
			var a := with_tower.get_pixel(x, y)
			var b := without.get_pixel(x, y)
			var luminance := 0.2126 * a.r + 0.7152 * a.g + 0.0722 * a.b
			rect_sum += luminance
			if maxf(maxf(absf(a.r - b.r), absf(a.g - b.g)), absf(a.b - b.b)) > MASK_THRESHOLD:
				sum += luminance
				count += 1
	return [sum / maxi(count, 1), count, rect_sum / maxi(with_tower.get_width() * with_tower.get_height(), 1)]


func _report_dolly(distances: Array[float], luminance: Array[float]) -> void:
	var jumps: Array[float] = []
	for i in range(distances.size()):
		if i == 0 or i == distances.size() - 1 or luminance[i] <= 0.0:
			jumps.append(0.0)
		else:
			jumps.append(absf(luminance[i] - 0.5 * (luminance[i - 1] + luminance[i + 1])) / luminance[i])
	var outside: Array[float] = []
	for i in range(distances.size()):
		var inside := false
		for band: Array in BAND_WINDOWS:
			inside = inside or (distances[i] >= band[0] * band[3] and distances[i] <= band[1] * band[3])
		if not inside:
			outside.append(jumps[i])
	outside.sort()
	print("dolly: %d steps, second difference outside the windows: median %.2f %%, max %.2f %%" % [
			distances.size(), 100.0 * outside[outside.size() / 2], 100.0 * outside[-1]])
	for band: Array in BAND_WINDOWS:
		var worst := 0.0
		for i in range(distances.size()):
			if distances[i] >= band[0] * band[3] and distances[i] <= band[1] * band[3]:
				worst = maxf(worst, jumps[i])
		print("dolly: %s (%.0f m +%d %% / -%d %%): max luminance jump %.2f %%" % [band[2], band[3],
				roundi(100.0 * (band[1] - 1.0)), roundi(100.0 * (1.0 - band[0])), 100.0 * worst])
		_check(worst <= MAX_JUMP, "%s: luminance jump %.2f %% <= %.0f %%" % [band[2], 100.0 * worst, 100.0 * MAX_JUMP])


## Each tier shown alone at distances around its boundary: the rectangle's mean luminance must agree (<= MAX_JUMP).
func _report_tiers(tower: Node3D, lattice: Node3D) -> void:
	for pair: Array in TIER_PAIRS:
		for distance: float in pair[2]:
			await _place(tower, distance)
			var rect := _tower_rect(tower, distance)
			var values: Array[float] = []
			for tier: int in [pair[0], pair[1]]:
				_force_tier(lattice, tier)
				await _frames(3)
				var image := root.get_viewport().get_texture().get_image().get_region(rect)
				values.append(_mean_luminance(image))
			if _dump:
				_force_tier(lattice, pair[1])
				await _frames(3)
				root.get_viewport().get_texture().get_image().get_region(rect).save_png(
						_out.path_join("tier_%s_%d.png" % [TIER_NAMES[pair[1]], int(distance)]))
				_force_tier(lattice, pair[0])
				await _frames(3)
				root.get_viewport().get_texture().get_image().get_region(rect).save_png(
						_out.path_join("tier_%s_%d.png" % [TIER_NAMES[pair[0]], int(distance)]))
			_force_tier(lattice, -1)
			await _frames(3)
			var as_built := _mean_luminance(root.get_viewport().get_texture().get_image().get_region(rect))
			var difference := absf(values[0] - values[1]) / maxf(values[0], 1.0e-6)
			print("tiers: %s at %.0f m: %.4f vs %.4f, difference %.2f %% (as built %.4f)" % [TIER_NAMES[pair[0]] + " / "
					+ TIER_NAMES[pair[1]], distance, values[0], values[1], 100.0 * difference, as_built])
			_check(difference <= MAX_JUMP, "%s at %.0f m agree within %.0f %%" % [TIER_NAMES[pair[0]] + " / "
					+ TIER_NAMES[pair[1]], distance, 100.0 * MAX_JUMP])


func _mean_luminance(image: Image) -> float:
	var sum := 0.0
	for y in range(image.get_height()):
		for x in range(image.get_width()):
			var p := image.get_pixel(x, y)
			sum += 0.2126 * p.r + 0.7152 * p.g + 0.0722 * p.b
	return sum / maxi(image.get_width() * image.get_height(), 1)


## tier 0: members (MultiMeshes), 1: merged mesh, 2: lattice shader, each alone without ranges; -1: as built.
func _force_tier(lattice: Node3D, tier: int) -> void:
	for section in lattice.get_children():
		for node in section.get_children():
			var geometry := node as GeometryInstance3D
			if geometry == null or node.name == "shadow_proxy":
				continue
			var own := 0 if node is MultiMeshInstance3D else (1 if node.name == "mid" else 2)
			if tier < 0:
				geometry.visible = true
				LatticeBuilder.set_range(geometry, [0.0, LatticeBuilder.NEAR_END_M, LatticeBuilder.MID_END_M][own],
						[LatticeBuilder.NEAR_END_M, LatticeBuilder.MID_END_M, 0.0][own])
			else:
				geometry.visible = own == tier
				LatticeBuilder.set_range(geometry, 0.0, 0.0)
	return


func _report_shimmer(tower: Node3D, lattice: Node3D) -> void:
	for distance: float in SHIMMER_DISTANCES:
		await _place(tower, distance)
		var with_ranges := await _shimmer(tower, distance)
		_set_hand_over(lattice, false)
		await _frames(2)
		var unreduced := await _shimmer(tower, distance)
		_set_hand_over(lattice, true)
		await _frames(2)
		var ratio := with_ranges / maxf(unreduced, 1.0e-6)
		print("shimmer: %5.0f m: %.5f with the hand-over, %.5f unreduced, ratio %.3f" % [distance, with_ranges,
				unreduced, ratio])
		_check(ratio <= MAX_SHIMMER_RATIO, "shimmer ratio at %.0f m: %.3f <= %.2f" % [distance, ratio,
				MAX_SHIMMER_RATIO])


## Mean absolute frame-to-frame change of the luminance in the tower's rectangle over a head turn.
func _shimmer(tower: Node3D, distance: float) -> float:
	var rect := _tower_rect(tower, distance)
	var start := _world.camera.azimuth
	var previous: Image
	var total := 0.0
	var pairs := 0
	for k in range(SHIMMER_STEPS + 1):
		_world.camera.azimuth = start + k * SHIMMER_STEP_PX / _focal_px
		await _frames(3)
		var image := root.get_viewport().get_texture().get_image().get_region(rect)
		if previous != null:
			total += _mean_abs_difference(previous, image)
			pairs += 1
		previous = image
	_world.camera.azimuth = start
	return total / maxi(pairs, 1)


func _mean_abs_difference(a: Image, b: Image) -> float:
	var sum := 0.0
	for y in range(a.get_height()):
		for x in range(a.get_width()):
			var p := a.get_pixel(x, y)
			var q := b.get_pixel(x, y)
			sum += absf((0.2126 * p.r + 0.7152 * p.g + 0.0722 * p.b) - (0.2126 * q.r + 0.7152 * q.g + 0.0722 * q.b))
	return sum / maxi(a.get_width() * a.get_height(), 1)


## Image rectangle around the tower's axis between 30 m and 140 m (projected), a margin of 40 % of its width each side.
func _tower_rect(tower: Node3D, distance: float) -> Rect2i:
	var lattice: Node3D = tower.get_node("Lattice")
	var base: Vector3 = (lattice.get_node("section_0") as Node3D).global_position
	var top: Vector3 = (lattice.get_node("section_11") as Node3D).global_position
	var up: Vector3 = (top - base).normalized()
	var camera := _world.camera
	var low := camera.unproject_position(base + up * 30.0)
	var high := camera.unproject_position(base + up * 140.0)
	var half_width := 0.5 * 12.0 * _focal_px / distance * 1.4 + 3.0
	var x0 := clampi(int(minf(low.x, high.x) - half_width), 0, root.size.x - 2)
	var x1 := clampi(int(maxf(low.x, high.x) + half_width), x0 + 1, root.size.x)
	var y0 := clampi(int(minf(low.y, high.y)), 0, root.size.y - 2)
	var y1 := clampi(int(maxf(low.y, high.y)), y0 + 1, root.size.y)
	return Rect2i(x0, y0, x1 - x0, y1 - y0)


## true: the visibility ranges as built; false: every member (near MultiMeshes unlimited, mid and far hidden).
func _set_hand_over(lattice: Node3D, enabled: bool) -> void:
	for section in lattice.get_children():
		for node in section.get_children():
			var geometry := node as GeometryInstance3D
			if geometry == null or node.name == "shadow_proxy":
				continue
			if node is MultiMeshInstance3D:
				geometry.visibility_range_end = LatticeBuilder.NEAR_END_M if enabled else 0.0
				geometry.visibility_range_end_margin = LatticeBuilder.FADE_MARGIN_M if enabled else 0.0
			else:
				geometry.visible = enabled


## The width in pixels of a brace at the start of the near range's fade and of a leg at the start of the mid range's.
func _report_member_widths(lattice: Node3D) -> void:
	var detail := TomlLite.load_file(Paths.config_dir().path_join(StructureModels.DETAIL_FILE))
	var brace: float = detail["profiles.brace"]["size_m"]
	var leg: float = detail["profiles.leg"]["size_m"]
	var brace_px := _focal_px * brace / (LatticeBuilder.NEAR_END_M - LatticeBuilder.FADE_MARGIN_M)
	var leg_px := _focal_px * leg / (LatticeBuilder.MID_END_M - LatticeBuilder.FADE_MARGIN_M)
	print("members: focal %.0f px: brace %.2f px where its fade starts (%.0f m), leg %.2f px where its fade starts (%.0f m)" % [
			_focal_px, brace_px, LatticeBuilder.NEAR_END_M - LatticeBuilder.FADE_MARGIN_M, leg_px,
			LatticeBuilder.MID_END_M - LatticeBuilder.FADE_MARGIN_M])
	_check(brace_px >= 1.0, "brace >= 1 px at the start of its fade")
	_check(leg_px >= 1.0, "leg >= 1 px at the start of its fade")


func _check(condition: bool, message: String) -> void:
	print(("  ok: " if condition else "  FAIL: "), message)
	if not condition:
		_failures += 1
