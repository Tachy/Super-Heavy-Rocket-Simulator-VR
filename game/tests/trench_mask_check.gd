# Terrain mask of the Pad B mount and trench (terrain_earth.gdshader, TowerView._set_trench_mask). Needs a GPU (no
# --headless) and data/models/starbase_olp2 (tools/structures/build_models.py).
# godot --path game --script res://tests/trench_mask_check.gd -- [--out=<dir>] [--distances=2,60,...]
# Per camera distance from the mount, three frames of the same view:
#   A  terrain alone (mask off, mount and trench hidden),
#   B  mask on, mount and trench hidden: the holes the mask cuts show what lies behind them,
#   C  everything as drawn.
# D = the pixels where A and B differ. Checks:
#   1. every D pixel lies on the projected footprint (central rectangle + the two ramps, dilated by DILATE_PX): the mask
#      leaves no hole beyond the site, at the camera distances of every terrain LOD level and at the far seam;
#   2. at least COVER_MIN of the D pixels differ between B and C: the drawn pit and mount fill what the mask removed;
#   3. beyond the mask's range (+ the footprint's size) there is no D pixel (mask off), up to orbit;
#   4. inside the range the mask is visible where the footprint is (at least MIN_HOLE_PX pixels) when the footprint is
#      larger than that on screen.
# Images (C, and D in red over B) and the table go to --out (default screenshots/pad-b/fine).
extends SceneTree

const SITE_LAT_DEG := 25.99686
const SITE_LON_DEG := -97.15394
const SITE_HEIGHT_M := -21.11
const BEARING_DEG := 135.0  ## the camera lies on this bearing from the mount, looking back at it
const START_UTC := "2026-10-12T14:10:00"
const FOV_DEG := 60.0
const FRAMES_TO_SETTLE := 5
const MORPH_FRAMES := 150  ## [E] 2.5 s at 60 Hz
const TERRAIN_MAX_FRAMES := 900
## Camera distances (m) from the mount axis: ground view, terrain LOD levels, the mask's range (20 km), far, orbit.
const DISTANCES := [30.0, 60.0, 150.0, 500.0, 1500.0, 5000.0, 12000.0, 19000.0, 24000.0, 50000.0, 400000.0]
const DIFF_THRESHOLD := 0.02  ## luminance difference (0..1) of two frames at one pixel
const DILATE_PX := 3.0  ## [E] antialiasing and the inset of the mask edge (0.05 m)
const COVER_MIN := 0.98
const MIN_NEIGHBOURS := 4  ## [E] neighbours (of 8) that must differ too
const MIN_HOLE_PX := 40  ## the footprint must be at least this large on screen for check 4
const HALF_ALONG_M := 20.0
const HALF_ACROSS_M := 22.3
const RAMP_HALF_M := 13.0
const RAMP_END_M := 43.7

var _main: Node
var _world: World
var _view: TowerView
var _out := ""
var _failures := 0
var _range_m := 0.0


func _initialize() -> void:
	var args := {}
	for arg in OS.get_cmdline_user_args():
		var parts := arg.trim_prefix("--").split("=", true, 1)
		args[parts[0]] = parts[1] if parts.size() > 1 else ""
	_out = args.get("out", ProjectSettings.globalize_path("res://").path_join("../screenshots/pad-b/fine").simplify_path())
	DirAccess.make_dir_recursive_absolute(_out)
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)
	var distances: Array[float] = []
	if args.has("distances"):
		for token: String in String(args["distances"]).split(","):
			distances.append(float(token))
	else:
		for d: float in DISTANCES:
			distances.append(d)
	_run(distances)


func _run(distances: Array[float]) -> void:
	await process_frame
	_world = _main.get_node("World")
	_world.clock.warp = 0.0
	_world.weather.enabled = false
	TimePanel.show_frame_rate = false
	_world.clock.set_time(_world.core.utc_to_tdb_s(START_UTC))
	_world.atmosphere.eye_adaptation_enabled = false
	_world.set_focus(World.FOCUS_EARTH)
	_view = _world.towers[0] if not _world.towers.is_empty() else null
	if _view == null or not _view.has_models:
		print("trench_mask_check: no Pad B models (build_models.py): nothing to check")
		quit(1)
		return
	_range_m = float(ShaderGlobals.value(&"trench_range_m"))
	print("mask range %.0f m" % _range_m)
	for distance in distances:
		await _check_distance(distance)
	print("TRENCH MASK CHECK ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _place(distance: float) -> void:
	var azimuth := deg_to_rad(BEARING_DEG)
	# far views look down at about 30 deg from a height above the ground
	var horizontal := distance if distance < 20000.0 else distance * 0.87
	var eye := 2.0 if distance < 100.0 else (15.0 if distance < 3000.0 else minf(distance * 0.5, 6.0e5))
	var lat := SITE_LAT_DEG + rad_to_deg(horizontal * cos(azimuth) / World.EARTH_SEMI_MAJOR_AXIS_M)
	var lon := SITE_LON_DEG + rad_to_deg(horizontal * sin(azimuth)
			/ (World.EARTH_SEMI_MAJOR_AXIS_M * cos(deg_to_rad(SITE_LAT_DEG))))
	var samples: Dictionary = _world.core.surface_samples("earth", PackedFloat64Array([lat]), PackedFloat64Array([lon]))
	var height: float = samples["height_m"][0] if bool(samples["present"]) else SITE_HEIGHT_M
	_world.set_ground_site({"id": "custom", "name": "custom", "body": "earth", "latitude_deg": lat,
			"longitude_deg": lon, "height_m": height, "eye_height_m": eye})
	_world.update_world()
	_world.camera.fov = FOV_DEG if distance < 5000.0 else maxf(2.0, rad_to_deg(2.0 * atan(300.0 / horizontal)) * 1.0)
	_world.camera.azimuth = deg_to_rad(BEARING_DEG + 180.0)
	_world.camera.elevation = atan2(10.0 - eye, horizontal)
	_world.atmosphere.snap_adaptation()
	await _settle()
	for i in range(MORPH_FRAMES):  # new terrain nodes fade in (morph floor): wait it out
		await process_frame


func _settle() -> void:
	var idle := 0
	var frames := 0
	while idle < FRAMES_TO_SETTLE and frames < TERRAIN_MAX_FRAMES:
		await process_frame
		frames += 1
		var loading: int = _world.earth_terrain.stats["nodes_loading"]
		idle = idle + 1 if loading == 0 and not _world.atmosphere.loading else 0


func _grab() -> Image:
	for i in range(3):
		await process_frame
	return root.get_viewport().get_texture().get_image()


func _set_mask(on: bool) -> void:
	ShaderGlobals.set_value(&"trench_range_m", _range_m if on else 0.0)


func _set_models(on: bool) -> void:
	_view.get_node("MountModel").visible = on
	_view.get_node("TrenchModel").visible = on


## Projected footprint polygons (screen px), or [] where a corner lies behind the camera.
func _footprint_polygons() -> Array[PackedVector2Array]:
	var rects: Array = [[HALF_ALONG_M, HALF_ACROSS_M, -HALF_ALONG_M, -HALF_ACROSS_M]]
	for sy in [-1.0, 1.0]:
		rects.append([RAMP_HALF_M, sy * RAMP_END_M, -RAMP_HALF_M, sy * HALF_ACROSS_M])
	var polygons: Array[PackedVector2Array] = []
	var camera := _world.camera
	var model := _view.get_node("MountModel/Mount") as Node3D  # glTF axes: X tower, Y across, Z up (the footprint's frame)
	for r: Array in rects:
		var polygon := PackedVector2Array()
		for corner: Vector2 in [Vector2(r[0], r[1]), Vector2(r[2], r[1]), Vector2(r[2], r[3]), Vector2(r[0], r[3])]:
			var site := Vector3(corner.x, corner.y, 0.0)
			var world: Vector3 = model.global_transform * site
			if camera.is_position_behind(world):
				return []
			polygon.append(camera.unproject_position(world))
		polygons.append(polygon)
	return polygons


func _on_footprint(p: Vector2, polygons: Array[PackedVector2Array]) -> bool:
	for polygon in polygons:
		if Geometry2D.is_point_in_polygon(p, polygon):
			return true
		for i in range(polygon.size()):
			var a := polygon[i]
			var b := polygon[(i + 1) % polygon.size()]
			var closest := Geometry2D.get_closest_point_to_segment(p, a, b)
			if closest.distance_to(p) <= DILATE_PX:
				return true
	return false


func _check_distance(distance: float) -> void:
	await _place(distance)
	_set_mask(false)
	_set_models(false)
	var a := await _grab()
	var a2 := await _grab()  # the same frame again: what changes by itself is not the mask
	_set_mask(true)
	var b := await _grab()
	_set_models(true)
	var c := await _grab()
	var polygons := _footprint_polygons()
	var width := a.get_width()
	var height := a.get_height()
	var diff_count := 0
	var outside := 0
	var covered := 0
	var mark := b.duplicate() as Image
	var polygon_px := 0.0
	var outside_box := Rect2()
	for polygon in polygons:
		var bounds := Rect2(polygon[0], Vector2.ZERO)
		for p in polygon:
			bounds = bounds.expand(p)
		polygon_px += polygon_area(polygon)
	# Candidate pixels, then only those with MIN_NEIGHBOURS of their 8 neighbours too: the terrain's own frame-to-frame
	# noise (isolated pixels on the rough ground) is not a hole.
	var flags := PackedByteArray()
	flags.resize(width * height)
	for y in range(height):
		for x in range(width):
			var la := a.get_pixel(x, y).get_luminance()
			var lb := b.get_pixel(x, y).get_luminance()
			if absf(la - lb) > DIFF_THRESHOLD * maxf(la, 0.05) \
		and absf(la - a2.get_pixel(x, y).get_luminance()) <= DIFF_THRESHOLD * maxf(la, 0.05):
				flags[y * width + x] = 1
	for y in range(1, height - 1):
		for x in range(1, width - 1):
			if flags[y * width + x] == 0:
				continue
			var near := 0
			for dy in range(-1, 2):
				for dx in range(-1, 2):
					near += flags[(y + dy) * width + x + dx]
			if near - 1 < MIN_NEIGHBOURS:
				continue
			var lb := b.get_pixel(x, y).get_luminance()
			diff_count += 1
			mark.set_pixel(x, y, Color(1.0, 0.0, 0.0))
			if polygons.is_empty() or not _on_footprint(Vector2(x + 0.5, y + 0.5), polygons):
				outside += 1
				outside_box = outside_box.expand(Vector2(x, y)) if outside > 1 else Rect2(Vector2(x, y), Vector2.ZERO)
			# Per channel: the hazed dark ramp floor can match the hole's luminance while its colour differs.
			var drawn := c.get_pixel(x, y)
			var hole := b.get_pixel(x, y)
			var channel := maxf(absf(drawn.r - hole.r), maxf(absf(drawn.g - hole.g), absf(drawn.b - hole.b)))
			if channel > DIFF_THRESHOLD * maxf(lb, 0.05):
				covered += 1
	if outside > 0:
		print("  outside pixels in %s; footprint polygons %s" % [outside_box, polygons])
	var cover := float(covered) / maxf(float(diff_count), 1.0)
	var site_distance := _site_distance()
	print("distance %.0f m (camera %.0f m from the site): footprint %.0f px, mask holes %d px, outside the footprint %d, "
			% [distance, site_distance, polygon_px, diff_count, outside]
			+ "covered by the model %.1f %%" % (100.0 * cover))
	c.save_png(_out.path_join("mask_%06d_c.png" % int(distance)))
	mark.save_png(_out.path_join("mask_%06d_holes.png" % int(distance)))
	_check(outside == 0, "%.0f m: no hole beyond the footprint (%d px outside)" % [distance, outside])
	if diff_count > 0:
		_check(cover >= COVER_MIN, "%.0f m: the model covers the mask's holes (%.1f %%)" % [distance, 100.0 * cover])
	if site_distance > _range_m + 80.0:
		_check(diff_count == 0, "%.0f m: beyond the mask range no hole (%d px)" % [distance, diff_count])
	elif polygon_px > MIN_HOLE_PX * 4.0:
		_check(diff_count >= MIN_HOLE_PX, "%.0f m: the mask is visible on the footprint (%d px)" % [distance, diff_count])


func polygon_area(polygon: PackedVector2Array) -> float:
	var sum := 0.0
	for i in range(polygon.size()):
		var a := polygon[i]
		var b := polygon[(i + 1) % polygon.size()]
		sum += a.x * b.y - b.x * a.y
	return absf(0.5 * sum)


func _site_distance() -> float:
	return _world.camera.global_position.distance_to(_view.get_node("MountModel").global_position)


func _check(condition: bool, message: String) -> void:
	if condition:
		print("  ok: ", message)
	else:
		print("  FAIL: ", message)
		_failures += 1
