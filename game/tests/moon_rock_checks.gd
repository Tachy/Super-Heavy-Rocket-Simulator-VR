class_name MoonRockChecks
## The drawn-rock checks of moon_surface_check.gd (moon procedural surface plan Task 11; MoonRockLayer):
## - transforms: a node's MultiMesh buffers against the core's rocks, found independently cell by cell
##   (StarsimCore.moon_rocks): the same ids, centres within 1 mm, axes and template data exact to float32;
## - patch_rocks: the core's rocks of a square around a point laid onto a flat test patch (the radiance gate);
## - camera_jump: a streamed (worker) quadtree with the rock layer, converged 2 m above the site, then the camera
##   moved 10 km: the rock layer's main-thread share per frame (attach at uploads, detach at releases; the rock lists
##   are built on the workers) stays under 5 ms while the new site streams in; the quadtree's whole update time is
##   printed beside it (and without rocks with --jump_baseline: the quadtree's own selection / dispatch / upload cost).

## The brief's position tolerance (m).
const POSITION_TOLERANCE_M := 0.001
## The brief's main-thread limit per frame during a camera jump (ms).
const JUMP_MAIN_THREAD_MS := 5.0
const JUMP_DISTANCE_M := 10000.0
## [E] Pixel angle of the checks' node lists (the stereo mock's ~5e-4 rad per pixel).
const CHECK_PIXEL_ANGLE := 5.0e-4
const JUMP_FRAME_MS := 11  ## a 90 Hz frame between the updates (the workers run meanwhile)
const JUMP_MAX_FRAMES := 20000


## Node buffers of MoonRockLayer.build_node against the core's per-cell rocks for nodes at the site.
static func transforms(core: StarsimCore, layer: MoonRockLayer, site: Vector2, check: Callable) -> void:
	layer.pixel_angle_rad = CHECK_PIXEL_ANGLE
	var heights := HeightSource.new()
	heights.body = "moon"
	heights.core = core
	var dir := CubeSphere.geodetic_direction(site.x, site.y)
	for level in [18, 20, 22]:
		var key := CubeSphere.key_at(dir, level)
		var built := TerrainNodeBuilder.build(key, heights, null, HeightSource.MOON_RADIUS_M, Vector4i(-1, -1, -1, -1),
				PackedFloat64Array(), layer)
		var center: Vector3 = built["center_fixed"]
		var rocks: Dictionary = built["rocks"]
		var min_d: float = rocks["node_min_m"]
		var expected := _cell_rocks(core, key, min_d)
		var drawn := {}  # id -> [lod list, instance floats]
		var ids: PackedInt64Array = rocks.get("ids", PackedInt64Array())
		var buffers: Array = rocks.get("buffers", [])
		var counts: PackedInt32Array = rocks.get("counts", PackedInt32Array())
		var chunk_counts: Array = rocks.get("chunk_counts", [])
		var in_some_lod := 0
		var consistent := true
		for lod in counts.size():
			var listed := 0
			for c in (chunk_counts[lod] as PackedInt32Array).size():
				var buffer: PackedFloat32Array = (buffers[lod] as Array)[c]
				var n_chunk: int = (chunk_counts[lod] as PackedInt32Array)[c]
				listed += n_chunk
				consistent = consistent and n_chunk <= MoonRockLayer.BUCKET_MAX
				for k in n_chunk:
					var f := buffer.slice(k * MoonRockLayer.FLOATS_PER_INSTANCE, (k + 1) * MoonRockLayer.FLOATS_PER_INSTANCE)
					var key_id := _instance_key(f)
					if drawn.has(key_id):
						consistent = consistent and drawn[key_id] == f
					else:
						drawn[key_id] = f
			consistent = consistent and listed == counts[lod]
		# Match by the instance's centre (the buffer carries no id): every core rock's instance at its centre.
		var worst_m := 0.0
		var worst_axis := 0.0
		var matched := 0
		var unmatched := 0
		for id: int in expected:
			var r: Dictionary = expected[id]
			var local: Vector3 = (r["center"] as Vector3) - center
			var best := INF
			var best_f := PackedFloat32Array()
			for key_id: Vector3i in _near_keys(local):
				if drawn.has(key_id):
					var f: PackedFloat32Array = drawn[key_id]
					var d := (Vector3(f[11], f[3], f[7]) - local).length()
					if d < best:
						best = d
						best_f = f
			if best_f.is_empty():
				unmatched += 1
				continue
			matched += 1
			worst_m = maxf(worst_m, best)
			# Columns (render rows -> fixed): a-axis x a, b-axis x b, up x c.
			for c in 3:
				var column := Vector3(best_f[8 + c], best_f[c], best_f[4 + c])
				var want: Vector3 = r["columns"][c]
				worst_axis = maxf(worst_axis, (column - want).length() / want.length())
			consistent = consistent and absf(best_f[16] - float(r["diameter"])) <= 1.0e-6 * float(r["diameter"]) \
					and int(best_f[19]) == int(r["template"])
		check.call(ids.size() == expected.size() and unmatched == 0 and matched == expected.size(),
				"rock list of node %s (D >= %.3f m): %d drawn, %d from the core cell by cell, %d matched"
				% [key, min_d, ids.size(), expected.size(), matched])
		check.call(_same_ids(ids, expected), "rock ids of node %s equal the core's per-cell rocks" % key)
		check.call(worst_m < POSITION_TOLERANCE_M and worst_axis < 1.0e-5 and consistent,
				"rock transforms of node %s: centres within %.4f mm (1 mm), axes %.7f relative, D / template exact, LOD copies identical"
				% [key, worst_m * 1000.0, worst_axis])
		check.call(drawn.size() == ids.size(), "every listed rock in at least one LOD buffer (%d of %d)"
				% [drawn.size(), ids.size()])


static func _instance_key(f: PackedFloat32Array) -> Vector3i:
	return Vector3i(roundi(f[11] * 1000.0), roundi(f[3] * 1000.0), roundi(f[7] * 1000.0))


## The rounded-mm keys around a node-local centre (render rows hold the fixed (y, z, x) components: f[11] is fixed x).
static func _near_keys(local_fixed: Vector3) -> Array[Vector3i]:
	var out: Array[Vector3i] = []
	var base := Vector3i(roundi(local_fixed.x * 1000.0), roundi(local_fixed.y * 1000.0), roundi(local_fixed.z * 1000.0))
	for dx in [-1, 0, 1]:
		for dy in [-1, 0, 1]:
			for dz in [-1, 0, 1]:
				out.append(base + Vector3i(dx, dy, dz))
	return out


static func _same_ids(ids: PackedInt64Array, expected: Dictionary) -> bool:
	if ids.size() != expected.size():
		return false
	for id in ids:
		if not expected.has(id):
			return false
	return true


## The node's rocks found cell by cell through StarsimCore.moon_rocks: the octaves' cells below the node (finer
## levels) and its ancestor cells (coarser levels, filtered to bases inside the node). id -> {center, columns,
## diameter, template}.
static func _cell_rocks(core: StarsimCore, key: Vector4i, min_d: float) -> Dictionary:
	var out := {}
	var stats: Dictionary = core.moon_surface_statistics()
	for level in range(maxi(key.y - 8, 0), 26):
		var cells: Array[Vector4i] = []
		if level >= key.y:
			var n := 1 << (level - key.y)
			if n * n > 70000:
				continue
			for j in n:
				for i in n:
					cells.append(Vector4i(key.x, level, key.z * n + i, key.w * n + j))
		else:
			cells.append(Vector4i(key.x, level, key.z >> (key.y - level), key.w >> (key.y - level)))
		for cell in cells:
			var data: Dictionary = core.moon_rocks(cell.x, cell.y, cell.z, cell.w, min_d)
			var ids: PackedInt64Array = data["ids"]
			var base: PackedFloat64Array = data["base_fixed_m"]
			var basis: PackedFloat64Array = data["basis"]
			var axes: PackedFloat64Array = data["axes_m"]
			for r in ids.size():
				var point := Vector3(base[3 * r], base[3 * r + 1], base[3 * r + 2])
				if level < key.y and CubeSphere.key_at(point, key.y) != key:
					continue
				var up := Vector3(basis[9 * r + 6], basis[9 * r + 7], basis[9 * r + 8])
				out[ids[r]] = {"center": point - up * (data["burial_m"] as PackedFloat64Array)[r],
						"columns": [Vector3(basis[9 * r], basis[9 * r + 1], basis[9 * r + 2]) * axes[3 * r],
						Vector3(basis[9 * r + 3], basis[9 * r + 4], basis[9 * r + 5]) * axes[3 * r + 1], up * axes[3 * r + 2]],
						"diameter": (data["diameter_m"] as PackedFloat64Array)[r],
						"template": (data["template"] as PackedInt32Array)[r]}
	return out


## The core's rocks (D >= min_d) whose base lies in the square of edge edge_m around the direction, laid onto a flat
## patch (ground y = 0, up +Y): one MultiMesh buffer in MoonRockLayer's layout, the template exposed heights from
## top_z. {buffer, count, cover (the rocks' plan-view area pi D^2 / 4 over the square)}.
static func patch_rocks(core: StarsimCore, top_z: PackedFloat32Array, center_dir: Vector3, edge_m: float,
		min_d: float) -> Dictionary:
	var c := center_dir.normalized()
	var e1 := Vector3(0, 0, 1).cross(c).normalized()
	var e2 := c.cross(e1)
	var radius := HeightSource.MOON_RADIUS_M
	# Level-17 cells (~20 m) over the square, each asked once.
	var keys := {}
	var steps := int(ceil(edge_m / 5.0)) + 1
	for a in steps + 1:
		for b in steps + 1:
			var x := -0.5 * edge_m - 5.0 + (edge_m + 10.0) * a / steps
			var y := -0.5 * edge_m - 5.0 + (edge_m + 10.0) * b / steps
			keys[CubeSphere.key_at(c + (e1 * x + e2 * y) / radius, 17)] = true
	var buffer := PackedFloat32Array()
	var count := 0
	var area := 0.0
	var diameters := PackedFloat64Array()
	var half := 0.5 * edge_m
	for key: Vector4i in keys:
		var data: Dictionary = core.moon_rocks_in_node(key.x, key.y, key.z, key.w, min_d)
		var ids: PackedInt64Array = data["ids"]
		var base: PackedFloat64Array = data["base_fixed_m"]
		var basis: PackedFloat64Array = data["basis"]
		var axes: PackedFloat64Array = data["axes_m"]
		var diameter: PackedFloat64Array = data["diameter_m"]
		var burial: PackedFloat64Array = data["burial_m"]
		var albedo: PackedFloat64Array = data["albedo"]
		var templates: PackedInt32Array = data["template"]
		for r in ids.size():
			var p := Vector3(base[3 * r], base[3 * r + 1], base[3 * r + 2])
			var offset := p - c * p.dot(c)
			var x := offset.dot(e1)
			var z := -offset.dot(e2)
			if absf(x) > half or absf(z) > half:
				continue
			var cols: Array[Vector3] = []
			for k in 3:
				var v := Vector3(basis[9 * r + 3 * k], basis[9 * r + 3 * k + 1], basis[9 * r + 3 * k + 2]) * axes[3 * r + k]
				cols.append(Vector3(v.dot(e1), v.dot(c), -v.dot(e2)))  # fixed -> patch (X e1, Y up, Z -e2)
			var up: Vector3 = cols[2].normalized()
			var o := Vector3(x, 0.0, z) - up * burial[r]
			buffer.append_array([cols[0].x, cols[1].x, cols[2].x, o.x, cols[0].y, cols[1].y, cols[2].y, o.y,
					cols[0].z, cols[1].z, cols[2].z, o.z, 0.0, 0.0, 0.5, 0.5, diameter[r],
					top_z[templates[r]] - burial[r] / axes[3 * r + 2], albedo[r], templates[r]])
			count += 1
			area += 0.25 * PI * diameter[r] * diameter[r]
			diameters.append(diameter[r])
	return {"buffer": buffer, "count": count, "cover": area / (edge_m * edge_m), "diameters": diameters}


## The camera jump (blocking: the updates are 11 ms apart, the workers build meanwhile). {settle_ms, jump_frames,
## max_update_ms, p99_update_ms, rocks (attached instances after the jump), max_node_ms}.
static func camera_jump(tree: TerrainQuadtree, layer: MoonRockLayer, site_fixed: Vector3, check: Callable) -> void:
	var up := site_fixed.normalized()
	var east := Vector3(0, 0, 1).cross(up).normalized()
	var worst_phases := {}
	var rock_ms: Array[float] = []
	var frames := _stream(tree, site_fixed, [], worst_phases, layer, [])
	var settled_rocks: int = layer.stats["attached_instances"] if layer != null else 0
	var jumped := (up * site_fixed.length() + east * JUMP_DISTANCE_M).normalized() * site_fixed.length()
	var times: Array[float] = []
	worst_phases.clear()
	var jump_frames := _stream(tree, jumped, times, worst_phases, layer, rock_ms)
	rock_ms.sort()
	var rock_worst: float = rock_ms.back() if not rock_ms.is_empty() else 0.0
	var rock_p99: float = rock_ms[mini(rock_ms.size() - 1, int(0.99 * rock_ms.size()))] if not rock_ms.is_empty() else 0.0
	var rock_over := 0
	for ms in rock_ms:
		rock_over += 1 if ms > JUMP_MAIN_THREAD_MS else 0
	times.sort()
	var worst: float = times.back() if not times.is_empty() else 0.0
	var p99: float = times[mini(times.size() - 1, int(0.99 * times.size()))] if not times.is_empty() else 0.0
	var line := "camera jump of %.0f km at 2 m %s: main-thread update max %.2f ms (phases %s), p99 %.2f ms over %d frames (limit %.0f ms)" % [
			JUMP_DISTANCE_M / 1000.0, "with the rock layer" if layer != null else "WITHOUT rocks (baseline)", worst,
			worst_phases, p99, jump_frames, JUMP_MAIN_THREAD_MS]
	if layer == null:
		print("  info  " + line)
		return
	# The rock layer's own main-thread share (attach at uploads, detach at releases) is what Task 11 adds; the
	# quadtree's selection / dispatch / upload cost without rocks is printed by --jump_baseline (pre-existing).
	check.call(rock_worst < JUMP_MAIN_THREAD_MS and frames < JUMP_MAX_FRAMES and jump_frames < JUMP_MAX_FRAMES,
			line + "; the rock layer's share max %.2f ms per frame (gate %.0f ms), p99 %.2f ms, %d frames over, largest single attach %.2f ms (material, multimeshes: %s); pool %d instances (%.0f MB), %d allocations after the prewarm, %d unpooled, peak in use per bucket %s, slowest allocation %.2f ms" % [
			rock_worst, JUMP_MAIN_THREAD_MS, rock_p99, rock_over, float(layer.stats.get("max_attach_ms", 0.0)),
			layer.stats.get("max_attach_parts_ms", []), layer.stats["pool_instances"], layer.stats["pool_mb"],
			layer.stats["allocations"], layer.stats["unpooled"], layer.stats["bucket_peak"], float(layer.stats.get("max_allocate_ms", 0.0))]
			+ "; rock instances %d before, %d after; worker rock lists %d nodes, %.1f ms summed, %.1f ms max per node (%d rocks max)"
			% [settled_rocks, layer.stats["attached_instances"], layer.stats["nodes_built"], layer.stats["build_ms"],
			layer.stats["max_node_ms"], layer.stats["max_node_rocks"]]
			+ "; pool peak %.0f MB (bound %.0f), %d nodes kept statistical at the bound, %d parked, %d trimmed; largest upload per frame %.1f MB (staging pool %d MB)"
			% [float(layer.stats.get("pool_peak_mb", 0.0)), MoonRockLayer.POOL_MAX_MB,
			int(layer.stats.get("over_bound_nodes", 0)), int(layer.stats.get("parked_nodes", 0)),
			int(layer.stats.get("trimmed", 0)), float(layer.stats.get("max_frame_upload_mb", 0.0)),
			int(ProjectSettings.get_setting("rendering/rendering_device/staging_buffer/max_size_mb", 128))])


## Updates every JUMP_FRAME_MS until nothing has loaded for 30 updates; returns the update count, appends each
## update's main-thread time (ms) and keeps the slowest update's phase times in worst_phases.
static func _stream(tree: TerrainQuadtree, camera_fixed: Vector3, times: Array[float], worst_phases: Dictionary,
		layer: MoonRockLayer, rock_ms: Array[float]) -> int:
	var idle := 0
	var worst := -1.0
	for frame in JUMP_MAX_FRAMES:
		var rock_before: float = layer.stats["main_ms"] if layer != null else 0.0
		tree.update_for_camera(camera_fixed, JUMP_FRAME_MS / 1000.0)
		if layer != null:
			rock_ms.append(float(layer.stats["main_ms"]) - rock_before)
		var ms := float(tree.stats["update_ms"])
		times.append(ms)
		if ms > worst:
			worst = ms
			worst_phases.assign(tree.stats["phase_ms"])
		idle = idle + 1 if tree.stats["nodes_loading"] == 0 else 0
		if idle >= 30:
			return frame
		# A frame is drawn between the updates as in the game: the RenderingDevice recycles its staging buffers at a
		# frame. Without it the uploads of hundreds of updates pile up and one upload stalls on the full staging pool
		# (measured: a single 17 ms buffer call per run, on whichever call hit it - terrain mesh, MultiMesh mesh or
		# buffer; Task 11 fix B report).
		RenderingServer.force_draw(false)
		OS.delay_msec(JUMP_FRAME_MS)
	return JUMP_MAX_FRAMES


## The budget control (GPU floor guard): the threshold scale rises smoothly while the GPU frame is over the floor and the
## rocks' own cost could bring it back (no step), holds when the frame without the rocks is already over the floor (no
## wind-up on terrain cost), relaxes back below the floor, stays in [1, SCALE_MAX], ignores an unmeasured frame, and
## does nothing at a frame under the floor (the 9 ms VR target no longer trades rock quality).
static func budget(check: Callable) -> void:
	var floor_ms := MoonRockLayer.GPU_FLOOR_MS
	var scale := 1.0
	var largest_step := 0.0
	for frame in 900:  # 10 s at 90 Hz, 50 % over the floor, the rocks 60 % of the frame
		var next := MoonRockLayer.budget_step(scale, 1.5 * floor_ms, floor_ms, 1.0 / 90.0, 0.9 * floor_ms)
		largest_step = maxf(largest_step, next / scale - 1.0)
		scale = next
	var held := MoonRockLayer.budget_step(scale, 0.0, floor_ms, 1.0 / 90.0, 1.0)
	var terrain := 1.0
	for frame in 900:  # the terrain alone over the floor, the rocks 0.4 ms: no wind-up
		terrain = MoonRockLayer.budget_step(terrain, 1.2 * floor_ms, floor_ms, 1.0 / 90.0, 0.4)
	var nominal := 1.0
	for frame in 900:  # 12 ms, under the floor (over the old 9 ms target): the nominal threshold stays
		nominal = MoonRockLayer.budget_step(nominal, 12.0, floor_ms, 1.0 / 90.0, 3.0)
	var relaxed := scale
	for frame in 9000:
		relaxed = MoonRockLayer.budget_step(relaxed, 0.5 * floor_ms, floor_ms, 1.0 / 90.0, 1.0)
	check.call(scale > 2.0 and scale <= MoonRockLayer.SCALE_MAX and largest_step < 0.01 and held == scale
			and terrain == 1.0 and nominal == 1.0 and relaxed == 1.0,
			"rock budget control (floor %.1f ms): 50 %% over for 10 s -> scale %.2f (largest step %.2f %% per frame), unmeasured frame keeps it, terrain alone over the floor -> %.2f (no wind-up), 12 ms -> %.2f (nominal), back to %.2f after 100 s below"
			% [floor_ms, scale, 100.0 * largest_step, terrain, nominal, relaxed])
