# GroundAlbedoNear (follow-ups of the Moon procedural surface plan, U-4): the imagery's ground albedo around the eye by
# horizontal distance and azimuth, for the radiance cube's ground bounce, read on a worker. Headless, exit code 1 on failure;
# the imagery checks are skipped (reported) without data/imagery.
# godot --headless --path game --script res://tests/ground_albedo_near_test.gd
extends SceneTree

const PAD_LAT_DEG := 25.99677
const PAD_LON_DEG := -97.15804
const METRES_PER_DEGREE := 111132.0

var _failures := 0


func _check(name_: String, ok: bool, detail := "") -> void:
	print("ground_albedo_near_test: %s %s %s" % ["ok  " if ok else "FAIL", name_, detail])
	if not ok:
		_failures += 1


func _luminance(rgb: Vector3) -> float:
	return 0.2126 * rgb.x + 0.7152 * rgb.y + 0.0722 * rgb.z


## A synthetic profile: sector s, ring k holds (0.5 - 0.1 k + 0.01 s) in every channel.
func _synthetic() -> PackedVector3Array:
	var profile := PackedVector3Array()
	for s in range(GroundAlbedoNear.SECTORS):
		for k in range(GroundAlbedoNear.ring_count()):
			var v := 0.5 - 0.1 * k + 0.01 * s
			profile.append(Vector3(v, v, v))
	return profile


func _initialize() -> void:
	# The interpolation rule, without data: cell values held at the rings' mid-distances and the sectors' centres, linear in
	# between (sky.gdshader draws the same rule).
	var p := _synthetic()
	var n := GroundAlbedoNear.ring_count()
	var edges: Array = GroundAlbedoNear.RING_EDGES_M
	var mid := func(k: int) -> float: return 0.5 * (float(edges[k]) + float(edges[k + 1]))
	var sector_rad := TAU / GroundAlbedoNear.SECTORS
	_check("profile: %d sectors x %d rings" % [GroundAlbedoNear.SECTORS, n], p.size() == GroundAlbedoNear.profile_size())
	_check("profile: ring 0 holds inside its mid-distance (north)", GroundAlbedoNear.profile_at(p, 0.5 * mid.call(0), 0.0) == p[0])
	_check("profile: at a mid-distance and a sector centre the cell's value (ring 1, sector 2)",
			GroundAlbedoNear.profile_at(p, mid.call(1), 2.0 * sector_rad).is_equal_approx(p[2 * n + 1]))
	_check("profile: halfway between two mid-distances the mean",
			GroundAlbedoNear.profile_at(p, 0.5 * (mid.call(0) + mid.call(1)), 0.0).is_equal_approx((p[0] + p[1]) * 0.5))
	_check("profile: halfway between two sector centres the mean (ring 1, sectors 3 and 4)",
			GroundAlbedoNear.profile_at(p, mid.call(1), 3.5 * sector_rad).is_equal_approx((p[3 * n + 1] + p[4 * n + 1]) * 0.5))
	_check("profile: the azimuth wraps (between the last sector and north)",
			GroundAlbedoNear.profile_at(p, mid.call(1), -0.5 * sector_rad).is_equal_approx(
			(p[(GroundAlbedoNear.SECTORS - 1) * n + 1] + p[1]) * 0.5))
	_check("profile: the last ring holds beyond its mid-distance",
			GroundAlbedoNear.profile_at(p, float(edges[n]) - 1.0, 0.0) == p[n - 1])
	# The face weighting: a uniform profile gives its value; a face sees mostly the sectors it looks at.
	var uniform := PackedVector3Array()
	uniform.resize(GroundAlbedoNear.profile_size())
	uniform.fill(Vector3(0.2, 0.2, 0.2))
	var global := Vector3(0.1, 0.1, 0.1)
	var face_uniform := GroundAlbedoNear.face_albedo(uniform, 2.0, 0.0, global)
	_check("face albedo: a uniform profile at a low eye gives its value (the global texture's share beyond 1.5 km ~ 0.1 %)",
			absf(face_uniform.x - 0.2) < 0.001, "%.4f" % face_uniform.x)
	var east := uniform.duplicate()
	for k in range(n):
		east[2 * n + k] = Vector3(0.4, 0.4, 0.4)
	var face_east := GroundAlbedoNear.face_albedo(east, 10.0, PI / 2.0, global).x
	var face_west := GroundAlbedoNear.face_albedo(east, 10.0, -PI / 2.0, global).x
	_check("face albedo: a bright east sector lights an east face, not a west face", face_east > 0.22 and absf(face_west - 0.2) < 0.002,
			"east %.4f, west %.4f" % [face_east, face_west])
	_check("refresh distance: half the inner ring at a low eye, half the height higher up",
			GroundAlbedoNear.refresh_distance_m(2.0) == GroundAlbedoNear.REFRESH_MIN_M
			and is_equal_approx(GroundAlbedoNear.refresh_distance_m(400.0), 200.0))
	_check("no imagery: no profile", GroundAlbedoNear.ring_albedo(null, PAD_LAT_DEG, PAD_LON_DEG).is_empty())

	var imagery := ImagerySource.new(Paths.data_dir().path_join("imagery/earth/index.toml"))
	if imagery.is_empty():
		print("ground_albedo_near_test: no data/imagery, imagery checks skipped")
		quit(1 if _failures > 0 else 0)
		return
	var sync_profile := GroundAlbedoNear.ring_albedo(imagery, PAD_LAT_DEG, PAD_LON_DEG)
	_check("one value per cell at the pad", sync_profile.size() == GroundAlbedoNear.profile_size(), "%d cells" % sync_profile.size())
	var plausible := sync_profile.size() == GroundAlbedoNear.profile_size()
	var ring_mean: Array[Vector3] = []
	ring_mean.resize(GroundAlbedoNear.ring_count())
	ring_mean.fill(Vector3.ZERO)
	for i in range(sync_profile.size()):
		var y := _luminance(sync_profile[i])
		plausible = plausible and y > 0.05 and y < 0.3
		ring_mean[i % GroundAlbedoNear.ring_count()] += sync_profile[i]
	for ring in ring_mean:
		plausible = plausible and ring.x > ring.z  # land on the azimuth mean; single sectors over the Gulf or bay are bluer
	_check("every cell is a plausible surface albedo (luminance 0.05 .. 0.30; each ring's azimuth mean red above blue)", plausible,
			"luminance %s" % [Array(sync_profile).map(func(c: Vector3) -> float: return snappedf(_luminance(c), 0.001))])
	_check("outside the imagery's coverage (equator, Atlantic): no profile",
			GroundAlbedoNear.ring_albedo(imagery, 0.0, -30.0).is_empty())

	var near := GroundAlbedoNear.new(imagery)
	var first := near.at(PAD_LAT_DEG, PAD_LON_DEG)
	_check("the first call starts a read and returns its profile only when the worker was fast", near.reads() == 1
			and (first.is_empty() or first == sync_profile))
	var value := near.finish()
	_check("the worker's profile equals the synchronous one", value == sync_profile)
	_check("the read's wall time is recorded", near.last_read_ms() > 0.0, "%.1f ms" % near.last_read_ms())
	near.at(PAD_LAT_DEG + 10.0 / METRES_PER_DEGREE, PAD_LON_DEG)
	_check("10 m on at the ground: no new read", near.reads() == 1)
	near.at(PAD_LAT_DEG + 10.0 / METRES_PER_DEGREE, PAD_LON_DEG, 100.0)
	_check("10 m on at 100 m height: no new read either (refresh 50 m)", near.reads() == 1)
	near.at(PAD_LAT_DEG + 20.0 / METRES_PER_DEGREE, PAD_LON_DEG)
	_check("20 m on at the ground: a new read", near.reads() == 2)
	near.at(PAD_LAT_DEG + 500.0 / METRES_PER_DEGREE, PAD_LON_DEG)
	_check("a read in flight blocks a further one until it is collected", near.reads() == 2)
	near.finish()
	near.at(PAD_LAT_DEG + 500.0 / METRES_PER_DEGREE, PAD_LON_DEG)
	_check("after it is collected the moved eye starts the next read", near.reads() == 3)
	near.finish()
	print("ground_albedo_near_test: %s" % ["PASSED" if _failures == 0 else "FAILED (%d)" % _failures])
	quit(1 if _failures > 0 else 0)
