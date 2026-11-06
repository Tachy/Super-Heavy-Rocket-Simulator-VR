# GroundAlbedoNear (follow-ups of the Moon procedural surface plan, U-4): the imagery's ground albedo around the eye by distance,
# for the radiance cube's ground bounce, read on a worker. Headless, exit code 1 on failure; the imagery checks are skipped
# (reported) without data/imagery.
# godot --headless --path game --script res://tests/ground_albedo_near_test.gd
extends SceneTree

const PAD_LAT_DEG := 25.99677
const PAD_LON_DEG := -97.15804

var _failures := 0


func _check(name_: String, ok: bool, detail := "") -> void:
	print("ground_albedo_near_test: %s %s %s" % ["ok  " if ok else "FAIL", name_, detail])
	if not ok:
		_failures += 1


func _luminance(rgb: Vector3) -> float:
	return 0.2126 * rgb.x + 0.7152 * rgb.y + 0.0722 * rgb.z


func _initialize() -> void:
	# The interpolation rule, without data: ring values held at the mid-distances, linear in between.
	var rings := PackedVector3Array([Vector3(0.4, 0.4, 0.4), Vector3(0.3, 0.3, 0.3), Vector3(0.2, 0.2, 0.2), Vector3(0.1, 0.1, 0.1)])
	_check("profile: ring 0 holds inside its mid-distance (50 m)", GroundAlbedoNear.profile_at(rings, 20.0) == rings[0])
	_check("profile: at a mid-distance the ring's value (175 m)", GroundAlbedoNear.profile_at(rings, 175.0).is_equal_approx(rings[1]))
	_check("profile: halfway between two mid-distances the mean (112.5 m)",
			GroundAlbedoNear.profile_at(rings, 112.5).is_equal_approx((rings[0] + rings[1]) * 0.5))
	_check("profile: the last ring holds beyond its mid-distance", GroundAlbedoNear.profile_at(rings, 1400.0) == rings[3])
	_check("no imagery: no profile", GroundAlbedoNear.ring_albedo(null, PAD_LAT_DEG, PAD_LON_DEG).is_empty())

	var imagery := ImagerySource.new(Paths.data_dir().path_join("imagery/earth/index.toml"))
	if imagery.is_empty():
		print("ground_albedo_near_test: no data/imagery, imagery checks skipped")
		quit(1 if _failures > 0 else 0)
		return
	var sync_rings := GroundAlbedoNear.ring_albedo(imagery, PAD_LAT_DEG, PAD_LON_DEG)
	_check("one value per ring at the pad", sync_rings.size() == GroundAlbedoNear.ring_count(), "%d rings" % sync_rings.size())
	var plausible := sync_rings.size() == GroundAlbedoNear.ring_count()
	for ring in sync_rings:
		var y := _luminance(ring)
		plausible = plausible and y > 0.05 and y < 0.3 and ring.x > ring.z
	_check("every ring is a plausible surface albedo (luminance 0.05 .. 0.30, red above blue)", plausible,
			"luminance %s" % [Array(sync_rings).map(_luminance)])
	_check("outside the imagery's coverage (equator, Atlantic): no profile",
			GroundAlbedoNear.ring_albedo(imagery, 0.0, -30.0).is_empty())

	var near := GroundAlbedoNear.new(imagery)
	var first := near.at(PAD_LAT_DEG, PAD_LON_DEG)
	_check("the first call starts a read and returns its profile only when the worker was fast", near.reads() == 1
			and (first.is_empty() or first == sync_rings))
	var value := near.finish()
	_check("the worker's profile equals the synchronous one", value == sync_rings)
	_check("the read's wall time is recorded", near.last_read_ms() > 0.0, "%.1f ms" % near.last_read_ms())
	near.at(PAD_LAT_DEG + 50.0 / 111132.0, PAD_LON_DEG)
	_check("50 m on: no new read", near.reads() == 1)
	near.at(PAD_LAT_DEG + 200.0 / 111132.0, PAD_LON_DEG)
	_check("200 m on: a new read", near.reads() == 2)
	near.at(PAD_LAT_DEG + 500.0 / 111132.0, PAD_LON_DEG)
	_check("a read in flight blocks a further one until it is collected", near.reads() <= 3)
	near.finish()
	print("ground_albedo_near_test: %s" % ["PASSED" if _failures == 0 else "FAILED (%d)" % _failures])
	quit(1 if _failures > 0 else 0)
