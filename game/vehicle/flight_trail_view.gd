class_name FlightTrailView
extends Node3D
## The flown path of every body (white, user 2026-10-23): positions sampled from the snapshots and kept Earth-fixed, so an
## ascent and a booster's return stay over the pad as seen from the ground (user choice: rotating with the Earth; in
## orbit the trail drifts ~22 deg per revolution against the inertial green orbit). Per body one mesh of the samples,
## rebuilt only when a sample is added - res://shaders/trail_ribbon.gdshader turns it with the Earth and places it
## relative to the camera - and one small mesh from the newest sample to the body, rebuilt every frame. The trail fades in
## only from 1 km behind the body so it does not clutter the vehicle.

const SAMPLE_INTERVAL_S := 1.0  ## simulation time between samples (time warp keeps its spacing)
const MIN_SPACING_M := 50.0  ## no new sample while the body stands still (pad, catch)
const MAX_SAMPLES := 20000  ## per body; beyond it every second old sample is dropped
const FADE_IN_START_M := 1000.0  ## user 2026-10-23: invisible for the first kilometre behind the body
const FADE_IN_LENGTH_M := 2000.0  ## [E] "slowly": full strength 3 km behind the body
const NEVER_CULLED := AABB(Vector3(-1.0e9, -1.0e9, -1.0e9), Vector3(2.0e9, 2.0e9, 2.0e9))  ## the shader moves the vertices
const SHADER := preload("res://shaders/trail_ribbon.gdshader")

var _trails := {}  ## body index -> Trail
var _mission_start_tdb_s := NAN


class Trail:
	var samples := PackedVector3Array()  ## Earth-fixed metres in render axes (Frames.to_render), oldest first
	var path_m := PackedFloat32Array()  ## per sample, along the trail from the oldest (float32: ~10 m at 1e8 m)
	var last_sample_s := -INF
	var material := ShaderMaterial.new()
	var body := MeshInstance3D.new()  ## the samples
	var head := MeshInstance3D.new()  ## newest sample -> body


func update_view(sim: FlightSim, camera_gcrf_m: Vector3, fixed_to_gcrf: Basis) -> void:
	if not sim.is_active() or sim.mission_start_tdb_s() != _mission_start_tdb_s:  # a new flight: the old trails go
		_clear()
		if not sim.is_active():
			return
		_mission_start_tdb_s = sim.mission_start_tdb_s()
	var gcrf_to_fixed := fixed_to_gcrf.transposed()
	var fixed_to_world := Frames.basis_to_render(fixed_to_gcrf)
	var earth_center_world := Frames.to_render(-camera_gcrf_m)  # GCRF is Earth-centred
	var now_s := sim.mission_time_s()
	for index in sim.body_count():
		var trail := _trail(index)
		var near_earth: bool = sim.telemetry(index).get("reference_body", "earth") == "earth"
		trail.body.visible = near_earth  # an Earth-fixed trail means nothing near the Moon
		trail.head.visible = near_earth
		if not near_earth:
			continue
		var position_fixed := Frames.to_render(gcrf_to_fixed * sim.body_position_gcrf_m(index))
		if _sample(trail, position_fixed, now_s):
			var mesh := ArrayMesh.new()
			RibbonMesh.add_local(mesh, trail.samples, trail.path_m, trail.material)
			trail.body.mesh = mesh
		var head_path_m := 0.0
		var newest_path_m := 0.0
		var head_mesh := ArrayMesh.new()
		if not trail.samples.is_empty():
			var last := trail.samples.size() - 1
			var newest := trail.samples[last]
			head_path_m = newest.distance_to(position_fixed)
			newest_path_m = trail.path_m[last]
			RibbonMesh.add_local(head_mesh, PackedVector3Array([newest, position_fixed]),
					PackedFloat32Array([newest_path_m, newest_path_m + head_path_m]), trail.material)
		trail.head.mesh = head_mesh
		trail.material.set_shader_parameter("earth_center_world", earth_center_world)
		trail.material.set_shader_parameter("fixed_to_world", fixed_to_world)
		trail.material.set_shader_parameter("head_path_m", head_path_m)
		trail.material.set_shader_parameter("newest_path_m", newest_path_m)


## Appends a sample once per SAMPLE_INTERVAL_S of simulation time if the body moved; true when the trail changed.
func _sample(trail: Trail, position_fixed: Vector3, now_s: float) -> bool:
	var count := trail.samples.size()
	if count > 0 and now_s - trail.last_sample_s < SAMPLE_INTERVAL_S:
		return false
	if count > 0 and trail.samples[count - 1].distance_to(position_fixed) < MIN_SPACING_M:
		return false
	trail.last_sample_s = now_s
	var along_m := trail.path_m[count - 1] + trail.samples[count - 1].distance_to(position_fixed) if count > 0 else 0.0
	trail.samples.append(position_fixed)
	trail.path_m.append(along_m)
	if trail.samples.size() > MAX_SAMPLES:
		var samples := PackedVector3Array()
		var path := PackedFloat32Array()
		var keep_from := trail.samples.size() - MAX_SAMPLES / 2
		for i in trail.samples.size():
			if i % 2 == 0 or i >= keep_from:
				samples.append(trail.samples[i])
				path.append(trail.path_m[i])
		trail.samples = samples
		trail.path_m = path
	return true


func _trail(index: int) -> Trail:
	if _trails.has(index):
		return _trails[index]
	var trail := Trail.new()
	trail.material.shader = SHADER
	trail.material.set_shader_parameter("scaled_space_distance_m", CelestialBody.SCALED_SPACE_DISTANCE_M)
	trail.material.set_shader_parameter("earth_radius_m", World.EARTH_MEAN_RADIUS_M)
	trail.material.set_shader_parameter("atmosphere_height_m", TrajectoryView.ATMOSPHERE_FADE_HEIGHT_M)
	trail.material.set_shader_parameter("fade_in_start_m", FADE_IN_START_M)
	trail.material.set_shader_parameter("fade_in_length_m", FADE_IN_LENGTH_M)
	for instance in [trail.body, trail.head]:
		instance.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		instance.custom_aabb = NEVER_CULLED
		add_child(instance)
	_trails[index] = trail
	return trail


func _clear() -> void:
	for trail: Trail in _trails.values():
		trail.body.queue_free()
		trail.head.queue_free()
	_trails.clear()
	_mission_start_tdb_s = NAN
