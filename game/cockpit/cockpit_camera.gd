class_name CockpitCamera
extends RefCounted
## The pilot's head pose in the Crew Dragon (docs/subsystems/cockpit.md, "Camera mode and toggle"): the cabin's eye point
## (CockpitConfig.eye, capsule stage body frame) carried by the capsule's body - position and attitude from the simulation
## snapshot in float64 (never from the float32 scene nodes), plus the desktop mouse look relative to the seat.
## World._update_camera hands the result to the OrbitCamera (mode COCKPIT); in VR the pose is the XR base and the headset
## moves on top (XrManager.set_base).

## Desktop look limits relative to the seat (plan Task 6): yaw either side, pitch up / down.
const YAW_LIMIT_RAD := deg_to_rad(170.0)
const PITCH_LIMIT_RAD := deg_to_rad(80.0)

var position_gcrf_m := Vector3.ZERO  ## the head, GCRF
var basis_gcrf := Basis.IDENTITY  ## the head's axes (columns right, up, back of the camera) as GCRF vectors, look included
var body_index := -1  ## the simulated body carrying the capsule, -1 before the first successful update
var look_yaw_rad := 0.0  ## mouse look about the seat's up axis (positive: to the left)
var look_pitch_rad := 0.0  ## mouse look about the seat's right axis (positive: up)


## The index of the body that carries the capsule stage of `config`, -1 when there is none (no Dragon in the session, or
## the capsule is gone). `sim`: a FlightSim (typed Object so that a test can pass a stub with the same getters).
static func capsule_body(sim: Object, config: CockpitConfig) -> int:
	if config == null or not sim.is_active():
		return -1
	for i in sim.body_count():
		for stage: Dictionary in sim.body_stages(i):
			if String(stage["id"]) == config.capsule_stage_id and String(stage.get("vehicle_id", "")).contains(config.id):
				return i
	return -1


## The capsule stage's base station (m) in the body frame of body `index`; NAN when it has no such stage.
static func capsule_base_station_m(sim: Object, index: int, config: CockpitConfig) -> float:
	for stage: Dictionary in sim.body_stages(index):
		if String(stage["id"]) == config.capsule_stage_id:
			return float(stage["base_station_m"])
	return NAN


## Takes the head pose from the simulation snapshot. Returns false (pose kept) when no body carries the capsule.
func update(sim: Object, config: CockpitConfig) -> bool:
	var index := capsule_body(sim, config)
	if index < 0:
		return false
	var base_m := capsule_base_station_m(sim, index, config)
	set_body_pose(sim.body_position_gcrf_m(index), Basis(sim.body_attitude_gcrf(index) as Quaternion),
			sim.body_cg_station_m(index), base_m, config)
	body_index = index
	return true


## The pose from a body's state: position + attitude * (stage base + eye - CG) along the station axis (body x), the head's
## axes attitude * eye basis, turned by the look.
func set_body_pose(body_position_gcrf_m: Vector3, attitude: Basis, cg_station_m: float, base_station_m: float,
		config: CockpitConfig) -> void:
	var eye_body_m := Vector3(base_station_m - cg_station_m, 0.0, 0.0) + config.eye.origin
	position_gcrf_m = body_position_gcrf_m + attitude * eye_body_m
	var seat := attitude * config.eye.basis
	basis_gcrf = seat * Basis(Vector3.UP, look_yaw_rad) * Basis(Vector3.RIGHT, look_pitch_rad)


## The head's orientation in render axes (the OrbitCamera's basis).
func basis_render() -> Basis:
	return Basis(Frames.to_render(basis_gcrf.x), Frames.to_render(basis_gcrf.y), Frames.to_render(basis_gcrf.z))


## Mouse look: turns the view by (pitch, yaw) radians, clamped to the limits relative to the seat.
func look(delta_rad: Vector2) -> void:
	look_pitch_rad = clampf(look_pitch_rad + delta_rad.x, -PITCH_LIMIT_RAD, PITCH_LIMIT_RAD)
	look_yaw_rad = clampf(look_yaw_rad + delta_rad.y, -YAW_LIMIT_RAD, YAW_LIMIT_RAD)


## F12: the view back to the seat's forward direction.
func reset_look() -> void:
	look_pitch_rad = 0.0
	look_yaw_rad = 0.0
