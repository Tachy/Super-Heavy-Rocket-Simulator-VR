class_name FreeCameraPose
extends RefCounted
## The free camera's pose, kept in the body-fixed frame of the body it is near, so it stays put over the ground while the
## body turns (Earth ~465 m/s at the equator) - far from both bodies it stays inertial (GCRF). Position float64 in the
## frame's axes (GCRF-aligned axes; z = north for a body frame), orientation in render axes of that frame.
## The frame is chosen with hysteresis; a switch re-expresses the same GCRF pose (no jump).

const FRAME_INERTIAL := ""
const FRAME_EARTH := "earth"
const FRAME_MOON := "moon"
## [E] Within this distance of a body's centre the camera co-rotates with it (geostationary orbit: 4.2e7 m from Earth's centre
## is outside, so a far view of the whole Earth is inertial). Leaving needs HYSTERESIS x this distance.
const EARTH_FRAME_RADIUS_M := 3.0e7
const MOON_FRAME_RADIUS_M := 1.0e7
const HYSTERESIS := 1.1

var frame := FRAME_INERTIAL
var position_frame_m := Vector3.ZERO
var basis_frame := Basis.IDENTITY  ## render axes of the frame


## Takes the pose (GCRF position, render-axes orientation) as the current one. `earth_rotation` / `moon_rotation`: fixed ->
## GCRF in GCRF axes; `moon_center_gcrf_m`.
func reset_to(position_gcrf_m: Vector3, view_basis: Basis, earth_rotation: Basis, moon_rotation: Basis,
		moon_center_gcrf_m: Vector3) -> void:
	frame = FRAME_INERTIAL
	_choose_frame(position_gcrf_m, moon_center_gcrf_m)
	_store(position_gcrf_m, view_basis, earth_rotation, moon_rotation, moon_center_gcrf_m)


## Moves the pose by `velocity_local_mps` (camera axes) and turns it by `rates` (x pitch, y yaw, z roll about the camera's
## axes, rad/s) over `delta_s`; returns {position_gcrf_m, basis} (render axes) for the camera.
func step(rates: Vector3, velocity_local_mps: Vector3, delta_s: float, earth_rotation: Basis, moon_rotation: Basis,
		moon_center_gcrf_m: Vector3) -> Dictionary:
	basis_frame = FreeCameraControl.rotate_local(basis_frame, rates, delta_s)
	position_frame_m += Frames.from_render(basis_frame * velocity_local_mps) * delta_s
	var pose := to_gcrf(earth_rotation, moon_rotation, moon_center_gcrf_m)
	var previous := frame
	_choose_frame(pose["position_gcrf_m"], moon_center_gcrf_m)
	if frame != previous:  # the same pose in the new frame
		_store(pose["position_gcrf_m"], pose["basis"], earth_rotation, moon_rotation, moon_center_gcrf_m)
	return pose


## Turns the camera by `angle_rad` (x pitch, y yaw, z roll about its own axes).
func turn(angle_rad: Vector3) -> void:
	basis_frame = FreeCameraControl.rotate_local(basis_frame, angle_rad, 1.0)


## The pose in GCRF: {position_gcrf_m, basis}.
func to_gcrf(earth_rotation: Basis, moon_rotation: Basis, moon_center_gcrf_m: Vector3) -> Dictionary:
	var rotation := _rotation(frame, earth_rotation, moon_rotation)
	var center := moon_center_gcrf_m if frame == FRAME_MOON else Vector3.ZERO
	return {"position_gcrf_m": center + rotation * position_frame_m,
			"basis": Frames.basis_to_render(rotation) * basis_frame}


func _store(position_gcrf_m: Vector3, view_basis: Basis, earth_rotation: Basis, moon_rotation: Basis,
		moon_center_gcrf_m: Vector3) -> void:
	var rotation := _rotation(frame, earth_rotation, moon_rotation)
	var center := moon_center_gcrf_m if frame == FRAME_MOON else Vector3.ZERO
	position_frame_m = rotation.transposed() * (position_gcrf_m - center)
	basis_frame = (Frames.basis_to_render(rotation).transposed() * view_basis).orthonormalized()


func _choose_frame(position_gcrf_m: Vector3, moon_center_gcrf_m: Vector3) -> void:
	var earth_distance := position_gcrf_m.length()
	var moon_distance := (position_gcrf_m - moon_center_gcrf_m).length()
	var keep := HYSTERESIS if frame != FRAME_INERTIAL else 1.0
	if frame == FRAME_EARTH and earth_distance < EARTH_FRAME_RADIUS_M * keep:
		return
	if frame == FRAME_MOON and moon_distance < MOON_FRAME_RADIUS_M * keep:
		return
	if earth_distance < EARTH_FRAME_RADIUS_M:
		frame = FRAME_EARTH
	elif moon_distance < MOON_FRAME_RADIUS_M:
		frame = FRAME_MOON
	else:
		frame = FRAME_INERTIAL


static func _rotation(frame_name: String, earth_rotation: Basis, moon_rotation: Basis) -> Basis:
	match frame_name:
		FRAME_EARTH:
			return earth_rotation
		FRAME_MOON:
			return moon_rotation
		_:
			return Basis.IDENTITY
