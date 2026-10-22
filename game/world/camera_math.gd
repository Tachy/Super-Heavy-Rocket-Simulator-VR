class_name CameraMath
extends RefCounted
## Pure math of the external camera (render axes, see frames.gd): horizon frame of the reference body, orbit pose
## about the target, zoom, the blend of "up" when the reference body changes, and the VR base pose.
## The camera is never rolled: its x-axis always lies in the horizon plane of the frame.

## [E] Pitch limit (rad, ~88.8 deg): keeps the view off the exact local vertical.
const PITCH_LIMIT := 1.55
## [E] Duration of the "up" blend when the reference body changes (Earth <-> Moon), real seconds.
const UP_BLEND_S := 3.0
## [E] Distance factor per mouse-wheel notch (logarithmic zoom).
const ZOOM_FACTOR_PER_NOTCH := 1.15
## [E] Below this length a direction counts as degenerate (up parallel to the pole axis, zero vectors).
const DEGENERATE_LENGTH := 1e-6


## Local horizon frame: y = up, -z = north (towards the render pole axis +Y), x = east. When up is parallel to the
## pole axis, north is taken from `previous_north` (projected into the horizon plane), else any horizontal axis.
static func horizon_frame(up_render: Vector3, previous_north: Vector3) -> Basis:
	var up := up_render.normalized() if up_render.length() > DEGENERATE_LENGTH else Vector3.UP
	var north := _horizontal(Vector3.UP, up)
	if north == Vector3.ZERO:
		north = _horizontal(previous_north, up)
	if north == Vector3.ZERO:
		north = _horizontal(Vector3.FORWARD, up)
	if north == Vector3.ZERO:
		north = _horizontal(Vector3.RIGHT, up)
	return Basis(north.cross(up), up, -north)


## Camera position relative to the target (render axes): `yaw` about the frame's up, `pitch` about the camera's
## horizontal axis (negative = camera above the target).
static func orbit_offset(frame: Basis, yaw: float, pitch: float, distance_m: float) -> Vector3:
	return orbit_basis(frame, yaw, pitch) * Vector3(0.0, 0.0, distance_m)


## Camera orientation for orbit_offset: looks at the target, x-axis horizontal (no roll) for every pitch.
static func orbit_basis(frame: Basis, yaw: float, pitch: float) -> Basis:
	return frame * Basis.from_euler(Vector3(pitch, yaw, 0.0))


## Rotation between two frames (slerp), `t` clamped to [0, 1].
static func blend_frame(from: Basis, to: Basis, t: float) -> Basis:
	var s := clampf(t, 0.0, 1.0)
	if s == 0.0:
		return from
	if s == 1.0:
		return to
	return Basis(from.get_rotation_quaternion().slerp(to.get_rotation_quaternion(), s))


## Distance after `notches` wheel notches (positive = closer), ZOOM_FACTOR_PER_NOTCH each, clamped.
static func zoom(distance_m: float, notches: int, min_m: float, max_m: float) -> float:
	var factor := pow(ZOOM_FACTOR_PER_NOTCH, absi(notches))
	var zoomed := distance_m / factor if notches >= 0 else distance_m * factor
	return clampf(zoomed, min_m, max_m)


## VR recentre offset from the head pose relative to the XR origin: yaw and position only, so the horizon never
## tilts (pattern of the Stunt Track Racer VR project).
static func recenter_offset(head_local: Transform3D) -> Transform3D:
	var forward := -head_local.basis.z
	var yaw := atan2(-forward.x, -forward.z) if Vector2(forward.x, forward.z).length() > DEGENERATE_LENGTH else 0.0
	return Transform3D(Basis(Vector3.UP, yaw), head_local.origin)


## XR origin transform that puts the head at the render origin with the base orientation when the headset is at
## its recentre pose.
static func vr_origin_transform(base: Basis, recenter: Transform3D) -> Transform3D:
	return Transform3D(base, Vector3.ZERO) * recenter.affine_inverse()


## Unit component of `v` perpendicular to the unit vector `up`; zero if `v` is (nearly) parallel to it.
static func _horizontal(v: Vector3, up: Vector3) -> Vector3:
	var h := v - up * up.dot(v)
	return h.normalized() if h.length() > DEGENERATE_LENGTH else Vector3.ZERO
