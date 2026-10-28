class_name XrProjectionRecord
extends RefCounted
## The headset's per-eye projections and eye transforms as the JSON the offscreen stereo harness (StereoMockXr, tests/)
## loads; shipped code (PerfLog writes it at a VR start), so not in tests/.


const IPD_MIN_M := 0.05  ## [E] the eye separation range of a headset (human IPD 5.4-7.4 cm, adjustable hardware)
const IPD_MAX_M := 0.08
const EYE_OFFSET_MAX_M := 0.05  ## [E] y / z eye offsets from the head origin: a few cm (display / lens depth)


## The JSON dictionary for an initialised interface (what PerfLog records): its projections for the aspect of the render
## target at the given near / far, and the eyes' transforms relative to the head. head_in_origin: the tracked head pose
## in the XR origin's space (XRServer.get_hmd_transform()). The interface's get_transform_for_view(view, IDENTITY) is the
## eye pose in the origin's space, head pose included, hence eye_from_head = head^-1 * eye. A mock that returns
## eye_from_head directly is recorded with the default IDENTITY head.
static func record(interface: XRInterface, near: float, far: float,
		head_in_origin: Transform3D = Transform3D.IDENTITY) -> Dictionary:
	var size := interface.get_render_target_size()
	var views := []
	for view in range(interface.get_view_count()):
		var projection := interface.get_projection_for_view(view, size.x / size.y, near, far)
		var eye := head_in_origin.affine_inverse() * interface.get_transform_for_view(view, Transform3D.IDENTITY)
		views.append({"projection": projection_to_array(projection), "eye_from_head": transform_to_array(eye)})
	return {"render_target": [int(size.x), int(size.y)], "views": views, "near_m": near, "far_m": far}


## Whether the head pose looks like a real tracked pose (not the default identity of a session that has no tracking yet).
static func head_pose_is_tracked(head_in_origin: Transform3D) -> bool:
	return not head_in_origin.is_equal_approx(Transform3D.IDENTITY) and head_in_origin.origin.length() > 1.0e-3


## "" when the two-view record is plausible for a headset, else why not: the eye x offsets sum to an IPD of 5-8 cm,
## view 0 (left) at x < 0, y / z offsets within a few cm, and the left eye's frustum shifted outward (left) relative to
## the right eye's (projection element [8] = (r + l) / (r - l) smaller for the left eye).
static func validate(data: Dictionary) -> String:
	var views: Array = data.get("views", [])
	if views.size() != 2:
		return "expected 2 views, got %d" % views.size()
	var eye: Array = []
	var shift: Array = []
	for view in views:
		var t: Array = view["eye_from_head"]
		eye.append(Vector3(t[9], t[10], t[11]))
		shift.append(float(view["projection"][8]))
	var ipd: float = eye[1].x - eye[0].x
	if ipd < IPD_MIN_M or ipd > IPD_MAX_M:
		return "eye x separation %.4f m outside %.2f-%.2f m (view 0 x %.4f, view 1 x %.4f)" % [
				ipd, IPD_MIN_M, IPD_MAX_M, eye[0].x, eye[1].x]
	if eye[0].x >= 0.0:
		return "view 0 (left) at x = %.4f m, expected < 0" % eye[0].x
	for v in eye:
		if absf(v.y) > EYE_OFFSET_MAX_M or absf(v.z) > EYE_OFFSET_MAX_M:
			return "eye offset y / z (%.4f, %.4f) m beyond %.2f m: not relative to the head" % [v.y, v.z, EYE_OFFSET_MAX_M]
	if shift[0] >= shift[1]:
		return "projection shift view 0 %.4f not left of view 1 %.4f: eye order does not match the frusta" % [
				shift[0], shift[1]]
	return ""


static func projection_to_array(p: Projection) -> Array:
	return [p.x.x, p.x.y, p.x.z, p.x.w, p.y.x, p.y.y, p.y.z, p.y.w, p.z.x, p.z.y, p.z.z, p.z.w, p.w.x, p.w.y, p.w.z,
			p.w.w]


static func transform_to_array(t: Transform3D) -> Array:
	return [t.basis.x.x, t.basis.x.y, t.basis.x.z, t.basis.y.x, t.basis.y.y, t.basis.y.z, t.basis.z.x, t.basis.z.y,
			t.basis.z.z, t.origin.x, t.origin.y, t.origin.z]
