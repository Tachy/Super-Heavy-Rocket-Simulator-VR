class_name XrProjectionRecord
extends RefCounted
## The headset's per-eye projections and eye transforms as the JSON the offscreen stereo harness (StereoMockXr, tests/)
## loads; shipped code (PerfLog writes it at a VR start), so not in tests/.


## The JSON dictionary for an initialised interface (what PerfLog records): its projections for the aspect of the render
## target at the given near / far, and the eyes' transforms relative to the head.
static func record(interface: XRInterface, near: float, far: float) -> Dictionary:
	var size := interface.get_render_target_size()
	var views := []
	for view in range(interface.get_view_count()):
		var projection := interface.get_projection_for_view(view, size.x / size.y, near, far)
		var eye := interface.get_transform_for_view(view, Transform3D.IDENTITY)
		views.append({"projection": projection_to_array(projection), "eye_from_head": transform_to_array(eye)})
	return {"render_target": [int(size.x), int(size.y)], "views": views, "near_m": near, "far_m": far}


static func projection_to_array(p: Projection) -> Array:
	return [p.x.x, p.x.y, p.x.z, p.x.w, p.y.x, p.y.y, p.y.z, p.y.w, p.z.x, p.z.y, p.z.z, p.z.w, p.w.x, p.w.y, p.w.z,
			p.w.w]


static func transform_to_array(t: Transform3D) -> Array:
	return [t.basis.x.x, t.basis.x.y, t.basis.x.z, t.basis.y.x, t.basis.y.y, t.basis.y.z, t.basis.z.x, t.basis.z.y,
			t.basis.z.z, t.origin.x, t.origin.y, t.origin.z]
