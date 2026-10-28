# Headless test of the cloud pass's per-eye (VR) geometry: the matrices CloudEffect builds for each view and the ray /
# depth math of shaders/cloud_pass.gdshaderinc, mirrored here in double precision (the compute kernels need a GPU).
# Two eye views are built the way Godot's OpenXR path builds them: asymmetric off-axis frusta from typical headset
# FOV angles (the starsim openxr-double-projection patch: set_frustum(tan(angle) * near, ...)), eyes at +-IPD/2, and
# RendererSceneRender::set_multiview_camera's view_projection[v] = P_eye * Projection(view_offset[v].inverse())
# (the eye offset is INSIDE the projection RenderSceneDataRD.get_view_projection returns), with the RD depth
# correction (reversed Z, y flipped).
# Checks, for a cloud point at a known distance: the per-eye pixels the pass shows it at give the physical disparity
# (IPD / distance), the depth -> distance of the pass returns the true scene distance, and the temporal reprojection
# of a static point returns to its own pixel. CloudCentreView (VR performance part 2): the centre view covers both
# eyes (also canted ones), keeps a single view as it is and caps a very wide union; the centre march's depth from an
# eye's buffer is within IPD/2 of the centre ray's own; each eye's upsample reads a cloud point's centre sample at the
# eye's true pixel of the point (mirrors of cloud_pass.gdshaderinc CLOUD_CENTRE and the VR CLOUD_UPSAMPLE).
# The mirrors (_pass_*) must stay in step with cloud_pass.gdshaderinc (cloud_ray_earth, cloud_scene_distance, the
# temporal kernel) and cloud_effect.gd (_render_callback's per-view matrices).
# Run: godot --headless --path game --script res://tests/cloud_vr_stereo_test.gd
extends SceneTree

const IPD_M := 0.064
## [S: Meta Quest 2 runtime, XrFovf of the left eye, rad] (the right eye mirrored): asymmetric, wider to the outside.
const FOV_LEFT_EYE := {"left": -0.9424778, "right": 0.6981317, "up": 0.8203047, "down": -0.8377581}
const FAR_M := 3.0e7  # OrbitCamera.far
const NEARS_M := [0.2, 1.0]  # OrbitCamera's site view and the smallest orbit-view near plane
const DISTANCES_M := [2000.0, 20000.0]
## A disparity error of 1 urad is ~0.2 arcsec, far below stereo acuity (~10 arcsec = 48 urad).
const DISPARITY_TOLERANCE_RAD := 1.0e-6
const DISTANCE_TOLERANCE_REL := 1.0e-6
const REPROJECTION_TOLERANCE_NDC := 1.0e-7
## cloud_pass.gdshaderinc CLOUD_CENTRE_PROBE_M: where a centre ray is projected into an eye to find its depth texel.
const CENTRE_PROBE_M := 1.0e6
## cloud_pass.gdshaderinc: the soft edge's width in metres and the tent radius limits (centre pixels).
const EDGE_SOFT_M := 20.0
const TENT_RADIUS_MIN := 1.0
const TENT_RADIUS_MAX := 2.0
## cloud_view_cache.gdshaderinc (cloud view cache Task 4): the lookup's parallax iterations and the distance of a texel
## without cloud (rotation only).
const CACHE_PARALLAX_ITERATIONS := 3
const CACHE_FAR_M := 1.0e7
## The cache lookup's bound: a cloud point lands within a quarter of an eye pixel of the eye's direct ray, at the
## headset's ~35 px/deg (cloud view cache spec).
const CACHE_PARALLAX_TOLERANCE_PX := 0.25
const EYE_PX_PER_RAD := 35.0 * 180.0 / PI
## The camera's offset from the anchor in the parallax case (m): a moved camera before a re-anchor.
const CACHE_CAMERA_OFFSET_M := 50.0
## The near shell's radius (CloudLayer.near_radius_m(), cloud view cache Task 5): the cache holds clouds from there on,
## seen from its anchor. A tile keeps the anchor it was rendered from until it is refilled; the camera's offsets from a
## tile's anchor checked (m) with the lookup's iterations, for a cloud at R_near: the re-anchor's 5 % of R_near and the
## eviction's keep distance (CloudViewCacheSchedule.ANCHOR_EVICT_RATIO 0.1 of the tile's cloud distance; fix round 1:
## the same angle for a cloud at 5 R_near from 1 km, the keep cap). Measured worst eye px of the deck at 2 km with three
## iterations: 100 m 1.2e-6, 200 m 3.8e-4, 300 m 0.011, 400 m 0.13, 500 m 0.88, 600 m 4.0 (five: 600 m 0.0012).
const CACHE_NEAR_RADIUS_M := 2000.0
const CACHE_TILE_ANCHOR_CASES := [Vector3(100.0, CACHE_PARALLAX_ITERATIONS, 1.0),
		Vector3(200.0, CACHE_PARALLAX_ITERATIONS, 1.0), Vector3(1000.0, CACHE_PARALLAX_ITERATIONS, 5.0)]
## cloud_view_cache.gdshaderinc CACHE_ANCHOR_ITERATIONS (Task 5 fix round 1): parallax solves of one eye ray until the
## tile it lands on shows the anchor it was solved from.
const CACHE_ANCHOR_ITERATIONS := 3
## The anchor boundary case (fix round 1): tiles of up to three anchors side by side in azimuth, the camera at the
## current one, older ones this far off (m; the eviction's keep distance at R_near) - eye rays swept across the
## boundaries every CACHE_BOUNDARY_STEP_RAD.
const CACHE_BOUNDARY_OFFSET_M := 200.0
const CACHE_REANCHOR_M := 100.0  # CloudViewCacheSchedule.REANCHOR_FRACTION x R_near
const CACHE_BOUNDARY_STEP_RAD := 0.0005
## The deck field is asserted from this distance on (m): a deck nearer than ~2 km seen 50 m off the anchor needs a 4th
## iteration (0.6 eye px left at 300 m after 3) - the near shell's reach (R_near, cloud view cache Task 5), marched per
## frame, not looked up. Printed below it.
const CACHE_DECK_ASSERTED_M := 2000.0

var _failures := 0


func _initialize() -> void:
	for near_m: float in NEARS_M:
		for distance_m: float in DISTANCES_M:
			for azimuth in [0.0, 0.3]:  # straight ahead and 17 deg to the right
				_check_point(near_m, distance_m, azimuth)
	_test_centre_view()
	_test_centre_size_hysteresis()
	for wall_m in [50.0, 2000.0, 20000.0]:
		_test_centre_gather(wall_m)
	for distance_m in [100.0, 2000.0, 20000.0]:
		for azimuth in [0.0, 0.3, -0.6]:
			_test_centre_upsample(distance_m, azimuth)
	_test_edge_radius()
	for distance_m in [300.0, 2000.0, 20000.0]:
		_test_cache_parallax(distance_m)
	for case: Vector3 in CACHE_TILE_ANCHOR_CASES:
		_test_tile_anchor_parallax(case.x, int(case.y), case.z)
	for anchors in [2, 3]:
		_test_anchor_boundary(anchors)
	print("cloud_vr_stereo_test: %s" % ("PASS" if _failures == 0 else "FAIL (%d)" % _failures))
	quit(1 if _failures > 0 else 0)


## The per-eye data CloudEffect uses (view: 0 left, 1 right); the head (camera) at the origin looking along -z.
func _view(view: int, near_m: float) -> Dictionary:
	var sign := -1.0 if view == 0 else 1.0
	var fov_l: float = FOV_LEFT_EYE["left"] if view == 0 else -FOV_LEFT_EYE["right"]
	var fov_r: float = FOV_LEFT_EYE["right"] if view == 0 else -FOV_LEFT_EYE["left"]
	var eye_projection := Projection.create_frustum(tan(fov_l) * near_m, tan(fov_r) * near_m,
			tan(FOV_LEFT_EYE["down"]) * near_m, tan(FOV_LEFT_EYE["up"]) * near_m, near_m, FAR_M)
	var eye_offset := Vector3(sign * IPD_M / 2.0, 0.0, 0.0)
	var view_offset := Transform3D(Basis.IDENTITY, eye_offset)
	var correction := Projection.create_depth_correction(true)
	return {
		"eye_projection": correction * eye_projection,  # eye space -> clip (what the eye's pixels are)
		"eye_offset": eye_offset,
		# RenderSceneDataRD.get_view_projection(view): correction * P_eye * Projection(view_offset.inverse())
		"projection": correction * eye_projection * Projection(view_offset.affine_inverse()),
		"cam": Transform3D.IDENTITY,  # scene.get_cam_transform() (the head)
	}


## cloud_effect.gd: the per-view block (inv_projection, view_to_world) and clip_from_earth.
func _pass_block(v: Dictionary) -> Dictionary:
	var projection: Projection = v["projection"]
	var cam: Transform3D = v["cam"]
	var view_to_world := cam  # cloud_effect.gd: the head (the projection holds the eye offset)
	var earth_to_world := Projection(Transform3D(Basis.IDENTITY, cam.origin))
	return {
		"inv_projection": projection.inverse(),
		"view_to_world": view_to_world,
		"eye_earth": cam.basis * (v["eye_offset"] as Vector3),  # world_to_earth = identity here
		"clip_from_earth": projection * Projection(view_to_world.affine_inverse()) * earth_to_world,  # :325
	}


## cloud_pass.gdshaderinc cloud_march: the ray's origin relative to the head (cloud_view.eye_earth: each eye marches
## from its own position).
func _pass_origin(block: Dictionary) -> Vector3:
	return block["eye_earth"]


## cloud_pass.gdshaderinc cloud_ray_earth: the ray direction.
func _pass_ray(block: Dictionary, ndc: Vector2) -> Vector3:
	var inv: Projection = block["inv_projection"]
	var a: Vector4 = inv * Vector4(ndc.x, ndc.y, 0.5, 1.0)
	var b: Vector4 = inv * Vector4(ndc.x, ndc.y, 1.0, 1.0)
	var dir_view := (Vector3(a.x, a.y, a.z) / a.w - Vector3(b.x, b.y, b.z) / b.w).normalized()
	return ((block["view_to_world"] as Transform3D).basis * dir_view).normalized()


## cloud_pass.gdshaderinc cloud_scene_distance.
func _pass_scene_distance(block: Dictionary, ndc: Vector2, depth: float) -> float:
	var p: Vector4 = (block["inv_projection"] as Projection) * Vector4(ndc.x, ndc.y, depth, 1.0)
	return (Vector3(p.x, p.y, p.z) / p.w).length()


## The eye's true viewing direction through a pixel (eye space = head axes here).
func _eye_ray(v: Dictionary, ndc: Vector2) -> Vector3:
	var p: Vector4 = (v["eye_projection"] as Projection).inverse() * Vector4(ndc.x, ndc.y, 0.5, 1.0)
	return (Vector3(p.x, p.y, p.z) / p.w).normalized()


func _clip(m: Projection, point: Vector3) -> Vector4:
	return m * Vector4(point.x, point.y, point.z, 1.0)


func _azimuth(dir: Vector3) -> float:
	return atan2(dir.x, -dir.z)


func _check_point(near_m: float, distance_m: float, azimuth: float) -> void:
	var target := Vector3(sin(azimuth), 0.0, -cos(azimuth)) * distance_m  # from the head
	var label := "near %.1f m, cloud at %.0f m, azimuth %.2f" % [near_m, distance_m, azimuth]
	var shown := []  # per eye: the azimuth the eye looks along at the pixel where the pass draws the point
	for view in [0, 1]:
		var v := _view(view, near_m)
		var block := _pass_block(v)
		# The eye's row of the point (its true ndc y; the pass's error is horizontal for a horizontal eye offset).
		var c_true := _clip(v["projection"], target)
		var ndc_y := c_true.y / c_true.w
		# The pixel whose pass ray meets the point: bisection on ndc x (monotonic azimuth).
		var want := _azimuth(target - _pass_origin(block))
		var lo := -1.0
		var hi := 1.0
		for i in 80:
			var mid := 0.5 * (lo + hi)
			if _azimuth(_pass_ray(block, Vector2(mid, ndc_y))) < want:
				lo = mid
			else:
				hi = mid
		var ndc := Vector2(0.5 * (lo + hi), ndc_y)
		shown.append(_azimuth(_eye_ray(v, ndc)))

		# Depth -> distance: the depth the renderer writes for the point in this eye, read back by the pass.
		var depth := c_true.z / c_true.w
		var true_ndc := Vector2(c_true.x / c_true.w, ndc_y)
		var read_m := _pass_scene_distance(block, true_ndc, depth)
		var rel := absf(read_m - target.length()) / target.length()
		_check(rel < DISTANCE_TOLERANCE_REL, "%s, eye %d: depth -> distance %.4f m vs %.4f m (rel %s)"
				% [label, view, read_m, target.length(), String.num_scientific(rel)])

		# Temporal reprojection, no motion: the pixel's ray at the point's distance -> the same eye's clip.
		var ray := _pass_ray(block, true_ndc)
		var back := _clip(block["clip_from_earth"], _pass_origin(block) + ray * target.length())
		var drift := (Vector2(back.x, back.y) / back.w - true_ndc).length()
		_check(drift < REPROJECTION_TOLERANCE_NDC, "%s, eye %d: static reprojection drift %s ndc"
				% [label, view, String.num_scientific(drift)])

	var expected: float = atan2(IPD_M / 2.0 * cos(azimuth), distance_m) * 2.0  # ~ IPD cos(az) / distance
	var disparity: float = shown[0] - shown[1]  # crossed (> 0): left eye looks right, right eye left
	var error := absf(disparity - expected)
	var perceived := IPD_M * cos(azimuth) / disparity if disparity > 0.0 else INF
	_check(error < DISPARITY_TOLERANCE_RAD,
			"%s: disparity %s rad vs IPD/D %s rad (perceived distance %.3f m)"
			% [label, String.num_scientific(disparity), String.num_scientific(expected), perceived])


## A pair of eyes as the renderer hands them over, optionally canted (each eye's frustum rotated outward by `cant_rad`
## about the head's up axis, as on Pimax headsets) and with the per-side FOV scaled by `widen` (the tangents).
func _eye_projections(near_m: float, cant_rad: float, widen := 1.0) -> Array[Projection]:
	var result: Array[Projection] = []
	for view in [0, 1]:
		var sign := -1.0 if view == 0 else 1.0
		var fov_l: float = FOV_LEFT_EYE["left"] if view == 0 else -FOV_LEFT_EYE["right"]
		var fov_r: float = FOV_LEFT_EYE["right"] if view == 0 else -FOV_LEFT_EYE["left"]
		var eye_projection := Projection.create_frustum(tan(fov_l) * widen * near_m, tan(fov_r) * widen * near_m,
				tan(FOV_LEFT_EYE["down"]) * near_m, tan(FOV_LEFT_EYE["up"]) * near_m, near_m, FAR_M)
		var view_offset := Transform3D(Basis(Vector3.UP, -sign * cant_rad), Vector3(sign * IPD_M / 2.0, 0.0, 0.0))
		result.append(Projection.create_depth_correction(true) * eye_projection
				* Projection(view_offset.affine_inverse()))
	return result


## Head-space direction of a clip-space ndc through a projection (the pass's two-depth difference).
func _head_ray(projection: Projection, ndc: Vector2) -> Vector3:
	var inv := projection.inverse()
	var a: Vector4 = inv * Vector4(ndc.x, ndc.y, 0.5, 1.0)
	var b: Vector4 = inv * Vector4(ndc.x, ndc.y, 1.0, 1.0)
	return (Vector3(a.x, a.y, a.z) / a.w - Vector3(b.x, b.y, b.z) / b.w).normalized()


## CloudCentreView: the centre view covers every eye's frustum (corner rays inside its ndc), keeps a single view as
## it is, and caps a very wide union at MAX_HALF_ANGLE_RAD.
func _test_centre_view() -> void:
	var size := Vector2i(3530, 2834)
	for cant in [0.0, deg_to_rad(10.0)]:
		var eyes := _eye_projections(1.0, cant)
		var centre := CloudCentreView.build(eyes, size, 3)
		var projection: Projection = centre["projection"]
		var worst := 0.0
		for eye in eyes:
			for corner in [Vector2(-1, -1), Vector2(1, -1), Vector2(-1, 1), Vector2(1, 1)]:
				var c := _clip(projection, _head_ray(eye, corner) * 1.0e6)
				worst = maxf(worst, maxf(absf(c.x / c.w), absf(c.y / c.w)))
		_check(worst <= 1.0 + 1.0e-9,
				"centre view, cant %.0f deg: every eye corner inside the centre (max |ndc| %.12f)"
				% [rad_to_deg(cant), worst])
		var low: Vector2i = centre["low"]
		_check(low.x > ceili(size.x / 3.0) and low.y >= ceili(size.y / 3.0) and low.x < ceili(size.x / 3.0) * 2,
				"centre view, cant %.0f deg: low size %s (one eye %s / 3)" % [rad_to_deg(cant), str(low), str(size)])
		var pixel_angle: float = centre["pixel_angle"]
		_check(is_equal_approx(pixel_angle, 2.0 / (absf(projection.y.y) * low.y)), "centre view: pixel angle")
	var mono := _eye_projections(0.2, 0.0)
	var single := CloudCentreView.build([mono[0]], Vector2i(1601, 899), 2)
	_check(single["projection"] == mono[0] and single["low"] == Vector2i(801, 450),
			"centre view of one view: that view, low = ceil(size / divisor) (%s)" % str(single["low"]))
	var wide := CloudCentreView.build(_eye_projections(1.0, deg_to_rad(40.0), 1.6), size, 3)
	var edge := _head_ray(wide["projection"], Vector2(1.0, 0.0))
	var half_angle := atan2(edge.x, -edge.z)
	_check(half_angle <= CloudCentreView.MAX_HALF_ANGLE_RAD + 1.0e-9,
			"centre view of a very wide canted pair: capped at %.1f deg (%.3f deg)"
			% [rad_to_deg(CloudCentreView.MAX_HALF_ANGLE_RAD), rad_to_deg(half_angle)])


## The runtime's field of view jitters by a hair from frame to frame (Pimax, VR run 2026-10-27: the centre height
## alternated 946 / 947): the kept size must not follow it (each new size re-created the images and showed the previous
## frame's clouds - the layer lagged the head and jumped), but must follow a real change (quality preset, resize).
func _test_centre_size_hysteresis() -> void:
	var size := Vector2i(3534, 2838)  # the run's per-eye target
	var base := CloudCentreView.build(_eye_projections(1.0, 0.0), size, 3)
	var low: Vector2i = base["low"]
	var changed := 0
	for i in range(40):
		# the run's alternation: the built size one row / column off (ceil of a size on an integer boundary)
		var built := base.duplicate()
		built["low"] = low + Vector2i(i % 2, (i + 1) % 2)
		var view := CloudCentreView.keep_size(built, low)
		if view["low"] != low:
			changed += 1
		if i == 0:
			var projection: Projection = view["projection"]
			var kept: Vector2i = view["low"]
			_check(is_equal_approx(view["pixel_angle"], 2.0 / (absf(projection.y.y) * kept.y)),
					"centre size hysteresis: pixel angle of the kept size")
	_check(changed == 0, "centre size hysteresis: a size one row / column off keeps the size (%d changes in 40)" % changed)
	var finer := CloudCentreView.keep_size(CloudCentreView.build(_eye_projections(1.0, 0.0), size, 2), low)
	_check(finer["low"] != low, "centre size hysteresis: a new divisor takes its size (%s -> %s)"
			% [str(low), str(finer["low"])])
	var first := CloudCentreView.keep_size(base, Vector2i.ZERO)
	_check(first["low"] == low, "centre size hysteresis: no previous size takes the built one")


## cloud_pass.gdshaderinc CLOUD_CENTRE gather (mirror): a centre ray's scene distance read from the eye whose ndc holds
## the ray (left first) at the texel where the ray's far point (CLOUD_CENTRE_PROBE_M) projects; the distance the eye's
## depth gives (from the head, as cloud_scene_distance returns it) differs from the centre ray's own by at most the eye
## offset IPD/2 (the parallax of the geometry). Scene: a wall at `wall_m` in front of the head.
func _test_centre_gather(wall_m: float) -> void:
	var eyes := _eye_projections(1.0, 0.0)
	var centre: Projection = CloudCentreView.build(eyes, Vector2i(3530, 2834), 3)["projection"]
	var worst := 0.0
	var picked_left := 0
	for ndc in [Vector2(-0.8, 0.3), Vector2(-0.2, -0.4), Vector2(0.0, 0.0), Vector2(0.3, 0.6), Vector2(0.85, -0.2)]:
		var dir := _head_ray(centre, ndc)
		var true_m := wall_m / -dir.z  # along the centre ray from the head to the wall
		var eye := 0
		var probe := _clip(eyes[0], dir * CENTRE_PROBE_M)
		var eye_ndc := Vector2(probe.x, probe.y) / probe.w
		if absf(eye_ndc.x) > 1.0 or absf(eye_ndc.y) > 1.0:
			eye = 1
			probe = _clip(eyes[1], dir * CENTRE_PROBE_M)
			eye_ndc = Vector2(probe.x, probe.y) / probe.w
		else:
			picked_left += 1
		# The eye's pixel ray (from the eye) meets the wall: the depth the renderer writes there.
		var eye_pos := Vector3((-1.0 if eye == 0 else 1.0) * IPD_M / 2.0, 0.0, 0.0)
		var eye_dir := _head_ray(eyes[eye], eye_ndc)
		var hit := eye_pos + eye_dir * ((-wall_m - eye_pos.z) / eye_dir.z)
		var c := _clip(eyes[eye], hit)
		var p: Vector4 = eyes[eye].inverse() * Vector4(eye_ndc.x, eye_ndc.y, c.z / c.w, 1.0)
		var read_m := (Vector3(p.x, p.y, p.z) / p.w).length()
		worst = maxf(worst, absf(read_m - true_m))
	_check(worst <= IPD_M / 2.0 + 1.0e-6 * wall_m, "centre gather, wall at %.0f m: distance within IPD/2 (worst %s m)"
			% [wall_m, String.num_scientific(worst)])
	_check(picked_left >= 3, "centre gather: the left eye for rays inside its ndc (%d of 5)" % picked_left)


## cloud_pass.gdshaderinc CLOUD_UPSAMPLE, VR (mirror): an eye pixel reads the centre image where its own ray, at the
## cloud distance the centre measured from the head, projects. For a cloud point both eyes' pixels showing it (their
## true pixels: the physical disparity) must read the centre sample of that point - the same for both eyes (shared
## noise and blur) and at its true place (no disparity error).
func _test_centre_upsample(distance_m: float, azimuth: float) -> void:
	var eyes := _eye_projections(1.0, 0.0)
	var centre_view := CloudCentreView.build(eyes, Vector2i(3530, 2834), 3)
	var centre: Projection = centre_view["projection"]
	var low: Vector2i = centre_view["low"]
	var target := Vector3(sin(azimuth), 0.05, -cos(azimuth)) * distance_m  # from the head
	var c_true := _clip(centre, target)
	var want := Vector2(c_true.x, c_true.y) / c_true.w
	var worst_px := 0.0
	for view in [0, 1]:
		var c_eye := _clip(eyes[view], target)
		var eye_ndc := Vector2(c_eye.x, c_eye.y) / c_eye.w  # the eye's pixel that shows the point
		var eye_pos := Vector3((-1.0 if view == 0 else 1.0) * IPD_M / 2.0, 0.0, 0.0)
		var read := eye_pos + _head_ray(eyes[view], eye_ndc) * target.length()  # the centre's cloud distance
		var c_read := _clip(centre, read)
		var got := Vector2(c_read.x, c_read.y) / c_read.w
		worst_px = maxf(worst_px, ((got - want) * 0.5 * Vector2(low)).length())
	_check(worst_px < 0.01, "centre upsample, cloud at %.0f m, azimuth %.1f: both eyes read its centre sample "
			% [distance_m, azimuth] + "(worst %s centre px)" % String.num_scientific(worst_px))


## cloud_pass.gdshaderinc cloud_tent_radius (mirror): the eyes' soft-edge tent spans EDGE_SOFT_M at the cloud's distance
## (a cloud's frayed edge is about as wide in metres near and far, so it narrows on screen with distance; a fixed screen
## radius blurred far clouds as much as near ones, user 2026-10-26), between bilinear (1) and the soft maximum (2).
static func _tent_radius(cloud_m: float, pixel_angle: float) -> float:
	return clampf(EDGE_SOFT_M / (maxf(cloud_m, 1.0) * maxf(pixel_angle, 1.0e-9)), TENT_RADIUS_MIN, TENT_RADIUS_MAX)


func _test_edge_radius() -> void:
	var centre := CloudCentreView.build(_eye_projections(1.0, 0.0), Vector2i(3530, 2834), 3)
	var angle: float = centre["pixel_angle"]
	_check(_tent_radius(500.0, angle) == TENT_RADIUS_MAX, "edge radius: a cloud at 500 m is soft (%.2f px)"
			% _tent_radius(500.0, angle))
	_check(_tent_radius(50000.0, angle) == TENT_RADIUS_MIN, "edge radius: a cloud at 50 km is sharp (%.2f px)"
			% _tent_radius(50000.0, angle))
	var last := INF
	var falling := true
	for d in [1000.0, 3000.0, 6000.0, 8000.0, 12000.0, 20000.0]:
		var r := _tent_radius(d, angle)
		falling = falling and r <= last
		last = r
	_check(falling, "edge radius: narrows steadily with the distance")
	var in_metres := _tent_radius(8000.0, angle) * 8000.0 * angle
	_check(absf(in_metres - EDGE_SOFT_M) < 1.0e-6 or _tent_radius(8000.0, angle) in [TENT_RADIUS_MIN, TENT_RADIUS_MAX],
			"edge radius: %.1f m wide at 8 km between the limits" % in_metres)


## cloud_view_cache.gdshaderinc cloud_cache_lookup (mirror): the anchor's direction q an eye ray (offset o = eye -
## anchor, unit dir) reads the cache at - the root of g(s) = r(q(s)) - |o + s dir| along the eye ray, q(s) the
## direction of o + s dir and r the cached cloud distance from the anchor (no cloud: CACHE_FAR_M). The first two steps
## are fixed-point steps (the eye ray's point at the cached distance from the anchor, |o + s dir| = r), the next ones
## secant steps on g while both ends saw cloud (a flat deck seen 50 m off the anchor converged by only ~0.4 per
## fixed-point step: 25 eye px left after three); CACHE_PARALLAX_ITERATIONS + 1 distance reads. distance_of: q -> r
## (m, 0 without cloud).
static func _cache_lookup(offset: Vector3, dir: Vector3, distance_of: Callable,
		iterations := CACHE_PARALLAX_ITERATIONS) -> Vector3:
	var b := offset.dot(dir)
	var c := offset.length_squared()
	var r: float = distance_of.call(dir)
	var s_prev := -b + sqrt(maxf(b * b - c + _cache_far(r) * _cache_far(r), 0.0))
	r = distance_of.call((offset + dir * s_prev).normalized())
	var g_prev := r - (offset + dir * s_prev).length() if r > 0.0 else NAN
	var s := -b + sqrt(maxf(b * b - c + _cache_far(r) * _cache_far(r), 0.0))
	for i in range(1, iterations):
		r = distance_of.call((offset + dir * s).normalized())
		var g := r - (offset + dir * s).length() if r > 0.0 else NAN
		var next := -b + sqrt(maxf(b * b - c + _cache_far(r) * _cache_far(r), 0.0))
		if not is_nan(g) and not is_nan(g_prev) and g != g_prev:
			var secant := s - g * (s - s_prev) / (g - g_prev)
			next = secant if secant > 0.0 else next
		s_prev = s
		g_prev = g
		s = next
	return (offset + dir * s).normalized()


static func _cache_far(r: float) -> float:
	return r if r > 0.0 else CACHE_FAR_M


## The parallax of the view cache lookup (mirror): a cloud point at distance_m seen from each eye (IPD_M, the head at
## the anchor) and from a camera CACHE_CAMERA_OFFSET_M off the anchor (across, along and up the view): the cached point
## the lookup reads (A + r(q) q) lies within CACHE_PARALLAX_TOLERANCE_PX eye pixels of the eye's direct ray to the
## point. Two cached fields: the point's distance in every direction (a cloud at that distance) and a flat deck through
## the point (the distance varies with the direction; the point 30 deg below the horizon).
func _test_cache_parallax(distance_m: float) -> void:
	var point_dir := Vector3(0.3, -0.5, -0.8).normalized()
	var eyes: Array[Vector3] = [Vector3(-IPD_M / 2.0, 0.0, 0.0), Vector3(IPD_M / 2.0, 0.0, 0.0)]
	for offset_dir: Vector3 in [Vector3.RIGHT, Vector3.FORWARD, Vector3.UP]:
		eyes.append(offset_dir * CACHE_CAMERA_OFFSET_M)
	for field in ["shell", "deck"]:
		var worst_px := 0.0
		for eye: Vector3 in eyes:
			var point := eye + point_dir * distance_m  # the cloud point, distance_m from this viewpoint
			var normal := Vector3.UP
			var deck_height := point.dot(normal)  # the deck's plane through the point (below the anchor: < 0)
			var distance_of := func(q: Vector3) -> float:
				if field == "shell":
					return point.length()
				var down := q.dot(normal)
				return deck_height / down if down < 0.0 else 0.0  # the plane ahead of the anchor; above it no cloud
			var dir := (point - eye).normalized()
			var q := _cache_lookup(eye, dir, distance_of)
			var shown: Vector3 = q * float(distance_of.call(q))  # the cached point the eye pixel shows
			worst_px = maxf(worst_px, (shown - eye).angle_to(dir) * EYE_PX_PER_RAD)
		var label := "view cache parallax, %s cloud at %.0f m: eyes +-%.3f m and a " \
				% [field, distance_m, IPD_M / 2.0] + "camera %.0f m off the anchor land on the direct ray " \
				% CACHE_CAMERA_OFFSET_M + "(worst %s eye px)" % String.num_scientific(worst_px)
		if field == "deck" and distance_m < CACHE_DECK_ASSERTED_M:
			print("  info  %s (not asserted: inside the near shell's reach)" % label)
			continue
		_check(worst_px <= CACHE_PARALLAX_TOLERANCE_PX, label)


## The parallax of a tile seen from a camera off its anchor (cloud view cache Task 5, the progressive re-anchor: each
## tile is reprojected from its own anchor): the camera offset_m off the anchor in six directions, a cloud point at
## radii x the near shell's radius from the anchor (R_near: the nearest the cache holds) in the two cached fields of
## _test_cache_parallax - within CACHE_PARALLAX_TOLERANCE_PX eye pixels of the direct ray.
func _test_tile_anchor_parallax(offset_m: float, iterations: int, radii: float) -> void:
	var point_dir := Vector3(0.3, -0.5, -0.8).normalized()
	var point := point_dir * CACHE_NEAR_RADIUS_M * radii  # from the anchor
	var normal := Vector3.UP
	var deck_height := point.dot(normal)
	for field in ["shell", "deck"]:
		var worst_px := 0.0
		var distance_of := func(q: Vector3) -> float:
			if field == "shell":
				return point.length()
			var down := q.dot(normal)
			return deck_height / down if down < 0.0 else 0.0
		for offset_dir: Vector3 in [Vector3.RIGHT, Vector3.FORWARD, Vector3.UP, Vector3.LEFT, Vector3.BACK,
				Vector3.DOWN]:
			var eye := offset_dir * offset_m
			var dir := (point - eye).normalized()
			var q := _cache_lookup(eye, dir, distance_of, iterations)
			var shown: Vector3 = q * float(distance_of.call(q))
			worst_px = maxf(worst_px, (shown - eye).angle_to(dir) * EYE_PX_PER_RAD)
		_check(worst_px <= CACHE_PARALLAX_TOLERANCE_PX, "view cache parallax from a tile's own anchor, %s cloud "
				% field + "%.0f m from it, the camera %.0f m off it, %d iterations (worst %s eye px)"
				% [CACHE_NEAR_RADIUS_M * radii, offset_m, iterations, String.num_scientific(worst_px)])


## cloud_view_cache.gdshaderinc cloud_cache_pick (mirror, Task 5 fix round 1): the one rule by which the near shell's
## march and the lookup pick an eye ray's tile and anchor - the parallax lookup from the current anchor, again from the
## anchor of the tile it lands on until that tile shows the anchor solved from (CACHE_ANCHOR_ITERATIONS solves).
## anchors: the ring's positions (the head at the origin); anchor_of: q -> the anchor index of the tile at q (-1: no
## level); distance_of: q -> the cached distance there (m, from that tile's anchor; 0 without cloud). Returns {level
## (false: none), stable, anchor, q}.
static func _cache_pick(eye: Vector3, dir: Vector3, anchors: Array[Vector3], current: int, anchor_of: Callable,
		distance_of: Callable) -> Dictionary:
	var anchor := current
	for k in range(CACHE_ANCHOR_ITERATIONS):
		var q := _cache_lookup(eye - anchors[anchor], dir, distance_of)
		var shown: int = anchor_of.call(q)
		if shown < 0:
			return {"level": false, "stable": false, "anchor": anchor, "q": q}
		if shown == anchor:
			return {"level": true, "stable": true, "anchor": anchor, "q": q}
		anchor = shown
	return {"level": true, "stable": false, "anchor": anchor, "q": _cache_lookup(eye - anchors[anchor], dir,
			distance_of)}


## Tiles of n_anchors anchors side by side (cloud view cache Task 5 fix round 1, the review's I2 / I3): the current
## anchor at the head and older ones CACHE_BOUNDARY_OFFSET_M off, each holding the tiles of its azimuth sector (its
## cached distances its own view of one deck), the camera half a re-anchor off the current one, eye rays swept across
## the sectors' boundaries to a deck point at 2 R_near.
## The pick (shared by the near shell and the lookup) reads every settled ray from the anchor of the tile it reads,
## within CACHE_PARALLAX_TOLERANCE_PX of the direct ray; a ray it leaves unsettled lies where no tile holds its point
## (the march covers it). The rules before (the near shell: the tile at the R_near sphere's exit from the current
## anchor; the lookup: a second solve from the landed tile's anchor, its tile read unchecked) disagree there - shown.
func _test_anchor_boundary(n_anchors: int) -> void:
	# The camera (head) moved half a re-anchor on (-z) from the current anchor A0; A1 left of it, A2 behind it.
	var a0 := Vector3(0.0, 0.0, 0.5 * CACHE_REANCHOR_M)
	var anchors: Array[Vector3] = [a0, a0 + Vector3(-CACHE_BOUNDARY_OFFSET_M, 0.0, 0.0),
			a0 + Vector3(0.0, 0.0, CACHE_BOUNDARY_OFFSET_M)]
	anchors.resize(n_anchors)
	var limits := [0.0, -0.15]  # sector boundaries in azimuth (rad): A0 right of 0, A1 left of it (A2 left of -0.15)
	var anchor_of := func(q: Vector3) -> int:
		var azimuth := atan2(q.x, -q.z)
		if azimuth >= limits[0]:
			return 0
		return 1 if n_anchors == 2 or azimuth >= limits[1] else 2
	var depression := asin(0.5)  # 30 deg below the horizon: the deck 2 km below the head lies 2 R_near away
	var deck_y := -2.0 * CACHE_NEAR_RADIUS_M * sin(depression)
	var distance_of := func(q: Vector3) -> float:
		var a: Vector3 = anchors[anchor_of.call(q)]
		return (deck_y - a.y) / q.y if q.y < 0.0 else 0.0
	var settled := [0, 0, 0]
	var unsettled := 0
	var worst_px := 0.0
	var gap_ok := true
	var old_near_differs := 0
	var old_lookup_unchecked := 0
	var azimuth := -0.35
	while azimuth <= 0.15:
		var dir := Vector3(sin(azimuth) * cos(depression), -sin(depression), -cos(azimuth) * cos(depression))
		var point := dir * (deck_y / dir.y)  # the deck point on the eye ray
		var pick := _cache_pick(Vector3.ZERO, dir, anchors, 0, anchor_of, distance_of)
		if pick["stable"]:
			var a: int = pick["anchor"]
			settled[a] += 1
			var q: Vector3 = pick["q"]
			var shown: Vector3 = anchors[a] + q * float(distance_of.call(q))
			worst_px = maxf(worst_px, shown.angle_to(dir) * EYE_PX_PER_RAD)
			# The near shell's rule before: the tile at the R_near sphere's exit from the current anchor.
			var o := -a0
			var b := o.dot(dir)
			var exit := -b + sqrt(b * b - o.length_squared() + CACHE_NEAR_RADIUS_M * CACHE_NEAR_RADIUS_M)
			if anchor_of.call((o + dir * exit).normalized()) != a:
				old_near_differs += 1
		else:
			unsettled += 1
			for k in range(n_anchors):  # no tile holds the point: seen from each anchor it lies in another's sector
				gap_ok = gap_ok and anchor_of.call((point - anchors[k]).normalized()) != k
		# The lookup's rule before: a second solve from the landed tile's anchor, the tile it lands on read unchecked.
		var first: int = anchor_of.call(_cache_lookup(-a0, dir, distance_of))
		if first > 0:
			var again := _cache_lookup(-anchors[first], dir, distance_of)
			if anchor_of.call(again) != first:
				old_lookup_unchecked += 1
		azimuth += CACHE_BOUNDARY_STEP_RAD
	var found_all: bool = settled.slice(0, n_anchors).min() > 0
	_check(found_all and worst_px <= CACHE_PARALLAX_TOLERANCE_PX and gap_ok and unsettled > 0,
			"view cache pick across %d anchors' tiles (older %.0f m off, deck at %.0f m): settled per anchor %s within "
			% [n_anchors, CACHE_BOUNDARY_OFFSET_M, 2.0 * CACHE_NEAR_RADIUS_M, str(settled.slice(0, n_anchors))]
			+ "%s eye px of the direct ray, %d unsettled rays all where no tile holds the point (%s)"
			% [String.num_scientific(worst_px), unsettled, gap_ok])
	print("  info  the rules before Task 5 fix round 1 on the same rays: the near shell's tile at the sphere's exit "
			+ "named another anchor than the lookup's on %d settled rays, the lookup's second solve read a tile of a "
			% old_near_differs + "third anchor or none unchecked on %d" % old_lookup_unchecked)


func _check(ok: bool, message: String) -> void:
	if ok:
		print("  ok    %s" % message)
	else:
		_failures += 1
		print("  FAIL  %s" % message)
