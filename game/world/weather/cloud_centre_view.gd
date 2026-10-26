class_name CloudCentreView
## The clouds' centre view for VR (VR performance part 2, user 2026-10-26): one frustum at the head covering both eyes,
## in which the clouds are marched, accumulated and filtered once per frame; each eye's upsample reads that image at its
## own rays. Both eyes then see the same noise, history and blur (two marches with their own jitter and history looked
## grainy and different per eye). The eyes' frusta come from the renderer's projections (which hold each eye's offset
## and, on canted displays, its rotation), never from assumed parallel eyes. One view (the desktop) is its own centre.

## [E] Cap of the union per side: a canted pair can span more, and a rectilinear image near 90 deg wastes its pixels.
const MAX_HALF_ANGLE_RAD := deg_to_rad(75.0)
## [E] The centre projection's far plane where the eyes' is infinite or unreadable (only directions and the clip xy of
## the temporal reprojection use it).
const FALLBACK_FAR_M := 1.0e9


## {projection, low, pixel_angle} of the centre view for these eye projections (clip <- head space, the renderer's
## convention: reversed Z in [0, 1], 1 = near, y down), each eye's full-resolution size `view_size` and the cloud pass's
## resolution divisor. `low`: the centre image size at the eyes' pixel density / divisor; `pixel_angle`: its pixel's
## angle (the march's band limit), as CloudEffect computes it for a view.
static func build(view_projections: Array[Projection], view_size: Vector2i, divisor: int) -> Dictionary:
	divisor = maxi(divisor, 1)
	if view_projections.size() == 1:
		var only := view_projections[0]
		var low := Vector2i(ceili(float(view_size.x) / divisor), ceili(float(view_size.y) / divisor))
		return {"projection": only, "low": low, "pixel_angle": 2.0 / maxf(absf(only.y.y) * low.y, 1e-6)}
	var cap := tan(MAX_HALF_ANGLE_RAD)
	var union_min := Vector2(INF, INF)
	var union_max := Vector2(-INF, -INF)
	var density := Vector2.ZERO  # full-resolution pixels per tangent unit (the densest eye)
	for projection in view_projections:
		var eye_min := Vector2(INF, INF)
		var eye_max := Vector2(-INF, -INF)
		for corner in [Vector2(-1, -1), Vector2(1, -1), Vector2(-1, 1), Vector2(1, 1)]:
			var t := _tangents(_ray(projection, corner))
			eye_min = eye_min.min(t)
			eye_max = eye_max.max(t)
		union_min = union_min.min(eye_min)
		union_max = union_max.max(eye_max)
		density = density.max(Vector2(view_size) / (eye_max - eye_min))
	union_min = union_min.clamp(Vector2(-cap, -cap), Vector2(cap, cap))
	union_max = union_max.clamp(Vector2(-cap, -cap), Vector2(cap, cap))
	var near_m := _near_m(view_projections[0])
	var centre := Projection.create_depth_correction(true) * Projection.create_frustum(union_min.x * near_m,
			union_max.x * near_m, union_min.y * near_m, union_max.y * near_m, near_m, FALLBACK_FAR_M)
	var size := (union_max - union_min) * density / divisor
	var low := Vector2i(maxi(ceili(size.x), 1), maxi(ceili(size.y), 1))
	return {"projection": centre, "low": low, "pixel_angle": 2.0 / maxf(absf(centre.y.y) * low.y, 1e-6)}


## Head-space direction through a clip-space ndc: the difference of two unprojected depths (the eye's offset is inside
## the projection, so one unprojected point is not on the eye's ray; cloud_pass.gdshaderinc cloud_ray_earth).
static func _ray(projection: Projection, ndc: Vector2) -> Vector3:
	var inv := projection.inverse()
	var a: Vector4 = inv * Vector4(ndc.x, ndc.y, 0.5, 1.0)
	var b: Vector4 = inv * Vector4(ndc.x, ndc.y, 1.0, 1.0)
	return (Vector3(a.x, a.y, a.z) / a.w - Vector3(b.x, b.y, b.z) / b.w).normalized()


## The ray's tangents (x / -z, y / -z) in the head's frame (looking along -z, y up).
static func _tangents(dir: Vector3) -> Vector2:
	var forward := maxf(-dir.z, 1e-6)
	return Vector2(dir.x / forward, dir.y / forward)


## The eye's near plane distance (reversed Z: depth 1 is the near plane) along its centre ray.
static func _near_m(projection: Projection) -> float:
	var p: Vector4 = projection.inverse() * Vector4(0.0, 0.0, 1.0, 1.0)
	var near_m := absf(p.z / p.w)
	return near_m if is_finite(near_m) and near_m > 0.0 else 0.05
