class_name Frames
## Mapping between simulation frames (GCRF / body-fixed, z = north) and Godot render axes (y = up).
##
## The mapping is the cyclic permutation (x, y, z) -> (y, z, x): a proper rotation, so handedness is
## preserved and celestial north becomes render +Y. All values stay float64 (double-precision engine).

## GCRF or body-fixed vector -> render axes.
static func to_render(v: Vector3) -> Vector3:
	return Vector3(v.y, v.z, v.x)


## Render axes -> GCRF or body-fixed vector.
static func from_render(v: Vector3) -> Vector3:
	return Vector3(v.z, v.x, v.y)


## Rotation fixed -> GCRF (Basis rows = matrix rows) expressed in render axes: P * R * P^T.
static func basis_to_render(rotation: Basis) -> Basis:
	var x := to_render(rotation * from_render(Vector3.RIGHT))
	var y := to_render(rotation * from_render(Vector3.UP))
	var z := to_render(rotation * from_render(Vector3.BACK))
	return Basis(x, y, z)
