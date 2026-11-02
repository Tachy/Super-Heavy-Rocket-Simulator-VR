## Crew Dragon's IDSS docking mechanism, drawn in the capsule stage's body frame with its origin on the port (HCS mating plane). `set_state` is the only entry for
## docking visuals: ring extension (DockingMechanism::ring_extension_m), ring tilt, hooks. Numbers: the manifest's `idss` block (config/vehicles/detail/crew_dragon.toml [idss]);
## parts: idss_mechanism.gltf (frames there). Nodes: `flange`, `hcs_metal`, `hook_pockets` (fixed), `ring` with `petal_<i>` below it, `guide_pin_<i>`, `hook_<i>`, `struts` with
## `strut_<i>`. No physics here.
class_name DockingMechanismView
extends Node3D

var _idss := {}
var _ring: Node3D
var _hooks: Array[Node3D] = []
var _struts: Array[Dictionary] = []
var _ring_extension_m := 0.0
var _ring_tilt_rad := Vector2.ZERO
var _hook_closed := 0.0


## `models`: the crew_dragon set of VehicleModels.load_for; `material_of(part)`: the part's material.
func build(models: Dictionary, material_of: Callable) -> void:
	_idss = models["manifest"]["idss"]
	var parts: Dictionary = models["mechanism"]
	position = _vec(_idss["port_position_m"])
	for fixed: String in ["flange", "hcs_metal", "hook_pockets"]:
		_instance(fixed, parts, material_of, self)
	_ring = _instance("ring", parts, material_of, self)
	var count := 0
	for theta: float in _idss["petal_theta_deg"]:
		var petal := _instance("petal", parts, material_of, _ring, "petal_%d" % count)
		petal.basis = _about_axis(theta)
		count += 1
	count = 0
	for theta: float in _idss["pin_theta_deg"]:
		var pin := _instance("guide_pin", parts, material_of, self, "guide_pin_%d" % count)
		pin.position = _radial(theta) * 0.5 * float(_idss["pin_circle_diameter_m"])
		count += 1
	var hook_count := int(_idss["hook_count"])
	for k in hook_count:
		_hooks.append(_instance("hook", parts, material_of, self, "hook_%d" % k))
	var struts := Node3D.new()
	struts.name = "struts"
	add_child(struts)
	for pair: float in _idss["strut_pair_theta_deg"]:
		for side: float in [-1.0, 1.0]:
			var i := _struts.size()
			var holder := Node3D.new()
			holder.name = "strut_%d" % i
			struts.add_child(holder)
			var entry := {
				"top": _radial(pair + side * float(_idss["strut_top_half_deg"])) * float(_idss["strut_top_r_m"]) + Vector3(_ring_underside_x(), 0.0, 0.0),
				"base": _radial(pair + side * float(_idss["strut_base_half_deg"])) * float(_idss["strut_base_r_m"]) + Vector3(_floor_x(), 0.0, 0.0),
				"barrel": _instance("strut_barrel", parts, material_of, holder),
				"middle": _instance("strut_middle", parts, material_of, holder),
				"rod": _instance("strut_rod", parts, material_of, holder),
			}
			_struts.append(entry)
	set_state(0.0, Vector2.ZERO, 0.0)


## ring_extension_m: the ring's face above the HCS plane, 0 = retracted (then recessed by the IDD's 6 mm), the physics' full extension at most as the physics reports it.
## ring_tilt_rad: about the port's y and z (x along the axis), radians, about the ring's centre. hook_closed: 0 open (ready to dock) .. 1 closed (ready to hook, mated).
func set_state(ring_extension_m: float, ring_tilt_rad: Vector2, hook_closed: float) -> void:
	_ring_extension_m = ring_extension_m
	_ring_tilt_rad = ring_tilt_rad
	_hook_closed = clampf(hook_closed, 0.0, 1.0)
	var full := float(_idss["ring_extension_m"])
	var recess := float(_idss["scs_recess_m"]) * (1.0 - clampf(ring_extension_m / full, 0.0, 1.0))
	_ring.transform = Transform3D(_tilt_basis(ring_tilt_rad), Vector3(ring_extension_m - recess, 0.0, 0.0))
	for k in _hooks.size():
		_hooks[k].transform = hook_transform(k, _hook_closed)
	for strut: Dictionary in _struts:
		_pose_strut(strut)


func ring_plane_x_m() -> float:
	return _ring.position.x


## The ring-side end of strut i, this view's frame.
func strut_top_m(i: int) -> Vector3:
	return _ring.transform * (_struts[i]["top"] as Vector3)


## The attachment the strut's rod reaches (its far end), this view's frame: must equal strut_top_m.
func strut_rod_end_m(i: int) -> Vector3:
	return (_struts[i]["rod"] as Node3D).transform * Vector3(float(_idss["strut_tube_length_m"]), 0.0, 0.0)


func strut_base_m(i: int) -> Vector3:
	return _struts[i]["base"]


func strut_barrel_start_m(i: int) -> Vector3:
	return (_struts[i]["barrel"] as Node3D).position


func strut_count() -> int:
	return _struts.size()


## Pose of the active hook k at closing fraction f: about its pivot, the local (u, v) plane is (tangent, axis), the radial is its normal.
func hook_transform(k: int, f: float) -> Transform3D:
	var theta := float(_idss["hook_first_theta_deg"]) + 360.0 * k / float(_idss["hook_count"])
	var radial := _radial(theta)
	var azimuth := atan2(radial.z, radial.y)
	var e_r := Vector3(0.0, cos(azimuth), sin(azimuth))
	var tangent := Vector3(0.0, -sin(azimuth), cos(azimuth))
	var pivot: Array = _idss["hook_pivot_m"]  # [u along the tangent from the latch centre, x from the HCS plane]
	var origin := e_r * float(_idss["hook_radius_m"]) + tangent * float(pivot[0]) + Vector3(float(pivot[1]), 0.0, 0.0)
	var turn := deg_to_rad(float(_idss["hook_turn_deg"])) * (1.0 - f)
	return Transform3D(Basis(Vector3.RIGHT, azimuth) * Basis(Vector3.UP, turn), origin)


func _pose_strut(strut: Dictionary) -> void:
	var a: Vector3 = strut["base"]
	var b: Vector3 = _ring.transform * (strut["top"] as Vector3)
	var along := (b - a).normalized()
	var frame := _along_x(along)
	var length := float(_idss["strut_tube_length_m"])
	(strut["barrel"] as Node3D).transform = Transform3D(frame, a)
	(strut["rod"] as Node3D).transform = Transform3D(frame, b - along * length)
	(strut["middle"] as Node3D).transform = Transform3D(frame, 0.5 * (a + b) - along * 0.5 * length)


func _along_x(direction: Vector3) -> Basis:
	var side := direction.cross(Vector3.RIGHT)
	if side.length() < 1e-6:
		side = direction.cross(Vector3.UP)
	side = side.normalized()
	return Basis(direction, side, direction.cross(side))


func _ring_underside_x() -> float:
	return float(_idss["ring_section_m"][0][1])


func _floor_x() -> float:
	return float(_idss["strut_base_x_m"])


## The tilt in this view's frame: about the port's y and z. Port z = the port's up; port y = up x axis.
func _tilt_basis(tilt: Vector2) -> Basis:
	var up := _vec(_idss["port_up"])
	var axis := _vec(_idss["port_axis"])
	var y := up.cross(axis)
	return Basis(y, tilt.x) * Basis(up, tilt.y)


## Direction of the IDD's axial-view angle (viewed from outside onto the port, clockwise from `up`).
func _radial(theta_deg: float) -> Vector3:
	var up := _vec(_idss["port_up"])
	var right := up.cross(_vec(_idss["port_axis"]))
	return up * cos(deg_to_rad(theta_deg)) + right * sin(deg_to_rad(theta_deg))


## The rotation about the axis that takes azimuth 0 (+Y) to the angle theta.
func _about_axis(theta_deg: float) -> Basis:
	var radial := _radial(theta_deg)
	return Basis(Vector3.RIGHT, atan2(radial.z, radial.y))


func _instance(part: String, parts: Dictionary, material_of: Callable, parent: Node3D, node_name := "") -> MeshInstance3D:
	var instance := MeshInstance3D.new()
	instance.name = node_name if node_name != "" else part
	instance.mesh = parts[part]
	instance.material_override = material_of.call(part)
	parent.add_child(instance)
	return instance


static func _vec(values: Array) -> Vector3:
	return Vector3(float(values[0]), float(values[1]), float(values[2]))
