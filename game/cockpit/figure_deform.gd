class_name FigureDeform
extends RefCounted
## Joint folds of one crew figure (Task 8b, docs/subsystems/cockpit.md "Crew body and suit"): reads the bend angle of each joint
## (shoulder, elbow, hip, knee per side) from the posed skeleton and drives the suit's corrective blend shapes and the wrinkle weights of
## crew_suit.gdshader. Pure presentation: the pose comes from ArmIk / the rest pose, nothing here feeds back.
## The joints come from the manifest block crew_figure.joints (figure_layout.joint_table): theta = the angle between the reference
## direction (ref bone towards ref_tail, times ref_sign) and the child bone (towards child_tail); bend_weights() mirrors
## figure_layout.bend_weights.

const SWING_GATE_DEG := Vector2(25.0, 65.0)  ## figure_layout.SWING_GATE_DEG

var _skeleton: Skeleton3D
var _mesh: MeshInstance3D
var _joints: Array[Dictionary] = []  # manifest entries + bone indices + blend shape indices, in slot order
var wrinkle := PackedFloat32Array()  ## the last wrinkle weights per slot (tests)
var theta_deg := PackedFloat32Array()  ## the last bend angles per slot (tests)
var gate := PackedFloat32Array()  ## the last swing gate factors per slot (tests)


## The deformer of a figure; null when the skeleton or the manifest lacks the joints.
static func for_figure(skeleton: Skeleton3D, mesh: MeshInstance3D, joints: Dictionary, order: Array) -> FigureDeform:
	if skeleton == null or mesh == null or joints.is_empty():
		return null
	var d := FigureDeform.new()
	d._skeleton = skeleton
	d._mesh = mesh
	# skeleton frame -> figure frame: the transforms between the skeleton and the figure root (CockpitView names it figure_<seat>;
	# its own transform places the figure frame in the cabin)
	var to_figure := Basis.IDENTITY
	var node: Node = skeleton
	while node != null and not String(node.name).begins_with("figure_"):
		if node is Node3D:
			to_figure = (node as Node3D).transform.basis.orthonormalized() * to_figure
		node = node.get_parent()
	for name: String in order:
		var j: Dictionary = (joints.get(name, {}) as Dictionary).duplicate()
		var bones := {}
		for key: String in ["child", "child_tail", "ref", "ref_tail"]:
			bones[key] = skeleton.find_bone(String(j.get(key, "")))
			if bones[key] < 0:
				return null
		j["bones"] = bones
		# the baked swing direction (manifest `plane`, figure frame) in the skeleton's frame, rest pose
		var plane: Array = j.get("plane", [0.0, 0.0, 0.0])
		j["plane_skeleton"] = (to_figure.inverse() * Vector3(float(plane[0]), float(plane[1]), float(plane[2]))).normalized()
		j["flex_index"] = mesh.find_blend_shape_by_name(StringName(String(j.get("shape_flex", ""))))
		j["ext_index"] = mesh.find_blend_shape_by_name(StringName(String(j.get("shape_ext", ""))))
		d._joints.append(j)
	d.wrinkle.resize(d._joints.size())
	d.theta_deg.resize(d._joints.size())
	d.gate.resize(d._joints.size())
	return d


## (flex, ext, wrinkle): the corrective shapes ramp from the rest angle to the full bend / the extension limit; the wrinkle weight is 0
## at the neutral (straight) angle and 1 at the full bend (figure_layout.bend_weights).
static func bend_weights(joint: Dictionary, theta: float) -> Vector3:
	var rest := float(joint["rest_deg"])
	var full := float(joint["full_deg"])
	var ext := float(joint["ext_deg"])
	var neutral := float(joint["neutral_deg"])
	var flex := clampf((theta - rest) / (full - rest), 0.0, 1.0) if full > rest else 0.0
	var extw := clampf((rest - theta) / (rest - ext), 0.0, 1.0) if rest > ext else 0.0
	return Vector3(flex, extw, clampf((theta - neutral) / (full - neutral), 0.0, 1.0))


## Factor on a swing joint's (shoulder, hip: manifest `gated`) flex shape, mirrors figure_layout.swing_gate: the shape is baked for one
## swing direction, so it acts fully within SWING_GATE_DEG.x of it and not beyond SWING_GATE_DEG.y (smoothstep between); a sideways or
## backward swing gets plain skinning instead of a forward-baked bulge (fix round 1: the pinched shoulder at the HOTAS grip).
static func swing_gate(joint: Dictionary, swing_cos: float) -> float:
	if not bool(joint.get("gated", false)):
		return 1.0
	return smoothstep(cos(deg_to_rad(SWING_GATE_DEG.y)), cos(deg_to_rad(SWING_GATE_DEG.x)), swing_cos)


## Reads the pose and sets the blend shapes and the shader's instance uniforms.
func update() -> void:
	var slots := PackedFloat32Array()
	slots.resize(8)
	for k in _joints.size():
		var j: Dictionary = _joints[k]
		var b: Dictionary = j["bones"]
		var ref := (_origin(b["ref_tail"]) - _origin(b["ref"])) * float(j["ref_sign"])
		var child := _origin(b["child_tail"]) - _origin(b["child"])
		var theta := rad_to_deg(ref.angle_to(child)) if ref.length() > 1e-6 and child.length() > 1e-6 else float(j["rest_deg"])
		var w := bend_weights(j, theta)
		var g := 1.0
		if bool(j.get("gated", false)) and ref.length() > 1e-6:
			var r := ref.normalized()
			var swing := child - r * r.dot(child)
			if swing.length() > 1e-3 * child.length():
				var ref_rot := (_skeleton.get_bone_global_pose(b["ref"]).basis.orthonormalized()
						* _skeleton.get_bone_global_rest(b["ref"]).basis.orthonormalized().inverse())
				g = swing_gate(j, swing.normalized().dot((ref_rot * (j["plane_skeleton"] as Vector3)).normalized()))
		w.x *= g
		gate[k] = g
		if int(j["flex_index"]) >= 0:
			_mesh.set_blend_shape_value(int(j["flex_index"]), w.x)
		if int(j["ext_index"]) >= 0:
			_mesh.set_blend_shape_value(int(j["ext_index"]), w.y)
		var slot := int(j["slot"])
		if slot >= 0 and slot < 8:
			slots[slot] = w.z
		wrinkle[k] = w.z
		theta_deg[k] = theta
	_mesh.set_instance_shader_parameter(&"wrinkle_l", Vector4(slots[0], slots[1], slots[2], slots[3]))
	_mesh.set_instance_shader_parameter(&"wrinkle_r", Vector4(slots[4], slots[5], slots[6], slots[7]))


func _origin(bone: int) -> Vector3:
	return _skeleton.get_bone_global_pose(bone).origin
