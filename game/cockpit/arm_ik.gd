class_name ArmIk
extends RefCounted
## Two-bone arm IK for the suited figure (docs/subsystems/cockpit.md, "Console pointer"): shoulder - elbow - wrist, the hand with a
## straight index finger as the end effector. Pure geometry in the skeleton (figure) frame: `solve()` takes the shoulder, the index
## tip target and a pole direction and returns the joint positions and the global bases of the bones (the minimal rotation from the
## rest bone to the solved direction, so the rest roll is kept). Limits: the wrist bends at most MAX_WRIST_DEG from the forearm, the
## upper arm at most MAX_SHOULDER_DEG from its rest direction (the mesh is rigidly skinned: above ~60 deg the shoulder shows
## notches, Task 4 carry), the elbow never fully straightens; a target beyond reach is clamped (`reached` false, no NaN).
## `for_skeleton()` reads the chain from a Skeleton3D rest pose and `apply()` writes the solved pose (rest * delta) to it.

const MAX_WRIST_DEG := 20.0  ## [E] Task 4 carry: wrist bend of the glove / suit
const MAX_SHOULDER_DEG := 100.0  ## [E] raise of the upper arm from its hanging rest direction (reach to the top screen row)
const ELBOW_SLACK := 0.9995  ## the chain never extends beyond this share of its length (no flipping at full stretch)
const REACHED_TOLERANCE_M := 1.0e-3
const GRIP_CURL_DEG := 28.0  ## [E] curl per index joint of a hand at rest (grip); the extended finger is 0
const SOLVE_ITERATIONS := 16
const RELAX_RAD := 0.35  ## the hand direction moves at most this much per pass after the first

var upper_len := 0.0
var fore_len := 0.0
var tip_local := Vector3.ZERO  ## index tip (straight finger) in the hand bone frame, from the wrist
var upper_rest := Basis.IDENTITY  ## global rest bases (y along the bone)
var fore_rest := Basis.IDENTITY
var hand_rest := Basis.IDENTITY
var pole_rest := Vector3.DOWN  ## the rest elbow's offset from the shoulder-wrist line (unit): where the elbow tends to point
var shoulder_rest := Vector3.ZERO
var side := "l"
# skeleton binding (for_skeleton)
var _bones := {}  # role -> bone index
var _index_rest_local: Array[Basis] = []  # index_k straight in the hand frame (before the curl)
var _index_len: Array[float] = []
var _index_origin_local := Vector3.ZERO  # index_1 origin in the hand frame
var _curl_axis := Vector3.RIGHT  # hand frame


## A chain from numbers (tests): rest bases give the bone directions (y) and roll.
static func from_numbers(upper_m: float, fore_m: float, tip_local_m: Vector3, upper_basis: Basis, fore_basis: Basis, hand_basis: Basis,
		shoulder_m: Vector3 = Vector3.ZERO) -> ArmIk:
	var ik := ArmIk.new()
	ik.upper_len = upper_m
	ik.fore_len = fore_m
	ik.tip_local = tip_local_m
	ik.upper_rest = upper_basis
	ik.fore_rest = fore_basis
	ik.hand_rest = hand_basis
	ik.shoulder_rest = shoulder_m
	var elbow := shoulder_m + upper_basis.y * upper_m
	var wrist := elbow + fore_basis.y * fore_m
	ik.pole_rest = ik._perp(elbow - shoulder_m, wrist - shoulder_m, Vector3.DOWN)
	return ik


## The chain of one arm ("l" / "r") of a figure skeleton; `bones` is the manifest block crew_figure.bones (the finger tip's length).
## null when the skeleton lacks the bones.
static func for_skeleton(skeleton: Skeleton3D, arm_side: String, bones: Dictionary = {}) -> ArmIk:
	var names := {"shoulder": "shoulder_", "upper": "upper_arm_", "fore": "forearm_", "hand": "hand_", "index1": "index_1_", "index2": "index_2_", "index3": "index_3_"}
	var ik := ArmIk.new()
	ik.side = arm_side
	for role: String in names:
		var idx := skeleton.find_bone(String(names[role]) + arm_side)
		if idx < 0:
			return null
		ik._bones[role] = idx
	var rest := func(role: String) -> Transform3D:
		var t: Transform3D = skeleton.get_bone_global_rest(ik._bones[role])
		return Transform3D(t.basis.orthonormalized(), t.origin)
	var up_t: Transform3D = rest.call("upper")
	var fo_t: Transform3D = rest.call("fore")
	var ha_t: Transform3D = rest.call("hand")
	var i1: Transform3D = rest.call("index1")
	var i2: Transform3D = rest.call("index2")
	var i3: Transform3D = rest.call("index3")
	ik.upper_rest = up_t.basis
	ik.fore_rest = fo_t.basis
	ik.hand_rest = ha_t.basis
	ik.shoulder_rest = up_t.origin
	ik.upper_len = (fo_t.origin - up_t.origin).length()
	ik.fore_len = (ha_t.origin - fo_t.origin).length()
	var len3 := 0.018
	var entry: Dictionary = bones.get("index_3_" + arm_side, {})
	if entry.has("head") and entry.has("tail"):
		len3 = (CockpitConfig._vec(entry["tail"]) - CockpitConfig._vec(entry["head"])).length()
	ik._index_len = [(i2.origin - i1.origin).length(), (i3.origin - i2.origin).length(), len3]
	var to_hand := ha_t.basis.inverse()
	ik._index_origin_local = to_hand * (i1.origin - ha_t.origin)
	var straight: Array[Basis] = []
	for t: Transform3D in [i1, i2, i3]:
		var local: Basis = to_hand * t.basis
		straight.append(Basis(Quaternion(local.y.normalized(), Vector3.UP)) * local)
	ik._index_rest_local = straight
	ik.tip_local = ik._index_origin_local + Vector3.UP * (ik._index_len[0] + ik._index_len[1] + ik._index_len[2])
	var tilt: Vector3 = (to_hand * i3.basis).y
	tilt = (tilt - Vector3.UP * tilt.dot(Vector3.UP)).normalized()
	ik._curl_axis = Vector3.UP.cross(tilt).normalized() if tilt.length() > 0.1 else Vector3.RIGHT
	ik.pole_rest = ik._perp(fo_t.origin - up_t.origin, ha_t.origin - up_t.origin, Vector3.DOWN)
	return ik


## Longest shoulder-to-tip distance with a straight arm, the hand in line with the forearm.
func reach_m() -> float:
	return upper_len + fore_len + tip_local.length()


## The index tip with the arm at rest (straight finger).
func rest_tip() -> Vector3:
	var elbow := shoulder_rest + upper_rest.y * upper_len
	var wrist := elbow + fore_rest.y * fore_len
	return wrist + hand_rest * tip_local


## Solves the arm for the index tip at `target` (skeleton frame). `pole`: direction the elbow tends to (default: the rest elbow's).
## Returns {shoulder, elbow, wrist, tip, reached, upper_basis, fore_basis, hand_basis, upper_dir, fore_dir, hand_dir, wrist_bend_rad}.
func solve(shoulder: Vector3, target: Vector3, pole: Vector3 = Vector3.ZERO) -> Dictionary:
	var pole_dir := pole if pole.length() > 1e-6 else pole_rest
	if not target.is_finite() or not shoulder.is_finite():
		return solve(shoulder_rest, rest_tip(), pole)  # a broken input stays at rest (no NaN into the skeleton)
	var to_target := target - shoulder
	if to_target.length() < 1e-6:
		to_target = upper_rest.y  # degenerate: the target is the shoulder
	var h_des := to_target.normalized()
	var max_wrist := deg_to_rad(MAX_WRIST_DEG)
	var h := h_des
	var elbow := shoulder
	var wrist := shoulder
	var hand_basis := hand_rest
	for iteration in SOLVE_ITERATIONS:
		hand_basis = Basis(_arc(hand_rest.y, h)) * hand_rest
		wrist = target - hand_basis * tip_local
		elbow = _elbow(shoulder, wrist, pole_dir)
		var fd := (wrist - elbow)
		fd = fd.normalized() if fd.length() > 1e-9 else h_des
		var wanted := _toward(fd, h_des, max_wrist)
		h = wanted if iteration == 0 else _toward(h, wanted, RELAX_RAD)  # relaxed after the first pass: the fixed point converges, no flip-flop
	# final geometry with the hand direction settled
	hand_basis = Basis(_arc(hand_rest.y, h)) * hand_rest
	wrist = target - hand_basis * tip_local
	elbow = _elbow(shoulder, wrist, pole_dir)
	var upper_dir := (elbow - shoulder) / upper_len
	# shoulder limit: the upper arm may not rise further than MAX_SHOULDER_DEG from its rest direction
	var limit := deg_to_rad(MAX_SHOULDER_DEG)
	if upper_dir.angle_to(upper_rest.y) > limit:
		upper_dir = _toward(upper_rest.y, upper_dir, limit)
		elbow = shoulder + upper_dir * upper_len
	# the wrist is where the forearm reaches (the chain is as long as it is)
	var to_wrist := wrist - elbow
	wrist = elbow + (to_wrist.normalized() if to_wrist.length() > 1e-9 else fore_rest.y) * fore_len
	var fore_dir := (wrist - elbow).normalized()
	var tip := wrist + hand_basis * tip_local
	if tip.distance_to(target) > REACHED_TOLERANCE_M:
		# out of reach (or limited): the hand continues along the forearm within the wrist limit towards the target
		h = _toward(fore_dir, (target - wrist).normalized() if target.distance_to(wrist) > 1e-9 else fore_dir, max_wrist)
		hand_basis = Basis(_arc(hand_rest.y, h)) * hand_rest
		tip = wrist + hand_basis * tip_local
	# the hand follows the forearm within the wrist limit whatever the iteration left: the tip then misses (reached false) instead of a bent wrist
	if fore_dir.angle_to(hand_basis.y) > max_wrist + 1e-6:
		hand_basis = Basis(_arc(hand_rest.y, _toward(fore_dir, hand_basis.y, max_wrist))) * hand_rest
		tip = wrist + hand_basis * tip_local
	var hand_dir: Vector3 = hand_basis.y
	return {"shoulder": shoulder, "elbow": elbow, "wrist": wrist, "tip": tip, "reached": tip.distance_to(target) <= REACHED_TOLERANCE_M,
			"upper_basis": Basis(_arc(upper_rest.y, upper_dir)) * upper_rest, "fore_basis": Basis(_arc(fore_rest.y, fore_dir)) * fore_rest,
			"hand_basis": hand_basis, "upper_dir": upper_dir, "fore_dir": fore_dir, "hand_dir": hand_dir, "wrist_bend_rad": fore_dir.angle_to(hand_dir)}


## Writes a solved pose to the skeleton: upper arm, forearm, hand and the three index bones (the other hand bones follow the hand).
## `extend` 0 = relaxed grip curl, 1 = index straight.
func apply(skeleton: Skeleton3D, result: Dictionary, extend: float) -> void:
	if _bones.is_empty():
		return
	var chain := [["upper", result["upper_basis"], result["shoulder"]], ["fore", result["fore_basis"], result["elbow"]], ["hand", result["hand_basis"], result["wrist"]]]
	for link: Array in chain:
		_set_global(skeleton, _bones[link[0]], Transform3D((link[1] as Basis).orthonormalized(), link[2]))
	var hand_t := Transform3D((result["hand_basis"] as Basis).orthonormalized(), result["wrist"])
	var curl := deg_to_rad(GRIP_CURL_DEG) * (1.0 - clampf(extend, 0.0, 1.0))
	var origin: Vector3 = hand_t * _index_origin_local
	var roles := ["index1", "index2", "index3"]
	for k in 3:
		var basis: Basis = hand_t.basis * (Basis(_curl_axis, curl * float(k + 1)) * _index_rest_local[k])
		_set_global(skeleton, _bones[roles[k]], Transform3D(basis.orthonormalized(), origin))
		origin += basis.y * _index_len[k]


## Back to the rest pose.
func reset_pose(skeleton: Skeleton3D) -> void:
	for role: String in ["upper", "fore", "hand", "index1", "index2", "index3"]:
		if _bones.has(role):
			skeleton.set_bone_pose(_bones[role], skeleton.get_bone_rest(_bones[role]))


func _set_global(skeleton: Skeleton3D, bone: int, global: Transform3D) -> void:
	var parent := skeleton.get_bone_parent(bone)
	var local := global if parent < 0 else skeleton.get_bone_global_pose(parent).orthonormalized().affine_inverse() * global
	skeleton.set_bone_pose_position(bone, local.origin)
	skeleton.set_bone_pose_rotation(bone, local.basis.get_rotation_quaternion())
	skeleton.set_bone_pose_scale(bone, Vector3.ONE)


## The elbow of the two-bone chain shoulder - wrist, bent towards `pole_dir`; the wrist distance is kept within the reachable range.
func _elbow(shoulder: Vector3, wrist: Vector3, pole_dir: Vector3) -> Vector3:
	var d := wrist - shoulder
	var dist := d.length()
	var dn := d / dist if dist > 1e-9 else upper_rest.y
	dist = clampf(dist, absf(upper_len - fore_len) + 1e-4, (upper_len + fore_len) * ELBOW_SLACK)
	var a := (upper_len * upper_len - fore_len * fore_len + dist * dist) / (2.0 * dist)
	var height := sqrt(maxf(upper_len * upper_len - a * a, 0.0))
	var bend := _perp(pole_dir, dn, pole_rest)
	return shoulder + dn * a + bend * height


## The unit component of `v` perpendicular to `axis` (falls back to `fallback`'s, then to any perpendicular).
func _perp(v: Vector3, axis: Vector3, fallback: Vector3) -> Vector3:
	var n := axis.normalized()
	for candidate: Vector3 in [v, fallback, Vector3.UP, Vector3.RIGHT]:
		var p := candidate - n * candidate.dot(n)
		if p.length() > 1e-6:
			return p.normalized()
	return Vector3.UP


## The minimal rotation from unit vector a to unit vector b.
static func _arc(a: Vector3, b: Vector3) -> Quaternion:
	var d := a.dot(b)
	if d < -0.999999:
		var any := Vector3.UP if absf(a.y) < 0.9 else Vector3.RIGHT
		return Quaternion(a.cross(any).normalized(), PI)
	return Quaternion(a, b)


## `from` turned towards `to` by at most `max_angle` (unit vectors).
static func _toward(from: Vector3, to: Vector3, max_angle: float) -> Vector3:
	var angle := from.angle_to(to)
	if angle <= max_angle or angle < 1e-9:
		return to
	var axis := from.cross(to)
	axis = axis.normalized() if axis.length() > 1e-9 else (Vector3.UP if absf(from.y) < 0.9 else Vector3.RIGHT).cross(from).normalized()
	return (Quaternion(axis, max_angle) * from).normalized()
