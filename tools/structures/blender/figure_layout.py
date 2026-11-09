"""Crew figure: the pure-Python layout shared by gen_crew_figure.py (Blender), build_models.py (manifest) and the tests.

No Blender import: plain tuples. Figure frame (config/cockpits/crew_figure.toml): origin = the seat's hip reference point (seat-back plane meets
the pan surface), X = up the seat back, Y = the seat's chest normal (forward), Z = the crew's left. `_l` parts are at +Z. The bones' rest pose is
the seated pose; `bones(cfg)` gives head / tail / parent of every bone in that frame.
"""
import math

SIDES = ("l", "r")
BONE_NAMES = ["hips", "spine", "chest", "neck", "head"] + [
    f"{n}_{s}" for s in SIDES for n in
    ("shoulder", "upper_arm", "forearm", "hand", "index_1", "index_2", "index_3", "thumb_1", "thumb_2", "fingers", "fingers_2", "fingers_3",
     "thigh", "shin", "foot")]
MATERIALS = ("suit", "boots", "gloves", "helmet", "collar", "visor")
EYE_UP_M, EYE_FORWARD_M = 0.03, 0.06  # [E] the eyes sit this far above / ahead of the helmet centre (behind the visor)
SPINE_SPLIT_M = (0.12, 0.28)  # [E] heights above the hip joint where hips / spine / chest bones meet


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def mul(a, k):
    return tuple(x * k for x in a)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def norm(a):
    n = math.sqrt(dot(a, a))
    return tuple(x / n for x in a)


def length(a):
    return math.sqrt(dot(a, a))


def lerp_table(rows, s):
    """Row [s, v0, v1, ...] interpolated at s (clamped)."""
    if s <= rows[0][0]:
        return tuple(rows[0][1:])
    for r0, r1 in zip(rows, rows[1:]):
        if s <= r1[0]:
            t = (s - r0[0]) / (r1[0] - r0[0])
            return tuple(a + (b - a) * t for a, b in zip(r0[1:], r1[1:]))
    return tuple(rows[-1][1:])


# ---------- helmet shell (Task 5b: compact faceted shell after spacex-anzug.jpg) ----------
# Helmet-centre frame: x up, y forward, z left (the figure frame shifted to [pose] head_centre_*). The shell is star-shaped about the centre:
# a superellipsoid (front / rear, above / below, half width with the egg taper; exponent shell_exponent) cut by flat facets (the rear face, the
# sloped upper rear facet, the two flat sides): rho(direction) = min(superellipsoid, facet planes). Elevation / azimuth are GEOMETRIC angles of
# the direction from the centre (az 0 = forward, + toward the left). The open bottom follows the sloped cut plane (rear high, chin low).

def helmet_direction(el_deg, az_deg):
    e, a = math.radians(el_deg), math.radians(az_deg)
    return (math.sin(e), math.cos(e) * math.cos(a), math.cos(e) * math.sin(a))


def helmet_planes(h):
    """Facet planes (unit normal, offset) in the helmet-centre frame: the shell keeps n . p <= offset."""
    return [((0.0, -1.0, 0.0), h["rear_flat_m"]), ((0.0, 0.0, 1.0), h["side_flat_m"]), ((0.0, 0.0, -1.0), h["side_flat_m"]),
            (norm(tuple(h["rear_facet_normal"])), h["rear_facet_offset_m"])]


def helmet_rho(h, d):
    """Distance from the helmet centre to the shell along the unit direction d."""
    p = h["shell_exponent"]
    rx = h["half_above_m"] if d[0] > 0.0 else h["half_below_m"]
    ry = h["half_depth_front_m"] if d[1] > 0.0 else h["half_depth_rear_m"]
    horiz = math.hypot(d[1], d[2])
    rz = h["half_width_m"] * (1.0 - h["egg_taper"] * (d[1] / horiz if horiz > 1e-9 else 0.0))
    rho = (abs(d[0] / rx) ** p + abs(d[1] / ry) ** p + abs(d[2] / rz) ** p) ** (-1.0 / p)
    for n, off in helmet_planes(h):
        c = dot(n, d)
        if c > 1e-9:
            rho = min(rho, off / c)
    return rho


def helmet_facet(h, d):
    """Which surface the direction d hits: 0 = the curved shell, 1.. = helmet_planes index + 1 (the tests check that every facet exists)."""
    rho = helmet_rho(h, d)
    for i, (n, off) in enumerate(helmet_planes(h)):
        c = dot(n, d)
        if c > 1e-9 and abs(off / c - rho) < 1e-9:
            return i + 1
    return 0


def helmet_point(h, el_deg, az_deg):
    """(point, outward unit normal) on the shell in the helmet-centre frame; the normal from the surface's own tangents."""
    def at(e, a):
        d = helmet_direction(e, a)
        return mul(d, helmet_rho(h, d))
    p = at(el_deg, az_deg)
    step = 0.05
    t_el = sub(at(min(el_deg + step, 89.99), az_deg), at(max(el_deg - step, -89.99), az_deg))
    t_az = sub(at(el_deg, az_deg + step), at(el_deg, az_deg - step))
    n = cross(t_az, t_el)
    if length(n) < 1e-12:
        return p, helmet_direction(el_deg, az_deg)
    n = norm(n)
    return p, (n if dot(n, p) > 0.0 else mul(n, -1.0))


def helmet_cut_x(h, y):
    """Height (x) of the sloped bottom cut at depth y: through cut_rear_m = [x, y], falling toward the front by cut_slope_deg."""
    x0, y0 = h["cut_rear_m"]
    return x0 - (y - y0) * math.tan(math.radians(h["cut_slope_deg"]))


def helmet_bottom_el(h, az_deg):
    """Lowest elevation of the shell at this azimuth: where the meridian meets the bottom cut (bisection)."""
    def f(e):
        p = helmet_point(h, e, az_deg)[0]
        return p[0] - helmet_cut_x(h, p[1])
    lo, hi = -89.0, 80.0
    if f(lo) >= 0.0:
        return lo
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if f(mid) >= 0.0:
            hi = mid
        else:
            lo = mid
    return hi


def visor_half_az(h, el_deg):
    """Half width (azimuth) of the face opening at this elevation: visor_outline rows [el_deg, half_az_deg], bottom to top."""
    return lerp_table(h["visor_outline"], el_deg)[0]


def visor_el_range(h):
    rows = h["visor_outline"]
    return rows[0][0], rows[-1][0]


def helmet_groove_block(cfg):
    """The manifest's helmet panel lines (surface detail, drawn by cabin.gdshader on the helmet surface): plane rows in the FIGURE frame."""
    h, pose = cfg["helmet"], cfg["pose"]
    c = (pose["head_centre_up_m"], pose["head_centre_forward_m"], 0.0)
    planes = []
    for row in h["grooves"]:
        n, m = norm(tuple(row[0:3])), tuple(row[4:7])
        planes.append([round(v, 6) for v in (*n, row[3] + dot(n, c), *m, row[7] + dot(m, c))])
    return {"planes": planes, "centre": [round(v, 6) for v in c], "radius_m": h["groove_radius_m"], "width_m": h["groove_width_m"],
            "albedo": h["groove_albedo"]}


def torso_forward(cfg, s):
    """Forward offset (y) of the torso axis at height s above the hip joint ([body] torso_levels_m: the sections' forward offsets)."""
    rows = [[r[0], r[5]] for r in cfg["body"]["torso_levels_m"]]
    return cfg["figure"]["hip_joint_forward_m"] + lerp_table(rows, s)[0]


def seat_basis(back_angle_deg):
    """Columns (x, y, z) of the figure frame in the capsule frame for a seat with this back angle (config/cockpits/crew_dragon.toml)."""
    a = math.radians(back_angle_deg)
    return (math.cos(a), -math.sin(a), 0.0), (math.sin(a), math.cos(a), 0.0), (0.0, 0.0, 1.0)


def to_capsule(point, hip_m, back_angle_deg):
    bx, by, bz = seat_basis(back_angle_deg)
    return tuple(hip_m[i] + point[0] * bx[i] + point[1] * by[i] + point[2] * bz[i] for i in range(3))


def _rotate(v, k, a):
    """v rotated by a (rad) about the unit axis k (Rodrigues)."""
    c, s = math.cos(a), math.sin(a)
    return add(add(mul(v, c), mul(cross(k, v), s)), mul(k, dot(k, v) * (1.0 - c)))


def leg_joints(cfg, side):
    """(hip joint, knee, ankle) of the seated rest pose of `side`. The leg is first laid in the sagittal plane (thigh_raise_deg above the
    seat-pan plane, the interior knee_angle_deg), then the knee swings OUT about the hip-ankle line by knee_out_deg (crew1_1: knees
    apart, the boots on the tray at hip width): the ankle and the knee angle stay, the knee moves outward and a little toward that line."""
    fig = cfg["figure"]
    ln = chain_lengths(cfg)
    s = 1.0 if side == "l" else -1.0
    hj = (fig["hip_joint_up_m"], fig["hip_joint_forward_m"], s * fig["hip_joint_half_spacing_m"])
    a = math.radians(fig["thigh_raise_deg"])
    sa = a - math.radians(180.0 - fig["knee_angle_deg"])  # knee_angle_deg is the interior angle (180 = straight)
    knee = add(hj, mul((math.sin(a), math.cos(a), 0.0), ln["thigh"]))
    ankle = add(knee, mul((math.sin(sa), math.cos(sa), 0.0), ln["shin"]))
    axis = norm(sub(ankle, hj))
    out = math.radians(fig.get("knee_out_deg", 0.0))
    swung = [add(hj, _rotate(sub(knee, hj), axis, sign * out)) for sign in (1.0, -1.0)]
    knee = max(swung, key=lambda p: s * p[2])  # the swing that moves the knee away from the body centre
    return hj, knee, ankle


def thigh_frame(cfg, side=None):
    """(thigh direction, the thigh's upper surface normal) of the seated rest pose. Without `side`: the sagittal (x-y plane) thigh before
    the knee swings out; with `side`: that leg's own thigh (knee_out_deg), the normal kept perpendicular to it."""
    a = math.radians(cfg["figure"]["thigh_raise_deg"])
    td, n = (math.sin(a), math.cos(a), 0.0), (math.cos(a), -math.sin(a), 0.0)
    if side is None:
        return td, n
    hj, knee, _ = leg_joints(cfg, side)
    td3 = norm(sub(knee, hj))
    return td3, norm(sub(n, mul(td3, dot(n, td3))))


def hand_frame(cfg, side):
    """(forward f, down d, toward-the-thumb t) of the hand of `side` in the seated rest pose: palm down on the thigh, the fingers toward
    the knee, turned hand_inward_deg toward the body centre and tilted palm_lift_deg down to the thigh (the wrist sits higher)."""
    s = 1.0 if side == "l" else -1.0
    pose = cfg["pose"]
    td, n = thigh_frame(cfg, side)
    inward = math.radians(pose["hand_inward_deg"])
    f = norm(add(mul(td, math.cos(inward)), mul((0.0, 0.0, -s), math.sin(inward))))
    lift = math.radians(pose["palm_lift_deg"])
    f = norm(add(mul(f, math.cos(lift)), mul(n, -math.sin(lift))))
    d0 = mul(n, -1.0)
    d = norm(sub(d0, mul(f, dot(d0, f))))
    t = norm(cross(d, f)) if s > 0 else norm(cross(f, d))  # the thumb toward the body centre (-s z)
    if dot(t, (0.0, 0.0, -s)) < 0.0:
        t = mul(t, -1.0)
    return f, d, t


def curl(f, d, deg):
    a = math.radians(deg)
    return norm(add(mul(f, math.cos(a)), mul(d, math.sin(a))))


def chain_lengths(cfg):
    """Segment lengths (m) from the ANSUR II medians ([anthropometry])."""
    a = cfg["anthropometry"]
    return {"upper_arm": a["upper_arm_length"], "forearm": a["forearm_length"], "thigh": a["trochanter_height"] - a["knee_height"],
            "shin": a["knee_height"] - a["ankle_height"], "torso": a["acromial_height"] - a["trochanter_height"]}


def shoulder_joint(cfg, side):
    """The glenohumeral joint centre (the upper arm bone's head) in the rest frame: shoulder_joint_drop_m under the acromion level,
    shoulder_joint_inset_m medial of the acromion (biacromial_breadth / 2)."""
    fig, ln = cfg["figure"], chain_lengths(cfg)
    s = 1.0 if side == "l" else -1.0
    up = ln["torso"] - fig["shoulder_joint_drop_m"]
    return (fig["hip_joint_up_m"] + up, torso_forward(cfg, up), s * (cfg["anthropometry"]["biacromial_breadth"] / 2.0 - fig["shoulder_joint_inset_m"]))


def two_bone_elbow(shoulder, wrist, l1, l2, pole):
    """Elbow of the chain shoulder - wrist (lengths l1, l2), bent toward `pole` (the wrist is pulled in if out of reach)."""
    d = sub(wrist, shoulder)
    dist = min(max(length(d), abs(l1 - l2) + 1e-4), (l1 + l2) * 0.9999)
    dn = norm(d)
    a = (l1 * l1 - l2 * l2 + dist * dist) / (2.0 * dist)
    h = math.sqrt(max(l1 * l1 - a * a, 0.0))
    bend = norm(sub(pole, mul(dn, dot(pole, dn))))
    return add(add(shoulder, mul(dn, a)), mul(bend, h))


def bones(cfg):
    """{name: {"head": (x, y, z), "tail": (x, y, z), "parent": name or None}} in the figure frame, rest pose = seated (hands on the thighs)."""
    fig, pose, hand, helmet = cfg["figure"], cfg["pose"], cfg["hand"], cfg["helmet"]
    ln = chain_lengths(cfg)
    hip = (fig["hip_joint_up_m"], fig["hip_joint_forward_m"], 0.0)
    out = {}

    def put(name, head, tail, parent):
        out[name] = {"head": tuple(head), "tail": tuple(tail), "parent": parent}

    def on_axis(s):
        return (hip[0] + s, torso_forward(cfg, s), 0.0)

    put("hips", hip, on_axis(SPINE_SPLIT_M[0]), None)
    put("spine", on_axis(SPINE_SPLIT_M[0]), on_axis(SPINE_SPLIT_M[1]), "hips")
    put("chest", on_axis(SPINE_SPLIT_M[1]), on_axis(ln["torso"]), "spine")
    centre = (pose["head_centre_up_m"], pose["head_centre_forward_m"], 0.0)
    neck_top = (centre[0] - helmet["half_below_m"], centre[1], 0.0)
    put("neck", on_axis(ln["torso"]), neck_top, "chest")
    put("head", neck_top, (centre[0] + helmet["half_above_m"], centre[1], 0.0), "neck")
    shoulder_base = on_axis(ln["torso"])
    td, _ = thigh_frame(cfg)
    ang = math.radians(fig["thigh_raise_deg"])
    shin_ang = ang - math.radians(180.0 - fig["knee_angle_deg"])  # knee_angle_deg is the interior angle (180 = straight)
    foot_dir = (math.sin(shin_ang + math.pi / 2.0), math.cos(shin_ang + math.pi / 2.0), 0.0)  # the boot stays flat on the tray
    for side in SIDES:
        s = 1.0 if side == "l" else -1.0
        joint = shoulder_joint(cfg, side)
        put(f"shoulder_{side}", shoulder_base, joint, "chest")
        hj, knee, ankle = leg_joints(cfg, side)
        # pose wrist_m is given on the sagittal thigh; the hand rides along when the knee swings out (the share of the knee's move at the
        # wrist's station along the thigh)
        wrist = (pose["wrist_m"][0], pose["wrist_m"][1], s * pose["wrist_m"][2])
        knee0 = add(hj, mul(td, ln["thigh"]))
        share = max(0.0, min(1.0, dot(sub(wrist, hj), td) / ln["thigh"]))
        wrist = add(wrist, mul(sub(knee, knee0), share))
        pole = (pose["elbow_pole"][0], pose["elbow_pole"][1], s * pose["elbow_pole"][2])
        elbow = two_bone_elbow(joint, wrist, ln["upper_arm"], ln["forearm"], pole)
        wrist = add(elbow, mul(norm(sub(wrist, elbow)), ln["forearm"]))
        put(f"upper_arm_{side}", joint, elbow, f"shoulder_{side}")
        put(f"forearm_{side}", elbow, wrist, f"upper_arm_{side}")
        f, d, t = hand_frame(cfg, side)
        knuckle = add(wrist, mul(f, hand["palm_length_m"]))
        put(f"hand_{side}", wrist, knuckle, f"forearm_{side}")
        head, prev = add(knuckle, mul(t, hand["index_offset_m"])), f"hand_{side}"
        for k, seg in enumerate(hand["index_m"]):
            tail = add(head, mul(curl(f, d, hand["finger_curl_deg"] * (k + 1)), seg))
            put(f"index_{k + 1}_{side}", head, tail, prev)
            head, prev = tail, f"index_{k + 1}_{side}"
        head, prev = add(knuckle, mul(t, hand["fingers_offset_m"])), f"hand_{side}"
        for k, (name, seg) in enumerate(zip(("fingers", "fingers_2", "fingers_3"), hand["fingers_m"])):
            tail = add(head, mul(curl(f, d, hand["finger_curl_deg"] * (k + 1)), seg))
            put(f"{name}_{side}", head, tail, prev)
            head, prev = tail, f"{name}_{side}"
        sp = math.radians(hand["thumb_spread_deg"])
        tdir = norm(add(mul(f, math.cos(sp)), mul(t, math.sin(sp))))
        head, prev = add(add(wrist, mul(f, hand["thumb_root_forward_m"])), mul(t, hand["thumb_root_offset_m"])), f"hand_{side}"
        for k, seg in enumerate(hand["thumb_m"]):
            dirk = norm(add(mul(tdir, math.cos(math.radians(hand["finger_curl_deg"] * k))), mul(d, math.sin(math.radians(hand["finger_curl_deg"] * k)))))
            tail = add(head, mul(dirk, seg))
            put(f"thumb_{k + 1}_{side}", head, tail, prev)
            head, prev = tail, f"thumb_{k + 1}_{side}"
        put(f"thigh_{side}", hj, knee, "hips")
        put(f"shin_{side}", knee, ankle, f"thigh_{side}")
        put(f"foot_{side}", ankle, add(ankle, mul(foot_dir, cfg["suit"]["boot_length_m"] - cfg["suit"]["boot_heel_m"])), f"shin_{side}")
    assert sorted(out) == sorted(BONE_NAMES), sorted(set(out) ^ set(BONE_NAMES))
    return out


def arm_reach_m(cfg, side="l"):
    """Shoulder joint to the index tip along the bones (upper arm, forearm, hand, three index phalanges)."""
    b = bones(cfg)
    names = [f"upper_arm_{side}", f"forearm_{side}", f"hand_{side}", f"index_1_{side}", f"index_2_{side}", f"index_3_{side}"]
    return sum(length(sub(b[n]["tail"], b[n]["head"])) for n in names)


def eye_point(cfg):
    pose = cfg["pose"]
    return (pose["head_centre_up_m"] + EYE_UP_M, pose["head_centre_forward_m"] + EYE_FORWARD_M, 0.0)


# ---------- joints: bend angles, wrinkle and corrective weights (figure_body.py bakes, figure_deform.gd evaluates) ----------
# Each joint: the child bone, the reference (a parent bone's direction, or the torso's down axis = minus a torso bone's direction) and the
# [joints] row [neutral, full, extension limit] in degrees. The wrinkle slot orders the instance uniforms of crew_suit.gdshader.
JOINT_KINDS = ("shoulder", "elbow", "hip", "knee")
JOINT_BONES = {"shoulder": ("upper_arm", "chest", -1.0), "elbow": ("forearm", "upper_arm", 1.0), "hip": ("thigh", "hips", -1.0), "knee": ("shin", "thigh", 1.0)}


def joint_names():
    return [f"{k}_{s}" for s in SIDES for k in JOINT_KINDS]


def _tail_bone(b, name):
    """The child bone whose head is this bone's tail (the game measures a bone's direction head -> that child's head)."""
    for n in BONE_NAMES:
        if b[n]["parent"] == name and length(sub(b[n]["head"], b[name]["tail"])) < 1e-9:
            return n
    raise ValueError(f"no child at the tail of {name}")


SHOULDER_BAKE_OUT = 0.25  # [E] the shoulder's flex shape is baked for a raise toward the console: forward plus this share outward
SWING_GATE_DEG = (25.0, 65.0)  # [E] a swing joint's flex shape acts fully within 25 deg of its baked swing direction and not beyond 65 deg


def bake_direction(kind, side, ref, child_dir):
    """Unit direction (perpendicular to `ref`) toward which the joint's flex shape is baked: shoulder = forward and a little out (the raise
    to the console), hip = the thigh forward, elbow / knee = the hinge's own bend (the child's direction)."""
    s = 1.0 if side == "l" else -1.0
    plane = {"shoulder": (0.0, 1.0, SHOULDER_BAKE_OUT * s), "hip": (0.0, 1.0, 0.0)}.get(kind, child_dir)
    return norm(sub(plane, mul(ref, dot(plane, ref))))


def swing_gate(joint, swing_cos):
    """Factor on a SWING joint's (shoulder, hip) flex shape: the shape is baked for one swing direction, so it acts only near it.
    swing_cos = cos of the angle between the current swing (child direction minus its part along the reference) and the baked one;
    1 within SWING_GATE_DEG[0], 0 beyond SWING_GATE_DEG[1], smoothstep between. Hinges (elbow, knee) are not gated."""
    if not joint.get("gated", False):
        return 1.0
    lo, hi = math.cos(math.radians(SWING_GATE_DEG[1])), math.cos(math.radians(SWING_GATE_DEG[0]))
    t = max(0.0, min(1.0, (swing_cos - lo) / (hi - lo)))
    return t * t * (3.0 - 2.0 * t)


def joint_table(cfg):
    """{joint name: {child, ref, ref_sign, neutral_deg, full_deg, ext_deg, rest_deg, slot, shape_flex, shape_ext, plane, gated}} (slot 0..7:
    l side first). plane = the baked swing direction in the figure frame (bake_direction), gated = a swing joint (swing_gate)."""
    b = bones(cfg)
    out = {}
    for slot, name in enumerate(joint_names()):
        kind, side = name.rsplit("_", 1)
        child, ref, sign = JOINT_BONES[kind]
        child_bone = f"{child}_{side}"
        ref_bone = ref if ref in ("chest", "hips") else f"{ref}_{side}"
        neutral, full, ext = cfg["joints"][f"{kind}_deg"]
        rd = mul(norm(sub(b[ref_bone]["tail"], b[ref_bone]["head"])), sign)
        cd = norm(sub(b[child_bone]["tail"], b[child_bone]["head"]))
        rest = math.degrees(math.acos(max(-1.0, min(1.0, dot(rd, cd)))))
        out[name] = {"child": child_bone, "child_tail": _tail_bone(b, child_bone), "ref": ref_bone, "ref_tail": _tail_bone(b, ref_bone),
                     "ref_sign": sign, "neutral_deg": neutral, "full_deg": full, "ext_deg": ext,
                     "rest_deg": rest, "slot": slot, "shape_flex": f"{name}_flex", "shape_ext": f"{name}_ext",
                     "plane": list(bake_direction(kind, side, rd, cd)), "gated": kind in ("shoulder", "hip")}
    return out


def bend_weights(joint, theta_deg):
    """(flex, ext, wrinkle) weights of a joint at bend angle theta: the corrective shapes ramp from the rest angle to the full bend /
    the extension limit; the wrinkle weight is 0 at the neutral (straight) angle and 1 at the full bend."""
    rest, full, ext, neutral = joint["rest_deg"], joint["full_deg"], joint["ext_deg"], joint["neutral_deg"]
    clamp = lambda v: max(0.0, min(1.0, v))
    flex = clamp((theta_deg - rest) / (full - rest)) if full > rest else 0.0
    extw = clamp((rest - theta_deg) / (rest - ext)) if rest > ext else 0.0
    wrinkle = clamp((theta_deg - neutral) / (full - neutral))
    return flex, extw, wrinkle


def figure_manifest(cfg):
    """The manifest's `crew_figure` block: file, frame convention, bone rest positions, materials, eye point, joints, suit maps."""
    r = lambda v: [round(c, 5) for c in v]
    def rnd(v):
        return round(v, 4) if isinstance(v, float) else ([round(c, 5) for c in v] if isinstance(v, list) else v)
    joints = {n: {k: rnd(v) for k, v in j.items()} for n, j in joint_table(cfg).items()}
    look = cfg["suit_look"]
    return {
        "file": "crew_figure",
        "node": "crew_figure",
        "armature": "crew_figure_rig",
        "stature_m": cfg["figure"]["stature_m"],
        "frame": ("origin = the seat's hip reference point ([[seat]] hip_m); x = up the seat back, y = the seat's chest normal, z = the crew's left; "
                  "place at a seat: basis columns ((cos a, -sin a, 0), (sin a, cos a, 0), (0, 0, 1)) in the capsule frame, a = back_angle_deg"),
        "bones": {n: {"head": r(b["head"]), "tail": r(b["tail"]), "parent": b["parent"] or ""} for n, b in bones(cfg).items()},
        "bone_names": list(BONE_NAMES),
        "eye_m": r(eye_point(cfg)),
        "materials": {m: {"albedo": cfg["material"][m]["albedo"], "roughness": cfg["material"][m]["roughness"], "metallic": cfg["material"][m]["metallic"]}
                      for m in MATERIALS},
        "visor_material": "visor",
        "visor_tint": {"split": cfg["helmet"]["visor_tint_split"], "albedo_low": cfg["helmet"]["visor_low_albedo"],
                       "roughness_low": cfg["helmet"]["visor_low_roughness"], "metallic_low": cfg["helmet"]["visor_low_metallic"]},
        "helmet_material": "helmet",
        "helmet_grooves": helmet_groove_block(cfg),
        "suit_material": "suit",
        "suit_maps": {"albedo": SUIT_ALBEDO_FILE, "wrinkle": SUIT_WRINKLE_FILE, "atlas_m": list(look["atlas_m"]),
                      "fold_depth_m": cfg["joints"]["fold_depth_m"], "slope_range": WRINKLE_SLOPE_RANGE},
        "joints": joints,
        "joint_order": joint_names(),
    }


SUIT_ALBEDO_FILE = "suit_albedo.png"
SUIT_WRINKLE_FILE = "suit_wrinkle.png"
WRINKLE_SLOPE_RANGE = 1.6  # the wrinkle map stores the fold slopes dh/du, dh/dv (m/m) as 0.5 + slope / WRINKLE_SLOPE_RANGE
