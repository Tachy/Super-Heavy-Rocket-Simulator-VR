"""Crew figure: the pure-Python layout shared by gen_crew_figure.py (Blender), build_models.py (manifest) and the tests.

No Blender import: plain tuples. Figure frame (config/cockpits/crew_figure.toml): origin = the seat's hip reference point (seat-back plane meets
the pan surface), X = up the seat back, Y = the seat's chest normal (forward), Z = the crew's left. `_l` parts are at +Z. The bones' rest pose is
the seated pose; `bones(cfg)` gives head / tail / parent of every bone in that frame.
"""
import math

SIDES = ("l", "r")
BONE_NAMES = ["hips", "spine", "chest", "neck", "head"] + [
    f"{n}_{s}" for s in SIDES for n in
    ("shoulder", "upper_arm", "forearm", "hand", "index_1", "index_2", "index_3", "thumb_1", "thumb_2", "fingers", "thigh", "shin", "foot")]
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
    """Forward offset (y) of the torso axis at height s above the hip joint."""
    return cfg["figure"]["hip_joint_forward_m"] + lerp_table(cfg["suit"]["torso_rings_m"], s)[2]


def seat_basis(back_angle_deg):
    """Columns (x, y, z) of the figure frame in the capsule frame for a seat with this back angle (config/cockpits/crew_dragon.toml)."""
    a = math.radians(back_angle_deg)
    return (math.cos(a), -math.sin(a), 0.0), (math.sin(a), math.cos(a), 0.0), (0.0, 0.0, 1.0)


def to_capsule(point, hip_m, back_angle_deg):
    bx, by, bz = seat_basis(back_angle_deg)
    return tuple(hip_m[i] + point[0] * bx[i] + point[1] * by[i] + point[2] * bz[i] for i in range(3))


def hand_frame(cfg, side):
    """(forward f, down d, toward-the-thumb t) of the hand of `side` (+1 left, -1 right) in the figure frame."""
    s = 1.0 if side == "l" else -1.0
    f = norm((cfg["pose"]["forearm_dir"][0], cfg["pose"]["forearm_dir"][1], cfg["pose"]["forearm_dir"][2] * s))
    d0 = (-1.0, 0.0, 0.0)
    d = norm(sub(d0, mul(f, dot(d0, f))))
    t0 = (0.0, 0.0, -s)  # the thumb points to the body centre
    t = norm(sub(sub(t0, mul(f, dot(t0, f))), mul(d, dot(t0, d))))
    return f, d, t


def curl(f, d, deg):
    a = math.radians(deg)
    return norm(add(mul(f, math.cos(a)), mul(d, math.sin(a))))


def chain_lengths(cfg):
    p, H = cfg["proportion"], cfg["figure"]["stature_m"]
    return {"upper_arm": p["upper_arm"] * H, "forearm": p["forearm"] * H, "thigh": (p["hip_height"] - p["knee_height"]) * H,
            "shin": (p["knee_height"] - p["ankle_height"]) * H, "torso": (p["shoulder_height"] - p["hip_height"]) * H}


def bones(cfg):
    """{name: {"head": (x, y, z), "tail": (x, y, z), "parent": name or None}} in the figure frame, rest pose = seated."""
    fig, pose, hand, helmet = cfg["figure"], cfg["pose"], cfg["hand"], cfg["helmet"]
    H, ln = fig["stature_m"], chain_lengths(cfg)
    hip = (fig["hip_joint_up_m"], fig["hip_joint_forward_m"], 0.0)
    shoulder_up = hip[0] + ln["torso"]
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
    ang = math.radians(fig["thigh_raise_deg"])
    thigh_dir = (math.sin(ang), math.cos(ang), 0.0)
    shin_ang = ang - math.radians(180.0 - fig["knee_angle_deg"])  # knee_angle_deg is the interior angle (180 = straight)
    shin_dir = (math.sin(shin_ang), math.cos(shin_ang), 0.0)
    foot_dir = (math.sin(shin_ang + math.pi / 2.0), math.cos(shin_ang + math.pi / 2.0), 0.0)
    for side in SIDES:
        s = 1.0 if side == "l" else -1.0
        tip = (shoulder_base[0], shoulder_base[1], s * H * cfg["proportion"]["shoulder_width"] / 2.0)
        put(f"shoulder_{side}", shoulder_base, tip, "chest")
        ua = norm((pose["upper_arm_dir"][0], pose["upper_arm_dir"][1], pose["upper_arm_dir"][2] * s))
        elbow = add(tip, mul(ua, ln["upper_arm"]))
        put(f"upper_arm_{side}", tip, elbow, f"shoulder_{side}")
        f, d, t = hand_frame(cfg, side)
        wrist = add(elbow, mul(f, ln["forearm"]))
        put(f"forearm_{side}", elbow, wrist, f"upper_arm_{side}")
        knuckle = add(wrist, mul(f, hand["palm_length_m"]))
        put(f"hand_{side}", wrist, knuckle, f"forearm_{side}")
        head, prev = add(knuckle, mul(t, hand["index_offset_m"])), f"hand_{side}"
        for k, seg in enumerate(hand["index_m"]):
            tail = add(head, mul(curl(f, d, hand["finger_curl_deg"] * (k + 1)), seg))
            put(f"index_{k + 1}_{side}", head, tail, prev)
            head, prev = tail, f"index_{k + 1}_{side}"
        fh = add(knuckle, mul(t, hand["fingers_offset_m"]))
        put(f"fingers_{side}", fh, add(fh, mul(curl(f, d, hand["finger_curl_deg"] * 2.0), hand["fingers_m"])), f"hand_{side}")
        sp = math.radians(hand["thumb_spread_deg"])
        tdir = norm(add(mul(f, math.cos(sp)), mul(t, math.sin(sp))))
        head, prev = add(add(wrist, mul(f, hand["thumb_root_forward_m"])), mul(t, hand["thumb_root_offset_m"])), f"hand_{side}"
        for k, seg in enumerate(hand["thumb_m"]):
            dirk = norm(add(mul(tdir, math.cos(math.radians(hand["finger_curl_deg"] * k))), mul(d, math.sin(math.radians(hand["finger_curl_deg"] * k)))))
            tail = add(head, mul(dirk, seg))
            put(f"thumb_{k + 1}_{side}", head, tail, prev)
            head, prev = tail, f"thumb_{k + 1}_{side}"
        hj = (hip[0], hip[1], s * fig["hip_joint_half_spacing_m"])
        knee = add(hj, mul(thigh_dir, ln["thigh"]))
        put(f"thigh_{side}", hj, knee, "hips")
        ankle = add(knee, mul(shin_dir, ln["shin"]))
        put(f"shin_{side}", knee, ankle, f"thigh_{side}")
        put(f"foot_{side}", ankle, add(ankle, mul(foot_dir, cfg["proportion"]["foot_length"] * H - cfg["suit"]["boot_heel_m"])), f"shin_{side}")
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


def figure_manifest(cfg):
    """The manifest's `crew_figure` block: file, frame convention, bone rest positions, materials, eye point."""
    r = lambda v: [round(c, 5) for c in v]
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
    }
