"""Crew body and suit hull (Task 8b): pure Python + numpy, shared by gen_crew_figure.py (Blender), suit_textures.py (the maps) and the tests.

Our own procedural mesh (no third-party vertex data). Measures: ANSUR II male medians (crew_figure.toml [anthropometry] / [body]). Topology:
the edge-loop layout of the Blender Studio Human Base Meshes (CC0, reference only): one torso tube whose bottom ring splits into two leg
loops along a crotch bridge, an arm port cut out of the torso's side between the axilla and the shoulder top (its loop runs down the
anterior axillary fold, under the axilla, up the posterior fold and over the shoulder), transition rings from each port to the limb
(deltoid cap / gluteal fold), dense loops at the elbow and the knee.

Pipeline:
1. The BUILD pose: the rest skeleton (figure_layout.bones: seated, hands on the thighs) with the arms straightened in an A-pose and the
   legs straight down (`build_pose`: one rigid transform G per bone, rest -> build). The hull is modelled there, where every limb is a
   straight tube, and skinned with smooth weights (2 bones across each joint).
2. The REST mesh (exported): the build hull posed into the seated rest by dual-quaternion skinning (no joint collapse).
3. Corrective shapes per joint (`shapes`): the game skins linearly (LBS, glTF); for a joint bent to its full flexion (or extended to its
   limit) the shape is the bind-space offset that makes the LBS result equal the dual-quaternion pose of the build hull: elbows and knees
   keep their volume instead of collapsing. Weighted by the bend angle (figure_layout.bend_weights).
4. Wrinkle mask per vertex: the joint slot and a 0..1 mask around each joint (the shader blends the baked fold map by the joint's bend).

`hull(cfg, ease=True)` is the suit (the body sections plus the [suit] ease); `hull(cfg, ease=False)` the anatomical body (measured by the
tests; not exported: the suit covers it). Frames as figure_layout (x up the seat back, y forward, z the crew's left).
"""
import math

import numpy as np

import figure_layout as fl

ARM_STATIONS = (0.10, 0.16, 0.22, 0.28)  # [E] upper-arm rings (t from the shoulder joint) before the elbow loops
ELBOW_LOOPS = (-0.05, -0.025, 0.0, 0.025, 0.05)  # [E] five loops around the elbow (base-mesh layout: three plus two support loops)
FOREARM_STATIONS = (0.43, 0.50, 0.56)  # [E] forearm rings before the wrist ring
LEG_STATIONS = (0.10, 0.16, 0.22, 0.30)
KNEE_LOOPS = (-0.06, -0.03, 0.0, 0.03, 0.06)
ELBOW_BLEND_M = 0.07  # [E] half width of the upper arm -> forearm weight blend (fix round 1: 0.05 left a hard inner-elbow crease at 90 deg)
SHIN_STATIONS = (0.53, 0.62, 0.71, 0.80)
TUBE_TAIL_M = 0.03  # [E] the sleeve / leg runs on past the wrist / ankle into the glove lock / boot (the wrist ring is no end ring)
CROTCH_SAG_M = 0.012  # [E] the crotch bridge dips this far below the bottom ring
ARM_ATLAS = {"torso": (0.02, 0.02, 1.10), "arm_l": (0.02, 0.80, 0.40), "arm_r": (0.46, 0.80, 0.40), "leg_l": (1.14, 0.02, 0.66), "leg_r": (1.84, 0.02, 0.66)}
CAP_UV_M = (0.01, 0.01)  # an unpainted (white) corner of the atlas for the hidden end caps
SHAPE_EPS_M = 2.0e-5  # corrective offsets below this are dropped (zero)


# ---------- small geometry ----------

def _n(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def smoothstep(e0, e1, x):
    t = np.clip((np.asarray(x, dtype=float) - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def rot(axis, angle):
    """3x3 rotation about the unit axis by angle (rad)."""
    a = _n(axis)
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def min_rot(u, v):
    """The smallest rotation taking direction u to direction v."""
    u, v = _n(u), _n(v)
    c = float(np.clip(u @ v, -1.0, 1.0))
    axis = np.cross(u, v)
    if np.linalg.norm(axis) < 1e-12:
        if c > 0.0:
            return np.eye(3)
        axis = np.cross(u, [1.0, 0.0, 0.0]) if abs(u[0]) < 0.9 else np.cross(u, [0.0, 1.0, 0.0])
    return rot(axis, math.acos(c))


def rigid(r, pivot):
    """4x4: rotation r about the point pivot."""
    t = np.eye(4)
    t[:3, :3] = r
    t[:3, 3] = np.asarray(pivot, dtype=float) - r @ np.asarray(pivot, dtype=float)
    return t


def apply(t, p):
    p = np.asarray(p, dtype=float)
    return p @ t[:3, :3].T + t[:3, 3]


def superellipse_perimeter(a, b, p, n=720):
    th = np.linspace(0.0, 2.0 * math.pi, n + 1)
    c, s = np.cos(th), np.sin(th)
    x = a * np.sign(c) * np.abs(c) ** (2.0 / p)
    y = b * np.sign(s) * np.abs(s) ** (2.0 / p)
    return float(np.sum(np.hypot(np.diff(x), np.diff(y))))


def fit_exponent(a, b, girth):
    """Superellipse exponent (2 = ellipse .. 14 = near box) whose perimeter with half axes a, b equals girth (clamped)."""
    lo, hi = 1.6, 14.0
    if superellipse_perimeter(a, b, hi) <= girth:
        return hi
    if superellipse_perimeter(a, b, lo) >= girth:
        return lo
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        if superellipse_perimeter(a, b, mid) < girth:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def fit_half_width(b, girth):
    """Half width a of the ellipse with half depth b and this perimeter."""
    lo, hi = 0.2 * b, 4.0 * b
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if superellipse_perimeter(mid, b, 2.0) < girth:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


# ---------- skeleton: the build pose ----------

def descendants(bones, name):
    out, stack = [], [name]
    while stack:
        n = stack.pop()
        out.append(n)
        stack.extend(c for c, b in bones.items() if b["parent"] == n)
    return out


def build_pose(cfg):
    """{bone: 4x4 G} (rest -> build pose) and the build bones {name: (head, tail)}. Torso as rest; per side the arm straight in an A-pose
    (build_arm_abduction_deg out from the torso's down axis, in the x-z plane) and the leg straight down (build_leg_abduction_deg out)."""
    bones = fl.bones(cfg)
    g = {n: np.eye(4) for n in bones}
    body = cfg["body"]

    def pos(name, end):
        return apply(g[name], np.array(bones[name][end]))

    def align(name, target_dir):
        head, tail = pos(name, "head"), pos(name, "tail")
        t = rigid(min_rot(tail - head, target_dir), head)
        for d in descendants(bones, name):
            g[d] = t @ g[d]

    for side in fl.SIDES:
        s = 1.0 if side == "l" else -1.0
        a = math.radians(body["build_arm_abduction_deg"])
        arm = np.array([-math.cos(a), 0.0, s * math.sin(a)])
        for name in ("upper_arm", "forearm", "hand"):
            align(f"{name}_{side}", arm)
        lg = math.radians(body["build_leg_abduction_deg"])
        leg = np.array([-math.cos(lg), 0.0, s * math.sin(lg)])
        for name in ("thigh", "shin"):
            align(f"{name}_{side}", leg)
    built = {n: (pos(n, "head"), pos(n, "tail")) for n in bones}
    return g, built


# ---------- the hull ----------

class Hull:
    """Vertices (build pose), faces (vertex index tuples, outward winding), per-face part and corner UVs (atlas metres, v down the image),
    per-vertex bone weights {bone: w} and wrinkle (slot, mask)."""

    def __init__(self):
        self.v, self.w, self.mask = [], [], []
        self.faces, self.part, self.uv = [], [], []
        self.ports = {}  # part -> the root loop's vertex indices

    def add(self, p, weights, mask=(-1, 0.0)):
        self.v.append(np.asarray(p, dtype=float))
        tot = sum(weights.values())
        self.w.append({k: x / tot for k, x in weights.items() if x / tot > 1e-4})
        self.mask.append((int(mask[0]), float(mask[1])))
        return len(self.v) - 1

    def face(self, idx, uv, part, ref):
        """A face oriented so its normal points away from the reference point `ref` (the local axis)."""
        p = [self.v[i] for i in idx]
        nrm = np.cross(p[1] - p[0], p[2] - p[0]) if len(p) == 3 else np.cross(p[2] - p[0], p[3] - p[1])
        if nrm @ (sum(p) / len(p) - np.asarray(ref)) < 0.0:
            idx, uv = idx[::-1], uv[::-1]
        self.faces.append(tuple(idx))
        self.uv.append([tuple(c) for c in uv])
        self.part.append(part)

    def positions(self):
        return np.array(self.v)

    def triangles(self, parts=None):
        tris = []
        for f, pt in zip(self.faces, self.part):
            if parts is not None and pt not in parts:
                continue
            tris.append((f[0], f[1], f[2]))
            if len(f) == 4:
                tris.append((f[0], f[2], f[3]))
        return np.array(tris, dtype=int)


def torso_sections(cfg, ease, k=None):
    """Rows (s, hz, hf, hb, p, yc) of the torso levels; the exponent fitted to the ANSUR girth (body, without ease) where given, times the
    row's factor k (girth_factors: what the subdivision takes off)."""
    rows = cfg["body"]["torso_levels_m"]
    fig = cfg["figure"]
    k = k or [1.0] * len(rows)
    ps = []
    for (s, half, depth, back, girth, fwd), f in zip(rows, k):
        ps.append(fit_exponent(half, depth / 2.0, girth * f) if girth > 0.0 else None)
    known = [(r[0], p) for r, p in zip(rows, ps) if p is not None]
    out = []
    for (s, half, depth, back, girth, fwd), p in zip(rows, ps):
        if p is None:
            p = fl.lerp_table([[k, v] for k, v in known], s)[0]
        out.append((s, half + ease, depth * (1.0 - back) + ease, depth * back + ease, p, fig["hip_joint_forward_m"] + fwd))
    return out


def _fit_rows(rows, k=None):
    """Limb sections with the girth (times the row's factor k) fitted: [(t, a, b)]."""
    out = []
    for (t, a, b, girth), f in zip(rows, k or [1.0] * len(rows)):
        out.append((t, fit_half_width(b, girth * f) if girth > 0.0 else a, b))
    return out


def _section(rows, t):
    return fl.lerp_table([list(r) for r in rows], t)


def torso_point(sec, phi, hip_up):
    s, hz, hf, hb, p, yc = sec
    c, sn = math.cos(phi), math.sin(phi)
    z = hz * math.copysign(abs(sn) ** (2.0 / p), sn)
    y = yc + (hf if c >= 0.0 else hb) * math.copysign(abs(c) ** (2.0 / p), c)
    return np.array([hip_up + s, y, z])


def _torso_weights(s):
    a, b = fl.SPINE_SPLIT_M
    w_spine = float(smoothstep(a - 0.05, a + 0.05, s))
    w_chest = float(smoothstep(b - 0.05, b + 0.05, s))
    return {"hips": 1.0 - w_spine, "spine": w_spine - w_chest, "chest": w_chest}


def _ring_angles(points, centre, e1, e2):
    ang = np.array([math.atan2((p - centre) @ e2, (p - centre) @ e1) for p in points])
    return np.unwrap(ang)


def _uniform_angles(ang):
    n = len(ang)
    sign = 1.0 if ang[-1] > ang[0] else -1.0
    k = np.arange(n)
    off = math.atan2(np.mean(np.sin(ang - sign * 2.0 * math.pi * k / n)), np.mean(np.cos(ang - sign * 2.0 * math.pi * k / n)))
    return off + sign * 2.0 * math.pi * k / n, sign


def hull(cfg, ease=True, factors=None):
    """(Hull, layout): the suit (ease) or the body hull in the build pose; layout = the atlas / joint-zone facts suit_textures.py paints by."""
    body, suit, fig = cfg["body"], cfg["suit"], cfg["figure"]
    g, built = build_pose(cfg)
    ln = fl.chain_lengths(cfg)
    hip_up = fig["hip_joint_up_m"]
    n_t = body["torso_columns"]
    factors = factors or {}
    secs = torso_sections(cfg, suit["ease_torso_m"] if ease else 0.0, factors.get("torso"))
    h = Hull()
    layout = {"atlas_m": list(cfg["suit_look"]["atlas_m"]), "parts": {}, "zones": []}
    s_min = secs[0][0]
    tx, ty, tu = ARM_ATLAS["torso"]
    layout["parts"]["torso"] = {"origin": [tx, ty], "u_width": tu, "v0_s": s_min, "levels": [r[0] for r in secs], "columns": n_t,
                                "u_front": 0.5, "u_left": 0.75, "u_right": 0.25}
    # torso grid
    grid = []
    for sec in secs:
        grid.append([h.add(torso_point(sec, 2.0 * math.pi * c / n_t, hip_up), _torso_weights(sec[0])) for c in range(n_t)])
    rows = len(secs)
    r_ax, r_top = body["arm_port_rows"]
    pc = body["arm_port_columns"]
    port_cols = {"l": n_t // 4, "r": 3 * n_t // 4}

    def in_port(r, c):
        return r_ax <= r < r_top and any(port_cols[s] - pc <= c < port_cols[s] + pc for s in fl.SIDES)

    # u around the torso = the ARC-LENGTH share of the row's ring from the back centre (u 0) over the right side to the front (u 0.5): the
    # columns sit at equal angles, which crowds them at the corners of the near-box sections, so a column-index u squeezed the painted panels
    # (metres along tu) into the corners (fix round 1: the side panels and the chevron did not read)
    arc = []
    for r in range(rows):
        ring = [h.v[grid[r][(n_t // 2 + k) % n_t]] for k in range(n_t + 1)]
        seg = np.array([np.linalg.norm(ring[k + 1] - ring[k]) for k in range(n_t)])
        arc.append(np.concatenate([[0.0], np.cumsum(seg)]) / seg.sum())

    def torso_uv(r, c, closing=False):
        return 1.0 if closing else float(arc[r][(c - n_t // 2) % n_t])

    for r in range(rows - 1):
        for c in range(n_t):
            if in_port(r, c):
                continue
            c1 = (c + 1) % n_t
            closing = c1 == n_t // 2
            u0, u0b = torso_uv(r, c), torso_uv(r + 1, c)
            u1, u1b = torso_uv(r, c1, closing), torso_uv(r + 1, c1, closing)
            v0, v1 = secs[r][0] - s_min, secs[r + 1][0] - s_min
            idx = (grid[r][c], grid[r][c1], grid[r + 1][c1], grid[r + 1][c])
            uv = [(tx + u0 * tu, ty + v0), (tx + u1 * tu, ty + v0), (tx + u1b * tu, ty + v1), (tx + u0b * tu, ty + v1)]
            axis = np.array([hip_up + 0.5 * (secs[r][0] + secs[r + 1][0]), 0.5 * (secs[r][5] + secs[r + 1][5]), 0.0])
            h.face(idx, uv, "torso", axis)
    top = h.add(np.array([hip_up + secs[-1][0] + 0.01, secs[-1][5], 0.0]), _torso_weights(secs[-1][0]))
    for c in range(n_t):
        h.face((grid[-1][c], grid[-1][(c + 1) % n_t], top), [CAP_UV_M] * 3, "torso", np.array([hip_up + secs[-1][0] - 0.05, secs[-1][5], 0.0]))
    # the crotch bridge (shared by both leg loops), back to front
    nb = body["crotch_bridge"]
    back, front = h.v[grid[0][n_t // 2]], h.v[grid[0][0]]
    bridge = []
    for j in range(1, nb + 1):
        f = j / (nb + 1)
        p = back + (front - back) * f
        p = p - np.array([CROTCH_SAG_M * math.sin(math.pi * f), 0.0, 0.0])
        p[2] = 0.0
        bridge.append(h.add(p, {"hips": 1.0}))
    jt = fl.joint_table(cfg)
    for side in fl.SIDES:
        s = 1.0 if side == "l" else -1.0
        # ---- arm ----
        cc = port_cols[side]
        loop = [grid[r_ax][c] for c in range(cc - pc, cc + pc + 1)]
        loop += [grid[r][cc + pc] for r in range(r_ax + 1, r_top + 1)]
        loop += [grid[r_top][c] for c in range(cc + pc - 1, cc - pc - 1, -1)]
        loop += [grid[r][cc - pc] for r in range(r_top - 1, r_ax, -1)]
        loop = loop[pc:] + loop[:pc]  # index 0 = the middle of the axilla edge (the arm's seam under the arm)
        h.ports[f"arm_{side}"] = loop
        joint = built[f"upper_arm_{side}"][0]
        axis = _n(built[f"upper_arm_{side}"][1] - joint)
        e1 = _n(np.array([1.0, 0.0, 0.0]) - axis * axis[0])
        e2 = np.cross(axis, e1)
        thumb = g[f"hand_{side}"][:3, :3] @ np.array(fl.hand_frame(cfg, side)[2])
        ua = _n(thumb - axis * (thumb @ axis))
        ub = np.cross(axis, ua)
        stations = list(ARM_STATIONS) + [ln["upper_arm"] + d for d in ELBOW_LOOPS] + list(FOREARM_STATIONS) + [ln["upper_arm"] + ln["forearm"], ln["upper_arm"] + ln["forearm"] + TUBE_TAIL_M]
        rows_a = _fit_rows(body["arm_sections_m"], factors.get("arm"))
        elbow_t = ln["upper_arm"]

        def arm_ease(t):
            if not ease:
                return 0.0
            k = float(smoothstep(elbow_t - 0.05, elbow_t + 0.08, t))
            return suit["ease_upper_arm_m"] * (1.0 - k) + suit["ease_forearm_m"] * k

        def arm_ring(t, angles):
            a_half, b_half = _section(rows_a, t)
            e = arm_ease(t)
            a_half, b_half = a_half + e, b_half + e
            out = []
            for th in angles:
                d = math.cos(th) * e1 + math.sin(th) * e2
                r = 1.0 / math.sqrt((d @ ua) ** 2 / a_half ** 2 + (d @ ub) ** 2 / b_half ** 2)
                out.append(joint + axis * t + d * r)
            return out

        ports = [h.v[i] for i in loop]
        ang = _ring_angles(ports, joint, e1, e2)
        uni, _ = _uniform_angles(ang)
        first = arm_ring(stations[0], uni)
        delt = cfg["body"]["deltoid_radius_m"] + (suit["ease_upper_arm_m"] if ease else 0.0)
        nk = body["port_transition_rings"]
        rings, ring_t = [loop], [None]
        sl = jt[f"shoulder_{side}"]["slot"]
        el = jt[f"elbow_{side}"]["slot"]
        reach = cfg["joints"]["fold_reach_m"]
        for j in range(1, nk + 1):
            f = j / (nk + 1)
            ids = []
            for k in range(len(loop)):
                p = ports[k] * (1.0 - f) + first[k] * f
                q = p - joint
                up = float(smoothstep(0.2, 0.8, (q / np.linalg.norm(q)) @ np.array([0.0, 0.0, s])))  # the deltoid rounds the shoulder laterally
                ln_q = np.linalg.norm(q)
                if ln_q < delt:
                    p = joint + q / ln_q * (ln_q + (delt - ln_q) * up)
                w = float(smoothstep(0.0, 1.0, f))
                ids.append(h.add(p, {"chest": 1.0 - w, f"upper_arm_{side}": w}, (sl, 1.0)))
            rings.append(ids)
            ring_t.append(None)
        for t in stations:
            w_fa = float(smoothstep(elbow_t - ELBOW_BLEND_M, elbow_t + ELBOW_BLEND_M, t))
            w_ua = 1.0 - w_fa
            w_ch = 0.0 if t > stations[0] + 1e-9 else 0.06
            m_sh = 1.0 - float(smoothstep(0.12, 0.22, t))
            m_el = 1.0 - float(smoothstep(0.6 * reach[1], reach[1], abs(t - elbow_t)))
            mask = (sl, m_sh) if m_sh > 0.0 else ((el, m_el) if m_el > 0.0 else (-1, 0.0))
            rings.append([h.add(p, {"chest": w_ch, f"upper_arm_{side}": w_ua * (1.0 - w_ch), f"forearm_{side}": w_fa * (1.0 - w_ch)}, mask)
                          for p in arm_ring(t, uni)])
            ring_t.append(t)
        _tube(h, rings, f"arm_{side}", ARM_ATLAS[f"arm_{side}"], cap_weights={f"forearm_{side}": 1.0})
        u_of = _u_of(e1, e2, uni)
        out_dir = np.array([0.0, 0.0, s])
        rest_ua = np.array(fl.bones(cfg)[f"upper_arm_{side}"]["tail"]) - np.array(fl.bones(cfg)[f"upper_arm_{side}"]["head"])
        rest_fa = np.array(fl.bones(cfg)[f"forearm_{side}"]["tail"]) - np.array(fl.bones(cfg)[f"forearm_{side}"]["head"])
        inner = _n(rest_fa - _n(rest_ua) * (_n(rest_ua) @ rest_fa))
        inner_build = g[f"upper_arm_{side}"][:3, :3] @ inner
        v_st = _ring_v(h, rings)
        layout["parts"][f"arm_{side}"] = {"origin": list(ARM_ATLAS[f"arm_{side}"][:2]), "u_width": ARM_ATLAS[f"arm_{side}"][2],
                                         "u_out": u_of(out_dir), "u_front": u_of(np.array([0.0, 1.0, 0.0])), "u_back": u_of(np.array([0.0, -1.0, 0.0])),
                                         "u_in": u_of(-out_dir), "u_elbow_inner": u_of(inner_build),
                                         "v_port_end": v_st[nk + 1], "v_elbow": _v_at(v_st, ring_t, elbow_t), "v_wrist": v_st[-1],
                                         "v_length": v_st[-1], "side": side}
        layout["zones"].append({"joint": f"shoulder_{side}", "part": f"arm_{side}", "v": v_st[nk + 1] * 0.5, "reach": v_st[nk + 1] * 0.5 + 0.06,
                                "u_in": u_of(e1), "slot": sl})
        layout["zones"].append({"joint": f"elbow_{side}", "part": f"arm_{side}", "v": _v_at(v_st, ring_t, elbow_t), "reach": reach[1],
                                "u_in": u_of(inner_build), "slot": el})
        # ---- leg ----
        half = list(range(0, n_t // 2 + 1)) if side == "l" else [n_t // 2 + k for k in range(n_t // 2)] + [0]
        loop = [grid[0][c] for c in half] + (bridge if side == "l" else bridge[::-1])
        h.ports[f"leg_{side}"] = loop
        hj = built[f"thigh_{side}"][0]
        axis = _n(built[f"thigh_{side}"][1] - hj)
        e1 = _n(np.array([0.0, 1.0, 0.0]) - axis * axis[1])
        e2 = np.cross(axis, e1)
        la = _n(np.array([0.0, 0.0, 1.0]) - axis * axis[2])
        lb = np.cross(axis, la)
        rows_l = _fit_rows(body["leg_sections_m"], factors.get("leg"))
        knee_t = ln["thigh"]
        stations = list(LEG_STATIONS) + [knee_t + d for d in KNEE_LOOPS] + list(SHIN_STATIONS) + [ln["thigh"] + ln["shin"], ln["thigh"] + ln["shin"] + TUBE_TAIL_M]

        def leg_ease(t):
            if not ease:
                return 0.0
            k = float(smoothstep(knee_t - 0.04, knee_t + 0.10, t))
            return suit["ease_thigh_m"] * (1.0 - k) + suit["ease_shank_m"] * k

        def leg_ring(t, angles):
            a_half, b_half = _section(rows_l, t)
            e = leg_ease(t)
            a_half, b_half = a_half + e, b_half + e
            centre = hj + axis * t + s * la * body["thigh_lateral_offset_m"] * max(0.0, 1.0 - t / knee_t)
            out = []
            for th in angles:
                d = math.cos(th) * e1 + math.sin(th) * e2
                r = 1.0 / math.sqrt((d @ la) ** 2 / a_half ** 2 + (d @ lb) ** 2 / b_half ** 2)
                out.append(centre + d * r)
            return out

        ports = [h.v[i] for i in loop]
        ang = _ring_angles(ports, hj, e1, e2)
        uni, _ = _uniform_angles(ang)
        first = leg_ring(stations[0], uni)
        hp = jt[f"hip_{side}"]["slot"]
        kn = jt[f"knee_{side}"]["slot"]
        rings, ring_t = [loop], [None]
        for j in range(1, nk + 1):
            f = j / (nk + 1)
            w = float(smoothstep(0.0, 1.0, f))
            rings.append([h.add(ports[k] * (1.0 - f) + first[k] * f, {"hips": 1.0 - w, f"thigh_{side}": w}, (hp, 1.0)) for k in range(len(loop))])
            ring_t.append(None)
        for t in stations:
            w_sh = float(smoothstep(knee_t - 0.06, knee_t + 0.06, t))
            w_hp = 0.0 if t > stations[0] + 1e-9 else 0.06
            m_hp = 1.0 - float(smoothstep(0.12, 0.22, t))
            m_kn = 1.0 - float(smoothstep(0.6 * reach[3], reach[3], abs(t - knee_t)))
            mask = (hp, m_hp) if m_hp > 0.0 else ((kn, m_kn) if m_kn > 0.0 else (-1, 0.0))
            rings.append([h.add(p, {"hips": w_hp, f"thigh_{side}": (1.0 - w_sh) * (1.0 - w_hp), f"shin_{side}": w_sh * (1.0 - w_hp)}, mask)
                          for p in leg_ring(t, uni)])
            ring_t.append(t)
        _tube(h, rings, f"leg_{side}", ARM_ATLAS[f"leg_{side}"], cap_weights={f"shin_{side}": 1.0})
        u_of = _u_of(e1, e2, uni)
        v_st = _ring_v(h, rings)
        layout["parts"][f"leg_{side}"] = {"origin": list(ARM_ATLAS[f"leg_{side}"][:2]), "u_width": ARM_ATLAS[f"leg_{side}"][2],
                                         "u_out": u_of(np.array([0.0, 0.0, s])), "u_front": u_of(np.array([0.0, 1.0, 0.0])),
                                         "u_back": u_of(np.array([0.0, -1.0, 0.0])), "u_in": u_of(np.array([0.0, 0.0, -s])),
                                         "v_port_end": v_st[nk + 1], "v_knee": _v_at(v_st, ring_t, knee_t), "v_ankle": v_st[-1],
                                         "v_boot": _v_at(v_st, ring_t, knee_t + ln["shin"] * (1.0 - suit["boot_shaft_fraction"])),
                                         "v_length": v_st[-1], "side": side}
        layout["zones"].append({"joint": f"hip_{side}", "part": f"leg_{side}", "v": v_st[nk + 1] * 0.5, "reach": v_st[nk + 1] * 0.5 + 0.08,
                                "u_in": u_of(np.array([0.0, 1.0, 0.0])), "slot": hp})
        layout["zones"].append({"joint": f"knee_{side}", "part": f"leg_{side}", "v": _v_at(v_st, ring_t, knee_t), "reach": reach[3],
                                "u_in": u_of(np.array([0.0, -1.0, 0.0])), "slot": kn})
    return h, layout


def _u_of(e1, e2, uni):
    """u (0..1 around a tube) of a direction, from the uniform ring angles (u = k / n at angle uni[k])."""
    n = len(uni)
    sign = 1.0 if uni[-1] > uni[0] else -1.0

    def u(direction):
        th = math.atan2(direction @ e2, direction @ e1)
        k = sign * (th - uni[0]) / (2.0 * math.pi) * n
        return float((k % n) / n)
    return u


def _ring_v(h, rings):
    """Arc length (m) along the tube at each ring: the path through the ring centroids."""
    cents = [sum(h.v[i] for i in ring) / len(ring) for ring in rings]
    v = [0.0]
    for a, b in zip(cents, cents[1:]):
        v.append(v[-1] + float(np.linalg.norm(b - a)))
    return v


def _v_at(v_st, ring_t, t):
    pairs = [(rt, v) for rt, v in zip(ring_t, v_st) if rt is not None]
    return fl.lerp_table([[a, b] for a, b in pairs], t)[0]


def _tube(h, rings, part, atlas, cap_weights):
    """Quads between consecutive rings (u = k / n around, v = arc length), a fan cap at the far end."""
    ox, oy, uw = atlas
    v_st = _ring_v(h, rings)
    n = len(rings[0])
    for r in range(len(rings) - 1):
        c0 = sum(h.v[i] for i in rings[r]) / n
        c1 = sum(h.v[i] for i in rings[r + 1]) / n
        for k in range(n):
            k1 = (k + 1) % n
            u0, u1 = k / n, (k + 1) / n
            idx = (rings[r][k], rings[r][k1], rings[r + 1][k1], rings[r + 1][k])
            uv = [(ox + u0 * uw, oy + v_st[r]), (ox + u1 * uw, oy + v_st[r]), (ox + u1 * uw, oy + v_st[r + 1]), (ox + u0 * uw, oy + v_st[r + 1])]
            h.face(idx, uv, part, 0.5 * (c0 + c1))
    last = rings[-1]
    c = sum(h.v[i] for i in last) / n
    prev = sum(h.v[i] for i in rings[-2]) / n
    tip = h.add(c + _n(c - prev) * 0.004, cap_weights)
    for k in range(n):
        h.face((last[k], last[(k + 1) % n], tip), [CAP_UV_M] * 3, part, c - _n(c - prev) * 0.05)


# ---------- Catmull-Clark (the cage -> the smooth hull; base-mesh workflow) ----------

def subdivide(h):
    """One Catmull-Clark step of a closed cage: vertex points keep their indices (ports stay valid), then edge points, then face points.
    Bone weights and the wrinkle masks follow the same stencils as the positions (affine: the weights stay normalised); face-corner UVs
    are interpolated bilinearly (edge corner = the mean of its two corners, face corner = the mean of all)."""
    nv = len(h.v)
    pos = h.positions()
    names = sorted({k for d in h.w for k in d})
    col = {n: i for i, n in enumerate(names)}
    wts = np.zeros((nv, len(names)))
    for i, d in enumerate(h.w):
        for k, x in d.items():
            wts[i, col[k]] = x
    slots = 1 + max([m[0] for m in h.mask] + [0])
    msk = np.zeros((nv, slots))
    for i, (sl, val) in enumerate(h.mask):
        if sl >= 0:
            msk[i, sl] = val
    attr = np.concatenate([pos, wts, msk], axis=1)
    face_pts = np.array([attr[list(f)].mean(axis=0) for f in h.faces])
    edge_faces = {}
    for fi, f in enumerate(h.faces):
        for k in range(len(f)):
            e = (min(f[k], f[(k + 1) % len(f)]), max(f[k], f[(k + 1) % len(f)]))
            edge_faces.setdefault(e, []).append(fi)
    edges = sorted(edge_faces)
    eidx = {e: nv + i for i, e in enumerate(edges)}
    edge_pts = np.zeros((len(edges), attr.shape[1]))
    for i, e in enumerate(edges):
        fs = edge_faces[e]
        if len(fs) == 2:
            edge_pts[i] = (attr[e[0]] + attr[e[1]] + face_pts[fs[0]] + face_pts[fs[1]]) / 4.0
        else:
            edge_pts[i] = (attr[e[0]] + attr[e[1]]) / 2.0
    vf = [[] for _ in range(nv)]
    ve = [[] for _ in range(nv)]
    for fi, f in enumerate(h.faces):
        for i in f:
            vf[i].append(fi)
    boundary = [False] * nv
    for e, fs in edge_faces.items():
        for i in e:
            ve[i].append(e)
        if len(fs) != 2:
            boundary[e[0]] = boundary[e[1]] = True
    vert_pts = np.array(attr)
    for i in range(nv):
        if boundary[i] or not vf[i]:
            continue
        n = len(ve[i])
        f_avg = face_pts[vf[i]].mean(axis=0)
        r_avg = np.mean([(attr[a] + attr[b]) / 2.0 for a, b in ve[i]], axis=0)
        vert_pts[i] = (f_avg + 2.0 * r_avg + (n - 3.0) * attr[i]) / n
    allp = np.concatenate([vert_pts, edge_pts, face_pts])
    out = Hull()
    out.ports = dict(h.ports)
    for row in allp:
        w = {names[k]: float(x) for k, x in enumerate(row[3:3 + len(names)]) if x > 1e-4}
        m = row[3 + len(names):]
        sl = int(np.argmax(m)) if m.size and m.max() > 1e-4 else -1
        out.add(row[:3], w, (sl, float(m[sl]) if sl >= 0 else 0.0))
    fbase = nv + len(edges)
    for fi, (f, uv, part) in enumerate(zip(h.faces, h.uv, h.part)):
        k = len(f)
        fc = fbase + fi
        uv_f = tuple(np.mean(np.array(uv), axis=0))
        for j in range(k):
            a, b, c = f[(j - 1) % k], f[j], f[(j + 1) % k]
            e_prev = eidx[(min(a, b), max(a, b))]
            e_next = eidx[(min(b, c), max(b, c))]
            uv_b = uv[j]
            uv_n = tuple((np.array(uv[j]) + np.array(uv[(j + 1) % k])) / 2.0)
            uv_p = tuple((np.array(uv[(j - 1) % k]) + np.array(uv[j])) / 2.0)
            out.faces.append((b, e_next, fc, e_prev))
            out.uv.append([uv_b, uv_n, uv_f, uv_p])
            out.part.append(part)
    return out


# ---------- skinning ----------

def weight_matrix(h, names):
    idx = {n: i for i, n in enumerate(names)}
    w = np.zeros((len(h.v), len(names)))
    for i, d in enumerate(h.w):
        for k, x in d.items():
            w[i, idx[k]] = x
    return w


def _quat(r):
    """(w, x, y, z) of a rotation matrix."""
    t = np.trace(r)
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        q = [0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2.0
        q = [0.0] * 4
        q[0] = (r[k, j] - r[j, k]) / s
        q[1 + i] = 0.25 * s
        q[1 + j] = (r[j, i] + r[i, j]) / s
        q[1 + k] = (r[k, i] + r[i, k]) / s
    return np.array(q)


def _qmul(a, b):
    w1, x1, y1, z1 = np.moveaxis(a, -1, 0)
    w2, x2, y2, z2 = np.moveaxis(b, -1, 0)
    return np.stack([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], axis=-1)


def dqs(points, w, transforms):
    """Dual-quaternion skinning of points (n, 3) with weights w (n, B) by the rigid 4x4 transforms (B)."""
    qr = np.array([_quat(t[:3, :3]) for t in transforms])
    qt = np.array([[0.0, *t[:3, 3]] for t in transforms])
    qd = 0.5 * _qmul(qt, qr)
    dom = np.argmax(w, axis=1)
    sign = np.sign(np.einsum("bk,nk->nb", qr, qr[dom]))
    sign[sign == 0.0] = 1.0
    ws = w * sign
    br = ws @ qr
    bd = ws @ qd
    nrm = np.linalg.norm(br, axis=1, keepdims=True)
    br, bd = br / nrm, bd / nrm
    conj = br * np.array([1.0, -1.0, -1.0, -1.0])
    t = 2.0 * _qmul(bd, conj)[:, 1:]
    pq = np.concatenate([np.zeros((len(points), 1)), points], axis=1)
    rotated = _qmul(_qmul(br, pq), conj)[:, 1:]
    return rotated + t


def lbs_matrices(w, transforms):
    """(n, 3, 4) blended matrices of linear blend skinning."""
    m = np.array([t[:3, :] for t in transforms])
    return np.einsum("nb,bij->nij", w, m)


def lbs(points, w, transforms):
    m = lbs_matrices(w, transforms)
    return np.einsum("nij,nj->ni", m[:, :, :3], points) + m[:, :, 3]


# ---------- poses ----------

def joint_pose(cfg, joint, theta_deg):
    """{bone: 4x4} (rest -> posed) with only this joint bent to theta (degrees): the child bone and its descendants rotate about the child's
    head so that the angle to the reference direction is theta, in the plane of the rest pose (shoulder: toward forward and a little out =
    the raise to the console; hip: the thigh forward)."""
    b = fl.bones(cfg)
    ref = np.array(b[joint["ref"]]["tail"]) - np.array(b[joint["ref"]]["head"])
    ref = _n(ref) * joint["ref_sign"]
    head = np.array(b[joint["child"]]["head"])
    cd = _n(np.array(b[joint["child"]]["tail"]) - head)
    p = _n(np.array(joint["plane"]))  # figure_layout.bake_direction: the swing direction the shapes are baked for
    th = math.radians(theta_deg)
    target = math.cos(th) * ref + math.sin(th) * p
    t = rigid(min_rot(cd, target), head)
    poses = {n: np.eye(4) for n in b}
    for d in descendants(b, joint["child"]):
        poses[d] = t
    return poses


# ---------- the exported suit: rest mesh, weights, shapes ----------

SUBDIVISIONS = 1  # [E] Catmull-Clark steps of the cage (base-mesh workflow: the cage's loops, a smooth surface)


GIRTH_PASSES = 3  # [E] fixed-point passes of girth_factors (the residual after three is < 1 mm)
_FACTORS = {}


def girth_factors(cfg):
    """Per-row factors on the ANSUR girths so the SUBDIVIDED body (Catmull-Clark rounds the cage inward) has them: measured on the body
    (no ease) with figure_measure, three fixed-point passes; the suit uses the same factors. Cached per configuration."""
    import json
    import figure_measure as fm
    key = json.dumps(cfg["body"], sort_keys=True) + json.dumps(cfg["anthropometry"], sort_keys=True)
    if key in _FACTORS:
        return _FACTORS[key]
    body = cfg["body"]
    f = {"torso": [1.0] * len(body["torso_levels_m"]), "arm": [1.0] * len(body["arm_sections_m"]), "leg": [1.0] * len(body["leg_sections_m"])}
    for _ in range(GIRTH_PASSES):
        h, _ = hull(cfg, False, f)
        for _s in range(SUBDIVISIONS):
            h = subdivide(h)
        p = h.positions()
        for i, row in enumerate(body["torso_levels_m"]):
            if row[4] > 0.0:
                f["torso"][i] *= row[4] / fm.torso(h, p, cfg, row[0])[0]
        for name, part in (("arm", "arm_l"), ("leg", "leg_l")):
            for i, row in enumerate(body[f"{name}_sections_m"]):
                if row[3] > 0.0:
                    f[name][i] *= row[3] / fm.limb(cfg, h, p, part, row[0])
    _FACTORS[key] = f
    return f


def surface(cfg, ease=True):
    """(Hull, layout): the subdivided hull - the surface the figure shows (ease) / the body the tests measure (no ease)."""
    h, layout = hull(cfg, ease, girth_factors(cfg))
    for _ in range(SUBDIVISIONS):
        h = subdivide(h)
    return h, layout


def suit_mesh(cfg, ease=True):
    """{"hull", "layout", "names" (bones), "w" (n, B), "build" (n, 3), "rest" (n, 3), "shapes" {name: (n, 3) rest-frame offsets}}."""
    h, layout = surface(cfg, ease)
    names = list(fl.BONE_NAMES)
    g, _ = build_pose(cfg)
    w = weight_matrix(h, names)
    build = h.positions()
    to_rest = [np.linalg.inv(g[n]) for n in names]
    rest = dqs(build, w, to_rest)
    refs = {part: np.mean(rest[loop], axis=0) for part, loop in h.ports.items()}  # fixed volume references: the rest port loops' centroids
    shapes = {}
    for jname, joint in fl.joint_table(cfg).items():
        part = ("arm_" if jname.startswith(("shoulder", "elbow")) else "leg_") + jname[-1]
        zone = np.array([m[1] if m[0] == joint["slot"] else 0.0 for m in h.mask])
        ref = refs[part]
        v_rest = part_volume(h, rest, part, ref)
        for key, theta in (("shape_flex", joint["full_deg"]), ("shape_ext", joint["ext_deg"])):
            target, pose = posed_target(cfg, joint, theta, w, names, g, build)
            target = bulge(h, target, part, zone, v_rest, ref)
            shapes[joint[key]] = corrective(target, pose, w, names, rest)
    return {"hull": h, "layout": layout, "names": names, "w": w, "build": build, "rest": rest, "shapes": shapes, "g": g, "refs": refs}


def posed_target(cfg, joint, theta, w, names, g, build):
    """The dual-quaternion pose of the build hull with the joint at theta (rest elsewhere)."""
    pose = joint_pose(cfg, joint, theta)
    return dqs(build, w, [pose[n] @ np.linalg.inv(g[n]) for n in names]), pose


def vertex_normals(h, points, part):
    """(area-weighted unit normals, vertex areas) of the part's faces; zero rows off the part."""
    tris = h.triangles({part})
    p = np.asarray(points)
    fn = np.cross(p[tris[:, 1]] - p[tris[:, 0]], p[tris[:, 2]] - p[tris[:, 0]])
    nrm = np.zeros_like(p)
    area = np.zeros(len(p))
    for k in range(3):
        np.add.at(nrm, tris[:, k], fn)
        np.add.at(area, tris[:, k], np.linalg.norm(fn, axis=1) / 6.0)
    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
    return np.where(ln > 0.0, nrm / np.maximum(ln, 1e-12), 0.0), area


def bulge(h, target, part, zone, volume, ref):
    """Cloth instead of a collapse: the joint zone (wrinkle mask) pushed out along its normals until the limb's volume equals `volume`
    (the rest pose's): the fabric over a bent elbow / knee bunches outward."""
    out = np.array(target)
    for _ in range(3):
        nrm, area = vertex_normals(h, out, part)
        deficit = volume - part_volume(h, out, part, ref)
        gain = float(np.sum(zone * area))
        if gain <= 0.0 or abs(deficit) < 1e-9:
            break
        out = out + nrm * (zone * deficit / gain)[:, None]
    return out


def corrective(target, pose, w, names, rest):
    """Bind-space offsets d with LBS(rest + d) = target under the pose (zero where tiny)."""
    m = lbs_matrices(w, [pose[n] for n in names])
    d = np.linalg.solve(m[:, :, :3], (target - m[:, :, 3])[:, :, None])[:, :, 0] - rest
    d[np.linalg.norm(d, axis=1) < SHAPE_EPS_M] = 0.0
    return d


def posed(mesh, cfg, thetas, corrected=True):
    """The exported rest mesh as the game skins it: joints {name: theta_deg} bent together (each about its own rest plane), LBS of
    rest + the weighted corrective shapes (corrected) or plain LBS."""
    table = fl.joint_table(cfg)
    names = mesh["names"]
    pose = {n: np.eye(4) for n in names}
    offs = np.zeros_like(mesh["rest"])
    for jname, theta in thetas.items():
        joint = table[jname]
        jp = joint_pose(cfg, joint, theta)
        for n in names:
            if not np.allclose(jp[n], np.eye(4)):
                pose[n] = jp[n] @ pose[n]
        flex, ext, _ = fl.bend_weights(joint, theta)
        if corrected:
            offs = offs + flex * mesh["shapes"][joint["shape_flex"]] + ext * mesh["shapes"][joint["shape_ext"]]
    return lbs(mesh["rest"] + offs, mesh["w"], [pose[n] for n in names])


def part_volume(h, points, part, ref=None):
    """Signed volume enclosed by the part's faces and the fan from `ref` (default: the part's port loop centroid) - the limb's volume
    measured from its (fixed) root loop."""
    tris = h.triangles({part})
    p = np.asarray(points)
    if ref is None:
        ref = np.mean(p[h.ports[part]], axis=0)
    a, b, c = p[tris[:, 0]] - ref, p[tris[:, 1]] - ref, p[tris[:, 2]] - ref
    return float(np.sum(np.einsum("ij,ij->i", a, np.cross(b, c))) / 6.0)
