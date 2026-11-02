"""Crew Dragon: gen_dragon.py -- --spec <json> --out <dir> writes dragon_capsule.gltf, dragon_nose.gltf, dragon_trunk.gltf.

Own rebuild from the typed tables of config/vehicles/detail/crew_dragon.toml (measured on the reference model, see there); no third-party mesh.
Frames: each stage's body frame (X station axis, nose +X). dragon_capsule.gltf: station 0 = the heat shield's apex; nodes `heat_shield`, `band`,
`capsule` (skin r(x, az): body of revolution + four cowl lobes, ending on the tilted seam), `superdraco_pod` (bay floors and walls), `superdraco_nozzle`,
`draco_pod` (port slots), `window`, `hatch` (hatch panel, window flanges), `seams`, `hinge_fairing`, `deck` (seat on the seam, conical skirt to the flange), `recess` (floor, well, hatch end; the mechanism itself: idss_mechanism.py).
dragon_nose.gltf: node `nose_cone` (shell + inside) and `nose_seal`, ORIGIN AT THE HINGE POINT (vertices relative to it), closed pose =
identity: the game rotates about the hinge axis (manifest `nose`). dragon_trunk.gltf: station 0 = the trunk's base; `trunk` (shell, fins, floor), `trunk_array`,
`trunk_radiator`, `umbilical`. Plain surfaces. Stations of the tables count from the shield apex; the trunk file adds the stage length.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402,F401
import bmesh  # noqa: E402
import vehicle_common as vc  # noqa: E402
from mathutils import Vector  # noqa: E402

AXIS_X = Vector((1.0, 0.0, 0.0))
PATCH_SEGMENTS = 8  # [E] segments of a solar / radiator patch across its angle
LOBE_V = (0.0, 0.1, 0.2, 0.3, 0.4, 0.49, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0)  # [E] cowl columns: |v| <= 0.49 floor, 0.5 outer wall edge, 1 lobe's edge
FLOOR_V = 0.49
LOBE_ROUND_FROM = 0.45  # [E] the lobe's crown falls off over the outer part of the cheek (M2: soft valley lines)
WARP_FROM_M = 1.2  # [E] the bay's arch bends the rows above this station
ROW_STEP_BAY_M = 0.05  # [E] row pitch around the bay
ROW_STEP_WALL_M = 0.1  # [E] row pitch above it
SEAM_ROWS = 8  # [E] rows from the last plain row to the seam
SEAM_ROW_FROM_M = 3.55  # [E] just under the seam's lowest point
NOSE_ROWS = 24  # [E] rows of the nose's outer surface, denser at the tip


def tangent(az):
    return Vector((0.0, -math.sin(az), math.cos(az)))


def lerp(table, x):
    if x <= table[0][0]:
        return table[0][1]
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0) if x1 > x0 else y1
    return table[-1][1]


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


class Body:
    """The measured skin: r(x, az), the seam plane, and the column layout (cowl columns follow the bay's edge)."""

    def __init__(self, spec):
        self.c = spec["detail"]["capsule"]
        self.pods = spec["stages"]["capsule"]["superdraco_lobes_deg"]
        self.profile = self.c["profile_m"]
        self.tip = self.profile[-1][0]

    def base(self, x):
        return lerp(self.profile, x)

    def hood(self, x):
        outer = self.c["hood_outer_m"]
        return max(lerp(outer, x) - self.base(x), 0.0) if outer[0][0] <= x <= outer[-1][0] else 0.0

    def bay_half(self, x):
        return lerp(self.c["bay_half_width_deg_m"], x)

    def top(self, v):
        return self.c["bay_top_x_m"] - self.c["bay_corner_drop_m"] * (min(abs(v), 0.5) / 0.5) ** self.c["bay_corner_power"]

    def lobe_fall(self, d, x):
        half, width = self.bay_half(x), self.c["hood_half_width_deg"]
        if d <= half:
            return 1.0
        t = (d - half) / (width - half)
        if t <= LOBE_ROUND_FROM:
            return 1.0
        return 0.5 + 0.5 * math.cos(math.pi * min((t - LOBE_ROUND_FROM) / (1.0 - LOBE_ROUND_FROM), 1.0))

    def surface(self, x, az_deg):
        """Outer radius anywhere on the skin (details are laid on it)."""
        for pod in self.pods:
            d = abs(wrap_deg(az_deg - pod))
            if d < self.c["hood_half_width_deg"]:
                half = self.bay_half(x)
                if d <= half and x <= self.top(0.5 * d / half):
                    return lerp(self.c["bay_floor_m"], x)
                return self.base(x) + self.hood(x) * self.lobe_fall(d, x)
        return self.base(x)

    def seam(self, az_deg):
        """(x, r) of the seam plane on the skin of revolution at this azimuth."""
        cos_az = math.cos(math.radians(az_deg))
        x = self.c["seam_axis_x_m"]
        for _ in range(30):
            x = self.c["seam_axis_x_m"] + self.c["seam_slope"] * self.base(x) * cos_az
        return x, self.base(x)

    def rows(self):
        c = self.c
        top = c["bay_top_x_m"]
        stations = {c["band_top_x_m"]}
        x = c["band_top_x_m"]
        while x < 2.1:
            stations.add(round(x, 4))
            x += ROW_STEP_BAY_M
        while x < SEAM_ROW_FROM_M:
            stations.add(round(x, 4))
            x += ROW_STEP_WALL_M
        stations.update((top, top + 0.01, SEAM_ROW_FROM_M))
        stations = sorted(s for s in stations if not (top - 0.02 < s < top or top + 0.01 < s < top + 0.03))
        return stations

    def columns(self):
        """Columns around the body: (pod azimuth or None, signed v or the fixed azimuth), from az 0 upward."""
        edges = []
        width = self.c["hood_half_width_deg"]
        for pod in sorted(p % 360.0 for p in self.pods):
            edges.append((pod - width, pod))
        cols = []
        az = 0.0
        step = self.c["az_step_deg"]
        for start, pod in edges:
            n = max(int(round((start - az) / step)), 1)
            cols += [(None, az + (start - az) * i / n) for i in range(n)]
            for v in [-w for w in reversed(LOBE_V[1:])] + list(LOBE_V):
                cols.append((pod, v))
            az = pod + width
        n = max(int(round((360.0 - az) / step)), 1)
        cols += [(None, az + (360.0 - az) * i / n) for i in range(n)]
        return cols

    def col_az(self, col, x):
        pod, v = col
        if pod is None:
            return v
        half, width = self.bay_half(min(x, self.c["bay_top_x_m"])), self.c["hood_half_width_deg"]
        a = abs(v)
        d = half * a / 0.5 if a <= 0.5 else half + (width - half) * (a - 0.5) / 0.5
        return pod + math.copysign(d, v)

    def cell_x(self, col, x):
        """The row's real station in this column: the bay's arch bends the rows near the lip; above the plain rows they run to the seam."""
        top0 = self.c["bay_top_x_m"]
        if col[0] is not None and WARP_FROM_M < x <= top0 + 0.2:
            a = abs(col[1])
            top = self.top(col[1]) if a <= 0.5 else top0 - self.c["bay_corner_drop_m"] * (1.0 - (a - 0.5) / 0.5)  # no bend left at the lobe's edge
            if x <= top0:
                return WARP_FROM_M + (x - WARP_FROM_M) * (top - WARP_FROM_M) / (top0 - WARP_FROM_M)
            f = (x - top0 - 0.01) / (0.2 - 0.01)
            return x - (top0 - top) * (1.0 - f)
        return x

    def point(self, col, row, x_row):
        x = self.cell_x(col, x_row)
        az = self.col_az(col, x)
        if col[0] is not None and abs(col[1]) <= FLOOR_V and x_row <= self.c["bay_top_x_m"]:
            r = lerp(self.c["bay_floor_m"], x)
        elif col[0] is not None:
            r = self.base(x) + self.hood(x) * self.lobe_fall(abs(wrap_deg(az - col[0])), x)
        else:
            r = self.base(x)
        return vc.pt(x, math.radians(az), r), az, x


def build_skin(body, hull, floor):
    c = body.c
    cols = body.columns()
    rows = body.rows()
    top0 = c["bay_top_x_m"]
    ncols = len(cols)
    grid = []
    for x_row in rows:
        grid.append([body.point(col, 0, x_row) for col in cols])
    # Rows from the last plain row to the seam: per column, up to the plane minus the gap.
    last = grid[-1]
    for k in range(1, SEAM_ROWS + 1):
        line = []
        for j, col in enumerate(cols):
            _, az, x0 = last[j]
            xs = body.seam(az)[0] - c["seam_gap_m"]
            x = x0 + (xs - x0) * k / SEAM_ROWS
            rr = body.base(x) + (body.hood(x) * body.lobe_fall(abs(wrap_deg(az - col[0])), x) if col[0] is not None else 0.0)
            line.append((vc.pt(x, math.radians(az), rr), az, x))
        grid.append(line)
    top_index = rows.index(top0)
    for i in range(len(grid) - 1):
        for j in range(ncols):
            ca, cb = cols[j], cols[(j + 1) % ncols]
            (p00, a0, x00), (p01, a1, x01) = grid[i][j], grid[i][(j + 1) % ncols]
            (p10, _, x10), (p11, _, x11) = grid[i + 1][j], grid[i + 1][(j + 1) % ncols]
            uvs = [(math.radians(a0) * 2.0, 1.0 - x00), (math.radians(a1 if a1 > a0 else a1 + 360.0) * 2.0, 1.0 - x01),
                   (math.radians(a1 if a1 > a0 else a1 + 360.0) * 2.0, 1.0 - x11), (math.radians(a0) * 2.0, 1.0 - x10)]
            pts = [p00, p01, p11, p10]
            same_pod = ca[0] is not None and ca[0] == cb[0]
            in_bay = same_pod and i < top_index
            mid = math.radians(0.5 * (a0 + (a1 if a1 > a0 else a1 + 360.0)))
            if in_bay and abs(ca[1]) <= FLOOR_V and abs(cb[1]) <= FLOOR_V:
                floor.face(pts, uvs, vc.radial_dir(mid), ("floor", ca[0]))
            elif in_bay and (abs(ca[1]) <= FLOOR_V) != (abs(cb[1]) <= FLOOR_V):  # side wall, faces the bay's middle
                toward = -1.0 if ca[1] > 0.0 or cb[1] > 0.0 else 1.0
                floor.face(pts, uvs, tangent(mid) * toward, None)
            elif same_pod and i == top_index and abs(ca[1]) <= FLOOR_V and abs(cb[1]) <= FLOOR_V:  # the lip, faces down
                floor.face(pts, uvs, -AXIS_X, None)
            else:
                hull.face(pts, uvs, vc.radial_dir(mid), "skin")
    return grid[-1], cols


def ellipse(mesh, body, x0, az0, hx, haz, lift, power=2.0, rings=4, n=28, hole=0.0, lean_deg=0.0):
    """A patch on the skin: superellipse in (x, az), following r(x, az) + lift. `hole` > 0: a ring from that scale to 1.
    `lean_deg` turns the patch on the skin; the azimuth half width is then given in metres at the local radius."""
    scales = [hole + (1.0 - hole) * k / rings for k in range(rings + 1)]

    def p(s, k):
        t = 2.0 * math.pi * k / n
        ct, st = math.cos(t), math.sin(t)
        e = 2.0 / power
        dx = hx * s * math.copysign(abs(ct) ** e, ct)
        da = haz * s * math.copysign(abs(st) ** e, st)
        if lean_deg:
            sin_l, cos_l = math.sin(math.radians(lean_deg)), math.cos(math.radians(lean_deg))
            dx, da_m = dx * cos_l - da * sin_l, dx * sin_l + da * cos_l
            da = math.degrees(da_m / body.base(x0))
        x, a = x0 + dx, az0 + da
        return vc.pt(x, math.radians(a), body.surface(x, a) + lift), (da * 0.03, 1.0 - dx)

    out = vc.radial_dir(math.radians(az0))
    for r in range(rings):
        s0, s1 = scales[r], scales[r + 1]
        for k in range(n):
            a, ua = p(s0, k)
            b, ub = p(s0, k + 1)
            cc, uc = p(s1, k + 1)
            d, ud = p(s1, k)
            mesh.face([a, b, cc, d], [ua, ub, uc, ud], out, None)


def skirt_point(body, az_deg, r, lift):
    """On the conical skirt between the flange's edge and the seat on the seam plane, at this azimuth and radius r."""
    c = body.c
    flange_x, flange_r = c["neck_m"][0]
    seam_x, _ = body.seam(az_deg)
    seat_r = body.surface(seam_x, az_deg) - c["lip_thickness_m"]  # with the cowl's lobe, as the seat is drawn
    f = (r - flange_r) / (seat_r - flange_r)
    return vc.pt(flange_x + (seam_x - flange_x) * f + lift, math.radians(az_deg), r)


def skirt_patch(mesh, body, az_deg, r, half_t, half_r, lift, rings=3, n=24):
    """A dark ellipse on the skirt: half_t along the circle, half_r across it."""
    def p(s, k):
        a = 2.0 * math.pi * k / n
        along, across = half_t * s * math.cos(a), half_r * s * math.sin(a)
        local_r = r + across
        return skirt_point(body, az_deg + math.degrees(along / r), local_r, lift)
    for ring in range(rings):
        s0, s1 = ring / rings, (ring + 1) / rings
        for k in range(n):
            mesh.face([p(s0, k), p(s0, k + 1), p(s1, k + 1), p(s1, k)], [(0, 0), (1, 0), (1, 1), (0, 1)], AXIS_X, None)


def build_deck(body, deck, rim, cols, nose_x_lift):
    """The seat under the seam, then the conical skirt to the flange's edge (the flange: neck_m, DockingMechanismView's part on it)."""
    c = body.c
    ncols = len(cols)
    lip = c["lip_thickness_m"]
    flange_x, neck_r = c["neck_m"][0]
    for j in range(ncols):
        (p0, a0, x0), (p1, a1, x1) = rim[j], rim[(j + 1) % ncols]
        r0, r1 = math.hypot(p0.y, p0.z), math.hypot(p1.y, p1.z)
        az0, az1 = math.radians(a0), math.radians(a1)
        mid = 0.5 * (az0 + (az1 if a1 > a0 else az1 + 2.0 * math.pi))
        # the seat on the seam plane
        q0, q1 = vc.pt(x0 - c["seam_slope"] * lip * math.cos(az0) * 0.0, az0, r0 - lip), vc.pt(x1, az1, r1 - lip)
        uv = [(0, 0), (1, 0), (1, 1), (0, 1)]
        deck.face([p0, p1, q1, q0], uv, AXIS_X, ("seat", 0))
        # the skirt, from the seat to the flange's edge
        n0, n1 = vc.pt(flange_x, az0, neck_r), vc.pt(flange_x, az1, neck_r)
        deck.face([q0, q1, n1, n0], uv, AXIS_X + vc.radial_dir(mid) * 0.3, ("deck", 0))


def build_capsule(spec, out):
    c, cap = spec["detail"]["capsule"], spec["stages"]["capsule"]
    body = Body(spec)
    vc.common.clear_scene()
    root = vc.empty("dragon_capsule")
    n = int(round(360.0 / c["az_step_deg"]))
    shield, band, hull, floor = vc.VMesh(), vc.VMesh(), vc.VMesh(), vc.VMesh()
    prof = body.profile
    edge, band_top = c["shield_edge_x_m"], c["band_top_x_m"]
    vc.revolve(shield, [([p for p in prof if p[0] <= edge], (-1.0, 0.0, 0.0))], n, 2.0, "heat_shield")
    vc.revolve(band, [([p for p in prof if edge <= p[0] <= band_top], "out")], n, 2.0, "band")
    rim, cols = build_skin(body, hull, floor)
    deck = vc.VMesh()
    build_deck(body, deck, rim, cols, 0.0)
    neck = c["neck_m"]
    recess = vc.VMesh()  # below the flange and its tunnel (idss_mechanism.py): floor, well, hatch end
    vc.revolve(recess, [(neck[2:4], (1.0, 0.0, 0.0)), (neck[3:5], "in"), (neck[4:6], (1.0, 0.0, 0.0))], n, 1.0, "recess")
    nozzles, draco, window, panel, seams = vc.VMesh(), vc.VMesh(), vc.VMesh(), vc.VMesh(), vc.VMesh()
    for pod in cap["superdraco_lobes_deg"]:
        for side in (-1.0, 1.0):
            ellipse(nozzles, body, c["nozzle_x_m"], pod + side * cap["superdraco_half_gap_deg"], c["nozzle_half_x_m"], c["nozzle_half_az_deg"], 0.006, rings=1, hole=c["nozzle_outline_scale"])
    lift_panel, lift_detail = c["lift_panel_m"], c["lift_detail_m"]
    for cluster in c["draco_clusters_az_deg"]:
        toward_bay = 1.0 if cluster < 90.0 else -1.0  # the groups lie mirrored about az 90: each leans to its neighbouring bay
        for mirror in (1.0, -1.0):
            for x, half_len, half_wid, off, lean in c["draco_slots"]:
                ellipse(draco, body, x, mirror * (cluster + toward_bay * off), half_len, half_wid, lift_detail, rings=2, lean_deg=mirror * toward_bay * lean)
    for az, x, hx, haz in c["windows"]:
        for mirror in (1.0, -1.0):
            ellipse(window, body, x, mirror * az, hx, haz, lift_detail, rings=2)
            ellipse(panel, body, x, mirror * az, hx * c["window_flange_scale"], haz * c["window_flange_scale"], lift_panel, rings=3)
    h_az, h_x, h_hx, h_haz = c["hatch"]
    ellipse(panel, body, h_x, h_az, h_hx, h_haz, lift_panel, power=4.0, rings=6, n=48)
    seam_scale = 1.0 - c["hatch_seam_m"] / h_hx
    ellipse(seams, body, h_x, h_az, h_hx, h_haz, lift_detail, power=4.0, rings=1, n=48, hole=seam_scale)
    for az, r, radius in c["deck_holes"]:
        skirt_patch(seams, body, az, r, radius, radius, lift_detail)
    slot_az, slot_r, slot_t, slot_w = c["deck_slot"]
    skirt_patch(seams, body, slot_az, slot_r, slot_t, slot_w, lift_detail)
    fair = vc.VMesh()
    x0, x1, r_out, width = c["hinge_fairing_m"]
    az = math.radians(c["nose_hinge_azimuth_deg"])
    r_in = body.base(0.5 * (x0 + x1)) - 0.02
    e_r = vc.radial_dir(az)
    vc.box(fair, AXIS_X * 0.5 * (x0 + x1) + e_r * 0.5 * (r_in + r_out), AXIS_X, e_r, tangent(az), (0.5 * (x1 - x0), 0.5 * (r_out - r_in), 0.5 * width))
    parts = (("capsule", hull), ("heat_shield", shield), ("band", band), ("deck", deck), ("recess", recess), ("superdraco_pod", floor), ("superdraco_nozzle", nozzles),
             ("draco_pod", draco), ("window", window), ("hatch", panel), ("seams", seams), ("hinge_fairing", fair))
    tris = sum(m.to_object(name, root)[1] for name, m in parts)
    vc.export_selected(Path(out) / "dragon_capsule.gltf", [root])
    vc.report_triangles("dragon_capsule", tris)


def build_nose(spec, out):
    c, nose = spec["detail"]["capsule"], spec["nose"]
    body = Body(spec)
    n = int(c["nose_columns"])
    lip = c["lip_thickness_m"]
    vc.common.clear_scene()
    root = vc.empty("dragon_nose")
    mesh, seal = vc.VMesh(), vc.VMesh()
    # Outer skin above the seam plane, inner shell, lip on the plane.
    ring = []
    inner = []
    sx, sr = c["nose_inner_shoulder_m"]
    for j in range(n):
        az = 360.0 * j / n
        xs, rs = body.seam(az)
        outer = []
        for k in range(NOSE_ROWS + 1):
            s = 1.0 - k / NOSE_ROWS
            x = body.tip - (body.tip - xs) * s * s
            r = body.base(x) if k < NOSE_ROWS else 0.0
            outer.append(vc.pt(x, math.radians(az), r))
        ring.append(outer)
        inner.append([vc.pt(xs, math.radians(az), rs - lip), vc.pt(sx + (xs - sx) * 0.5, math.radians(az), sr + (rs - lip - sr) * 0.5), vc.pt(sx, math.radians(az), sr)])
    uv = [(0, 0), (1, 0), (1, 1), (0, 1)]
    for j in range(n):
        a, b = ring[j], ring[(j + 1) % n]
        mid = math.radians(360.0 * (j + 0.5) / n)
        for k in range(NOSE_ROWS):
            mesh.face([a[k], b[k], b[k + 1], a[k + 1]], uv, vc.radial_dir(mid) + AXIS_X * (k / NOSE_ROWS), "skin")
        ia, ib = inner[j], inner[(j + 1) % n]
        mesh.face([a[0], b[0], ib[0], ia[0]], uv, -AXIS_X, None)  # the lip
        for k in range(2):
            mesh.face([ia[k], ib[k], ib[k + 1], ia[k + 1]], uv, -vc.radial_dir(mid), "inner")
    vc.revolve(mesh, [([(sx, sr), tuple(c["nose_inner_cone_m"][0])], (-1.0, 0.0, 0.0)), (c["nose_inner_cone_m"][:2], "in"), (c["nose_inner_cone_m"][1:], (-1.0, 0.0, 0.0))], n, 1.0, "inner_cone")
    sx2, r0, r1 = c["nose_seal_m"]
    vc.revolve(seal, [([(sx2, r1), (sx2, r0)], (-1.0, 0.0, 0.0))], n, 1.0, "seal")
    # Hinge arm: from the pin along the inside to the seal ring, in the shell's node so the pin has a vertex on the hinge point.
    hinge = vc.pt(nose["hinge_station_m"], math.radians(nose["hinge_azimuth_deg"]), nose["hinge_radius_m"])
    az = math.radians(nose["hinge_azimuth_deg"])
    e_r, t = vc.radial_dir(az), tangent(az)
    width, thick = c["hinge_arm_m"]
    far = AXIS_X * sx2 + e_r * r1  # the seal ring's outer edge: the strap clears the flange
    along = (far - hinge).normalized()
    normal = along.cross(t).normalized()
    pin = [hinge, hinge + t * (width / 2.0), hinge - t * (width / 2.0)]
    seal.face([pin[1], pin[2], far - t * (width / 2.0), far + t * (width / 2.0)], uv, -e_r, None)
    mesh.face([pin[0], pin[1] + normal * thick, pin[2] + normal * thick], [(0, 0), (1, 0), (0, 1)], -AXIS_X, None)  # the pin: its first vertex IS the hinge point
    bmesh.ops.translate(mesh.bm, vec=-hinge, verts=mesh.bm.verts)  # the origin is the hinge point
    bmesh.ops.translate(seal.bm, vec=-hinge, verts=seal.bm.verts)
    tris = mesh.to_object("nose_cone", root)[1] + seal.to_object("nose_seal", root)[1]
    vc.export_selected(Path(out) / "dragon_nose.gltf", [root])
    vc.report_triangles("dragon_nose", tris)


def patch(mesh, x0, x1, r, a0, a1, group):
    """A thin curved panel on the cylinder of radius r between stations x0..x1 and azimuths a0..a1 (radians)."""
    for i in range(PATCH_SEGMENTS):
        b0 = a0 + (a1 - a0) * i / PATCH_SEGMENTS
        b1 = a0 + (a1 - a0) * (i + 1) / PATCH_SEGMENTS
        mesh.face([vc.pt(x0, b0, r), vc.pt(x0, b1, r), vc.pt(x1, b1, r), vc.pt(x1, b0, r)],
                  [(b0 * r, 1.0 - x0), (b1 * r, 1.0 - x0), (b1 * r, 1.0 - x1), (b0 * r, 1.0 - x1)],
                  vc.radial_dir(0.5 * (b0 + b1)), group)


def fin(mesh, az_rad, outline, offset, thickness):
    """A flat fin from the outline [x above the shield apex, r], thin plate with its edge faces."""
    e_r, t = vc.radial_dir(az_rad), tangent(az_rad)
    pts = [AXIS_X * max(x + offset, 0.0) + e_r * r for x, r in outline]
    uv = [(p.x, r) for p, (_, r) in zip(pts, outline)]
    for side in (-1.0, 1.0):
        mesh.face([p + t * (side * thickness / 2.0) for p in pts], uv, t * side)
    for a, b in zip(pts, pts[1:]):
        out = (e_r + (AXIS_X if b.x > a.x else -AXIS_X) * 0.5 * abs(b.dot(e_r) - a.dot(e_r)) / max(abs(b.x - a.x), 1e-6)).normalized()
        mesh.face([a - t * (thickness / 2.0), a + t * (thickness / 2.0), b + t * (thickness / 2.0), b - t * (thickness / 2.0)], [(0, 0), (1, 0), (1, 1), (0, 1)], out)


def build_trunk(spec, out):
    c, tr = spec["detail"]["trunk"], spec["stages"]["trunk"]
    r, n = tr["diameter_m"] / 2.0, int(c["segments"])
    length = tr["length_m"]
    top = length + c["shield_well_m"]
    vc.common.clear_scene()
    root = vc.empty("dragon_trunk")
    wall = c["top_wall_m"]
    floor = top - c["floor_depth_m"]
    body = vc.VMesh()
    vc.revolve(body, [([(0.0, r), (top, r)], "out"), ([(top, r), (top, r - wall)], (1.0, 0.0, 0.0)),
                      ([(top, r - wall), (floor, r - wall)], "in"), ([(floor, r - wall), (floor, 0.0)], (1.0, 0.0, 0.0)),
                      ([(0.0, c["base_inner_r_m"]), (0.0, r)], (-1.0, 0.0, 0.0))], n, r, "trunk")
    first = math.radians(c["fin_first_azimuth_deg"])
    step = 2.0 * math.pi / c["fin_count"]
    for k in range(int(c["fin_count"])):
        fin(body, first + k * step, c["fin_outline_m"], length, c["fin_thickness_m"])
    array, radiator = vc.VMesh(), vc.VMesh()
    vc.revolve(array, [([(0.0, 0.0), (0.0, c["base_inner_r_m"])], (-1.0, 0.0, 0.0))], n, r, "base")
    x_top = length + c["array_top_x_m"]
    for k, (a0, a1) in enumerate(c["radiator_azimuths_deg"]):
        patch(radiator, c["panel_base_m"], x_top, r + c["panel_proud_m"], math.radians(a0), math.radians(a1), ("radiator", k))
    for k, (a0, a1) in enumerate(c["array_azimuths_deg"]):
        patch(array, c["panel_base_m"], x_top, r + c["panel_proud_m"], math.radians(a0), math.radians(a1), ("array", k))
    x0, x1, width = c["trunk_slots_m"]
    for az in c["trunk_slots_azimuths_deg"]:
        half = 0.5 * width / r
        patch(array, length + x0, length + x1, r + 2.0 * c["panel_proud_m"], math.radians(az) - half, math.radians(az) + half, ("slot", az))
    umb = vc.VMesh()
    a = math.radians(c["umbilical_azimuth_deg"])
    e_r, t = vc.radial_dir(a), tangent(a)
    w = 0.5 * c["umbilical_width_m"]
    inner = c["umbilical_outline_m"][0][1]
    outline = [(length + x, rr) for x, rr in c["umbilical_outline_m"]]
    uv = [(0, 0), (1, 0), (1, 1), (0, 1)]
    for side in (-1.0, 1.0):
        umb.face([AXIS_X * x + e_r * rr + t * side * w for x, rr in outline] + [AXIS_X * outline[-1][0] + e_r * inner + t * side * w], [(x, rr) for x, rr in outline] + [(outline[-1][0], inner)], t * side)
    for (xa, ra), (xb, rb) in zip(outline, outline[1:]):
        umb.face([AXIS_X * xa + e_r * ra - t * w, AXIS_X * xa + e_r * ra + t * w, AXIS_X * xb + e_r * rb + t * w, AXIS_X * xb + e_r * rb - t * w], uv, e_r, ("top", 0))
    tris = body.to_object("trunk", root)[1] + array.to_object("trunk_array", root)[1] + radiator.to_object("trunk_radiator", root)[1] + umb.to_object("umbilical", root)[1]
    vc.export_selected(Path(out) / "dragon_trunk.gltf", [root])
    vc.report_triangles("dragon_trunk", tris)


def main():
    args = vc.common.script_args()
    spec = vc.load_spec(args["spec"])
    build_capsule(spec, args["out"])
    build_nose(spec, args["out"])
    build_trunk(spec, args["out"])


if __name__ == "__main__":
    main()
