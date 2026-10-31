"""Pure layout of the Pad B launch mount, its trench and hardstand: polygons per glTF node, no Blender.

gen_mount.py / gen_trench.py add the polygons to meshes; pytest reads the same lists (shape, clearances).
Frame (both models): X toward the tower, Y across, Z up, origin on the mount axis at the ground (the site height).
A surface is (verts, uvs): planar polygon, counter-clockwise seen from outside (the winding is fixed by an outward hint
in `Surfaces.add`, so the glTF normals point out). UV in metres on the surface; the shell shader repeats the panel map.

Shape (rulings-mount-shape.md, photos DSC06000 / DSC06022 / CRL05690 / DSC05957; launch-structures-measurements.md "Mount
shape"): a BLOCK with straight vertical walls (X from -block_x_back to +block_x_tower, Y +-block_half_y, flat deck, round
opening with the ring), a pyramid ANNEX only on the side away from the tower (trapezoid footprint, plated slopes, door
group at its foot, white vent plates, dark louvre boxes), and ONE tunnel of one width end to end through the block along
Y: level floor at -pit_depth under the ring, straight ramps up to the ground at both ends, vertical walls, the walls
outside the block with a top that falls from the deck height with the annex to a 0.5 m wall at its end (`wall_height_m`), then runs level to the ramp end. No deflector, no flare.
The two booster QD heads stand on the ring under the two hoods (no cabinets on the deck). The floor is one closed surface.
Heights and plan sizes are the measured ones (deck_height_m 14.5 m); the physics mount height follows in the physics plan.
"""
from __future__ import annotations

import math

SHELL, PLINTH, INTERIOR, HOOD, HARDWARE, DOOR, VENT = (
    "shell", "plinth", "interior", "hood", "hardware", "door", "vent")
CIRCLE_SEGMENTS = 32  # [E] ring, chamber: the opening reads round at the 9 m size


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _mul(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _unit(a):
    n = math.sqrt(_dot(a, a))
    return (a[0] / n, a[1] / n, a[2] / n)


def newell_normal(verts):
    nx = ny = nz = 0.0
    for i, p in enumerate(verts):
        q = verts[(i + 1) % len(verts)]
        nx += (p[1] - q[1]) * (p[2] + q[2])
        ny += (p[2] - q[2]) * (p[0] + q[0])
        nz += (p[0] - q[0]) * (p[1] + q[1])
    return (nx, ny, nz)


class Surfaces:
    """Polygons per node."""

    def __init__(self):
        self.nodes: dict[str, list] = {}

    def add(self, node, verts, hint, uv=None):
        """`hint`: a vector the polygon's normal must point along (outward); the winding flips when it does not."""
        verts = [tuple(float(c) for c in v) for v in verts]
        if len(verts) == 4 and abs(_dot(_sub(verts[2], verts[0]), _cross(_sub(verts[1], verts[0]),
                                                                          _sub(verts[3], verts[0])))) > 1e-6:
            self.add(node, [verts[0], verts[1], verts[2]], hint, uv)  # a warped quad: two triangles
            self.add(node, [verts[0], verts[2], verts[3]], hint, uv)
            return
        if _dot(newell_normal(verts), hint) < 0.0:
            verts.reverse()
        uvs = [uv(v) for v in verts] if uv else [(v[0], v[1]) for v in verts]
        self.nodes.setdefault(node, []).append((verts, uvs))

    def solid(self, node, corners, uv=None):
        """Closed hexahedron from 8 corners (bottom 4 in order around, then the 4 above them)."""
        c = [tuple(float(x) for x in p) for p in corners]
        centre = tuple(sum(p[i] for p in c) / 8.0 for i in range(3))
        for idx in ((0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)):
            face = [c[i] for i in idx]
            mid = tuple(sum(p[i] for p in face) / 4.0 for i in range(3))
            self.add(node, face, _sub(mid, centre), uv)

    def box(self, node, lo, hi, uv=None):
        x0, y0, z0 = lo
        x1, y1, z1 = hi
        self.solid(node, [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
                          (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)], uv)

    def points(self, per_triangle=6):
        """Sample points on every polygon (fan triangles, barycentric grid): the clearance tests."""
        out = []
        for polys in self.nodes.values():
            for verts, _ in polys:
                for i in range(1, len(verts) - 1):
                    a, b, c = verts[0], verts[i], verts[i + 1]
                    n = per_triangle
                    for u in range(n + 1):
                        for v in range(n + 1 - u):
                            w = n - u - v
                            out.append(tuple((u * a[k] + v * b[k] + w * c[k]) / n for k in range(3)))
        return out

    def extent(self, node=None):
        pts = [p for nd, polys in self.nodes.items() if node in (None, nd) for verts, _ in polys for p in verts]
        return tuple(min(p[k] for p in pts) for k in range(3)), tuple(max(p[k] for p in pts) for k in range(3))


class Shape:
    """The mount's numbers derived from the [mount] / [trench] tables."""

    def __init__(self, detail: dict):
        m = detail["mount"]
        self.m, self.t = m, detail["trench"]
        self.height = m["deck_height_m"]  # ring top
        self.deck_z = self.height - m["deck_drop_m"]
        self.floor_z = -m["pit_depth_m"]
        self.x_front = m["block_x_tower_m"]  # the vertical wall toward the tower
        self.x_back = m["block_x_back_m"]  # deck edge away from the tower, the annex starts here
        self.half_y = m["block_half_y_m"]  # the block's end faces, the portals
        self.annex_x = m["annex_end_x_m"]
        self.annex_half_y = m["annex_half_y_m"]
        self.annex_tip_half_y = m["annex_tip_half_y_m"]
        self.half_w = 0.5 * self.t["width_m"]  # tunnel half width: one value from end to end
        self.ring_r = 0.5 * m["ring_outer_diameter_m"]
        self.open_r = 0.5 * m["ring_opening_diameter_m"]
        self.ring_bottom = self.height - m["ring_thickness_m"]
        self.open_top = m["opening_top_m"]
        self.ramp_end = self.half_y + self.t["length_m"]
        self.wall_out = {-1.0: -self.x_back, 1.0: self.x_front}  # the walls' outer faces are flush with the deck plate's side edges
        self.lip_y = self.annex_half_y  # the wall top has fallen to the small wall where the annex footprint ends
        self.lip_z = self.t["wall_height_m"]  # small wall of this height above the ground, level to the ramp end

    def back_z(self, x):
        """Height of the annex's back slope at x (x from -annex_x to -x_back)."""
        return self.deck_z * (x + self.annex_x) / (self.annex_x - self.x_back)

    def back_x(self, z):
        return -self.annex_x + (self.annex_x - self.x_back) * z / self.deck_z

    def floor_at(self, y):
        """Tunnel floor height at |y|: level under the block, straight up to the ground at the ramp end."""
        a = abs(y)
        if a <= self.half_y:
            return self.floor_z
        return self.floor_z * max(0.0, 1.0 - (a - self.half_y) / (self.ramp_end - self.half_y))


def _circle(r, z, angles):
    return [(r * math.cos(a), r * math.sin(a), z) for a in angles]


def _box_point(a, x0, x1, y0, y1, z):
    """Where the ray at angle a from the origin leaves the rectangle x0..x1, y0..y1 (origin inside)."""
    c, s = math.cos(a), math.sin(a)
    tx = (x1 / c if c > 1e-12 else x0 / c) if abs(c) > 1e-12 else 1e9
    ty = (y1 / s if s > 1e-12 else y0 / s) if abs(s) > 1e-12 else 1e9
    t = min(tx, ty)
    return (t * c, t * s, z)


def _uv_x(p):  # walls in a plane of constant Y: u along X, v up
    return (p[0], p[2])


def _uv_y(p):  # walls in a plane of constant X: u along Y, v up
    return (p[1], p[2])


def _uv_plan(p):
    return (p[0], p[1])


def _plane_frame(p0, p1, p2, hint):
    """(a, b, n) of a planar face: a horizontal along it, b up the face, n out (towards `hint`)."""
    n = _unit(_cross(_sub(p1, p0), _sub(p2, p0)))
    if _dot(n, hint) < 0.0:
        n = _mul(n, -1.0)
    a = _unit(_cross((0.0, 0.0, 1.0), n))
    b = _cross(n, a)
    if b[2] < 0.0:
        a, b = _mul(a, -1.0), _mul(b, -1.0)
    return a, b, n


def _plane_uv(frame, origin):
    a, b, _ = frame
    return lambda p: (_dot(_sub(p, origin), a), _dot(_sub(p, origin), b))


def _annulus_to_box(S, node, radius, x0, x1, y0, y1, z, hint):
    """Flat ring between a circle and the rectangle around it, as a fan of quads (corner angles included)."""
    base = [2.0 * math.pi * k / CIRCLE_SEGMENTS for k in range(CIRCLE_SEGMENTS)]
    corners = [math.atan2(y, x) % (2.0 * math.pi) for x, y in ((x1, y1), (x0, y1), (x0, y0), (x1, y0))]
    angles = sorted(base + corners)
    for i, a in enumerate(angles):
        b = angles[(i + 1) % len(angles)] + (2.0 * math.pi if i == len(angles) - 1 else 0.0)
        S.add(node, [_circle(radius, z, [a])[0], _circle(radius, z, [b])[0], _box_point(b, x0, x1, y0, y1, z),
                     _box_point(a, x0, x1, y0, y1, z)], hint, _uv_plan)


def _block(S, sh):
    """The vertical walls of the block: the tower side, and both ends (+-Y) with the tunnel's rectangular portal."""
    zd, zo, hw = sh.deck_z, sh.open_top, sh.half_w
    S.add(SHELL, [(sh.x_front, -sh.half_y, 0.0), (sh.x_front, sh.half_y, 0.0), (sh.x_front, sh.half_y, zd),
                  (sh.x_front, -sh.half_y, zd)], (1.0, 0.0, 0.0), _uv_y)
    for sy in (-1.0, 1.0):
        y = sy * sh.half_y
        n = (0.0, sy, 0.0)
        S.add(SHELL, [(-sh.x_back, y, 0.0), (-hw, y, 0.0), (-hw, y, zd), (-sh.x_back, y, zd)], n, _uv_x)
        S.add(SHELL, [(hw, y, 0.0), (sh.x_front, y, 0.0), (sh.x_front, y, zd), (hw, y, zd)], n, _uv_x)
        S.add(SHELL, [(-hw, y, zo), (hw, y, zo), (hw, y, zd), (-hw, y, zd)], n, _uv_x)


def _annex(S, sh):
    """The pyramid on the side away from the tower: the back slope, the two side slopes and the flank triangles."""
    zd = sh.deck_z
    xb, xa = -sh.x_back, -sh.annex_x
    top = [(xb, sy * sh.half_y, zd) for sy in (-1.0, 1.0)]
    tips = [(xa, sy * sh.annex_tip_half_y, 0.0) for sy in (-1.0, 1.0)]
    back = [top[0], top[1], tips[1], tips[0]]
    f = _plane_frame(back[0], back[1], back[2], (-1.0, 0.0, 0.4))
    S.add(SHELL, back, f[2], _plane_uv(f, back[3]))
    for sy in (-1.0, 1.0):
        t = (xb, sy * sh.half_y, zd)
        foot = (xb, sy * sh.annex_half_y, 0.0)
        tip = (xa, sy * sh.annex_tip_half_y, 0.0)
        f = _plane_frame(t, foot, tip, (-0.4, sy, 0.4))
        S.add(SHELL, [t, foot, tip], f[2], _plane_uv(f, tip))
        S.add(SHELL, [t, (xb, sy * sh.half_y, 0.0), foot], (1.0, 0.0, 0.0), _uv_y)  # the flank against the block


def _slab_on_face(S, node, p0, p1, p2, hint, centre, w, h, depth):
    """A thin box of w x h (metres, in the face) standing `depth` out of the planar face p0-p1-p2, at `centre` on it."""
    a, b, n = _plane_frame(p0, p1, p2, hint)
    pts = []
    for sa, sb in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        pts.append(_add(centre, _add(_mul(a, 0.5 * w * sa), _mul(b, 0.5 * h * sb))))
    S.solid(node, pts + [_add(p, _mul(n, depth)) for p in pts], _plane_uv((a, b, n), centre))


def _annex_details(S, sh):
    """Door group at the foot of the back slope, white vent plates on it and on the flanks, dark louvre boxes along the
    slanted edges (CRL05690, DSC06000)."""
    m = sh.m
    zd = sh.deck_z
    xb, xa = -sh.x_back, -sh.annex_x
    lift = m["door_lift_m"]
    for sy in (-1.0, 1.0):
        yc, half = sy * m["door_y_m"], 0.5 * m["door_width_m"]
        pts = [(yc - half, 0.0), (yc + half, 0.0), (yc + half, m["door_height_m"]), (yc - half, m["door_height_m"])]
        S.add(DOOR, [(sh.back_x(z) - lift, y, z) for y, z in pts], (-1.0, 0.0, 0.7), _uv_y)
    back = [(xb, -sh.half_y, zd), (xb, sh.half_y, zd), (xa, sh.annex_tip_half_y, 0.0)]
    vent_w, vent_h = m["vent_width_m"], m["vent_height_m"]
    for sy in (-1.0, 1.0):
        for yv, zc in ((sy * m["vent_y_m"], zd - m["vent_below_deck_m"]), (sy * m["vent_outer_y_m"] * 0.7, zd - m["vent_outer_below_deck_m"])):
            centre = (sh.back_x(zc), yv, zc)
            _slab_on_face(S, VENT, back[0], back[1], back[2], (-1.0, 0.0, 0.7), centre, vent_w, vent_h, m["vent_depth_m"])
        t = (xb, sy * sh.half_y, zd)
        foot = (xb, sy * sh.annex_half_y, 0.0)
        tip = (xa, sy * sh.annex_tip_half_y, 0.0)
        for k in range(int(m["louvre_count"])):
            w = (k + 1.0) / (m["louvre_count"] + 1.0)
            # along the slanted edge from the top corner to the foot corner, a little inside it
            centre = _add(_add(_mul(t, 0.8 * (1.0 - w) + 0.1), _mul(foot, 0.8 * w)), _mul(tip, 0.1))
            _slab_on_face(S, INTERIOR, t, foot, tip, (-0.4, sy, 0.4), centre, m["louvre_width_m"], m["louvre_height_m"],
                          m["louvre_depth_m"])
        centre = _add(_add(_mul(t, 0.25), _mul(foot, 0.35)), _mul(tip, 0.4))  # a white plate in the flank's middle
        _slab_on_face(S, VENT, t, foot, tip, (-0.4, sy, 0.4), centre, vent_w, vent_h, m["vent_depth_m"])


def _tunnel(S, sh):
    """The part of the tunnel inside the block: side walls, ceiling with the chamber hole, chamber wall, deck plate."""
    m = sh.m
    zf, zo, zd, hw = sh.floor_z, sh.open_top, sh.deck_z, sh.half_w
    for side in (-1.0, 1.0):
        x = side * hw
        S.add(INTERIOR, [(x, -sh.half_y, zf), (x, sh.half_y, zf), (x, sh.half_y, zo), (x, -sh.half_y, zo)],
              (-side, 0.0, 0.0), _uv_y)
    _annulus_to_box(S, INTERIOR, m["chamber_radius_m"], -hw, hw, -sh.half_y, sh.half_y, zo, (0.0, 0.0, -1.0))  # ceiling
    ring_angles = [2.0 * math.pi * k / CIRCLE_SEGMENTS for k in range(CIRCLE_SEGMENTS + 1)]
    r_c = m["chamber_radius_m"]
    z_top = zd - m["deck_plate_m"]
    for a, b in zip(ring_angles, ring_angles[1:]):
        pa, pb = _circle(r_c, 0.0, [a])[0], _circle(r_c, 0.0, [b])[0]
        inward = (-0.5 * (pa[0] + pb[0]), -0.5 * (pa[1] + pb[1]), 0.0)
        S.add(INTERIOR, [(pa[0], pa[1], zo), (pb[0], pb[1], zo), (pb[0], pb[1], z_top), (pa[0], pa[1], z_top)], inward,
              _uv_y)
        ia, ib = _circle(sh.ring_r, z_top, [a])[0], _circle(sh.ring_r, z_top, [b])[0]
        S.add(INTERIOR, [ia, ib, (pb[0], pb[1], z_top), (pa[0], pa[1], z_top)], (0.0, 0.0, -1.0), _uv_plan)


def _deck_and_ring(S, sh):
    m = sh.m
    zd = sh.deck_z
    up = (0.0, 0.0, 1.0)
    _annulus_to_box(S, SHELL, sh.ring_r, -sh.x_back, sh.x_front, -sh.half_y, sh.half_y, zd, up)  # flat deck
    z_low = zd - m["deck_plate_m"]
    ring_angles = [2.0 * math.pi * k / CIRCLE_SEGMENTS for k in range(CIRCLE_SEGMENTS + 1)]
    r_in, r_out, z_t, z_b = sh.open_r, sh.ring_r, sh.height, sh.ring_bottom
    for a, b in zip(ring_angles, ring_angles[1:]):
        pa_i, pb_i = _circle(r_in, z_t, [a])[0], _circle(r_in, z_t, [b])[0]
        pa_o, pb_o = _circle(r_out, z_t, [a])[0], _circle(r_out, z_t, [b])[0]
        S.add(HARDWARE, [pa_i, pb_i, pb_o, pa_o], up, _uv_plan)
        mid = (0.5 * (pa_o[0] + pb_o[0]), 0.5 * (pa_o[1] + pb_o[1]), 0.0)
        S.add(HARDWARE, [pa_o, pb_o, (pb_o[0], pb_o[1], z_low), (pa_o[0], pa_o[1], z_low)], mid, _uv_y)
        S.add(HARDWARE, [pa_i, pb_i, (pb_i[0], pb_i[1], z_b), (pa_i[0], pa_i[1], z_b)], (-mid[0], -mid[1], 0.0), _uv_y)
        S.add(INTERIOR, [(pa_i[0], pa_i[1], z_b), (pb_i[0], pb_i[1], z_b), (pb_o[0], pb_o[1], z_b), (pa_o[0], pa_o[1], z_b)],
              (0.0, 0.0, -1.0), _uv_plan)
        S.add(INTERIOR, [(pa_o[0], pa_o[1], z_b), (pb_o[0], pb_o[1], z_b), (pb_o[0], pb_o[1], z_low),
                         (pa_o[0], pa_o[1], z_low)], (-mid[0], -mid[1], 0.0), _uv_y)  # seen from inside the chamber
    for k in range(int(m["clamp_count"])):
        a = 2.0 * math.pi * (k + 0.5) / m["clamp_count"]
        ca, sa = math.cos(a), math.sin(a)
        r0, r1 = m["clamp_inner_radius_m"], m["clamp_inner_radius_m"] + m["clamp_width_m"]
        hl = 0.5 * m["clamp_length_m"]
        z0, z1 = sh.height, sh.height + m["clamp_height_m"]

        def at(r, t, z):
            return (r * ca - t * sa, r * sa + t * ca, z)

        S.solid(HARDWARE, [at(r0, -hl, z0), at(r1, -hl, z0), at(r1, hl, z0), at(r0, hl, z0),
                           at(r0, -hl, z1), at(r1, -hl, z1), at(r1, hl, z1), at(r0, hl, z1)])


def _about_z(p, angle):
    c, s = math.cos(angle), math.sin(angle)
    return (c * p[0] - s * p[1], s * p[0] + c * p[1], p[2])


def _radial_uv(p):
    return (math.hypot(p[0], p[1]), p[2])


def _hood(S, sh, profile, half_width, turn):
    """One bent duct drawn along -X, turned about the ring's axis: side faces, the outer skin, the root block."""
    m = sh.m
    zd = sh.deck_z
    x_root = m["hood_root_x_m"]

    def at(x, y, z):
        return _about_z((x, y, z), turn)

    for sy in (-1.0, 1.0):
        S.add(HOOD, [at(x, sy * half_width, z) for x, z in profile], at(0.0, sy, 0.0), _radial_uv)
    for (xa, za), (xb, zb) in zip(profile, profile[1:]):
        out = (-(zb - za), 0.0, xb - xa)  # the profile runs clockwise seen from +Y: this is its outside
        if _dot(out, (xa + xb - 2.0 * x_root + 0.1, 0.0, za + zb - 2.0 * zd)) < 0.0:
            out = (-out[0], 0.0, -out[2])
        S.add(HOOD, [at(xa, -half_width, za), at(xb, -half_width, zb), at(xb, half_width, zb), at(xa, half_width, za)],
              _about_z(out, turn), _radial_uv)
    # Tan root block on the ring side of the duct (padb_chopsticks_above_mount_2026-05).
    x1, z1 = x_root + m["hood_root_depth_m"], zd + m["hood_root_height_m"]
    S.solid(PLINTH, [at(x_root, -half_width, zd), at(x1, -half_width, zd), at(x1, half_width, zd), at(x_root, half_width, zd),
                     at(x_root, -half_width, z1), at(x1, -half_width, z1), at(x1, half_width, z1), at(x_root, half_width, z1)],
            _radial_uv)


def hood_yaws(detail: dict) -> tuple:
    """The two hoods' turns about the ring axis (rad): symmetric about the line away from the tower."""
    m = detail["mount"]
    yaw, half_split = math.radians(m["plan_yaw_deg"]), 0.5 * math.radians(m["hood_split_deg"])
    return yaw - half_split, yaw + half_split


def _hoods(S, sh):
    """Two ducts `hood_split_deg` apart (DSC05957): the long one runs over the deck edge and down the middle of the annex,
    the short one ends at the -Y deck edge above the portal. Each covers a booster QD head standing on the ring."""
    m = sh.m
    zd = sh.deck_z
    x_root, x_roof, x_front = m["hood_root_x_m"], m["hood_roof_x_m"], m["hood_front_x_m"]
    top, shoulder = zd + m["hood_height_m"], zd + m["hood_shoulder_m"]
    turn_long, turn_short = hood_yaws({"mount": m})
    z_slope = sh.back_z(x_front)  # the box stands on the annex's back slope
    _hood(S, sh, [(x_root, zd), (x_root, top), (x_roof, top), (x_front, shoulder), (x_front, z_slope)],
          0.5 * m["hood_long_width_m"], turn_long)
    x_short = -m["hood_short_front_m"]
    _hood(S, sh, [(x_root, zd), (x_root, top), (-m["hood_short_roof_m"], top), (x_short, shoulder), (x_short, zd)],
          0.5 * m["hood_short_width_m"], turn_short)


def _qd_heads(S, sh):
    """Booster QD heads on the launch ring, one under each hood, `qd_head_radius_m` from the axis (0.35 m off the hull);
    static (whether the covers move at liftoff is unknown)."""
    m = sh.m
    front, depth = m["qd_head_radius_m"], m["qd_head_depth_m"]
    hw, zb, zt = 0.5 * m["qd_head_width_m"], sh.height, sh.height + m["qd_head_height_m"]
    for turn in hood_yaws({"mount": m}):
        S.solid(HARDWARE, [_about_z(p, turn) for p in (
            (-front, -hw, zb), (-front - depth, -hw, zb), (-front - depth, hw, zb), (-front, hw, zb),
            (-front, -hw, zt), (-front - depth, -hw, zt), (-front - depth, hw, zt), (-front, hw, zt))])


def mount(detail: dict) -> Surfaces:
    sh = Shape(detail)
    S = Surfaces()
    _block(S, sh)
    _annex(S, sh)
    _annex_details(S, sh)
    _tunnel(S, sh)
    _deck_and_ring(S, sh)
    _hoods(S, sh)
    _qd_heads(S, sh)
    return S


def hardstand_outline(detail: dict) -> list:
    """The concrete outline in the mount frame (X at bearing azimuth + plan yaw, Y 90 deg clockwise of it): the user's sketch
    `hardstand_outline_m` ("east,north east,north ..." in metres from the ring centre: a string, Blender's TOML reader has no arrays) rotated into that frame. Not convex."""
    site = detail["site_tower"]
    base = math.radians(site["azimuth_deg"] + site["plan_yaw_deg"])
    return [(e * math.sin(base) + n * math.cos(base),
             e * math.sin(base + 0.5 * math.pi) + n * math.cos(base + 0.5 * math.pi))
            for e, n in (tuple(float(c) for c in pair.split(",")) for pair in detail["trench"]["hardstand_outline_m"].split())]


def polygon_area(poly) -> float:
    return 0.5 * abs(sum(p[0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * p[1] for i, p in enumerate(poly)))


def _signed_area(poly) -> float:
    return 0.5 * sum(p[0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * p[1] for i, p in enumerate(poly))


def _cross2(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def triangulate(poly) -> list:
    """Ear clipping of a simple polygon: counter-clockwise triangles."""
    pts = list(poly) if _signed_area(poly) > 0.0 else list(reversed(poly))
    tris = []
    while len(pts) > 3:
        for i in range(len(pts)):
            a, b, c = pts[i - 1], pts[i], pts[(i + 1) % len(pts)]
            if _cross2(a, b, c) <= 1e-12:
                continue
            if any(q not in (a, b, c) and _cross2(a, b, q) >= 0.0 and _cross2(b, c, q) >= 0.0 and _cross2(c, a, q) >= 0.0
                   for q in pts):
                continue
            tris.append((a, b, c))
            del pts[i]
            break
        else:
            raise ValueError("outline is not a simple polygon")
    tris.append(tuple(pts))
    return tris


def clip_halfplane(poly, nx, ny, d):
    """The part of the convex `poly` with nx x + ny y <= d (Sutherland-Hodgman)."""
    out = []
    for i, p in enumerate(poly):
        q = poly[(i + 1) % len(poly)]
        fp, fq = nx * p[0] + ny * p[1] - d, nx * q[0] + ny * q[1] - d
        if fp <= 0.0:
            out.append(p)
        if (fp < 0.0 < fq) or (fq < 0.0 < fp):
            t = fp / (fp - fq)
            out.append((p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])))
    return out if len(out) >= 3 and polygon_area(out) > 1e-6 else []


def clip_rect(poly, x0, x1, y0, y1):
    for nx, ny, d in ((1.0, 0.0, x1), (-1.0, 0.0, -x0), (0.0, 1.0, y1), (0.0, -1.0, -y0)):
        poly = clip_halfplane(poly, nx, ny, d) if poly else poly
    return poly


def minus_rect(poly, x0, x1, y0, y1):
    """The convex `poly` minus the rectangle: up to four convex pieces."""
    pieces = []
    for nx, ny, d in ((-1.0, 0.0, -x1), (1.0, 0.0, x0), (0.0, -1.0, -y1), (0.0, 1.0, y0)):  # beyond one side of the rectangle
        part = clip_halfplane(poly, nx, ny, d)
        if part:
            pieces.append(part)
        poly = clip_halfplane(poly, -nx, -ny, -d)
        if not poly:
            break
    return pieces


def hardstand_holes(sh) -> list:
    """Rectangles (x0, x1, y0, y1) the apron leaves open: the block with its annex and the two ramps with their walls."""
    ramps = [(sh.wall_out[-1.0], sh.wall_out[1.0], sy0, sy1) for sy0, sy1 in ((sh.half_y, sh.ramp_end), (-sh.ramp_end, -sh.half_y))]
    return [(-sh.annex_x, sh.x_front, -sh.half_y, sh.half_y)] + ramps


def hardstand_pieces(detail: dict, holes=None) -> list:
    """The apron as convex pieces (mount frame, plan): the outline's ear triangles minus `holes` (default the mount's block, annex and ramps)."""
    holes = hardstand_holes(Shape(detail)) if holes is None else holes
    pieces = []
    for tri in triangulate(hardstand_outline(detail)):
        parts = [list(tri)]
        for hole in holes:
            parts = [q for part in parts for q in minus_rect(part, *hole)]
        pieces += parts
    return pieces


def _hardstand(S, sh, detail):
    """The flat light concrete apron at `hardstand_lift_m` over the site height (the area is graded: one plane, no terrain following;
    the terrain under it is masked, see hardstand_mask): the user's outline minus the block, annex and ramps."""
    lift = sh.t["hardstand_lift_m"]
    for piece in hardstand_pieces(detail):
        S.add("hardstand", [(x, y, lift) for x, y in piece], (0.0, 0.0, 1.0), _uv_plan)


def trench(detail: dict) -> Surfaces:
    """The tunnel's closed floor (level under the block, straight ramps to both ends), the tunnel walls outside the block (falling with the annex to a low lip, then level),
    the hardstand and the tower's foot block; frame as the mount."""
    sh = Shape(detail)
    t = sh.t
    S = Surfaces()
    zf, zd = sh.floor_z, sh.deck_z
    y0, y1 = sh.half_y, sh.ramp_end
    hw = sh.half_w
    up = (0.0, 0.0, 1.0)
    S.add("concrete", [(-hw, -y0, zf), (hw, -y0, zf), (hw, y0, zf), (-hw, y0, zf)], up, _uv_plan)  # under the ring
    for sy in (-1.0, 1.0):
        S.add("concrete", [(-hw, sy * y0, zf), (hw, sy * y0, zf), (hw, sy * y1, 0.0), (-hw, sy * y1, 0.0)], up, _uv_plan)
        for side in (-1.0, 1.0):
            x, xo = side * hw, sh.wall_out[side]
            # Wall top (photo CRL05690): from the deck edge it falls with the annex to the low lip, then runs level to the ramp end.
            yl, zl = sh.lip_y, sh.lip_z
            for xs, z_bottom, out in ((x, None, -side), (xo, 0.0, side)):
                low0 = zf if z_bottom is None else z_bottom
                S.add("wall", [(xs, sy * y0, low0), (xs, sy * y1, 0.0), (xs, sy * y1, zl), (xs, sy * yl, zl)], (out, 0.0, 0.0), _uv_y)
                S.add("wall", [(xs, sy * y0, low0), (xs, sy * yl, zl), (xs, sy * y0, zd)], (out, 0.0, 0.0), _uv_y)
            S.add("wall", [(x, sy * y0, zd), (xo, sy * y0, zd), (xo, sy * yl, zl), (x, sy * yl, zl)], up, _uv_plan)
            S.add("wall", [(x, sy * yl, zl), (xo, sy * yl, zl), (xo, sy * y1, zl), (x, sy * y1, zl)], up, _uv_plan)
            S.add("wall", [(x, sy * y1, 0.0), (xo, sy * y1, 0.0), (xo, sy * y1, zl), (x, sy * y1, zl)], (0.0, sy, 0.0), _uv_x)
    _hardstand(S, sh, detail)
    # The tower's foot: a block cut vertically on the side toward the mount, its far side chamfered up to the legs' plinth.
    cx, cy, half, top = t["foot_x_m"], t["foot_y_m"], 0.5 * t["foot_width_m"], t["foot_height_m"]
    near, far = t["foot_near_x_m"], t["foot_far_x_m"]
    roof, roof_half = t["foot_roof_height_m"], t["foot_roof_half_m"]
    S.box("foot", (near, cy - half, 0.0), (far, cy + half, top), _uv_x)
    S.solid("foot", [(near, cy - half, top), (far, cy - half, top), (far, cy + half, top), (near, cy + half, top),
                     (near, cy - roof_half, roof), (cx + roof_half, cy - roof_half, roof),
                     (cx + roof_half, cy + roof_half, roof), (near, cy + roof_half, roof)], _uv_x)
    return S


def ramp_end_m(detail: dict) -> float:
    return Shape(detail).ramp_end


# ---------------------------------------------------------------- clearance of the stack

def bell_radius_m(engine: dict, depth_m: float) -> float:
    """Radius of a Raptor bell at `depth_m` below its gimbal point (gen_engine.py's contour; the chamber housing above)."""
    r_e, r_t = 0.5 * engine["nozzle_exit_diameter_m"], 0.5 * engine["throat_diameter_m"]
    length, throat = engine["bell_length_m"], engine["throat_from_exit_m"]
    x_throat = length - throat  # depth of the throat below the gimbal point
    r_h = engine["mount_radius_frac"] * r_e
    if depth_m <= 0.0:
        return r_h
    if depth_m < x_throat:
        return r_h + (r_t - r_h) * depth_m / x_throat  # the housing's cone from the gimbal point to the throat
    z = min(depth_m - x_throat, throat)
    return r_t + (r_e - r_t) * (z / throat) ** engine["contour_exponent"]


def stack_violations(surfaces: Surfaces, height: float, hull_radius: float, ring_radii: list, engine: dict,
                     margin: float = 0.04) -> list:
    """Sample points of the mount inside the stack's volume (hull above the deck, the engines' bells below it)."""
    bad = []
    for p in surfaces.points():
        r = math.hypot(p[0], p[1])
        if p[2] > height + 1e-6 and r < hull_radius + margin - 1e-9:
            bad.append(p)
            continue
        depth = height - p[2]
        if 0.0 < depth <= engine["bell_length_m"] and r < max(rr + bell_radius_m(engine, depth) for rr in ring_radii) + margin:
            bad.append(p)
    return bad
