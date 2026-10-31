"""Pure layout of the Pad B launch mount and its trench: polygons per glTF node, no Blender.

gen_mount.py / gen_trench.py add the polygons to meshes; pytest reads the same lists (shape, clearances).
Frame (both models): X toward the tower, Y across, Z up, origin on the mount axis at the ground (the site height).
A surface is (verts, uvs): planar polygon, counter-clockwise seen from outside (the winding is fixed by an outward hint
in `Surfaces.add`, so the glTF normals point out). UV in metres on the surface; the shell shader repeats the panel map.

Shape (launch-structures-measurements.md "Mount shape" + the pyramid photos): a stepped pyramid, plinth band, ledge,
four sloped faces up to a flat deck, the ring (opening, clamps) on the deck, two tunnel mouths in the +-Y faces
(the exits, each on its trench ramp), two hoods flanking the deck on the tower side, two vent boxes, the booster QDs,
two door arches and a diagonal stair strip in the -X (front) face, an engine chamber with a flame ridge below the ring.
The frustum's vertical scale is the physics deck height (deck_height_m) over the 14.5 m measured: only the slope's
steepness differs from the photos, the plan sizes are the measured ones.
"""
from __future__ import annotations

import math

SHELL, PLINTH, INTERIOR, DEFLECTOR, HOOD, HARDWARE, STAIR, DOOR = (
    "shell", "plinth", "interior", "deflector", "hood", "hardware", "stair", "door")
CIRCLE_SEGMENTS = 32  # [E] ring, chamber: the opening reads round at the 9 m size


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


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
        self.plinth_z = m["plinth_height_m"]
        self.half_a = 0.5 * m["base_across_m"]
        self.half_t = 0.5 * m["base_toward_m"]
        self.run = self.half_a - m["ledge_m"] - 0.5 * m["top_across_m"]
        self.top_half_a = 0.5 * m["top_across_m"]
        self.top_half_t = self.half_t - m["ledge_m"] - self.run
        self.slope_len = math.hypot(self.run, self.deck_z - self.plinth_z) / (self.deck_z - self.plinth_z)
        self.half_w = 0.5 * m["tunnel_width_m"]
        self.ring_r = 0.5 * m["ring_outer_diameter_m"]
        self.open_r = 0.5 * m["ring_opening_diameter_m"]
        self.ring_bottom = self.height - m["ring_thickness_m"]

    def hx(self, z):  # half extent along the mount-tower line at height z on the slope
        return self.half_t - self.m["ledge_m"] - self.run * (z - self.plinth_z) / (self.deck_z - self.plinth_z)

    def hy(self, z):
        return self.half_a - self.m["ledge_m"] - self.run * (z - self.plinth_z) / (self.deck_z - self.plinth_z)


def _circle(r, z, angles):
    return [(r * math.cos(a), r * math.sin(a), z) for a in angles]


def _rect_point(a, hx, hy, z):
    """Where the ray at angle a leaves the rectangle |x| <= hx, |y| <= hy."""
    c, s = math.cos(a), math.sin(a)
    t = min(hx / abs(c) if abs(c) > 1e-12 else 1e9, hy / abs(s) if abs(s) > 1e-12 else 1e9)
    return (t * c, t * s, z)


def mount(detail: dict) -> Surfaces:
    sh = Shape(detail)
    m = sh.m
    S = Surfaces()
    up = (0.0, 0.0, 1.0)
    zp, zd, zf = sh.plinth_z, sh.deck_z, sh.floor_z
    ledge = m["ledge_m"]
    zm = m["tunnel_top_m"]
    w = sh.half_w

    def slope_uv_x(p):  # +-X faces: u across (Y), v along the slope
        return (p[1], p[2] * sh.slope_len)

    def slope_uv_y(p):  # +-Y faces
        return (p[0], p[2] * sh.slope_len)

    def wall_uv_x(p):
        return (p[1], p[2])

    def wall_uv_y(p):
        return (p[0], p[2])

    def floor_uv(p):
        return (p[0], p[1])

    for sx in (-1.0, 1.0):  # +-X faces (front: -X, tower side: +X): plain
        hz0, hz1 = sh.hy(zp), sh.hy(zd)
        out = (sx, 0.0, 0.0)
        S.add(SHELL, [(sx * sh.hx(zp), -hz0, zp), (sx * sh.hx(zp), hz0, zp), (sx * sh.hx(zd), hz1, zd),
                      (sx * sh.hx(zd), -hz1, zd)], (sx, 0.0, 0.4), slope_uv_x)
        S.add(PLINTH, [(sx * sh.half_t, -sh.half_a, zf), (sx * sh.half_t, sh.half_a, zf),
                       (sx * sh.half_t, sh.half_a, zp), (sx * sh.half_t, -sh.half_a, zp)], out, wall_uv_x)
    # Ledge: the plinth's flat top between the plinth edge and the slope's foot (the +-Y strips are cut by the mouth).
    for sx in (-1.0, 1.0):
        S.add(PLINTH, [(sx * sh.half_t, -sh.half_a, zp), (sx * sh.half_t, sh.half_a, zp), (sx * sh.hx(zp), sh.hy(zp), zp),
                       (sx * sh.hx(zp), -sh.hy(zp), zp)], up, floor_uv)
    for sy in (-1.0, 1.0):
        for side in (-1.0, 1.0):
            S.add(PLINTH, [(side * w, sy * sh.half_a, zp), (side * sh.half_t, sy * sh.half_a, zp),
                           (side * sh.hx(zp), sy * sh.hy(zp), zp), (side * w, sy * sh.hy(zp), zp)], up, floor_uv)
    # +-Y faces with the mouth: plinth wall, slope pieces, lintel, tunnel walls / ceiling / floor.
    for sy in (-1.0, 1.0):
        out = (0.0, sy, 0.0)
        for lo, hi in ((-sh.half_t, -w), (w, sh.half_t)):
            S.add(PLINTH, [(lo, sy * sh.half_a, zf), (hi, sy * sh.half_a, zf), (hi, sy * sh.half_a, zp),
                           (lo, sy * sh.half_a, zp)], out, wall_uv_y)
        for side in (-1.0, 1.0):
            edge = side * w
            S.add(SHELL, [(side * sh.hx(zp), sy * sh.hy(zp), zp), (edge, sy * sh.hy(zp), zp),
                          (edge, sy * sh.hy(zd), zd), (side * sh.hx(zd), sy * sh.hy(zd), zd)], (0.0, sy, 0.4),
                  slope_uv_y)
        S.add(HOOD, [(-w, sy * sh.hy(zm), zm), (w, sy * sh.hy(zm), zm), (w, sy * sh.hy(zd), zd),
                     (-w, sy * sh.hy(zd), zd)], (0.0, sy, 0.4), slope_uv_y)  # the light-grey panel over the mouth
        y_in = math.sqrt(max(sh.m["chamber_radius_m"] ** 2 - w * w, 0.0))
        for side in (-1.0, 1.0):  # tunnel side walls (face the tunnel axis)
            x = side * w
            S.add(INTERIOR, [(x, sy * y_in, zf), (x, sy * sh.half_a, zf), (x, sy * sh.half_a, zp),
                             (x, sy * sh.hy(zm), zm), (x, sy * y_in, zm)], (-side, 0.0, 0.0), wall_uv_y)
        S.add(INTERIOR, [(-w, sy * y_in, zm), (w, sy * y_in, zm), (w, sy * sh.hy(zm), zm), (-w, sy * sh.hy(zm), zm)],
              (0.0, 0.0, -1.0), floor_uv)  # ceiling
        S.add(INTERIOR, [(-w, sy * y_in, zf), (w, sy * y_in, zf), (w, sy * sh.half_a, zf), (-w, sy * sh.half_a, zf)],
              up, floor_uv)  # floor (the ramp continues it)
    # Deck: a flat plate between the slope's top edge and the ring, as a fan of quads (corner angles included).
    base_angles = [2.0 * math.pi * k / CIRCLE_SEGMENTS for k in range(CIRCLE_SEGMENTS)]
    corner = math.atan2(sh.hy(zd), sh.hx(zd))
    angles = sorted(base_angles + [corner, math.pi - corner, math.pi + corner, -corner + 2.0 * math.pi])
    for i, a in enumerate(angles):
        b = angles[(i + 1) % len(angles)] + (2.0 * math.pi if i == len(angles) - 1 else 0.0)
        S.add(SHELL, [(*_circle(sh.ring_r, zd, [a])[0][:2], zd), (*_circle(sh.ring_r, zd, [b])[0][:2], zd),
                      _rect_point(b, sh.hx(zd), sh.hy(zd), zd), _rect_point(a, sh.hx(zd), sh.hy(zd), zd)], up,
              floor_uv)
    # Ring: top annulus, inner / outer wall, underside; clamps on the top.
    ring_angles = [2.0 * math.pi * k / CIRCLE_SEGMENTS for k in range(CIRCLE_SEGMENTS + 1)]
    for a, b in zip(ring_angles, ring_angles[1:]):
        r_in, r_out, z_t, z_b = sh.open_r, sh.ring_r, sh.height, sh.ring_bottom
        pa_i, pb_i = _circle(r_in, z_t, [a])[0], _circle(r_in, z_t, [b])[0]
        pa_o, pb_o = _circle(r_out, z_t, [a])[0], _circle(r_out, z_t, [b])[0]
        S.add(HARDWARE, [pa_i, pb_i, pb_o, pa_o], up, floor_uv)
        mid = (0.5 * (pa_o[0] + pb_o[0]), 0.5 * (pa_o[1] + pb_o[1]), 0.0)
        S.add(HARDWARE, [pa_o, pb_o, (pb_o[0], pb_o[1], z_b), (pa_o[0], pa_o[1], z_b)], mid, wall_uv_x)
        S.add(HARDWARE, [pa_i, pb_i, (pb_i[0], pb_i[1], z_b), (pa_i[0], pa_i[1], z_b)], (-mid[0], -mid[1], 0.0),
              wall_uv_x)
        c_o = _circle(sh.m["chamber_radius_m"], z_b, [a])[0], _circle(sh.m["chamber_radius_m"], z_b, [b])[0]
        S.add(INTERIOR, [(pa_i[0], pa_i[1], z_b), (pb_i[0], pb_i[1], z_b), (c_o[1][0], c_o[1][1], z_b),
                         (c_o[0][0], c_o[0][1], z_b)], (0.0, 0.0, -1.0), floor_uv)
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
    # Chamber: wall (arcs only where no tunnel opens below the mouth top, full above), floor, flame ridge.
    r_c = m["chamber_radius_m"]
    for a, b in zip(ring_angles, ring_angles[1:]):
        pa, pb = _circle(r_c, 0.0, [a])[0], _circle(r_c, 0.0, [b])[0]
        mid = (0.5 * (pa[0] + pb[0]), 0.5 * (pa[1] + pb[1]))
        inward = (-mid[0], -mid[1], 0.0)
        for z0, z1 in ((zf, zm), (zm, sh.ring_bottom)):
            if z0 == zf and abs(mid[0]) < w:
                continue  # a tunnel opens here
            S.add(INTERIOR, [(pa[0], pa[1], z0), (pb[0], pb[1], z0), (pb[0], pb[1], z1), (pa[0], pa[1], z1)], inward,
                  wall_uv_x)
    disc = [_circle(r_c, zf, [a])[0] for a in ring_angles[:-1]]
    for i in range(len(disc)):
        S.add(INTERIOR, [(0.0, 0.0, zf), disc[i], disc[(i + 1) % len(disc)]], up, floor_uv)  # floor fan (the ridge stands on it)
    rw, rh = 0.5 * m["deflector_width_m"], m["deflector_height_m"]
    for sx in (-1.0, 1.0):
        S.add(DEFLECTOR, [(sx * rw, -r_c + 0.3, zf), (sx * rw, r_c - 0.3, zf), (0.0, r_c - 0.3, zf + rh),
                          (0.0, -r_c + 0.3, zf + rh)], (sx * rh, 0.0, rw), floor_uv)
    for sy in (-1.0, 1.0):
        S.add(DEFLECTOR, [(-rw, sy * (r_c - 0.3), zf), (rw, sy * (r_c - 0.3), zf), (0.0, sy * (r_c - 0.3), zf + rh)],
              (0.0, sy, 0.0), floor_uv)
    # Hoods: angular grey housings on both sides of the deck, tower side; each with the roof sloping toward the tower.
    for sy in (-1.0, 1.0):
        y0, y1 = sy * m["hood_inner_y_m"], sy * m["hood_outer_y_m"]
        x0, x1 = m["hood_front_x_m"], m["hood_back_x_m"]
        z0 = zd
        hf, hb, ch = m["hood_height_front_m"], m["hood_height_back_m"], m["hood_chamfer_m"]
        S.solid(HOOD, [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
                       (x0, y0, z0 + hf), (x1, y0, z0 + hb), (x1, y1 - sy * ch, z0 + hb - 0.8),
                       (x0, y1 - sy * ch, z0 + hf - 0.8)])
    # Vent boxes on the tower-side face.
    for sy in (-1.0, 1.0):
        zc = zd - 5.0
        x_face = sh.hx(zc)
        S.box(HOOD, (x_face - 0.1, sy * m["vent_y_m"] - 0.5 * m["vent_width_m"], zc),
              (x_face + m["vent_depth_m"], sy * m["vent_y_m"] + 0.5 * m["vent_width_m"], zc + m["vent_height_m"]))
    # Booster QDs: static cabinets with a boom and a head at 4.9 m from the axis, on the front side of the deck.
    r_head = m["qd_head_radius_m"]
    for sy in (-1.0, 1.0):
        cx, cy = m["qd_x_m"], sy * m["qd_y_m"]
        hx_, hy_ = 0.5 * m["qd_size_x_m"], 0.5 * m["qd_size_y_m"]
        S.box(HARDWARE, (cx - hx_, cy - hy_, zd), (cx + hx_, cy + hy_, zd + m["qd_size_z_m"]))
        zb = zd + m["qd_boom_z_m"]
        half = 0.5 * m["qd_boom_size_m"]
        x_h = -math.sqrt(max(r_head ** 2 - (abs(cy) - 2.0 * half) ** 2, 0.0))  # the head's corner nearest the axis
        S.box(HARDWARE, (cx + hx_, cy - half, zb - half), (x_h - 0.2, cy + half, zb + half))
        S.box(HARDWARE, (x_h - 0.3, cy - 2.0 * half, zb - 2.0 * half), (x_h, cy + 2.0 * half, zb + 2.0 * half))
    # Door arches (dark decals 0.04 m proud of the front face) and the diagonal stair strip.
    zc = zp + 0.4
    for sy in (-1.0, 1.0):
        yc, dw, dh = sy * m["door_y_m"], 0.5 * m["door_width_m"], m["door_height_m"]
        arch = [(yc - dw, zc), (yc + dw, zc)] + [(yc + dw * math.cos(math.pi * k / 8.0),
                                                  zc + dh - dw + dw * math.sin(math.pi * k / 8.0)) for k in range(9)]
        pts = []
        for y, z in arch:
            pts.append((-(sh.hx(z) + 0.04), y, z))
        S.add(DOOR, pts, (-1.0, 0.0, 0.4), lambda p: (p[1], p[2]))
    y_a, y_b = m["stair_from_y_m"], m["stair_to_y_m"]
    z_a = m["stair_from_z_m"]
    z_b = z_a + (y_b - y_a) * 0.9  # the strip climbs diagonally across the face
    lift, wd = m["stair_lift_m"], 0.5 * m["stair_width_m"]
    p_a, p_b = (y_a, z_a), (y_b, min(z_b, zd - 0.6))
    dy, dz = p_b[0] - p_a[0], p_b[1] - p_a[1]
    ln = math.hypot(dy, dz)
    ny_, nz_ = -dz / ln, dy / ln  # in-face normal of the strip's line
    corners = []
    for (y, z), s in ((p_a, -1.0), (p_a, 1.0), (p_b, 1.0), (p_b, -1.0)):
        yy, zz = y + s * wd * ny_, z + s * wd * nz_
        corners.append((-(sh.hx(zz) + lift), yy, zz))
    S.add(STAIR, corners, (-1.0, 0.0, 0.4), lambda p: (p[1], p[2]))
    return S


def trench(detail: dict) -> Surfaces:
    """The two exit ramps (floor rising to the ground, side walls) and the deluge wall; same frame as the mount."""
    sh = Shape(detail)
    t = sh.t
    S = Surfaces()
    zf = sh.floor_z
    y0, y1 = sh.half_a, sh.half_a + t["length_m"]
    hw = 0.5 * t["width_m"]
    lip = t["wall_height_m"]
    up = (0.0, 0.0, 1.0)
    for sy in (-1.0, 1.0):
        S.add("concrete", [(-hw, sy * y0, zf), (hw, sy * y0, zf), (hw, sy * y1, 0.0), (-hw, sy * y1, 0.0)], up,
              lambda p: (p[0], p[1]))
        for side in (-1.0, 1.0):
            x = side * hw
            S.add("concrete", [(x, sy * y0, zf), (x, sy * y1, 0.0), (x, sy * y1, lip), (x, sy * y0, lip)],
                  (-side, 0.0, 0.0), lambda p: (p[1], p[2]))
    wall_t = t["deluge_thickness_m"]
    wx = t["deluge_x_m"]
    ya = -t["deluge_start_y_m"]
    S.box("wall", (wx - 0.5 * wall_t, ya - t["deluge_length_m"], 0.0), (wx + 0.5 * wall_t, ya, t["deluge_height_m"]),
          lambda p: (p[1], p[2]))
    return S


def ramp_end_m(detail: dict) -> float:
    sh = Shape(detail)
    return sh.half_a + sh.t["length_m"]


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
