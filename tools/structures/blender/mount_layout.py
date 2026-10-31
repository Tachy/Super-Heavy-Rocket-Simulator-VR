"""Pure layout of the Pad B launch mount, its trench and hardstand: polygons per glTF node, no Blender.

gen_mount.py / gen_trench.py add the polygons to meshes; pytest reads the same lists (shape, clearances).
Frame (both models): X toward the tower, Y across, Z up, origin on the mount axis at the ground (the site height).
A surface is (verts, uvs): planar polygon, counter-clockwise seen from outside (the winding is fixed by an outward hint
in `Surfaces.add`, so the glTF normals point out). UV in metres on the surface; the shell shader repeats the panel map.

Shape (launch-structures-measurements.md "Mount shape", images.jpeg, padb_mount_pyramid_front_person_scale_2026-05,
padb_mount_side_pyramid_dusk_2025-09, padb_aerial_mount_trench_topdown_2025-09): four sloped corner piers and a deck.
The +-X faces (front -X with two grey doors, vents and the two hood ducts; tower side +X) are closed plated slopes.
Each +-Y side is one big portal: the piers' faces flank a trapezoid opening whose sides follow the piers, a vertical
spandrel with a heavy lintel and a ribbed duct above it, and inside the bucket: a ribbed flame deflector curving down
from a ridge under the ring to the pit floor, which runs out into the exit ramp.
The vertical scale is the physics deck height (deck_height_m) over the 14.5 m measured: only the slopes' steepness and
the opening's height differ from the photos, the plan sizes are the measured ones.
"""
from __future__ import annotations

import math

SHELL, PLINTH, PIER, INTERIOR, DEFLECTOR, HOOD, HARDWARE, DOOR = (
    "shell", "plinth", "pier", "interior", "deflector", "hood", "hardware", "door")
CIRCLE_SEGMENTS = 32  # [E] ring, chamber: the opening reads round at the 9 m size
DEFLECTOR_SEGMENTS = 14  # [E] per side: the curve of the bucket floor reads smooth
DUCT_SIDES = 10  # [E] the duct above the lintel reads round
DUCT_RIB_FINS = 18  # [E] rib rings on the duct, images padb_mount_side_pyramid_dusk_2025-09


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
        self.run = self.half_a - 0.5 * m["top_across_m"]
        self.top_half_a = 0.5 * m["top_across_m"]
        self.top_half_t = self.half_t - self.run
        self.slope_len = math.hypot(self.run, self.deck_z - self.plinth_z) / (self.deck_z - self.plinth_z)
        self.ring_r = 0.5 * m["ring_outer_diameter_m"]
        self.open_r = 0.5 * m["ring_opening_diameter_m"]
        self.ring_bottom = self.height - m["ring_thickness_m"]
        self.open_top = m["opening_top_m"]
        self.open_half_ground = 0.5 * m["opening_width_ground_m"]
        self.open_half_deck = 0.5 * m["opening_width_deck_m"]

    def _slope(self, z):
        return self.run * (z - self.plinth_z) / (self.deck_z - self.plinth_z)

    def hx(self, z):  # half extent along the mount-tower line at height z on the slope
        return self.half_t - self._slope(z)

    def hy(self, z):
        return self.half_a - self._slope(z)

    def xi(self, z):  # half width of the portal opening at height z: its sides follow the piers
        return self.open_half_ground + (self.open_half_deck - self.open_half_ground) * z / self.deck_z


def _circle(r, z, angles):
    return [(r * math.cos(a), r * math.sin(a), z) for a in angles]


def _rect_point(a, hx, hy, z):
    """Where the ray at angle a leaves the rectangle |x| <= hx, |y| <= hy."""
    c, s = math.cos(a), math.sin(a)
    t = min(hx / abs(c) if abs(c) > 1e-12 else 1e9, hy / abs(s) if abs(s) > 1e-12 else 1e9)
    return (t * c, t * s, z)


def _uv_x(p):  # walls in a plane of constant Y: u along X, v up
    return (p[0], p[2])


def _uv_y(p):  # walls in a plane of constant X: u along Y, v up
    return (p[1], p[2])


def _uv_plan(p):
    return (p[0], p[1])


def _annulus_to_rect(S, node, radius, hx, hy, z, hint):
    """Flat ring between a circle and the rectangle around it, as a fan of quads (corner angles included)."""
    base = [2.0 * math.pi * k / CIRCLE_SEGMENTS for k in range(CIRCLE_SEGMENTS)]
    corner = math.atan2(hy, hx)
    angles = sorted(base + [corner, math.pi - corner, math.pi + corner, -corner + 2.0 * math.pi])
    for i, a in enumerate(angles):
        b = angles[(i + 1) % len(angles)] + (2.0 * math.pi if i == len(angles) - 1 else 0.0)
        S.add(node, [_circle(radius, z, [a])[0], _circle(radius, z, [b])[0], _rect_point(b, hx, hy, z),
                     _rect_point(a, hx, hy, z)], hint, _uv_plan)


def _closed_slopes(S, sh):
    """The plated +-X faces and the low foot band all round the base."""
    zp, zd, zf = sh.plinth_z, sh.deck_z, sh.floor_z
    for sx in (-1.0, 1.0):
        S.add(SHELL, [(sx * sh.hx(zp), -sh.hy(zp), zp), (sx * sh.hx(zp), sh.hy(zp), zp),
                      (sx * sh.hx(zd), sh.hy(zd), zd), (sx * sh.hx(zd), -sh.hy(zd), zd)], (sx, 0.0, 0.4),
              lambda p: (p[1], p[2] * sh.slope_len))
        S.add(PLINTH, [(sx * sh.half_t, -sh.half_a, zf), (sx * sh.half_t, sh.half_a, zf),
                       (sx * sh.half_t, sh.half_a, zp), (sx * sh.half_t, -sh.half_a, zp)], (sx, 0.0, 0.0), _uv_y)


def _portal(S, sh):
    """Both +-Y sides: pier faces, foot band, spandrel, lintel, duct and the opening's side walls."""
    m = sh.m
    zp, zd, zf, zo = sh.plinth_z, sh.deck_z, sh.floor_z, sh.open_top
    yb = sh.hy(zd)  # the spandrel stands under the deck edge
    for sy in (-1.0, 1.0):
        for side in (-1.0, 1.0):
            # Pier face in the sloped plane between the corner edge and the opening.
            S.add(PIER, [(side * sh.hx(zp), sy * sh.hy(zp), zp), (side * sh.xi(zp), sy * sh.hy(zp), zp),
                         (side * sh.xi(zd), sy * sh.hy(zd), zd), (side * sh.hx(zd), sy * sh.hy(zd), zd)],
                  (0.0, sy, 0.4), lambda p: (p[0], p[2] * sh.slope_len))
            lo, hi = sorted((side * sh.open_half_ground, side * sh.half_t))
            S.add(PLINTH, [(lo, sy * sh.half_a, zf), (hi, sy * sh.half_a, zf), (hi, sy * sh.half_a, zp),
                           (lo, sy * sh.half_a, zp)], (0.0, sy, 0.0), _uv_x)
            # The opening's side wall: a wedge in the plane x = xi(z) from the sloped face back to the spandrel.
            S.add(INTERIOR, [(side * sh.xi(zp), sy * yb, zp), (side * sh.xi(zp), sy * sh.hy(zp), zp),
                             (side * sh.xi(zd), sy * yb, zd)], (-side, 0.0, 0.0), _uv_y)
        # Spandrel: flat plate between the piers, opening top to the deck edge.
        S.add(HOOD, [(-sh.xi(zo), sy * yb, zo), (sh.xi(zo), sy * yb, zo), (sh.xi(zd), sy * yb, zd),
                     (-sh.xi(zd), sy * yb, zd)], (0.0, sy, 0.0), _uv_x)
        # Heavy lintel along the opening's top, standing out of the spandrel.
        ld, lh = m["lintel_depth_m"], m["lintel_height_m"]
        hw = sh.xi(zo)
        S.box(PLINTH, (-hw, min(sy * yb, sy * (yb + ld)), zo), (hw, max(sy * yb, sy * (yb + ld)), zo + lh), _uv_x)
        # Ribbed duct along the deck edge above the spandrel: a prism with a rib ring every few metres.
        _duct(S, sh, sy * (yb + m["duct_radius_m"]), zd - m["duct_radius_m"], sh.xi(zd) + 0.5 * m["duct_radius_m"])


def _duct(S, sh, yc, zc, half_x):
    m = sh.m
    r = m["duct_radius_m"]
    a = [2.0 * math.pi * k / DUCT_SIDES for k in range(DUCT_SIDES + 1)]

    def rim(x, radius, k):
        return (x, yc + radius * math.cos(a[k]), zc + radius * math.sin(a[k]))

    for k in range(DUCT_SIDES):
        mid = 0.5 * (a[k] + a[k + 1])
        out = (0.0, math.cos(mid), math.sin(mid))
        S.add(PLINTH, [rim(-half_x, r, k), rim(half_x, r, k), rim(half_x, r, k + 1), rim(-half_x, r, k + 1)], out, _uv_y)
    fin = m["duct_rib_m"]
    for j in range(DUCT_RIB_FINS):
        x = -half_x + 2.0 * half_x * (j + 0.5) / DUCT_RIB_FINS
        for k in range(DUCT_SIDES):
            mid = 0.5 * (a[k] + a[k + 1])
            out = (0.0, math.cos(mid), math.sin(mid))
            S.add(PLINTH, [rim(x - fin, r + fin, k), rim(x + fin, r + fin, k), rim(x + fin, r + fin, k + 1),
                           rim(x - fin, r + fin, k + 1)], out, _uv_y)
            for sx in (-1.0, 1.0):
                S.add(PLINTH, [rim(x + sx * fin, r, k), rim(x + sx * fin, r + fin, k), rim(x + sx * fin, r + fin, k + 1),
                               rim(x + sx * fin, r, k + 1)], (sx, 0.0, 0.0), _uv_y)


def _bucket(S, sh):
    """Below the opening: side walls, ceiling with the chamber, the ribbed deflector with its ridge under the ring."""
    m = sh.m
    zf, zo, zd, zp = sh.floor_z, sh.open_top, sh.deck_z, sh.plinth_z
    xw = sh.open_half_ground
    yb = sh.hy(zd)
    for side in (-1.0, 1.0):
        x = side * xw
        S.add(INTERIOR, [(x, -sh.half_a, zf), (x, sh.half_a, zf), (x, sh.half_a, 0.0), (x, -sh.half_a, 0.0)],
              (-side, 0.0, 0.0), _uv_y)  # the pit's side walls
        S.add(INTERIOR, [(x, -sh.half_a, 0.0), (x, sh.half_a, 0.0), (side * sh.xi(zp), sh.hy(zp), zp),
                         (side * sh.xi(zp), -sh.hy(zp), zp)], (-side, 0.0, 0.0), _uv_y)
        # Above the foot band the wall leans with the opening's sides, up to the ceiling.
        S.add(INTERIOR, [(side * sh.xi(zp), -yb, zp), (side * sh.xi(zp), yb, zp), (side * sh.xi(zo), yb, zo),
                         (side * sh.xi(zo), -yb, zo)], (-side, 0.0, 0.0), _uv_y)
    _annulus_to_rect(S, INTERIOR, m["chamber_radius_m"], sh.xi(zo), yb, zo, (0.0, 0.0, -1.0))  # ceiling
    # Chamber wall from the ceiling to the deck's underside, and the plate between ring and chamber.
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
    _deflector(S, sh)


def _deflector_z(sh, y):
    """Height of the bucket floor at |y|: a parabola, steep under the ring, flat at the pit floor."""
    frac = max(0.0, 1.0 - abs(y) / sh.half_a)
    return sh.floor_z + (sh.m["ridge_top_m"] - sh.floor_z) * frac ** 2


def _deflector(S, sh):
    """Ribs run down the slope; each is a shallow V seen across, so the floor reads ribbed in the raking sun."""
    pitch, depth = sh.m["rib_pitch_m"], sh.m["rib_depth_m"]
    xw = sh.open_half_ground
    count = int(round(2.0 * xw / pitch))
    step = 2.0 * xw / count
    up = (0.0, 0.0, 1.0)
    for sy in (-1.0, 1.0):
        ys = [sy * sh.half_a * k / DEFLECTOR_SEGMENTS for k in range(DEFLECTOR_SEGMENTS + 1)]
        for y0, y1 in zip(ys, ys[1:]):
            z0, z1 = _deflector_z(sh, y0), _deflector_z(sh, y1)
            for i in range(count):
                xa, xm, xb = -xw + i * step, -xw + (i + 0.5) * step, -xw + (i + 1) * step
                S.add(DEFLECTOR, [(xa, y0, z0), (xm, y0, z0 + depth), (xm, y1, z1 + depth), (xa, y1, z1)], up, _uv_plan)
                S.add(DEFLECTOR, [(xm, y0, z0 + depth), (xb, y0, z0), (xb, y1, z1), (xm, y1, z1 + depth)], up, _uv_plan)


def _deck_and_ring(S, sh):
    m = sh.m
    zd = sh.deck_z
    up = (0.0, 0.0, 1.0)
    _annulus_to_rect(S, SHELL, sh.ring_r, sh.hx(zd), sh.hy(zd), zd, up)  # the flat deck, the ring stands in its middle
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


def _hoods(S, sh):
    """Two long bent ducts on the front side: root block at the ring, roof over the deck edge, box down the front face."""
    m = sh.m
    zd = sh.deck_z
    x_root, x_roof, x_front = m["hood_root_x_m"], m["hood_roof_x_m"], m["hood_front_x_m"]
    top, shoulder, drop = zd + m["hood_height_m"], zd + m["hood_shoulder_m"], zd - m["hood_drop_m"]
    x_foot = -sh.hx(drop)  # where the box's bottom meets the sloped face
    profile = [(x_root, zd), (x_root, top), (x_roof, top), (x_front, shoulder), (x_front, drop), (x_foot, drop)]
    for sy in (-1.0, 1.0):
        y0, y1 = sy * m["hood_inner_y_m"], sy * m["hood_outer_y_m"]
        for yy, hint in ((y0, -sy), (y1, sy)):
            S.add(HOOD, [(x, yy, z) for x, z in profile], (0.0, hint, 0.0), _uv_x)
        for i in range(len(profile) - 1):
            (xa, za), (xb, zb) = profile[i], profile[i + 1]
            out = (-(zb - za), 0.0, xb - xa)  # the profile runs clockwise seen from +Y: this is its outside
            if _dot(out, (xa + xb - 2.0 * x_root + 0.1, 0.0, za + zb - 2.0 * zd)) < 0.0:
                out = (-out[0], 0.0, -out[2])
            S.add(HOOD, [(xa, y0, za), (xb, y0, zb), (xb, y1, zb), (xa, y1, za)], out, _uv_y)
        # Tan root block on the ring side of the duct (padb_chopsticks_above_mount_2026-05).
        a, b = sorted((y0, y1))
        S.box(PLINTH, (x_root, a, zd), (x_root + m["hood_root_depth_m"], b, zd + m["hood_root_height_m"]), _uv_x)


def _front_details(S, sh):
    """Doors and vent boxes in the front (-X) face, person-scale photo, and the booster QDs on the deck."""
    m = sh.m
    zp, zd = sh.plinth_z, sh.deck_z
    for sy in (-1.0, 1.0):
        yc, half = sy * m["door_y_m"], 0.5 * m["door_width_m"]
        pts = [(yc - half, zp), (yc + half, zp), (yc + half, m["door_height_m"]), (yc - half, m["door_height_m"])]
        S.add(DOOR, [(-(sh.hx(z) + m["door_lift_m"]), y, z) for y, z in pts], (-1.0, 0.0, 0.4), _uv_y)
        for yv, zc, width in ((sy * m["door_y_m"], m["door_height_m"] + 0.3, m["vent_lower_width_m"]),
                              (sy * m["vent_y_m"], zd - m["vent_below_deck_m"], m["vent_width_m"]),
                              (sy * m["vent_outer_y_m"], zd - m["vent_outer_below_deck_m"], m["vent_width_m"])):
            hw = 0.5 * width
            ztop = zc + m["vent_height_m"]
            xb, xt = -sh.hx(zc), -sh.hx(ztop)
            xout = xt - m["vent_depth_m"]
            # A wedge standing off the sloped face: its top flat, its underside open-looking (dark louvres in the map).
            S.solid(HOOD, [(xb + 0.05, yv - hw, zc), (xout, yv - hw, zc), (xout, yv + hw, zc), (xb + 0.05, yv + hw, zc),
                           (xt + 0.05, yv - hw, ztop), (xout, yv - hw, ztop), (xout, yv + hw, ztop),
                           (xt + 0.05, yv + hw, ztop)])
    r_head = m["qd_head_radius_m"]
    for sy in (-1.0, 1.0):  # static cabinets with a boom and a head at 4.9 m from the axis, front side of the deck
        cx, cy = m["qd_x_m"], sy * m["qd_y_m"]
        hx_, hy_ = 0.5 * m["qd_size_x_m"], 0.5 * m["qd_size_y_m"]
        S.box(HARDWARE, (cx - hx_, cy - hy_, zd), (cx + hx_, cy + hy_, zd + m["qd_size_z_m"]))
        zb = zd + m["qd_boom_z_m"]
        half = 0.5 * m["qd_boom_size_m"]
        x_h = -math.sqrt(max(r_head ** 2 - (abs(cy) - 2.0 * half) ** 2, 0.0))  # the head's corner nearest the axis
        S.box(HARDWARE, (cx + hx_, cy - half, zb - half), (x_h - 0.2, cy + half, zb + half))
        S.box(HARDWARE, (x_h - 0.3, cy - 2.0 * half, zb - 2.0 * half), (x_h, cy + 2.0 * half, zb + 2.0 * half))


def mount(detail: dict) -> Surfaces:
    sh = Shape(detail)
    S = Surfaces()
    _closed_slopes(S, sh)
    _portal(S, sh)
    _bucket(S, sh)
    _deck_and_ring(S, sh)
    _hoods(S, sh)
    _front_details(S, sh)
    return S


def _x_range(poly, y):
    """x interval of the convex polygon `poly` at height y (plan), or None outside."""
    xs = []
    for i, a in enumerate(poly):
        b = poly[(i + 1) % len(poly)]
        if min(a[1], b[1]) - 1e-9 <= y <= max(a[1], b[1]) + 1e-9:
            if abs(b[1] - a[1]) < 1e-12:
                xs += [a[0], b[0]]
            else:
                xs.append(a[0] + (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]))
    return (min(xs), max(xs)) if xs else None


def _hardstand(S, sh):
    """The flat light concrete apron: the convex outline minus the mount's footprint and the ramps, cut in Y bands."""
    t = sh.t
    poly = [tuple(float(c) for c in pair.split(",")) for pair in t["hardstand_outline_m"].split()]
    lift = t["hardstand_lift_m"]
    y_end = sh.half_a + t["length_m"]
    hw = 0.5 * t["width_m"]
    cuts = sorted({round(p[1], 6) for p in poly} | {-y_end, -sh.half_a, sh.half_a, y_end})
    up = (0.0, 0.0, 1.0)
    for y0, y1 in zip(cuts, cuts[1:]):
        r0, r1 = _x_range(poly, y0), _x_range(poly, y1)
        if r0 is None or r1 is None:
            continue
        mid = abs(0.5 * (y0 + y1))
        hole = sh.half_t if mid < sh.half_a else hw if mid < y_end else None
        if hole is None:
            S.add("hardstand", [(r0[0], y0, lift), (r0[1], y0, lift), (r1[1], y1, lift), (r1[0], y1, lift)], up, _uv_plan)
            continue
        S.add("hardstand", [(r0[0], y0, lift), (-hole, y0, lift), (-hole, y1, lift), (r1[0], y1, lift)], up, _uv_plan)
        S.add("hardstand", [(hole, y0, lift), (r0[1], y0, lift), (r1[1], y1, lift), (hole, y1, lift)], up, _uv_plan)


def trench(detail: dict) -> Surfaces:
    """The two flat exit ramps with their low side walls, the hardstand and the tower's foot block; frame as the mount."""
    sh = Shape(detail)
    t = sh.t
    S = Surfaces()
    zf = sh.floor_z
    y0, y1 = sh.half_a, sh.half_a + t["length_m"]
    hw = 0.5 * t["width_m"]
    lip, wt = t["wall_height_m"], t["wall_thickness_m"]
    up = (0.0, 0.0, 1.0)
    for sy in (-1.0, 1.0):
        S.add("concrete", [(-hw, sy * y0, zf), (hw, sy * y0, zf), (hw, sy * y1, 0.0), (-hw, sy * y1, 0.0)], up, _uv_plan)
        for side in (-1.0, 1.0):
            x = side * hw
            S.add("wall", [(x, sy * y0, zf), (x, sy * y1, 0.0), (x, sy * y1, lip), (x, sy * y0, lip)],
                  (-side, 0.0, 0.0), _uv_y)
            xo = side * (hw + wt)
            S.add("wall", [(x, sy * y0, lip), (x, sy * y1, lip), (xo, sy * y1, lip), (xo, sy * y0, lip)], up, _uv_plan)
            S.add("wall", [(xo, sy * y0, 0.0), (xo, sy * y1, 0.0), (xo, sy * y1, lip), (xo, sy * y0, lip)],
                  (side, 0.0, 0.0), _uv_y)
            S.add("wall", [(x, sy * y1, 0.0), (xo, sy * y1, 0.0), (xo, sy * y1, lip), (x, sy * y1, lip)], (0.0, sy, 0.0), _uv_x)
    _hardstand(S, sh)
    # The tower's foot: a low block with a chamfered roof up to the legs' plinth.
    cx, half, top = t["foot_x_m"], 0.5 * t["foot_width_m"], t["foot_height_m"]
    roof, roof_half = t["foot_roof_height_m"], t["foot_roof_half_m"]
    S.box("foot", (cx - half, -half, 0.0), (cx + half, half, top), _uv_x)
    S.solid("foot", [(cx - half, -half, top), (cx + half, -half, top), (cx + half, half, top), (cx - half, half, top),
                     (cx - roof_half, -roof_half, roof), (cx + roof_half, -roof_half, roof),
                     (cx + roof_half, roof_half, roof), (cx - roof_half, roof_half, roof)], _uv_x)
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
