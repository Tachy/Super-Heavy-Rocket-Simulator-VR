"""Grid fin of the booster: the outline from the factory photo of a real fin (chamfered square, unequal chamfers), the cusped
saddle walls and the hinge details of the installed fin (user photos 2026-10-31).

Grid fin of the booster: gen_grid_fin.py -- --spec <json> --out <dir> writes grid_fin.gltf.

Frame (Blender axes kept): the fin of azimuth 0. The pivot (origin) is the hinge on the hull surface, the hinge axis is +Y
(radial outward). The plate lies in the Y-Z plane and its cells open along X: at deflection 0 the air of an axial flow
passes through them (core/src/aero/grid_fin.cpp: the lift normal is the tangent at d = 0, a right-handed rotation by +d
about the radial axis turns it toward +X). Span Y from root_gap_m, chord Z, depth X = +-t / 2 (+X = forward).
The outline is the measured polygon (detail file `outline_z_m` / `outline_y_m`, y from the root vertex). Walls run at +-45 deg
to the axes (diamond cells, pitch `cell_pitch_m`). Every wall is cut into saddles between two crossings: both free edges
(forward and aft face) dip towards the middle, the tips stand at the crossings as sharp cusps (what the earlier "spikes under
every crossing" were). The heart shapes seen through the cells against the sky are these two scalloped edges. A heavier cross
rib with its own wave runs across the whole fin. The rim follows the outline, flat light band on the forward face, scalloped
aft edge. The root corner carries a round bolted ring, a short tapered root block and a fan of three triangular struts.
Nodes: `grid_fin` (root) with `frame` (rim), `band` (flat rim face), `lattice` (walls, cross rib), `pylon` (ring, block,
struts), `bearing` (flat bar under the root, catch fins only), `plate` (far stand-in: one opaque polygon, the view swaps it in beyond the distance where the walls alias) and
`shaft` (the hinge shaft along +Y).
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402,F401
import vehicle_common as vc  # noqa: E402
from mathutils import Vector  # noqa: E402

SHAFT_SIDES = 12  # [E]
RING_SIDES = 20  # [E]
TOOTH_STEPS = 3  # [E] rim scallop segments per tooth
RIB_STEPS_PER_WAVE = 8  # [E]
STRUT_HALF_DEPTH_FRAC = 0.36  # [E] strut base half width / thickness_m
STRUT_END_Y_M = 1.1  # [E] strut tips end this far (span) from the hull: inside the lattice
STRUT_END_Z_M = 0.85  # [E] fan half spread at the strut tips
STRUT_THICK_M = 0.04  # [E]
BLOCK_HALF_Z_M = 0.25  # [E] root block width at the ring
BLOCK_HALF_X_M = 0.12  # [E] root block depth at the ring


def outline_points(d):
    """Measured polygon (z chord, y span), counter-clockwise, y shifted by the root gap."""
    return [(z, y + d["root_gap_m"]) for z, y in zip(d["outline_z_m"], d["outline_y_m"])]


def signed_area(pts):
    return 0.5 * sum(pts[i][0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * pts[i][1]
                     for i in range(len(pts)))


def offset_polygon(pts, dist):
    """Convex polygon moved inwards by `dist` (mitred corners)."""
    n = len(pts)
    lines = []
    for i in range(n):
        (z0, y0), (z1, y1) = pts[i], pts[(i + 1) % n]
        length = math.hypot(z1 - z0, y1 - y0)
        dz, dy = (z1 - z0) / length, (y1 - y0) / length
        lines.append(((z0 - dy * dist, y0 + dz * dist), (dz, dy)))  # left (inner) normal of a CCW polygon: (-dy, dz)
    out = []
    for i in range(n):
        (pz, py), (dz, dy) = lines[i - 1]
        (qz, qy), (ez, ey) = lines[i]
        den = dz * ey - dy * ez
        t = ((qz - pz) * ey - (qy - py) * ez) / den
        out.append((pz + dz * t, py + dy * t))
    return out


def inside(outline, z, y, margin):
    """True when (z, y) lies inside the convex outline by at least `margin`."""
    n = len(outline)
    for i in range(n):
        (z0, y0), (z1, y1) = outline[i], outline[(i + 1) % n]
        ez, ey = z1 - z0, y1 - y0
        if (ez * (y - y0) - ey * (z - z0)) / math.hypot(ez, ey) < margin:
            return False
    return True


def clip_line(outline, pz, py, dz, dy):
    """Parameter range of the line (pz, py) + s (dz, dy) inside the convex outline, or None."""
    lo, hi = -1.0e9, 1.0e9
    n = len(outline)
    for i in range(n):
        (z0, y0), (z1, y1) = outline[i], outline[(i + 1) % n]
        nz, ny = -(y1 - y0), z1 - z0
        den = nz * dz + ny * dy
        dist = nz * (pz - z0) + ny * (py - y0)
        if abs(den) < 1e-9:
            if dist < 0.0:
                return None
        elif den > 0.0:
            lo = max(lo, -dist / den)
        else:
            hi = min(hi, -dist / den)
    return (lo, hi) if hi > lo else None


def horn(u):
    """Cusp profile 0 at the tip (u = 0, 1) to 1 in the middle: vertical tangent at the tips (sharp horns)."""
    return math.sqrt(max(0.0, math.sin(math.pi * u)))


def _rim_side(frame, a, b, centre, d, x_top, x_aft):
    """One side of the rim: outer + inner skin; the aft edge is a row of horns (scallops between tips)."""
    (z0, y0), (z1, y1) = a, b
    length = math.hypot(z1 - z0, y1 - y0)
    teeth = max(1, round(length / d["edge_tooth_pitch_m"]))
    steps = teeth * TOOTH_STEPS
    out = Vector((0.0, (y0 + y1) / 2.0 - centre[1], (z0 + z1) / 2.0 - centre[0])).normalized()
    inner = -out * d["frame_m"]

    def at(i, x):
        t = i / steps
        return Vector((x, y0 + (y1 - y0) * t, z0 + (z1 - z0) * t))

    def edge(i):
        u = (i % TOOTH_STEPS) / TOOTH_STEPS
        return at(i, x_aft + d["edge_tooth_depth_m"] * horn(u))

    for i in range(steps):
        e0, e1 = edge(i), edge(i + 1)
        top0, top1 = at(i, x_top), at(i + 1, x_top)
        uv = [(0.0, 0.0)] * 4
        frame.face([e0, e1, top1, top0], uv, out)
        frame.face([e0 + inner, e1 + inner, top1 + inner, top0 + inner], uv, -out)
        frame.face([e0, e1, e1 + inner, e0 + inner], uv, Vector((-1.0, 0.0, 0.0)))


def _saddle_wall(mesh, p0, p1, s0, s1, d, x_top, x_aft):
    """Wall between points p0 (at along-coordinate s0) and p1 (s1), cusped on both faces; double-sided."""
    pitch, dip = d["cell_pitch_m"], d["cusp_dip_m"]
    (z0, y0), (z1, y1) = p0, p1
    along = Vector((0.0, y1 - y0, z1 - z0)).normalized()
    side = Vector((1.0, 0.0, 0.0)).cross(along)
    steps = d["cusp_steps"]
    pieces = max(1, math.ceil((s1 - s0) / pitch * steps - 1e-6))
    pts = []
    for j in range(pieces + 1):
        t = j / pieces
        s = s0 + (s1 - s0) * t
        u = s / pitch - math.floor(s / pitch + 1e-9)
        if abs(u - 1.0) < 1e-6:
            u = 0.0
        h = dip * horn(u)
        pts.append((Vector((0.0, y0 + (y1 - y0) * t, z0 + (z1 - z0) * t)), x_top - h, x_aft + h))
    for j in range(pieces):
        (c0, t0, a0), (c1, t1, a1) = pts[j], pts[j + 1]
        quad = [c0 + Vector((a0, 0, 0)), c1 + Vector((a1, 0, 0)), c1 + Vector((t1, 0, 0)), c0 + Vector((t0, 0, 0))]
        mesh.face(quad, [(0.0, 0.0)] * 4, side)
        mesh.face(quad, [(0.0, 0.0)] * 4, -side)


def _lattice(lat, outline, d, x_top, x_aft):
    pitch = d["cell_pitch_m"]
    cz = sum(p[0] for p in outline) / len(outline)
    cy = sum(p[1] for p in outline) / len(outline)
    inner = offset_polygon(outline, d["frame_m"])
    s2 = math.sqrt(0.5)
    reach = int(max(max(p[0] for p in outline) - min(p[0] for p in outline),
                    max(p[1] for p in outline) - min(p[1] for p in outline)) / pitch) + 2
    for sign in (-1.0, 1.0):  # the two wall families, +-45 deg
        dz, dy = sign * s2, s2
        across_z, across_y = s2, -sign * s2
        for k in range(-reach, reach + 1):
            pz, py = cz + k * pitch * across_z, cy + k * pitch * across_y
            span = clip_line(inner, pz, py, dz, dy)
            if span is None or span[1] - span[0] < 0.02:
                continue
            first, last = math.floor(span[0] / pitch + 1e-9), math.ceil(span[1] / pitch - 1e-9)
            for m in range(first, last):  # saddle from crossing m to crossing m + 1, cut at the rim
                sa, sb = max(span[0], m * pitch), min(span[1], (m + 1) * pitch)
                if sb - sa < 0.02:
                    continue
                _saddle_wall(lat, (pz + dz * sa, py + dy * sa), (pz + dz * sb, py + dy * sb), sa, sb, d, x_top, x_aft)
    # heavier cross rib across the whole fin with its own wave (person photo: just above the root box)
    y_rib = d["root_gap_m"] + d["rib_y_m"]
    span = clip_line(inner, 0.0, y_rib, 1.0, 0.0)
    half_t, wave = d["rib_thickness_m"] / 2.0, d["rib_wave_m"]
    period = pitch * math.sqrt(2.0)
    n = max(2, round((span[1] - span[0]) / period * RIB_STEPS_PER_WAVE))
    cols = []
    for j in range(n + 1):
        z = span[0] + (span[1] - span[0]) * j / n
        h = wave * 0.5 * (1.0 - math.cos(2.0 * math.pi * z / period))
        cols.append((z, x_top - h, x_aft + h))
    for j in range(n):
        (za, ta, aa), (zb, tb, ab) = cols[j], cols[j + 1]
        for sy, hint in ((-half_t, Vector((0, -1, 0))), (half_t, Vector((0, 1, 0)))):
            lat.face([Vector((aa, y_rib + sy, za)), Vector((ab, y_rib + sy, zb)), Vector((tb, y_rib + sy, zb)),
                      Vector((ta, y_rib + sy, za))], [(0.0, 0.0)] * 4, hint)
        lat.face([Vector((ta, y_rib - half_t, za)), Vector((tb, y_rib - half_t, zb)), Vector((tb, y_rib + half_t, zb)),
                  Vector((ta, y_rib + half_t, za))], [(0.0, 0.0)] * 4, Vector((1, 0, 0)))
        lat.face([Vector((aa, y_rib - half_t, za)), Vector((ab, y_rib - half_t, zb)), Vector((ab, y_rib + half_t, zb)),
                  Vector((aa, y_rib + half_t, za))], [(0.0, 0.0)] * 4, Vector((-1, 0, 0)))


def _pylon(mesh, outline, d, x_top):
    """Round bolted ring on the hinge axis, a tapered root block up to the root chamfer, a fan of triangular struts."""
    t = d["thickness_m"]
    y_top = d["ring_length_m"]
    vc.tube(mesh, (0.0, 0.0, 0.0), (0.0, y_top, 0.0), d["hinge_ring_radius_m"], RING_SIDES, "ring")
    ex, ey, ez = Vector((1, 0, 0)), Vector((0, 1, 0)), Vector((0, 0, 1))
    bolt = 0.022
    for i in range(int(d["ring_bolts"])):
        a = 2.0 * math.pi * i / d["ring_bolts"]
        r = d["hinge_ring_radius_m"] * 0.8
        vc.box(mesh, (math.cos(a) * r, y_top + bolt, math.sin(a) * r), ex, ey, ez, (bolt, bolt, bolt))
    (zb, yb), (zc, yc) = outline[1], outline[2]  # the root chamfer
    inset = 0.04

    def chamfer_y(z):
        return yb + (yc - yb) * (z - zb) / (zc - zb)

    za, zz = zb + inset, zc - inset
    bottom = [Vector((-BLOCK_HALF_X_M, y_top, -BLOCK_HALF_Z_M)), Vector((BLOCK_HALF_X_M, y_top, -BLOCK_HALF_Z_M)),
              Vector((BLOCK_HALF_X_M, y_top, BLOCK_HALF_Z_M)), Vector((-BLOCK_HALF_X_M, y_top, BLOCK_HALF_Z_M))]
    top = [Vector((-0.4 * t, chamfer_y(za), za)), Vector((0.4 * t, chamfer_y(za), za)),
           Vector((0.4 * t, chamfer_y(zz), zz)), Vector((-0.4 * t, chamfer_y(zz), zz))]
    centre = sum(bottom + top, Vector((0, 0, 0))) / 8.0
    for i in range(4):
        j = (i + 1) % 4
        quad = [bottom[i], bottom[j], top[j], top[i]]
        mesh.face(quad, [(0.0, 0.0)] * 4, sum(quad, Vector((0, 0, 0))) / 4.0 - centre)
    apex = Vector((0.0, y_top, 0.0))
    count = int(d["strut_count"])
    for i in range(count):
        zt = STRUT_END_Z_M * (2.0 * i / (count - 1) - 1.0) if count > 1 else 0.0
        tip = Vector((0.0, d["root_gap_m"] + STRUT_END_Y_M, zt))
        along = (tip - apex).normalized()
        perp = Vector((1.0, 0.0, 0.0)).cross(along) * (STRUT_THICK_M / 2.0)
        w = ex * (STRUT_HALF_DEPTH_FRAC * t)
        for sgn in (-1.0, 1.0):
            mesh.face([apex - w + perp * sgn, apex + w + perp * sgn, tip + perp * sgn], [(0.0, 0.0)] * 3, perp * sgn)


def build_fin(spec):
    d = spec["detail"]["grid_fin"]
    t = d["thickness_m"]
    outline = outline_points(d)
    assert signed_area(outline) > 0.0, "outline must be counter-clockwise in (z, y)"
    cz = sum(p[0] for p in outline) / len(outline)
    cy = sum(p[1] for p in outline) / len(outline)
    x_top, x_aft = t / 2.0, -t / 2.0
    n = len(outline)
    frame = vc.VMesh()
    for i in range(n):
        _rim_side(frame, outline[i], outline[(i + 1) % n], (cz, cy), d, x_top, x_aft)

    band = vc.VMesh()  # light flat face of the rim on the forward side, wider than the rim plate
    inner = offset_polygon(outline, d["band_m"])
    for i in range(n):
        j = (i + 1) % n
        band.face([Vector((x_top, outline[i][1], outline[i][0])), Vector((x_top, outline[j][1], outline[j][0])),
                   Vector((x_top, inner[j][1], inner[j][0])), Vector((x_top, inner[i][1], inner[i][0]))],
                  [(0.0, 0.0)] * 4, Vector((1.0, 0.0, 0.0)))

    lat = vc.VMesh()
    _lattice(lat, outline, d, x_top, x_aft)

    pylon = vc.VMesh()
    _pylon(pylon, outline, d, x_top)

    bearing = vc.VMesh()  # flat bar under the root, only on the catch fins (the view skips it on the others)
    slant = math.radians(d["bearing_slant_deg"])
    along = Vector((0.0, math.sin(slant), math.cos(slant)))
    across = Vector((0.0, -math.cos(slant), math.sin(slant)))
    vc.box(bearing, (x_aft - 0.05, d["root_gap_m"] + 0.05, 0.12), Vector((1.0, 0.0, 0.0)), along, across,
           (d["bearing_thickness_m"] / 2.0, d["bearing_length_m"] / 2.0, d["bearing_width_m"] / 2.0))

    plate = vc.VMesh()  # far stand-in: opaque polygon on the aft side of the walls' valleys
    plate.face([Vector((x_aft + d["cusp_dip_m"], y, z)) for z, y in outline], [(0.0, 0.0)] * n, Vector((-1.0, 0.0, 0.0)))
    plate.face([Vector((x_top, y, z)) for z, y in outline], [(0.0, 0.0)] * n, Vector((1.0, 0.0, 0.0)))

    shaft = vc.VMesh()
    vc.tube(shaft, (0.0, -d["shaft_inset_m"], 0.0), (0.0, d["root_gap_m"] + 0.3, 0.0), d["shaft_radius_m"], SHAFT_SIDES,
            "shaft")
    root = vc.empty("grid_fin")
    tris = {"frame": frame.to_object("frame", root)[1], "band": band.to_object("band", root)[1],
            "lattice": lat.to_object("lattice", root)[1], "pylon": pylon.to_object("pylon", root)[1],
            "bearing": bearing.to_object("bearing", root)[1],
            "plate": plate.to_object("plate", root)[1], "shaft": shaft.to_object("shaft", root)[1]}
    return root, tris


def main():
    args = vc.common.script_args()
    spec = vc.load_spec(args["spec"])
    vc.common.clear_scene()
    root, tris = build_fin(spec)
    vc.export_selected(Path(args["out"]) / "grid_fin.gltf", [root])
    for name, count in tris.items():
        print(f"fin part {name} {count}")
    vc.report_triangles("grid_fin", sum(tris.values()))


if __name__ == "__main__":
    main()
