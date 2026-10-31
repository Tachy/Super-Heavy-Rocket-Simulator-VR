"""Grid fin of the booster, drawn from real frames (f12 file 1804 s from below, Pad B aerial 2026-04 from above, NSF photo
2026-03 from the side; the 1:100 model gives the across-flats size only). The pivot stays on the hinge.

Grid fin of the booster: gen_grid_fin.py -- --spec <json> --out <dir> writes grid_fin.gltf.

Frame (Blender axes kept): the fin of azimuth 0. The pivot (origin) is the hinge on the hull surface, the hinge axis is +Y
(radial outward). The plate lies in the Y-Z plane and its cells open along X: at deflection 0 the air of an axial flow
passes through them (core/src/aero/grid_fin.cpp: the lift normal is the tangent at d = 0, a right-handed rotation by +d
about the radial axis turns it toward +X). Span Y from root_gap_m, chord Z = +-across_flats / 2, depth X = +-t / 2 (+X =
forward). Regular octagon hung on a CORNER at the root. Walls run at +-45 deg to the span (diamond cells); under every
wall crossing a four-sided spike points aft (-X); the rim plate's aft edge is toothed. Nodes: `grid_fin` (root) with
`frame` (rim), `lattice` (walls + spikes), `plate` (the far stand-in: one opaque octagon, the view swaps it in beyond the
distance where the walls alias) and `shaft` (the hinge shaft along +Y).
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402,F401
import vehicle_common as vc  # noqa: E402
from mathutils import Vector  # noqa: E402

SHAFT_SIDES = 12  # [E]
OCTAGON_SIDES = 8


def fin_outline(d):
    """Octagon vertices (z chord, y span), counter-clockwise, first vertex at the root on the radial axis."""
    radius = d["across_flats_m"] / 2.0 / math.cos(math.pi / OCTAGON_SIDES)
    centre = d["root_gap_m"] + radius
    return [(radius * math.sin(2.0 * math.pi * i / OCTAGON_SIDES),
             centre - radius * math.cos(2.0 * math.pi * i / OCTAGON_SIDES)) for i in range(OCTAGON_SIDES)]


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


def _rim_plate(mesh, a, b, centre, d, x_top, x_aft):
    """One side of the rim: outer + inner skin and top edge; the aft edge is a triangle wave (teeth pointing aft)."""
    (z0, y0), (z1, y1) = a, b
    length = math.hypot(z1 - z0, y1 - y0)
    steps = max(1, round(length / (d["edge_tooth_pitch_m"] / 2.0)))
    out = Vector((0.0, (y0 + y1) / 2.0 - centre[1], (z0 + z1) / 2.0 - centre[0])).normalized()
    inner = -out * d["frame_m"]

    def edge(i):
        t = i / steps
        tip = i % 2 == 0
        return Vector((x_aft if tip else x_aft + d["edge_tooth_depth_m"], y0 + (y1 - y0) * t, z0 + (z1 - z0) * t))

    for i in range(steps):
        t0, t1 = i / steps, (i + 1) / steps
        top0 = Vector((x_top, y0 + (y1 - y0) * t0, z0 + (z1 - z0) * t0))
        top1 = Vector((x_top, y0 + (y1 - y0) * t1, z0 + (z1 - z0) * t1))
        e0, e1 = edge(i), edge(i + 1)
        uv = [(0.0, 0.0)] * 4
        mesh.face([e0, e1, top1, top0], uv, out)
        mesh.face([e0 + inner, e1 + inner, top1 + inner, top0 + inner], uv, -out)
        mesh.face([e0, e1, e1 + inner, e0 + inner], uv, Vector((-1.0, 0.0, 0.0)))
        mesh.face([top0, top1, top1 + inner, top0 + inner], uv, Vector((1.0, 0.0, 0.0)))


def build_fin(spec):
    d = spec["detail"]["grid_fin"]
    t = d["thickness_m"]
    outline = fin_outline(d)
    cz = sum(p[0] for p in outline) / len(outline)
    cy = sum(p[1] for p in outline) / len(outline)
    x_top, x_aft = t / 2.0, -t / 2.0
    frame = vc.VMesh()
    n = len(outline)
    for i in range(n):
        _rim_plate(frame, outline[i], outline[(i + 1) % n], (cz, cy), d, x_top, x_aft)

    lat = vc.VMesh()
    pitch, wall = d["cell_pitch_m"], d["wall_m"]
    x_wall_aft = x_aft + d["spike_length_m"]
    mid_x, half_x = (x_top + x_wall_aft) / 2.0, (x_top - x_wall_aft) / 2.0
    s2 = math.sqrt(0.5)
    reach = int(d["across_flats_m"] / pitch) + 2
    ey = Vector((0.0, 1.0, 0.0))
    ez = Vector((0.0, 0.0, 1.0))
    ex = Vector((1.0, 0.0, 0.0))
    for sign in (-1.0, 1.0):  # the two wall families, +-45 deg to the span
        along = ey * s2 + ez * (sign * s2)
        across = ex.cross(along).normalized()
        dz, dy = sign * s2, s2
        for k in range(-reach, reach + 1):
            pz = cz + k * pitch * across.z  # walls through the octagon's centre lattice
            py = cy + k * pitch * across.y
            span = clip_line(outline, pz, py, dz, dy)
            if span is None:
                continue
            mid = 0.5 * (span[0] + span[1])
            centre = Vector((mid_x, py + dy * mid, pz + dz * mid))
            vc.box(lat, centre, ex, along, across, (half_x, 0.5 * (span[1] - span[0]), wall / 2.0))
    # spikes under the wall crossings (f12 file 1804 s): a four-sided pyramid pointing aft, base along the walls
    base = pitch * d["spike_base_frac"]
    for i in range(-reach, reach + 1):
        for j in range(-reach, reach + 1):
            z = cz + (i - j) * pitch * s2
            y = cy + (i + j) * pitch * s2
            if not inside(outline, z, y, d["frame_m"] * 0.5):
                continue
            tip = Vector((x_aft, y, z))
            corners = [Vector((x_wall_aft, y + base * s2 * a, z + base * s2 * b))
                       for a, b in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
            for k in range(4):
                c0, c1 = corners[k], corners[(k + 1) % 4]
                outward = (c0 + c1) / 2.0 - Vector((x_wall_aft, y, z)) - ex * 0.05
                lat.face([tip, c0, c1], [(0.0, 0.0)] * 3, outward)

    plate = vc.VMesh()  # far stand-in: opaque octagon on the aft side of the walls
    plate.face([Vector((x_wall_aft, y, z)) for z, y in outline], [(0.0, 0.0)] * n, Vector((-1.0, 0.0, 0.0)))
    plate.face([Vector((x_top, y, z)) for z, y in outline], [(0.0, 0.0)] * n, Vector((1.0, 0.0, 0.0)))

    shaft = vc.VMesh()
    vc.tube(shaft, (0.0, -d["shaft_inset_m"], 0.0), (0.0, d["root_gap_m"] + 0.3, 0.0), d["shaft_radius_m"], SHAFT_SIDES,
            "shaft")
    root = vc.empty("grid_fin")
    tris = {"frame": frame.to_object("frame", root)[1], "lattice": lat.to_object("lattice", root)[1],
            "plate": plate.to_object("plate", root)[1],
            "shaft": shaft.to_object("shaft", root)[1]}
    return root, tris


def main():
    args = vc.common.script_args()
    spec = vc.load_spec(args["spec"])
    vc.common.clear_scene()
    root, tris = build_fin(spec)
    vc.export_selected(Path(args["out"]) / "grid_fin.gltf", [root])
    vc.report_triangles("grid_fin", sum(tris.values()))


if __name__ == "__main__":
    main()
