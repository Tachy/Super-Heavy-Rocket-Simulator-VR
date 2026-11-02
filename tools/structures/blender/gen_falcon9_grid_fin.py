"""Falcon 9 grid fin: gen_falcon9_grid_fin.py -- --spec <json> --out <dir> writes falcon9_grid_fin.gltf.

Same frame and node meaning as the Starship's gen_grid_fin.py, so the view poses both alike: the fin of azimuth 0, pivot
(origin) on the hull, hinge axis +Y radial, plate in the Y-Z plane with cells open along X. A plain chamfered rectangle
(photos: F_Falcon_9_first_stage_in_hangar, Starlink 15-25 file 796 s), walls at +-45 deg. Nodes: `frame` (rim), `lattice`,
`plate` (far stand-in), `pylon` (hinge housing).
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402,F401
import vehicle_common as vc  # noqa: E402
from mathutils import Vector  # noqa: E402

AXIS_X = Vector((1.0, 0.0, 0.0))
PIVOT_SIDES = 12  # [E]


def outline(f):
    """Convex polygon (z chord, y span), counter-clockwise, root corners chamfered."""
    w, c, g, s = f["chord_m"] / 2.0, f["root_chamfer_m"], f["root_gap_m"], f["span_m"]
    return [(-w + c, g), (w - c, g), (w, g + c), (w, g + s), (-w, g + s), (-w, g + c)]


def clip_line(poly, p, d):
    """Part of the line p + t d inside the convex polygon, or None."""
    t0, t1 = -math.inf, math.inf
    for i, a in enumerate(poly):
        b = poly[(i + 1) % len(poly)]
        nx, ny = -(b[1] - a[1]), b[0] - a[0]  # inward normal of a counter-clockwise edge
        denom = nx * d[0] + ny * d[1]
        dist = nx * (p[0] - a[0]) + ny * (p[1] - a[1])
        if abs(denom) < 1e-12:
            if dist < 0.0:
                return None
            continue
        t = -dist / denom
        if denom > 0.0:
            t0 = max(t0, t)
        else:
            t1 = min(t1, t)
    return (t0, t1) if t1 > t0 else None


def bar(mesh, a, b, across, depth):
    """A wall from (z, y) point a to b, `across` thick in the plate, `depth` along X."""
    pa, pb = Vector((0.0, a[1], a[0])), Vector((0.0, b[1], b[0]))
    along = (pb - pa).normalized()
    side = AXIS_X.cross(along)
    vc.box(mesh, (pa + pb) / 2.0, AXIS_X, along, side, (depth / 2.0, (pb - pa).length / 2.0, across / 2.0))


def main():
    args = vc.common.script_args()
    f = vc.load_spec(args["spec"])["detail"]["grid_fin"]
    vc.common.clear_scene()
    root = vc.empty("falcon9_grid_fin")
    poly = outline(f)
    frame, lattice, plate, pylon = vc.VMesh(), vc.VMesh(), vc.VMesh(), vc.VMesh()
    for i, a in enumerate(poly):
        bar(frame, a, poly[(i + 1) % len(poly)], f["rim_m"], f["depth_m"])
    pitch, reach = f["cell_pitch_m"], f["chord_m"] + f["span_m"]
    root2 = math.sqrt(0.5)
    for sign in (1.0, -1.0):
        d = (root2 * sign, root2)
        normal = (root2, -root2 * sign)  # perpendicular to the wall: walls `pitch` apart, one through the plate centre
        steps = int(reach / pitch) + 1
        centre = (0.0, f["root_gap_m"] + f["span_m"] / 2.0)
        for k in range(-steps, steps + 1):
            p = (centre[0] + k * pitch * normal[0], centre[1] + k * pitch * normal[1])
            span = clip_line(poly, p, d)
            if span is None:
                continue
            a = (p[0] + span[0] * d[0], p[1] + span[0] * d[1])
            b = (p[0] + span[1] * d[0], p[1] + span[1] * d[1])
            bar(lattice, a, b, f["wall_m"], f["depth_m"])
    pts = [Vector((0.0, y, z)) for z, y in poly]
    plate.face(pts, [(z, 1.0 - y) for z, y in poly], (1.0, 0.0, 0.0))
    plate.face(pts, [(z, 1.0 - y) for z, y in poly], (-1.0, 0.0, 0.0))
    vc.tube(pylon, (0.0, 0.0, 0.0), (0.0, f["pivot_length_m"], 0.0), f["pivot_radius_m"], PIVOT_SIDES, "pylon")
    tris = 0
    for mesh, name in ((frame, "frame"), (lattice, "lattice"), (plate, "plate"), (pylon, "pylon")):
        tris += mesh.to_object(name, root)[1]
    vc.export_selected(Path(args["out"]) / "falcon9_grid_fin.gltf", [root])
    vc.report_triangles("falcon9_grid_fin", tris)


if __name__ == "__main__":
    main()
