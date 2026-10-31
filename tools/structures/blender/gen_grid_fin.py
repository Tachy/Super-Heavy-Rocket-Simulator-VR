"""Grid fin of the booster: gen_grid_fin.py -- --spec <json> --out <dir> writes grid_fin.gltf.

Frame (Blender axes kept): the fin of azimuth 0. The pivot (origin) is the hinge on the hull surface: the hinge axis is
+Y (radial outward, core/src/aero/grid_fin.cpp: the deflection d turns the lattice normal from the tangent toward +X,
that is a right-handed rotation by +d about the radial axis; zero = rest pose). The lattice lies in the X-Y plane: chord
X = +-c/2 around the hinge station (c = area / span), span Y = 0 .. span (span = 2 (radius_m - hull radius), so the
centre of area sits at the config's radius), thickness Z = +-t/2 (the cells open along Z). The game rotates it about
the stage's X axis to the fin's azimuth and puts the pivot at (station_m, radius R). Nodes: `grid_fin` (root) with
`lattice` and `shaft` (the hinge shaft along +Y, reaching shaft_inset_m into the hull).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402,F401
import vehicle_common as vc  # noqa: E402
from mathutils import Vector  # noqa: E402

SHAFT_SIDES = 12  # [E]
AX = (Vector((1.0, 0.0, 0.0)), Vector((0.0, 1.0, 0.0)), Vector((0.0, 0.0, 1.0)))


def fin_dims(spec):
    st = spec["stages"]["booster"]
    span = 2.0 * (st["grid_fins"]["radius_m"] - st["diameter_m"] / 2.0)
    return st["grid_fins"]["area_m2"] / span, span


def build_fin(spec):
    d = spec["detail"]["grid_fin"]
    chord, span = fin_dims(spec)
    t, wall, frame = d["thickness_m"], d["wall_m"], d["frame_m"]
    lat = vc.VMesh()

    def slab(x0, x1, y0, y1):
        vc.box(lat, ((x0 + x1) / 2, (y0 + y1) / 2, 0.0), *AX, ((x1 - x0) / 2, (y1 - y0) / 2, t / 2))

    hx = chord / 2.0
    slab(-hx, -hx + frame, 0.0, span)  # frame
    slab(hx - frame, hx, 0.0, span)
    slab(-hx + frame, hx - frame, 0.0, frame)
    slab(-hx + frame, hx - frame, span - frame, span)
    nx, ny = max(1, round(chord / d["cell_pitch_m"])), max(1, round(span / d["cell_pitch_m"]))
    xs = [-hx + chord * i / nx for i in range(nx + 1)]
    ys = [span * j / ny for j in range(ny + 1)]
    for x in xs[1:-1]:  # walls across the chord, full span between the frame bars
        slab(x - wall / 2, x + wall / 2, frame, span - frame)
    for y in ys[1:-1]:  # walls along the chord, between the others
        for a, b in zip(xs[:-1], xs[1:]):
            slab(a + wall / 2 if a > -hx else -hx + frame, b - wall / 2 if b < hx else hx - frame,
                 y - wall / 2, y + wall / 2)
    shaft = vc.VMesh()
    vc.tube(shaft, (0.0, -d["shaft_inset_m"], 0.0), (0.0, frame + 0.2, 0.0), d["shaft_radius_m"], SHAFT_SIDES, "shaft")
    root = vc.empty("grid_fin")
    tris = {"lattice": lat.to_object("lattice", root)[1], "shaft": shaft.to_object("shaft", root)[1]}
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
