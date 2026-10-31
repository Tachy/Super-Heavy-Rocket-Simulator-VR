"""Grid fin of the booster, redrawn from the 1:100 reference model (regular octagon lattice with chamfer bars and the
catch hooks; the drawn size is the model's, the pivot stays on the hinge).

Grid fin of the booster: gen_grid_fin.py -- --spec <json> --out <dir> writes grid_fin.gltf.

Frame (Blender axes kept): the fin of azimuth 0. The pivot (origin) is the hinge on the hull surface: the hinge axis is
+Y (radial outward, core/src/aero/grid_fin.cpp: the deflection d turns the lattice normal from the tangent toward +X,
that is a right-handed rotation by +d about the radial axis; zero = rest pose). The lattice lies in the X-Y plane: chord
X = +-a/2 around the hinge station, span Y = 0 .. a (a = across flats of the model's octagon, root flat at Y = 0),
thickness Z = +-t/2 (the cells open along Z). The game rotates it about
the stage's X axis to the fin's azimuth and puts the pivot at (station_m, radius R). Nodes: `grid_fin` (root) with
`lattice`, `hooks` (catch hardware) and `shaft` (the hinge shaft along +Y, reaching shaft_inset_m into the hull).
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402,F401
import vehicle_common as vc  # noqa: E402
from mathutils import Vector  # noqa: E402

SHAFT_SIDES = 12  # [E]
AX = (Vector((1.0, 0.0, 0.0)), Vector((0.0, 1.0, 0.0)), Vector((0.0, 0.0, 1.0)))


def fin_dims(spec):
    """Drawn fin size: the 1:100 model's octagon, chord (X) = span (Y) = across flats. The physics fin (config area and
    radius) is 13.5 m^2 at a 3.0 m span; the deviation is documented in docs/subsystems/vehicle-models.md."""
    a = spec["detail"]["grid_fin"]["across_flats_m"]
    return a, a


def build_fin(spec):
    d = spec["detail"]["grid_fin"]
    a = d["across_flats_m"]
    s = a / (1.0 + math.sqrt(2.0))  # regular octagon: side length
    k = (a + s) / 2.0  # |x| + |yc| <= k inside the diagonal edges (yc = y - a/2)
    t, wall, frame = d["thickness_m"], d["wall_m"], d["frame_m"]
    lat = vc.VMesh()

    def slab(x0, x1, y0, y1):
        vc.box(lat, ((x0 + x1) / 2, (y0 + y1) / 2, 0.0), *AX, ((x1 - x0) / 2, (y1 - y0) / 2, t / 2))

    inner = frame * 0.9  # lattice walls stop inside the frame bars
    for sx in (-1.0, 1.0):  # axis-aligned flat bars: root / tip (along X) and the two sides (along Y)
        slab(sx * (a / 2 - frame), sx * a / 2, a / 2 - s / 2 - frame * 0.5, a / 2 + s / 2 + frame * 0.5)
    slab(-s / 2 - frame * 0.5, s / 2 + frame * 0.5, 0.0, frame)
    # serrated outer edge (photo of the caught booster: stepped notches along the tip): the tip bar sits `depth` inside,
    # the lattice walls run on to the edge, and only every second cell is closed by a cap plate at the rim
    depth = d["edge_tooth_depth_m"]
    slab(-s / 2 - frame * 0.5, s / 2 + frame * 0.5, a - depth - frame, a - depth)
    pitch_e = d["edge_tooth_pitch_m"]
    for i in range(int(s / pitch_e) + 1):
        xc = -s / 2 + frame + (i + 0.5) * pitch_e
        if abs(xc) < s / 2 - frame:
            slab(xc - pitch_e / 4.0, xc + pitch_e / 4.0, a - frame, a)
    edge_len = (a - s) * math.sqrt(2.0) / 2.0
    for sx in (-1.0, 1.0):  # the four chamfer bars
        for sy in (-1.0, 1.0):
            ex = Vector((sx / math.sqrt(2.0), -sy / math.sqrt(2.0), 0.0))
            ey = Vector((sx / math.sqrt(2.0), sy / math.sqrt(2.0), 0.0))
            mid = Vector((sx * (s + a) / 4.0, a / 2 + sy * (s + a) / 4.0, 0.0)) - ey * (frame / 2.0)
            vc.box(lat, mid, ex, ey, Vector((0.0, 0.0, 1.0)), (edge_len / 2.0, frame / 2.0, t / 2.0))
    pitch = d["cell_pitch_m"]  # the model's measured lattice: walls every 0.35 m, centred on the fin
    walls = int((a - 2.0 * frame) / pitch)
    offsets = [(i - (walls - 1) / 2.0) * pitch for i in range(walls)]
    for x in offsets:  # walls across the chord, clipped by the octagon
        half = min(a / 2 - inner, k - abs(x) - inner * 1.3)
        slab(x - wall / 2, x + wall / 2, a / 2 - half, a / 2 + half)
    for yc in offsets:  # walls along the chord
        half = min(a / 2 - inner, k - abs(yc) - inner * 1.3)
        slab(-half, half, a / 2 + yc - wall / 2, a / 2 + yc + wall / 2)
    hooks = vc.VMesh()  # catch hardware: half-round hooks along the -Z face next to the root
    hr = d["catch_hook_radius_m"]
    for i in range(int(d["catch_hook_count"])):
        y = 0.55 + i * d["catch_hook_pitch_m"] * 2.0
        vc.tube(hooks, (-s / 2, y, -t / 2), (s / 2, y, -t / 2), hr, 12, ("hook", i))
    shaft = vc.VMesh()
    vc.tube(shaft, (0.0, -d["shaft_inset_m"], 0.0), (0.0, frame + 0.2, 0.0), d["shaft_radius_m"], SHAFT_SIDES, "shaft")
    root = vc.empty("grid_fin")
    tris = {"lattice": lat.to_object("lattice", root)[1], "hooks": hooks.to_object("hooks", root)[1],
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
