"""Flaps of the ship: gen_flap.py -- --spec <json> --out <dir> writes flap_fwd.gltf and flap_aft.gltf.

Frame (Blender axes kept): the HINGE LINE is the X axis from the hinge's aft end (X = 0, the origin) to its forward end
(X = hinge length = station_fwd - station_aft); the panel lies along +Z in the stowed pose (span, Z = 0 .. span), its
thickness is Y (+-t/2) and the outer (windward when deflected, tile-covered) face is +Y. Placement and rotation by the
game: basis (X = station axis, Y = e_r radial outward at the hinge azimuth, Z = tangent X x e_r), mirrored in Z when the
hinge's stow direction points to the belly side; the deflection d rotates the panel about +X by
`rotation_sign_about_plus_x` * d (manifest, from core/src/aero/flap.cpp), zero = rest pose. UV = (X metres, Z metres):
glTF v is Z. Nodes: `flap` (root) with `panel` and `hinge` (the hinge fairing tube, centred on the X axis).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402,F401
import vehicle_common as vc  # noqa: E402
from mathutils import Vector  # noqa: E402

HINGE_SIDES = 16  # [E]


def flap_dims(spec, kind):
    for st in spec["stages"].values():
        for f in st["flaps"]:
            if f["kind"] == kind:
                return f["station_fwd_m"] - f["station_aft_m"], f["span_m"]
    raise SystemExit(f"no flap of kind {kind}")


def build_flap(spec, kind):
    d = spec["detail"]["flap"]
    length, span = flap_dims(spec, kind)
    t, c = d["thickness_m"], d["corner_chamfer_m"]
    outline = [(0.0, 0.0), (length, 0.0), (length, span - c), (length - c, span), (c, span), (0.0, span - c)]
    cx, cz = length / 2.0, span / 2.0
    mesh = vc.VMesh()
    n = len(outline)
    for i in range(n):
        (x0, z0), (x1, z1) = outline[i], outline[(i + 1) % n]
        pts = [Vector((x0, -t / 2, z0)), Vector((x1, -t / 2, z1)), Vector((x1, t / 2, z1)), Vector((x0, t / 2, z0))]
        uvs = [(x0, 1.0 - z0), (x1, 1.0 - z1), (x1, 1.0 - z1), (x0, 1.0 - z0)]
        mesh.face(pts, uvs, Vector(((x0 + x1) / 2 - cx, 0.0, (z0 + z1) / 2 - cz)))
    for y, sign in ((-t / 2, -1.0), (t / 2, 1.0)):
        mesh.face([Vector((x, y, z)) for x, z in outline], [(x, 1.0 - z) for x, z in outline], Vector((0, sign, 0)))
    hinge = vc.VMesh()
    vc.tube(hinge, (0.0, 0.0, 0.0), (length, 0.0, 0.0), d["hinge_radius_m"], HINGE_SIDES, "hinge")
    root = vc.empty("flap")
    tris = {"panel": mesh.to_object("panel", root)[1], "hinge": hinge.to_object("hinge", root)[1]}
    return root, tris


def main():
    args = vc.common.script_args()
    spec = vc.load_spec(args["spec"])
    out = Path(args["out"])
    for kind in ("fwd", "aft"):
        vc.common.clear_scene()
        root, tris = build_flap(spec, kind)
        vc.export_selected(out / f"flap_{kind}.gltf", [root])
        vc.report_triangles(f"flap_{kind}", sum(tris.values()))


if __name__ == "__main__":
    main()
