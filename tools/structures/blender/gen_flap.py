"""Flaps of the ship, redrawn from the 1:100 reference model (planform, wedge section, hinge lugs; the detail file's
[flap.aft] / [flap.fwd] outlines; the drawn hinge edge is centred on the physics hinge,
the physics sizes stay in the config).

Flaps of the ship: gen_flap.py -- --spec <json> --out <dir> writes flap_fwd.gltf and flap_aft.gltf.

Frame (Blender axes kept): the HINGE LINE is the X axis from the hinge's aft end (X = 0, the origin) to its forward end
(X = hinge length = station_fwd - station_aft); the panel lies along +Z in the stowed pose (span, Z = 0 .. span), its
thickness is Y (+-t/2, a wedge: thick at the hinge, thin at the outer edge) and the outer (windward when deflected, tile-covered) face is +Y. Placement and rotation by the
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


def flap_outline(spec, kind):
    """(u, v) outline points of the drawn flap in the frame (X along the hinge, Z = span) and the hinge's X range.

    The drawn hinge edge (1:100 model) is centred on the physics hinge: u0 = (physics length - drawn length) / 2."""
    length, _ = flap_dims(spec, kind)
    f = spec["detail"]["flap"][kind]
    u0 = (length - f["hinge_drawn_length_m"]) / 2.0
    pts = [(u0 + u, v) for u, v in zip(f["outline_u_m"], f["outline_v_m"])]
    return pts, (u0, u0 + f["hinge_drawn_length_m"]), [u0 + u for u in f["lug_u_m"]]


def build_flap(spec, kind):
    d = spec["detail"]["flap"]
    outline, (h0, h1), lugs = flap_outline(spec, kind)
    v_max = max(v for _, v in outline)

    def half_t(v):  # wedge section: thick at the hinge, thin at the outer edge (symmetric about the hinge plane)
        return 0.5 * (d["root_thickness_m"] + (d["tip_thickness_m"] - d["root_thickness_m"]) * v / v_max)

    cx = sum(x for x, _ in outline) / len(outline)
    cz = sum(z for _, z in outline) / len(outline)
    mesh = vc.VMesh()
    n = len(outline)
    for i in range(n):
        (x0, z0), (x1, z1) = outline[i], outline[(i + 1) % n]
        pts = [Vector((x0, -half_t(z0), z0)), Vector((x1, -half_t(z1), z1)),
               Vector((x1, half_t(z1), z1)), Vector((x0, half_t(z0), z0))]
        uvs = [(x0, 1.0 - z0), (x1, 1.0 - z1), (x1, 1.0 - z1), (x0, 1.0 - z0)]
        mesh.face(pts, uvs, Vector(((x0 + x1) / 2 - cx, 0.0, (z0 + z1) / 2 - cz)))
    for sign in (-1.0, 1.0):
        mesh.face([Vector((x, sign * half_t(z), z)) for x, z in outline], [(x, 1.0 - z) for x, z in outline],
                  Vector((0, sign, 0)))
    hinge = vc.VMesh()  # the rounded hinge fairing along the hinge edge, and one knuckle per lug
    vc.tube(hinge, (h0, 0.0, 0.0), (h1, 0.0, 0.0), d["hinge_radius_m"], HINGE_SIDES, "hinge")
    for k, x in enumerate(lugs):
        half = (d["lug_length_m"] / 2.0, d["hinge_radius_m"] * 1.3, d["lug_height_m"] / 2.0)
        vc.box(hinge, (x, 0.0, d["lug_height_m"] / 2.0 - 0.1), Vector((1.0, 0.0, 0.0)), Vector((0.0, 1.0, 0.0)),
               Vector((0.0, 0.0, 1.0)), half)
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
