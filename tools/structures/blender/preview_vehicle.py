"""Diagnostic Workbench renders of the assembled vehicle (not part of the model build, not hashed).

Run: blender -b --factory-startup -P preview_vehicle.py -- --spec <json> --out <png dir> [--flap-deg D] [--fin-deg D]
Assembles stack / booster / ship from the generators' parts exactly as the game places them (hull at station 0, fins at
their azimuths with the pivot on the hull, flaps by their hinge basis and rotation sign, engines at the config rings) and
renders them standing (station axis up) from a few angles into <png dir> (git-ignored screenshots/vehicles/).
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402
import gen_engine  # noqa: E402
import gen_flap  # noqa: E402
import gen_grid_fin  # noqa: E402
import gen_hull  # noqa: E402
import vehicle_common as vc  # noqa: E402
from mathutils import Matrix, Vector  # noqa: E402


def _place(src, parent, matrix):
    """Copy of the object tree `src` (shared meshes) under `parent` with `matrix` as the root's local transform."""
    root = src.copy()
    bpy.context.scene.collection.objects.link(root)
    root.parent = parent
    root.matrix_parent_inverse = Matrix.Identity(4)
    root.matrix_basis = matrix
    for ch in src.children:
        c = ch.copy()
        bpy.context.scene.collection.objects.link(c)
        c.parent = root
    return root


def _basis(x, y, z):
    m = Matrix.Identity(4)
    for i, v in enumerate((x, y, z)):
        m[0][i], m[1][i], m[2][i] = v
    return m


def assemble(spec, parent, base_x, stage_id, parts, fin_deg, flap_deg):
    st = spec["stages"][stage_id]
    shift = Matrix.Translation((base_x, 0.0, 0.0))
    hull_root, _ = gen_hull.build_stage(spec, stage_id)
    hull_root.parent = parent
    hull_root.matrix_basis = shift
    if stage_id == "booster" and st["grid_fins"]:
        gf = st["grid_fins"]
        for az in gf["azimuths_deg"]:
            a = math.radians(az)
            m = (shift @ Matrix.Rotation(a, 4, "X") @ Matrix.Translation((gf["station_m"], st["diameter_m"] / 2.0, 0.0))
                 @ Matrix.Rotation(math.radians(fin_deg), 4, "Y"))
            _place(parts["fin"], parent, m)
    for f in st["flaps"]:
        a = math.radians(f["hinge_azimuth_deg"])
        e_r = Vector((0.0, math.cos(a), math.sin(a)))
        stow = Vector((1.0, 0.0, 0.0)).cross(e_r)
        if stow.dot(Vector((0.0, math.cos(math.radians(st["belly_azimuth_deg"])),
                            math.sin(math.radians(st["belly_azimuth_deg"]))))) > 0.0:
            stow = -stow
        pivot = Matrix.Translation(Vector((f["station_aft_m"], 0.0, 0.0)) + e_r * (st["diameter_m"] / 2.0))
        rot = Matrix.Rotation(f["rotation_sign_about_plus_x"] * math.radians(flap_deg), 4, "X")
        _place(parts["flap_" + f["kind"]], parent, shift @ pivot @ rot @ _basis(Vector((1, 0, 0)), e_r, stow))
    for g in st["engines"]:
        for az in g["angles_deg"]:
            a = math.radians(az)
            at = Vector((g["station_m"], g["ring_radius_m"] * math.cos(a), g["ring_radius_m"] * math.sin(a)))
            # model +X = thrust direction = body +x
            _place(parts[g["engine"]], parent, shift @ Matrix.Translation(at))


def main():
    args = vc.common.script_args()
    spec = vc.load_spec(args["spec"])
    out = Path(args["out"])
    fin_deg, flap_deg = float(args.get("fin-deg", 0.0)), float(args.get("flap-deg", 0.0))
    vc.common.clear_scene()
    parts = {"fin": gen_grid_fin.build_fin(spec)[0], "flap_fwd": gen_flap.build_flap(spec, "fwd")[0],
             "flap_aft": gen_flap.build_flap(spec, "aft")[0]}
    for eid in spec["engines"]:
        parts[eid] = gen_engine.build_engine(spec, eid)[0]
    standing = vc.empty("standing")
    standing.rotation_euler = (0.0, -math.pi / 2.0, 0.0)  # station axis up
    bl, sl = spec["stages"]["booster"]["length_m"], spec["stages"]["ship"]["length_m"]
    assemble(spec, standing, 0.0, "booster", parts, fin_deg, flap_deg)
    assemble(spec, standing, bl, "ship", parts, fin_deg, flap_deg)
    for ob in parts.values():  # the originals stay at the origin: hide them
        for o in [ob, *ob.children_recursive]:
            o.hide_render = True
    shots = [("stack", (0, 0, (bl + sl) / 2), 330.0, 0.0, 12.0, (900, 1500)),
             ("ship_top", (0, 0, bl + sl * 0.7), 85.0, 30.0, 18.0, (1400, 1000)),
             ("ship_base", (0, 0, bl + 8), 60.0, 200.0, 35.0, (1400, 1000)),
             ("booster_base", (0, 0, 6), 45.0, 30.0, -25.0, (1400, 1000)),
             ("booster_top", (0, 0, bl - 6), 55.0, 150.0, 20.0, (1400, 1000)),
             ("fin_close", (0, 0, 64), 22.0, 20.0, 15.0, (1400, 1000))]
    for name, centre, dist, az, el, size in shots:
        common_render(out / f"{name}.png", centre, dist, az, el, size)
        print("PREVIEW", name)


def common_render(path, centre, dist, az, el, size):
    # the scene is rendered with a fresh camera each time
    for ob in [o for o in bpy.data.objects if o.type == "CAMERA"]:
        bpy.data.objects.remove(ob, do_unlink=True)
    vc.common.render_preview(path, centre, dist, az, el, size)


if __name__ == "__main__":
    main()
