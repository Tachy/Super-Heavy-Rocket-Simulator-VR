"""Bare Raptor 3 and RVac 3 bells: gen_engine.py -- --spec <json> --out <dir> writes raptor.gltf and rvac.gltf.

Frame (Blender axes kept): the engine's GIMBAL POINT is the origin, the axis is X with the thrust direction +X, so the
bell opens toward -X and the exit plane is X = -bell_length_m (the plume's start); the chamber / pump housing sits above
the gimbal (X = 0 .. mount_height_m, inside the stage's skirt). The outer rim of the lip has the diameter of
`nozzle_exit_diameter_m` in config/engines/<id>.toml. Nodes: `<file stem>` root with `bell` (outer wall, inner wall,
lip) and `housing`. The game places one instance per engine at the config ring position (no cant, gimbal in the game).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402,F401
import vehicle_common as vc  # noqa: E402

BELL_RINGS = 14  # [E] profile samples along the bell


def build_engine(spec, eid):
    e = spec["engines"][eid]
    r_e, r_t = e["nozzle_exit_diameter_m"] / 2.0, e["throat_diameter_m"] / 2.0
    length, wall, n = e["bell_length_m"], e["wall_m"], int(e["segments"])
    throat = e["throat_from_exit_m"]  # the bell proper; from there to the gimbal point the chamber (frames: f14 2515 s)
    exp = e["contour_exponent"]
    zs = [throat * i / BELL_RINGS for i in range(BELL_RINGS + 1)]  # distance from the throat along the bell
    x_throat = throat - length  # X of the throat (exit plane at -length)

    def r_out(z):
        return r_t + (r_e - r_t) * (z / throat) ** exp

    outer = [(x_throat - z, r_out(z)) for z in reversed(zs)]  # exit -> throat, X increasing
    inner = [(x_throat - z, r_out(z) - wall) for z in zs]  # throat -> exit
    bell = vc.VMesh()
    vc.revolve(bell, [([(x_throat, 0.0), (x_throat, inner[0][1])], (-1.0, 0.0, 0.0)), (inner, "in"),
                      ([(-length, r_e - wall), (-length, r_e)], (-1.0, 0.0, 0.0)), (outer, "out")], n, r_e, "bell")
    housing = vc.VMesh()
    rh = max(e["mount_radius_frac"] * r_e, r_t * 1.1)
    h = e["mount_height_m"]
    vc.revolve(housing, [([(x_throat, r_t), (0.0, rh)], "out"), ([(0.0, rh), (h, rh)], "out"),
                         ([(h, rh), (h, 0.0)], (1.0, 0.0, 0.0))], n, r_e, "housing")
    root = vc.empty(Path(e["file"]).stem)
    tris = {"bell": bell.to_object("bell", root)[1], "housing": housing.to_object("housing", root)[1]}
    return root, tris


def main():
    args = vc.common.script_args()
    spec = vc.load_spec(args["spec"])
    for eid, e in sorted(spec["engines"].items()):
        vc.common.clear_scene()
        root, tris = build_engine(spec, eid)
        vc.export_selected(Path(args["out"]) / e["file"], [root])
        vc.report_triangles(Path(e["file"]).stem, sum(tris.values()))


if __name__ == "__main__":
    main()
