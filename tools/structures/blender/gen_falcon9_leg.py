"""Falcon 9 landing leg: gen_falcon9_leg.py -- --spec <json> --out <dir> writes falcon9_leg.gltf.

The real mechanism (F_Falcon_9_B1046, Starlink 15-25 file 1124 s): a rigid A-shaped leg hinged at the base, stowed along the
hull with its foot up, turned out and down by a telescoping pusher from the hull's upper ring. VehicleModels.leg_poses is the
rig, a function of the snapshot's deployment only. Frames:
- `leg_main` (leg + foot pad): origin the hinge, +X from the hinge to the foot point (L, 0, 0), +Y the face outward when stowed,
  Z along the hinge. The pad's underside lies flat on the ground in the deployed pose, the foot point at its centre.
- `leg_pusher`: unit tube along +X from 0 to 1; the rig scales it along X to the distance anchor -> leg.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402,F401
import vehicle_common as vc  # noqa: E402
from mathutils import Vector  # noqa: E402

TUBE_SIDES = 12  # [E]
PAD_SIDES = 24  # [E]


def main():
    args = vc.common.script_args()
    leg = vc.load_spec(args["spec"])["leg"]
    length = leg["length_m"]
    stowed = math.radians(leg["stowed_angle_deg"])
    deployed = math.radians(leg["deployed_angle_deg"])

    def outer_y(u):
        """Outer face in the leg frame: constant radius when stowed (stowed_outer_radius_m), flush along the hull."""
        return (leg["stowed_outer_radius_m"] - leg["hinge_radius_m"] - u * math.sin(stowed)) / math.cos(stowed)

    # Tapered beam from the root (wide, deep) to the tip.
    tip_outer = outer_y(length)
    corners = []
    for u, half, y_out, y_in in ((0.0, leg["root_width_m"] / 2.0, outer_y(0.0), 0.0),
                                 (length, leg["tip_width_m"] / 2.0, tip_outer, tip_outer - leg["tip_depth_m"])):
        corners.append([Vector((u, y_out, -half)), Vector((u, y_out, half)), Vector((u, y_in, half)), Vector((u, y_in, -half))])
    a, b = corners
    main_mesh = vc.VMesh()
    for i in range(4):
        j = (i + 1) % 4
        pts = [a[i], a[j], b[j], b[i]]
        centre = sum(pts, Vector((0.0, 0.0, 0.0))) / 4.0
        out = centre - Vector((centre.x, (tip_outer + outer_y(0.0)) / 4.0, 0.0))
        main_mesh.face(pts, [(0, 1), (1, 1), (1, 0), (0, 0)], out)
    main_mesh.face(a, [(0, 1), (1, 1), (1, 0), (0, 0)], (-1.0, 0.0, 0.0))
    main_mesh.face(b, [(0, 1), (1, 1), (1, 0), (0, 0)], (1.0, 0.0, 0.0))

    # Foot pad: a disc whose axis is the body's +X in the deployed pose, in the leg frame R_z(-deployed) X.
    axis = Vector((math.cos(deployed), -math.sin(deployed), 0.0))
    foot = Vector((length, 0.0, 0.0))
    vc.tube(main_mesh, foot, foot + axis * leg["pad_thickness_m"], leg["pad_radius_m"], PAD_SIDES, "pad")

    pusher = vc.VMesh()
    vc.tube(pusher, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), leg["pusher_radius_m"], TUBE_SIDES, "pusher")

    vc.common.clear_scene()
    root = vc.empty("falcon9_leg")
    tris = main_mesh.to_object("leg_main", root)[1] + pusher.to_object("leg_pusher", root)[1]
    vc.export_selected(Path(args["out"]) / "falcon9_leg.gltf", [root])
    vc.report_triangles("falcon9_leg", tris)


if __name__ == "__main__":
    main()
