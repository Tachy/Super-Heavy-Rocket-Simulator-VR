"""The ISS's parts on the joints: iss_arrays.py -- --spec <json> --out <dir> writes iss_arrays.gltf. Own drawing from the tables of config/vehicles/detail/iss.toml.

Every mesh is relative to its joint's origin (manifest `joints`), drawn in the file pose (blankets in the x-z plane, masts along x); the game's joint nodes carry the stated pose.
Nodes, i in the [[wing]] order: `array_<i>` (blankets; on beta_joint_<i>), `mast_<i>` (mast, canister, tip bars), `irosa_<i>` (where the wing has one), `truss_outboard_<k>` and
`pv_radiator_<k>` (on alpha_joint_<k>), `radiator_<j>` (on the fixed radiator joint: j in the [[gamma]] order).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402,F401
import iss_station as st  # noqa: E402
import vehicle_common as vc  # noqa: E402
from mathutils import Vector  # noqa: E402


def by_id(rows, key="id"):
    return {r[key]: r for r in rows}


def shifted(p, o):
    return (p[0] - o[0], p[1] - o[1], p[2] - o[2])


def draw_wing(blanket, mast, box, joint_o, shape):
    """Two blankets either side of the mast, in the plane through the joint (y of the joint)."""
    lo, hi = box["min_m"], box["max_m"]
    x0, x1, z0, z1 = lo[0], hi[0], lo[2], hi[2]
    y = joint_o[1]
    w = shape["blanket_width_m"]
    for za, zb in ((z0, z0 + w), (z1 - w, z1)):
        st.plate(blanket, [shifted(p, joint_o) for p in ((x0, y, za), (x1, y, za), (x1, y, zb), (x0, y, zb))], 0.02)
    zm = 0.5 * (z0 + z1)
    tip_at_low_x = joint_o[0] > 0.5 * (x0 + x1)  # the canister stands at the joint's end
    can_l, can_w, can_t = shape["canister_m"]
    x_root = x1 - 0.5 * can_l if tip_at_low_x else x0 + 0.5 * can_l
    x_tip = x0 if tip_at_low_x else x1
    vc.box(mast, shifted((x_root, y, zm), joint_o), (1, 0, 0), (0, 1, 0), (0, 0, 1), (0.5 * can_l, 0.5 * can_t, 0.5 * can_w))
    vc.tube(mast, shifted((x_root, y, zm), joint_o), shifted((x_tip, y, zm), joint_o), shape["mast_radius_m"], 8, "mast")
    bar = shape["tip_bar_m"]
    vc.box(mast, shifted((x_tip, y, zm), joint_o), (1, 0, 0), (0, 1, 0), (0, 0, 1), (0.5 * bar, 0.08, 0.5 * (z1 - z0)))


def main():
    args = vc.common.script_args()
    spec = vc.load_spec(args["spec"])
    d = spec["detail"]
    arrays = by_id(d["array"])
    irosas = by_id(d["irosa_plane"])
    joints = by_id(d["joint"])
    vc.common.clear_scene()
    root = vc.empty("iss_arrays")
    tris = 0
    shape = d["array_shape"]
    for i, wing in enumerate(d["wing"]):
        o = joints[wing["joint"]]["origin_m"]
        blanket, mast = vc.VMesh(), vc.VMesh()
        draw_wing(blanket, mast, arrays[wing["array"]], o, shape)
        tris += blanket.to_object(f"array_{i}", root)[1]
        tris += mast.to_object(f"mast_{i}", root)[1]
        if wing["irosa"]:
            ir = irosas[wing["irosa"]]
            mesh, boom = vc.VMesh(), vc.VMesh()
            st.plate(mesh, [shifted(p, o) for p in ir["corners_m"]], 0.02)
            c = ir["corners_m"]
            for a, b in ((c[0], c[1]), (c[2], c[3])):  # the two booms along the long edges
                vc.tube(boom, shifted(a, o), shifted(b, o), shape["irosa_boom_m"], 6, ("boom", a[2]))
            mid = [0.5 * (c[0][k] + c[3][k]) for k in range(3)] if abs(c[0][0] - o[0]) < abs(c[1][0] - o[0]) else [0.5 * (c[1][k] + c[2][k]) for k in range(3)]
            vc.tube(boom, (0.0, 0.0, 0.0), shifted(mid, o), 0.12, 8, "modkit")
            tris += mesh.to_object(f"irosa_{i}", root)[1]
            tris += boom.to_object(f"irosa_boom_{i}", root)[1]
    alpha_ids = {1: "PORT_ALPHA_ROT", 2: "STBD_ALPHA_ROT"}
    for k, joint_id in alpha_ids.items():
        o = joints[joint_id]["origin_m"]
        truss, pvr = vc.VMesh(), vc.VMesh()
        for bay in d["bay"]:
            if bay["outboard"] == k:
                moved = dict(bay)
                moved["y_m"] = [bay["y_m"][0] - o[1], bay["y_m"][1] - o[1]]
                moved["x_m"] = [bay["x_m"][0] - o[0], bay["x_m"][1] - o[0]]
                moved["z_m"] = [bay["z_m"][0] - o[2], bay["z_m"][1] - o[2]]
                st.lattice(truss, moved, d["truss_lattice"])
        tag = "P" if k == 1 else "S"
        for row in d["plate"]:
            if row["id"].startswith("PVR_" + tag):
                st.plate(pvr, [shifted(p, o) for p in row["corners_m"]], 0.04)
        for row in d["tube"]:
            if row["id"].startswith("PVR_root_" + tag):
                vc.tube(truss, shifted(row["a_m"], o), shifted(row["b_m"], o), row["radius_m"], 8, row["id"])
        tris += truss.to_object(f"truss_outboard_{k - 1}", root)[1]
        tris += pvr.to_object(f"pv_radiator_{k - 1}", root)[1]
    for j, row in enumerate(d["gamma"]):
        o = joints[row["joint"]]["origin_m"]
        mesh = vc.VMesh()
        x0, x1 = row["x_m"]
        z = row["z_m"]
        count, amp = row["zigzag_m"]
        for y0, y1 in row["panels_y_m"]:
            for k in range(int(count)):  # the accordion: panels hinge alternately high and low
                xa, xb = x0 + (x1 - x0) * k / count, x0 + (x1 - x0) * (k + 1) / count
                za, zb = z + (amp if k % 2 else -amp), z + (-amp if k % 2 else amp)
                st.plate(mesh, [shifted(p, o) for p in ((xa, y0, za), (xb, y0, zb), (xb, y1, zb), (xa, y1, za))], 0.05)
        tris += mesh.to_object(f"radiator_{j}", root)[1]
    vc.export_selected(Path(args["out"]) / "iss_arrays.gltf", [root])
    vc.report_triangles("iss_arrays", tris)


if __name__ == "__main__":
    main()
