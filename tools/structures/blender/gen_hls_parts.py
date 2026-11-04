"""HLS hull parts: gen_hls_parts.py -- --spec <json> --out <dir> writes hls_parts.gltf (specs with an `hls_parts` block: starship_hls).

Frame: the stage's body frame (station X, nose +X), so the file lines up with ship.gltf. Nodes (shapes and numbers: config/vehicles/detail/starship_hls.toml,
profiles in hls_layout.py), every vertex in body coordinates:
  `thruster_<i>`  one landing thruster pod per config engine of the `landing` group: the EXIT PLANE CENTRE on the config position, the axis (local +X of
                  the profile) on the config thrust direction; bell and pod lie behind the exit plane, towards the hull
  `nose_port`     visual-only docking collar on the nose tip (not in the config, no physics)
The white skin, the solar band, the windows and the thruster bays are textures (hull_textures.py), not nodes.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402,F401
import gen_dorsal_hardware as gdh  # noqa: E402  (frame, merge, revolved: the same placement code as the probes)
import hls_layout as hl  # noqa: E402
import vehicle_common as vc  # noqa: E402
from mathutils import Matrix  # noqa: E402


def main():
    args = vc.common.script_args()
    spec = vc.load_spec(args["spec"])
    hls = spec["hls_parts"]
    d = hls["drawing"]
    seg_t = int(d["landing_thruster"]["segments"])
    vc.common.clear_scene()
    root = vc.empty("hls_parts")
    tris = 0
    for t in hl.thruster_nodes(hls["thrusters"]):
        part = gdh.revolved(hl.thruster_runs(d["landing_thruster"], t), seg_t, hl.pod_radius_m(d["landing_thruster"], t))
        mesh = vc.VMesh()
        gdh.merge(mesh, part, gdh.frame(t["axis"], t["position_m"]))
        tris += mesh.to_object(t["node"], root)[1]
    port = gdh.revolved(hl.nose_port_runs(d["nose_port"], hls["length_m"]), seg_t * 2, d["nose_port"]["radius_m"])
    mesh = vc.VMesh()
    gdh.merge(mesh, port, Matrix.Identity(4))
    tris += mesh.to_object("nose_port", root)[1]
    vc.export_selected(Path(args["out"]) / "hls_parts.gltf", [root])
    vc.report_triangles("hls_parts", tris)


if __name__ == "__main__":
    main()
