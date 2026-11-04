"""HLS hull parts: gen_hls_parts.py -- --spec <json> --out <dir> writes hls_parts.gltf (specs with an `hls_parts` block: starship_hls).

Frame: the stage's body frame (station X, nose +X), so the file lines up with ship.gltf. Nodes (shapes and numbers: config/vehicles/detail/starship_hls.toml,
profiles in hls_layout.py), every vertex in body coordinates:
  `thruster_<i>`  one landing thruster pod per config engine of the `landing` group: the EXIT PLANE CENTRE on the config position, the axis (local +X of
                  the profile) on the config thrust direction; bell and pod lie behind the exit plane, towards the hull
  `leg_<i>` `foot_<i>`  the six fixed legs (sleeve, strut, brace, hull saddles) and their pads with the piston stem (the stem and pad slide along +X by the
                  leg's compression); `airlock_hatch`, `elevator_track`, `elevator_basket` fixed nodes (Task 6, hls_legs_layout.py)
  `nose_port`     visual-only docking collar on the nose tip (not in the config, no physics)
The white skin, the solar band, the windows and the thruster bays are textures (hull_textures.py), not nodes.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402,F401
import gen_dorsal_hardware as gdh  # noqa: E402  (frame, merge, revolved: the same placement code as the probes)
import hls_layout as hl  # noqa: E402
import hls_legs_layout as ll  # noqa: E402
import vehicle_common as vc  # noqa: E402
from mathutils import Matrix  # noqa: E402


def _mesh_of(prims, sides):
    """A VMesh from the layout's primitives (body-frame coordinates)."""
    mesh = vc.VMesh()
    for prim in prims:
        if prim[0] == "tube":
            _, p0, p1, r0, r1 = prim
            vc.tube(mesh, p0, p1, r0, sides, "t", radius1=r1)
        else:
            _, centre, ex, ey, ez, half = prim
            vc.box(mesh, centre, ex, ey, ez, half)
    return mesh


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
    legs, hull_r = hls["legs"], hls["legs"]["hull_radius_m"]
    sides = int(d["landing_leg"]["sides"])
    for n in ll.foot_nodes(legs):
        for name, prims in ((n["leg_node"], ll.leg_prims(d["landing_leg"], legs, hull_r, n["azimuth_rad"])),
                            (n["foot_node"], ll.foot_prims(d["landing_leg"], legs, n["azimuth_rad"]))):
            tris += _mesh_of(prims, sides).to_object(name, root)[1]
    tris += _mesh_of(ll.hatch_prims(dict(d["airlock"]), hull_r), sides).to_object("airlock_hatch", root)[1]
    tris += _mesh_of(ll.track_prims(d["elevator"], hull_r), sides).to_object("elevator_track", root)[1]
    tris += _mesh_of(ll.basket_prims(d["elevator"], hull_r), sides).to_object("elevator_basket", root)[1]
    port = gdh.revolved(hl.nose_port_runs(d["nose_port"], hls["length_m"]), seg_t * 2, d["nose_port"]["radius_m"])
    mesh = vc.VMesh()
    gdh.merge(mesh, port, Matrix.Identity(4))
    tris += mesh.to_object("nose_port", root)[1]
    vc.export_selected(Path(args["out"]) / "hls_parts.gltf", [root])
    vc.report_triangles("hls_parts", tris)


if __name__ == "__main__":
    main()
