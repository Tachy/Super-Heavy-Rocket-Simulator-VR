"""Dorsal transfer hardware: gen_dorsal_hardware.py -- --spec <json> --out <dir> writes dorsal_hardware.gltf (ship variants with a `[dorsal]` table).

Tanker: four docking PROBES (soft_capture "active") and two transfer connectors. The same code draws DROGUES for a passive port (the depot, Task 4: node origin = the centre of the mouth plane = the config element).
Frame: the vehicle's body frame (station X, nose +X), so the file lines up with ship.gltf. Nodes (shapes and the invented-hardware statement:
config/vehicles/detail/dock_hardware.toml [dorsal]; layout and local frames: dorsal_layout.py):
  `probe_<i>` / `drogue_<i>`  origin = probe: the EXTENDED TIP (config element = the probe ROOT, tip at root + cone_depth + extension), drogue: the centre of the mouth (the config element); +x = the port axis; the game moves a probe along it
  `connector_<name>`          origin = the centre of the coupling face (config position_m)
  `dock_fairing`              fixed, in body coordinates: one fairing with a bore per probe element (none for drogues)
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402,F401
import dorsal_layout as dl  # noqa: E402
import vehicle_common as vc  # noqa: E402
from mathutils import Matrix, Vector  # noqa: E402


def frame(axis, origin=(0.0, 0.0, 0.0)) -> Matrix:
    """Local +X onto `axis`, then the translation to `origin`."""
    rot = Vector((1.0, 0.0, 0.0)).rotation_difference(Vector(axis).normalized()).to_matrix().to_4x4()
    return Matrix.Translation(Vector(origin)) @ rot


def merge(dst: vc.VMesh, src: vc.VMesh, matrix: Matrix) -> None:
    """Copy `src` into `dst` through `matrix` (a rotation and translation: the normals follow), keeping shared vertices and UVs."""
    vmap = {v: dst.bm.verts.new(matrix @ v.co) for v in src.bm.verts}
    for f in src.bm.faces:
        nf = dst.bm.faces.new([vmap[v] for v in f.verts])
        nf.smooth = f.smooth
        for new_loop, old_loop in zip(nf.loops, f.loops):
            new_loop[dst.uv].uv = old_loop[src.uv].uv
    src.bm.free()


def revolved(runs, segments, uv_radius) -> vc.VMesh:
    mesh = vc.VMesh()
    vc.revolve(mesh, runs, segments, uv_radius, "p")
    return mesh


def main():
    args = vc.common.script_args()
    spec = vc.load_spec(args["spec"])
    dorsal = spec["dorsal"]
    d = dorsal["drawing"]
    seg = int(d["segments"])
    hull_r = 0.5 * spec["stages"]["ship"]["diameter_m"]
    vc.common.clear_scene()
    root = vc.empty("dorsal_hardware")
    tris = 0
    fairing = vc.VMesh()
    for n in dl.element_nodes(dorsal["ports"]):
        axis = n["axis"]
        if n["kind"] == "probe":
            part = revolved(dl.probe_runs(d), seg, d["probe_head_radius_m"])
            g = dl.fairing_geometry(d, hull_r, n["element_m"], axis)
            tip = [e + a * dl.probe_offsets_m(d, hull_r, n["element_m"], axis)["extended"] for e, a in zip(n["element_m"], axis)]
            merge(fairing, revolved(dl.fairing_runs(d, g), seg, d["fairing_radius_m"]), frame(axis, tip))
        else:
            part = revolved(dl.drogue_runs(d, dl.stand_length(d, hull_r, n["element_m"], axis)), seg, d["drogue_mouth_radius_m"])
        mesh = vc.VMesh()
        merge(mesh, part, frame(axis))  # rotation only: the node's origin is the element
        tris += mesh.to_object(n["node"], root)[1]
    for c in dl.connector_nodes(dorsal["connectors"], dorsal["ports"][0]["axis"]):
        part = revolved(dl.connector_runs(d, hull_r, c["position_m"], c["axis"], c["line_diameter_m"]), seg, d["connector_pedestal_radius_m"])
        mesh = vc.VMesh()
        merge(mesh, part, frame(c["axis"]))
        tris += mesh.to_object(c["node"], root)[1]
    if len(fairing.bm.faces):
        tris += fairing.to_object("dock_fairing", root)[1]
    else:
        fairing.bm.free()
    vc.export_selected(Path(args["out"]) / "dorsal_hardware.gltf", [root])
    vc.report_triangles("dorsal_hardware", tris)


if __name__ == "__main__":
    main()
