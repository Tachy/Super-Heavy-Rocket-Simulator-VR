"""Falcon 9 first stage hull: gen_falcon9_hull.py -- --spec <json> --out <dir> writes falcon9_stage1.gltf.

Frame: the vehicle body frame (X station axis, station 0 = the outer Merlins' exit plane, (Y, Z) = r (cos a, sin a)).
Nodes: `hull` (plain lathe from the top of the aft cover to the tank top, raceways), `interstage` (open cylinder up to the
stage's length, floor, RCS thruster bumps at the config's pods), `octaweb` (black aft cover, base plate, a clevis block per
leg). Plain surfaces: detail belongs into textures. Everything placed from the config; sizes from the detail file.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402,F401
import vehicle_common as vc  # noqa: E402
from mathutils import Vector  # noqa: E402

AXIS_X = Vector((1.0, 0.0, 0.0))


def tangent(az):
    return Vector((0.0, -math.sin(az), math.cos(az)))


def radial_box(mesh, station, az, r_base, length, width, height):
    """A block on the hull: `length` along the station axis, `height` proud of r_base."""
    centre = vc.pt(station, az, r_base + height / 2.0)
    vc.box(mesh, centre, AXIS_X, vc.radial_dir(az), tangent(az), (length / 2.0, height / 2.0, width / 2.0))


def main():
    args = vc.common.script_args()
    spec = vc.load_spec(args["spec"])
    stage, d = spec["stages"]["first_stage"], spec["detail"]
    r, top = stage["diameter_m"] / 2.0, stage["length_m"]
    h, o = d["hull"], d["octaweb"]
    n, base, joint = int(h["segments"]), h["base_station_m"], h["interstage_base_m"]
    vc.common.clear_scene()
    root = vc.empty("falcon9_stage1")

    hull = vc.VMesh()
    vc.revolve(hull, [([(o["cover_top_m"], r), (joint, r)], "out")], n, r, "hull")
    for az_deg in h["raceway_azimuths_deg"]:
        length = joint - h["raceway_first_m"]
        radial_box(hull, h["raceway_first_m"] + length / 2.0, math.radians(az_deg), r, length, h["raceway_width_m"],
                   h["raceway_height_m"])

    inter = vc.VMesh()
    wall = h["interstage_wall_m"]
    vc.revolve(inter, [([(joint, r), (top, r)], "out"), ([(top, r), (top, r - wall)], (1.0, 0.0, 0.0)),
                       ([(top, r - wall), (joint, r - wall)], "in"), ([(joint, r - wall), (joint, 0.0)], (1.0, 0.0, 0.0))],
               n, r, "interstage")
    for pod in stage["rcs_pods"]:
        for az_deg in pod["azimuths_deg"]:
            radial_box(inter, pod["station_m"], math.radians(az_deg), r, h["pod_length_m"], h["pod_width_m"], h["pod_height_m"])

    octaweb = vc.VMesh()
    c = o["chamfer_m"]
    vc.revolve(octaweb, [([(base, 0.0), (base, r - c)], (-1.0, 0.0, 0.0)), ([(base, r - c), (base + c, r)], "out"),
                         ([(base + c, r), (o["cover_top_m"], r)], "out")], n, r, "octaweb")
    for az_deg in stage["landing_legs"]["azimuths_deg"]:
        radial_box(octaweb, base + c + o["fitting_length_m"] / 2.0, math.radians(az_deg), r, o["fitting_length_m"],
                   o["fitting_width_m"], o["fitting_proud_m"])

    tris = hull.to_object("hull", root)[1] + inter.to_object("interstage", root)[1] + octaweb.to_object("octaweb", root)[1]
    vc.export_selected(Path(args["out"]) / "falcon9_stage1.gltf", [root])
    vc.report_triangles("falcon9_stage1", tris)


if __name__ == "__main__":
    main()
