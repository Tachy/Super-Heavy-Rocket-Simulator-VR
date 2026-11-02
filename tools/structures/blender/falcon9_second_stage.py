"""Falcon 9 second stage: falcon9_second_stage.py -- --spec <json> --out <dir> writes falcon9_stage2.gltf and mvac.gltf.

Frame: the stage's body frame (X station axis, station 0 = the interstage top, where the hull stands; the engine's gimbal
point below it at the config's engine station, nose +X).
falcon9_stage2.gltf: `hull` (plain lathe from the skirt to the top plane, the thruster bumps of the config's pods),
`engine_plate` (the dark blanketed cone from station 0 down to the engine mount); the hull's rim ends flat on the config's
length_m, where the payload's base stands; `exhaust` is part of the engine.
mvac.gltf: root `mvac` with `bell` (regenerative bell, flange, radiatively cooled extension, inner wall, lip) and `housing`
and `duct` (turbine exhaust duct along the extension). Origin = gimbal point (placed by the physics' engine station), thrust +X, bell opens toward -X, exit plane
X = -bell_length_m (the plume's start), rim diameter = config/engines/merlin_vacuum.toml. Everything is plain: detail
belongs into textures; sizes come from the detail file, placement from the config.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402,F401
import vehicle_common as vc  # noqa: E402
from mathutils import Vector  # noqa: E402

AXIS_X = Vector((1.0, 0.0, 0.0))
EXTENSION_RINGS = 16  # [E] profile samples along the extension


def tangent(az):
    return Vector((0.0, -math.sin(az), math.cos(az)))


def build_stage(spec, out):
    stage, d = spec["stages"]["second_stage"], spec["detail"]
    r, top = stage["diameter_m"] / 2.0, stage["length_m"]
    s, h = d["second_stage"], d["hull"]
    n = int(s["segments"])
    vc.common.clear_scene()
    root = vc.empty("falcon9_stage2")
    hull = vc.VMesh()
    band_from = top - s["top_band_height_m"]
    vc.revolve(hull, [([(s["plate_thickness_m"], r), (band_from, r)], "out")], n, r, "hull")
    # The dark line under the trunk: paint on the rim, one surface with the hull, no gap, the rim on the config's length.
    band = vc.VMesh()
    vc.revolve(band, [([(band_from, r), (top, r)], "out"), ([(top, r), (top, 0.0)], (1.0, 0.0, 0.0))], n, r, "hull")
    for pod in stage["rcs_pods"]:
        for az_deg in pod["azimuths_deg"]:
            az = math.radians(az_deg)
            centre = vc.pt(pod["station_m"], az, r + h["pod_height_m"] / 2.0)
            vc.box(hull, centre, AXIS_X, vc.radial_dir(az), tangent(az),
                   (h["pod_length_m"] / 2.0, h["pod_height_m"] / 2.0, h["pod_width_m"] / 2.0))
    plate = vc.VMesh()
    # The aft cone: Starlink 15-25 files 772 / 1156 s, a blanketed cone narrowing from the tank to the engine; it meets the
    # engine's housing top and stays inside the interstage.
    mount_x = stage["engine_station_m"] + spec["mvac"]["mount_height_m"]
    mount_r = spec["mvac"]["mount_radius_m"]
    vc.revolve(plate, [([(mount_x, 0.0), (mount_x, mount_r)], (-1.0, 0.0, 0.0)),
                       ([(mount_x, mount_r), (0.0, s["skirt_radius_m"]), (s["plate_thickness_m"], r)], "out")], n, r, "plate")
    tris = hull.to_object("hull", root)[1] + band.to_object("hull_band", root)[1] + plate.to_object("engine_plate", root)[1]
    vc.export_selected(Path(out) / "falcon9_stage2.gltf", [root])
    vc.report_triangles("falcon9_stage2", tris)


def build_mvac(spec, out):
    m = spec["mvac"]
    r_e, length, wall, n = m["nozzle_exit_diameter_m"] / 2.0, m["bell_length_m"], m["wall_m"], int(m["segments"])
    ext = m["extension_from_exit_m"]
    r_top, exp = m["top_radius_m"], m["contour_exponent"]
    x_top = -length + ext  # X of the extension's top (the flange)
    zs = [ext * i / EXTENSION_RINGS for i in range(EXTENSION_RINGS + 1)]  # distance from the extension's top

    def r_out(z):
        return r_top + (r_e - r_top) * (z / ext) ** exp

    outer = [(x_top - z, r_out(z)) for z in reversed(zs)]  # exit -> top, X increasing
    inner = [(x_top - z, r_out(z) - wall) for z in zs]  # top -> exit
    fl, fh = m["flange_radius_m"], m["flange_height_m"]
    vc.common.clear_scene()
    root = vc.empty(Path(m["file"]).stem)
    bell = vc.VMesh()
    vc.revolve(bell, [(inner, "in"), ([(-length, r_e - wall), (-length, r_e)], (-1.0, 0.0, 0.0)), (outer, "out"),
                      ([(x_top, r_top), (x_top, fl), (x_top + fh, fl), (x_top + fh, r_top)], "out"),
                      ([(x_top, r_top), (0.0, m["chamber_radius_m"])], "out")], n, r_e, "bell")
    housing = vc.VMesh()
    rm, hm = m["mount_radius_m"], m["mount_height_m"]
    vc.revolve(housing, [([(0.0, m["chamber_radius_m"]), (0.0, rm), (hm, rm)], "out"), ([(hm, rm), (hm, 0.0)], (1.0, 0.0, 0.0))],
               n, r_e, "housing")
    duct = vc.VMesh()
    az = math.radians(spec["detail"]["second_stage"]["exhaust_azimuth_deg"])
    dr = spec["detail"]["second_stage"]["exhaust_radius_m"]
    z0, z1 = m["duct_start_from_exit_m"], m["duct_end_from_exit_m"]
    # the duct lies on the extension's outer wall: centre one duct radius outside it at both ends
    def on_wall(dist_from_exit):
        zt = ext - dist_from_exit  # distance from the extension's top
        return vc.pt(-length + dist_from_exit, az, r_out(max(zt, 0.0)) + dr * 0.8)

    vc.tube(duct, on_wall(z0), on_wall(z1), dr, 12, "duct", radius1=dr * 0.8)
    tris = (bell.to_object("bell", root)[1] + housing.to_object("housing", root)[1] + duct.to_object("duct", root)[1])
    vc.export_selected(Path(out) / m["file"], [root])
    vc.report_triangles(Path(m["file"]).stem, tris)


def main():
    args = vc.common.script_args()
    spec = vc.load_spec(args["spec"])
    build_stage(spec, args["out"])
    build_mvac(spec, args["out"])


if __name__ == "__main__":
    main()
