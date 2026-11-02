"""Crew Dragon: gen_dragon.py -- --spec <json> --out <dir> writes dragon_capsule.gltf, dragon_nose.gltf, dragon_trunk.gltf.

Frames: each stage's body frame (X station axis, nose +X). dragon_capsule.gltf: station 0 = the heat shield's apex; nodes
`capsule` (shoulder, band, sidewall), `heat_shield` (spherical dome of the config's nose radius), `deck` (the dark deck under the
nose with the collar: the place of the docking mechanism), `superdraco_pod` (four lobes at the middle of the config's engine pairs),
`draco_pod` (side windows at the config's pods, the four nose Dracos on the deck). dragon_nose.gltf: node `nose_cone` with its
ORIGIN AT THE HINGE POINT (the vertices are relative to it), closed pose = identity: the game rotates it about the hinge axis
(manifest `nose`). dragon_trunk.gltf: station 0 = the trunk's base; `trunk` (shell, fins, base ring; its top ring stands at the
shield edge, 0.375 m above the stage length, the dome sits in it), `trunk_array`, `trunk_radiator`. Plain surfaces.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402,F401
import bmesh  # noqa: E402
import vehicle_common as vc  # noqa: E402
from mathutils import Vector  # noqa: E402

AXIS_X = Vector((1.0, 0.0, 0.0))
DOME_RINGS = 12  # [E] profile samples of the shield dome
NOSE_RINGS = 10  # [E] profile samples of the nose cone's ellipse
PATCH_SEGMENTS = 8  # [E] segments of a solar / radiator patch across its angle


def tangent(az):
    return Vector((0.0, -math.sin(az), math.cos(az)))


class Wall:
    """The capsule's sidewall profile (station, radius): shoulder, band, two slopes to the hull top."""

    def __init__(self, spec):
        c = spec["detail"]["capsule"]
        cap = spec["stages"]["capsule"]
        self.sag = cap["shield_nose_radius_m"] - math.sqrt(cap["shield_nose_radius_m"] ** 2 - c["shield_edge_radius_m"] ** 2)
        full = cap["diameter_m"] / 2.0
        self.points = [(self.sag, c["shield_edge_radius_m"]), (self.sag + c["shoulder_rise_m"], full), (c["band_top_m"], full),
                       (c["sidewall_mid_station_m"], c["sidewall_mid_radius_m"]), (c["nose_hinge_station_m"], cap["hull_top_radius_m"])]

    def frame(self, station):
        """(radius, slant unit (dx, dr), outward normal (nx, nr)) of the sidewall at `station` above the band."""
        for (x0, r0), (x1, r1) in zip(self.points[2:], self.points[3:]):
            if station <= x1 + 1e-9:
                length = math.hypot(x1 - x0, r1 - r0)
                dx, dr = (x1 - x0) / length, (r1 - r0) / length
                return r0 + (r1 - r0) * (station - x0) / (x1 - x0), (dx, dr), (-dr, dx)
        raise ValueError("station above the hull top")


def wall_box(mesh, wall, station, az_rad, length, width, proud):
    radius, (dx, dr), (nx, nr) = wall.frame(station)
    e_r = vc.radial_dir(az_rad)
    ex = AXIS_X * dx + e_r * dr
    ey = AXIS_X * nx + e_r * nr
    centre = vc.pt(station, az_rad, radius) + ey * (proud / 2.0)
    vc.box(mesh, centre, ex, ey, tangent(az_rad), (length / 2.0, proud / 2.0, width / 2.0))


def build_capsule(spec, out):
    c, cap = spec["detail"]["capsule"], spec["stages"]["capsule"]
    n = int(c["segments"])
    wall = Wall(spec)
    r_n, edge = cap["shield_nose_radius_m"], c["shield_edge_radius_m"]
    vc.common.clear_scene()
    root = vc.empty("dragon_capsule")
    dome = [(r_n - math.sqrt(r_n ** 2 - (edge * i / DOME_RINGS) ** 2), edge * i / DOME_RINGS) for i in range(DOME_RINGS + 1)]
    shield = vc.VMesh()
    vc.revolve(shield, [(dome, (-1.0, 0.0, 0.0))], n, edge, "heat_shield")
    hull = vc.VMesh()
    vc.revolve(hull, [(wall.points, "out")], n, cap["diameter_m"] / 2.0, "capsule")
    deck = vc.VMesh()
    hs, top_r = c["nose_hinge_station_m"], cap["hull_top_radius_m"]
    cr, ch = c["deck_collar_radius_m"], c["deck_collar_height_m"]
    vc.revolve(deck, [([(hs, top_r), (hs, cr)], (1.0, 0.0, 0.0)), ([(hs, cr), (hs + ch, cr)], "out"),
                      ([(hs + ch, cr), (hs + ch, 0.0)], (1.0, 0.0, 0.0))], n, top_r, "deck")
    sd = vc.VMesh()
    for lobe in cap["superdraco_lobes_deg"]:
        wall_box(sd, wall, c["lobe_centre_station_m"], math.radians(lobe), c["lobe_length_m"], c["lobe_width_m"], c["lobe_proud_m"])
    draco = vc.VMesh()
    for pod in cap["rcs_pods"]:
        for az_deg in pod["azimuths_deg"]:
            az = math.radians(az_deg)
            if pod["id"] == "nose":  # under the nose: a small nozzle from the deck up to the config's station
                vc.tube(draco, vc.pt(hs, az, pod["radius_m"]), vc.pt(pod["station_m"], az, pod["radius_m"]),
                        c["nose_draco_radius_m"] * 0.6, 12, "nose_draco", radius1=c["nose_draco_radius_m"])
            else:
                wall_box(draco, wall, pod["station_m"], az, c["draco_length_m"], c["draco_width_m"], c["draco_proud_m"])
    tris = sum(m.to_object(name, root)[1] for name, m in
               (("capsule", hull), ("heat_shield", shield), ("deck", deck), ("superdraco_pod", sd), ("draco_pod", draco)))
    vc.export_selected(Path(out) / "dragon_capsule.gltf", [root])
    vc.report_triangles("dragon_capsule", tris)


def build_nose(spec, out):
    c, cap, nose = spec["detail"]["capsule"], spec["stages"]["capsule"], spec["nose"]
    n = int(c["segments"])
    length, base_r = nose["length_m"], cap["hull_top_radius_m"]
    ellipse = [(nose["hinge_station_m"] + length * math.sin(0.5 * math.pi * i / NOSE_RINGS),
                base_r * math.cos(0.5 * math.pi * i / NOSE_RINGS)) for i in range(NOSE_RINGS + 1)]
    vc.common.clear_scene()
    root = vc.empty("dragon_nose")
    mesh = vc.VMesh()
    vc.revolve(mesh, [(ellipse, "out"), ([(nose["hinge_station_m"], base_r), (nose["hinge_station_m"], 0.0)], (-1.0, 0.0, 0.0))],
               n, base_r, "nose_cone")
    hinge = vc.pt(nose["hinge_station_m"], math.radians(nose["hinge_azimuth_deg"]), nose["hinge_radius_m"])
    bmesh.ops.translate(mesh.bm, vec=-hinge, verts=mesh.bm.verts)  # the origin is the hinge point
    tris = mesh.to_object("nose_cone", root)[1]
    vc.export_selected(Path(out) / "dragon_nose.gltf", [root])
    vc.report_triangles("dragon_nose", tris)


def patch(mesh, x0, x1, r, a0, a1, group):
    """A thin curved panel on the cylinder of radius r between stations x0..x1 and azimuths a0..a1 (radians)."""
    for i in range(PATCH_SEGMENTS):
        b0 = a0 + (a1 - a0) * i / PATCH_SEGMENTS
        b1 = a0 + (a1 - a0) * (i + 1) / PATCH_SEGMENTS
        mesh.face([vc.pt(x0, b0, r), vc.pt(x0, b1, r), vc.pt(x1, b1, r), vc.pt(x1, b0, r)],
                  [(b0 * r, 1.0 - x0), (b1 * r, 1.0 - x0), (b1 * r, 1.0 - x1), (b0 * r, 1.0 - x1)],
                  vc.radial_dir(0.5 * (b0 + b1)), group)


def fin(mesh, az_rad, r_skin, tip_r, height, thickness):
    """Triangular fin: base on station 0 from the skin out to tip_r, tapering to the skin at `height`."""
    e_r, t = vc.radial_dir(az_rad), tangent(az_rad)
    pts = [AXIS_X * 0.0 + e_r * r_skin, e_r * tip_r, AXIS_X * height + e_r * r_skin]
    for side in (-1.0, 1.0):
        mesh.face([p + t * (side * thickness / 2.0) for p in pts], [(0, 0), (1, 0), (0, 1)], t * side)
    mesh.face([pts[0] - t * (thickness / 2.0), pts[0] + t * (thickness / 2.0), pts[1] + t * (thickness / 2.0), pts[1] - t * (thickness / 2.0)],
              [(0, 0), (1, 0), (1, 1), (0, 1)], -AXIS_X)
    mesh.face([pts[1] - t * (thickness / 2.0), pts[1] + t * (thickness / 2.0), pts[2] + t * (thickness / 2.0), pts[2] - t * (thickness / 2.0)],
              [(0, 0), (1, 0), (1, 1), (0, 1)], e_r + AXIS_X)


def build_trunk(spec, out):
    c, tr = spec["detail"]["trunk"], spec["stages"]["trunk"]
    r, n = tr["diameter_m"] / 2.0, int(c["segments"])
    top = tr["length_m"] + c["shield_well_m"]
    vc.common.clear_scene()
    root = vc.empty("dragon_trunk")
    wall = 0.04  # [E] ring wall at the open top
    floor = top - 0.9  # [E] the dome's recess floor
    body = vc.VMesh()
    vc.revolve(body, [([(0.0, r), (top, r)], "out"), ([(top, r), (top, r - wall)], (1.0, 0.0, 0.0)),
                      ([(top, r - wall), (floor, r - wall)], "in"), ([(floor, r - wall), (floor, 0.0)], (1.0, 0.0, 0.0)),
                      ([(0.0, 0.0), (0.0, r)], (-1.0, 0.0, 0.0))], n, r, "trunk")
    first = math.radians(c["fin_first_azimuth_deg"])
    step = 2.0 * math.pi / c["fin_count"]
    for k in range(int(c["fin_count"])):
        fin(body, first + k * step, r * 0.97, c["fin_tip_radius_m"], c["fin_height_m"], c["fin_thickness_m"])
    gap = math.radians(c["panel_gap_deg"])
    array, radiator = vc.VMesh(), vc.VMesh()
    for k in range(int(c["fin_count"])):
        a0, a1 = first + k * step + gap, first + (k + 1) * step - gap
        patch(radiator, c["base_ring_height_m"], c["radiator_to_m"], r + c["panel_proud_m"], a0, a1, ("radiator", k))
        patch(array, c["array_from_m"], tr["length_m"] - 0.1, r + c["panel_proud_m"], a0, a1, ("array", k))
    tris = body.to_object("trunk", root)[1] + array.to_object("trunk_array", root)[1] + radiator.to_object("trunk_radiator", root)[1]
    vc.export_selected(Path(out) / "dragon_trunk.gltf", [root])
    vc.report_triangles("dragon_trunk", tris)


def main():
    args = vc.common.script_args()
    spec = vc.load_spec(args["spec"])
    build_capsule(spec, args["out"])
    build_nose(spec, args["out"])
    build_trunk(spec, args["out"])


if __name__ == "__main__":
    main()
