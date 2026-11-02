"""Crew Dragon's IDSS docking mechanism: idss_mechanism.py -- --spec <json> --out <dir> writes idss_mechanism.gltf.

Own drawing from the IDSS IDD Rev E numbers typed in [idss] of config/vehicles/detail/crew_dragon.toml; no third-party mesh. Parts that move are single models the game places
(`game/vehicle/docking_mechanism_view.gd`). Frame: body axes, X = the port axis, azimuth 0 = +Y. Origins:
`flange` (fixed: tunnel, flange), `hcs_metal` (fixed: seals, passive hooks, receptacles, separators), `hook_pockets` (fixed): the HCS mating plane on the axis, clocked as on the vehicle.
`ring` (capture ring, strikers): the SCS plane on the axis, clocked. `petal`: the same origin, drawn on azimuth 0. `guide_pin`: base centre on the HCS plane.
`hook` (the active hook): its pivot, u along +Z (the circle's tangent at azimuth 0), v along +X, width along Y; the outline's own pose is the closed one.
`strut_barrel`, `strut_middle`, `strut_rod`: from the origin along +X, one tube length.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402,F401
import vehicle_common as vc  # noqa: E402
from mathutils import Vector  # noqa: E402

AXIS_X = Vector((1.0, 0.0, 0.0))
UV4 = [(0, 0), (1, 0), (1, 1), (0, 1)]
PETAL_COLS = 16  # [E] columns across a petal's base
PETAL_ROWS = 4   # [E] rows base to tip
PIN_SIDES = 16   # [E]
TUBE_SIDES = 16  # [E]
RECEPTACLE_SIDES = 24  # [E]
RECEPTACLE_INNER_SCALE = 0.65  # [E] the hole's share of the collar, Fig 3.2.3-1 detail by eye


def theta_azimuth(theta_deg, up, axis):
    """Body azimuth (rad) of the IDD's axial-view angle: viewed from outside onto the port, clockwise from `up`."""
    u, v = Vector(up), Vector(axis)
    right = u.cross(v)
    t = math.radians(theta_deg)
    d = u * math.cos(t) + right * math.sin(t)
    return math.atan2(d.z, d.y)


def tangent(az):
    return Vector((0.0, -math.sin(az), math.cos(az)))


def extrude(mesh, outline, width, place):
    """Prism of an (u, v) outline over `width`; place(u, v, w) -> point. Flat faces."""
    area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(outline, outline[1:] + outline[:1]))
    sign = 1.0 if area > 0.0 else -1.0
    for w, out in ((-0.5 * width, -1.0), (0.5 * width, 1.0)):
        pts = [place(u, v, w) for u, v in outline]
        centre = place(outline[0][0], outline[0][1], w) + (place(outline[0][0], outline[0][1], w) - place(outline[0][0], outline[0][1], -w))
        mesh.face(pts, [(u, v) for u, v in outline], centre - place(outline[0][0], outline[0][1], 0.0), None)
    for (u0, v0), (u1, v1) in zip(outline, outline[1:] + outline[:1]):
        normal_uv = (sign * (v1 - v0), -sign * (u1 - u0))
        out = place(u0 + normal_uv[0], v0 + normal_uv[1], 0.0) - place(u0, v0, 0.0)
        mesh.face([place(u0, v0, -0.5 * width), place(u1, v1, -0.5 * width), place(u1, v1, 0.5 * width), place(u0, v0, 0.5 * width)], UV4, out, None)


def tangent_box(mesh, az, r, x, half_t, half_r, half_x):
    e_r, t = vc.radial_dir(az), tangent(az)
    vc.box(mesh, AXIS_X * x + e_r * r, t, e_r, AXIS_X, (half_t, half_r, half_x))


def build_flange(flange, metal, pockets, d, up, axis):
    seg = int(d["segments"])
    r_t = 0.5 * d["tunnel_diameter_m"]
    neck_outer = spec_neck_outer
    vc.revolve(flange, [([(0.0, neck_outer), (0.0, r_t)], (1.0, 0.0, 0.0)), ([(0.0, r_t), (-d["tunnel_wall_m"], r_t)], "in")], seg, 1.0, "flange")
    width = d["seal_width_m"]
    for dia in d["seal_diameters_m"]:
        r = 0.5 * dia
        h = d["seal_height_m"]
        vc.revolve(metal, [([(0.0, r - 0.5 * width), (h, r)], "in"), ([(h, r), (0.0, r + 0.5 * width)], "out")], seg, 1.0, ("seal", dia))
    count = int(d["hook_count"])
    r_h = d["hook_radius_m"]
    outline = [(u / 1000.0, v / 1000.0) for u, v in d["hook_passive_mm"]]
    top = max(v for _, v in outline)
    base_x = d["hook_passive_top_above_hcs_m"] - top
    tip_u = max(u for u, v in outline)
    for k in range(count):
        az = theta_azimuth(d["hook_first_theta_deg"] + 360.0 * k / count, up, axis)
        e_r, t = vc.radial_dir(az), tangent(az)
        centre = e_r * r_h
        u0 = d["hook_passive_edge_m"] - tip_u

        def place(u, v, w):
            return centre + t * (u0 + u) + AXIS_X * (base_x + v) + e_r * w

        extrude(metal, outline, d["hook_width_m"], place)
        half = 0.5 * d["hook_pocket_length_m"]
        half_w = 0.5 * d["hook_width_m"]
        lift = 0.0006
        pts = [centre + t * s * half + e_r * q * half_w + AXIS_X * lift for s, q in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        pockets.face(pts, UV4, AXIS_X, None)
    r_pin = 0.5 * d["pin_circle_diameter_m"]
    for theta in d["receptacle_theta_deg"]:
        az = theta_azimuth(theta, up, axis)
        c = vc.radial_dir(az) * r_pin
        e_r, t = vc.radial_dir(az), tangent(az)
        r0 = 0.5 * d["receptacle_outer_m"]
        r1 = r0 * RECEPTACLE_INNER_SCALE
        lift = 0.001
        for i in range(RECEPTACLE_SIDES):
            a0, a1 = 2.0 * math.pi * i / RECEPTACLE_SIDES, 2.0 * math.pi * (i + 1) / RECEPTACLE_SIDES
            d0, d1 = e_r * math.cos(a0) + t * math.sin(a0), e_r * math.cos(a1) + t * math.sin(a1)
            metal.face([c + d0 * r0 + AXIS_X * lift, c + d1 * r0 + AXIS_X * lift, c + d1 * r1 + AXIS_X * lift, c + d0 * r1 + AXIS_X * lift], UV4, AXIS_X, None)
            pockets.face([c + AXIS_X * lift, c + d0 * r1 + AXIS_X * lift, c + d1 * r1 + AXIS_X * lift], UV4[:3], AXIS_X, None)
    hs = d["separator_box_m"]
    for theta in d["separator_theta_deg"]:
        tangent_box(metal, theta_azimuth(theta, up, axis), d["separator_r_m"], hs[2], hs[0], hs[1], hs[2])


def build_ring(ring, d, up, axis):
    seg = int(d["segments"])
    section = d["ring_section_m"]
    profile = [(x, r) for r, x in section]
    runs = [([profile[i], profile[i + 1]], ("in" if profile[i + 1][1] < 0.59 else "out")) for i in range(len(profile) - 1)]
    runs.append(([profile[-1], profile[0]], (-1.0, 0.0, 0.0)))
    vc.revolve(ring, runs, seg, 1.0, "ring")
    half_t, half_r, half_x = d["striker_box_m"]
    r_in = section[0][0]
    for theta in d["striker_theta_deg"]:
        tangent_box(ring, theta_azimuth(theta, up, axis), r_in - half_r, -half_x - 0.012, half_t, half_r, half_x)


def build_petal(petal, d):
    """One petal on azimuth 0: a plate on the 45 deg cone from the base circle to the tip circle, with a rear rib and the latch box. X = 0 at the SCS plane (peak of the ring)."""
    r0, r1 = 0.5 * d["petal_base_diameter_m"], 0.5 * d["petal_tip_diameter_m"]
    height = d["petal_height_m"]
    h0, h1 = d["petal_base_half_deg"], d["petal_tip_half_deg"]
    th = d["petal_thickness_m"]
    slope = height / (r0 - r1)
    length = math.hypot(height, r0 - r1)
    nx, nr = (r0 - r1) / length, height / length  # normal of the cone's inner face, towards the axis and up

    def grid(offset):
        rows = []
        for i in range(PETAL_ROWS + 1):
            s = i / PETAL_ROWS
            r = r0 + (r1 - r0) * s
            half = h0 + (h1 - h0) * s
            row = []
            for j in range(PETAL_COLS + 1):
                az = math.radians(half * (2.0 * j / PETAL_COLS - 1.0))
                x = (r0 - r) * slope
                row.append(AXIS_X * (x + offset * nx) + vc.radial_dir(az) * (r - offset * nr))
            rows.append(row)
        return rows

    top, under = grid(0.0), grid(-th)
    up_in = AXIS_X * nx - vc.radial_dir(0.0) * nr
    for i in range(PETAL_ROWS):
        for j in range(PETAL_COLS):
            petal.face([top[i][j], top[i][j + 1], top[i + 1][j + 1], top[i + 1][j]], UV4, up_in, "top")
            petal.face([under[i][j], under[i][j + 1], under[i + 1][j + 1], under[i + 1][j]], UV4, -up_in, "under")
    for i in range(PETAL_ROWS):
        for j in (0, PETAL_COLS):
            side = tangent(0.0) * (1.0 if j else -1.0)
            petal.face([top[i][j], top[i + 1][j], under[i + 1][j], under[i][j]], UV4, side, None)
    for j in range(PETAL_COLS):
        petal.face([top[-1][j], top[-1][j + 1], under[-1][j + 1], under[-1][j]], UV4, -vc.radial_dir(0.0), None)
    foot_r, foot_d = d["petal_rib_foot_m"]
    half = 0.5 * th
    rib = [(r1, height), (r0, 0.0), (foot_r, -foot_d)]
    for side in (-1.0, 1.0):
        petal.face([AXIS_X * x + vc.radial_dir(0.0) * r + tangent(0.0) * side * half for r, x in rib], UV4[:3], tangent(0.0) * side, None)
    for (ra, xa), (rb, xb) in zip(rib, rib[1:] + rib[:1]):
        petal.face([AXIS_X * xa + vc.radial_dir(0.0) * ra - tangent(0.0) * half, AXIS_X * xa + vc.radial_dir(0.0) * ra + tangent(0.0) * half,
                    AXIS_X * xb + vc.radial_dir(0.0) * rb + tangent(0.0) * half, AXIS_X * xb + vc.radial_dir(0.0) * rb - tangent(0.0) * half], UV4, -AXIS_X - vc.radial_dir(0.0), None)
    r_l, h_l = d["latch_box_m"]
    x_l = (r0 - r_l) * slope
    centre = AXIS_X * x_l + vc.radial_dir(0.0) * r_l + up_in * 0.004
    e_a = (AXIS_X * nr + vc.radial_dir(0.0) * nx).normalized()  # along the face, towards the base outward
    vc.box(petal, centre, tangent(0.0), e_a, up_in, (h_l, h_l, 0.004))


def build_pin(pin, d):
    r = 0.5 * d["pin_diameter_m"]
    shaft, height = d["pin_shaft_m"], d["pin_height_m"]
    vc.revolve(pin, [([(0.0, r), (shaft, r)], "out"), ([(shaft, r), (height, 0.0)], "out")], PIN_SIDES, r, "pin")


def build_hook(hook, d):
    outline = [(u / 1000.0, v / 1000.0) for u, v in d["hook_active_mm"]]
    extrude(hook, outline, d["hook_width_m"], lambda u, v, w: Vector((v, w, u)))


def build_tube(mesh, length, radius):
    vc.tube(mesh, (0.0, 0.0, 0.0), (length, 0.0, 0.0), radius, TUBE_SIDES, "tube")


spec_neck_outer = 0.0


def main():
    global spec_neck_outer
    args = vc.common.script_args()
    spec = vc.load_spec(args["spec"])
    d = spec["detail"]["idss"]
    cap = spec["stages"]["capsule"]
    up, axis = cap["port_up"], cap["port_axis"]
    spec_neck_outer = spec["detail"]["capsule"]["neck_m"][0][1]
    vc.common.clear_scene()
    root = vc.empty("idss_mechanism")
    flange, metal, pockets, ring, petal, pin, hook = vc.VMesh(), vc.VMesh(), vc.VMesh(), vc.VMesh(), vc.VMesh(), vc.VMesh(), vc.VMesh()
    build_flange(flange, metal, pockets, d, up, axis)
    build_ring(ring, d, up, axis)
    build_petal(petal, d)
    build_pin(pin, d)
    build_hook(hook, d)
    tubes = {}
    for name, radius in zip(("strut_barrel", "strut_middle", "strut_rod"), d["strut_radii_m"]):
        tubes[name] = vc.VMesh()
        build_tube(tubes[name], d["strut_tube_length_m"], radius)
    parts = [("flange", flange), ("hcs_metal", metal), ("hook_pockets", pockets), ("ring", ring), ("petal", petal), ("guide_pin", pin), ("hook", hook)] + list(tubes.items())
    tris = sum(m.to_object(name, root)[1] for name, m in parts)
    vc.export_selected(Path(args["out"]) / "idss_mechanism.gltf", [root])
    vc.report_triangles("idss_mechanism", tris)


if __name__ == "__main__":
    main()
