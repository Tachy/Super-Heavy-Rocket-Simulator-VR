"""Pure layout of the Mechazilla modules: chopstick, carriage, QD arm as lists of members, no Blender.

The generators (gen_chopstick.py, gen_carriage.py, gen_qd_arm.py) add the members to meshes; pytest reads the same lists
for the clearance and parallel bearing cases (Blender's Python 3.10 and the tools' Python both import this file).

A member is (node, kind, size_m, p0, p1, size_y_m) in the model frame; size_y_m only for 'rect'. Node names are the glTF
mesh nodes, one material each.
Chopstick frame: hinge (physics pivot) at the origin, the bearing piece along +X, +Y toward the catch axis (the arm
closes that way; the game mirrors the -across arm), strip top Z = 0 (the plane the catch pins land on).
Carriage frame: X toward the pad, Y across, Z up, origin in the hinge line, on the tower's centre plane, in the deck plane.
QD arm frame: X toward the pad, Y across, Z up, origin at the tower's pad-side face, on the centre plane, in the tip deck.
"""
from __future__ import annotations

import math

# Member map of one arm, from the photos: launch-structures-measurements.md "Arm member map". Large tubes only.
ARM_MEMBERS = {"top": 5, "caps": 3, "lower_chord": 2, "knee_strut": 1, "root_columns": 3, "ties": 2, "posts": 4,
               "diagonals": 3, "probe": 1}
SUPPLY_LINES = 3  # thin lines that read as supply lines, never as structure
SMALLEST_STRUCTURE_M = 0.4  # the hanging probe; every other member is at least 0.8 m
POST_FRACTIONS = (0.25, 0.36, 0.575, 0.82)  # [E] +-0.03 stations of the 4 posts, teslaoracle 2025-03 far arm


def _member(node, kind, size, p0, p1, size_y=None):
    return (node, kind, size, tuple(p0), tuple(p1), size_y)


def bearing_tube_y(c: dict) -> float:
    """Axis of the bearing tube: the flat plate on its inner side has its face on `inner_face_y_m`."""
    return c["inner_face_y_m"] - c["plate_thickness_m"] - 0.5 * c["bearing_tube_m"]


def top_plan(detail: dict) -> list:
    """Plan polyline (x, y) of the U's outer top tube: hinge, bend, end of the leg, connector's end on the bearing tube."""
    c = detail["chopstick"]
    leg_y = bearing_tube_y(c) - c["leg_offset_m"]
    return [(0.0, 0.0), (c["root_kink_x_m"], -c["bend_out_m"]), (c["leg_end_x_m"], leg_y),
            (c["connector_end_x_m"], bearing_tube_y(c))]


def lower_chord_z(c: dict, x: float) -> float:
    """Lower chord axis: rises from the knee to the kink (the 'inclined lower chord'), then gently to its tip end."""
    length = c["length_m"]
    pts = [(c["knee_frac"] * length, -c["knee_depth_m"]), (c["kink_frac"] * length, -c["depth_m"]),
           (c["lower_end_frac"] * length, -c["end_depth_m"])]
    for (x0, z0), (x1, z1) in zip(pts, pts[1:]):
        if x <= x1:
            return z0 + (z1 - z0) * (x - x0) / (x1 - x0)
    return pts[-1][1]


def _on_leg(plan: list, x: float) -> float:
    (x0, y0), (x1, y1) = plan[1], plan[2]
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


def chopstick(detail: dict) -> list:
    c, prof = detail["chopstick"], detail["profiles"]
    chord, brace, thin = (prof[c["chord_profile"]]["size_m"], prof[c["brace_profile"]]["size_m"],
                          prof[c["thin_profile"]]["size_m"])
    length, bearing = c["length_m"], c["bearing_tube_m"]
    y_b, x_b = bearing_tube_y(c), c["bearing_start_x_m"]
    z_top = -0.5 * bearing - c["strip_thickness_m"] + 0.05  # the strip lies on the bearing tube (0.05 m weld), top Z = 0
    plan = top_plan(detail)
    x_knee, x_kink, x_end = c["knee_frac"] * length, c["kink_frac"] * length, c["lower_end_frac"] * length
    ms = []

    def tube(node, size, p0, p1):
        ms.append(_member(node, "tube", size, p0, p1))

    def inner_y(x):  # the truss plane's top: inner root tube, then the bearing tube
        return y_b * min(x, x_b) / x_b

    def top(x, y):
        return (x, y, z_top)

    def low(x):
        return (x, y_b, lower_chord_z(c, x))

    # The U in plan: root out from the hinge, leg back in, connector to the bearing tube; inner root tube to the bearing
    # tube's start; the bearing tube runs along the arm's X, so both are parallel at the core's closed pose (opening 0).
    for a, b in zip(plan, plan[1:]):
        tube("truss", chord, top(*a), top(*b))
    tube("truss", chord, top(0.0, 0.0), top(x_b, y_b))
    tube("truss", bearing, top(x_b, y_b), top(length, y_b))
    (x0, y0), (x1, y1) = plan[1], plan[2]  # round caps on the leg's and the bearing tube's end (real: both end proud)
    run = math.hypot(x1 - x0, y1 - y0)
    tube("truss", chord * c["cap_scale"], top(x1 - 0.1 * (x1 - x0) / run, y1 - 0.1 * (y1 - y0) / run),
         top(x1 + 0.1 * (x1 - x0) / run, y1 + 0.1 * (y1 - y0) / run))
    tube("truss", bearing * c["cap_scale"], top(length - 0.1, y_b), top(length + 0.1, y_b))

    # One truss plane under the bearing line: lower chord, 4 posts, 3 diagonals falling outward (not a V section).
    tube("truss", chord, low(x_knee), low(x_kink))
    tube("truss", chord, low(x_kink), low(x_end))
    tube("truss", chord * c["cap_scale"], low(x_end - 0.1), low(x_end + 0.1))
    posts = [f * length for f in POST_FRACTIONS]
    for x in posts:
        tube("truss", brace, top(x, inner_y(x)), low(x))
    for xa, xb in zip(posts, posts[1:]):
        tube("truss", brace, top(xa, inner_y(xa)), low(xb))
    for x, y_outer in ((c["root_kink_x_m"], plan[1][1]), (x_b, _on_leg(plan, x_b))):  # the two ties across the top
        tube("truss", brace, top(x, y_outer), top(x, inner_y(x)))

    # Root: hinge column, outer and inner root columns and the knee strut, all to one foot.
    foot = (c["column_x_m"], c["foot_y_m"], -c["foot_depth_m"])
    tube("truss", c["column_size_m"], top(c["column_x_m"], 0.0), foot)
    tube("truss", chord, top(*plan[1]), foot)
    tube("truss", chord, top(c["root_kink_x_m"], inner_y(c["root_kink_x_m"])), foot)
    tube("truss", chord, low(x_knee), foot)

    # Flat bearing plate on the inner side of the bearing tube, tan strip on top of it, contact pad on the plate.
    y_plate = c["inner_face_y_m"] - 0.5 * c["plate_thickness_m"]
    z_plate = -c["strip_thickness_m"] - 0.5 * c["plate_height_m"]
    ms.append(_member("bearing", "rect", c["plate_thickness_m"], (x_b + c["plate_inset_m"], y_plate, z_plate),
                      (c["plate_end_x_m"], y_plate, z_plate), c["plate_height_m"]))
    z_strip = -0.5 * c["strip_thickness_m"]
    ms.append(_member("strip", "rect", c["strip_width_m"], (x_b, y_b, z_strip), (c["plate_end_x_m"], y_b, z_strip),
                      c["strip_thickness_m"]))
    pad = c["pad_length_m"], c["pad_width_m"], c["pad_height_m"]
    x_pad, y_pad = c["pad_x_m"], c["inner_face_y_m"] + c["pad_proud_m"] - pad[1] / 2.0
    z_pad = -c["pad_top_depth_m"] - pad[2] / 2.0
    ms.append(_member("pad", "rect", pad[1], (x_pad - pad[0] / 2.0, y_pad, z_pad),
                      (x_pad + pad[0] / 2.0, y_pad, z_pad), pad[2]))

    # Telescopic pushers: short cylinders on top of the tip [S NSF 2025-08], in two stages.
    x0, x1 = c["pusher_start_x_m"], c["pusher_end_x_m"]
    z = c["pusher_size_m"] / 2.0
    for dy in (0.0, c["pusher_spacing_m"]):
        y0 = y_b - dy
        tube("hardware", c["pusher_size_m"], (x0, y0, z), (0.5 * (x0 + x1), y0, z))
        tube("hardware", c["pusher_size_m"] * 0.6, (0.5 * (x0 + x1), y0, z), (x1, y0, z))

    # Stabilizer arm: a hanging probe at the lower chord's end and its long thin brace [S NSF 2025-08 "stabilizer arms"].
    x_probe = x_end - 0.4
    z_probe = lower_chord_z(c, x_probe)
    probe_foot = (x_probe, y_b, z_probe - c["probe_length_m"])
    tube("truss", c["probe_size_m"], (x_probe, y_b, z_probe), probe_foot)
    tube("fittings", thin, low(c["probe_brace_frac"] * length), probe_foot)
    ms.append(_member("hardware", "rect", 0.5, (x_probe, y_b, probe_foot[2] - 0.6), (x_probe, y_b, probe_foot[2]), 0.5))

    # Thin supply lines (not structure): along the leg, the bearing tube and the lower chord.
    d = c["supply_line_m"]
    tube("hardware", d, top(plan[1][0], plan[1][1] - 0.6), top(plan[2][0], plan[2][1] - 0.6))
    tube("hardware", d, (x_b, y_b - 0.7, z_top - 0.5), (length - 0.5, y_b - 0.7, z_top - 0.5))
    tube("hardware", d, (x_kink, y_b - 0.6, lower_chord_z(c, x_kink) + 0.3), (x_end, y_b - 0.6, -c["end_depth_m"] + 0.3))

    # Two ladders from the leg to the lower chord as one flat black panel each, a platform on the leg's outer side, a mast.
    for frac in c["ladder_a_frac"], c["ladder_b_frac"]:
        x = frac * length
        ms.append(_member("fittings", "rect", 0.5, (x, _on_leg(plan, x), z_top), low(x + 0.9), 0.04))
    x_p = c["platform_frac"] * length
    y_p = _on_leg(plan, x_p) - 1.1
    ms.append(_member("hardware", "rect", 1.3, (x_p - 1.3, y_p, -2.6), (x_p + 1.3, y_p, -2.6), 0.1))
    x_m = c["mast_x_m"]
    tube("hardware", c["mast_size_m"], (x_m, _on_leg(plan, x_m), 0.0), (x_m, _on_leg(plan, x_m), c["mast_height_m"]))
    return ms


def carriage(detail: dict) -> list:
    k, prof = detail["carriage"], detail["profiles"]
    chord = prof[detail["chopstick"]["chord_profile"]]["size_m"]
    half, top, bottom = k["width_m"] / 2.0, k["top_z_m"], -k["bottom_depth_m"]
    x_face = k["frame_x_m"]
    ms = []
    box = k["beam_m"]

    def beam(node, p0, p1, size=box):
        ms.append(_member(node, "rect", size, p0, p1, size))

    for z in (top - box / 2.0, bottom + box / 2.0):
        beam("frame", (x_face, -half, z), (x_face, half, z))
    for y in (-half, half):
        beam("frame", (x_face, y, bottom), (x_face, y, top))
    beam("frame", (x_face, -half, bottom), (x_face, 0.0, top), box * 0.6)
    beam("frame", (x_face, half, bottom), (x_face, 0.0, top), box * 0.6)
    # Hinge frame at each corner: a box out to the hinge line (the arm-following actuator is drawn by the game).
    for sign in (-1.0, 1.0):
        y = sign * k["hinge_y_m"]
        beam("frame", (x_face, y, -box / 2.0), (0.0, y, -box / 2.0), box * 1.2)
        beam("frame", (x_face, y, bottom + box / 2.0), (0.0, y, -k["hinge_lower_m"]), box * 0.8)
    # Platforms: the walkway beam across the face reaching past the corner, with railing.
    p_end = half + k["platform_reach_m"]
    ms.append(_member("frame", "rect", k["platform_width_m"], (x_face, -p_end, top + 0.1), (x_face, p_end, top + 0.1),
                      0.2))
    for sign in (-1.0, 1.0):
        y0 = sign * (k["width_m"] / 2.0 + k["platform_reach_m"])
        ms.append(_member("hardware", "tube", 0.08, (x_face + k["platform_width_m"] / 2.0, y0, top + 1.2),
                          (x_face + k["platform_width_m"] / 2.0, 0.0, top + 1.2)))
    # The drum / sheave block above the carriage with the hoist cables (carriage photo 2025-12: worker for scale).
    d, w, z_d = k["drum_diameter_m"], k["drum_width_m"], top + k["drum_diameter_m"] / 2.0 + k["drum_gap_m"]
    ms.append(_member("frame", "tube", d, (x_face, -w / 2.0, z_d), (x_face, w / 2.0, z_d)))
    beam("frame", (x_face, 0.0, top), (x_face, 0.0, z_d), chord)
    return ms


def qd_arm(detail: dict) -> list:
    q, prof = detail["qd_arm"], detail["profiles"]
    chord, brace = prof[detail["chopstick"]["chord_profile"]]["size_m"] * 0.7, prof["chopstick_brace"]["size_m"] * 0.6
    reach, x_step = q["reach_m"], q["root_length_m"]
    half_root, half_tip = q["root_width_m"] / 2.0, q["tip_width_m"] / 2.0
    rise, low_tip, low_root, foot = q["root_rise_m"], -q["tip_depth_m"], -q["root_depth_m"], -q["foot_depth_m"]
    ms = []

    def tube(node, size, p0, p1):
        ms.append(_member(node, "tube", size, p0, p1))

    def box(node, a, b, w, h):
        ms.append(_member(node, "rect", w, a, b, h))

    n = max(2, round(x_step / q["panel_m"]))
    for side in (-1.0, 1.0):
        y = side * half_root
        tube("truss", chord, (0.0, y, rise), (x_step, y, 0.0))  # top chord falling to the tip part
        tube("truss", chord, (0.0, y, low_root), (x_step, y, low_tip))  # lower chord
        tube("truss", chord, (0.0, y, low_root), (0.0, y, foot))  # column at the face, the big curved tube's line
        tube("truss", chord, (x_step * 0.45, y, low_root * 0.75 + low_tip * 0.25), (0.0, y, foot))
        tube("truss", brace, (0.0, y, rise), (0.0, y, low_root))
        for i in range(n):
            x0, x1 = x_step * i / n, x_step * (i + 1) / n
            z_t0, z_t1 = rise * (1.0 - x0 / x_step), rise * (1.0 - x1 / x_step)
            z_b0 = low_root + (low_tip - low_root) * x0 / x_step
            z_b1 = low_root + (low_tip - low_root) * x1 / x_step
            tube("truss", brace, (x1, y, z_b1), (x1, y, z_t1))
            lo, hi = ((x0, z_b0), (x1, z_t1)) if i % 2 else ((x0, z_t0), (x1, z_b1))
            tube("truss", brace, (lo[0], y, lo[1]), (hi[0], y, hi[1]))
    # White cabinets inside the root truss (QD photo 2026-05).
    for side in (-1.0, 1.0):  # two small cabinets beside the centre line, not one slab across the root
        box("hardware", (x_step * 0.25, side * half_root * 0.5, rise * 0.4 - 1.2),
            (x_step * 0.5, side * half_root * 0.5, rise * 0.25 - 1.2), 1.8, 2.4)
    # Tip part: a shallow lattice platform, deck on top, stair up to it, connector head at the end.
    for side in (-1.0, 1.0):
        y = side * half_tip
        tube("truss", brace, (x_step, y, 0.0), (reach, y, 0.0))
        tube("truss", brace, (x_step, y, low_tip), (reach, y, low_tip))
        tube("truss", brace, (reach, y, 0.0), (reach, y, low_tip))
        tube("truss", brace, (x_step, y, low_tip), (reach, y, 0.0))
    box("truss", (x_step, 0.0, low_tip - 0.05), (reach, 0.0, low_tip - 0.05), 2.0 * half_tip, 0.1)
    head_x0 = reach - q["head_length_m"]
    box("hardware", (head_x0, 0.0, q["head_z_m"] - 0.6), (reach, 0.0, q["head_z_m"] - 0.6), q["head_width_m"], 1.2)
    tube("hardware", 0.5, (head_x0 - 1.0, 0.0, 0.25), (head_x0, 0.0, q["head_z_m"] - 0.6))
    for side in (-1.0, 1.0):
        tube("hardware", 0.08, (0.0, side * half_root, rise + 1.1), (x_step, side * half_root, 1.1))
        tube("hardware", 0.08, (x_step, side * half_tip, 1.1), (reach, side * half_tip, 1.1))
    return ms
