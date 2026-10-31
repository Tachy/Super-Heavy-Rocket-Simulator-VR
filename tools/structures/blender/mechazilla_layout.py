"""Pure layout of the Mechazilla modules: chopstick, carriage, QD arm as lists of members, no Blender.

The generators (gen_chopstick.py, gen_carriage.py, gen_qd_arm.py) add the members to meshes; pytest reads the same lists
for the clearance and parallel-tips cases (Blender's Python 3.10 and the tools' Python both import this file).

A member is (node, kind, size_m, p0, p1, size_y_m) in the model frame; size_y_m only for 'rect'. Node names are the glTF
mesh nodes, one material each.
Chopstick frame: hinge (physics pivot) at the origin, the arm's main beam along +X, +Y toward the catch axis (the arm
closes that way; the game mirrors the -across arm), deck top Z = 0 (the plane the catch pins land on).
Carriage frame: X toward the pad, Y across, Z up, origin in the hinge line, on the tower's centre plane, in the deck plane.
QD arm frame: X toward the pad, Y across, Z up, origin at the tower's pad-side face, on the centre plane, in the tip deck.
"""
from __future__ import annotations

import math

# Fractions of the arm that come from the plan / side views: see the [chopstick] table of the detail file.
RING_POST_FRACTIONS = (0.35, 0.48, 0.61, 0.74, 0.87)  # [E] posts: 4 panels between knee and tip, side view 2025-08


def _member(node, kind, size, p0, p1, size_y=None):
    return (node, kind, size, tuple(p0), tuple(p1), size_y)


def plan_line(c: dict, y_main: float) -> list:
    """Plan polyline (x, y) of a top chord whose main beam runs at `y_main`: root bend, parallel main beam, tip bend."""
    x_root, x_tip = c["root_kink_x_m"], c["tip_kink_x_m"]
    y0 = y_main - x_root * math.tan(math.radians(c["root_kink_deg"]))
    y_end = y_main + (c["length_m"] - x_tip) * math.tan(math.radians(c["tip_kink_deg"]))
    return [(0.0, y0), (x_root, y_main), (x_tip, y_main), (c["length_m"], y_end)]


def chord_y(c: dict, prof: dict) -> tuple:
    """(inner, outer) main-beam chord axes, from the inner face and the width."""
    r = prof["size_m"] / 2.0
    return c["inner_face_y_m"] - r, c["inner_face_y_m"] - c["width_m"] + r


def lower_chord_z(c: dict, x: float) -> float:
    """Lower chord axis: rises from the knee to the kink (the 'inclined lower chord'), then gently to its tip end."""
    length = c["length_m"]
    pts = [(c["knee_frac"] * length, -c["knee_depth_m"]), (c["kink_frac"] * length, -c["depth_m"]),
           (c["lower_end_frac"] * length, -c["end_depth_m"])]
    for (x0, z0), (x1, z1) in zip(pts, pts[1:]):
        if x <= x1:
            return z0 + (z1 - z0) * (x - x0) / (x1 - x0)
    return pts[-1][1]


def _at(line: list, x: float) -> float:
    for (x0, y0), (x1, y1) in zip(line, line[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return line[-1][1]


def chopstick(detail: dict) -> list:
    c, prof = detail["chopstick"], detail["profiles"]
    chord, brace, thin = (prof[c["chord_profile"]]["size_m"], prof[c["brace_profile"]]["size_m"],
                          prof[c["thin_profile"]]["size_m"])
    length = c["length_m"]
    y_in, y_out = chord_y(c, prof[c["chord_profile"]])
    y_mid = 0.5 * (y_in + y_out)
    z_top = -chord / 2.0 - c["deck_thickness_m"] + 0.05  # chord axis: the deck plate lies on the chords (0.05 m weld)
    lines = [plan_line(c, y_in), plan_line(c, y_out)]
    x_knee, x_end = c["knee_frac"] * length, c["lower_end_frac"] * length
    ms = []

    def tube(node, size, p0, p1):
        ms.append(_member(node, "tube", size, p0, p1))

    for line in lines:  # top chords with round end caps
        for (x0, y0), (x1, y1) in zip(line, line[1:]):
            tube("truss", chord, (x0, y0, z_top), (x1, y1, z_top))
        xe, ye = line[-1]
        tube("truss", chord * c["cap_scale"], (xe - 0.1, ye, z_top), (xe + 0.1, ye, z_top))
    # Single lower chord on the centre line: the arm's section is a V (user view 2026-10-31: struts converge to a node).
    stations = [x_knee, c["kink_frac"] * length, x_end]
    for x0, x1 in zip(stations, stations[1:]):
        tube("truss", chord, (x0, y_mid, lower_chord_z(c, x0)), (x1, y_mid, lower_chord_z(c, x1)))
    tube("truss", chord * c["cap_scale"], (x_end - 0.1, y_mid, -c["end_depth_m"]),
         (x_end + 0.1, y_mid, -c["end_depth_m"]))
    # V frames at the posts and W diagonals in both V planes.
    posts = [f * length for f in RING_POST_FRACTIONS]
    for x in posts:
        for line in lines:
            tube("truss", brace, (x, y_mid, lower_chord_z(c, x)), (x, _at(line, x), z_top))
    for i, (x0, x1) in enumerate(zip(posts, posts[1:])):
        lo, hi = (x0, x1) if i % 2 == 0 else (x1, x0)
        for line in lines:
            tube("truss", brace, (lo, y_mid, lower_chord_z(c, lo)), (hi, _at(line, hi), z_top))
    for line in lines:  # last strut: from the lower chord's end up to the tip
        tube("truss", brace, (x_end, y_mid, -c["end_depth_m"]), (line[-2][0], line[-2][1], z_top))
    for x in posts + [length - 0.3, c["root_kink_x_m"]]:  # ties across the top
        tube("truss", thin, (x, _at(lines[0], x), z_top), (x, _at(lines[1], x), z_top))

    # Root: hinge column to the foot, inclined columns from the top chords and the knee to the foot (side view 2025-08).
    x_col = c["column_x_m"]
    foot = (x_col, y_mid, -c["foot_depth_m"])
    tube("truss", c["column_size_m"], (x_col, y_mid, z_top), foot)
    for line in lines:
        tube("truss", chord, (c["root_kink_x_m"], line[1][1], z_top), foot)
        tube("truss", brace, (c["root_kink_x_m"], line[1][1], z_top), (x_knee, y_mid, -c["knee_depth_m"]))
    tube("truss", chord, (x_knee, y_mid, -c["knee_depth_m"]), foot)
    tube("truss", chord, (x_knee, y_mid, -c["knee_depth_m"]), (x_col, y_mid, -c["knee_depth_m"] + 2.0))

    # Deck in three plates (root, main, tip), the tan strip on its inner half, railing on the outer edge.
    deck_y = c["deck_thickness_m"]
    z_deck = -deck_y / 2.0
    width = y_in - y_out + chord - 0.1
    centre = plan_line(c, y_mid)
    for i, (a, b) in enumerate(zip(centre, centre[1:])):
        plate = c["tip_deck_frac"] * width if i == 2 else width  # the bent tip's plate narrows: its corner clears the hull
        ms.append(_member("deck", "rect", plate, (a[0], a[1], z_deck), (b[0], b[1], z_deck), deck_y))
    ms.append(_member("strip", "rect", c["strip_width_m"], (c["root_kink_x_m"], y_in - c["strip_width_m"], 0.01),
                      (c["tip_kink_x_m"], y_in - c["strip_width_m"], 0.01), 0.02))
    y_rail = y_out - chord / 2.0 + 0.1
    h = c["railing_height_m"]
    rail_line = plan_line(c, y_rail)
    for a, b in zip(rail_line, rail_line[1:]):
        tube("hardware", 0.08, (a[0], a[1], h), (b[0], b[1], h))
    x = 1.0
    while x < length - 1.0:
        y = _at(rail_line, x)
        tube("hardware", 0.08, (x, y, 0.0), (x, y, h))
        x += c["railing_post_pitch_m"]

    # Contact pad under the fin at the closed pose: a grey block on the inner face at the axis station.
    pad = c["pad_length_m"], c["pad_width_m"], c["pad_height_m"]
    x_pad, y_pad = c["pad_x_m"], c["inner_face_y_m"] + c["pad_proud_m"] - pad[1] / 2.0
    z_pad = -c["pad_top_depth_m"] - pad[2] / 2.0
    ms.append(_member("pad", "rect", pad[1], (x_pad - pad[0] / 2.0, y_pad, z_pad),
                      (x_pad + pad[0] / 2.0, y_pad, z_pad), pad[2]))

    # Telescopic pushers: long cylinders on the deck at the tip [S NSF 2025-08], in two stages.
    x0, x1 = c["pusher_start_x_m"], c["pusher_end_x_m"]
    outer_line = plan_line(c, y_out + 0.2)
    for dy in (0.0, c["pusher_spacing_m"]):
        y0 = _at(outer_line, c["tip_kink_x_m"]) + dy
        z = c["pusher_size_m"] / 2.0
        tube("hardware", c["pusher_size_m"], (x0, y0, z), (0.5 * (x0 + x1), y0, z))
        tube("hardware", c["pusher_size_m"] * 0.6, (0.5 * (x0 + x1), y0, z), (x1, y0, z))

    # Stabilizer arm: a hanging probe at the lower chord's end and its long thin brace [S NSF 2025-08 "stabilizer arms"].
    x_probe = x_end - 0.4
    z_probe = lower_chord_z(c, x_probe)
    probe_foot = (x_probe, y_mid, z_probe - c["probe_length_m"])
    tube("truss", c["probe_size_m"], (x_probe, y_mid, z_probe), probe_foot)
    tube("truss", thin, (c["probe_brace_frac"] * length, y_mid, lower_chord_z(c, c["probe_brace_frac"] * length)),
         probe_foot)
    ms.append(_member("hardware", "rect", 0.5, (x_probe, y_mid, probe_foot[2] - 0.6), (x_probe, y_mid, probe_foot[2]),
                      0.5))

    # Two ladders between the chords and a small inspection platform (they read in every close view).
    for frac in c["ladder_a_frac"], c["ladder_b_frac"]:
        x = frac * length
        top = (x, y_out - 0.3, z_top)
        bottom = (x + 0.9, y_mid - 0.9, lower_chord_z(c, x + 0.9))
        for side in (-0.25, 0.25):
            tube("hardware", 0.07, (top[0] + side, top[1], top[2]), (bottom[0] + side, bottom[1], bottom[2]))
        for k in range(1, 8):
            t = k / 8.0
            p = [top[i] + (bottom[i] - top[i]) * t for i in range(3)]
            tube("hardware", 0.05, (p[0] - 0.25, p[1], p[2]), (p[0] + 0.25, p[1], p[2]))
    x_p = c["platform_frac"] * length
    ms.append(_member("hardware", "rect", 1.3, (x_p - 1.3, y_out - 1.1, -2.6), (x_p + 1.3, y_out - 1.1, -2.6), 0.1))
    tube("hardware", 0.08, (x_p - 1.3, y_out - 1.7, -1.5), (x_p + 1.3, y_out - 1.7, -1.5))
    tube("hardware", 0.35, (x_p, y_out - 1.1, -2.6), (x_p, y_out - 1.1, -4.4))  # the grey actuator under it
    tube("hardware", 0.10, (c["mast_x_m"], y_rail, 0.0), (c["mast_x_m"], y_rail, c["mast_height_m"]))
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
    # Hinge frame at each corner: a box out to the hinge line and the grey actuator drum beside the arm's root.
    for sign in (-1.0, 1.0):
        y = sign * k["hinge_y_m"]
        beam("frame", (x_face, y, -box / 2.0), (0.0, y, -box / 2.0), box * 1.2)
        beam("frame", (x_face, y, bottom + box / 2.0), (0.0, y, -k["hinge_lower_m"]), box * 0.8)
        ms.append(_member("hardware", "tube", k["drum_small_m"],
                          (k["actuator_x_m"], y - sign * k["actuator_inset_m"], -k["actuator_z_m"] - 1.2),
                          (k["actuator_x_m"], y - sign * k["actuator_inset_m"], -k["actuator_z_m"] + 1.2)))
    # Platforms: the walkway beam across the face reaching past the corner, with railing.
    p_end = half + k["platform_reach_m"]
    ms.append(_member("frame", "rect", k["platform_width_m"], (x_face, -p_end, top + 0.1), (x_face, p_end, top + 0.1),
                      0.2))
    for sign in (-1.0, 1.0):
        y0 = sign * (k["width_m"] / 2.0 + k["platform_reach_m"])
        ms.append(_member("hardware", "tube", 0.08, (x_face + k["platform_width_m"] / 2.0, y0, top + 1.2),
                          (x_face + k["platform_width_m"] / 2.0, 0.0, top + 1.2)))
    # The drum / sheave block above the carriage with the hoist cables (carriage photo 2025-12: worker for scale).
    d, w, z_d = k["drum_diameter_m"], k["drum_width_m"], top + k["drum_diameter_m"] / 2.0 + 0.6
    ms.append(_member("frame", "tube", d, (x_face, -w / 2.0, z_d), (x_face, w / 2.0, z_d)))
    beam("frame", (x_face, 0.0, top), (x_face, 0.0, z_d), chord)
    return ms


def carriage_drum_top_z(detail: dict) -> float:
    k = detail["carriage"]
    return k["top_z_m"] + 0.6 + k["drum_diameter_m"]


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
    box("hardware", (x_step * 0.2, 0.0, rise * 0.45 - 1.2), (x_step * 0.9, 0.0, rise * 0.1 - 1.2), half_root * 1.4, 2.4)
    # Tip part: a shallow lattice platform, deck on top, stair up to it, connector head at the end.
    for side in (-1.0, 1.0):
        y = side * half_tip
        tube("truss", brace, (x_step, y, 0.0), (reach, y, 0.0))
        tube("truss", brace, (x_step, y, low_tip), (reach, y, low_tip))
        tube("truss", brace, (reach, y, 0.0), (reach, y, low_tip))
        tube("truss", brace, (x_step, y, low_tip), (reach, y, 0.0))
    box("truss", (x_step, 0.0, low_tip - 0.05), (reach, 0.0, low_tip - 0.05), 2.0 * half_tip, 0.1)
    box("hardware", (x_step, 0.0, -0.05), (reach, 0.0, -0.05), 2.0 * half_tip, 0.1)
    box("hardware", (0.0, 0.0, rise), (x_step, 0.0, 0.0), 2.0 * half_root, 0.1)
    head_x0 = reach - q["head_length_m"]
    box("hardware", (head_x0, 0.0, q["head_z_m"] - 0.6), (reach, 0.0, q["head_z_m"] - 0.6), q["head_width_m"], 1.2)
    tube("hardware", 0.5, (head_x0 - 1.0, 0.0, 0.25), (head_x0, 0.0, q["head_z_m"] - 0.6))
    for side in (-1.0, 1.0):
        tube("hardware", 0.08, (0.0, side * half_root, rise + 1.1), (x_step, side * half_root, 1.1))
        tube("hardware", 0.08, (x_step, side * half_tip, 1.1), (reach, side * half_tip, 1.1))
    return ms
