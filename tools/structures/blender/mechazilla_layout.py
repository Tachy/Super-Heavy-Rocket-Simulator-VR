"""Pure layout of the Mechazilla modules: chopstick, carriage, QD arm as lists of members, no Blender.

The generators (gen_chopstick.py, gen_carriage.py, gen_qd_arm.py) add the members to meshes; pytest reads the same lists
for the clearance and parallel bearing cases (Blender's Python 3.10 and the tools' Python both import this file).

A member is (node, kind, size_m, p0, p1, size_y_m) in the model frame; size_y_m only for 'rect'. Node names are the glTF
mesh nodes, one material each.
Chopstick frame: the common hinge axis at the origin, the bearing piece along +X, +Y toward the catch axis (the game mirrors
the -across arm), strip top Z = 0 (the plane the catch pins land on). The corner lies off the catch line, so the two arms'
roots differ: `chopstick(detail, side)`, everything beyond the root is the same at `arm_line_offset`.
Carriage frame (also the rails'): X toward the pad, Y = -across, Z up, origin on the common hinge axis (outside the tower box's
corner toward the mount, physics `[site.tower]`) in the deck plane (carriage offset 0).
QD arm frame: X along the arm toward the catch axis, Y left of it, Z up, origin on the swing axis at the tip deck's height.
"""
from __future__ import annotations

import math

# Member map of one arm, from the photos: launch-structures-measurements.md "Arm member map". Large tubes only.
ARM_MEMBERS = {"top": 5, "caps": 3, "lower_chord": 2, "root_columns": 3, "ties": 2, "posts": 4,
               "diagonals": 3, "probe": 1}
SUPPLY_LINES = 3  # thin lines that read as supply lines, never as structure
SMALLEST_STRUCTURE_M = 0.4  # the hanging probe; every other member is at least 0.8 m
POST_FRACTIONS = (0.25, 0.36, 0.575, 0.82)  # [E] +-0.03 stations of the 4 posts, teslaoracle 2025-03 far arm


def _member(node, kind, size, p0, p1, size_y=None):
    return (node, kind, size, tuple(p0), tuple(p1), size_y)


def pivot_site(t: dict) -> tuple:
    """The common hinge axis in (along the pad -> tower line, across): `hinge_outside_corner_m` outside the physics box's corner
    toward the mount, on its outward diagonal, as core TowerGeometry::hinge_pivot. `t` is a `[site.tower]` table."""
    yaw = math.radians(t["plan_yaw_deg"])
    c, s = math.cos(yaw), math.sin(yaw)
    reach = 0.5 * t["width_m"] + t["hinge_outside_corner_m"] / math.sqrt(2.0)
    return (t["distance_m"] - reach * (c + s), -reach * (c - s))


def arm_line_offset(t: dict, side: float) -> float:
    """From the axis outward to the arm's centre line, arm `side` (+1 +across, -1 -across), as core arm_line_offset_m."""
    return 0.5 * t["arm_closed_gap_m"] + 0.5 * t["arm_width_m"] - side * pivot_site(t)[1]


def _line_y(detail: dict, side: float) -> float:
    return -arm_line_offset(detail["site_tower"], side)


def bearing_tube_y(c: dict) -> float:
    """Axis of the bearing tube against the arm's line: the flat plate on its inner side has its face on `inner_face_y_m`."""
    return c["inner_face_y_m"] - c["plate_thickness_m"] - 0.5 * c["bearing_tube_m"]


def top_plan(detail: dict, side: float = 1.0) -> list:
    """Plan polyline (x, y) of the U's outer top tube: axis, bend, end of the leg, connector's end on the bearing tube."""
    c = detail["chopstick"]
    y_line = _line_y(detail, side)
    y_b = y_line + bearing_tube_y(c)
    return [(0.0, 0.0), (c["root_kink_x_m"], y_line - c["bend_out_m"]), (c["leg_end_x_m"], y_b - c["leg_offset_m"]),
            (c["connector_end_x_m"], y_b)]


def lower_chord_z(c: dict, x: float) -> float:
    """Lower chord axis: rises from the root's foot to the start of the bearing piece, then gently to its tip end."""
    pts = [(0.0, -c["foot_depth_m"]), (c["bearing_start_x_m"], -c["depth_m"]),
           (c["lower_end_frac"] * c["length_m"], -c["end_depth_m"])]
    for (x0, z0), (x1, z1) in zip(pts, pts[1:]):
        if x <= x1:
            return z0 + (z1 - z0) * (x - x0) / (x1 - x0)
    return pts[-1][1]


def _on_leg(plan: list, x: float) -> float:
    (x0, y0), (x1, y1) = plan[1], plan[2]
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


def chopstick(detail: dict, side: float = 1.0) -> list:
    c, prof = detail["chopstick"], detail["profiles"]
    chord, brace, thin = (prof[c["chord_profile"]]["size_m"], prof[c["brace_profile"]]["size_m"],
                          prof[c["thin_profile"]]["size_m"])
    length, bearing = c["length_m"], c["bearing_tube_m"]
    y_line = _line_y(detail, side)
    y_b, x_b = y_line + bearing_tube_y(c), c["bearing_start_x_m"]
    z_top = -0.5 * bearing - c["strip_thickness_m"] + 0.05  # the strip lies on the bearing tube (0.05 m weld), top Z = 0
    plan = top_plan(detail, side)
    x_end = c["lower_end_frac"] * length
    ms = []

    def tube(node, size, p0, p1):
        ms.append(_member(node, "tube", size, p0, p1))

    def inner_y(x):  # the truss plane in plan: from the axis along the inner root tube, then the bearing tube
        return y_b * min(x, x_b) / x_b

    def top(x, y):
        return (x, y, z_top)

    def low(x):
        return (x, inner_y(x), lower_chord_z(c, x))

    # The U in plan: root out from the common axis, leg back in, connector to the bearing tube; inner root tube to the bearing
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

    # One truss plane under the inner root and bearing line: lower chord from the foot (side view a long triangle), 4 posts,
    # 3 diagonals.
    tube("truss", chord, low(0.0), low(x_b))
    tube("truss", chord, low(x_b), low(x_end))
    tube("truss", chord * c["cap_scale"], low(x_end - 0.1), low(x_end + 0.1))
    posts = [f * length for f in POST_FRACTIONS]
    for x in posts:
        tube("truss", brace, top(x, inner_y(x)), low(x))
    for xa, xb in zip(posts, posts[1:]):
        tube("truss", brace, top(xa, inner_y(xa)), low(xb))
    for x, y_outer in ((c["root_kink_x_m"], plan[1][1]), (x_b, _on_leg(plan, x_b))):  # the two ties across the top
        tube("truss", brace, top(x, y_outer), top(x, inner_y(x)))

    # Root: three thick columns from the top to one foot on the common axis (the axis itself is the carriage's hinge column).
    foot = (0.0, 0.0, -c["foot_depth_m"])
    x_r = c["root_kink_x_m"]
    tube("truss", chord, top(*plan[1]), foot)
    tube("truss", chord, top(x_r, 0.5 * (plan[1][1] + inner_y(x_r))), foot)
    tube("truss", chord, top(x_r, inner_y(x_r)), foot)

    # Flat bearing plate on the inner side of the bearing tube, tan strip on top of it, contact pad on the plate.
    y_plate = y_line + c["inner_face_y_m"] - 0.5 * c["plate_thickness_m"]
    z_plate = -c["strip_thickness_m"] - 0.5 * c["plate_height_m"]
    ms.append(_member("bearing", "rect", c["plate_thickness_m"], (x_b + c["plate_inset_m"], y_plate, z_plate),
                      (c["plate_end_x_m"], y_plate, z_plate), c["plate_height_m"]))
    z_strip = -0.5 * c["strip_thickness_m"]
    ms.append(_member("strip", "rect", c["strip_width_m"], (x_b, y_b, z_strip), (c["plate_end_x_m"], y_b, z_strip),
                      c["strip_thickness_m"]))
    pad = c["pad_length_m"], c["pad_width_m"], c["pad_height_m"]
    x_pad, y_pad = c["pad_x_m"], y_line + c["inner_face_y_m"] + c["pad_proud_m"] - pad[1] / 2.0
    z_pad = -c["pad_top_depth_m"] - pad[2] / 2.0
    ms.append(_member("pad", "rect", pad[1], (x_pad - pad[0] / 2.0, y_pad, z_pad),
                      (x_pad + pad[0] / 2.0, y_pad, z_pad), pad[2]))

    # Closed box on the tip, on the bearing tube's line, its top on the arm's plane (crop_rail_colleen).
    box = c["tip_box_m"]
    ms.append(_member("fittings", "rect", box, (c["tip_box_start_x_m"], c["tip_box_y_m"] + y_line, -0.5 * box),
                      (c["tip_box_end_x_m"], c["tip_box_y_m"] + y_line, -0.5 * box), box))

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
    tube("hardware", d, (x_b, y_b - 0.6, lower_chord_z(c, x_b) + 0.3), (x_end, y_b - 0.6, -c["end_depth_m"] + 0.3))

    # Two ladders from the leg to the lower chord as one flat black panel each, a platform on the leg's outer side, a mast.
    for frac in c["ladder_a_frac"], c["ladder_b_frac"]:
        x = frac * length
        ms.append(_member("fittings", "rect", 0.5, (x, _on_leg(plan, x), z_top), low(x + 0.9), 0.04))
    x_p = c["platform_frac"] * length
    y_p = _on_leg(plan, x_p) - 1.1
    ms.append(_member("fittings", "rect", 1.3, (x_p - 1.3, y_p, -2.6), (x_p + 1.3, y_p, -2.6), 0.1))
    for dx in (-1.3, 1.3):  # the platform hangs from the leg on two braces (it is dark steel like the leg, never a loose plate)
        tube("fittings", thin, (x_p + dx, _on_leg(plan, x_p + dx), z_top), (x_p + dx, y_p - 0.65, -2.6))
    tube("fittings", d, (x_p - 1.3, y_p - 0.65, -1.5), (x_p + 1.3, y_p - 0.65, -1.5))
    x_m = c["mast_x_m"]
    tube("fittings", c["mast_size_m"], (x_m, _on_leg(plan, x_m), 0.0), (x_m, _on_leg(plan, x_m), c["mast_height_m"]))
    return ms


def _unit_axes(detail: dict):
    """Box axes of the tower in the carriage frame (X, Y): ex along the face from the corner toward the mount, ey along the other."""
    yaw = math.radians(detail["site_tower"]["plan_yaw_deg"])
    c, s = math.cos(yaw), math.sin(yaw)
    return (-c, s), (-s, -c)


def tower_corner(detail: dict) -> dict:
    """The box corner toward the mount (the frame's origin is the hinge axis outside it), the face axes and the face width W (physics box)."""
    ex, ey = _unit_axes(detail)
    back = detail["site_tower"]["hinge_outside_corner_m"] / math.sqrt(2.0)
    return {"c": (back * (ex[0] + ey[0]), back * (ex[1] + ey[1])), "ex": ex, "ey": ey, "w": detail["site_tower"]["width_m"]}


def rail_points(detail: dict) -> dict:
    """Axes (X, Y) of the three rails on the faces, `rail_inboard_m` inboard of the corner edge: the hinge corner and neighbour `b`
    on the pad-side face (ey, both rails of crop_rail_colleen), neighbour `a` on the other side face (ex). The rear corner has none."""
    g = tower_corner(detail)
    (cx, cy), ex, ey, w = g["c"], g["ex"], g["ey"], g["w"]
    d = detail["carriage"]["rail_inboard_m"]
    return {"corner": (cx + d * ey[0], cy + d * ey[1]), "a": (cx + (w - d) * ex[0], cy + (w - d) * ex[1]),
            "b": (cx + (w - d) * ey[0], cy + (w - d) * ey[1])}


def rollers(detail: dict) -> list:
    """The carriage's 6 roller points (X, Y, Z, rail key): upper and lower level on the three rails."""
    z_up, z_lo = collar_levels(detail)
    return [(p[0], p[1], z, key) for z in (z_up, z_lo) for key, p in rail_points(detail).items()]


def collar_levels(detail: dict) -> tuple:
    """Chord heights (Z) of the upper and the lower collar."""
    k = detail["carriage"]
    return k["top_z_m"] - 0.5 * k["collar_chord_m"], k["top_z_m"] - k["collar_gap_m"]


def cable_spread_axis(detail: dict) -> tuple:
    """Plan unit vector (X, Y) from one hoist cable group to the other: across the corner's outward diagonal."""
    ex, ey = _unit_axes(detail)
    return ((ey[0] - ex[0]) / math.sqrt(2.0), (ey[1] - ex[1]) / math.sqrt(2.0))


def hoist_cables(detail: dict) -> list:
    """Plan points (X, Y) of the vertical hoist cables: two groups either side of the sheave block, the load centred on the axis.
    Inside a group the cables stand along the outward diagonal."""
    k = detail["carriage"]
    t = cable_spread_axis(detail)
    n = (-t[1], t[0])
    per_group = round(k["cable_count"] / 2)
    out = []
    for sign in (-1.0, 1.0):
        for i in range(per_group):
            lateral = sign * 0.5 * k["cable_group_spacing_m"]
            along = (i - 0.5 * (per_group - 1)) * k["cable_spacing_m"]
            out.append((lateral * t[0] + along * n[0], lateral * t[1] + along * n[1]))
    return out


def rails(detail: dict) -> list:
    """Static rails of the carriage: three vertical boxes from the plinth top to the lattice top (carriage frame, offset 0)."""
    k, t = detail["carriage"], detail["site_tower"]
    z0 = detail["tower"]["plinth"]["height_m"] - t["catch_height_m"]
    z1 = detail["tower"]["lattice_height_m"] - t["catch_height_m"]
    return [_member("rail", "rect", k["rail_size_m"], (x, y, z0), (x, y, z1), k["rail_size_m"])
            for x, y in rail_points(detail).values()]


def carriage_faces(detail: dict) -> list:
    """(from corner, to neighbour corner, face direction, other axis) of the two side faces the collars wrap."""
    g = tower_corner(detail)
    (cx, cy), ex, ey, w = g["c"], g["ex"], g["ey"], g["w"]
    return [(g["c"], (cx + w * ex[0], cy + w * ex[1]), ex, ey), (g["c"], (cx + w * ey[0], cy + w * ey[1]), ey, ex)]


def carriage(detail: dict) -> list:
    k, prof = detail["carriage"], detail["profiles"]
    chord = prof[detail["chopstick"]["chord_profile"]]["size_m"]
    g, rp = tower_corner(detail), rail_points(detail)
    ex, ey, w = g["ex"], g["ey"], g["w"]
    z_up, z_lo = collar_levels(detail)
    top, cc = k["top_z_m"], k["collar_chord_m"]
    faces = carriage_faces(detail)
    ms = []

    def rect(node, p0, p1, size, size_y=None):
        ms.append(_member(node, "rect", size, p0, p1, size if size_y is None else size_y))

    def tube(node, size, p0, p1):
        ms.append(_member(node, "tube", size, p0, p1))

    def at(xy, z):
        return (xy[0], xy[1], z)

    # Two collars, each wrapping the corner and both side faces, from the corner rail to the neighbour rails.
    for z in (z_up, z_lo):
        for p0, p1, _, _ in faces:
            rect("collar", at(p0, z), at(p1, z), cc)
    # The lower collar is a truss (second chord and diagonals).
    z_hi = z_lo + k["truss_depth_m"]
    n = max(2, round(w / k["truss_panel_m"]))
    for p0, p1, _, _ in faces:
        rect("frame", at(p0, z_hi), at(p1, z_hi), 0.8 * cc)
        for i in range(n):
            a = tuple(p0[j] + (p1[j] - p0[j]) * i / n for j in range(2))
            b = tuple(p0[j] + (p1[j] - p0[j]) * (i + 1) / n for j in range(2))
            lo, hi = (a, b) if i % 2 == 0 else (b, a)
            tube("frame", 0.3, at(lo, z_lo), at(hi, z_hi))
    # Horizontal struts at both collar levels from the axis back to the corner and a little along both faces.
    c0 = g["c"]
    reach = k["rail_inboard_m"]
    for z in (z_up, z_lo):
        tube("frame", chord, (0.0, 0.0, z), at(c0, z))
        for u in (ex, ey):
            tube("frame", 0.6 * chord, (0.0, 0.0, z), at((c0[0] + reach * u[0], c0[1] + reach * u[1]), z))
    # The rotary hinge: one column on the common axis, from the arms' foot to the top; the 6 rollers on the rails.
    tube("frame", k["hinge_column_m"], (0.0, 0.0, -detail["chopstick"]["foot_depth_m"]), (0.0, 0.0, top))
    for x, y, z, _ in rollers(detail):
        tube("frame", k["roller_size_m"], (x, y, z - 0.5 * k["roller_size_m"]), (x, y, z + 0.5 * k["roller_size_m"]))
    # Walkway on the upper collar, outside the face, as dark steel (crop_rail_colleen: grating deck, railing, knee braces down to the
    # face), closed round the corner: each deck runs one walkway width past the corner rail so the two overlap there.
    width = k["platform_width_m"]
    z_deck = z_up + 0.5 * cc + 0.05
    for p0, p1, _, other in faces:
        run = math.dist(p0, p1)
        u = ((p1[0] - p0[0]) / run, (p1[1] - p0[1]) / run)
        start = (p0[0] - width * u[0], p0[1] - width * u[1])  # past the corner, round it
        off = lambda q, d: (q[0] - d * other[0], q[1] - d * other[1])  # noqa: E731
        d_mid, d_edge = 0.5 * width, width
        rect("frame", at(off(start, d_mid), z_deck), at(off(p1, d_mid), z_deck), width, 0.1)
        rect("frame", at(off(start, d_edge), z_up), at(off(p1, d_edge), z_up), 0.25, cc)  # outer edge beam
        n_post = max(2, round((run + width) / k["rail_post_pitch_m"]))
        for i in range(n_post + 1):
            f = i / n_post
            q = (start[0] + (p1[0] - start[0]) * f, start[1] + (p1[1] - start[1]) * f)
            tube("frame", 0.08, at(off(q, d_edge), z_deck), at(off(q, d_edge), z_deck + k["rail_height_m"]))
            tube("frame", 0.2, at(off(q, d_edge), z_up), at(q, z_up - k["brace_drop_m"]))  # knee brace to the face
        for h in (k["rail_height_m"], 0.5 * k["rail_height_m"]):
            tube("frame", 0.08, at(off(start, d_edge), z_deck + h), at(off(p1, d_edge), z_deck + h))
    # White actuator drum at the hinge, one per arm, both beside the axis (the arm's rod pulls from its anchor, `actuator_*`).
    for sign in (-1.0, 1.0):
        y = sign * k["actuator_anchor_y_m"]
        tube("hardware", k["actuator_barrel_m"] * 3.0, (k["actuator_anchor_x_m"] - 1.2, y, -k["actuator_anchor_depth_m"]),
             (k["actuator_anchor_x_m"] + 0.4, y, -k["actuator_anchor_depth_m"]))
    # The sheave block above the hinge, on the common axis: the two hoist cable groups come straight down either side of it
    # onto a crossbeam on the hinge's top (carriage photo 2025-12: worker for scale; crop_rail_colleen; statics: user 2026-10-31).
    d, wd = k["drum_diameter_m"], k["drum_width_m"]
    tx, ty = cable_spread_axis(detail)
    z_d = top + k["drum_gap_m"] + d / 2.0
    tube("frame", d, (-0.5 * wd * tx, -0.5 * wd * ty, z_d), (0.5 * wd * tx, 0.5 * wd * ty, z_d))
    tube("frame", 0.6 * chord, (0.0, 0.0, top), (0.0, 0.0, z_d))
    tube("frame", chord, at(c0, z_up), (0.0, 0.0, top))
    beam = 0.5 * k["cable_group_spacing_m"] + 0.3
    rect("frame", (-beam * tx, -beam * ty, top), (beam * tx, beam * ty, top), 0.6, 0.4)
    return ms


HULL_RADIUS_M = 4.5  # [S] 9 m booster diameter


def qd_geometry(detail: dict) -> dict:
    """The ship QD arm on the tower's rear side in the site frame (X toward the pad, Y across the line): the swing axis
    `root`, the heading to the catch axis, the reach to the head's face, the tower corner the bearing bracket stands on
    (`corner_model`, in the arm frame)."""
    t, q = detail["site_tower"], detail["qd_arm"]
    yaw = math.radians(t["plan_yaw_deg"])
    c, s = math.cos(yaw), math.sin(yaw)
    ex, ey = (c, -s), (s, c)  # box axes in (toward, across)
    half, w = 0.5 * t["width_m"], t["width_m"]
    corner0 = (t["distance_m"] - half * (ex[0] + ey[0]), -half * (ex[1] + ey[1]))
    corner = (corner0[0] + w * ey[0], corner0[1] + w * ey[1])  # the neighbour corner on the rear side
    bx, by = q["root_x_m"], w + q["root_beyond_m"]
    root = (corner0[0] + bx * ex[0] + by * ey[0], corner0[1] + bx * ex[1] + by * ey[1])
    dist = math.hypot(*root)
    heading = (-root[0] / dist, -root[1] / dist)
    left = (-heading[1], heading[0])
    rel = (corner[0] - root[0], corner[1] - root[1])
    return {"root": root, "heading": heading, "left": left, "distance_m": dist, "reach_m": dist - HULL_RADIUS_M - q["head_gap_m"],
            "corner_model": (rel[0] * heading[0] + rel[1] * heading[1], rel[0] * left[0] + rel[1] * left[1])}


def qd_swung(x: float, y: float, swing_rad: float) -> tuple:
    """Plan point of the arm frame after a swing: positive turns the head away from the catch line (to the right, -Y)."""
    c, s = math.cos(swing_rad), math.sin(swing_rad)
    return (x * c + y * s, -x * s + y * c)


def qd_arm(detail: dict, swing_rad: float = 0.0) -> list:
    q, prof = detail["qd_arm"], detail["profiles"]
    chord, brace = prof[detail["chopstick"]["chord_profile"]]["size_m"] * 0.7, prof["chopstick_brace"]["size_m"] * 0.6
    geo = qd_geometry(detail)
    reach, x_step = geo["reach_m"], q["root_length_m"]
    half_root, half_tip = q["root_width_m"] / 2.0, q["tip_width_m"] / 2.0
    rise, low_tip, low_root, foot = q["root_rise_m"], -q["tip_depth_m"], -q["root_depth_m"], -q["foot_depth_m"]
    moving, fixed = [], []

    def tube(node, size, p0, p1):
        moving.append(_member(node, "tube", size, p0, p1))

    def box(node, a, b, w, h):
        moving.append(_member(node, "rect", w, a, b, h))

    n = max(2, round(x_step / q["panel_m"]))
    for side in (-1.0, 1.0):
        y = side * half_root
        tube("truss", chord, (0.0, y, rise), (x_step, y, 0.0))  # top chord falling to the tip part
        tube("truss", chord, (0.0, y, low_root), (x_step, y, low_tip))  # lower chord
        tube("truss", chord, (0.0, y, low_root), (0.0, y, foot))  # column at the swing bearing
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
    for side in (-1.0, 1.0):
        box("hardware", (x_step * 0.25, side * half_root * 0.5, rise * 0.4 - 1.2),
            (x_step * 0.5, side * half_root * 0.5, rise * 0.25 - 1.2), 0.8, 2.4)
    # Swing bearing: a round drum on the axis under the root.
    tube("hardware", q["root_width_m"] * 0.8, (0.0, 0.0, low_root - 0.5), (0.0, 0.0, low_root))
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
    # Fixed bracket from the swing bearing to the tower's rear corner (does not swing): beams at the top and the foot of the root.
    cx, cy = geo["corner_model"]
    for z in (rise, low_root):
        fixed.append(_member("bracket", "rect", 0.8, (0.0, 0.0, z), (cx, cy, z), 0.8))
    fixed.append(_member("bracket", "tube", 1.0, (0.0, 0.0, low_root - 0.5), (0.0, 0.0, rise + 0.5)))
    swung = [(m[0], m[1], m[2], (*qd_swung(m[3][0], m[3][1], swing_rad), m[3][2]),
              (*qd_swung(m[4][0], m[4][1], swing_rad), m[4][2]), m[5]) for m in moving]
    return swung + fixed
