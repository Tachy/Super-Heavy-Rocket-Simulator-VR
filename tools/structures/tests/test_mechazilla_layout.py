"""Chopstick / QD arm layout against the physics: the U with parallel bearing pieces, hull clearance at the closed pose."""
from __future__ import annotations

import math
import sys
import tomllib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools" / "structures" / "blender"))
sys.path.insert(0, str(ROOT / "tools" / "structures"))
import check_envelope  # noqa: E402
import detail_file  # noqa: E402
import mechazilla_layout as layout  # noqa: E402

HULL_RADIUS_M = 4.5  # [S] 9 m booster diameter
HINGE_OUTSIDE_M = 2.0  # the axis outside the corner, on the outward diagonal
_REACH_M = 6.65 + HINGE_OUTSIDE_M / math.sqrt(2.0)
HINGE_TO_AXIS_M = 31.7 - _REACH_M * (math.cos(math.radians(37.4)) + math.sin(math.radians(37.4)))  # the axis along the line, hand-computed
HINGE_ACROSS_M = -_REACH_M * (math.cos(math.radians(37.4)) - math.sin(math.radians(37.4)))  # the axis across the line, hand-computed


@pytest.fixture(scope="module")
def detail():
    return detail_file.load(ROOT / "config" / "structures" / "starbase_olp2.toml")


def _physics():
    with open(ROOT / "config" / "launch_sites.toml", "rb") as f:
        return check_envelope.tower(tomllib.load(f))


def _closed_world(phys, side, point, opening_rad=0.0):
    """Arm-frame plan point (x along the arm, y toward the axis) in the frame (from the pivot's along-line station toward the
    pad, across). Built like core TowerGeometry: a rigid turn of the closed arm about the common axis; the layout's +Y points to
    the axis, the arm at `side` = -1 is its mirror."""
    x, y = point
    cos_o, sin_o = math.cos(opening_rad), math.sin(opening_rad)
    return (x * cos_o + y * sin_o, HINGE_ACROSS_M + side * (x * sin_o - y * cos_o))


def _bearing_faces(detail, side=1.0):
    """The flat bearing plates: (x0, y) and (x1, y) of their faces toward the axis."""
    plates = [m for m in layout.chopstick(detail, side) if m[0] == "bearing"]
    assert len(plates) == 1
    _, _, thick, p0, p1, _ = plates[0]
    return [(p[0], p[1] + 0.5 * thick) for p in (p0, p1)]


def _heading_deg(a, b):
    return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))


def test_pivot_stands_outside_the_physics_corner():
    assert _physics()["hinge_outside_corner_m"] == HINGE_OUTSIDE_M
    along, across = layout.pivot_site(_physics())
    assert along == pytest.approx(20.396, abs=0.001) and across == pytest.approx(-1.508, abs=0.001)
    assert along == pytest.approx(HINGE_TO_AXIS_M, abs=0.001)
    assert across == pytest.approx(HINGE_ACROSS_M, abs=0.001)


def test_bearing_pieces_parallel_and_on_the_gap_at_the_closed_pose(detail):
    phys = _physics()
    plus = [_closed_world(phys, 1.0, p) for p in _bearing_faces(detail, 1.0)]
    minus = [_closed_world(phys, -1.0, p) for p in _bearing_faces(detail, -1.0)]
    assert abs(_heading_deg(*plus)) <= 0.25 and abs(_heading_deg(*minus)) <= 0.25  # parallel, along the line tower -> axis
    for a, b in zip(plus, minus):
        assert a[1] - b[1] == pytest.approx(phys["arm_closed_gap_m"], abs=0.05)
        assert a[1] + b[1] == pytest.approx(0.0, abs=1e-9)  # centred on the catch axis


def test_bearing_pieces_span_the_catch_axis_station(detail):
    face = _bearing_faces(detail)
    assert face[0][0] < HINGE_TO_AXIS_M - 5.0 and face[1][0] > HINGE_TO_AXIS_M + 0.5


def test_opening_turns_the_bearing_pieces_apart(detail):
    phys = _physics()
    opening = math.radians(phys["arm_standby_deg"])
    for side in (1.0, -1.0):
        ends = [_closed_world(phys, side, p, opening) for p in _bearing_faces(detail, side)]
        assert side * _heading_deg(*ends) == pytest.approx(phys["arm_standby_deg"], abs=0.25)
        station = (HINGE_TO_AXIS_M - ends[0][0]) / (ends[1][0] - ends[0][0])
        assert side * (ends[0][1] + station * (ends[1][1] - ends[0][1])) - HULL_RADIUS_M > 2.5  # clear of the hull at standby


def test_plan_is_a_u_root_out_leg_back_in(detail):
    c = detail["chopstick"]
    root, bend, leg_end, connector_end = layout.top_plan(detail)
    assert root == (0.0, 0.0)  # the common axis
    assert bend[1] < 0.0 and leg_end[1] < 0.0  # both outward of the axis, +Y is toward the catch axis
    assert bend[1] < leg_end[1]  # the leg runs back in
    assert connector_end[1] == pytest.approx(layout.bearing_tube_y(c) - layout.arm_line_offset(detail["site_tower"], 1.0))  # the connector lands on the bearing tube
    assert 3.0 < _heading_deg(leg_end, bend) + 180.0 < 10.0 or 3.0 < abs(_heading_deg(bend, leg_end)) < 10.0


def test_members_match_the_table_large_tubes_only(detail):
    ms = layout.chopstick(detail)
    truss = [m for m in ms if m[0] == "truss"]
    assert len(truss) == sum(layout.ARM_MEMBERS.values())
    assert all(m[1] == "tube" for m in truss)
    assert all(m[2] >= layout.SMALLEST_STRUCTURE_M for m in truss)  # no small struts: thin things are not structure
    assert all(m[2] >= 0.8 for m in truss if m[2] != detail["chopstick"]["probe_size_m"])
    thin = [m for m in ms if m[0] == "hardware" and m[1] == "tube" and m[2] <= detail["chopstick"]["supply_line_m"]]
    assert len(thin) == layout.SUPPLY_LINES <= 3


def _surface_distance_to_axis(member, side):
    """Least distance from the catch axis of a member's axis (10 cm steps), minus its half size. Closed pose."""
    _, kind, size, p0, p1, size_y = member
    half = size / 2.0
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    plan = math.hypot(dx, dy)
    nx, ny = (-dy / plan, dx / plan) if plan > 1e-6 else (0.0, 1.0)
    best = math.inf
    n = max(1, int(math.dist(p0, p1) / 0.1))
    for i in range(n + 1):
        t = i / n
        x, y = p0[0] + dx * t, p0[1] + dy * t
        for s in (-half, half):  # both edges across the member's plan direction
            best = min(best, math.hypot(x + nx * s - HINGE_TO_AXIS_M, HINGE_ACROSS_M - side * (y + ny * s)))
    return best


def test_closed_arm_clears_the_hull(detail):
    for side in (1.0, -1.0):
        near = [m for m in layout.chopstick(detail, side) if m[0] != "pad"]
        assert min(_surface_distance_to_axis(m, side) for m in near) >= HULL_RADIUS_M + 0.03


def test_pad_stands_at_the_hull_under_the_catch_axis_station(detail):
    c = detail["chopstick"]
    for side in (1.0, -1.0):
        pad = [m for m in layout.chopstick(detail, side) if m[0] == "pad"][0]
        face = pad[3][1] + 0.5 * pad[2]
        assert side * (HINGE_ACROSS_M - side * face) - HULL_RADIUS_M == pytest.approx(0.1, abs=0.02)
    end = c["pad_x_m"] + 0.5 * c["pad_length_m"]
    assert end < c["length_m"]  # the arm's tip is the physics margin beyond the pad
    assert c["pad_x_m"] - 0.5 * c["pad_length_m"] < HINGE_TO_AXIS_M < end  # the pad spans the catch axis station


def test_closed_arm_overshoots_the_axis_with_the_physics_margin_and_lower_chord_rises(detail):
    c = detail["chopstick"]
    assert c["length_m"] == pytest.approx(_physics()["arm_length_m"], abs=0.01)  # drawn arm = physics arm (user 2026-10-31: 3.6 m margin)
    assert 0.0 < c["length_m"] - HINGE_TO_AXIS_M <= 4.0
    assert layout.lower_chord_z(c, 0.0) == pytest.approx(-c["foot_depth_m"])
    assert layout.lower_chord_z(c, c["bearing_start_x_m"]) == pytest.approx(-c["depth_m"])
    assert c["foot_depth_m"] > c["depth_m"] > c["end_depth_m"]


def test_root_columns_and_chord_meet_in_one_foot_on_the_axis(detail):
    c = detail["chopstick"]
    for side in (1.0, -1.0):
        truss = [m for m in layout.chopstick(detail, side) if m[0] == "truss"]
        foot = (0.0, 0.0, -c["foot_depth_m"])
        at_foot = [m for m in truss if math.dist(m[3], foot) < 1e-9 or math.dist(m[4], foot) < 1e-9]
        assert len(at_foot) == layout.ARM_MEMBERS["root_columns"] + 1  # + the lower chord


def test_posts_and_diagonals_in_one_plane(detail):
    c = detail["chopstick"]
    x_b = c["bearing_start_x_m"]
    for side in (1.0, -1.0):
        y_b = layout.top_plan(detail, side)[3][1]
        for m in layout.chopstick(detail, side):
            if m[0] == "truss" and m[2] == detail["profiles"][c["brace_profile"]]["size_m"] and abs(m[3][2] - m[4][2]) > 1.0:
                for p in (m[3], m[4]):
                    assert p[1] == pytest.approx(y_b * min(p[0], x_b) / x_b, abs=1e-9)


def test_open_arm_stays_clear_of_the_tower_box(detail):
    """The root's bend (the arm's point nearest the side faces) at full opening lies outside the box's corner wedge."""
    phys = _physics()
    g = layout.tower_corner(detail)
    for side in (1.0, -1.0):
        bend = layout.top_plan(detail, side)[1]
        p = _closed_world(phys, side, bend, math.radians(phys["arm_open_deg"]))
        rel = (p[0], -(p[1] - HINGE_ACROSS_M))  # carriage frame: X toward the pad, Y = -across, from the corner
        u = rel[0] * g["ex"][0] + rel[1] * g["ex"][1]
        v = rel[0] * g["ey"][0] + rel[1] * g["ey"][1]
        assert min(u, v) < -0.5  # outside at least one face by more than half a tube


def test_open_frame_with_a_narrow_strip_on_the_inner_chord(detail):
    ms = layout.chopstick(detail)
    assert not [m for m in ms if m[0] == "deck"]  # no slab between the chords
    strips = [m for m in ms if m[0] == "strip"]
    assert len(strips) == 1 and strips[0][2] <= 0.8  # one narrow strip on the bearing tube, not a deck
    assert max(max(m[3][2], m[4][2]) + 0.5 * (m[5] or 0.0) for m in strips) == pytest.approx(0.0, abs=1e-9)


def _box_corners_carriage(phys):
    """Independent of the layout: the physics box's corners (toward, across) -> carriage frame (origin on the hinge axis: X = axis station - toward, Y = pivot across - across).
    Returns {"c": corner toward the mount, "a": neighbour along the x axis, "b": neighbour along the y axis}."""
    yaw = math.radians(phys["plan_yaw_deg"])
    ex, ey = (math.cos(yaw), -math.sin(yaw)), (math.sin(yaw), math.cos(yaw))
    half = 0.5 * phys["width_m"]
    centre = (phys["distance_m"], 0.0)
    c = (centre[0] - half * (ex[0] + ey[0]), centre[1] - half * (ex[1] + ey[1]))
    out = {"c": c, "a": (c[0] + 2 * half * ex[0], c[1] + 2 * half * ex[1]), "b": (c[0] + 2 * half * ey[0], c[1] + 2 * half * ey[1])}
    return {k: (HINGE_TO_AXIS_M - v[0], HINGE_ACROSS_M - v[1]) for k, v in out.items()}


def test_rails_stand_inboard_of_three_corners(detail):
    phys = _physics()
    corners = _box_corners_carriage(phys)
    assert math.hypot(*corners["c"]) == pytest.approx(HINGE_OUTSIDE_M, abs=1e-9)  # the frame's origin is the axis outside the corner
    rails = layout.rails(detail)
    assert len(rails) == 3  # the rear corner has no carriage rail
    for member, key in zip(rails, ("c", "a", "b")):
        _, _, size, p0, p1, _ = member
        assert math.dist(p0[:2], p1[:2]) < 1e-9  # vertical
        assert math.dist(p0[:2], corners[key]) == pytest.approx(detail["carriage"]["rail_inboard_m"], abs=1e-9)  # inboard of its corner edge ...
        ex, ey = layout._unit_axes(detail)
        rel = (p0[0] - corners["c"][0], p0[1] - corners["c"][1])
        u, v = rel[0] * ex[0] + rel[1] * ex[1], rel[0] * ey[0] + rel[1] * ey[1]
        assert min(abs(u), abs(v)) < 1e-9 and 0.0 < max(u, v) < phys["width_m"]  # ... on a face of the box
        assert p0[2] < -phys["catch_height_m"] + detail["tower"]["plinth"]["height_m"] + 0.01  # from the plinth top ...
        assert p1[2] > detail["tower"]["lattice_height_m"] - phys["catch_height_m"] - 0.01  # ... to the lattice top


def test_rails_cover_the_carriage_travel(detail):
    phys = _physics()
    low = min(r[2] for r in layout.rollers(detail)) - phys["carriage_stroke_m"]
    high = max(r[2] for r in layout.rollers(detail))
    for r in layout.rails(detail):
        assert r[3][2] <= low and r[4][2] >= high


def test_carriage_has_six_rollers_on_three_corners(detail):
    rails = {k: m for k, m in zip(("corner", "a", "b"), layout.rails(detail))}
    rollers = layout.rollers(detail)
    assert len(rollers) == 6
    z_up, z_lo = layout.collar_levels(detail)
    for z in (z_up, z_lo):
        level = [r for r in rollers if r[2] == z]
        assert sorted(r[3] for r in level) == ["a", "b", "corner"]  # 3 per level, one on each rail
        for x, y, _, key in level:
            assert math.dist((x, y), rails[key][3][:2]) <= 0.3
    # Drawn: a roller housing at each point, within 0.3 m of the rail by construction of the member.
    size = detail["carriage"]["roller_size_m"]
    assert len([m for m in layout.carriage(detail) if m[1] == "tube" and m[2] == size]) == 6


def test_collars_run_from_the_corner_to_both_neighbour_corners(detail):
    phys = _physics()
    corners = _box_corners_carriage(phys)
    collar = [m for m in layout.carriage(detail) if m[0] == "collar"]
    assert len(collar) == 4  # top and bottom, both faces
    z_up, z_lo = layout.collar_levels(detail)
    for z in (z_up, z_lo):
        ends = [m for m in collar if abs(m[3][2] - z) < 1e-9]
        assert len(ends) == 2
        reached = set()
        for m in ends:
            assert m[3][2] == m[4][2]  # horizontal
            assert math.dist(m[3][:2], corners["c"]) <= 0.3  # one end on the corner, struts reach from there to the axis
            for key in ("a", "b"):
                if math.dist(m[4][:2], corners[key]) <= 0.3:
                    reached.add(key)
        assert reached == {"a", "b"}


def test_one_hinge_column_on_the_axis_no_outriggers(detail):
    k = detail["carriage"]
    cols = [m for m in layout.carriage(detail) if m[1] == "tube" and m[2] == k["hinge_column_m"]]
    assert len(cols) == 1
    assert cols[0][3][:2] == (0.0, 0.0) and cols[0][4][:2] == (0.0, 0.0)
    assert cols[0][3][2] == pytest.approx(-detail["chopstick"]["foot_depth_m"]) and cols[0][4][2] == pytest.approx(k["top_z_m"])
    assert "beam_m" not in k and "hinge_y_m" not in k


def test_struts_reach_from_the_axis_back_to_the_corner(detail):
    corner = layout.tower_corner(detail)["c"]
    z_up, z_lo = layout.collar_levels(detail)
    for z in (z_up, z_lo):
        struts = [m for m in layout.carriage(detail) if m[3][:2] == (0.0, 0.0) and m[3][2] == z and m[4][2] == z]
        assert any(math.dist(m[4][:2], corner) < 1e-9 for m in struts)


def test_hoist_cables_in_two_groups_centred_on_the_axis(detail):
    k = detail["carriage"]
    cables = layout.hoist_cables(detail)
    assert len(cables) >= 8
    t = layout.cable_spread_axis(detail)
    lateral = [c[0] * t[0] + c[1] * t[1] for c in cables]
    minus, plus = [v for v in lateral if v < 0.0], [v for v in lateral if v > 0.0]
    assert len(minus) == len(plus) == len(cables) / 2
    spacing = sum(plus) / len(plus) - sum(minus) / len(minus)
    assert abs(spacing - 1.5) <= 0.3
    mid = tuple(sum(c[i] for c in cables) / len(cables) for i in range(2))
    assert math.hypot(*mid) <= 0.3  # the load centred on the axis
    # Either side of the sheave block, which stands on the axis.
    assert 0.5 * k["drum_width_m"] <= min(abs(v) for v in lateral) - 0.5 * k["cable_size_m"]
    drum = [m for m in layout.carriage(detail) if m[1] == "tube" and m[2] == k["drum_diameter_m"]]
    assert len(drum) == 1
    mid = tuple(0.5 * (drum[0][3][i] + drum[0][4][i]) for i in range(2))
    assert math.hypot(*mid) == pytest.approx(0.0, abs=1e-9)


def _qd_site(detail, point, swing=0.0):
    """Arm-frame plan point -> (toward, across) after the swing."""
    geo = layout.qd_geometry(detail)
    x, y = layout.qd_swung(point[0], point[1], swing) if swing else point
    return (geo["root"][0] + x * geo["heading"][0] + y * geo["left"][0], geo["root"][1] + x * geo["heading"][1] + y * geo["left"][1])


def test_qd_arm_head_reaches_the_hull(detail):
    geo = layout.qd_geometry(detail)
    head = [m for m in layout.qd_arm(detail) if m[0] == "hardware" and m[2] == detail["qd_arm"]["head_width_m"]]
    assert len(head) == 1
    tip = _qd_site(detail, head[0][4][:2])
    assert math.hypot(*tip) - HULL_RADIUS_M == pytest.approx(1.0, abs=0.05)  # the head's face is 1 m off the 4.5 m hull
    assert geo["heading"][0] < 0.0  # it points to the catch axis from the tower's rear side
    t = _physics()
    # The root lies on the rear side: farther from the axis than the tower's near corner, beyond the box's rear face.
    assert geo["distance_m"] > HINGE_TO_AXIS_M + 0.5 * t["width_m"]
    # The head stands at the ship's QD station height: the tip deck is the detail file's, 5.5 m below the 20 m stack's.
    assert detail["qd_arm"]["deck_m"] == pytest.approx(86.8 - (20.0 - 14.5), abs=0.01)


def _plan_segments_arm(phys, detail, side, opening):
    ms = [m for m in layout.chopstick(detail, side) if m[0] in ("truss", "bearing", "strip")]
    out = []
    for m in ms:
        a = _closed_world(phys, side, m[3][:2], opening)
        b = _closed_world(phys, side, m[4][:2], opening)
        out.append(((HINGE_TO_AXIS_M - a[0], a[1]), (HINGE_TO_AXIS_M - b[0], b[1]), 0.5 * m[2]))
    return out


def _dist_point_segment(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    t = 0.0 if ax == ay == 0.0 else max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / (ax * ax + ay * ay)))
    return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)


def test_qd_arm_swung_out_clears_the_arms(detail):
    phys = _physics()
    swing = math.radians(90.0)
    moving = [m for m in layout.qd_arm(detail, swing) if m[0] != "bracket"]
    # Seen from above, even at the height of the arms: no member of the swung arm within 1 m of a chopstick member at any
    # opening from the closed pose to fully open, either side.
    points = []
    for m in moving:
        n = max(1, int(math.dist(m[3][:2], m[4][:2]) / 0.5))
        for i in range(n + 1):
            f = i / n
            p = (m[3][0] + (m[4][0] - m[3][0]) * f, m[3][1] + (m[4][1] - m[3][1]) * f)
            points.append((_qd_site(detail, p), 0.5 * m[2]))
    worst = math.inf
    for opening in (0.0, math.radians(phys["arm_standby_deg"]), math.radians(phys["arm_open_deg"])):
        for side in (-1.0, 1.0):
            for a, b, half in _plan_segments_arm(phys, detail, side, opening):
                for p, r in points:
                    worst = min(worst, _dist_point_segment(p, a, b) - half - r)
    assert worst >= 1.0
    # In height: the QD arm's top against the lowest drawn arm / carriage point at every carriage position of the stroke.
    q = detail["qd_arm"]
    qd_top = q["deck_m"] + q["root_rise_m"] + 1.2
    low_carriage = phys["catch_height_m"] - phys["carriage_stroke_m"] + layout.collar_levels(detail)[1] - detail["carriage"]["collar_chord_m"]
    low_arm = phys["catch_height_m"] - phys["carriage_stroke_m"] - detail["chopstick"]["foot_depth_m"]
    assert qd_top + 5.0 < min(low_carriage, low_arm)


def test_every_member_is_inside_its_table(detail):
    for build in (layout.chopstick, layout.carriage, layout.qd_arm, layout.rails):
        members = build(detail)
        assert members
        assert all(m[1] in ("tube", "rect") and math.dist(m[3], m[4]) > 0.0 for m in members)


def test_envelope_passes_and_lists_only_table_rows():
    with open(ROOT / "config" / "launch_sites.toml", "rb") as f:
        sites = tomllib.load(f)
    failures, exceptions = check_envelope.check(detail_file.load(check_envelope.DETAIL), check_envelope.tower(sites))
    assert failures == []
    assert set(exceptions) <= set(check_envelope.ALLOWED)


def _seg_dist(a0, a1, b0, b1):
    """Distance between two 3D segments (sampled finely: members are metres long, the tolerance 0.3 m)."""
    def pts(p0, p1):
        n = max(2, int(math.dist(p0, p1) / 0.05) + 1)
        return [tuple(p0[i] + (p1[i] - p0[i]) * k / (n - 1) for i in range(3)) for k in range(n)]
    pa, pb = pts(a0, a1), pts(b0, b1)
    return min(math.dist(p, q) for p in pa for q in pb)


def test_no_collar_member_floats(detail):
    """Every member of the carriage (collars, walkways, railings, braces) lies within 0.3 m of another member, a rail or a tower face."""
    g = layout.tower_corner(detail)
    ms = layout.carriage(detail) + layout.rails(detail)
    (cx, cy), ex, ey, w = g["c"], g["ex"], g["ey"], g["w"]

    def box_gap(p):  # plan distance of a point to the tower box, 0 inside
        rx, ry = p[0] - cx, p[1] - cy
        u, v = rx * ex[0] + ry * ex[1], rx * ey[0] + ry * ey[1]
        du, dv = max(-u, u - w, 0.0), max(-v, v - w, 0.0)
        return math.hypot(du, dv)

    worst = 0.0
    for i, m in enumerate(ms):
        half = 0.5 * m[2]
        n = max(2, int(math.dist(m[3], m[4]) / 0.1) + 1)
        samples = [tuple(m[3][j] + (m[4][j] - m[3][j]) * k / (n - 1) for j in range(3)) for k in range(n)]
        near_face = min(box_gap(p) for p in samples) - half
        near_member = min(_seg_dist(m[3], m[4], o[3], o[4]) - half - 0.5 * o[2] for j, o in enumerate(ms) if j != i)
        gap = min(near_face, near_member)
        worst = max(worst, gap)
        assert gap <= 0.3, f"{m[0]} {m[3]} -> {m[4]} floats {gap:.2f} m"
    assert worst <= 0.3


def test_walkway_is_dark_steel_with_a_railing(detail):
    ms = layout.carriage(detail)
    assert not [m for m in ms if m[0] == "hardware" and m[1] == "rect"]  # no white plate: the deck is the frame's dark grating
    posts = [m for m in ms if m[0] == "frame" and m[1] == "tube" and abs(m[2] - 0.08) < 1e-9]
    assert len(posts) >= 6
