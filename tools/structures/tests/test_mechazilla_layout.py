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
HINGE_TO_AXIS_M = 24.0  # physics: tower 30 m - half width 6 m
HINGE_LATERAL_M = 5.8  # physics: 9.6 / 2 + 2 / 2


@pytest.fixture(scope="module")
def detail():
    return detail_file.load(ROOT / "config" / "structures" / "starbase_olp2.toml")


def _physics():
    with open(ROOT / "config" / "launch_sites.toml", "rb") as f:
        return check_envelope.tower(tomllib.load(f))


def _closed_world(phys, side, point, opening_rad=0.0):
    """Arm-frame plan point (x along the arm, y toward the axis) in the tower frame (toward the pad, across from the
    catch axis). Built like core TowerGeometry: hinge at +-(gap / 2 + width / 2) across, the arm direction turns outward
    by `opening_rad` (0 = the core's closed pose); the layout's +Y points to the axis, the arm at `side` = -1 is its mirror."""
    lateral = 0.5 * phys["arm_closed_gap_m"] + 0.5 * phys["arm_width_m"]
    x, y = point
    cos_o, sin_o = math.cos(opening_rad), math.sin(opening_rad)
    return (x * cos_o + y * sin_o, side * (lateral + x * sin_o - y * cos_o))


def _bearing_faces(detail):
    """The flat bearing plates: (x0, y) and (x1, y) of their faces toward the axis."""
    plates = [m for m in layout.chopstick(detail) if m[0] == "bearing"]
    assert len(plates) == 1
    _, _, thick, p0, p1, _ = plates[0]
    return [(p[0], p[1] + 0.5 * thick) for p in (p0, p1)]


def _heading_deg(a, b):
    return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))


def test_bearing_pieces_parallel_and_on_the_gap_at_the_closed_pose(detail):
    phys = _physics()
    face = _bearing_faces(detail)
    plus = [_closed_world(phys, 1.0, p) for p in face]
    minus = [_closed_world(phys, -1.0, p) for p in face]
    assert abs(_heading_deg(*plus) - _heading_deg(*minus)) <= 0.5  # parallel, seen from the physics hinges
    assert abs(_heading_deg(*plus)) <= 0.25  # and along the line tower -> catch axis
    for a, b in zip(plus, minus):
        assert a[1] - b[1] == pytest.approx(phys["arm_closed_gap_m"], abs=0.05)
        assert a[1] + b[1] == pytest.approx(0.0, abs=1e-9)  # centred on the catch axis


def test_bearing_pieces_span_the_catch_axis_station(detail):
    face = _bearing_faces(detail)
    assert face[0][0] < HINGE_TO_AXIS_M - 5.0 and face[1][0] > HINGE_TO_AXIS_M + 0.5


def test_opening_turns_the_bearing_pieces_apart(detail):
    phys = _physics()
    face = _bearing_faces(detail)
    opening = math.radians(phys["arm_standby_deg"])
    plus = [_closed_world(phys, 1.0, p, opening) for p in face]
    assert _heading_deg(*plus) == pytest.approx(phys["arm_standby_deg"], abs=0.25)
    station = (HINGE_TO_AXIS_M - plus[0][0]) / (plus[1][0] - plus[0][0])
    assert plus[0][1] + station * (plus[1][1] - plus[0][1]) - HULL_RADIUS_M > 2.5  # clear of the hull at standby


def test_plan_is_a_u_root_out_leg_back_in(detail):
    c = detail["chopstick"]
    root, bend, leg_end, connector_end = layout.top_plan(detail)
    assert root == (0.0, 0.0)
    assert bend[1] < 0.0 and leg_end[1] < 0.0  # both outward of the hinge line, +Y is toward the axis
    assert bend[1] < leg_end[1]  # the leg runs back in
    assert connector_end[1] == pytest.approx(layout.bearing_tube_y(c))  # the connector lands on the bearing tube
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


def _surface_distance_to_axis(member):
    """Least distance from the catch axis of a member's axis (10 cm steps), minus its half size. Arm frame, closed pose."""
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
            best = min(best, math.hypot(x + nx * s - HINGE_TO_AXIS_M, y + ny * s - HINGE_LATERAL_M))
    return best


def test_closed_arm_clears_the_hull(detail):
    near = [m for m in layout.chopstick(detail) if m[0] != "pad"]
    assert min(_surface_distance_to_axis(m) for m in near) >= HULL_RADIUS_M + 0.03


def test_pad_stands_at_the_hull_at_the_arm_end(detail):
    c = detail["chopstick"]
    lateral_face = HINGE_LATERAL_M - (c["inner_face_y_m"] + c["pad_proud_m"])
    assert lateral_face - HULL_RADIUS_M == pytest.approx(0.1, abs=0.02)
    end = c["pad_x_m"] + 0.5 * c["pad_length_m"]
    assert abs(end - c["length_m"]) <= 0.1  # the pad ends at the arm's end
    assert c["pad_x_m"] - 0.5 * c["pad_length_m"] < HINGE_TO_AXIS_M < end  # and spans the catch axis station


def test_closed_arm_overshoots_the_axis_by_at_most_two_metres_and_lower_chord_rises(detail):
    c = detail["chopstick"]
    assert 0.0 < c["length_m"] - HINGE_TO_AXIS_M <= 2.0
    assert layout.lower_chord_z(c, c["knee_frac"] * c["length_m"]) == pytest.approx(-c["knee_depth_m"])
    assert layout.lower_chord_z(c, c["kink_frac"] * c["length_m"]) == pytest.approx(-c["depth_m"])
    assert c["knee_depth_m"] > c["depth_m"] > c["end_depth_m"]


def test_open_frame_with_a_narrow_strip_on_the_inner_chord(detail):
    ms = layout.chopstick(detail)
    assert not [m for m in ms if m[0] == "deck"]  # no slab between the chords
    strips = [m for m in ms if m[0] == "strip"]
    assert len(strips) == 1 and strips[0][2] <= 0.8  # one narrow strip on the bearing tube, not a deck
    assert max(max(m[3][2], m[4][2]) + 0.5 * (m[5] or 0.0) for m in strips) == pytest.approx(0.0, abs=1e-9)


def test_qd_arm_head_is_clear_of_the_hull_and_fits_between_the_closed_arms(detail):
    q = detail["qd_arm"]
    assert HINGE_TO_AXIS_M - q["reach_m"] - HULL_RADIUS_M == pytest.approx(1.0, abs=0.05)
    assert q["root_width_m"] < 2.0 * (HINGE_LATERAL_M - detail["chopstick"]["inner_face_y_m"])


def test_every_member_is_inside_its_table(detail):
    for build in (layout.chopstick, layout.carriage, layout.qd_arm):
        members = build(detail)
        assert members
        assert all(m[1] in ("tube", "rect") and math.dist(m[3], m[4]) > 0.0 for m in members)


def test_envelope_passes_and_lists_only_table_rows():
    with open(ROOT / "config" / "launch_sites.toml", "rb") as f:
        sites = tomllib.load(f)
    failures, exceptions = check_envelope.check(detail_file.load(check_envelope.DETAIL), check_envelope.tower(sites))
    assert failures == []
    assert set(exceptions) <= set(check_envelope.ALLOWED)
