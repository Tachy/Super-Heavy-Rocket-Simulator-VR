"""Chopstick / QD arm layout against the physics: parallel main beams, tip bend, hull clearance at the closed pose."""
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


def _direction(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _physics():
    with open(ROOT / "config" / "launch_sites.toml", "rb") as f:
        return check_envelope.tower(tomllib.load(f))


def _world_line(c, phys, side, y_main):
    """Plan polyline of an arm's chord in the tower frame at the closed pose: (distance toward the pad, across).
    The pose is built from the physics hinge, not from the layout: the arm points to the pad (-toward) from the hinge at
    +-(gap / 2 + width / 2) across; the layout's +Y is toward the catch axis, the arm at `side` = -1 is its mirror."""
    lateral = 0.5 * phys["arm_closed_gap_m"] + 0.5 * phys["arm_width_m"]
    return [(phys["distance_m"] - 0.5 * phys["width_m"] - x, side * (lateral - y))
            for x, y in layout.plan_line(c, y_main)]


def _heading_deg(a, b):
    return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))


def test_main_beams_parallel_and_tips_bend_inward_at_the_closed_pose(detail):
    c, phys = detail["chopstick"], _physics()
    y_in, _ = layout.chord_y(c, detail["profiles"][c["chord_profile"]])
    plus, minus = _world_line(c, phys, 1.0, y_in), _world_line(c, phys, -1.0, y_in)
    # main beams: both run straight to the pad (heading 180 deg) and keep the same distance from each other
    for line in (plus, minus):
        assert abs(_heading_deg(line[2], line[1])) == pytest.approx(0.0, abs=0.1) or             abs(abs(_heading_deg(line[1], line[2])) - 180.0) < 0.1
    gaps = [plus[i][1] - minus[i][1] for i in (1, 2)]
    assert gaps[0] == pytest.approx(gaps[1], abs=1e-6)
    assert gaps[0] == pytest.approx(phys["arm_closed_gap_m"] + 2.0 * (c["inner_face_y_m"] - y_in), abs=1e-6)
    # the outer pieces are NOT parallel: they converge by the tip angle each, the root pieces fan out
    assert plus[3][1] - minus[3][1] < gaps[0] < plus[0][1] - minus[0][1]
    tip_turn = abs(abs(_heading_deg(plus[2], plus[3])) - 180.0)
    assert tip_turn == pytest.approx(c["tip_kink_deg"], abs=0.1)


def test_bends_point_inward_at_root_and_tip(detail):
    c = detail["chopstick"]
    line = layout.plan_line(c, -0.5)
    assert math.degrees(_direction(line[0], line[1])) == pytest.approx(c["root_kink_deg"])
    assert math.degrees(_direction(line[2], line[3])) == pytest.approx(c["tip_kink_deg"])
    assert line[0][1] < line[1][1] < line[3][1]  # +Y is toward the axis


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
    assert strips and all(m[2] <= 0.8 for m in strips)  # a narrow strip, not the 4 m width
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
