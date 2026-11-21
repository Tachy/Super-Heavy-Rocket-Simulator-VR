"""Crew seats (fidelity Task 8): seat_shape geometry, the [seat_shape] table against its measurement (cabin_fit/parts/seats.py), the
ksc_crew3 camera row against its fit, the crew figure inside the seat (no interpenetration > 1 cm), the faces on their atlas regions, and the
generated seat nodes."""
from __future__ import annotations

import math
import sys
import tomllib
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "blender"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import cabin_textures as ct  # noqa: E402
import seat_shape as ss  # noqa: E402
import vehicle_spec  # noqa: E402
from cabin_fit.parts import seats as seats_fit  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
FIGURE_TOL_M = 0.01  # [S] brief: no interpenetration of the crew figure and its seat over 1 cm


@pytest.fixture(scope="module")
def spec():
    return vehicle_spec.load("crew_dragon")


def _seat_frame(seat):
    a = math.radians(seat["back_angle_deg"])
    h = np.asarray(seat["hip_m"], float)
    b, n = np.array([math.cos(a), -math.sin(a), 0.0]), np.array([math.sin(a), math.cos(a), 0.0])

    def to_seat(p):
        d = np.asarray(p, float) - h
        return (float(d @ b), float(d @ n), float(d[2]))
    return to_seat


def test_section_is_closed_symmetric_and_outward(spec):
    shape = spec["cockpit"]["seat_shape"]
    for x in (0.0, 0.4, 0.8):
        sec = ss.back_section(shape, x)
        n = len(sec)
        assert n % 2 == 0 and sec[0][1] == pytest.approx(0.0) and sec[n // 2][1] == pytest.approx(0.0)
        for k in range(1, n // 2):  # mirror symmetric about z = 0
            assert sec[k][0] == pytest.approx(sec[n - k][0]) and sec[k][1] == pytest.approx(-sec[n - k][1])
        assert sec[0][2] == "pad" and sec[n // 2][2] == "shell"
        area = sum(sec[i][0] * sec[(i + 1) % n][1] - sec[(i + 1) % n][0] * sec[i][1] for i in range(n)) / 2.0  # (y, z) shoelace: counter-clockwise seen from +x > 0
        assert area > 0.0
        hw = ss.interp_rows(shape["back_rows"], x)[0]
        assert max(abs(p[1]) for p in sec) == pytest.approx(hw, abs=1e-9)


def test_seat_shape_table_is_the_measurement(spec):
    shape = spec["cockpit"]["seat_shape"]
    table = seats_fit.table()
    w, sw = table["shoulder_width_m (edges)"]
    shoulders = [r for r in shape["back_rows"] if 0.25 <= r[0] <= 0.6]
    assert shoulders and all(abs(2.0 * r[1] - w) <= 0.002 for r in shoulders)  # the widest rows = the measured shell width
    w2, _ = table["shoulder_width_m (pitch - gap)"]
    assert abs(w - w2) < sw  # the two reads (edges, pitch - gap; KSC and crew1_1) agree
    for end, k in (("knee", 0), ("foot", 1)):
        v, s = table[f"frame_{end}_m"]
        assert shape["frame_width_m"][k] == pytest.approx(v, abs=0.002) and s < 0.05
    assert shape["rail_section_m"][0] == pytest.approx(table["frame_rail_m"][0], abs=0.002)
    v, s = table["frame_end_m"]
    assert shape["frame_end_m"] == pytest.approx(v, abs=0.002) and s < 0.02
    for key, val in (("wide", shape["armrest_size_m"][2]), ("length", shape["armrest_size_m"][0]), ("out", shape["armrest_centre_m"][2])):
        v, s = table[f"armrest_{key}_m"]
        assert val == pytest.approx(v, abs=0.002) and s < 0.03, key
    seat_pitch = sorted(s["hip_m"][2] for s in spec["cockpit"]["seat"])
    assert seat_pitch[2] - seat_pitch[1] == pytest.approx(seats_fit.PITCH_M)  # the ratios are scaled by the table's pitch


def test_ksc_camera_row_is_its_fit():
    res = seats_fit.fit_camera()
    assert res.residual_px[seats_fit.PHOTO] < 5.0  # of 3603 px
    assert all(res.constrained_by_data[k] for k in res.constrained_by_data)
    cam = res.cameras[seats_fit.PHOTO].to_row()
    rows = tomllib.loads((ROOT / "config" / "cockpits" / "photo_cameras.toml").read_text(encoding="utf-8"))["photo"]
    row = next(r for r in rows if r["id"] == seats_fit.PHOTO)
    assert row["position_m"] == pytest.approx(cam["position_m"], abs=0.001)
    assert row["forward"] == pytest.approx(cam["forward"], abs=1e-4) and row["up"] == pytest.approx(cam["up"], abs=1e-4)
    assert row["fov_h_deg"] == pytest.approx(cam["fov_h_deg"], abs=0.01)


def test_figure_inside_seat_shell(spec):
    """The crew figure (its seated rest pose, crew_figure.gltf in the seat frame) does not reach more than 1 cm into the moulded back or pan,
    the leg-support rails, the foot plate or the armrests of its seat."""
    import test_crew_figure as tcf
    if not tcf.GENERATED:
        pytest.skip("no generated Dragon models")
    _, _, prims = tcf._mesh()
    pts = [tuple(float(c) for c in p) for _, pos, _ in prims for p in pos]
    ck = spec["cockpit"]
    raise_deg = spec["crew_figure"]["figure"]["thigh_raise_deg"]
    for seat in ck["seat"]:
        worst = max(ss.inside_depth(ck["seat_shape"], seat["width_m"], raise_deg, p) for p in pts)
        assert worst <= FIGURE_TOL_M, (seat["id"], "back / pan", worst)
        bars, slab = ss.seat_parts(ck["seat_shape"], seat, raise_deg, _seat_frame(seat))
        for k, bar in enumerate(bars):
            assert max(ss.bar_inside_depth(bar, p) for p in pts) <= FIGURE_TOL_M, (seat["id"], "bar", k)
        assert max(ss.slab_inside_depth(slab, p) for p in pts) <= FIGURE_TOL_M, (seat["id"], "foot plate")


def test_leg_support_is_one_closed_rounded_frame(spec):
    """The leg support is one carbon U-frame (crew1_1 / KSC): from the knee end along the shins to a closing bar frame_end_m below the plate's
    top face, over rounded corners (no kink > the bend step), outer widths frame_width_m at both ends; the armrests are short black blocks."""
    ck = spec["cockpit"]
    shape = ck["seat_shape"]
    raise_deg = spec["crew_figure"]["figure"]["thigh_raise_deg"]
    w, _ = shape["rail_section_m"]
    for seat in ck["seat"]:
        to_seat = _seat_frame(seat)
        plate_c, plate_n, _ = ss._plate_frame(seat, to_seat)
        knee = ss._knee_end(shape, raise_deg)
        path, sides, _ = ss.leg_support_path(shape, knee, plate_c, plate_n, 0.0)
        assert path[0][:2] == pytest.approx(knee[:2]) and path[-1][:2] == pytest.approx(knee[:2])
        assert abs(path[0][2] - path[-1][2]) + w == pytest.approx(shape["frame_width_m"][0])  # knee end outer width
        below = [-float(np.dot(np.subtract(p, plate_c), plate_n)) for p in path]
        assert max(below) + 0.5 * w == pytest.approx(shape["frame_end_m"], abs=1e-6)  # the closing bar's outer edge
        assert max(abs(p[2]) for p in path) + 0.5 * w == pytest.approx(0.5 * shape["frame_width_m"][1], abs=0.003)  # less the corner fillet
        for a, b, c in zip(path, path[1:], path[2:]):  # rounded: no kink sharper than one bend step
            u, v = np.subtract(b, a), np.subtract(c, b)
            ang = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(u, v) / np.linalg.norm(u) / np.linalg.norm(v))))))
            assert ang <= 90.0 / ss.FRAME_BEND_STEPS + 1e-6
        faces = ss.seat_faces(shape, seat, raise_deg, to_seat)
        assert sum(1 for f in faces if f["group"] == ("frame",)) > 0
        arm = [f for f in faces if f["group"] in (("armrest0",), ("armrest1",))]
        assert arm and all(f["part"] == "pad" for f in arm)  # black padded blocks (the photos), not carbon
        span = max(p[1] for f in arm for p in f["pts"]) - min(p[1] for f in arm for p in f["pts"])
        assert span == pytest.approx(shape["armrest_size_m"][0], abs=1e-6)


def test_smooth_groups_are_named(spec):
    """Smooth groups are keyed by explicit names (no id() of temporaries, which CPython reuses)."""
    ck = spec["cockpit"]
    seat = ck["seat"][1]
    groups = {f["group"] for f in ss.seat_faces(ck["seat_shape"], seat, spec["crew_figure"]["figure"]["thigh_raise_deg"], _seat_frame(seat))}
    groups.discard(None)
    assert groups == {("back", "pad"), ("back", "shell"), ("pan", "pad"), ("pan", "shell"), ("frame",), ("plate",), ("armrest0",), ("armrest1",)}


def test_seat_faces_lie_on_their_atlas_regions(spec):
    ck = spec["cockpit"]
    seat = ck["seat"][1]
    faces = ss.seat_faces(ck["seat_shape"], seat, spec["crew_figure"]["figure"]["thigh_raise_deg"], _seat_frame(seat))
    albedo, _, _ = ct.bake_seat(ck)
    px = ct.SEAT_ATLAS_PX / ss.SEAT_ATLAS_M
    wrong = []
    for f in faces:
        u = sum(q[0] for q in f["uv"]) / len(f["uv"])
        v = sum(q[1] for q in f["uv"]) / len(f["uv"])
        assert 0.0 <= u <= ss.SEAT_ATLAS_M and 0.0 <= v <= ss.SEAT_ATLAS_M, f["part"]
        lum = float(albedo[min(ct.SEAT_ATLAS_PX - 1, int(v * px)), min(ct.SEAT_ATLAS_PX - 1, int(u * px))].mean())
        # shell faces on the white paint / grey rim (>= 0.28), pad and rest faces on black (the drawn buckle 0.35 lies on the pan's pad, the white 0.62)
        if (f["part"] == "shell" and lum < 0.2) or (f["part"] != "shell" and lum > 0.5):
            wrong.append((f["part"], round(u, 3), round(v, 3), round(lum, 3)))
    assert not wrong, wrong[:10]


def test_generated_seats_follow_the_shape(spec):
    import test_dragon_cabin as tdc
    if not tdc.GENERATED:
        pytest.skip("no generated Dragon models")
    nodes = tdc._nodes()
    ck = spec["cockpit"]
    hw_max = max(r[1] for r in ck["seat_shape"]["back_rows"])
    for seat in ck["seat"]:
        to_seat = _seat_frame(seat)
        pad = [to_seat(p) for p in nodes[f"seat_pad_{seat['id']}"]["pos"]]
        shell = [to_seat(p) for p in nodes[f"seat_shell_{seat['id']}"]["pos"]]
        assert max(abs(p[2]) for p in shell) == pytest.approx(hw_max, abs=0.003)  # the shoulders' measured half width
        top = max(p[0] for p in pad)
        assert top == pytest.approx(ck["seat_shape"]["back_rows"][-1][0], abs=0.002)  # the rounded top ends on the last row
        # the cushion's middle is the figure's seat envelope: 1 cm behind the hip point
        mid = [p for p in pad if abs(p[2]) < 0.05 and 0.1 < p[0] < 0.6]
        assert mid and max(p[1] for p in mid) == pytest.approx(ss.BACK_SURFACE_Y, abs=0.002)
        rest = [to_seat(p) for p in nodes[f"seat_rest_{seat['id']}"]["pos"]]
        foot = ck["seat_shape"]["frame_width_m"][1]
        assert max(abs(p[2]) for p in rest if p[0] < -0.3) == pytest.approx(0.5 * foot, abs=0.003)  # the frame's outer width at the foot
