"""Gimbal sweep of the engine rings: every gimballing bell over its full cone against the fixed outer ring and against
every neighbour at its own tilt. Real bell contour (mount_layout.bell_radius_m, the generator's contour), pivot at the
engine's gimbal point (bell_length_m above the exit plane), exit planes of all engines in one plane (station 0)."""
import math
import sys
import tomllib
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "blender"))
import mount_layout  # noqa: E402
import vehicle_spec  # noqa: E402

TILT_STEPS = 3     # 0, half, full cone
AZIMUTH_STEP_DEG = 30.0
CLEARANCE_M = 0.0  # signed radial clearance between two bell walls must stay positive


def _engines(vehicle):
    """[(engine id, pivot x, y, gimbal flag, ring id)] of one stage's engine groups that are Raptors/RVacs."""
    with open(vehicle_spec.VEHICLES / f"{vehicle[0]}.toml", "rb") as fh:
        stage = next(s for s in tomllib.load(fh)["stage"] if s["id"] == vehicle[1])
    spec = vehicle_spec.load("starship_block3")
    out = []
    for g in stage["engine_group"]:
        if g["engine"] not in spec["engines"]:
            continue
        angles = g.get("angles_deg") or [360.0 * i / g["count"] for i in range(g["count"])]
        for a in angles:
            r = float(g["ring_radius_m"])
            out.append((g["engine"], r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), bool(g["gimbal"])))
    return spec, out


_TABLE = {}


def _radius_table(engine):
    """(depth grid, outer bell radius) of an engine, cached; depth 0 at the gimbal point."""
    key = engine["file"]
    if key not in _TABLE:
        depth = np.linspace(0.0, engine["bell_length_m"], 400)
        _TABLE[key] = (depth, np.array([mount_layout.bell_radius_m(engine, d) for d in depth]))
    return _TABLE[key]


def _bell_points(engine):
    """Points on the outer bell wall in the engine frame (z up, pivot at 0, exit plane at z = -bell_length)."""
    length = engine["bell_length_m"]
    depth = np.unique(np.concatenate([np.linspace(0.0, length, 40), length - np.linspace(0.0, 0.1, 6)]))
    radius = np.interp(depth, *_radius_table(engine))
    psi = np.linspace(0.0, 2.0 * math.pi, 36, endpoint=False)
    return np.array([(r * math.cos(p), r * math.sin(p), -d) for d, r in zip(depth, radius) for p in psi])


_POINTS = {}


def _points(engine):
    if engine["file"] not in _POINTS:
        _POINTS[engine["file"]] = _bell_points(engine)
    return _POINTS[engine["file"]]


def _rotation(tilt_rad, azimuth_rad):
    """Axis (0, 0, 1) tilted by tilt_rad towards the azimuth direction (Rodrigues about the horizontal axis)."""
    k = np.array([-math.sin(azimuth_rad), math.cos(azimuth_rad), 0.0])
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(tilt_rad) * kx + (1.0 - math.cos(tilt_rad)) * (kx @ kx)


def _states(max_deg):
    out = [np.eye(3)]
    if max_deg <= 0.0:
        return out
    for t in range(1, TILT_STEPS):
        for a in np.arange(0.0, 360.0, AZIMUTH_STEP_DEG):
            out.append(_rotation(math.radians(max_deg * t / (TILT_STEPS - 1)), math.radians(a)))
    return out


def _clearance(spec, a, ra, b, rb):
    """Smallest radial distance of A's wall points outside B's wall (negative: inside). A, B = (engine id, pivot xyz)."""
    ea, eb = spec["engines"][a[0]], spec["engines"][b[0]]
    pts = _points(ea) @ ra.T + a[1]
    q = (pts - b[1]) @ rb  # into B's frame (rb orthogonal)
    depth = -q[:, 2]
    ok = (depth >= 0.0) & (depth <= eb["bell_length_m"])
    if not ok.any():
        return math.inf
    radius_b = np.interp(depth[ok], *_radius_table(eb))
    return float(np.min(np.hypot(q[ok, 0], q[ok, 1]) - radius_b))


def _gimbal_deg(spec, eid):
    with open(vehicle_spec.ROOT / "config" / "engines" / f"{eid}.toml", "rb") as fh:
        return float(tomllib.load(fh)["gimbal"]["max_angle_deg"])


def _command_clearance(spec, nodes):
    """Worst clearance when every gimballing engine follows one pitch/yaw/roll command: the common vector (pitch, yaw) plus
    a roll share along the ring's tangent, each engine's total tilt capped at its cone (the core realises roll differentially)."""
    worst = (math.inf, None)
    for alpha in np.arange(0.0, 360.0, 15.0):
        for scale in (0.5, 1.0):
            for roll in (-1.0, -0.5, 0.0, 0.5, 1.0):
                rots = []
                for eid, pivot, gimbal in nodes:
                    if not gimbal:
                        rots.append(np.eye(3))
                        continue
                    cone = math.radians(_gimbal_deg(spec, eid)) * scale
                    phi = math.atan2(pivot[1], pivot[0])
                    tangent = np.array([-math.sin(phi), math.cos(phi)])
                    v = (1.0 - abs(roll)) * np.array([math.cos(math.radians(alpha)), math.sin(math.radians(alpha))]) + roll * tangent
                    n = float(np.hypot(*v))
                    rots.append(_rotation(cone * min(n, 1.0), math.atan2(v[1], v[0])) if n > 1e-9 else np.eye(3))
                for i, a in enumerate(nodes):
                    for j, b in enumerate(nodes):
                        if i == j or not (a[2] or b[2]) or np.hypot(*(a[1][:2] - b[1][:2])) > 5.5:
                            continue
                        c = _clearance(spec, a, rots[i], b, rots[j])
                        if c < worst[0]:
                            worst = (c, (i, j, float(alpha), scale, roll))
    return worst


def sweep(vehicle, mode="independent"):
    """Worst signed clearance and where, over every ordered engine pair that has a gimballing member."""
    spec, engines = _engines(vehicle)
    nodes = [(e[0], np.array([e[1], e[2], spec["engines"][e[0]]["bell_length_m"]]), e[3]) for e in engines]
    worst = (math.inf, None)
    if mode == "command":
        return _command_clearance(spec, nodes), nodes
    for i, a in enumerate(nodes):
        for j, b in enumerate(nodes):
            if i == j or not (a[2] or b[2]):
                continue
            if mode == "fixed" and b[2]:
                continue  # partner at rest: the fixed bells only
            if np.hypot(*(a[1][:2] - b[1][:2])) > 5.5:
                continue
            sa = _states(_gimbal_deg(spec, a[0])) if a[2] else [np.eye(3)]
            sb = _states(_gimbal_deg(spec, b[0])) if b[2] else [np.eye(3)]
            for ia, ra in enumerate(sa):
                for ib, rb in enumerate(sb):
                    if mode == "common" and ia != ib:
                        continue
                    c = _clearance(spec, a, ra, b, rb)
                    if c < worst[0]:
                        worst = (c, (i, j, ia, ib))
    return worst, nodes


VEHICLES = [("starship_block3", "booster"), ("starship_block3", "ship"), ("starship_hls", "hls")]


@pytest.mark.parametrize("vehicle", VEHICLES)
@pytest.mark.parametrize("mode", ["fixed", "common", "command"])
def test_gimbal_sweep_keeps_every_bell_clear(vehicle, mode):
    (clearance, where), nodes = sweep(vehicle, mode)
    print(f"{vehicle} {mode}: worst signed bell-wall clearance over the full gimbal cone {clearance:+.3f} m at {where}")
    assert clearance > CLEARANCE_M, (clearance, where)


def test_independent_opposed_tilts_would_touch_neighbours_on_a_ring():
    """Information, not a limit: two neighbours tilting at each other (not a command the core issues) close the gap."""
    (clearance, _), _ = sweep(VEHICLES[0], "independent")
    print(f"booster independent worst-case pair tilt: {clearance:+.3f} m")
    assert clearance < 0.0
