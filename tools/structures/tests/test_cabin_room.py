"""The room of the Crew Dragon cabin (fidelity plan Task 9): window reveals and the ceiling ring's steps against crew1_1
(cabin_fit/parts/room.py), the red hand-holds in the forward tunnel, the closed docking hatch, the vents, the window rims and gaskets.
Generated-model cases skip without generated Dragon models (python tools/structures/build_models.py --vehicle crew_dragon)."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "blender"))
import cabin_layout as cl  # noqa: E402
import vehicle_spec  # noqa: E402
from cabin_fit.parts import room  # noqa: E402
from test_dragon_cabin import GENERATED, _nodes  # noqa: E402

ID = "crew_dragon"


@pytest.fixture(scope="module")
def spec():
    return vehicle_spec.load(ID)


@pytest.fixture(scope="module")
def nodes():
    if not GENERATED:
        pytest.skip("no generated Dragon models")
    return _nodes()


@pytest.fixture(scope="module")
def window_fit():
    return room.fit_windows()


def test_window_reveal_is_the_crew1_1_fit(spec, window_fit):
    """[cabin] window_reveal_radius_m / window_bezel_m are the crew1_1 fit within its sigma; the fitted rings lie on the reads (rms
    under 4 read sigmas: the rounded rim's edges are soft and the model has no lens distortion)."""
    vals, res, rms = window_fit
    cab = spec["cockpit"]["cabin"]
    for key, name in (("window_reveal_radius_m", "rho0"), ("window_bezel_m", "bezel")):
        value, sigma = vals[name]
        assert abs(cab[key] - value) <= sigma, (key, cab[key], value, sigma)
    assert rms < 4.0 * room.READ_SIGMA_PX
    assert max(abs(r) for r in res) < 50.0  # every read row is reached by its ring


def test_ceiling_ring_steps_lie_on_crew1_1(spec):
    """The tunnel's far rim (ceiling_x_m, tunnel_radius_m) and the closed hatch's far edge (tunnel_top_x_m) on crew1_1's middle column
    within one read sigma; the neighbouring columns within 4 sigma (the photo's arcs flatten away from the middle: lens, cockpit.md)."""
    res = room.ceiling_step_residuals(spec["cockpit"])
    for name, r in res.items():
        middle = r[list(room.CEILING_STEP_READS[name]).index(600)]
        assert abs(middle) <= room.READ_SIGMA_PX, (name, r)
        assert max(abs(v) for v in r) <= 4.0 * room.READ_SIGMA_PX, (name, r)


def test_hatch_handles_exist(spec, nodes):
    """Two red hand-holds (node tunnel_handles, CockpitView.HANDLE_RED) in the forward tunnel's mouth at +-handle_az_deg: the grip
    handle_standoff_m in from the tunnel wall, handle_rise_m above the ceiling plane, feet reaching the tunnel wall."""
    cab = spec["cockpit"]["cabin"]
    pos = nodes["tunnel_handles"]["pos"]
    assert nodes["tunnel_handles"]["idx"]
    r_grip = cab["tunnel_radius_m"] - cab["handle_standoff_m"]
    bar = cab["handle_bar_radius_m"]
    for sign in (1.0, -1.0):
        side = [p for p in pos if math.copysign(1.0, p[2]) == sign]
        assert side, sign
        azs = [math.degrees(math.atan2(p[2], p[1])) for p in side]
        mid = 0.5 * (max(azs) + min(azs))
        assert mid == pytest.approx(sign * cab["handle_az_deg"], abs=1.0)
        span = max(azs) - min(azs)
        assert cab["handle_arc_deg"] <= span <= cab["handle_arc_deg"] + 2.0 * math.degrees(2.0 * bar / r_grip)
        radii = [math.hypot(p[1], p[2]) for p in side]
        assert min(radii) == pytest.approx(r_grip - bar, abs=2e-3)
        assert max(radii) >= cab["tunnel_radius_m"]  # the feet reach into the tunnel wall
        xs = [p[0] for p in side]
        x_h = cab["ceiling_x_m"] + cab["handle_rise_m"]
        assert min(xs) >= x_h - bar - 1e-3 and max(xs) <= x_h + bar + 1e-3
        assert min(xs) > cab["ceiling_x_m"] and max(xs) < cab["tunnel_top_x_m"]  # inside the tunnel's mouth


def test_forward_hatch_and_vents_are_their_own_nodes(spec, nodes):
    cab = spec["cockpit"]["cabin"]
    door = nodes["forward_hatch"]["pos"]
    assert door and all(p[0] == pytest.approx(cab["tunnel_top_x_m"], abs=1e-4) for p in door)
    assert max(math.hypot(p[1], p[2]) for p in door) == pytest.approx(cab["tunnel_radius_m"], abs=1e-3)
    assert all(p[0] < cab["tunnel_top_x_m"] - 1e-4 or math.hypot(p[1], p[2]) > cab["tunnel_radius_m"] - 1e-3
               for p in nodes["cabin_shell"]["pos"])  # the door left the shell (the tunnel wall's top ring stays)
    vents = nodes["ceiling_vents"]["pos"]
    r0, r1 = cab["vent_radii_m"]
    a0, a1 = cab["vent_az_deg"]
    for p in vents:
        assert r0 - 1e-3 <= math.hypot(p[1], p[2]) <= r1 + 1e-3
        assert a0 - 0.1 <= abs(math.degrees(math.atan2(p[2], p[1]))) <= a1 + 0.1
        assert p[0] < cab["ceiling_x_m"]


@pytest.mark.parametrize("w", range(4))
def test_window_rim_stands_proud_and_the_gasket_rings_the_glass(spec, nodes, w):
    """The rim's crest stands window_rim_height_m proud of the wall at the reveal's edge; the dark faces (glTF UV.y 1) form a band
    window_gasket_m wide along the funnel next to the glass; every other face is white (UV.y 0)."""
    cab = spec["cockpit"]["cabin"]
    profile = spec["detail"]["capsule"]["profile_m"]
    ap = cl.window_apertures(cab)[w]
    node = nodes[f"window_{w}"]
    pos, uv = node["pos"], node["uv"]
    assert {round(v[1], 6) for v in uv} == {0.0, 1.0}
    glass = nodes[f"window_glass_{w}"]["pos"]
    glass_r = {tuple(round(c, 4) for c in p) for p in glass}
    dark = [p for p, t in zip(pos, uv) if t[1] > 0.5 and tuple(round(c, 4) for c in p) not in glass_r]
    assert dark
    run = [q for q in cab["profile_m"][1:] if q[0] <= cab["ceiling_x_m"] + 1e-9]
    nr = cl.ext_normal(profile, ap["x_m"])[1]
    proud = max((cl.lerp(run, p[0]) - math.hypot(p[1], p[2])) * nr for p in pos)  # in from the wall beneath, along its normal
    assert proud == pytest.approx(cab["window_rim_height_m"], abs=0.004)
    for p in dark:  # the gasket band's inner ring lies window_gasket_m from the glass ring along the funnel
        d = min(math.dist(p, g) for g in glass)
        assert d == pytest.approx(cab["window_gasket_m"], abs=0.003)
