"""Task 2b: wake range, boundary layer credit, direction-aware fin CA, the generated stack table."""
import csv
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import buildup  # noqa: E402
import derive_ascent_descent as dad  # noqa: E402
import exposure  # noqa: E402
import geometry  # noqa: E402

models = pytest.mark.skipif(not (geometry.MODEL_ROOT / "starship_block3" / "manifest.json").exists(),
                            reason="built vehicle models are git-ignored")
AERO = geometry.ROOT / "config" / "aero"


def disc(x, r_in, r_out, n=64):
    """Annulus of triangles at station x, seen along the axis."""
    a = np.linspace(0.0, 2.0 * math.pi, n + 1)
    out = []
    for a0, a1 in zip(a, a[1:]):
        p = [(r_in, a0), (r_out, a0), (r_out, a1), (r_in, a1)]
        q = [[x, s * math.cos(t), s * math.sin(t)] for s, t in p]
        out += [[q[0], q[1], q[2]], [q[0], q[2], q[3]]]
    return np.array(out)


def scene(*items):
    """(name, tris) ... -> tris, names, ids"""
    names = [n for n, _ in items]
    tris = np.concatenate([t for _, t in items])
    ids = np.concatenate([np.full(len(t), i, dtype=np.int64) for i, (_, t) in enumerate(items)])
    return tris, names, ids


FLAT_HULL = lambda x: np.full_like(np.asarray(x, float), 4.0)  # noqa: E731


def test_a_part_far_upstream_shields_nothing_a_near_one_does():
    # forward ring r 4-5 (height 1 m, wake 10 heights = 10 m), then rings behind it at 5 m and at 60 m
    tris, names, ids = scene(("a/front", disc(100.0, 4.0, 5.0)), ("a/near", disc(95.0, 4.0, 5.0)), ("a/far", disc(40.0, 4.0, 5.0)))
    load = exposure.exposed_hits(tris, ids, names, FLAT_HULL, cell_m=0.05)
    k = load.k_by_part(1.0)
    annulus = math.pi * (25.0 - 16.0)
    assert k["a/front"] == pytest.approx(annulus, rel=0.02)
    assert "a/near" not in k
    assert k["a/far"] == pytest.approx(annulus, rel=0.02)


def test_the_hull_stops_the_ray_and_a_part_does_not_load_itself_twice():
    tris, names, ids = scene(("s/hull", disc(100.0, 0.0, 4.0)), ("a/behind", disc(10.0, 0.0, 3.0)),
                             ("a/ring", disc(90.0, 4.0, 5.0)), ("a/ring_back", disc(89.5, 4.0, 5.0)))
    names[3] = "a/ring"  # the ring's back shell: same part, 0.5 m behind
    k = exposure.exposed_hits(tris, ids, names, FLAT_HULL, cell_m=0.05).k_by_part(1.0)
    assert k.get("a/behind", 0.0) < 0.01  # single rays through the disc's triangle seams
    assert k["a/ring"] == pytest.approx(math.pi * 9.0, rel=0.02)


def test_fin_shells_form_one_group_per_fin():
    assert exposure.group_of("fin/90/frame") == exposure.group_of("fin/90/lattice") == "fin/90"
    assert exposure.group_of("flap/aft_left/panel") == "flap/aft_left"
    assert exposure.group_of("booster/skirt") == "booster/skirt"


def test_one_seventh_profile_share():
    assert buildup.boundary_layer_q_share(1.0, 1.0) == pytest.approx(1.0)
    assert buildup.boundary_layer_q_share(5.0, 1.0) == pytest.approx(1.0)
    assert buildup.boundary_layer_q_share(0.5, 1.0) == pytest.approx(0.5 ** (2.0 / 7.0))
    # a band from the wall to h: the mean over y is (7/9)(h/delta)^(2/7), review finding 3: 0.65 at h/delta 0.54
    y = (np.arange(200000) + 0.5) / 200000 * 0.54
    assert float(np.mean(buildup.boundary_layer_q_share(y, 1.0))) == pytest.approx(7.0 / 9.0 * 0.54 ** (2.0 / 7.0), rel=1e-3)
    assert 7.0 / 9.0 * 0.54 ** (2.0 / 7.0) == pytest.approx(0.65, abs=0.01)


def test_layer_thickness_at_peak_q():
    # rho 0.45, 413 m/s, 1.24e7 /m: 0.37 x Re^-0.2 x gives about 0.33 m at 52 m and 0.65 m at 121 m (review finding 3)
    assert buildup.turbulent_layer_thickness_m(52.0, 1.24e7 * 52.0) == pytest.approx(0.33, abs=0.02)
    assert buildup.turbulent_layer_thickness_m(121.0, 1.24e7 * 121.0) == pytest.approx(0.65, abs=0.03)


def test_thick_grid_entry_term():
    # Idel'chik tau: small for a deep cell (l/dh 1.6), larger for the scooped face's shallower mean depth
    assert buildup.thick_grid_tau(1.6) == pytest.approx(0.063, abs=0.005)
    assert buildup.thick_grid_tau(1.035) > 0.2
    nose = buildup.grid_fin_ca(0.3, 0.694, 900.0, 0.289, 0.5, 0.35, wall_m=0.04, entry_depth_m=0.5)[0]
    tail = buildup.grid_fin_ca(0.3, 0.694, 900.0, 0.128, 0.5, 0.35, wall_m=0.04, entry_depth_m=0.32)[0]
    assert tail > nose > 0.554


@models
class TestBuilt:
    @pytest.fixture(scope="module")
    def rows(self):
        return dad.run()

    def test_mean_cell_depth(self):
        v = geometry.load_vehicle("starship_block3")
        assert v.grid_fin_mean_depth_m("fwd") == pytest.approx(0.5)
        assert 0.25 < v.grid_fin_mean_depth_m("aft") < 0.4

    def test_band_is_one_term(self, rows):
        stack = [r for r in rows if r["case"] == "ascent_stack"]
        area = 2.0 * math.pi * 4.5 * (2.88 - 0.32)
        for r in stack:
            assert r["gap"] == pytest.approx(0.03 * area / dad.A_REF_M2)
            assert "booster/top_truss" not in r["other_parts"]

    def test_fins_use_the_config_count_and_load_evenly(self, rows):
        v = geometry.load_vehicle("starship_block3")
        count = len(v.stage("booster")["grid_fins"]["azimuths_deg"])
        for r in rows:
            if r["case"] == "ascent_stack":
                assert r["fins"] == pytest.approx(r["fin_ca_nose"] * count * v.grid_fin_gross_area_m2() / dad.A_REF_M2)

    def test_both_fin_directions_are_reported(self, rows):
        ca = {r["mach"]: r for r in rows if r["case"] == "descent_total_drawn_12.63"}
        assert ca[5.0]["fin_ca_nose"] > 1.5 * ca[5.0]["fin_ca_gross"]  # 1.85 at wall 0.012, the flat face leads
        assert ca[0.3]["fin_ca_gross"] > ca[0.3]["fin_ca_nose"]  # the scooped entry loses more when it is upstream

    def test_stack_table_is_the_derived_total(self, rows):
        with open(AERO / "starship_stack_ascent.csv", encoding="utf-8") as f:
            lines = f.readlines()
        assert any("[E]" in l and l.startswith("#") for l in lines)
        table = list(csv.DictReader(l for l in lines if not l.startswith("#")))
        derived = {(r["mach"], r["column"]): r["derived"] for r in rows if r["case"] == "ascent_stack"}
        assert float(table[0]["mach"]) == 0.0 and float(table[-1]["mach"]) == dad.TABLE_MAX_MACH
        for t in table[1:]:
            m = float(t["mach"])
            assert float(t["cd_axial_power_on"]) == pytest.approx(derived[(m, "on")], abs=6e-4)
            assert float(t["cd_axial_power_off"]) == pytest.approx(derived[(m, "off")], abs=6e-4)


def test_ship_table_keeps_the_old_values():
    with open(AERO / "ship_ascent.csv", encoding="utf-8") as f:
        table = list(csv.DictReader(l for l in f if not l.startswith("#")))
    values = {float(t["mach"]): (float(t["cd_axial_power_on"]), float(t["cd_axial_power_off"])) for t in table}
    assert values[0.0] == (0.30, 0.38)
    assert values[1.2] == (0.62, 0.72)
    assert values[6.0] == (0.30, 0.35)
    assert values[25.0] == (0.25, 0.29)
