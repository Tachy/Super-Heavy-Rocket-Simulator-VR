"""Task 4b: lattice-wing derivation of the grid fin's normal force slope and limit."""
import csv
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import grid_fin_lattice as gl  # noqa: E402


def test_cascade_thin_limit_is_the_isolated_plate():
    sigma = 1e-4
    assert gl.cascade_slope(sigma, 0.0) == pytest.approx(2.0 * math.pi * sigma, rel=1e-3)


def test_cascade_dense_limit_is_the_guide_vane_momentum():
    # mass flow rho U per unit area turned by alpha: F / q = 2 alpha, whatever the solidity beyond this
    assert gl.cascade_slope(50.0, 0.0) == pytest.approx(2.0, rel=1e-6)


def test_cascade_prandtl_glauert_thin_limit():
    sigma, mach = 1e-4, 0.6
    assert gl.cascade_slope(sigma, mach) == pytest.approx(2.0 * math.pi * sigma / math.sqrt(1 - mach * mach), rel=1e-3)


def test_oblique_shock_reference_case():
    # NACA 1135 charts: Mach 2, 10 deg: wave angle 39.3 deg, p2/p1 1.707 (tolerance from the chart reading)
    p_ratio = gl.oblique_shock_pressure_ratio(2.0, math.radians(10.0))
    assert p_ratio == pytest.approx(1.707, abs=0.01)


def test_prandtl_meyer_reference_case():
    assert math.degrees(gl.prandtl_meyer(2.0)) == pytest.approx(26.38, abs=0.01)


def test_wave_chord_follows_reflection():
    p_beta = 0.5
    assert gl.wave_chord(0.3, p_beta) == pytest.approx(0.3)  # no reflection yet: full Ackeret
    assert gl.wave_chord(0.5, p_beta) == pytest.approx(0.5)  # ideal: the wave reaches the trailing edge of the next wall
    assert gl.wave_chord(0.8, p_beta) == pytest.approx(0.2)  # the reflected wave unloads the rear
    assert gl.wave_chord(1.0, p_beta) == pytest.approx(0.0)  # Busemann: no net turning
    assert gl.wave_chord(1.2, p_beta) == pytest.approx(0.2)


def test_supersonic_slope_is_ackeret_for_short_cells_small_alpha():
    # chord well inside the first Mach wave, no scoop, full active area
    lat = gl.Lattice(tip_m=0.2, scoop_m=0.0, pitch_m=0.35, wall_m=0.0, active=1.0)
    mach = 3.0
    expected = 4.0 / math.sqrt(mach * mach - 1.0) * 0.2 / 0.35
    assert gl.cn_alpha(mach, lat, alpha_rad=math.radians(0.2)) == pytest.approx(expected, rel=1e-2)


def test_depth_scales_supersonic_slope_and_saturates_subsonic():
    short = gl.Lattice(tip_m=0.4, scoop_m=0.2, pitch_m=0.35, wall_m=0.01, active=0.9)
    deep = gl.Lattice(tip_m=0.7, scoop_m=0.35, pitch_m=0.35, wall_m=0.01, active=0.9)
    assert gl.cn_alpha(3.0, deep) / gl.cn_alpha(3.0, short) == pytest.approx(0.7 / 0.4, rel=0.02)
    assert gl.cn_alpha(0.3, deep) / gl.cn_alpha(0.3, short) < 1.3


def test_subsonic_slope_cannot_pass_the_momentum_bound():
    deep = gl.Lattice(tip_m=2.0, scoop_m=0.0, pitch_m=0.2, wall_m=0.0, active=1.0)
    assert gl.cn_alpha(0.2, deep) <= 2.0


def test_choking_dip_deepens_with_the_wall():
    thin = gl.Lattice(tip_m=0.45, scoop_m=0.22, pitch_m=0.35, wall_m=0.01, active=0.9)
    thick = gl.Lattice(tip_m=0.45, scoop_m=0.22, pitch_m=0.35, wall_m=0.04, active=0.9)
    assert gl.cn_alpha(1.1, thick) < gl.cn_alpha(1.1, thin)
    assert gl.start_mach(thick) > gl.start_mach(thin) > 1.0


def test_limit_rises_with_mach_above_the_transonic_band():
    lat = gl.NOMINAL
    assert gl.limit_deg(1.1, lat) < gl.limit_deg(2.0, lat) < gl.limit_deg(3.0, lat)
    assert all(0.0 < gl.limit_deg(m, lat) <= gl.LIMIT_CAP_DEG for m in gl.MACH_GRID)


def test_authority_moments_three_fins():
    a = gl.authority(q_pa=100e3, slope=2.0, limit_deg=30.0, area_m2=10.0, radius_m=7.0, lever_m=44.0, deflection_deg=30.0)
    f = 100e3 * 10.0 * 2.0 * 0.5
    assert a["fin_force_n"] == pytest.approx(f)
    assert a["roll_nm"] == pytest.approx(3.0 * f * 7.0)
    assert a["pitch_nm"] == pytest.approx(2.0 * f * 44.0)  # the opposite pair
    assert a["yaw_nm"] == pytest.approx(f * 44.0)  # the single fin


def test_authority_stops_at_the_limit():
    a = gl.authority(q_pa=1.0, slope=1.0, limit_deg=10.0, area_m2=1.0, radius_m=1.0, lever_m=1.0, deflection_deg=30.0)
    assert a["fin_force_n"] == pytest.approx(math.sin(math.radians(10.0)))


def test_every_row_has_values_and_errors():
    rows = gl.table(gl.NOMINAL)
    assert [r["mach"] for r in rows] == gl.MACH_GRID
    for r in rows:
        assert r["cn_alpha"] > 0.0 and 0.0 < r["cn_err_rel"] < 1.0
        assert r["limit_deg"] > 0.0 and r["limit_err_deg"] > 0.0
        assert r["cn_low"] <= r["cn_alpha"] <= r["cn_high"]


def test_config_table_is_read_unchanged():
    t = gl.config_table()
    assert t["mach"][0] == 0.0 and t["cn_alpha_per_rad"][0] == pytest.approx(2.9)
    assert gl.config_value(t, "cn_alpha_per_rad", 2.5) == pytest.approx(1.65)


def _super_heavy_table():
    with open(gl.ROOT / "config" / "aero" / "super_heavy_grid_fin.csv", encoding="utf-8") as handle:
        return list(csv.DictReader(line for line in handle if not line.startswith("#")))


def test_super_heavy_table_is_the_derivation():
    rows = _super_heavy_table()
    assert len(rows) == len(gl.MACH_GRID)
    for row, derived in zip(rows, gl.table()):
        assert float(row["mach"]) == derived["mach"]
        assert float(row["cn_alpha_per_rad"]) == pytest.approx(derived["cn_alpha"], abs=0.006)
        assert float(row["limit_deg"]) == pytest.approx(derived["limit_deg"], abs=0.06)


def test_super_heavy_table_keeps_the_descent_axial_curve():
    generic = gl.config_table()
    for row in _super_heavy_table():
        assert float(row["ca_axial"]) == pytest.approx(gl.config_value(generic, "ca_axial", float(row["mach"])), abs=1e-4)


def test_falcon_9_keeps_the_generic_table():
    text = (gl.ROOT / "config" / "vehicles" / "falcon9_block5.toml").read_text(encoding="utf-8")
    assert 'table = "aero/grid_fin.csv"' in text
    assert 'table = "aero/super_heavy_grid_fin.csv"' in (gl.ROOT / "config" / "vehicles" / "starship_block3.toml").read_text(encoding="utf-8")
    generic = gl.config_table()
    assert generic["cn_alpha_per_rad"][0] == 2.9 and generic["limit_deg"][-1] == 30.0
