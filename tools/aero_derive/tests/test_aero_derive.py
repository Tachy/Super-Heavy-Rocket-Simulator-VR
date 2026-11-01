"""Validation of the derivation tool against closed-form Newtonian results and flat-plate friction."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import buildup  # noqa: E402
import geometry  # noqa: E402
import newtonian  # noqa: E402


def sphere(radius=1.0, n_lat=120, n_lon=240):
    theta = np.linspace(0.0, math.pi, n_lat + 1)
    phi = np.linspace(0.0, 2.0 * math.pi, n_lon + 1)
    def p(t, f):
        return np.array([radius * math.cos(t), radius * math.sin(t) * math.cos(f), radius * math.sin(t) * math.sin(f)])
    out = []
    for i in range(n_lat):
        for j in range(n_lon):
            a, b, c, d = p(theta[i], phi[j]), p(theta[i + 1], phi[j]), p(theta[i + 1], phi[j + 1]), p(theta[i], phi[j + 1])
            out += [[a, b, c], [a, c, d]]
    return np.array(out)


def disc(radius=1.0, x=0.0, n=720):
    phi = np.linspace(0.0, 2.0 * math.pi, n + 1)
    centre = np.array([x, 0.0, 0.0])
    return np.array([[centre, [x, radius * math.cos(phi[i]), radius * math.sin(phi[i])],
                      [x, radius * math.cos(phi[i + 1]), radius * math.sin(phi[i + 1])]] for i in range(n)])


def cylinder_side(radius=1.0, length=4.0, n=720):
    """Open tube along x plus its end discs."""
    phi = np.linspace(0.0, 2.0 * math.pi, n + 1)
    out = []
    for i in range(n):
        y0, z0, y1, z1 = radius * math.cos(phi[i]), radius * math.sin(phi[i]), radius * math.cos(phi[i + 1]), radius * math.sin(phi[i + 1])
        out += [[[0, y0, z0], [length, y0, z0], [length, y1, z1]], [[0, y0, z0], [length, y1, z1], [0, y1, z1]]]
    return np.concatenate([np.array(out, float), disc(radius, 0.0, n), disc(radius, length, n)])


def test_cp_max_mach_10_rayleigh_pitot():
    # NACA 1135 eq. 100 at M 10, gamma 1.4: p02/p1 = 129.2, so Cp = (129.2 - 1) / (0.7 * 100) = 1.832
    assert newtonian.cp_max(10.0) == pytest.approx(1.832, abs=0.002)
    # incompressible limit
    assert newtonian.cp_max(0.01) == pytest.approx(1.0, abs=1e-3)


def test_sphere_mach_10():
    # the integral of cp_max cos^2(phi) over the front hemisphere is cp_max / 2 per pi R^2 = 0.916 at Mach 10
    cp = newtonian.cp_max(10.0)
    ca, cn = newtonian.axial_force_coefficient(sphere(), (1.0, 0.0, 0.0), cp, math.pi)
    assert ca == pytest.approx(cp / 2.0, rel=0.02)
    assert ca == pytest.approx(0.92, rel=0.02)
    assert abs(cn) < 0.01


def test_disc_head_on_is_cp_max():
    cp = newtonian.cp_max(10.0)
    ca, cn = newtonian.axial_force_coefficient(disc(), (1.0, 0.0, 0.0), cp, math.pi)
    assert ca == pytest.approx(cp, rel=0.02)
    assert abs(cn) < 0.01


def test_cylinder_side_on_per_diameter_length():
    cp = newtonian.cp_max(10.0)
    radius, length = 1.0, 4.0
    ca, _ = newtonian.axial_force_coefficient(cylinder_side(radius, length), (0.0, 0.0, 1.0), cp, 2.0 * radius * length)
    assert ca == pytest.approx(4.0 / 3.0 * cp / 2.0, rel=0.02)


def test_shadowed_faces_carry_no_load():
    cp = newtonian.cp_max(10.0)
    front = disc(1.0, 0.0)
    behind = disc(0.8, 3.0)
    alone, _ = newtonian.axial_force_coefficient(front, (1.0, 0.0, 0.0), cp, math.pi)
    both, _ = newtonian.axial_force_coefficient(np.concatenate([front, behind]), (1.0, 0.0, 0.0), cp, math.pi)
    assert both == pytest.approx(alone, rel=1e-3)  # grid ties on edges
    # the rear part of a larger disc behind a small one takes only the uncovered ring
    hit = newtonian.first_hit(np.concatenate([disc(0.5, 0.0), disc(1.0, 3.0)]), (1.0, 0.0, 0.0), cell_m=0.005,
                              part_ids=np.array([0] * 720 + [1] * 720), part_names=["front", "rear"])
    assert hit.k_by_part(math.pi)["front"] == pytest.approx(0.25, rel=0.02)
    assert hit.k_by_part(math.pi)["rear"] == pytest.approx(0.75, rel=0.02)


def test_winding_does_not_matter():
    cp = newtonian.cp_max(5.0)
    tris = disc()
    flipped = tris[:, [0, 2, 1]]
    a, _ = newtonian.axial_force_coefficient(tris, (1.0, 0.0, 0.0), cp, math.pi)
    b, _ = newtonian.axial_force_coefficient(flipped, (1.0, 0.0, 0.0), cp, math.pi)
    assert a == pytest.approx(b, rel=1e-3)  # grid ties on edges


def schoenherr(re):
    """Karman-Schoenherr 0.242 / sqrt(Cf) = log10(Re Cf): independent of the formula the tool uses."""
    cf = 0.002
    for _ in range(100):
        cf = (0.242 / math.log10(re * cf)) ** 2
    return cf


def test_flat_plate_friction_re_1e8_mach_0():
    cf, _ = buildup.flat_plate_cf(1.0e8, 0.0, 288.15)
    assert cf == pytest.approx(2.13e-3, rel=0.03)  # Schlichting, Prandtl-Schlichting law
    assert cf == pytest.approx(schoenherr(1.0e8), rel=0.03)


def test_compressible_friction_falls_with_mach():
    # Adiabatic wall: Cf / Cf_inc ~ 0.8 at Mach 2 and ~ 0.45 at Mach 5 (Sommer-Short, NACA TN 3391 figures)
    cf0, _ = buildup.flat_plate_cf(1.0e7, 0.0, 288.15)
    cf2, _ = buildup.flat_plate_cf(1.0e7, 2.0, 288.15)
    cf5, _ = buildup.flat_plate_cf(1.0e7, 5.0, 288.15)
    assert 0.70 < cf2 / cf0 < 0.88
    assert 0.35 < cf5 / cf0 < 0.60


def test_buildup_values_are_positive_with_errors():
    for mach in (0.3, 0.95, 1.05, 3.0, 25.0):
        for value, err in (buildup.skin_friction_cd(1000.0, 50.0, mach, 10000.0, 63.6),
                           buildup.nose_pressure_cd(mach, 1.5),
                           buildup.base_cd(mach, 63.6, False, 63.6),
                           buildup.protuberance_cd(mach, 1.0, 63.6),
                           buildup.cavity_cd(113.0, 63.6)):
            assert value >= 0.0 and 0.0 < err < 2.0
    assert buildup.base_cd(2.0, 63.6, True, 63.6, exit_area_m2=30.0)[0] < buildup.base_cd(2.0, 63.6, False, 63.6)[0]


def test_grid_fin_choking_exceeds_open_flow():
    # choked cells spill air: transonic CA above the started supersonic value; open fraction 1 has no blockage loss
    trans = buildup.grid_fin_ca(1.2, 0.76, 10000.0, 0.2, 0.5, 0.35, 0.04, 0.5)[0]
    started = buildup.grid_fin_ca(4.0, 0.76, 25000.0, 0.2, 0.5, 0.35, 0.04, 0.5)[0]
    assert trans > started


model_present = (geometry.MODEL_ROOT / "starship_block3" / "manifest.json").exists()


@pytest.fixture(scope="module")
def loaded_vehicle():
    return geometry.load_vehicle("starship_block3")


@pytest.mark.skipif(not model_present, reason="built vehicle models are git-ignored")
class TestBuiltModel:
    @pytest.fixture
    def vehicle(self, loaded_vehicle):
        return loaded_vehicle

    def test_areas_against_the_documented_drawing(self, vehicle):
        assert vehicle.flap_area_m2("aft") == pytest.approx(36.5, abs=0.2)
        assert vehicle.flap_area_m2("fwd") == pytest.approx(12.37, abs=0.05)
        assert vehicle.grid_fin_gross_area_m2() == pytest.approx(12.63, abs=0.1)
        # mesh projection and outline must agree
        assert vehicle.flap_mesh_area_m2("aft") == pytest.approx(vehicle.flap_area_m2("aft"), rel=0.01)
        assert vehicle.flap_mesh_area_m2("fwd") == pytest.approx(vehicle.flap_area_m2("fwd"), rel=0.01)

    def test_open_fraction_near_the_cell_walls(self, vehicle):
        assert 0.6 < vehicle.grid_fin_open_fraction() < (1.0 - 0.04 / 0.35) ** 2 + 0.01

    def test_wetted_hull_is_a_lathe(self, vehicle):
        assert vehicle.hull_wetted_area_m2("booster") == pytest.approx(math.pi * 9.0 * 69.42, rel=0.01)
        area = vehicle.hull_wetted_area_m2("ship")
        assert math.pi * 9.0 * 38.7 < area < math.pi * 9.0 * 52.1
        assert vehicle.hull_mesh_wetted_area_m2("ship") == pytest.approx(area, rel=0.02)
        assert vehicle.hull_mesh_wetted_area_m2("booster") == pytest.approx(vehicle.hull_wetted_area_m2("booster"), rel=0.02)

    def test_flap_at_90_is_in_the_meridian_plane(self, vehicle):
        tris, names = vehicle.triangles(stages=("ship",), flap_deg=90.0, fin_deg=0.0)
        parts = np.array(names[1])
        flap = tris[np.array([names[0][i].startswith("flap/aft_right") for i in parts])]
        # azimuth 0 hinge: radial is +y, the panel at d = 90 lies in z = 0
        assert abs(flap[..., 2]).max() < 0.5
        assert flap[..., 1].max() > 7.9

    def test_far_plate_stand_in_is_not_physics(self, vehicle):
        _, (names, _) = vehicle.triangles(stages=("booster",), flap_deg=90.0, fin_deg=0.0)
        assert not any(n.endswith("/plate") for n in names)

    def test_stack_tail_first_hull_loads_the_base_only(self, vehicle):
        tris, (names, ids) = vehicle.triangles(stages=("booster",), flap_deg=90.0, fin_deg=0.0)
        hit = newtonian.first_hit(tris, (1.0, 0.0, 0.0), cell_m=0.05, part_ids=ids, part_names=names)
        k = hit.k_by_part(63.617)
        assert 0.7 < k["booster/hull"] <= 1.01

    def test_stack_nose_first_loads_no_booster_hull(self, vehicle):
        tris, (names, ids) = vehicle.triangles(stages=("booster", "ship"), flap_deg=90.0, fin_deg=0.0)
        hit = newtonian.first_hit(tris, (-1.0, 0.0, 0.0), cell_m=0.05, part_ids=ids, part_names=names)
        k = hit.k_by_part(63.617)
        assert k.get("booster/hull", 0.0) < 0.002  # hidden behind the ship
        assert k.get("ship/engine_bay", 0.0) == 0.0  # internal
        assert k["ship/hull"] > 0.05


@pytest.mark.parametrize("azimuth_deg", [0.0, 90.0, 180.0])
@pytest.mark.parametrize("deflection_deg", [0.0, 20.0])
def test_fin_pose_agrees_with_the_core(azimuth_deg, deflection_deg):
    """fin_transform's cell axis and normal against grid_fin_force: n = t cos d + x sin d, cells axial at d = 0."""
    a, d = math.radians(azimuth_deg), math.radians(deflection_deg)
    m = geometry.rotation((1.0, 0.0, 0.0), a) @ geometry.rotation((0.0, 1.0, 0.0), d)
    tangential = np.array([0.0, -math.sin(a), math.cos(a)])
    x = np.array([1.0, 0.0, 0.0])
    assert m @ np.array([0.0, 0.0, 1.0]) == pytest.approx(tangential * math.cos(d) + x * math.sin(d), abs=1e-12)
    assert m @ x == pytest.approx(x * math.cos(d) - tangential * math.sin(d), abs=1e-12)
