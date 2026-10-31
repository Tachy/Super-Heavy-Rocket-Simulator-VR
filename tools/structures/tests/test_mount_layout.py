"""Mount, trench and hardstand layout: measured plan, physics deck height, ring, portals, stack clearance."""
from __future__ import annotations

import math
import sys
import tomllib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools" / "structures"))
sys.path.insert(0, str(ROOT / "tools" / "structures" / "blender"))
import detail_file  # noqa: E402
import mount_layout as ml  # noqa: E402

DETAIL = ROOT / "config" / "structures" / "starbase_olp2.toml"


@pytest.fixture(scope="module")
def detail():
    return detail_file.load(DETAIL)


@pytest.fixture(scope="module")
def mount(detail):
    return ml.mount(detail)


@pytest.fixture(scope="module")
def trench(detail):
    return ml.trench(detail)


def _physics():
    with open(ROOT / "config" / "launch_sites.toml", "rb") as f:
        sites = tomllib.load(f)
    return next(s for s in sites["site"] if s["id"] == "starbase_olp1")["tower"]


def _engine():
    with open(ROOT / "config" / "engines" / "raptor3.toml", "rb") as f:
        e = tomllib.load(f)
    with open(ROOT / "config" / "vehicles" / "detail" / "starship_block3.toml", "rb") as f:
        d = tomllib.load(f)["engines"]["raptor3"]
    return {"nozzle_exit_diameter_m": e["performance"]["nozzle_exit_diameter_m"], **d}


def _ring_radii():
    with open(ROOT / "config" / "vehicles" / "starship_block3.toml", "rb") as f:
        stage = tomllib.load(f)["stage"][0]
    return sorted({g["ring_radius_m"] for g in stage["engine_group"]})


def test_mount_keys_are_positive_and_tagged(detail):
    assert detail["mount"]["deck_height_m"] > 0.0 and detail["trench"]["width_m"] > 0.0
    assert detail_file.value_lines_without_tag(DETAIL) == []


def test_ring_top_is_the_measured_deck(detail, mount):
    h = detail["mount"]["measured_height_m"]
    assert detail["mount"]["deck_height_m"] == pytest.approx(h, abs=0.01)
    # the ring's top annulus and the clamps start at that height: some hardware polygon lies in that plane
    tops = [v[2] for verts, _ in mount.nodes["hardware"] for v in verts]
    assert any(abs(z - h) < 1e-9 for z in tops)


def test_base_and_top_plan_are_the_measured_ones(detail, mount):
    lo, hi = mount.extent("plinth")
    assert hi[1] - lo[1] == pytest.approx(44.6, abs=0.1)  # base across
    assert hi[0] - lo[0] == pytest.approx(40.0, abs=0.1)  # base along the mount-tower line
    deck = [v for verts, _ in mount.nodes["shell"] for v in verts if abs(v[2] - ml.Shape(detail).deck_z) < 1e-9]
    ys = [abs(v[1]) for v in deck]
    assert 2.0 * max(ys) == pytest.approx(18.7, abs=0.1)  # top deck width
    assert detail["mount"]["measured_height_m"] == 14.5


def test_slope_is_the_measured_frustum(detail):
    sh = ml.Shape(detail)
    assert sh.run == pytest.approx(12.95, abs=0.01)
    assert sh.top_half_t == pytest.approx(7.05, abs=0.01)
    angle = math.degrees(math.atan2(sh.deck_z - sh.plinth_z, sh.run))
    assert 43.0 < angle < 50.0  # 48 (5) deg measured


def test_ring_outer_and_opening(detail, mount):
    pts = [v for verts, _ in mount.nodes["hardware"] for v in verts if abs(v[2] - detail["mount"]["deck_height_m"]) < 1e-9]
    radii = [math.hypot(v[0], v[1]) for v in pts]
    assert min(radii) == pytest.approx(4.5, abs=0.01) and max(radii) == pytest.approx(5.2 + 0.0, abs=0.7)
    ring_top = [math.hypot(v[0], v[1]) for verts, _ in mount.nodes["hardware"] for v in verts
                if abs(v[2] - detail["mount"]["deck_height_m"]) < 1e-9]
    assert max(r for r in ring_top if r < 5.3) == pytest.approx(5.2, abs=0.01)
    clamp_tops = {round(v[2], 6) for verts, _ in mount.nodes["hardware"] for v in verts
                  if abs(v[2] - (detail["mount"]["deck_height_m"] + detail["mount"]["clamp_height_m"])) < 1e-9}
    assert clamp_tops, "clamps on the ring"
    assert detail["mount"]["clamp_count"] == 20


def test_two_exit_ramps_opposite_and_perpendicular_to_the_tower_line(detail, trench):
    lo, hi = trench.extent("concrete")
    # ramps on both +-Y sides (across), none along the mount-tower line (X)
    assert lo[1] < -40.0 and hi[1] > 40.0
    assert max(abs(lo[0]), abs(hi[0])) == pytest.approx(0.5 * detail["trench"]["width_m"], abs=0.01)
    sh = ml.Shape(detail)
    mid = 0.5 * (sh.half_a + ml.ramp_end_m(detail))
    assert mid == pytest.approx(detail["trench"]["centre_m"], abs=0.5)  # 33 m from the axis
    assert lo[2] == pytest.approx(-detail["mount"]["pit_depth_m"], abs=1e-9)
    assert hi[2] == pytest.approx(0.0, abs=1e-9)
    assert trench.extent("wall")[1][2] == pytest.approx(detail["trench"]["wall_height_m"], abs=1e-9)  # low wall


def test_ramps_are_as_wide_as_the_portal_foot(detail):
    assert detail["mount"]["opening_width_ground_m"] == pytest.approx(detail["trench"]["width_m"], abs=0.01)


def _covering(mount, nodes, y_sign, x, z):
    """Polygons of `nodes` across the portal plane in front of the bucket at (x, z)."""
    out = []
    for node in nodes:
        for verts, _ in mount.nodes[node]:
            if min(v[1] * y_sign for v in verts) > 5.0 and _inside_xz(verts, x, z):
                out.append(node)
    return out


def test_the_portal_is_one_big_opening_on_both_sides(detail, mount):
    sh = ml.Shape(detail)
    nodes = ("shell", "plinth", "pier", "hood")
    for sy in (-1.0, 1.0):
        for z in (0.5 * sh.plinth_z + 1.0, 0.5 * sh.open_top, sh.open_top - 0.2):
            for frac in (-0.95, 0.0, 0.95):
                assert _covering(mount, nodes, sy, frac * sh.xi(z), z) == [], f"opening blocked at z {z}, side {frac}"
        # above the opening: the spandrel; outside it: the pier
        assert _covering(mount, nodes, sy, 0.0, sh.open_top + 2.0)
        assert _covering(mount, ("pier",), sy, sh.xi(2.0) + 1.0, 2.0)
    assert sh.open_top / sh.deck_z == pytest.approx(0.5, abs=0.05)  # the opening is half the height (dusk 2025-09)
    assert 2.0 * sh.xi(0.0) / detail["mount"]["base_toward_m"] > 0.6  # and most of the width


def test_front_face_is_closed_with_rectangular_doors(detail, mount):
    doors = mount.nodes["door"]
    assert len(doors) == 2 and all(len(verts) == 4 for verts, _ in doors)  # no arches
    assert all(max(v[0] for v in verts) < -10.0 for verts, _ in doors)  # in the -X face


def test_no_bar_outside_the_apron(detail, trench):
    assert "wall" in trench.nodes
    lo, hi = trench.extent("wall")
    assert max(abs(lo[0]), abs(hi[0])) == pytest.approx(0.5 * detail["trench"]["width_m"] + detail["trench"]["wall_thickness_m"])
    assert hi[1] == pytest.approx(ml.ramp_end_m(detail))  # the walls end with the ramp: no separate long wall


def test_bucket_floor_runs_from_the_ridge_to_the_pit(detail, mount):
    lo, hi = mount.extent("deflector")
    sh = ml.Shape(detail)
    assert lo[2] == pytest.approx(sh.floor_z, abs=1e-9)
    assert hi[2] == pytest.approx(detail["mount"]["ridge_top_m"] + detail["mount"]["rib_depth_m"], abs=1e-9)
    assert hi[2] < sh.open_top  # the ridge stays below the ceiling


def test_hardstand_surrounds_the_site_and_leaves_the_pit_open(detail, trench):
    lo, hi = trench.extent("hardstand")
    sh = ml.Shape(detail)
    assert lo[2] == hi[2] == pytest.approx(detail["trench"]["hardstand_lift_m"])
    assert lo[0] < -sh.half_t - 10.0 and hi[0] > detail["trench"]["foot_x_m"] + 30.0
    assert lo[1] < -ml.ramp_end_m(detail) - 10.0 and hi[1] > ml.ramp_end_m(detail) + 10.0
    for verts, _ in trench.nodes["hardstand"]:
        cx = sum(v[0] for v in verts) / len(verts)
        cy = sum(v[1] for v in verts) / len(verts)
        assert not (abs(cx) < sh.half_t and abs(cy) < sh.half_a), "hardstand over the mount"
        assert not (abs(cx) < 0.5 * detail["trench"]["width_m"] and abs(cy) < ml.ramp_end_m(detail)), "hardstand over a ramp"


def test_tower_foot_stands_on_the_physics_axis_and_off_the_mount(detail, trench):
    t = detail["trench"]
    assert t["foot_x_m"] == pytest.approx(_physics()["distance_m"], abs=0.01)
    lo, hi = trench.extent("foot")
    assert lo[0] >= ml.Shape(detail).half_t  # off the mount's base
    assert hi[2] == pytest.approx(t["foot_roof_height_m"])


def _inside_xz(verts, x, z):
    """Point (x, z) inside the polygon projected on the XZ plane."""
    inside = False
    for i, a in enumerate(verts):
        b = verts[(i + 1) % len(verts)]
        if (a[2] > z) != (b[2] > z) and x < (b[0] - a[0]) * (z - a[2]) / (b[2] - a[2]) + a[0]:
            inside = not inside
    return inside


def test_booster_stack_clears_deck_ring_and_hoods(detail, mount):
    """Hull above the ring top and every bell of the booster's 33 engines below it clear every mount surface."""
    h = detail["mount"]["deck_height_m"]
    bad = ml.stack_violations(mount, h, 4.5, _ring_radii(), _engine())
    assert bad == [], f"{len(bad)} mount points inside the stack, first {bad[:3]}"


def test_clearance_test_catches_a_wrong_ring(detail):
    narrow = dict(detail)
    narrow["mount"] = dict(detail["mount"], ring_opening_diameter_m=8.0)
    bad = ml.stack_violations(ml.mount(narrow), detail["mount"]["deck_height_m"], 4.5, _ring_radii(), _engine())
    assert bad, "a 8 m opening must hit the hull and the outer engines"


def test_hoods_and_qd_heads_stand_off_the_hull(detail, mount):
    hull = 4.5
    for node in ("hood", "hardware"):
        for verts, _ in mount.nodes[node]:
            for v in verts:
                if v[2] > detail["mount"]["deck_height_m"] + 1e-6 and node == "hood":
                    assert math.hypot(v[0], v[1]) >= hull + 1.0
    qd_heads = [math.hypot(v[0], v[1]) for verts, _ in mount.nodes["hardware"] for v in verts
                if v[2] > detail["mount"]["deck_height_m"] + 1.0]
    assert min(qd_heads) >= hull + 0.3


def test_hoods_are_bent_ducts_down_the_front_face(detail, mount):
    m = detail["mount"]
    above = [v for verts, _ in mount.nodes["hood"] for v in verts if v[2] > m["deck_height_m"]]
    assert max(v[0] for v in above) < 0.0  # on the front side of the ring, none toward the tower
    ys = {math.copysign(1.0, v[1]) for v in above if abs(v[1]) > 3.0}
    assert ys == {-1.0, 1.0}
    deck = ml.Shape(detail).deck_z
    box = [v for verts, _ in mount.nodes["hood"] for v in verts if v[0] == m["hood_front_x_m"]]
    assert min(v[2] for v in box) == pytest.approx(deck - m["hood_drop_m"])  # the box runs down the sloped face
    assert max(abs(v[1]) for v in box) < ml.Shape(detail).top_half_a


def test_every_polygon_is_planar_and_outward_hinted(mount, trench):
    for surf in (mount, trench):
        for polys in surf.nodes.values():
            for verts, uvs in polys:
                assert len(verts) == len(uvs) >= 3
                n = ml.newell_normal(verts)
                length = math.sqrt(sum(c * c for c in n))
                assert length > 1e-9, "degenerate polygon"
                for v in verts[3:]:  # planarity
                    d = sum((v[k] - verts[0][k]) * n[k] / length for k in range(3))
                    assert abs(d) < 1e-6
