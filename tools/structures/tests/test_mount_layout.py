"""Mount and trench layout (task 3): measured plan, physics deck height, ring, exits, stack clearance."""
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


def test_ring_top_is_the_physics_deck(detail, mount):
    h = _physics()["mount_height_m"]
    assert detail["mount"]["deck_height_m"] == pytest.approx(h, abs=0.01)
    lo, hi = mount.extent("hardware")
    # the ring's top annulus and the clamps start at the physics height: some hardware polygon lies in that plane
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


def test_slope_is_the_stretched_measured_frustum(detail):
    sh = ml.Shape(detail)
    # plan run of the slope (12.15 m) over the 18.4 m above the plinth: the vertical scale is the physics deck / 14.5 m
    assert sh.run == pytest.approx(12.15, abs=0.01)
    assert sh.top_half_t == pytest.approx(7.05, abs=0.01)
    angle = math.degrees(math.atan2(sh.deck_z - sh.plinth_z, sh.run))
    assert 50.0 < angle < 62.0  # 48 deg measured at 14.5 m, steeper when stretched to the physics deck


def test_ring_outer_and_opening(detail, mount):
    pts = [v for verts, _ in mount.nodes["hardware"] for v in verts if abs(v[2] - detail["mount"]["deck_height_m"]) < 1e-9]
    radii = [math.hypot(v[0], v[1]) for v in pts]
    assert min(radii) == pytest.approx(4.5, abs=0.01) and max(radii) == pytest.approx(5.2 + 0.0, abs=0.7)
    ring_top = [math.hypot(v[0], v[1]) for verts, _ in mount.nodes["hardware"] for v in verts
                if abs(v[2] - detail["mount"]["deck_height_m"]) < 1e-9 and abs(v[2] - 20.0) < 1e-9]
    assert max(r for r in ring_top if r < 5.3) == pytest.approx(5.2, abs=0.01)
    clamp_tops = {round(v[2], 6) for verts, _ in mount.nodes["hardware"] for v in verts
                  if abs(v[2] - (detail["mount"]["deck_height_m"] + detail["mount"]["clamp_height_m"])) < 1e-9}
    assert clamp_tops, "clamps on the ring"
    assert detail["mount"]["clamp_count"] == 12


def test_two_exit_ramps_opposite_and_perpendicular_to_the_tower_line(detail, trench):
    lo, hi = trench.extent("concrete")
    # ramps on both +-Y sides (across), none along the mount-tower line (X)
    assert lo[1] < -40.0 and hi[1] > 40.0
    assert max(abs(lo[0]), abs(hi[0])) == pytest.approx(10.0, abs=0.01)  # 20 m wide
    sh = ml.Shape(detail)
    mid = 0.5 * (sh.half_a + ml.ramp_end_m(detail))
    assert mid == pytest.approx(detail["trench"]["centre_m"], abs=0.5)  # 33 m from the axis
    assert detail["trench"]["width_m"] == pytest.approx(20.0, abs=0.01)
    floor = [v for verts, _ in trench.nodes["concrete"] for v in verts]
    assert min(v[2] for v in floor) == pytest.approx(-detail["mount"]["pit_depth_m"], abs=1e-9)
    assert max(v[2] for v in floor) == pytest.approx(detail["trench"]["wall_height_m"], abs=1e-9)  # curb above the ground


def test_tunnel_mouths_open_in_the_side_faces_only(detail, mount):
    w = detail["mount"]["tunnel_width_m"] / 2.0
    # no shell polygon covers the mouth: sample the face plane in the mouth and test it is not inside a shell polygon
    sh = ml.Shape(detail)
    for sy in (-1.0, 1.0):
        z = 4.0
        y = sy * sh.hy(z)
        covering = [1 for verts, _ in mount.nodes["shell"]
                    if all(abs(v[1] - verts[0][1]) < 1e-6 or True for v in verts)
                    and min(v[1] for v in verts) * sy > 0 and _inside_xz(verts, 0.0, z) and abs(verts[0][1]) > 10.0]
        assert covering == [], "the mouth in the face must be open"
    assert w * 2.0 == pytest.approx(12.0)


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
    h = _physics()["mount_height_m"]
    bad = ml.stack_violations(mount, h, 4.5, _ring_radii(), _engine())
    assert bad == [], f"{len(bad)} mount points inside the stack, first {bad[:3]}"


def test_clearance_test_catches_a_wrong_ring(detail):
    narrow = dict(detail)
    narrow["mount"] = dict(detail["mount"], ring_opening_diameter_m=8.0)
    bad = ml.stack_violations(ml.mount(narrow), _physics()["mount_height_m"], 4.5, _ring_radii(), _engine())
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


def test_hoods_flank_the_deck_on_the_tower_side(detail, mount):
    xs = [v[0] for verts, _ in mount.nodes["hood"] for v in verts if v[2] > detail["mount"]["deck_height_m"]]
    assert max(xs) <= ml.Shape(detail).top_half_t
    ys = {math.copysign(1.0, v[1]) for verts, _ in mount.nodes["hood"] for v in verts if abs(v[1]) > 5.0}
    assert ys == {-1.0, 1.0}


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
