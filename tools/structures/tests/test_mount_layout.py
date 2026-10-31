"""Mount, trench and hardstand layout: the block, the annex, the one-width tunnel, the floor, the foot, stack clearance."""
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


def _shell_polys(mount):
    return [verts for verts, _ in mount.nodes["shell"]]


def _unit_normal(verts):
    n = ml.newell_normal(verts)
    length = math.sqrt(sum(c * c for c in n))
    return tuple(c / length for c in n)


def _inside_xz(verts, x, z):
    """Point (x, z) inside the polygon projected on the XZ plane."""
    inside = False
    for i, a in enumerate(verts):
        b = verts[(i + 1) % len(verts)]
        if (a[2] > z) != (b[2] > z) and x < (b[0] - a[0]) * (z - a[2]) / (b[2] - a[2]) + a[0]:
            inside = not inside
    return inside


def _covering(mount, nodes, y_sign, x, z):
    """Polygons of `nodes` across the portal plane in front of the tunnel at (x, z)."""
    out = []
    for node in nodes:
        for verts, _ in mount.nodes.get(node, []):
            if min(v[1] * y_sign for v in verts) > 5.0 and _inside_xz(verts, x, z):
                out.append(node)
    return out


def test_mount_keys_are_positive_and_tagged(detail):
    assert detail["mount"]["deck_height_m"] > 0.0 and detail["trench"]["width_m"] > 0.0
    assert detail_file.value_lines_without_tag(DETAIL) == []


def test_ring_top_is_the_measured_deck(detail, mount):
    h = detail["mount"]["measured_height_m"]
    assert detail["mount"]["deck_height_m"] == pytest.approx(h, abs=0.01)
    tops = [v[2] for verts, _ in mount.nodes["hardware"] for v in verts]
    assert any(abs(z - h) < 1e-9 for z in tops)


def test_ring_top_is_the_physics_deck(detail):
    with open(ROOT / "config" / "launch_sites.toml", "rb") as f:
        site = next(s for s in tomllib.load(f)["site"] if s.get("tower") and s["id"].startswith("starbase"))
    assert detail["mount"]["deck_height_m"] == pytest.approx(site["tower"]["mount_height_m"], abs=0.01)


def test_block_walls_are_vertical_on_the_tower_side(detail, mount):
    sh = ml.Shape(detail)
    assert sh.deck_z == pytest.approx(detail["mount"]["deck_height_m"] - detail["mount"]["deck_drop_m"])
    facing = [v for v in _shell_polys(mount) if _unit_normal(v)[0] > 0.5 and all(p[0] > 0.0 for p in v)]
    assert facing, "a wall on the tower side"
    for verts in facing:
        assert all(abs(p[0] - sh.x_front) < 1e-9 for p in verts)  # one vertical plane
        assert abs(_unit_normal(verts)[2]) < 1e-9
    wall = [p for v in facing for p in v]
    assert min(p[2] for p in wall) == pytest.approx(0.0) and max(p[2] for p in wall) == pytest.approx(sh.deck_z)
    assert max(abs(p[1]) for p in wall) == pytest.approx(detail["mount"]["block_half_y_m"])
    deck = [p for v in _shell_polys(mount) for p in v if abs(p[2] - sh.deck_z) < 1e-9]  # flat, plain deck
    assert max(p[0] for p in deck) == pytest.approx(sh.x_front) and min(p[0] for p in deck) == pytest.approx(-sh.x_back)


def test_annex_only_away_from_the_tower(detail, mount):
    sh = ml.Shape(detail)
    lo, hi = mount.extent()
    assert hi[0] == pytest.approx(sh.x_front, abs=1e-9)  # nothing beyond the block on the tower side
    assert min(p[0] for v in _shell_polys(mount) for p in v) >= -sh.annex_x - 1e-9  # (decals and vent plates stand proud)
    for verts in _shell_polys(mount):
        xs = [p[0] for p in verts]
        if min(xs) < -sh.x_back - 1e-9:
            assert max(xs) <= -sh.x_back + 1e-9  # an annex polygon never reaches over the block
    ground = [p for v in _shell_polys(mount) for p in v if p[2] == 0.0 and p[0] < -sh.x_back + 1e-9]
    assert max(abs(p[1]) for p in ground) == pytest.approx(detail["mount"]["annex_half_y_m"])
    far = [p for p in ground if p[0] < -sh.x_back - 1e-9]
    assert min(p[0] for p in far) == pytest.approx(-detail["mount"]["annex_end_x_m"])
    assert all(abs(abs(p[1]) - detail["mount"]["annex_tip_half_y_m"]) < 1e-9 for p in far if p[0] < -sh.x_back - 1e-6)
    assert detail["mount"]["annex_tip_half_y_m"] < detail["mount"]["annex_half_y_m"]  # a trapezoid, narrowing away
    for node in ("door", "vent"):
        assert all(max(p[0] for p in verts) < -sh.x_back + 0.2 for verts, _ in mount.nodes[node])


def test_tunnel_has_one_width_end_to_end(detail, mount, trench):
    sh = ml.Shape(detail)
    hw = 0.5 * detail["trench"]["width_m"]
    assert sh.half_w == hw
    floor = [p for v, _ in trench.nodes["concrete"] for p in v]
    assert {round(abs(p[0]), 9) for p in floor} == {hw}  # the floor: under the block and on both ramps
    inside = [p for v, _ in mount.nodes["interior"] for p in v if abs(abs(p[0]) - hw) < 1e-9 and abs(p[1]) <= sh.half_y + 1e-9]
    assert inside, "the tunnel's walls inside the block stand on the same X"
    inner = [p for v, _ in trench.nodes["wall"] for p in v if abs(p[0]) < hw + 1e-9]
    assert {round(abs(p[0]), 9) for p in inner} == {hw}  # the ramp walls' inner faces continue it: no flare
    for sy in (-1.0, 1.0):  # the portal is a rectangle of the tunnel's width in the block's end face
        for z in (0.5, 0.5 * sh.open_top, sh.open_top - 0.1):
            for frac in (-0.95, 0.0, 0.95):
                assert _covering(mount, ("shell", "plinth", "hood"), sy, frac * hw, z) == []
        assert _covering(mount, ("shell",), sy, 0.0, sh.open_top + 1.0)  # the block above the opening
        assert _covering(mount, ("shell",), sy, hw + 0.5, 3.0)  # and beside it


def test_floor_is_straight_and_level_under_the_ring(detail, trench, mount):
    sh = ml.Shape(detail)
    zf = -detail["mount"]["pit_depth_m"]
    assert sh.floor_z == zf
    floor = [verts for verts, _ in trench.nodes["concrete"]]
    under = [v for v in floor if max(abs(p[1]) for p in v) <= sh.half_y + 1e-9]
    assert under and all(abs(p[2] - zf) < 1e-9 for v in under for p in v)  # level under the block
    ends = ml.ramp_end_m(detail)
    for sy in (-1.0, 1.0):
        ramp = [v for v in floor if min(p[1] * sy for p in v) >= sh.half_y - 1e-9 and max(p[1] * sy for p in v) > sh.half_y + 1.0]
        assert len(ramp) == 1  # one straight plane per ramp: no ridge, no curve
        for p in ramp[0]:
            assert p[2] == pytest.approx(zf * (1.0 - (abs(p[1]) - sh.half_y) / (ends - sh.half_y)))  # linear, 0 at the end
    assert max(p[2] for v in floor for p in v) == pytest.approx(0.0)
    assert "deflector" not in trench.nodes and "deflector" not in mount.nodes
    assert "ridge_top_m" not in detail["mount"]


def test_trench_wall_falls_with_the_annex_then_runs_level(detail, trench):
    sh = ml.Shape(detail)
    lip = detail["trench"]["wall_height_m"]  # 0.5 m (user estimate)
    assert sh.lip_y == detail["mount"]["annex_half_y_m"]  # the slope ends where the annex footprint ends
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            side = [p for v, _ in trench.nodes["wall"] for p in v if p[0] * sx > 0.0 and p[1] * sy > 0.0 and p[2] > 0.0]
            assert max(p[2] for p in side) == pytest.approx(sh.deck_z)  # deck height at the block edge ...
            for p in side:
                y = abs(p[1])
                if y <= sh.lip_y + 1e-9:  # ... falling linearly to the 0.5 m top at the annex end
                    assert p[2] <= sh.deck_z - (sh.deck_z - lip) * (y - sh.half_y) / (sh.lip_y - sh.half_y) + 1e-6
                else:  # then level at 0.5 m to the ramp end
                    assert p[2] == pytest.approx(lip)
            at_lip = [p for p in side if abs(abs(p[1]) - sh.lip_y) < 1e-9]
            assert at_lip and all(p[2] == pytest.approx(lip) for p in at_lip)
            ends = [p for p in side if abs(abs(p[1]) - ml.ramp_end_m(detail)) < 1e-9]
            assert ends and all(p[2] == pytest.approx(lip) for p in ends)  # the floor has come up to the ground there
    assert lip == pytest.approx(0.5)
    # The outer faces are flush with the deck plate's side edges (user 2026-10-31), both sides, 0.01 m.
    xs = [p[0] for v, _ in trench.nodes["wall"] for p in v]
    assert min(xs) == pytest.approx(-detail["mount"]["block_x_back_m"], abs=0.01)
    assert max(xs) == pytest.approx(detail["mount"]["block_x_tower_m"], abs=0.01)
    inner = sorted({round(abs(x), 6) for x in xs} - {round(sh.half_w, 6)})
    assert 0.5 * detail["trench"]["width_m"] in {abs(x) for x in xs}  # inner faces on the tunnel's width


def test_two_exit_ramps_opposite_and_perpendicular_to_the_tower_line(detail, trench):
    lo, hi = trench.extent("concrete")
    assert lo[1] == pytest.approx(-41.5, abs=0.01) and hi[1] == pytest.approx(41.5, abs=0.01)
    assert max(abs(lo[0]), abs(hi[0])) == pytest.approx(0.5 * detail["trench"]["width_m"], abs=0.01)
    sh = ml.Shape(detail)
    assert 0.5 * (sh.half_y + ml.ramp_end_m(detail)) == pytest.approx(detail["trench"]["centre_m"], abs=0.5)
    assert lo[2] == pytest.approx(-detail["mount"]["pit_depth_m"], abs=1e-9)
    assert hi[2] == pytest.approx(0.0, abs=1e-9)


def test_ring_outer_and_opening(detail, mount):
    h = detail["mount"]["deck_height_m"]
    radii = [math.hypot(v[0], v[1]) for verts, _ in mount.nodes["hardware"] for v in verts if abs(v[2] - h) < 1e-9]
    assert min(radii) == pytest.approx(4.5, abs=0.01)
    assert max(r for r in radii if r < 5.3) == pytest.approx(5.2, abs=0.01)
    clamp_tops = {round(v[2], 6) for verts, _ in mount.nodes["hardware"] for v in verts
                  if abs(v[2] - (h + detail["mount"]["clamp_height_m"])) < 1e-9}
    assert clamp_tops, "clamps on the ring"
    assert detail["mount"]["clamp_count"] == 20


def test_front_face_has_a_door_group_and_no_deck_cabinets(detail, mount):
    doors = mount.nodes["door"]
    assert len(doors) == 2 and all(len(verts) == 4 for verts, _ in doors)  # no arches
    assert all(min(v[2] for v in verts) == 0.0 for verts, _ in doors)  # at the annex's foot
    assert not any(k.startswith("qd_") and k not in ("qd_count", "qd_head_radius_m", "qd_head_depth_m", "qd_head_width_m",
                                                      "qd_head_height_m") for k in detail["mount"])
    above = [p for v, _ in mount.nodes["hardware"] for p in v if p[2] > detail["mount"]["deck_height_m"] + 1.0]
    assert above and max(p[2] for p in above) <= detail["mount"]["deck_height_m"] + detail["mount"]["qd_head_height_m"] + 1e-9


def test_qd_heads_stand_on_the_ring_under_their_hoods(detail, mount):
    m = detail["mount"]
    for turn in ml.hood_yaws(detail):
        heads = [p for v, _ in mount.nodes["hardware"] for p in v
                 if p[2] > m["deck_height_m"] + 1.0 and abs(math.atan2(p[1], p[0]) % (2 * math.pi) - (math.pi + turn)) < 0.5]
        assert heads
        assert min(math.hypot(p[0], p[1]) for p in heads) >= 4.5 + 0.3  # off the hull
        assert max(math.hypot(p[0], p[1]) for p in heads) < -m["hood_root_x_m"] + 0.1  # inside the hood's root


def test_no_bar_outside_the_apron(detail, trench):
    lo, hi = trench.extent("wall")
    assert lo[0] == pytest.approx(-detail["mount"]["block_x_back_m"], abs=0.01)
    assert hi[0] == pytest.approx(detail["mount"]["block_x_tower_m"], abs=0.01)
    assert hi[1] == pytest.approx(ml.ramp_end_m(detail))  # the walls end with the ramp: no separate long wall


def test_hardstand_surrounds_the_site_and_leaves_the_pit_open(detail, trench):
    lo, hi = trench.extent("hardstand")
    sh = ml.Shape(detail)
    assert lo[2] == hi[2] == pytest.approx(detail["trench"]["hardstand_lift_m"])
    assert lo[0] < -sh.annex_x - 10.0 and hi[0] > detail["trench"]["foot_x_m"] + 30.0
    assert lo[1] < -ml.ramp_end_m(detail) - 10.0 and hi[1] > ml.ramp_end_m(detail) + 10.0
    for verts, _ in trench.nodes["hardstand"]:
        cx = sum(v[0] for v in verts) / len(verts)
        cy = sum(v[1] for v in verts) / len(verts)
        assert not (-sh.x_back < cx < sh.x_front and abs(cy) < sh.half_y), "hardstand over the block"
        assert not (sh.wall_out[-1.0] < cx < sh.wall_out[1.0] and abs(cy) < ml.ramp_end_m(detail)), "hardstand over a ramp"


def test_tower_foot_is_at_the_measured_place_and_off_the_mount(detail, trench):
    t = detail["trench"]
    assert math.hypot(t["foot_x_m"], t["foot_y_m"]) == pytest.approx(31.7, abs=0.2)  # DSC05957, user 2026-10-31
    yaw = math.radians(detail["mount"]["plan_yaw_deg"])
    # the square yaw puts the foot on the line from the ring along the physics tower direction
    assert -t["foot_x_m"] * math.sin(yaw) + t["foot_y_m"] * math.cos(yaw) == pytest.approx(0.0, abs=0.1)
    assert abs(detail["mount"]["plan_yaw_deg"] - 38.0) <= 6.0
    lo, hi = trench.extent("foot")
    gap = lo[0] - ml.Shape(detail).x_front
    assert gap == pytest.approx(7.0, abs=1.0)  # cut foot, cut mount: about 7 m between the two vertical faces
    assert hi[2] == pytest.approx(t["foot_roof_height_m"])
    near = [v for v, _ in trench.nodes["foot"] if all(abs(p[0] - lo[0]) < 1e-9 for p in v)]
    assert near and all(abs(_unit_normal(v)[2]) < 1e-9 for v in near)  # the cut face is vertical ...
    assert max(p[2] for v in near for p in v) == hi[2]  # ... up to the roof


def test_bells_clear_the_flat_floor_in_the_ignition_sag(detail, trench):
    """Every bell exit stays 1 m above the tunnel's level floor, on the ring top and 1 m lower."""
    sh = ml.Shape(detail)
    bell = _engine()["bell_length_m"]
    for sag_m in (0.0, 1.0):
        assert detail["mount"]["deck_height_m"] - sag_m - bell > sh.floor_z + 1.0
    floor = [p for v, _ in trench.nodes["concrete"] for p in v if abs(p[1]) <= sh.half_y]
    assert max(p[2] for p in floor) == sh.floor_z


def test_booster_stack_clears_deck_ring_and_qd(detail, mount, trench):
    """Hull above the ring top and every bell of the booster's 33 engines below it clear the mount and the floor."""
    h = detail["mount"]["deck_height_m"]
    for surf in (mount, trench):
        bad = ml.stack_violations(surf, h, 4.5, _ring_radii(), _engine())
        assert bad == [], f"{len(bad)} points inside the stack, first {bad[:3]}"


def test_clearance_test_catches_a_wrong_ring(detail):
    narrow = dict(detail)
    narrow["mount"] = dict(detail["mount"], ring_opening_diameter_m=8.0)
    bad = ml.stack_violations(ml.mount(narrow), detail["mount"]["deck_height_m"], 4.5, _ring_radii(), _engine())
    assert bad, "a 8 m opening must hit the hull and the outer engines"


def test_hoods_and_qd_heads_stand_off_the_hull(detail, mount):
    hull = 4.5
    for verts, _ in mount.nodes["hood"]:
        for v in verts:
            if v[2] > detail["mount"]["deck_height_m"] + 1e-6:
                assert math.hypot(v[0], v[1]) >= hull + 1.0
    qd_heads = [math.hypot(v[0], v[1]) for verts, _ in mount.nodes["hardware"] for v in verts
                if v[2] > detail["mount"]["deck_height_m"] + 1.0]
    assert min(qd_heads) >= hull + 0.3


def test_hoods_are_split_about_the_line_away_from_the_tower(detail, mount):
    m = detail["mount"]
    sh = ml.Shape(detail)
    above = [v for verts, _ in mount.nodes["hood"] for v in verts if v[2] > m["deck_height_m"]]
    azimuths = [math.degrees(math.atan2(v[1], v[0])) % 360.0 for v in above]
    axis = 180.0 + m["plan_yaw_deg"]
    assert min(azimuths) > axis - 60.0 and max(azimuths) < axis + 60.0  # none toward the tower
    assert m["hood_split_deg"] == pytest.approx(70.0, abs=10.0)
    assert any(a < axis - 20.0 for a in azimuths) and any(a > axis + 20.0 for a in azimuths)
    lowest = min(v[2] for verts, _ in mount.nodes["hood"] for v in verts if math.hypot(v[0], v[1]) > -m["hood_front_x_m"] - 0.5)
    assert lowest == pytest.approx(sh.back_z(m["hood_front_x_m"]), abs=0.1)  # the long hood's box stands on the annex slope
    # the short hood ends inside the block's deck
    deck_edge_y = sh.half_y
    short = [v for verts, _ in mount.nodes["hood"] for v in verts if v[2] > m["deck_height_m"] and v[1] < -1.0]
    assert short and max(abs(v[1]) for v in short) <= deck_edge_y + 1e-9


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


def _compass_to_mount(detail, east, north):
    """Compass offsets (m east / north of the ring centre) in the mount frame: X at bearing azimuth + yaw, Y 90 deg clockwise of it."""
    bearing = math.radians(detail["site_tower"]["azimuth_deg"] + detail["site_tower"]["plan_yaw_deg"])
    return (east * math.sin(bearing) + north * math.cos(bearing),
            east * math.sin(bearing + math.pi / 2) + north * math.cos(bearing + math.pi / 2))


def test_hardstand_is_the_measured_outline(detail):
    """Esri Wayback release 43291 / DSC05957: SW edge bearing 167 (3) deg, 218 (10) m road to the south tip; SE edge 63 (3) deg,
    203 (10) m from the tip; the third side is the road."""
    poly = ml.hardstand_outline(detail)
    assert len(poly) == 3
    tip, sw_end, se_end = poly
    base = math.radians(detail["site_tower"]["azimuth_deg"] + detail["site_tower"]["plan_yaw_deg"])

    def compass(p, q):  # bearing and length of the mount-frame segment p -> q
        dx, dy = q[0] - p[0], q[1] - p[1]
        return (math.degrees(base + math.atan2(dy, dx)) % 360.0, math.hypot(dx, dy))

    bearing, length = compass(sw_end, tip)
    assert bearing == pytest.approx(167.0, abs=3.0) and length == pytest.approx(218.0, abs=10.0)
    bearing, length = compass(tip, se_end)
    assert bearing == pytest.approx(63.0, abs=3.0) and length == pytest.approx(203.0, abs=10.0)
    east, north = -36.0, -74.6  # the tip: 25.9961 N, 97.1584 W against the ring centre 25.99677 N, 97.15804 W
    assert tip == pytest.approx(_compass_to_mount(detail, east, north), abs=8.0)
    # Concrete up to the road: the road side lies beyond the tower foot, and the site lies inside the outline.
    foot = (detail["trench"]["foot_x_m"], detail["trench"]["foot_y_m"])
    assert min(sw_end[0], se_end[0]) > detail["trench"]["foot_far_x_m"]  # the road lies beyond the tower foot
    sh = ml.Shape(detail)
    for p in [(0.0, 0.0), foot, (-sh.annex_x, sh.annex_half_y), (-sh.annex_x, -sh.annex_half_y),
              (0.0, ml.ramp_end_m(detail)), (0.0, -ml.ramp_end_m(detail))]:
        assert _inside_triangle(p, poly), p


def _inside_triangle(p, poly):
    signs = []
    for i, a in enumerate(poly):
        b = poly[(i + 1) % len(poly)]
        signs.append((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= 0.0)
    return all(signs) or not any(signs)
