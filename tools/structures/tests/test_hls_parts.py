"""HLS hull parts (plan 2026-11-04, Task 5): the landing thrusters on the config positions, the white skin, the band / window / bay overlay, the
nose port (visual only), the dorsal hardware. The glTF checks skip without data/models/starship_hls."""
import json
import math
import sys
import tomllib
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "blender"))
import build_models  # noqa: E402
import hls_layout as hl  # noqa: E402
import hls_legs_layout as ll  # noqa: E402
import hull_textures  # noqa: E402
import test_dorsal_hardware as td  # noqa: E402
import vehicle_spec  # noqa: E402

HLS = vehicle_spec.ROOT / "config" / "vehicles" / "starship_hls.toml"
MODELS = build_models.ROOT / "data" / "models" / "starship_hls"
TOL_M = 0.01
TOL_AXIS_DEG = 0.5


def _config_ship():
    with open(HLS, "rb") as f:
        return tomllib.load(f)["stage"][0]


def _landing_group():
    return next(g for g in _config_ship()["engine_group"] if g["id"] == "landing")


def _config_thrusters():
    """Position and thrust axis of every landing thruster, from the config alone (the core's resolution: vehicle.cpp)."""
    g = _landing_group()
    c = math.radians(g["cant_deg"])
    out = []
    for k in range(g["count"]):
        a = 2.0 * math.pi * k / g["count"]
        out.append(([g["station_m"], g["ring_radius_m"] * math.cos(a), g["ring_radius_m"] * math.sin(a)],
                    [math.cos(c), -math.sin(c) * math.cos(a), -math.sin(c) * math.sin(a)]))
    return out


def _hls_gltf(name):
    return td._load_gltf_at(MODELS / name)


def test_spec_holds_the_config_thrusters():
    spec = vehicle_spec.load_starship("starship_hls")
    g = _landing_group()
    assert (g["count"], g["station_m"]) == (12, 40.0)  # the config this task draws: unchanged (user ruling 2026-11-04)
    thr = spec["hls_parts"]["thrusters"]
    assert len(thr) == 12
    for t, (pos, axis) in zip(thr, _config_thrusters()):
        assert t["position_m"] == pytest.approx(pos, abs=1e-9) and t["axis"] == pytest.approx(axis, abs=1e-9)
        assert t["exit_diameter_m"] == 0.45  # config/engines/hls_landing_thruster.toml
    assert spec["white_paint"]["metallic"] == 0.0
    assert "hls_parts" not in vehicle_spec.load_starship("starship_tanker") and "white_paint" not in vehicle_spec.load_starship("starship_depot")


def test_detail_values_are_tagged_and_validated():
    assert vehicle_spec.value_lines_without_tag(vehicle_spec.VEHICLES / "detail" / "starship_hls.toml") == []
    detail = vehicle_spec.load_starship("starship_hls")["detail"]
    bad = dict(detail, windows={k: v for k, v in detail["windows"].items() if k != "count"})
    with pytest.raises(vehicle_spec.VehicleSpecError, match="windows"):
        vehicle_spec.hls_parts_block("x", "hls", _config_ship(), bad)
    with pytest.raises(vehicle_spec.VehicleSpecError, match="albedo"):
        vehicle_spec.white_paint_block("x", dict(detail["white_paint"], albedo=[1.2, 0.5, 0.5]))


def test_thruster_exits_on_config_positions():
    """The exit plane centre is on the config engine position (0.01 m), the axis on its direction (0.5 deg), nothing lies behind the exit plane."""
    doc, buffers = _hls_gltf("hls_parts.gltf")
    for i, (pos, axis) in enumerate(_config_thrusters()):
        pts = np.array(td._mesh_points(doc, buffers, f"thruster_{i}"), dtype=float)
        axis = np.array(axis)
        along = (pts - np.array(pos)) @ axis
        assert along.min() > -1e-3, f"thruster_{i}: geometry behind the exit plane"
        ring = np.unique(np.round(pts[np.abs(along) < 1e-3], 4), axis=0)  # the exit plane's vertices (bell lip, pod face), UV-seam duplicates merged
        assert len(ring) >= 24
        assert np.linalg.norm(ring.mean(axis=0) - pos) < TOL_M, f"thruster_{i}: exit centre"
        normal = np.linalg.svd(ring - ring.mean(axis=0))[2][-1]
        assert math.degrees(math.acos(min(1.0, abs(float(normal @ axis))))) < TOL_AXIS_DEG, f"thruster_{i}: axis"
        assert np.max(np.linalg.norm(ring - ring.mean(axis=0), axis=1)) == pytest.approx(0.5 * 0.45 + 0.095, abs=0.01)  # the pod face, not a wrong ring


def test_manifest_thruster_exit_is_the_config_position():
    manifest = json.loads((MODELS / "manifest.json").read_text(encoding="utf-8")) if (MODELS / "manifest.json").is_file() else pytest.skip("no manifest")
    for entry, (pos, axis) in zip(manifest["thruster_exit_m"], _config_thrusters()):
        assert entry["exit_m"] == pytest.approx(pos, abs=1e-9) and entry["axis"] == pytest.approx(axis, abs=1e-9)
    assert [e["node"] for e in manifest["thruster_exit_m"]] == [f"thruster_{i}" for i in range(12)]
    assert manifest["hls_nose_port"]["visual_only"] is True
    assert manifest["look"]["skin_drawn"] == "white" and "white" in manifest["look"]["steel_map"] and "overlay" in manifest["look"]["steel_map"]


def test_the_config_has_no_nose_port_so_the_drawn_one_is_visual_only():
    ports = _config_ship().get("docking_port", [])
    assert [p["id"] for p in ports] == ["transfer_dock"]  # only the dorsal dock: the nose port is a drawing, no physics
    spec = vehicle_spec.load_starship("starship_hls")
    assert [p["id"] for p in spec["dorsal"]["ports"]] == ["transfer_dock"] and spec["dorsal"]["ports"][0]["soft_capture"] == "active"
    assert len(spec["dorsal"]["connectors"]) == 2


def test_nose_port_sits_on_the_tip():
    doc, buffers = _hls_gltf("hls_parts.gltf")
    pts = np.array(td._mesh_points(doc, buffers, "nose_port"), dtype=float)
    d = vehicle_spec.load_starship("starship_hls")["detail"]["nose_port"]
    length = _config_ship()["length_m"]
    assert pts[:, 0].max() == pytest.approx(length + d["height_m"], abs=1e-3)
    assert np.hypot(pts[:, 1], pts[:, 2]).max() == pytest.approx(d["radius_m"], abs=1e-3)
    assert abs(pts[:, 1].max() + pts[:, 1].min()) < 2e-3 and abs(pts[:, 2].max() + pts[:, 2].min()) < 2e-3  # centred on the station axis


def test_thruster_profile_ends_in_the_exit_radius():
    spec = vehicle_spec.load_starship("starship_hls")
    d, t = spec["hls_parts"]["drawing"]["landing_thruster"], spec["hls_parts"]["thrusters"][0]
    contour = hl.bell_contour(d, t)
    assert contour[0] == (0.0, pytest.approx(0.225)) and contour[-1][1] == pytest.approx(0.5 * d["throat_diameter_m"])
    assert all(a[1] >= b[1] for a, b in zip(contour, contour[1:]))  # the flow wall narrows to the throat


def test_white_map_is_faint_plain_paint():
    spec = vehicle_spec.load_starship("starship_hls")
    radius = 0.5 * spec["stages"]["ship"]["diameter_m"]
    hull = spec["detail"]["hull"]
    img = hull_textures.bake_white(radius, hull, spec["white_paint"])
    assert img.shape == (hull_textures.SIZE_V, hull_textures.SIZE_U, 4)
    assert np.array_equal(img, hull_textures.bake_white(radius, hull, spec["white_paint"]))  # deterministic
    a = img[..., 3] / 255.0
    assert a.max() == pytest.approx(spec["white_paint"]["groove_darkening"], abs=0.005)  # faint lines, no weld or tape darkening
    assert (a > 0.5 * spec["white_paint"]["groove_darkening"]).mean() < 0.05  # lines only, a few percent of the area
    slope = (img[..., :2] / 255.0 - 0.5) * hull_textures.SLOPE_RANGE
    assert np.abs(slope).max() < 0.2  # no 0.24 weld bead, no quilting slope


def test_overlay_places_band_windows_and_bays():
    spec = vehicle_spec.load_starship("starship_hls")
    radius = 0.5 * spec["stages"]["ship"]["diameter_m"]
    hls = spec["hls_parts"]
    img = hull_textures.bake_overlay(radius, hls)
    band, win = hls["drawing"]["solar_band"], hls["drawing"]["windows"]
    h, w, _ = img.shape
    circ = 2.0 * math.pi * radius

    def px(arc_m, station_m):
        return img[int((station_m - band["station_from_m"]) / (band["station_to_m"] - band["station_from_m"]) * h) % h, int(arc_m / circ * w) % w]

    assert (img[..., 3] > 250).mean() > 0.99  # the band fills the map
    cell = px(0.5 * circ / band["columns"] + 0.2, band["station_from_m"] + 0.5 * band["row_pitch_m"] + 2.0)
    assert max(cell[:3]) < 80  # dark cells
    glass = px(math.radians(win["azimuth_deg"]) * radius + 0.5 * win["pitch_m"], win["station_m"])
    frame = px(math.radians(win["azimuth_deg"]) * radius + 0.5 * win["pitch_m"] + 0.5 * win["width_m"] + 0.02, win["station_m"])
    assert max(glass[:3]) < 20 and min(frame[:3]) > 90  # glass darker than its frame
    for t in hls["thrusters"][:3]:  # the bay's rim: lighter than the bay inside it
        a = math.atan2(t["position_m"][2], t["position_m"][1])
        r = hls["drawing"]["landing_thruster"]["bay_radius_m"]
        rim = px(a * radius + r + 0.5 * hull_textures.BAY_RIM_M, t["position_m"][0])
        inside = px(a * radius + 0.8 * r, t["position_m"][0])
        assert min(rim[:3]) > max(inside[:3]) + 20


def test_overlay_and_manifest_hash_inputs_leave_block3_alone():
    block3 = vehicle_spec.load_starship("starship_block3")
    assert "white_paint" not in block3 and "hls_parts" not in block3
    text = vehicle_spec.dumps(block3)
    assert "hls_parts" not in text and "white_paint" not in text


def test_dorsal_hardware_is_drawn_for_the_hls():
    manifest = json.loads((MODELS / "manifest.json").read_text(encoding="utf-8")) if (MODELS / "manifest.json").is_file() else pytest.skip("no manifest")
    assert manifest["dorsal_stage_id"] == "hls" and manifest["dorsal_mechanism"] == "probe"
    ship = _config_ship()
    assert manifest["dorsal_elements_m"] == ship["docking_port"][0]["elements_m"]
    assert sorted(manifest["dorsal_connectors"]) == ["connector_ch4", "connector_lox"]
    doc, buffers = _hls_gltf("dorsal_hardware.gltf")
    assert {n["name"] for n in doc["nodes"] if n.get("name", "").startswith("probe_")} == {f"probe_{i}" for i in range(4)}


def _config_legs():
    return _config_ship()["landing_legs"]


def _bottom_centre(pts):
    low = pts[:, 0].min()
    sel = pts[pts[:, 0] < low + 0.001]
    return 0.5 * (sel.min(axis=0) + sel.max(axis=0))


def test_spec_legs_are_the_config_legs():
    legs = vehicle_spec.load_starship("starship_hls")["hls_parts"]["legs"]
    cfg = _config_legs()
    assert (legs["azimuths_deg"], legs["footprint_radius_m"], legs["foot_station_m"], legs["stroke_m"]) == (
        cfg["azimuths_deg"], cfg["footprint_radius_m"], cfg["foot_station_m"], cfg["stroke_m"])
    assert (len(legs["azimuths_deg"]), legs["footprint_radius_m"], legs["foot_station_m"], legs["stroke_m"]) == (6, 12.0, -1.5, 0.6)  # the config this task draws (unchanged)
    assert vehicle_spec.value_lines_without_tag(vehicle_spec.VEHICLES / "detail" / "starship_hls.toml") == []


def test_feet_stand_on_the_config_foot_points():
    """The pad's bottom face centre of every `foot_<i>` is on (station, radius, azimuth) of the config, 0.01 m; the leg reaches from the foot's sleeve to the hull."""
    doc, buffers = _hls_gltf("hls_parts.gltf")
    cfg = _config_legs()
    for i, az in enumerate(cfg["azimuths_deg"]):
        a = math.radians(az)
        want = np.array([cfg["foot_station_m"], cfg["footprint_radius_m"] * math.cos(a), cfg["footprint_radius_m"] * math.sin(a)])
        foot = np.array(td._mesh_points(doc, buffers, f"foot_{i}"), dtype=float)
        assert np.linalg.norm(_bottom_centre(foot) - want) < TOL_M
        leg = np.array(td._mesh_points(doc, buffers, f"leg_{i}"), dtype=float)
        radial = np.hypot(leg[:, 1], leg[:, 2])
        assert radial.max() > cfg["footprint_radius_m"] - 0.3 and radial.min() < 5.0  # from the knuckle at the foot to the hinge by the hull
        mount = np.array(td._mesh_points(doc, buffers, f"leg_mount_{i}"), dtype=float)
        assert np.hypot(mount[:, 1], mount[:, 2]).min() < 4.5  # the hinge clevis sits in the skin
        # the sleeve stays clear of the pad at full stroke: the lowest leg point is above the pad's top by the stroke
        assert leg[:, 0].min() > cfg["foot_station_m"] + 0.15 + cfg["stroke_m"]


def test_manifest_has_legs_airlock_and_elevator():
    manifest = json.loads((MODELS / "manifest.json").read_text(encoding="utf-8"))
    cfg = _config_legs()
    legs = manifest["hls_legs"]
    assert legs["stroke_m"] == cfg["stroke_m"] and legs["stroke_axis"] == [1.0, 0.0, 0.0] and len(legs["legs"]) == 6
    doc, _ = _hls_gltf("hls_parts.gltf")
    names = {n["name"] for n in doc["nodes"]}
    assert {"airlock_hatch", "elevator_track", "elevator_basket"} <= names and {f"leg_{i}" for i in range(6)} <= names and {f"foot_{i}" for i in range(6)} <= names
    assert manifest["hls_airlock"]["azimuth_deg"] == manifest["hls_elevator"]["azimuth_deg"] == 150.0
    # the elevator stands between two legs (120 and 180), clear of both feet
    assert min(abs(150.0 - az) for az in cfg["azimuths_deg"]) == 30.0
    assert {f"pusher_{i}" for i in range(6)} <= names and {f"leg_mount_{i}" for i in range(6)} <= names
    spec = vehicle_spec.load_starship("starship_hls")["hls_parts"]
    assert legs["rig"] == pytest.approx(ll.leg_rig(spec["drawing"]["landing_leg"], spec["legs"]))


def _drawn(prims):
    """Points of the layout primitives (tube end rings, box corners), body frame."""
    pts = []
    for p in prims:
        if p[0] == "tube":
            _, a, b, r0, r1 = p
            a, b = np.array(a), np.array(b)
            z = (b - a) / np.linalg.norm(b - a)
            x = np.cross(z, [1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.cross(z, [0.0, 1.0, 0.0])
            x /= np.linalg.norm(x)
            y = np.cross(z, x)
            for c, r in ((a, r0), (b, r1)):
                pts += [c + r * (math.cos(t) * x + math.sin(t) * y) for t in np.linspace(0.0, 2.0 * math.pi, 32, endpoint=False)]
        else:
            _, c, ex, ey, ez, h = p
            pts += [np.array(c) + sx * h[0] * np.array(ex) + sy * h[1] * np.array(ey) + sz * h[2] * np.array(ez)
                    for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    return pts


def test_leg_rig_deploys_from_along_the_hull_to_the_config_foot():
    """Task 6b: stowed (0) every leg and foot point lies within the skin + stowed_thickness_m (and outside the skin), above the hull base and
    below the landing thrusters; deployed (1) the pose is the identity (the foot on the config's point); the pusher spans anchor to leg."""
    spec = vehicle_spec.load_starship("starship_hls")["hls_parts"]
    d, legs = spec["drawing"]["landing_leg"], spec["legs"]
    rig = ll.leg_rig(d, legs)
    hull = legs["hull_radius_m"]
    for az in legs["azimuths_deg"]:
        a = math.radians(az)
        drawn = _drawn(ll.leg_prims(d, legs, a) + ll.foot_prims(d, legs, a))
        stowed = np.array([ll.pose_point(rig, a, 0.0, p) for p in drawn])
        r = np.hypot(stowed[:, 1], stowed[:, 2])
        assert r.max() <= hull + d["stowed_thickness_m"] and r.min() > hull
        assert stowed[:, 0].min() > 0.0 and stowed[:, 0].max() < 12.0
        foot = ll.foot_point(legs, a)
        assert np.linalg.norm(np.array(ll.pose_point(rig, a, 1.0, foot)) - np.array(foot)) < 1e-9
        anchor, on_leg = ll.pusher_ends(rig, a, 0.0)
        assert 0.0 < math.dist(anchor, on_leg) < math.dist(*ll.pusher_ends(rig, a, 1.0))
    assert rig["deployed_angle_deg"] == pytest.approx(97.52, abs=0.01) and rig["stowed_angle_deg"] == 0.0

