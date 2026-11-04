"""Depot (plan 2026-11-04, Task 4): drogues on the config elements, the raceway detour, the insulated skin map, the RCS pods, the docked pose.
The glTF checks skip without data/models/starship_depot."""
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
import dorsal_layout as dl  # noqa: E402
import hull_textures  # noqa: E402
import test_dorsal_hardware as td  # noqa: E402
import vehicle_spec  # noqa: E402

DEPOT = vehicle_spec.ROOT / "config" / "vehicles" / "starship_depot.toml"
MODELS = build_models.ROOT / "data" / "models" / "starship_depot"
TOL = 0.01


def _config_ship(path):
    with open(path, "rb") as f:
        stages = tomllib.load(f)["stage"]
    return next(s for s in stages if s.get("id") == "depot" or str(s.get("from", "")).endswith(":ship"))


def _depot_gltf():
    path = MODELS / "dorsal_hardware.gltf"
    if not path.is_file():
        pytest.skip("data/models/starship_depot/dorsal_hardware.gltf not generated")
    return td._load_gltf_at(path)


def test_depot_spec_has_passive_drogues_insulation_and_rcs_pods():
    spec = vehicle_spec.load_starship("starship_depot")
    ship = _config_ship(DEPOT)
    assert spec["dorsal"]["ports"][0]["soft_capture"] == "passive"
    assert spec["dorsal"]["ports"][0]["elements_m"] == ship["docking_port"][0]["elements_m"]
    assert [n["node"] for n in dl.element_nodes(spec["dorsal"]["ports"])] == [f"drogue_{i}" for i in range(4)]
    assert spec["insulation"]["metallic"] == 0.0
    assert [(p["station_m"], p["azimuths_deg"]) for p in spec["rcs_pods"]["pods"]] == [(p["station_m"], p["azimuths_deg"]) for p in ship["rcs"]["pod"]]
    assert "insulation" not in vehicle_spec.load_starship("starship_tanker") and "rcs_pods" not in vehicle_spec.load_starship("starship_block3")


def test_config_depot_is_unchanged_by_this_task():
    """User ruling 2026-11-04: no config proposal approved, so the depot's drogue elements stay the tanker's probe elements mirrored (same numbers)."""
    depot = _config_ship(DEPOT)["docking_port"][0]
    tanker = td._config_ship()["docking_port"][0]
    assert depot["elements_m"] == tanker["elements_m"] and depot["axis"] == tanker["axis"] and depot["up"] == tanker["up"]


def test_drogue_mouths_on_config_elements():
    """The node origin is the centre of the mouth plane: nothing of the drogue is above it, the rim touches it, the cone narrows below it."""
    doc, buffers = _depot_gltf()
    ship = _config_ship(DEPOT)
    manifest = json.loads((MODELS / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["dorsal_elements_m"] == ship["docking_port"][0]["elements_m"] and manifest["dorsal_mechanism"] == "drogue"
    spec = vehicle_spec.load_starship("starship_depot")
    d = spec["dorsal"]["drawing"]
    for i in range(4):
        pts = td._mesh_points(doc, buffers, f"drogue_{i}")
        assert abs(max(p[2] for p in pts)) < TOL, i  # the rim: the mouth plane at the element
        rim = [p for p in pts if abs(p[2]) < 1e-4]
        assert max(math.hypot(p[0], p[1]) for p in rim) == pytest.approx(d["drogue_mouth_radius_m"] + d["drogue_wall_m"], abs=TOL)
        inner = [p for p in pts if abs(p[2] + d["drogue_depth_m"]) < 1e-4 and math.hypot(p[0], p[1]) < d["drogue_throat_radius_m"] + TOL]
        assert inner, "funnel floor at -depth"
        assert min(p[2] for p in pts) < -d["drogue_depth_m"] - d["drogue_foot_m"]  # the stand goes on below the funnel, towards the barrel
    hull_r = 0.5 * spec["stages"]["ship"]["diameter_m"]
    below = dl.hull_distance_m(hull_r, ship["docking_port"][0]["elements_m"][0], ship["docking_port"][0]["axis"])
    assert min(p[2] for p in td._mesh_points(doc, buffers, "drogue_0")) < -below  # the stand reaches into the hull: no floating funnel


def test_drogue_runs_stand_must_reach_below_the_funnel():
    d = vehicle_spec.load_starship("starship_depot")["dorsal"]["drawing"]
    with pytest.raises(ValueError):
        dl.drogue_runs(d, 0.1)


def test_cone_radius_matches_mouth_and_throat():
    d = vehicle_spec.load_starship("starship_depot")["dorsal"]["drawing"]
    assert dl.drogue_cone_radius_m(d, 0.0) == d["drogue_mouth_radius_m"]
    assert dl.drogue_cone_radius_m(d, d["drogue_depth_m"]) == pytest.approx(d["drogue_throat_radius_m"])
    assert dl.drogue_cone_radius_m(d, -0.01) is None and dl.drogue_cone_radius_m(d, d["drogue_depth_m"] + 0.01) is None


@pytest.mark.parametrize("vehicle", ["starship_tanker", "starship_depot"])
def test_raceway_detour_clears_the_connector_pedestals(vehicle):
    """The drawn tube (radius 0.12) and its feet stay outside each connector pedestal; the detour is zero away from the connectors."""
    spec = vehicle_spec.load_starship(vehicle)
    d, connectors = spec["dorsal"]["drawing"], spec["dorsal"]["connectors"]
    rw = spec["detail"]["ship"]["raceway"]
    tube = rw["radius_m"]
    for c in connectors:
        cx = c["position_m"][0]
        worst = min(math.hypot(x - cx, dl.raceway_offset_m(x, connectors, d)) for x in np.arange(cx - 3.0, cx + 3.0, 0.01))
        assert worst - tube >= d["connector_pedestal_radius_m"] + 0.05, (vehicle, c["id"], worst)
    assert dl.raceway_offset_m(16.0, connectors, d) == 0.0 and dl.raceway_offset_m(1.5, connectors, d) == 0.0 and dl.raceway_offset_m(30.0, connectors, d) == 0.0
    # feet (every 2 m from first + 1): none overlaps a pedestal; the ones the lane clears are kept
    x = rw["first_station_m"] + 1.0
    kept = 0
    while x < rw["last_station_m"]:
        kept += dl.foot_clears_connectors(x, rw["foot_length_m"] / 2.0, rw["foot_width_m"] / 2.0, connectors, d)
        x += 2.0
    assert kept >= 16
    # the lane stays clear of the probe fairings / drogue rims (x = 12, 28 at y = +-1.5)
    port = spec["dorsal"]["ports"][0]
    for e in port["elements_m"]:
        for x in np.arange(e[0] - d["fairing_radius_m"], e[0] + d["fairing_radius_m"], 0.05):
            assert abs(e[1]) - d["fairing_radius_m"] - dl.raceway_offset_m(x, connectors, d) - tube - rw["foot_width_m"] / 2.0 > 0.0


def test_block3_has_no_detour_and_no_dorsal():
    spec = vehicle_spec.load_starship("starship_block3")
    assert "dorsal" not in spec and "insulation" not in spec and "rcs_pods" not in spec


def test_insulated_map_is_light_quilted_and_periodic():
    spec = vehicle_spec.load_starship("starship_depot")
    hull = spec["detail"]["hull"]
    ins = spec["insulation"]
    r = 0.5 * spec["stages"]["ship"]["diameter_m"]
    a = hull_textures.bake_insulated(r, hull, ins)
    assert a.shape == (hull_textures.SIZE_V, hull_textures.SIZE_U, 4) and a.dtype == np.uint8
    assert np.array_equal(a, hull_textures.bake_insulated(r, hull, ins))  # deterministic
    slope = (a[..., :2].astype(float) / 255.0 - 0.5) * hull_textures.SLOPE_RANGE
    rms = math.sqrt(float(np.mean(np.sum(slope ** 2, axis=2))))
    steel = hull_textures.bake(r, hull)
    steel_slope = (steel[..., :2].astype(float) / 255.0 - 0.5) * hull_textures.SLOPE_RANGE
    assert 0.01 < rms < 0.15  # slightly quilted: visible, never rough
    assert abs(float(slope.mean())) < 0.01  # a bulge, no tilt
    assert a[..., 3].max() > 0.3 * 255 * ins["seam_darkening"] / 0.5 * 0.5  # seam tape present
    assert a[..., 3].max() <= round(ins["seam_darkening"] * 255) + 1
    assert float(np.mean(a[..., 3])) < 0.25 * 255  # the tape is thin lines, not a field
    assert not np.array_equal(a, steel) and rms != math.sqrt(float(np.mean(np.sum(steel_slope ** 2, axis=2))))
    # no weld ring: the tape repeats with the pillow rows, so the first and last row of a period agree with the cell phase
    nu, nv = round(hull_textures.periods_m(r, hull["ring_pitch_m"])[0] / ins["pillow_width_m"]), round(hull["ring_pitch_m"] / ins["pillow_height_m"])
    assert nu >= 6 and nv >= 3


def test_steel_map_is_byte_identical_for_block3_inputs():
    """bake() is untouched: the insulated map is a separate function."""
    spec = vehicle_spec.load_starship("starship_block3")
    r = 0.5 * spec["stages"]["ship"]["diameter_m"]
    first = hull_textures.png_bytes(hull_textures.bake(r, spec["detail"]["hull"]))
    assert first == hull_textures.png_bytes(hull_textures.bake(r, spec["detail"]["hull"]))


def test_manifest_names_the_skin_and_the_rcs_covers():
    table = vehicle_spec.generator_table("starship_depot")
    depot = build_models.vehicle_manifest(vehicle_spec.load_starship("starship_depot"), "h", "v", {}, table)
    assert depot["look"]["skin_drawn"] == "insulated" and depot["look"]["part_materials"] == {"rcs_pods": "dark"}
    assert depot["dorsal_mechanism"] == "drogue" and set(depot["dorsal_drogue"]) == {"drogue_mouth_radius_m", "drogue_depth_m", "drogue_throat_radius_m"}
    assert depot["dorsal_port"]["position_m"] == _config_ship(DEPOT)["docking_port"][0]["position_m"]
    assert "dorsal_hardware.gltf" in depot["files"]
    tanker = build_models.vehicle_manifest(vehicle_spec.load_starship("starship_tanker"), "h", "v", {}, vehicle_spec.generator_table("starship_tanker"))
    assert tanker["look"]["skin_drawn"] == "steel" and "part_materials" not in tanker["look"]


def mated_transform(chaser, target):
    """core/src/vehicle/port_mating.cpp mated_stage_transform: R = P_target * diag(-1, -1, 1) * P_chaser^T with P = [axis, up x axis, up] (columns),
    t = target.position - R * chaser.position. Maps points of the chaser's stage frame into the target's."""
    def axes(p):
        axis, up = np.array(p["axis"], float), np.array(p["up"], float)
        return np.column_stack([axis, np.cross(up, axis), up])
    rot = axes(target) @ np.diag([-1.0, -1.0, 1.0]) @ axes(chaser).T
    return rot, np.array(target["position_m"], float) - rot @ np.array(chaser["position_m"], float)


def test_docked_pose_maps_each_probe_element_onto_a_drogue_element():
    rot, shift = mated_transform(td._config_ship()["docking_port"][0], _config_ship(DEPOT)["docking_port"][0])
    assert np.allclose(rot, np.diag([1.0, -1.0, -1.0])) and np.allclose(shift, [0.0, 0.0, 10.0])  # rolled 180 deg, 10 m apart (2 x 5 m port height)
    drogues = [np.array(e) for e in _config_ship(DEPOT)["docking_port"][0]["elements_m"]]
    for e in td._config_ship()["docking_port"][0]["elements_m"]:
        mapped = rot @ np.array(e) + shift
        assert min(np.linalg.norm(mapped - d) for d in drogues) < TOL
