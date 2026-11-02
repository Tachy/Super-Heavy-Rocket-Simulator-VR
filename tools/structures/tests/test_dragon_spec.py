"""Crew Dragon spec and generated model: the nose cone's hinge, lobes derived from the config, sizes against the physics config,
no physics key in the detail file. Blender cases skip without generated models
(python tools/structures/build_models.py --vehicle crew_dragon)."""
import json
import math
import sys
import tomllib
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import build_models  # noqa: E402
import vehicle_spec  # noqa: E402

ID = "crew_dragon"
MODELS = build_models.ROOT / "data" / "models" / ID
PHYSICS_KEYS = {"length_m", "diameter_m", "hull_top_diameter_m", "nose_radius_m", "azimuths_deg", "angles_deg", "station_m",
                "radius_m", "position_m", "ring_radius_m", "dry_mass_kg", "dry_cg_station_m"}
CAPSULE_LENGTH_M = 4.85  # [E] shield apex to nose tip, measured (KUBAHA M2 / M1, drawing D_Dragon_2_DM_2_03 4.83): see the config
TOTAL_HEIGHT_M = 8.1  # [S] https://en.wikipedia.org/wiki/SpaceX_Dragon_2 with the trunk


def _keys(table):
    for k, v in table.items():
        yield k
        if isinstance(v, dict):
            yield from _keys(v)


def lerp_profile(spec, x):
    table = spec["detail"]["capsule"]["profile_m"]
    for (x0, r0), (x1, r1) in zip(table, table[1:]):
        if x0 <= x <= x1:
            return r0 + (r1 - r0) * (x - x0) / (x1 - x0)
    raise ValueError(x)


def _bounds(path, names):
    g = json.loads(Path(path).read_text(encoding="utf-8"))
    lo, hi = [math.inf] * 3, [-math.inf] * 3
    for node in g["nodes"]:
        if node.get("name") in names and "mesh" in node:
            assert "translation" not in node and "rotation" not in node and "scale" not in node
            for prim in g["meshes"][node["mesh"]]["primitives"]:
                acc = g["accessors"][prim["attributes"]["POSITION"]]
                lo = [min(a, b) for a, b in zip(lo, acc["min"])]
                hi = [max(a, b) for a, b in zip(hi, acc["max"])]
    return lo, hi


def test_published_heights_agree_with_the_config():
    spec = vehicle_spec.load(ID)
    assert spec["stages"]["capsule"]["length_m"] == pytest.approx(CAPSULE_LENGTH_M, abs=0.01)
    assert spec["stages"]["trunk"]["length_m"] + spec["stages"]["capsule"]["length_m"] == pytest.approx(TOTAL_HEIGHT_M, abs=0.01)


def test_detail_file_has_no_physics_key_and_every_value_is_tagged():
    path = vehicle_spec.VEHICLES / "detail" / f"{ID}.toml"
    with open(path, "rb") as f:
        detail = tomllib.load(f)
    assert PHYSICS_KEYS.isdisjoint(_keys(detail))
    assert vehicle_spec.value_lines_without_tag(path) == []


def test_superdraco_lobes_are_the_middles_of_the_config_pairs():
    spec = vehicle_spec.load(ID)
    assert spec["stages"]["capsule"]["superdraco_lobes_deg"] == pytest.approx([63.0, 117.0, 243.0, 297.0])
    with pytest.raises(vehicle_spec.VehicleSpecError):
        vehicle_spec.dragon_lobes([1.0, 2.0, 3.0])


def test_nose_hinge_is_the_measured_point_and_the_axis_is_tangential():
    spec = vehicle_spec.load(ID)
    nose = spec["nose"]
    assert nose["hinge_radius_m"] == spec["detail"]["capsule"]["nose_hinge_r_m"]
    assert nose["hinge_radius_m"] == pytest.approx(lerp_profile(spec, nose["hinge_station_m"]), abs=0.02)  # at the skin
    assert nose["hinge_station_m"] + nose["length_m"] == pytest.approx(CAPSULE_LENGTH_M)
    a = math.radians(nose["hinge_azimuth_deg"])
    assert nose["axis"] == pytest.approx([0.0, -math.sin(a), math.cos(a)])
    assert 90.0 <= nose["open_angle_deg"] <= 150.0 and nose["open_time_s"] > 0.0


@pytest.mark.skipif(not (MODELS / "dragon_capsule.gltf").is_file(), reason="models not generated")
def test_generated_sizes():
    spec = vehicle_spec.load(ID)
    lo, hi = _bounds(MODELS / "dragon_capsule.gltf", {"capsule", "heat_shield", "band", "deck", "recess", "superdraco_pod", "superdraco_nozzle", "draco_pod", "window", "hatch", "seams"})
    assert lo[0] == pytest.approx(0.0, abs=0.001)  # the shield apex is the stage's station 0
    assert hi[0] == pytest.approx(spec["stages"]["capsule"]["port_position_m"][0], abs=0.015)  # the flange top, the config's port, stands highest under the nose (skirt details lie a little proud)
    assert hi[1] == pytest.approx(spec["stages"]["capsule"]["diameter_m"] / 2.0, abs=0.12)  # the pods stand proud of the 4.0 m shoulder only by a few cm
    nlo, nhi = _bounds(MODELS / "dragon_nose.gltf", {"nose_cone"})
    # relative to the hinge: the tip is `length` above it; only the hinge pin (arm thickness) reaches below the hinge point
    assert nhi[0] == pytest.approx(spec["nose"]["length_m"], abs=0.001) and -spec["detail"]["capsule"]["hinge_arm_m"][1] <= nlo[0] <= 0.03
    tlo, thi = _bounds(MODELS / "dragon_trunk.gltf", {"trunk", "trunk_array", "trunk_radiator"})
    assert tlo[0] == pytest.approx(0.0, abs=0.001)
    assert thi[0] == pytest.approx(spec["stages"]["trunk"]["length_m"] + spec["detail"]["trunk"]["shield_well_m"], abs=0.001)
    assert max(abs(thi[1]), abs(thi[2])) == pytest.approx(max(p[1] for p in spec["detail"]["trunk"]["fin_outline_m"]), abs=0.01)
