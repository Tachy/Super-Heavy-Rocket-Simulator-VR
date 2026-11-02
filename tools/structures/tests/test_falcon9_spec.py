"""Falcon 9 first-stage spec and generated model: length, leg rig numbers, no physics key in the detail file.
Blender cases skip without generated models (python tools/structures/build_models.py --vehicle falcon9_block5)."""
import json
import math
import sys
import tomllib
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import build_models  # noqa: E402
import vehicle_spec  # noqa: E402

ID = "falcon9_block5"
STAGE_LENGTH_M = 47.7  # [S] https://en.wikipedia.org/wiki/Falcon_9 with the interstage (156.5 ft)
MODELS = build_models.ROOT / "data" / "models" / ID
PHYSICS_KEYS = {"length_m", "diameter_m", "dry_mass_kg", "dry_cg_station_m", "azimuths_deg", "station_m", "radius_m",
                "area_m2", "footprint_radius_m", "foot_station_m", "stroke_m", "ring_radius_m", "angles_deg", "nose_length_m"}


def _keys(table):
    for k, v in table.items():
        yield k
        if isinstance(v, dict):
            yield from _keys(v)


def _bounds(gltf_path, node_names):
    """(min, max) of the POSITION accessors of the named nodes (the accessors' stated bounds, nodes carry no transforms)."""
    g = json.loads(Path(gltf_path).read_text(encoding="utf-8"))
    lo, hi = [math.inf] * 3, [-math.inf] * 3
    for node in g["nodes"]:
        if node.get("name") in node_names and "mesh" in node:
            assert "translation" not in node and "rotation" not in node and "scale" not in node
            for prim in g["meshes"][node["mesh"]]["primitives"]:
                acc = g["accessors"][prim["attributes"]["POSITION"]]
                lo = [min(a, b) for a, b in zip(lo, acc["min"])]
                hi = [max(a, b) for a, b in zip(hi, acc["max"])]
    return lo, hi


def test_interstage_top_at_stage_length():
    spec = vehicle_spec.load(ID)
    stage = spec["stages"]["first_stage"]
    assert stage["length_m"] == pytest.approx(STAGE_LENGTH_M, abs=0.01)
    assert stage["interstage_top_m"] == pytest.approx(STAGE_LENGTH_M, abs=0.01)
    with open(vehicle_spec.VEHICLES / "detail" / f"{ID}.toml", "rb") as f:
        detail = tomllib.load(f)
    assert PHYSICS_KEYS.isdisjoint(_keys(detail))
    if (MODELS / "falcon9_stage1.gltf").is_file():
        lo, hi = _bounds(MODELS / "falcon9_stage1.gltf", {"hull", "interstage", "octaweb"})
        assert hi[0] == pytest.approx(STAGE_LENGTH_M, abs=0.01)
        assert lo[0] == pytest.approx(detail["hull"]["base_station_m"], abs=0.01)


def test_detail_values_are_tagged():
    assert vehicle_spec.value_lines_without_tag(vehicle_spec.VEHICLES / "detail" / f"{ID}.toml") == []


def test_leg_rig_reaches_the_configured_foot():
    spec = vehicle_spec.load(ID)
    leg, legs = spec["leg"], spec["stages"]["first_stage"]["landing_legs"]
    a = math.radians(leg["deployed_angle_deg"])
    assert leg["hinge_station_m"] + leg["length_m"] * math.cos(a) == pytest.approx(legs["foot_station_m"], abs=1e-9)
    assert leg["hinge_radius_m"] + leg["length_m"] * math.sin(a) == pytest.approx(legs["footprint_radius_m"], abs=1e-9)
    assert 0.0 < leg["stowed_angle_deg"] < 10.0 < leg["deployed_angle_deg"] < 180.0


def test_missing_detail_key_is_an_error(tmp_path):
    with open(vehicle_spec.VEHICLES / "detail" / f"{ID}.toml", "rb") as f:
        detail = tomllib.load(f)
    del detail["leg"]["hinge_radius_m"]
    with pytest.raises(vehicle_spec.VehicleSpecError):
        vehicle_spec.validate_falcon9_detail(detail)


def test_grid_fin_gross_area_near_the_config():
    spec = vehicle_spec.load(ID)
    fin, config = spec["detail"]["grid_fin"], spec["stages"]["first_stage"]["grid_fins"]
    gross = fin["chord_m"] * fin["span_m"]
    assert gross == pytest.approx(config["area_m2"], rel=1e-6)


def test_manifest_of_the_generated_set():
    if not (MODELS / build_models.MANIFEST).is_file():
        pytest.skip("no generated Falcon 9 models")
    m = json.loads((MODELS / build_models.MANIFEST).read_text(encoding="utf-8"))
    spec = vehicle_spec.load(ID)
    legs = spec["stages"]["first_stage"]["landing_legs"]
    assert m["vehicle"] == ID and set(m["files"]) == set(vehicle_spec.generator_table(ID)["outputs"])
    assert m["leg_foot_deployed_m"]["station"] == pytest.approx(legs["foot_station_m"])
    assert m["leg_foot_deployed_m"]["radius"] == pytest.approx(legs["footprint_radius_m"])
    assert m["engines"]["merlin1d"]["exit_plane_x_m"] == pytest.approx(-spec["engines"]["merlin1d"]["bell_length_m"])
    # the outer Merlins' exit plane is station 0: the drawn gimbal point sits one bell length above it
    assert m["engines"]["merlin1d"]["axial_offset_m_by_stage"]["first_stage"] == pytest.approx(spec["engines"]["merlin1d"]["bell_length_m"])
