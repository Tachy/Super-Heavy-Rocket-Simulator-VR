"""Ship variants (tanker, depot, HLS) and taken-over stages in the vehicle spec (plan 2026-11-04, Task 2). No Blender needed."""
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import build_models  # noqa: E402
import vehicle_spec  # noqa: E402

BLOCK3_NODES_OF_VARIANT_SWITCHES = ("chine", "hinge_covers", "catch_pins")


def test_block3_variant_defaults_reproduce_block3():
    assert vehicle_spec.variant_of({}) == {"tiles": True, "flaps": True, "chine": True, "catch_pins": True,
                                           "payload_door": True, "skin": "steel"}
    spec = vehicle_spec.load_starship("starship_block3")
    # no [variant] in Block 3's detail file: nothing new reaches the spec text, so the manifest hash is what it was
    assert "variant" not in spec and "variant" not in spec["detail"]
    assert set(spec["stages"]) == {"booster", "ship"}
    assert all("stage_id" not in st and "tanks" not in st for st in spec["stages"].values())
    assert len(spec["stages"]["ship"]["flaps"]) == 4 and spec["stages"]["ship"]["catch_pins"]
    text = vehicle_spec.dumps(spec)
    assert '"variant"' not in text and '"stage_id"' not in text


def test_variant_values_are_validated():
    with pytest.raises(vehicle_spec.VehicleSpecError, match="skin"):
        vehicle_spec.variant_of({"variant": {"skin": "gold"}})
    with pytest.raises(vehicle_spec.VehicleSpecError, match="tiles"):
        vehicle_spec.variant_of({"variant": {"tiles": "no"}})
    with pytest.raises(vehicle_spec.VehicleSpecError, match="unknown"):
        vehicle_spec.variant_of({"variant": {"tile": False}})


def test_depot_variant_has_no_tile_flap_chine_pin_nodes():
    spec = vehicle_spec.load_starship("starship_depot")
    ship = spec["stages"]["ship"]
    assert spec["variant"]["tiles"] is False and spec["variant"]["skin"] == "insulated"
    assert ship["stage_id"] == "depot" and ship["flaps"] == [] and not ship["catch_pins"]
    table = vehicle_spec.generator_table("starship_depot")
    assert "gen_flap.py" not in table["generators"]
    assert not any(o.startswith("flap_") for o in table["outputs"])
    manifest = build_models.vehicle_manifest(spec, "h", "v", {}, table)
    assert manifest["flaps"] == {} and manifest["look"]["variant"]["tiles"] is False
    assert manifest["look"]["stage_ids"] == {"booster": "booster", "ship": "depot"}
    assert set(manifest["look"]["tiles"]["coverage_half_angle_deg"]) == {0.0}  # no tile material anywhere
    assert not any(f.startswith("flap_") for f in manifest["files"])
    # the hull generator skips the nodes the switches name (the spec already dropped flaps and pins)
    src = (Path(build_models.SCRIPT_DIR) / "gen_hull.py").read_text(encoding="utf-8")
    for key in ("chine", "flaps"):
        assert f'variant.get("{key}", True)' in src


def test_hls_has_only_a_ship_stage():
    spec = vehicle_spec.load_starship("starship_hls")
    assert set(spec["stages"]) == {"ship"} and spec["stages"]["ship"]["stage_id"] == "hls"
    assert spec["variant"]["skin"] == "white" and spec["engines"]  # raptor / vacuum Raptor models, no thruster model yet
    assert vehicle_spec.generator_table("starship_hls")["outputs"] == ["ship.gltf", "raptor.gltf", "rvac.gltf"]


def test_tanker_ship_resolves_block3_physics_with_tanker_tanks():
    block3 = vehicle_spec.load_starship("starship_block3")
    tanker = vehicle_spec.load_starship("starship_tanker")
    assert tanker["stages"]["booster"]["length_m"] == block3["stages"]["booster"]["length_m"]
    for key in ("length_m", "diameter_m", "nose_length_m", "belly_azimuth_deg", "flaps", "engines", "catch_pins"):
        assert tanker["stages"]["ship"][key] == block3["stages"]["ship"][key], key
    assert tanker["variant"] == {"tiles": True, "flaps": True, "chine": True, "catch_pins": True,
                                 "payload_door": False, "skin": "steel"}
    tanks = tanker["stages"]["ship"]["tanks"]
    assert tanks["ch4_main"] == {"mass_kg": 418631.1, "base_station_m": 23.697}  # the tanker config's own values
    assert tanks["lox_header"]["base_station_m"] == 44.0  # untouched tanks come from the source stage


def test_taken_over_stage_resolves_through_a_scratch_config(tmp_path):
    (tmp_path / "detail").mkdir()
    src = vehicle_spec.VEHICLES
    (tmp_path / "starship_block3.toml").write_bytes((src / "starship_block3.toml").read_bytes())
    (tmp_path / "starship_x.toml").write_text('[vehicle]\nid = "starship_x"\n[[stage]]\nfrom = "starship_block3:ship"\n'
                                              '[[stage.tank]]\nid = "nope"\nmass_kg = 1.0\n', encoding="utf-8")
    with pytest.raises(vehicle_spec.VehicleSpecError, match="nope"):
        vehicle_spec.resolve_stages("starship_x", tmp_path)
    (tmp_path / "starship_x.toml").write_text('[vehicle]\nid = "starship_x"\n[[stage]]\nfrom = "starship_block3:gone"\n',
                                              encoding="utf-8")
    with pytest.raises(vehicle_spec.VehicleSpecError, match="no such stage"):
        vehicle_spec.resolve_stages("starship_x", tmp_path)


def test_new_vehicles_are_listed_for_the_game():
    gd = (build_models.ROOT / "game" / "vehicle" / "vehicle_models.gd").read_text(encoding="utf-8")
    for vehicle in ("starship_tanker", "starship_depot", "starship_hls"):
        assert f'"{vehicle}"' in gd
        table = vehicle_spec.generator_table(vehicle)
        assert table["kind"] == "starship"
        for script in table["scripts"][1:] + table["generators"]:  # common.py is not watched by the game (as Block 3)
            assert f'"{script}"' in gd, script
        assert json.dumps(vehicle) in gd
