"""The model pipeline for several vehicles: generator table, stub detail files, the driver's behaviour without generators,
and the Starship's manifest hash staying what it was before the table existed. No Blender needed."""
import hashlib
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import build_models  # noqa: E402
import vehicle_spec  # noqa: E402

IDS = ("starship_block3", "falcon9_block5", "crew_dragon", "iss")
GD = build_models.ROOT / "game" / "vehicle" / "vehicle_models.gd"


@pytest.mark.parametrize("vehicle_id", IDS)
def test_every_model_vehicle_has_a_table_row_with_existing_scripts(vehicle_id):
    table = vehicle_spec.generator_table(vehicle_id)
    assert table["kind"] in ("starship", "falcon9", "dragon", "iss", "generic")
    for p in build_models.vehicle_sources(table):
        assert p.is_file(), p
    assert f'"{vehicle_id}"' in GD.read_text(encoding="utf-8")  # the game's MODEL_VEHICLE_IDS lists it too


def test_game_source_lists_name_the_starship_scripts():
    text = GD.read_text(encoding="utf-8")
    table = vehicle_spec.generator_table("starship_block3")
    for name in [*table["scripts"][1:], *table["generators"], *(f"{b}.py" for b in table["bakers"])]:
        assert f'"{name}"' in text, name  # stale check of the loader watches the same files that the manifest hash covers


def test_unknown_vehicle_is_an_error():
    with pytest.raises(vehicle_spec.VehicleSpecError):
        vehicle_spec.generator_table("starship_tanker")
    with pytest.raises(vehicle_spec.VehicleSpecError):
        vehicle_spec.load("starship_tanker")


@pytest.mark.parametrize("vehicle_id", IDS[2:])
def test_stub_detail_file_is_valid_and_loads(vehicle_id):
    detail = vehicle_spec.VEHICLES / "detail" / f"{vehicle_id}.toml"
    assert vehicle_spec.value_lines_without_tag(detail) == []
    spec = vehicle_spec.load(vehicle_id)
    assert spec["vehicle_id"] == vehicle_id and spec["engines"] == {}
    assert spec["stages"]  # the vehicle config's stages with length and diameter


@pytest.mark.parametrize("vehicle_id", IDS[3:])
def test_vehicle_without_generators_writes_nothing_and_exits_zero(vehicle_id, tmp_path, monkeypatch, capsys):
    real_table = vehicle_spec.generator_table  # every vehicle has generators by now: a table row without any is what this case tests
    monkeypatch.setattr(vehicle_spec, "generator_table", lambda vid: {**real_table(vid), "generators": []})
    monkeypatch.setattr(build_models, "find_blender", lambda explicit=None: Path("blender.exe"))
    monkeypatch.setattr(build_models, "blender_version", lambda exe: "test")
    out = tmp_path / "models" / vehicle_id
    assert build_models.build_vehicle(vehicle_id, out=out, force=True) == 0
    assert "no generators yet" in capsys.readouterr().out
    assert not out.exists() and not out.with_name(vehicle_id + ".new").exists()


def test_starship_hash_is_the_one_of_the_fixed_generator_list():
    """The table must not change what the Starship's manifest hash covers: same files, same order, same bytes."""
    legacy = [build_models.SCRIPT_DIR / n for n in ("common.py", "vehicle_common.py", "hull_profile.py")]
    legacy += [build_models.ROOT / "tools" / "structures" / n for n in ("hull_textures.py", "hull_tiles.py")]
    legacy += [build_models.SCRIPT_DIR / n for n in ("gen_hull.py", "gen_flap.py", "gen_grid_fin.py", "gen_engine.py")]
    h = hashlib.sha256()
    h.update(b"v")
    h.update(b"spec")
    for p in legacy:
        h.update(p.name.encode())
        h.update(p.read_bytes().replace(b"\r\n", b"\n"))
    table = vehicle_spec.generator_table("starship_block3")
    assert build_models.vehicle_hash("spec", "v", table) == h.hexdigest()
