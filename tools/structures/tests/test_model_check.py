"""build_models --check, the packaging gate: a model set whose sources changed or that lacks a file is reported (no
Blender needed: the hash is recomputed with the Blender version the manifest records)."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import build_models  # noqa: E402

VERSION = "3.4.1"


def _fake_set(models: Path, set_id: str, digest: str | None = None, skip: str | None = None) -> Path:
    """A model set as build_models writes it (empty output files, a manifest with the current or a given hash)."""
    expected, outputs = build_models.expected_set(set_id, VERSION)
    out = models / set_id
    out.mkdir(parents=True)
    for f in outputs:
        if f != skip:
            (out / f).parent.mkdir(parents=True, exist_ok=True)
            (out / f).write_bytes(b"")
    manifest = {"input_hash": digest or expected, "blender": VERSION, "files": list(outputs)}
    (out / build_models.MANIFEST).write_text(json.dumps(manifest), encoding="utf-8")
    return out


def test_current_sets_pass(tmp_path):
    sets = [build_models.PAD_SET, "starship_hls"]
    for s in sets:
        _fake_set(tmp_path, s)
    assert build_models.stale_models(tmp_path, sets) == []


def test_changed_sources_are_reported(tmp_path):
    _fake_set(tmp_path, "starship_hls", digest="0" * 64)
    problems = build_models.stale_models(tmp_path, ["starship_hls"])
    assert problems == ["starship_hls: built from older sources (input hash differs)"]


def test_a_missing_output_file_is_reported(tmp_path):
    _, outputs = build_models.expected_set(build_models.PAD_SET, VERSION)
    _fake_set(tmp_path, build_models.PAD_SET, skip=outputs[0])
    problems = build_models.stale_models(tmp_path, [build_models.PAD_SET])
    assert problems == [f"{build_models.PAD_SET}: missing {outputs[0]}"]


def test_a_missing_set_is_reported_and_check_exits_1(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(build_models, "model_sets", lambda: ["crew_dragon"])
    assert build_models.check(tmp_path) == 1
    assert "crew_dragon: no readable manifest.json" in capsys.readouterr().out


def test_every_vehicle_with_generators_is_checked():
    sets = build_models.model_sets()
    assert sets[0] == build_models.PAD_SET
    assert {"starship_block3", "starship_hls", "falcon9_block5", "crew_dragon", "iss"} <= set(sets)
