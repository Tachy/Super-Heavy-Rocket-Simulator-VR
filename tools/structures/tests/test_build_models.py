"""Tests of the headless Blender model generator (skipped without Blender)."""
import hashlib
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import build_models  # noqa: E402
import detail_file  # noqa: E402

pytestmark = pytest.mark.skipif(build_models.find_blender() is None, reason="Blender not installed")
BOUND_TOL_M = 0.05


def _bounds(gltf_path):
    """Min / max of all POSITION accessors of the glTF (the nodes carry no transforms: meshes are in the root frame)."""
    g = json.loads(Path(gltf_path).read_text(encoding="utf-8"))
    lo, hi = [1e9] * 3, [-1e9] * 3
    for mesh in g["meshes"]:
        for prim in mesh["primitives"]:
            acc = g["accessors"][prim["attributes"]["POSITION"]]
            lo = [min(a, b) for a, b in zip(lo, acc["min"])]
            hi = [max(a, b) for a, b in zip(hi, acc["max"])]
    return g, lo, hi


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    out = tmp_path_factory.mktemp("models_a")
    assert build_models.build(build_models.DEFAULT_CONFIG, out) == 0
    return out


def test_chopstick_bounds_and_hinge(built):
    # the glTF is in Blender axes (export_yup off): X along the arm, Y across, Z up with the rail plane at 0
    detail = detail_file.load(build_models.DEFAULT_CONFIG)["chopstick"]
    g, lo, hi = _bounds(built / "chopstick.gltf")
    assert lo[0] == pytest.approx(0.0, abs=BOUND_TOL_M)          # hinge end at the origin
    assert hi[0] == pytest.approx(detail["length_m"], abs=BOUND_TOL_M)
    assert hi[1] - lo[1] == pytest.approx(detail["width_m"], abs=BOUND_TOL_M)
    assert hi[2] == pytest.approx(0.0, abs=BOUND_TOL_M)           # top of the catch rails
    assert hi[2] - lo[2] == pytest.approx(detail["depth_m"], abs=BOUND_TOL_M)
    names = {n["name"] for n in g["nodes"]}
    assert {"chopstick", "truss", "rail"} <= names
    root = next(n for n in g["nodes"] if n["name"] == "chopstick")
    assert "translation" not in root or root["translation"] == [0.0, 0.0, 0.0]


def test_profiles_are_unit_length_named_meshes(built):
    cfg = detail_file.load(build_models.DEFAULT_CONFIG)
    g, lo, hi = _bounds(built / "profiles.gltf")
    assert {m["name"] for m in g["meshes"]} == {f"profile_{n}" for n in cfg["profiles"]}
    assert lo[2] == pytest.approx(0.0, abs=1e-6) and hi[2] == pytest.approx(1.0, abs=1e-6)
    manifest = json.loads((built / "manifest.json").read_text(encoding="utf-8"))
    print("triangle counts:", manifest["triangles"])
    assert manifest["triangles"]["chopstick"] > 0


def test_two_runs_are_byte_identical(built, tmp_path):
    assert build_models.build(build_models.DEFAULT_CONFIG, tmp_path) == 0
    for name in ("chopstick.bin", "profiles.bin"):
        a = hashlib.sha256((built / name).read_bytes()).hexdigest()
        b = hashlib.sha256((tmp_path / name).read_bytes()).hexdigest()
        assert a == b, name


def test_manifest_follows_config_and_current_run_is_skipped(built, tmp_path, capsys):
    exe = build_models.find_blender()
    version = build_models.blender_version(exe)
    cfg = tmp_path / "changed.toml"
    cfg.write_text(build_models.DEFAULT_CONFIG.read_text(encoding="utf-8")
                   .replace("length_m = 26.0", "length_m = 25.0"), encoding="utf-8")
    h0 = build_models.input_hash(build_models.DEFAULT_CONFIG, version)
    assert build_models.input_hash(cfg, version) != h0
    assert json.loads((built / "manifest.json").read_text(encoding="utf-8"))["input_hash"] == h0
    capsys.readouterr()
    manifest = built / "manifest.json"
    os.utime(manifest, (1.0e9, 1.0e9))
    assert build_models.build(build_models.DEFAULT_CONFIG, built) == 0
    assert "up to date" in capsys.readouterr().out
    assert manifest.stat().st_mtime > 1.0e9 + 1.0e6, "the up-to-date branch touches the manifest (game mtime check)"


def test_failed_generator_leaves_the_old_models_and_manifest(built, tmp_path, monkeypatch, capsys):
    out = tmp_path / "models"
    shutil.copytree(built, out)
    before = {f.name: f.read_bytes() for f in out.iterdir()}
    real_run = subprocess.run

    def run(cmd, *args, **kwargs):  # the second generator (the chopstick) fails after the first one has written
        if any("gen_chopstick" in str(c) for c in cmd):
            return subprocess.CompletedProcess(cmd, 3, stdout="", stderr="boom")
        return real_run(cmd, *args, **kwargs)

    monkeypatch.setattr(build_models.subprocess, "run", run)
    assert build_models.build(build_models.DEFAULT_CONFIG, out, force=True) == 1
    assert {f.name: f.read_bytes() for f in out.iterdir()} == before
    assert not (tmp_path / "models.new").exists()
    assert "left as it was" in capsys.readouterr().out


def test_missing_blender_exits_zero(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(build_models, "find_blender", lambda explicit=None: None)
    assert build_models.build(build_models.DEFAULT_CONFIG, tmp_path / "none") == 0
    assert "Blender not found" in capsys.readouterr().out
    assert not (tmp_path / "none").exists()
    shutil.rmtree(tmp_path, ignore_errors=True)
