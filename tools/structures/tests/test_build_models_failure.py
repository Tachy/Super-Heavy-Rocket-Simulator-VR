"""Failure paths of the model driver that need no Blender: a run that produces no glTF keeps the old models."""
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import build_models  # noqa: E402


def _fake_blender(monkeypatch, returncode):
    """Blender stand-in: every generator 'runs' with this exit code and writes nothing."""
    calls = []

    def run(cmd, *args, **kwargs):
        calls.append(cmd)
        return subprocess.CompletedProcess(cmd, returncode, stdout="", stderr="Traceback: boom")

    monkeypatch.setattr(build_models, "find_blender", lambda explicit=None: Path("blender.exe"))
    monkeypatch.setattr(build_models, "blender_version", lambda exe: "test")
    monkeypatch.setattr(build_models.subprocess, "run", run)
    return calls


def _old_models(tmp_path):
    out = tmp_path / "models"
    out.mkdir()
    (out / "chopstick.gltf").write_text("old", encoding="utf-8")
    (out / "manifest.json").write_text('{"input_hash": "old"}', encoding="utf-8")
    return out


def _assert_kept(out, tmp_path, capsys):
    assert sorted(f.name for f in out.iterdir()) == ["chopstick.gltf", "manifest.json"]
    assert (out / "chopstick.gltf").read_text(encoding="utf-8") == "old"
    assert (out / "manifest.json").read_text(encoding="utf-8") == '{"input_hash": "old"}'
    assert not (tmp_path / "models.new").exists()
    assert "left as it was" in capsys.readouterr().out


def test_exit_zero_without_gltf_fails_and_keeps_the_old_models(tmp_path, monkeypatch, capsys):
    out = _old_models(tmp_path)
    _fake_blender(monkeypatch, 0)  # Blender's old behaviour on a Python exception
    assert build_models.build(build_models.DEFAULT_CONFIG, out, force=True) == 1
    _assert_kept(out, tmp_path, capsys)


def test_generator_exception_fails_and_keeps_the_old_models(tmp_path, monkeypatch, capsys):
    out = _old_models(tmp_path)
    calls = _fake_blender(monkeypatch, 1)
    assert build_models.build(build_models.DEFAULT_CONFIG, out, force=True) == 1
    _assert_kept(out, tmp_path, capsys)
    assert len(calls) == 1  # stops at the first failure


def test_every_generator_runs_with_a_python_exit_code(tmp_path):
    cmd = build_models.blender_command(Path("blender.exe"), Path("gen.py"), "--out", "x")
    flag = cmd.index("--python-exit-code")
    assert cmd[flag + 1] == "1"
    assert flag < cmd.index("-P")  # Blender reads its options in order: it must precede the script
    assert cmd[cmd.index("--") + 1:] == ["--out", "x"]


def test_swap_keeps_the_old_directory_when_the_rename_fails(tmp_path, monkeypatch):
    out = _old_models(tmp_path)
    staging = tmp_path / "models.new"
    staging.mkdir()
    real_rename = Path.rename

    def rename(self, target):
        if self == staging:
            raise OSError("locked")
        return real_rename(self, target)

    monkeypatch.setattr(Path, "rename", rename)
    try:
        build_models.swap_in(staging, out)
    except OSError:
        pass
    else:
        raise AssertionError("swap_in must raise")
    assert (out / "chopstick.gltf").read_text(encoding="utf-8") == "old"
