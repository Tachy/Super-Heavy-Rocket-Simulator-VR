"""Driver of the headless Blender generators: detail file -> glTF models in data/models/<site>/ with a manifest.

    python tools/structures/build_models.py [--force] [--config <toml>] [--out <dir>] [--blender <exe>]

The manifest holds the input hash (config + generator scripts + Blender version); a matching hash means no work.
Without Blender the driver prints a message and exits 0 (the game then runs with its boxes).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = ROOT / "tools" / "structures" / "blender"
DEFAULT_CONFIG = ROOT / "config" / "structures" / "starbase_olp2.toml"
DEFAULT_OUT = ROOT / "data" / "models" / "starbase_olp2"
BLENDER_DEFAULT = Path(r"C:\Program Files\Blender Foundation\Blender 3.4\blender.exe")
GENERATORS = ("gen_profiles.py", "gen_chopstick.py")
OUTPUTS = ("profiles.gltf", "chopstick.gltf")
MANIFEST = "manifest.json"


def find_blender(explicit: str | None = None) -> Path | None:
    for cand in (explicit, os.environ.get("STARSIM_BLENDER"), shutil.which("blender"), str(BLENDER_DEFAULT)):
        if cand and Path(cand).is_file():
            return Path(cand)
    return None


def blender_version(blender: Path) -> str:
    out = subprocess.run([str(blender), "-b", "--factory-startup", "--version"], capture_output=True, text=True,
                         check=True).stdout
    m = re.search(r"Blender\s+(\S+)", out)
    return m.group(1) if m else out.strip().splitlines()[0]


def input_hash(config: Path, version: str) -> str:
    h = hashlib.sha256()
    h.update(version.encode())
    for p in [config, SCRIPT_DIR / "common.py", *(SCRIPT_DIR / g for g in GENERATORS)]:
        h.update(p.name.encode())
        h.update(p.read_bytes().replace(b"\r\n", b"\n"))
    return h.hexdigest()


def is_current(out: Path, digest: str) -> bool:
    try:
        manifest = json.loads((out / MANIFEST).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return False
    return manifest.get("input_hash") == digest and all((out / f).is_file() for f in OUTPUTS)


def build(config: Path = DEFAULT_CONFIG, out: Path = DEFAULT_OUT, force: bool = False,
          blender: str | None = None) -> int:
    exe = find_blender(blender)
    if exe is None:
        print("build_models: Blender not found (set STARSIM_BLENDER or install Blender 3.4); no models generated")
        return 0
    version = blender_version(exe)
    digest = input_hash(config, version)
    if not force and is_current(out, digest):
        print(f"build_models: models up to date ({out})")
        return 0
    out.mkdir(parents=True, exist_ok=True)
    triangles: dict[str, int] = {}
    for gen in GENERATORS:
        cmd = [str(exe), "-b", "--factory-startup", "-P", str(SCRIPT_DIR / gen), "--", "--config", str(config),
               "--out", str(out)]
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode != 0:
            sys.stderr.write(res.stdout[-2000:] + res.stderr[-2000:])
            print(f"build_models: {gen} failed (exit {res.returncode})")
            return 1
        for m in re.finditer(r"^TRIANGLES (\S+) (\d+)$", res.stdout, re.M):
            triangles[m.group(1)] = int(m.group(2))
    manifest = {"input_hash": digest, "blender": version, "config": config.name, "files": list(OUTPUTS),
                "triangles": triangles,
                "frame": "Blender axes in glTF (export_yup off): arm along +X, rail plane Z = 0, hinge at origin"}
    (out / MANIFEST).write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"build_models: generated {', '.join(OUTPUTS)} in {out}; triangles {triangles}")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--force", action="store_true", help="regenerate even when the manifest hash matches")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--blender", default=None)
    a = ap.parse_args(argv)
    return build(a.config, a.out, a.force, a.blender)


if __name__ == "__main__":
    sys.exit(main())
