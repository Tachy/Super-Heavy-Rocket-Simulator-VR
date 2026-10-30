"""Ratio measurement of a reference image (spec A): lengths from a known-scale pair of points.

Points file (TOML, lives next to the image in the git-ignored reference folder):

    image = "padb_full_stack_on_mount_2026-05.jpg"
    [scale]                    # two image points of a known length (booster diameter 9 m [S])
    a = [988, 700]
    b = [1070, 700]
    length_m = 9.0
    [measure.tower_face_width_m]
    a = [880, 700]
    b = [985, 700]
    [measure.leg_size_m]       # optional: "foreshortening" divides the result (view angle correction, default 1)
    a = [..]
    b = [..]
    foreshortening = 0.9

Usage: python ratio.py <points.toml>   -> one "name = value_m" line per measure, with the scale in px per m.
Only valid for lengths in (roughly) the plane of the scale object: no camera solve (full build).
"""
from __future__ import annotations

import math
import sys
import tomllib
from pathlib import Path


def _dist(a, b) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def measure(points: dict) -> dict[str, float]:
    """Return {name: length_m} for every [measure.*] entry of a parsed points file."""
    scale = points["scale"]
    px_per_m = _dist(scale["a"], scale["b"]) / float(scale["length_m"])
    if px_per_m <= 0.0:
        raise ValueError("scale points coincide")
    out = {}
    for name, m in points.get("measure", {}).items():
        out[name] = _dist(m["a"], m["b"]) / px_per_m / float(m.get("foreshortening", 1.0))
    return out


def main(argv: list[str]) -> int:
    with open(Path(argv[1]), "rb") as f:
        points = tomllib.load(f)
    scale = points["scale"]
    print(f"# {points.get('image', '?')}: {_dist(scale['a'], scale['b']) / scale['length_m']:.2f} px/m")
    for name, value in measure(points).items():
        print(f"{name} = {value:.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
