"""Least-squares fit of the shapes' projection exponent (game/shaders/clouds.gdshaderinc CLOUD_PROJECTION_A / _P / _B / _Q,
look fixes T7c): a block of uniform per-height fraction f and depth D seen straight down covers P = 1 - (1 - f)^n,
n = 1 + A d^p (1 + B (1 - f)^q), d = D / (8 vertical shape texels), per type (0 cumulus, 1 stratiform, 2 cirrus). Data:
the shapes' own union (game/tests/cloud_far_check.gd section 1e, raw columns), projection_union_2026-11-08.txt; prints
the constants and the volume's fraction f_v for target covers. The coarse levels' cover gain (CLOUD_BAKED_LEVEL_COVER_GAIN
0.8, cloud_baked_bake.gdshaderinc) was chosen against level_cover_slabs_2026-11-08.txt (centre-dump statistics of
uniform slabs from 15 / 50 / 100 / 200 / 400 km, render_check --centre_dump; "v" the volume, "raw" without the depth
cover). usage: python fit_projection.py [probe.log]"""
import os
import re
import sys

import numpy as np
from scipy.optimize import least_squares

TILE_V = {0: 16000.0, 1: 16000.0, 2: 2000.0}  # the types' vertical shape tiles (m), 128 texels each
DEFAULT_DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "projection_union_2026-11-08.txt")


def n_of(x, f, d):
    a, p, b, q = x
    return 1.0 + a * d ** p * (1.0 + b * (1.0 - f) ** q)


def cover(x, f, d):
    return 1.0 - (1.0 - f) ** n_of(x, f, d)


def invert(x, target, d):
    lo, hi = 1e-6, target
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if cover(x, mid, d) < target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def main():
    data = {0: [], 1: [], 2: []}
    for line in open(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_DATA, encoding="utf-8", errors="replace"):
        m = re.match(r"\s*type (\d) D (\d+) m: (.*)", line)
        if not m:
            continue
        t, depth = int(m.group(1)), float(m.group(2))
        for f, raw in re.findall(r"([\d.]+): ([\d.]+) ->", m.group(3)):
            f, raw = float(f), float(raw)
            if raw < 0.999:
                data[t].append((f, depth * 128.0 / (TILE_V[t] * 8.0), raw))
    for t, rows in data.items():
        a = np.array(rows)
        f, d, p_raw = a[:, 0], a[:, 1], a[:, 2]
        res = least_squares(lambda x: cover(x, f, d) - p_raw, [0.6, 1.0, 1.0, 2.0],
                            bounds=([0, 0.3, 0, 0.5], [5, 1.5, 20, 6]))
        x = res.x
        print(f"type {t}: A {x[0]:.4f} p {x[1]:.4f} B {x[2]:.4f} q {x[3]:.4f}  rms {np.sqrt(np.mean(res.fun ** 2)):.4f}"
              f" max {np.abs(res.fun).max():.4f}")
        for dd in sorted(set(d)):
            fv = [invert(x, tt, dd) for tt in (0.02, 0.05, 0.1, 0.2, 0.4, 0.6, 0.8)]
            print(f"   d {dd:6.2f}: f_v " + " ".join(f"{v:.4f}" for v in fv))


if __name__ == "__main__":
    main()
