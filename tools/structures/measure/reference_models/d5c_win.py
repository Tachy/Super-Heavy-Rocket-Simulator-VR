# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
import sys
import numpy as np
import d5c_gl as gl
g, ps = gl.parts(sys.argv[1])
for p in ps:
    if p["node"] != 12:
        continue
    v = p["verts"] * 10
    b = np.c_[v[:, 1] - 0.03309, v[:, 2], v[:, 0]]
    r = np.hypot(b[:, 1], b[:, 2])
    b = b[r > 1.0]
    a = np.degrees(np.arctan2(b[:, 2], b[:, 1]))
    for lo, hi in ((20, 65), (65, 115), (-65, -20), (-115, -65)):
        m = (a > lo) & (a < hi)
        print("window", lo, hi, "n", m.sum(), "az", a[m].min().round(1), a[m].max().round(1), "X", b[m, 0].min().round(3), b[m, 0].max().round(3))
