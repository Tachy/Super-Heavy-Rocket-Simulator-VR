# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
"""Mesh nodes with a large extent, to choose what the rebuild must carry."""
import pickle, sys
import numpy as np
c = pickle.load(open(sys.argv[1], "rb"))
names, par, V = c["names"], c["parents"], c["verts"]
def top(i):
    p = [names[i]]
    while i in par:
        i = par[i]; p.append(names[i])
    return p[-2] if len(p) > 1 else p[0]
rows = []
for i, v in V.items():
    lo, hi = v.min(0), v.max(0); e = hi - lo
    if e.max() >= float(sys.argv[2]):
        rows.append((top(i), names[i], lo, hi, len(v)))
rows.sort(key=lambda r: r[0])
for t, n, lo, hi, k in rows:
    print(f"{t[:14]:14s} {n[:36]:36s} {lo[0]:6.2f} {hi[0]:6.2f} | {lo[1]:6.2f} {hi[1]:6.2f} | {lo[2]:6.2f} {hi[2]:6.2f} | {k}")
