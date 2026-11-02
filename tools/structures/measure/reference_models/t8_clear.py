# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
"""Clearance of the docked Dragon (open nose) from the station drawing: nearest vertex pairs, station parts except the mated IDA."""
import numpy as np, t8_cmp as C
from scipy.spatial import cKDTree
src, tris, man = C.ours()
for k, port in enumerate(man["ports"]):
    d = C.mated_dragon(port)
    n = port["number"]
    for label, parts in (("nose", [s for s in d if s[3] in ("nose_cone", "nose_seal")]), ("all", d)):
        pts = np.vstack([s[0] for s in parts])
        best = []
        for v, t, c, name in src:
            if name.startswith(f"ida_{n}"):
                continue
            dist, _ = cKDTree(v).query(pts, k=1)
            best.append((float(dist.min()), name))
        best.sort()
        print(port["id"], label, [(round(a, 2), b) for a, b in best[:4]])
