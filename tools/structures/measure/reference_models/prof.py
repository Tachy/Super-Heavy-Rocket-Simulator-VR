# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
import os, sys
import numpy as np
import iss_measure as im

st = im.Station(sys.argv[1])
s = st.set_scale_from_truss()
name = sys.argv[2]
axis = {"x": 0, "y": 1, "z": 2}[sys.argv[3]]
step = float(sys.argv[4]) if len(sys.argv) > 4 else 0.5
i = st.index(name)
p = st.points(i, include_children=False) * s
others = [k for k in range(3) if k != axis]
# axis centre: midpoint of the extents of the two normal directions
cen = (p.min(0) + p.max(0)) / 2
print(name, "min", p.min(0).round(2), "max", p.max(0).round(2), "ctr", cen.round(2))
r = np.hypot(p[:, others[0]] - cen[others[0]], p[:, others[1]] - cen[others[1]])
t = p[:, axis]
for a in np.arange(np.floor(t.min()), t.max(), step):
    m = (t >= a) & (t < a + step)
    if m.sum():
        print(f"{a:7.2f}..{a+step:7.2f} n={m.sum():5d} rmax={r[m].max():.2f} r50={np.median(r[m]):.2f}")
