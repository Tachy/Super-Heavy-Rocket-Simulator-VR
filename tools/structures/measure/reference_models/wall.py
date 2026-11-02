# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
"""Sidewall half-angle fit: python -I wall.py <folder> <cx> <cz> <nodes> <scale> <ylo> <yhi> <r_hi> <r_lo>  (r in model units before scale)"""
import sys
import numpy as np
sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
import gltree as g
doc, bins, world, parent, roots = g.build(sys.argv[1])
cx, cz = float(sys.argv[2]), float(sys.argv[3])
sel = [int(x) for x in sys.argv[4].split(",")]
sc = float(sys.argv[5]); ylo, yhi, rhi, rlo = (float(a) for a in sys.argv[6:10])
V = np.vstack([g.node_verts(doc, bins, world, i) for i in sel])
r = np.hypot(V[:, 0] - cx, V[:, 2] - cz) * sc; y = V[:, 1] * sc
ys, rs = [], []
for y0 in np.arange(ylo * sc, yhi * sc, 0.01 * sc):
    m = (y >= y0) & (y < y0 + 0.01 * sc)
    if m.any(): ys.append(y[m].mean()); rs.append(r[m].max())
ys = np.array(ys); rs = np.array(rs)
k = (rs <= rhi * sc) & (rs >= rlo * sc)
p = np.polyfit(ys[k], rs[k], 1)
print("points", k.sum(), "dr/dy", p[0], "half-angle from axis deg", np.degrees(np.arctan(-p[0])))
print("y range", ys[k].min(), ys[k].max(), "r range", rs[k].min(), rs[k].max())
