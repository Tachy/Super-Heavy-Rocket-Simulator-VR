# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
"""Cluster nozzle-exit vertices in xz. python -I engines.py <folder> <node,node> <cx> <cz> <ylo> <yhi> [linkdist]"""
import sys
import numpy as np
sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
import gltree as g
doc, bins, world, parent, roots = g.build(sys.argv[1])
sel = [int(x) for x in sys.argv[2].split(",")]
cx, cz, ylo, yhi = (float(a) for a in sys.argv[3:7])
link = float(sys.argv[7]) if len(sys.argv) > 7 else 0.25
V = np.vstack([g.node_verts(doc, bins, world, i) for i in sel])
V = V[(V[:, 1] >= ylo) & (V[:, 1] <= yhi)]
print("verts", len(V), "y", V[:, 1].min(), V[:, 1].max())
P = V[:, [0, 2]]
lab = -np.ones(len(P), int); k = 0
for s in range(len(P)):
    if lab[s] >= 0: continue
    stack = [s]; lab[s] = k
    while stack:
        a = stack.pop()
        d = np.hypot(*(P - P[a]).T)
        nb = np.where((d < link) & (lab < 0))[0]
        lab[nb] = k; stack.extend(nb.tolist())
    k += 1
print("clusters", k)
rows = []
for c in range(k):
    Q = P[lab == c]
    if len(Q) < 6: continue
    # exit rim = circle fit through the extreme points: centre = bbox centre, radius = mean of half extents
    ctr = (Q.min(0) + Q.max(0)) / 2
    rad = ((Q.max(0) - Q.min(0)) / 2).mean()
    yy = V[lab == c][:, 1]
    rows.append((ctr[0] - cx, ctr[1] - cz, rad, len(Q), yy.min(), yy.max()))
for ctr_x, ctr_z, rad, n, y0, y1 in sorted(rows, key=lambda t: (round(np.hypot(t[0], t[1]), 1), np.degrees(np.arctan2(t[0], t[1])) % 360)):
    print(f"ctr r={np.hypot(ctr_x, ctr_z):.3f} az={np.degrees(np.arctan2(ctr_x, ctr_z)) % 360:6.1f} half-extent(exit radius)={rad:.3f} n={n} y {y0:.3f}..{y1:.3f}")
