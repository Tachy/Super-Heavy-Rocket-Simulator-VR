# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
import sys
import numpy as np
sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
import gltree as g
doc, bins, world, parent, roots = g.build(sys.argv[1])
i = int(sys.argv[2]); cx = float(sys.argv[3]); cz = float(sys.argv[4])
V = g.node_verts(doc, bins, world, i)
r = np.hypot(V[:, 0] - cx, V[:, 2] - cz)
ys = np.round(V[:, 1], 2)
for y in np.unique(ys):
    m = ys == y
    print(f"y={y:8.2f} n={m.sum():5d} r {r[m].min():.3f}..{r[m].max():.3f}")
