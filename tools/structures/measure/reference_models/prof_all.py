# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
"""Combined profile: python -I prof_all.py <folder> <bin> <cx> <cz> [nodes comma] -> per bin rmax per node"""
import sys
import numpy as np
sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
import gltree as g
folder = sys.argv[1]; b = float(sys.argv[2]); cx = float(sys.argv[3]); cz = float(sys.argv[4])
doc, bins, world, parent, roots = g.build(folder)
sel = [int(x) for x in sys.argv[5].split(",")] if len(sys.argv) > 5 else [i for i, n in enumerate(doc["nodes"]) if "mesh" in n]
data = {}
for i in sel:
    V = g.node_verts(doc, bins, world, i)
    if V is not None: data[i] = (V[:, 1], np.hypot(V[:, 0] - cx, V[:, 2] - cz))
ymin = min(d[0].min() for d in data.values()); ymax = max(d[0].max() for d in data.values())
print("bin   " + " ".join(f"n{i:<9d}" for i in data))
for y0 in np.arange(np.floor(ymin / b) * b, ymax, b):
    row = []
    for i, (y, r) in data.items():
        m = (y >= y0) & (y < y0 + b)
        row.append(f"{r[m].min():.2f}-{r[m].max():.2f}" if m.any() else "    -     ")
    print(f"{y0:7.2f} " + " ".join(f"{s:<10s}" for s in row))
