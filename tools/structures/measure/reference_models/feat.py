# Measuring script of the plan 2026-11-02 (Falcon 9, Dragon, ISS): reads a reference model given by path (never executes
# it, never stores its data). Run with python -I; the reference files are not in this repository (docs/subsystems/vehicle-models.md).
import sys as _sys, pathlib as _pl
_sys.path[:0] = [str(_pl.Path(__file__).resolve().parent), str(_pl.Path(__file__).resolve().parent.parent)]
"""Azimuth clusters of vertices: python -I feat.py <folder> <cx> <cz> <nodes|all> <ymin> <ymax> <rmin> [rmax]
azimuth = atan2(x-cx, z-cz) in degrees (0 = +z, 90 = +x), clusters by gaps > 6 deg."""
import sys
import numpy as np
sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
import gltree as g
folder = sys.argv[1]; cx = float(sys.argv[2]); cz = float(sys.argv[3])
doc, bins, world, parent, roots = g.build(folder)
sel = [i for i, n in enumerate(doc["nodes"]) if "mesh" in n] if sys.argv[4] == "all" else [int(x) for x in sys.argv[4].split(",")]
ymin, ymax, rmin = float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7])
rmax = float(sys.argv[8]) if len(sys.argv) > 8 else 1e9
P = []
for i in sel:
    V = g.node_verts(doc, bins, world, i)
    if V is not None: P.append(V)
V = np.vstack(P)
r = np.hypot(V[:, 0] - cx, V[:, 2] - cz)
m = (V[:, 1] >= ymin) & (V[:, 1] <= ymax) & (r >= rmin) & (r <= rmax)
V = V[m]; r = r[m]
az = np.degrees(np.arctan2(V[:, 0] - cx, V[:, 2] - cz)) % 360
print("selected", len(V))
if len(V) == 0: sys.exit()
o = np.argsort(az); a = az[o]
gaps = np.where(np.diff(a) > 6)[0]
starts = np.r_[0, gaps + 1]; ends = np.r_[gaps, len(a) - 1]
groups = [(o[s:e + 1]) for s, e in zip(starts, ends)]
if len(groups) > 1 and (a[0] + 360 - a[-1]) <= 6:  # wrap merge
    groups = [np.r_[groups[-1], groups[0]]] + groups[1:-1]
for gi in groups:
    ang = az[gi]
    cen = np.degrees(np.arctan2(np.sin(np.radians(ang)).mean(), np.cos(np.radians(ang)).mean())) % 360
    print(f"az_c {cen:7.1f}  n={len(gi):6d}  y {V[gi,1].min():8.3f}..{V[gi,1].max():8.3f}  r {r[gi].min():.3f}..{r[gi].max():.3f}  "
          f"az span {np.ptp(((ang - cen + 180) % 360) - 180):.1f}")
