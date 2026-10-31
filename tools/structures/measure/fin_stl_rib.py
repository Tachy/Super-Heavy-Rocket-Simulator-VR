"""Walls of the 1:100 grid-fin STL by in-plane direction: area-weighted histogram of the wall normals, and the diagonal wall
(the cross rib) positions along the symmetry axis. Usage (python -I): fin_stl_rib.py <stl>  (thickness axis = 1, axis u = x, v = z)"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stl_measure import load_stl  # noqa: E402

t = load_stl(sys.argv[1])
a, b, c = t[:, 0], t[:, 1], t[:, 2]
n = np.cross(b - a, c - a)
area = 0.5 * np.linalg.norm(n, axis=1)
n = n / np.maximum(np.linalg.norm(n, axis=1), 1e-12)[:, None]
wall = np.abs(n[:, 1]) < 0.05  # walls stand along the thickness axis
ang = (np.degrees(np.arctan2(n[:, 2], n[:, 0])) % 180.0)[wall]
w = area[wall]
h, e = np.histogram(ang, bins=36, range=(0, 180), weights=w)
print("wall normal direction in the plate (deg from +u), area-weighted:")
for lo, v in zip(e[:-1], h):
    if v > 0.02 * h.max():
        print("  %5.0f %7.3f m^2" % (lo, v))
# diagonal walls (normal at 45 deg): s = (u + v) / sqrt 2 of the face centroids
sel = wall & (np.abs(((np.degrees(np.arctan2(n[:, 2], n[:, 0])) % 180.0) - 45.0)) < 3.0)
cen = (a + b + c)[sel] / 3.0
s = (cen[:, 0] + cen[:, 2]) / np.sqrt(2.0)
hh, ee = np.histogram(s, bins=int(s.max() / 0.01), weights=area[sel])
print("walls with the normal along the symmetry axis (u = v diagonal): s from the axis origin, area peaks")
for i in range(1, len(hh) - 1):
    if hh[i] > 0.05 * hh.max() and hh[i] >= hh[i - 1] and hh[i] > hh[i + 1]:
        print("  s = %.3f m  %.3f m^2" % ((ee[i] + ee[i + 1]) / 2, hh[i]))
