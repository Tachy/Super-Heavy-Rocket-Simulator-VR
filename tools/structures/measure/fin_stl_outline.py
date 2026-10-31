"""Plan geometry of the 1:100 grid-fin STL (data only; full scale = mm x 0.1): outline polygon, symmetry axis, lattice pitch and
cross-rib position. Usage (python -I): fin_stl_outline.py <stl> [axis_of_thickness=1]
Prints the simplified convex outline (u, v in metres), its mirror axis (fitted), the pitch from the wall positions and the
rib line. Nothing of the mesh is stored."""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stl_measure import load_stl, vertices  # noqa: E402

path = sys.argv[1]
t_ax = int(sys.argv[2]) if len(sys.argv) > 2 else 1
v = vertices(load_stl(path))
ua, va = [i for i in range(3) if i != t_ax]
pts = v[:, [ua, va]]
hull = cv2.convexHull(pts.astype(np.float32))[:, 0]
eps = 0.03
poly = cv2.approxPolyDP(hull.reshape(-1, 1, 2), eps, True)[:, 0]
print("outline vertices (u, v) m, hull simplified at %.2f m:" % eps)
n = len(poly)
for i, p in enumerate(poly):
    q = poly[(i + 1) % n]
    print(i, np.round(p, 3), "side to next %.3f m" % np.hypot(*(q - p)))
area = 0.5 * abs(sum(poly[i][0] * poly[(i + 1) % n][1] - poly[(i + 1) % n][0] * poly[i][1] for i in range(n)))
print("hull area %.3f m^2" % area)
# wall positions: thin vertical faces -> histogram of vertex u and v coordinates in the lower-left square block
for name, col in (("u", 0), ("v", 1)):
    sel = pts[(pts[:, 1 - col] < 1.6) & (pts[:, 1 - col] > 0.5)][:, col]
    h, e = np.histogram(sel, bins=int((sel.max() - sel.min()) / 0.005))
    peaks = [(e[i] + e[i + 1]) / 2 for i in range(1, len(h) - 1) if h[i] >= h[i - 1] and h[i] > h[i + 1] and h[i] > 0.4 * h.max()]
    # merge neighbours closer than 0.1 m (wall has two faces)
    merged = []
    for p in peaks:
        if merged and p - merged[-1][-1] < 0.1:
            merged[-1].append(p)
        else:
            merged.append([p])
    c = [np.mean(m) for m in merged]
    print("wall centres along %s:" % name, np.round(c, 3), "mean pitch %.4f" % np.mean(np.diff(c)))
# mirror axis: for a set of candidate axes through the outline centroid directions, minimise the vertex mirror error
cen = poly.mean(axis=0)
best = None
for ang in np.radians(np.arange(0, 180, 0.25)):
    d = np.array([np.cos(ang), np.sin(ang)])
    nrm = np.array([-d[1], d[0]])
    for off in np.arange(-0.5, 0.5, 0.01):
        mir = []
        for p in hull:
            r = p - cen - nrm * off
            m = r - 2 * (r @ nrm) * nrm
            mir.append(m + cen + nrm * off)
        mir = np.array(mir)
        # distance of each mirrored hull point to the hull polygon boundary
        dist = [abs(cv2.pointPolygonTest(hull.reshape(-1, 1, 2), (float(x), float(y)), True)) for x, y in mir[::4]]
        e = np.mean(dist)
        if best is None or e < best[0]:
            best = (e, np.degrees(ang), off)
print("best mirror axis: angle %.2f deg (from +u), offset %.3f m from centroid, mean mirror error %.4f m" % (
    best[1], best[2], best[0]))
