"""Outline vertices of a grid-fin image in lattice coordinates.
Usage: python fin_vertices.py <image> <prefix> <vertices.json>
<prefix>_h.npy comes from fin_fit.py (lattice -> image). vertices.json: {"v": [[x, y], ...] (image px, outline order),
"axis": [x, y] (a point on the symmetry axis: the hinge ring centre)}. Writes <prefix>_verts.png (vertices over the image) and
prints the vertices in lattice steps."""
import json
import sys

import cv2
import numpy as np

img = cv2.imread(sys.argv[1])
prefix = sys.argv[2]
d = json.load(open(sys.argv[3]))
h = np.load(prefix + "_h.npy")
hinv = np.linalg.inv(h)
v = np.array(d["v"], np.float32)
lat = cv2.perspectiveTransform(v[None], hinv.astype(np.float32))[0]
ax = cv2.perspectiveTransform(np.array([[d["axis"]]], np.float32), hinv.astype(np.float32))[0][0]
print("axis point (lattice steps)", ax)
for i, p in enumerate(lat):
    print(i, np.round(p, 2), "rel axis", np.round(p - ax, 2))
np.save(prefix + "_verts_lat.npy", lat)
scale = 3 if img.shape[1] < 800 else 1
ov = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
pts = (v * scale).astype(np.int32)
cv2.polylines(ov, [pts], True, (0, 0, 255), 2)
for i, p in enumerate(pts):
    cv2.putText(ov, str(i), tuple(int(c) for c in p), cv2.FONT_HERSHEY_SIMPLEX, 0.8 if scale > 1 else 1.6, (0, 255, 0), 2)
cv2.imwrite(prefix + "_verts.png", ov)
