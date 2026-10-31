"""Fit the lattice homography of one image from a point set (npy) and write the rectified image + index overlay.
Usage: python fin_fit.py <image> <pts.npy> <out_prefix> [scale_px_per_step]
Lattice units: one grid step = one wall pitch (cell centres or wall crossings of the square lattice)."""
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fin_lattice import assign_indices, fit  # noqa: E402

img = cv2.imread(sys.argv[1])
pts = np.load(sys.argv[2])
prefix = sys.argv[3]
scale = float(sys.argv[4]) if len(sys.argv) > 4 else 40.0
idx, step = assign_indices(pts)
keys = sorted(idx)
h, res_px, res_lat, lat = fit(pts, idx)
print("points", len(pts), "indexed", len(keys), "step px", round(step, 1))
print("residual px: mean %.2f max %.2f; in grid steps: rms %.3f max %.3f" % (
    res_px.mean(), res_px.max(), np.sqrt((res_lat ** 2).mean()), res_lat.max()))
# refit without the outliers (> 2.5 x median) once
keep = [k for k, r in zip(keys, res_lat) if r < max(2.5 * np.median(res_lat), 0.05)]
sub = {k: idx[k] for k in keep}
h, res_px, res_lat, lat = fit(pts, sub)
print("after outlier cut: n", len(keep), "residual px mean %.2f max %.2f; steps rms %.3f max %.3f" % (
    res_px.mean(), res_px.max(), np.sqrt((res_lat ** 2).mean()), res_lat.max()))
np.save(prefix + "_h.npy", h)
json.dump({"n": len(keep), "res_px_mean": float(res_px.mean()), "res_px_max": float(res_px.max()),
           "res_step_rms": float(np.sqrt((res_lat ** 2).mean())), "res_step_max": float(res_lat.max())},
          open(prefix + "_fit.json", "w"))
# rectified image: lattice plane (i, j) -> pixels, i right, j down, `scale` px per step
lat_all = np.array([idx[k] for k in keys])
lo, hi = lat_all.min(axis=0) - 4.0, lat_all.max(axis=0) + 4.0
size = (int((hi[0] - lo[0]) * scale), int((hi[1] - lo[1]) * scale))
s = np.array([[scale, 0, -lo[0] * scale], [0, scale, -lo[1] * scale], [0, 0, 1]])
m = s @ np.linalg.inv(h)
rect = cv2.warpPerspective(img, m, size, flags=cv2.INTER_CUBIC)
cv2.imwrite(prefix + "_rect.png", rect)
json.dump({"lo": lo.tolist(), "scale": scale, "m": m.tolist()}, open(prefix + "_rect.json", "w"))
print("rect", size, "lattice range", lo, hi)
