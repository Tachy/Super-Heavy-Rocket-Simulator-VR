"""Rectify the factory grid-fin photo with the wall vanishing points of fin_lines.py (orthogonal wall families, square
pixels, principal point at the centre). Output: rectified image rotated by 45 deg so the outline's sides are near the image
axes (the lattice is a diamond lattice), with a pixel grid, and the lattice pitch from the spectrum peak.

Usage: python fin_rectify.py <photo> <fin_lines.json> <out.png> [px_per_unit]
Plane units are arbitrary (distance to the plane = 1); the pitch of the lattice converts them to metres.
"""
from __future__ import annotations

import json
import sys

import cv2
import numpy as np


def build_homography(vp_slash, vp_back, f2, cx, cy):
    f = np.sqrt(f2)
    kinv = np.linalg.inv(np.array([[f, 0, cx], [0, f, cy], [0, 0, 1.0]]))
    d1 = kinv @ np.array([*vp_slash, 1.0])
    d2 = kinv @ np.array([*vp_back, 1.0])
    d1 /= np.linalg.norm(d1)
    d2 /= np.linalg.norm(d2)
    # d1, d2 are exactly orthogonal only when f2 is exact; re-orthogonalise d2
    d2 -= d1 * (d1 @ d2)
    d2 /= np.linalg.norm(d2)
    n = np.cross(d1, d2)
    if n[2] > 0:
        n = -n
    rows = np.array([d1, d2, n])
    # image point (x, y, 1) -> r = Kinv p; lambda = 1 / (r . n); u = lambda r.d1; v = lambda r.d2
    return kinv, rows


def to_plane(kinv, rows, xy):
    r = kinv @ np.array([xy[0], xy[1], 1.0])
    lam = 1.0 / (rows[2] @ r)
    return lam * (rows[0] @ r), lam * (rows[1] @ r)


def main(photo, lines_json, out_png, scale=1500.0):
    img = cv2.imread(photo)
    data = json.load(open(lines_json))
    h, w = img.shape[:2]
    kinv, rows = build_homography(data["vp"]["slash"], data["vp"]["backslash"], data["f2"], w / 2.0, h / 2.0)
    # corners of a region of interest -> plane extent
    roi = [(900, 760), (2750, 760), (2750, 2400), (900, 2400)]
    uv = np.array([to_plane(kinv, rows, p) for p in roi])
    # rotate by 45 deg: a = (u + v)/sqrt2, b = (v - u)/sqrt2
    ab = np.stack([(uv[:, 0] + uv[:, 1]) / np.sqrt(2), (uv[:, 1] - uv[:, 0]) / np.sqrt(2)], axis=1)
    a0, b0 = ab.min(axis=0)
    dst = ((ab - [a0, b0]) * scale).astype(np.float32)
    size = (int(dst[:, 0].max()) + 1, int(dst[:, 1].max()) + 1)
    hm = cv2.getPerspectiveTransform(np.array(roi, np.float32), dst)
    rect = cv2.warpPerspective(img, hm, size)
    np.save(out_png.replace(".png", "_h.npy"), hm)
    json.dump({"a0": float(a0), "b0": float(b0), "scale": scale, "size": size},
              open(out_png.replace(".png", ".json"), "w"))
    g = cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY).astype(np.float32)
    spec = np.abs(np.fft.fftshift(np.fft.fft2(g - g.mean())))
    cy0, cx0 = np.array(spec.shape) // 2
    spec[cy0 - 6:cy0 + 7, cx0 - 6:cx0 + 7] = 0
    k = np.unravel_index(np.argsort(spec.ravel())[-6:], spec.shape)
    for ky, kx in zip(*k):
        fy, fx = (ky - cy0) / spec.shape[0], (kx - cx0) / spec.shape[1]
        print("spectrum peak freq (cycles/px)", fx, fy, "period px", 1 / max(np.hypot(fx, fy), 1e-9))
    out = rect.copy()
    for x in range(0, out.shape[1], 100):
        cv2.line(out, (x, 0), (x, out.shape[0]), (255, 0, 255), 1)
        cv2.putText(out, str(x), (x + 2, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    for y in range(0, out.shape[0], 100):
        cv2.line(out, (0, y), (out.shape[1], y), (255, 0, 255), 1)
        cv2.putText(out, str(y), (2, y + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    cv2.imwrite(out_png, out)
    print("rect size", out.shape)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4]) if len(sys.argv) > 4 else 1500.0)
