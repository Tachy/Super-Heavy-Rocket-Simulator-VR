"""Wall lines of the factory grid-fin photo: LSD segments inside the fin, clustered by angle (two lattice families),
vanishing points by least squares, and the metric rectifying homography under the assumption that the two families are
orthogonal on the fin and the camera has square pixels with the principal point at the image centre.

Usage: python fin_lines.py <photo> <overlay.png> <result.json>
All coordinates are ORIGINAL photo pixels.
"""
from __future__ import annotations

import json
import sys

import cv2
import numpy as np

from fin_dots import POLY_DISPLAY, to_original


def vanishing_point(segs):
    """Least-squares intersection of the segments' lines."""
    a = np.zeros((3, 3))
    for x0, y0, x1, y1 in segs:
        line = np.cross([x0, y0, 1.0], [x1, y1, 1.0])
        line /= np.hypot(line[0], line[1])
        a += np.outer(line, line)
    w, v = np.linalg.eigh(a)
    return v[:, 0]


def main(photo, out_png, out_json):
    img = cv2.imread(photo)
    grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    poly = np.array([to_original(p) for p in POLY_DISPLAY], np.int32)
    mask = np.zeros_like(grey)
    cv2.fillPoly(mask, [poly], 255)
    mask = cv2.erode(mask, np.ones((40, 40), np.uint8))
    lsd = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)
    lines = lsd.detect(grey)[0].reshape(-1, 4)
    segs = []
    for x0, y0, x1, y1 in lines:
        mx, my = int((x0 + x1) / 2), int((y0 + y1) / 2)
        length = np.hypot(x1 - x0, y1 - y0)
        if length > 90 and mask[my, mx]:
            ang = np.degrees(np.arctan2(-(y1 - y0), x1 - x0)) % 180.0
            segs.append((x0, y0, x1, y1, ang, length))
    segs = np.array(segs)
    hist, edges = np.histogram(segs[:, 4], bins=36, range=(0, 180), weights=segs[:, 5])
    print("angle histogram (length weighted):", [(int(edges[i]), int(h)) for i, h in enumerate(hist) if h > 800])
    out = {"n_segments": len(segs)}
    families = {}
    for name, lo, hi in (("slash", 20, 60), ("backslash", 120, 165)):
        sel = segs[(segs[:, 4] >= lo) & (segs[:, 4] < hi)]
        families[name] = sel
        print(name, len(sel), "median angle", np.median(sel[:, 4]))
    vps = {k: vanishing_point(v[:, :4]) for k, v in families.items()}
    c = np.array([grey.shape[1] / 2.0, grey.shape[0] / 2.0])
    pts = {}
    for k, v in vps.items():
        pts[k] = (v[:2] / v[2]) if abs(v[2]) > 1e-12 else None
        print(k, "vanishing point", pts[k])
    out["vp"] = {k: (None if v is None else v.tolist()) for k, v in pts.items()}
    if all(v is not None for v in pts.values()):
        d = np.dot(pts["slash"] - c, pts["backslash"] - c)
        out["f2"] = float(-d)
        print("f^2 =", -d)
    colors = {"slash": (0, 255, 255), "backslash": (255, 0, 255)}
    ov = img.copy()
    for k, v in families.items():
        for x0, y0, x1, y1, _, _ in v:
            cv2.line(ov, (int(x0), int(y0)), (int(x1), int(y1)), colors[k], 3)
    cv2.imwrite(out_png, cv2.resize(ov[760:2400, 900:2750], None, fx=0.8, fy=0.8))
    with open(out_json, "w") as f:
        json.dump(out, f)


if __name__ == "__main__":
    main(*sys.argv[1:4])
