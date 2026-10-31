"""Find the tie-rod dots (bright specks at the wall crossings) of the factory grid-fin photo and write an overlay.

Usage: python fin_dots.py <photo> <out_overlay.png> <out_points.json>
Coordinates are ORIGINAL photo pixels. The inside-the-fin mask is a hand polygon (crop-display coordinates of
gridcrop.py 900 760 2750 2400, scale 0.865).
"""
from __future__ import annotations

import json
import sys

import cv2
import numpy as np

CROP_X0, CROP_Y0, CROP_SCALE = 900.0, 760.0, 0.865
POLY_DISPLAY = [(230, 110), (430, 60), (780, 10), (1000, 100), (1110, 210), (1300, 420), (1400, 540), (1440, 830),
                (1500, 1180), (1290, 1400), (900, 1350), (690, 1290), (440, 1240), (330, 1100), (120, 800),
                (150, 560), (190, 350)]


def to_original(p):
    return (CROP_X0 + p[0] / CROP_SCALE, CROP_Y0 + p[1] / CROP_SCALE)


def main(photo, out_png, out_json):
    img = cv2.imread(photo)
    grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    poly = np.array([to_original(p) for p in POLY_DISPLAY], np.int32)
    mask = np.zeros_like(grey)
    cv2.fillPoly(mask, [poly], 255)
    mask = cv2.erode(mask, np.ones((25, 25), np.uint8))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (21, 21))
    tophat = cv2.morphologyEx(grey, cv2.MORPH_TOPHAT, kernel)
    _, bw = cv2.threshold(tophat, 70, 255, cv2.THRESH_BINARY)
    bw = cv2.bitwise_and(bw, mask)
    n, _, stats, cent = cv2.connectedComponentsWithStats(bw)
    pts = [tuple(map(float, cent[i])) for i in range(1, n) if 6 <= stats[i, cv2.CC_STAT_AREA] <= 400]
    out = img.copy()
    for x, y in pts:
        cv2.circle(out, (int(x), int(y)), 14, (0, 255, 0), 3)
    cv2.polylines(out, [poly], True, (0, 0, 255), 3)
    crop = out[760:2400, 900:2750]
    cv2.imwrite(out_png, cv2.resize(crop, None, fx=0.8, fy=0.8))
    with open(out_json, "w") as f:
        json.dump(pts, f)
    print(len(pts), "dots")


if __name__ == "__main__":
    main(*sys.argv[1:4])
