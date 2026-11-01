"""Rim depth of two installed grid fins in one photo: the plain fin (azimuth 90, pointing away from the tower, right of the hull)
and a catch fin (azimuth 0 / 180, pointing at the camera), so depth is compared at one scale, light and elevation. Also the rim's
tooth spacing, which gives the cell pitch in metres, and the wall edge thickness in the factory photo.
Usage (python -I): fin_rim_depth.py <padb_chopsticks_around_booster_top_2026-03.jpg> <eiv42cz7buif1.jpg>
Source pixels. Top edges were read by eye on 7 x / 10 x zooms (+-1 px); the free edge comes from the luma profile per column."""
import sys

import numpy as np
from PIL import Image

PX_PER_M = 370.0 / 9.0  # hull 9 m = 370 px in this photo (vehicle-models.md, real-frames round)
ELEVATION_DEG = 17.0  # camera below the fins (same round)
FINS = {
    # name: columns of the rim free of the arm, top edge as two points, dark threshold
    "catch (front)": dict(cols=(1072, 1172), top=((1082.1, 496.4), (1161.4, 506.1)), rows=(490, 545)),
    "plain (right)": dict(cols=(1232, 1300), top=((1228.0, 603.0), (1304.5, 610.7)), rows=(598, 650)),
}


def free_edge(luma, cols, rows, top):
    (x0, y0), (x1, y1) = top
    out = []
    for x in range(*cols):
        ytop = y0 + (y1 - y0) * (x - x0) / (x1 - x0)
        col = luma[rows[0]:rows[1], x]
        # first row at least 3 px under the top edge that falls under half way to the lattice's dark value
        start = int(ytop - rows[0]) + 3
        below = col[start:]
        thr = 0.5 * (np.median(below[:3]) + np.percentile(below, 5))
        idx = int(np.argmax(below < thr)) + start
        # sub-pixel crossing
        a, b = float(col[idx - 1]), float(col[idx])
        y = idx - 1 + (a - thr) / max(a - b, 1e-6) + rows[0]
        out.append((x, y, y - ytop))
    return np.array(out)


def main():
    img = np.asarray(Image.open(sys.argv[1]).convert("L"), dtype=float)
    cos_el = np.cos(np.radians(ELEVATION_DEG))
    for name, f in FINS.items():
        e = free_edge(img, f["cols"], f["rows"], f["top"])
        h = np.convolve(e[:, 2], np.ones(3) / 3, mode="same")[1:-1]
        x = e[1:-1, 0]
        tips, crowns = [], []
        for i in range(4, len(h) - 4):
            if h[i] == h[i - 4:i + 5].max():
                tips.append((x[i], h[i]))
            if h[i] == h[i - 4:i + 5].min():
                crowns.append((x[i], h[i]))
        tips, crowns = np.array(tips), np.array(crowns)
        print(name)
        print("  tips  x", tips[:, 0].astype(int).tolist(), "height px", tips[:, 1].round(1).tolist())
        print("  crowns x", crowns[:, 0].astype(int).tolist(), "height px", crowns[:, 1].round(1).tolist())
        print("  tip %.2f m (sd %.2f), crown %.2f m (sd %.2f), mean rim %.2f m; tooth spacing %.1f px = %.3f m (image plane)" % (
            tips[:, 1].mean() / PX_PER_M / cos_el, tips[:, 1].std() / PX_PER_M / cos_el,
            crowns[:, 1].mean() / PX_PER_M / cos_el, crowns[:, 1].std() / PX_PER_M / cos_el,
            h.mean() / PX_PER_M / cos_el, np.diff(tips[:, 0]).mean(), np.diff(tips[:, 0]).mean() / PX_PER_M))
    # wall edge in the factory photo: bright line FWHM across walls on the fin's face, against the local crossing spacing
    fac = np.asarray(Image.open(sys.argv[2]).convert("L"), dtype=float)
    print("factory photo", fac.shape)
    sub = fac[900:2200, 1300:2500]
    gx = np.abs(np.diff(sub, axis=1))
    bright = sub > 170
    runs = []
    for row in bright[::7]:
        idx = np.flatnonzero(np.diff(row.astype(int)))
        runs += [int(b - a) for a, b in zip(idx[::2], idx[1::2]) if b - a < 40]
    runs = np.array(runs)
    print("  bright run width px along rows: median %d, p25 %d, p75 %d, n %d" % (
        np.median(runs), np.percentile(runs, 25), np.percentile(runs, 75), len(runs)))


main()
