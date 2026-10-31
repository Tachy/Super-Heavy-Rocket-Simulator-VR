"""Cell map of the grid fin: the STL outline (fin_stl_outline.py numbers) cut by the square wall lattice, cells per row along
the two 45 deg lines (to the symmetry axis), full and partial; plus the same count for the template image's heart cells.
Usage (python -I): fin_cell_map.py <out_png> [template_pts.npy template_h.npy]"""
import sys

import cv2
import numpy as np

PITCH = 0.350  # [E] STL wall pitch, fin_stl_outline.py
WALL0 = 0.02   # [E] first wall centre at u = v = 0.02 + k pitch (STL wall centres 0.37 + 0.35 k)
OUT = [(4.03, 3.785), (3.785, 4.03), (1.422, 3.918), (0.0, 2.495), (0.0, 1.4), (1.4, 0.0), (2.495, 0.0), (3.918, 1.422)]
RIB_S = 3.75  # [E] cross rib position along the axis from the tail, s = (u + v) / sqrt 2


def clip(poly, a, b, c):
    """Sutherland-Hodgman against the half plane a u + b v + c >= 0."""
    out = []
    for i in range(len(poly)):
        p, q = poly[i], poly[(i + 1) % len(poly)]
        fp, fq = a * p[0] + b * p[1] + c, a * q[0] + b * q[1] + c
        if fp >= 0:
            out.append(p)
        if (fp >= 0) != (fq >= 0):
            t = fp / (fp - fq)
            out.append((p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])))
    return out


def area(poly):
    return 0.5 * abs(sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1]
                         for i in range(len(poly)))) if len(poly) > 2 else 0.0


def frac(k, m, polygon):
    cell = [(WALL0 + PITCH * k, WALL0 + PITCH * m), (WALL0 + PITCH * (k + 1), WALL0 + PITCH * m),
            (WALL0 + PITCH * (k + 1), WALL0 + PITCH * (m + 1)), (WALL0 + PITCH * k, WALL0 + PITCH * (m + 1))]
    n = len(polygon)
    ccw = np.sign(sum(polygon[i][0] * polygon[(i + 1) % n][1] - polygon[(i + 1) % n][0] * polygon[i][1] for i in range(n)))
    for i in range(n):
        p, q = polygon[i], polygon[(i + 1) % n]
        # inside = left of p->q for a CCW polygon
        a, b = -(q[1] - p[1]) * ccw, (q[0] - p[0]) * ccw
        cell = clip(cell, a, b, -(a * p[0] + b * p[1]))
        if not cell:
            return 0.0
    return area(cell) / PITCH ** 2


def main():
    rows = {}
    grid = {}
    for m in range(-1, 12):
        for k in range(-1, 12):
            f = frac(k, m, OUT)
            if f > 0.02:
                grid[(k, m)] = f
    ms = sorted({m for _, m in grid})
    print("STL cell map: rows m (cells along the u line), cells k = 0..; fractions of a full cell; s = (u + v)/sqrt2")
    tot = 0.0
    for m in ms:
        row = [(k, f) for (k, mm), f in sorted(grid.items()) if mm == m]
        full = sum(1 for _, f in row if f > 0.98)
        part = [(k, round(f, 2)) for k, f in row if f <= 0.98]
        tot += sum(f for _, f in row)
        print("row %2d: %2d full + %d partial %s" % (m, full, len(part), part))
    print("cells in total (area / pitch^2): %.1f; full %d, partial %d" % (
        tot, sum(1 for f in grid.values() if f > 0.98), sum(1 for f in grid.values() if f <= 0.98)))
    # picture
    px = 120
    img = np.full((int(4.4 * px), int(4.4 * px), 3), 255, np.uint8)

    def pt(u, v):
        return (int(u * px) + 20, int((4.2 - v) * px))

    for (k, m), f in grid.items():
        c = (0, 160, 0) if f > 0.98 else (0, 140, 255)
        cv2.rectangle(img, pt(WALL0 + PITCH * k, WALL0 + PITCH * m), pt(WALL0 + PITCH * (k + 1), WALL0 + PITCH * (m + 1)), c, 1)
        if f <= 0.98:
            cv2.putText(img, "%d" % round(f * 100), pt(WALL0 + PITCH * (k + 0.2), WALL0 + PITCH * (m + 0.6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)
    cv2.polylines(img, [np.array([pt(*p) for p in OUT], np.int32)], True, (255, 0, 0), 2)
    cv2.line(img, pt(RIB_S * np.sqrt(0.5) * 2 / 2 * 1.0 * np.sqrt(2) * 0.5 / np.sqrt(0.5) * 0 + 0, 5.3), pt(1.4, 3.9), (0, 0, 0), 1)
    cv2.line(img, pt(1.422, 3.918), pt(3.918, 1.422), (0, 0, 0), 2)  # cross rib
    cv2.line(img, pt(0, 0), pt(4.0, 4.0), (255, 0, 255), 1)  # axis
    cv2.imwrite(sys.argv[1], img)
    if len(sys.argv) > 3:
        pts = np.load(sys.argv[2])
        h = np.load(sys.argv[3])
        sys.path.insert(0, ".")
        import os
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from fin_lattice import assign_indices
        idx, _ = assign_indices(pts)
        ij = np.array(sorted(idx.values()))
        for ax, name in ((1, "rows j (cells along i)"), (0, "rows i (cells along j)")):
            vals, counts = np.unique(ij[:, ax], return_counts=True)
            print("template", name, dict(zip(vals.tolist(), counts.tolist())), "total", len(ij))


main()
