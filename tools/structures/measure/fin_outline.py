"""Outline of the factory grid fin in fin coordinates from the vertices read off fin_rectify.py's picture.

Usage: python fin_outline.py   (prints the vertex table, side / chamfer lengths in cell pitches and metres, area)
Vertex pixels are read by eye from screenshots/vehicles/real/fo_rect.png (+-8 px). The wall pitch in that picture is
PITCH_PX (spectrum peak of fin_rectify.py, 75.7 px, +-5 %). The lattice is turned to exactly +-45 deg to the axes
(measured slash / backslash medians 39.0 / 127.3 deg: rotation ROT_DEG), z = right, y = down from the root vertex.
"""
from __future__ import annotations

import math

PITCH_PX = 75.7
PITCH_M = 0.49  # [E] from the 4.1 m overall width (NSF 2026-03) over the 8.3 pitches measured here: 0.495 m
ROT_DEG = 6.5
NAMES = "ABCDEFGHI"
VERTS_PX = [(128, 88), (350, 28), (470, 92), (655, 230), (705, 320), (712, 525), (668, 605), (300, 620), (88, 395)]


def outline_m():
    s = PITCH_M / PITCH_PX
    c, sn = math.cos(math.radians(ROT_DEG)), math.sin(math.radians(ROT_DEG))
    pts = []
    for x, y in VERTS_PX:
        ux, uy = x * s, -y * s  # y up
        pts.append((c * ux - sn * uy, sn * ux + c * uy))
    z0 = (pts[1][0] + pts[2][0]) / 2.0  # centre of the root chamfer B-C
    y0 = min(p[1] for p in pts)
    y0 = pts[1][1] if pts[1][1] > pts[2][1] else pts[2][1]
    return [(round(p[0] - z0, 3), round(y0 - p[1], 3)) for p in pts]  # (z, y): y = span from the root, z across


def main():
    pts = outline_m()
    n = len(pts)
    for i in range(n):
        (z0, y0), (z1, y1) = pts[i], pts[(i + 1) % n]
        length = math.hypot(z1 - z0, y1 - y0)
        print(f"{NAMES[i]}{NAMES[(i + 1) % n]} z {z0:+.2f} y {y0:.2f}  side {length:.2f} m = {length / PITCH_M:.1f} pitches"
              f"  angle {math.degrees(math.atan2(y1 - y0, z1 - z0)):.0f} deg")
    area = 0.5 * abs(sum(pts[i][0] * pts[(i + 1) % n][1] - pts[(i + 1) % n][0] * pts[i][1] for i in range(n)))
    cx = sum(p[0] for p in pts) / n
    zs, ys = [p[0] for p in pts], [p[1] for p in pts]
    print("width", max(zs) - min(zs), "span", max(ys) - min(ys), "area", area)
    print("outline_z_m =", [p[0] for p in pts])
    print("outline_y_m =", [p[1] for p in pts])


if __name__ == "__main__":
    main()
