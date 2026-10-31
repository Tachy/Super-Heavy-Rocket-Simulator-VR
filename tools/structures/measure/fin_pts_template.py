"""Cell-centre points of the installed-fin template image (hearts against the sky): bright connected components inside the
fin that do not touch the sky. Usage: python fin_pts_template.py <image> <out.npy> <overlay.png>"""
import sys

import cv2
import numpy as np

img = cv2.imread(sys.argv[1])
b, g, r = [c.astype(int) for c in cv2.split(img)]
sky = (b > 150) & (b - r > 40)  # blue sky and sky seen through the cells
sky = sky.astype(np.uint8)
n, lab, stats, cen = cv2.connectedComponentsWithStats(sky, connectivity=4)
h, w = sky.shape
pts = []
for k in range(1, n):
    x, y, bw, bh, area = stats[k]
    if x == 0 or y == 0 or x + bw >= w or y + bh >= h:
        continue
    if 30 <= area <= 400:
        pts.append(cen[k])
pts = np.array(pts)
print(len(pts), "components; area median", np.median([stats[k][4] for k in range(1, n) if 30 <= stats[k][4] <= 400]))
np.save(sys.argv[2], pts)
ov = cv2.resize(img, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC)
for p in pts:
    cv2.circle(ov, (int(p[0] * 3 + 1), int(p[1] * 3 + 1)), 4, (0, 0, 255), -1)
cv2.imwrite(sys.argv[3], ov)
