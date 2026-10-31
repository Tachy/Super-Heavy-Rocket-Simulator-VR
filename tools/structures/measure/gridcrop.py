"""Crop of a reference image with a labelled pixel grid, for identifying points by eye (original pixel coordinates).

Usage: python gridcrop.py <image> <x0> <y0> <x1> <y1> <out.png> [grid_px] [max_side_px]
The output goes to screenshots/<topic>/ (git-ignored); the grid labels are ORIGINAL image coordinates.
"""
from __future__ import annotations

import sys

from PIL import Image, ImageDraw


def gridcrop(path: str, box: tuple[int, int, int, int], out: str, grid: int = 50, max_side: int = 1400) -> None:
    img = Image.open(path).convert("RGB")
    x0, y0, x1, y1 = box
    crop = img.crop(box)
    scale = min(max_side / crop.width, max_side / crop.height)
    crop = crop.resize((max(1, round(crop.width * scale)), max(1, round(crop.height * scale))), Image.LANCZOS)
    d = ImageDraw.Draw(crop)
    gx = (x0 // grid + 1) * grid
    while gx < x1:
        px = (gx - x0) * scale
        d.line([(px, 0), (px, crop.height)], fill=(255, 0, 255) if gx % (grid * 2) == 0 else (0, 200, 255), width=1)
        if gx % (grid * 2) == 0:
            d.text((px + 2, 2), str(gx), fill=(255, 255, 0))
        gx += grid
    gy = (y0 // grid + 1) * grid
    while gy < y1:
        py = (gy - y0) * scale
        d.line([(0, py), (crop.width, py)], fill=(255, 0, 255) if gy % (grid * 2) == 0 else (0, 200, 255), width=1)
        if gy % (grid * 2) == 0:
            d.text((2, py + 2), str(gy), fill=(255, 255, 0))
        gy += grid
    crop.save(out)


if __name__ == "__main__":
    a = sys.argv
    gridcrop(a[1], (int(a[2]), int(a[3]), int(a[4]), int(a[5])), a[6], int(a[7]) if len(a) > 7 else 50,
             int(a[8]) if len(a) > 8 else 1400)
