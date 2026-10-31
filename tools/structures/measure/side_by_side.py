"""Side by side of a reference frame (left) and a render (right), both scaled to the same height: diagnostic only.

    python tools/structures/measure/side_by_side.py <reference> <render> <out.png> [height_px]
"""
import sys

from PIL import Image


def main() -> None:
    ref, render, out = (Image.open(p).convert("RGB") for p in sys.argv[1:3] + [sys.argv[2]])
    height = int(sys.argv[4]) if len(sys.argv) > 4 else 700
    images = [im.resize((round(im.width * height / im.height), height)) for im in (ref, render)]
    sheet = Image.new("RGB", (sum(im.width for im in images), height))
    x = 0
    for im in images:
        sheet.paste(im, (x, 0))
        x += im.width
    sheet.save(sys.argv[3])


if __name__ == "__main__":
    main()
