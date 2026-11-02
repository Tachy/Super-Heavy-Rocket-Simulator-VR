"""Read the stations of a side-on launcher image (row by row): silhouette extent and bright/dark bands along the axis.

usage: python -I stack_profile.py IMAGE AXIS_X_PX [--half-width PX] [--threshold 40] [--crop-out OUT.png TOP BOTTOM]
Prints the first and last rows of the silhouette, its width over rows, and every change of the mean brightness on the
axis. No third-party data is stored here; the image is an argument.
"""
import argparse

from PIL import Image


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("image")
    p.add_argument("axis_x_px", type=int)
    p.add_argument("--half-width", type=int, default=30, help="axis strip half width in pixels")
    p.add_argument("--threshold", type=int, default=40, help="silhouette: luminance above the black background")
    p.add_argument("--band-step", type=int, default=60, help="report a band change above this luminance step")
    a = p.parse_args()
    im = Image.open(a.image).convert("L")
    w, h = im.size
    px = im.load()
    rows = []
    for y in range(h):
        lit = [x for x in range(w) if px[x, y] > a.threshold]
        rows.append((y, lit[0] if lit else None, lit[-1] if lit else None))
    occupied = [r for r in rows if r[1] is not None]
    print(f"size {w} x {h}; lit rows {occupied[0][0]} .. {occupied[-1][0]}")
    last = None
    for y in range(h):
        strip = [px[x, y] for x in range(a.axis_x_px - a.half_width, a.axis_x_px + a.half_width + 1)]
        mean = sum(strip) / len(strip)
        if last is None or abs(mean - last) > a.band_step:
            print(f"row {y}: axis luminance {mean:.0f}")
            last = mean
    step = max(1, h // 80)
    for y, x0, x1 in rows[::step]:
        if x0 is not None:
            print(f"row {y}: lit x {x0}..{x1} width {x1 - x0 + 1}")


if __name__ == "__main__":
    main()
