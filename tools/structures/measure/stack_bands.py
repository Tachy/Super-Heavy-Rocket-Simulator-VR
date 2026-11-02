"""Band edges along a rocket's axis in a photo, and the hull's width: for reading stage heights against a known length.

usage: python -I stack_bands.py IMAGE AXIS_X ROW_FROM ROW_TO [--half-width 12] [--step 45]
       --edges: only prints; hull width is found in row_width by the steepest brightness change left and right of the axis.
The image is an argument; nothing of it is stored.
"""
import argparse

from PIL import Image


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("image")
    p.add_argument("axis_x", type=int)
    p.add_argument("row_from", type=int)
    p.add_argument("row_to", type=int)
    p.add_argument("--half-width", type=int, default=12)
    p.add_argument("--step", type=float, default=45.0)
    p.add_argument("--reach", type=int, default=250, help="search for the hull edges within this many pixels of the axis")
    a = p.parse_args()
    im = Image.open(a.image).convert("L")
    px = im.load()
    means = []
    for y in range(a.row_from, a.row_to):
        strip = [px[x, y] for x in range(a.axis_x - a.half_width, a.axis_x + a.half_width + 1)]
        means.append(sum(strip) / len(strip))
    # edges: the local maximum of the change over 4 rows
    grad = [means[i + 4] - means[i] for i in range(len(means) - 4)]
    i = 1
    while i < len(grad) - 1:
        if abs(grad[i]) > a.step and abs(grad[i]) >= abs(grad[i - 1]) and abs(grad[i]) >= abs(grad[i + 1]):
            print(f"edge at row {a.row_from + i + 2}: {means[i]:.0f} -> {means[i + 4]:.0f}")
            i += 6
        else:
            i += 1
    step = max(1, (a.row_to - a.row_from) // 25)
    for y in range(a.row_from, a.row_to, step):
        row = [px[x, y] for x in range(a.axis_x - a.reach, a.axis_x + a.reach + 1)]
        left = max(range(3, a.reach), key=lambda k: abs(row[k + 2] - row[k - 1]))
        right = max(range(a.reach + 1, 2 * a.reach - 2), key=lambda k: abs(row[k + 2] - row[k - 1]))
        print(f"row {y}: edges x {a.axis_x - a.reach + left} .. {a.axis_x - a.reach + right}, width {right - left}")


if __name__ == "__main__":
    main()
