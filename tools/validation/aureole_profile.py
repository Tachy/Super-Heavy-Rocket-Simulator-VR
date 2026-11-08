"""Look-fixes T9b: the sky's aureole around a centred source in a render (render_check --look_at_moon / --look_at_sun).

Linear display luminance (sRGB decoded, Rec. 709 weights) in thin rings at angular distances from the frame centre,
divided by the ring at the reference angle (default 25 deg): the measure of T13 fix 1 (open point 3) against
Krisciunas & Schaefer 1991 and the Mie phase function. The source must sit at the frame centre (the look_at options
aim it there); fov is the horizontal field of view in degrees.

Usage: python -I tools/validation/aureole_profile.py <png> <fov_deg> [ref_deg]
"""
import math
import sys

import numpy as np
from PIL import Image

ANGLES_DEG = (2.0, 3.0, 5.0, 10.0, 15.0, 25.0)


def linear_luma(png):
    a = np.asarray(Image.open(png).convert("RGB")).astype(float) / 255.0
    lin = np.where(a <= 0.04045, a / 12.92, ((a + 0.055) / 1.055) ** 2.4)
    return lin @ np.array([0.2126, 0.7152, 0.0722]), lin


def profile(png, fov_deg, ref_deg=25.0):
    y, lin = linear_luma(png)
    h, w = y.shape
    f = (w / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    yy, xx = np.mgrid[0:h, 0:w]
    r = np.hypot(xx - (w - 1) / 2.0, yy - (h - 1) / 2.0)
    theta = np.degrees(np.arctan(r / f))
    out = {}
    for a in ANGLES_DEG + (ref_deg,):
        ring = (theta >= a - 0.25) & (theta < a + 0.25)
        out[a] = (float(y[ring].mean()), float(lin[ring][:, 2].mean() / max(lin[ring][:, 0].mean(), 1e-9)))
    ref = out[ref_deg][0]
    return {a: (v / ref if ref > 0 else float("nan"), br) for a, (v, br) in out.items()}


def main():
    png, fov = sys.argv[1], float(sys.argv[2])
    ref = float(sys.argv[3]) if len(sys.argv) > 3 else 25.0
    p = profile(png, fov, ref)
    print(f"{png}: L(theta) / L({ref:g} deg) " + "  ".join(f"{a:g}: {v:.2f} (B/R {br:.2f})" for a, (v, br) in p.items()))


if __name__ == "__main__":
    main()
