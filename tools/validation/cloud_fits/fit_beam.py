"""Least-squares fit of the Sun's beam rule (core constants.hpp cloud_sun_beam_zenith_exponent / _slant / _slant_power,
look fixes T7c fix 1): the drawn shapes' effective shadow cover 1 - (1 - f)^n, n = n0 + b cot(e)^p, against the area
means of the direct Sun over eight sites (game/tests/cloud_check.gd with STARSIM_SLANT_TABLE=1; f 0.15 / 0.3 / 0.4 / 0.6
x e 5-75 deg, a 1 km layer of grid-mean depth 3). The table measured for the committed constants:
beam_slant_table_2026-11-08.txt (detail levels off). usage: python fit_beam.py [table.log]"""
import math
import os
import re
import sys

import numpy as np
from scipy.optimize import least_squares

TAU_GRID = 3.0  # cloud_check DECK_LIQUID_KG_M3: the test layer's vertical grid-mean optical depth
DEFAULT_TABLE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "beam_slant_table_2026-11-08.txt")


def load(path):
    rows = []
    for line in open(path, encoding="utf-8", errors="replace"):
        m = re.search(r"slant table: cf ([\d.]+) elevation\s+([\d.]+)\s+T ([\d.]+)", line)
        if m:
            rows.append(tuple(float(x) for x in m.groups()))
    return rows


def model(p, f, e_deg):
    """The direct Sun's transmittance of the core's column (independent columns, cloud_optics.hpp) for the rule p."""
    n0, b, power = p
    mu = math.sin(math.radians(e_deg))
    cot = math.cos(math.radians(e_deg)) / mu
    c = 1.0 - (1.0 - f) ** (n0 + b * cot ** power)
    return 1.0 - c + c * math.exp(-TAU_GRID / mu / c)


def main():
    rows = load(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_TABLE)
    fit = least_squares(lambda p: [model(p, f, e) - t for f, e, t in rows], [0.64, 0.02, 1.8],
                        bounds=([0.3, 0.0, 0.5], [1.5, 1.0, 4.0]))
    print("fit n0 %.4f b %.4f p %.4f  rms %.4f worst %.4f" % (*fit.x, np.sqrt(np.mean(np.square(fit.fun))),
                                                          np.max(np.abs(fit.fun))))
    committed = [model((0.62, 0.018, 1.85), f, e) - t for f, e, t in rows]
    print("committed (0.62, 0.018, 1.85): rms %.4f worst %.4f" % (np.sqrt(np.mean(np.square(committed))),
                                                                 np.max(np.abs(committed))))
    for (f, e, t), r in zip(rows, fit.fun):
        print("  f %.2f e %4.1f  shapes %.4f  rule %.4f  d %+.4f" % (f, e, t, t + r, r))


if __name__ == "__main__":
    main()
