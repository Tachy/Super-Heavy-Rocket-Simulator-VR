"""Scotopic luminance weights of the linear sRGB channels (night vision, game/world/eye_adaptation.gd).

The CIE 1951 scotopic luminous efficiency V'(lambda) is projected onto the linear sRGB channels (r, g, b colour
matching functions = XYZ_TO_SRGB x (x, y, z)bar, CIE 1931 2-degree, the table and matrix of
core/include/starsim/environment/atmosphere_spectra.hpp): the weights w minimise the relative error of the
scotopic-to-photopic ratio S/P = w . rgb / Y over a set of natural spectra (FIT_TEMPERATURES_K), with the photopic
channels in cd/m^2 (Y = 0.2126 R + 0.7152 G + 0.0722 B) and the scotopic luminance in scotopic cd/m^2,
L_s = 1700 int(L V') / (683 int(L ybar)) x Y [S: K'm = 1700 lm/W, Km = 683 lm/W, CIE 1951 / CIE 18.2]. Prints the
weights and the S/P error per spectrum (worst 2.2 %).

Offline tool, prints the weights; the game's constants are copied from its output. No arguments.
"""
import math
import re
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SPECTRA = ROOT / "core" / "include" / "starsim" / "environment" / "atmosphere_spectra.hpp"

# [S: CIE 1951 scotopic luminous efficiency V'(lambda), CVRL scvle_1.csv (http://www.cvrl.org), every 5 nm 380-780 nm]
V_PRIME = [
    0.000589, 0.001108, 0.002209, 0.00453, 0.00929, 0.01852, 0.03484, 0.0604, 0.0966, 0.1436, 0.1998, 0.2625,
    0.3281, 0.3931, 0.455, 0.513, 0.567, 0.62, 0.676, 0.734, 0.793, 0.851, 0.904, 0.949, 0.982, 0.998, 0.997,
    0.975, 0.935, 0.88, 0.811, 0.733, 0.65, 0.564, 0.481, 0.402, 0.3288, 0.2639, 0.2076, 0.1602, 0.1212, 0.0899,
    0.0655, 0.0469, 0.03315, 0.02312, 0.01593, 0.01088, 0.00737, 0.00497, 0.003335, 0.002235, 0.001497, 0.001005,
    0.000677, 0.000459, 0.0003129, 0.0002146, 0.000148, 0.0001026, 7.15e-05, 5.01e-05, 3.533e-05, 2.501e-05,
    1.78e-05, 1.273e-05, 9.14e-06, 6.6e-06, 4.78e-06, 3.482e-06, 2.546e-06, 1.87e-06, 1.379e-06, 1.022e-06, 7.6e-07,
    5.67e-07, 4.25e-07, 3.196e-07, 2.413e-07, 1.829e-07, 1.39e-07,
]
SCOTOPIC_KM = 1700.0  # lm/W
PHOTOPIC_KM = 683.0  # lm/W


def read_header():
    text = SPECTRA.read_text(encoding="utf-8")
    block = text[text.index("cie_1931_2deg_cmf[cmf_rows][4]"):]
    block = block[:block.index("};")]
    rows = [[float(x) for x in m.groups()] for m in re.finditer(
        r"\{\s*([\d.eE+-]+),\s*([\d.eE+-]+),\s*([\d.eE+-]+),\s*([\d.eE+-]+)\s*\}", block)]
    cmf = {int(r[0]): r[1:] for r in rows}
    mblock = text[text.index("xyz_to_linear_srgb[9]"):]
    mblock = mblock[mblock.index("{") + 1:mblock.index("}")]
    matrix = np.array([float(x) for x in mblock.replace("\n", " ").split(",") if x.strip()]).reshape(3, 3)
    return cmf, matrix


def planck(wavelength_nm, temperature_k):
    lam = wavelength_nm * 1e-9
    return 1.0 / (lam ** 5 * (math.exp(1.4387769e-2 / (lam * temperature_k)) - 1.0))


# [call] The fit's spectra: black bodies 2000-25 000 K (sunlight, moonlight, lamps) and the same x (lambda / 550 nm)^-4
# (Rayleigh-blue sky and twilight light), each weighted equally in relative S/P error. An unweighted projection of
# V'(lambda) onto the rgb functions (their negative lobes) erred 26 % on illuminant A.
FIT_TEMPERATURES_K = [2000, 2500, 2856, 3500, 4100, 5000, 5800, 6504, 8000, 10000, 15000, 25000]


def main():
    cmf, matrix = read_header()
    wavelengths = list(range(380, 785, 5))
    xyz = np.array([cmf[w] for w in wavelengths])  # (n, 3)
    rgb_cmf = xyz @ matrix.T  # (n, 3): linear sRGB colour matching functions
    v_prime = np.array(V_PRIME)
    rows, targets, names = [], [], []
    for temperature in FIT_TEMPERATURES_K:
        black_body = np.array([planck(w, temperature) for w in wavelengths])
        for name, spectrum in (("%d K" % temperature, black_body),
                               ("%d K x lambda^-4" % temperature, black_body * (np.array(wavelengths) / 550.0) ** -4)):
            photopic = PHOTOPIC_KM * spectrum @ xyz[:, 1]
            rows.append(PHOTOPIC_KM * spectrum @ rgb_cmf / photopic)  # rgb channels per photopic cd/m^2
            targets.append(SCOTOPIC_KM * spectrum @ v_prime / photopic)  # S/P
            names.append(name)
    rows, targets = np.array(rows), np.array(targets)
    weights, *_ = np.linalg.lstsq(rows / targets[:, None], np.ones(len(targets)), rcond=None)
    print("scotopic weights (scotopic cd/m^2 per photopic-cd/m^2 channel unit): %.4f %.4f %.4f" % tuple(weights))
    errors = rows @ weights / targets - 1.0
    for name, target, error in zip(names, targets, errors):
        print("%-22s S/P exact %.3f, from the weights %+.1f %%" % (name, target, 100.0 * error))
    print("worst relative S/P error %.1f %%; S/P of rgb white (1, 1, 1): %.3f" % (100.0 * np.max(np.abs(errors)),
                                                                                  weights.sum()))


if __name__ == "__main__":
    main()
