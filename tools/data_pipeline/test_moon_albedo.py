"""Tests of moon_albedo.py (the colour texture's large-scale albedo from the WAC Hapke w map)."""
import math

import numpy as np

import moon_albedo

HAPKE = {"w": 0.429, "b": 0.220, "c": 0.514, "bs0": 1.61, "hs": 0.054}


def test_reflectance_grows_with_w_and_is_hapke():
    w = np.array([0.2, 0.3, 0.429, 0.6])
    r = moon_albedo.reflectance(w, HAPKE)
    assert np.all(np.diff(r) > 0)
    # Hand evaluation at w = 0.429: i 30, e 0, g 30 deg.
    mu0, mu, g = math.cos(math.radians(30)), 1.0, math.radians(30)
    b, c = HAPKE["b"], HAPKE["c"]
    p = 0.5 * (1 + c) * (1 - b * b) / (1 - 2 * b * math.cos(g) + b * b) ** 1.5 \
        + 0.5 * (1 - c) * (1 - b * b) / (1 + 2 * b * math.cos(g) + b * b) ** 1.5
    shoe = 1 + HAPKE["bs0"] / (1 + math.tan(g / 2) / HAPKE["hs"])

    def h(x, ww):
        gm = math.sqrt(1 - ww)
        r0 = (1 - gm) / (1 + gm)
        return 1 / (1 - ww * x * (r0 + 0.5 * (1 - 2 * r0 * x) * math.log((1 + x) / x)))

    expected = 0.25 * 0.429 * mu0 / (mu0 + mu) * (p * shoe + h(mu0, 0.429) * h(mu, 0.429) - 1)
    assert abs(r[2] / expected - 1) < 1e-12


def test_tie_maps_large_scale_contrast_onto_the_w_map():
    # A synthetic mosaic: a dark "mare" half (luminance 0.04) and a bright half (0.30), fine noise; the w map says
    # w 0.22 / 0.39 for the halves. The output's half means must follow the Hapke I/F ratio, the global mean is kept,
    # the hue is kept and the fine detail survives.
    rng = np.random.default_rng(1)
    h, w = 360, 720
    lum = np.where(np.arange(w)[None, :] < w // 2, 0.04, 0.30) * np.ones((h, 1))
    lum = lum * (1.0 + 0.1 * rng.standard_normal((h, w)))
    lin = np.stack([lum * 1.05, lum, lum * 0.9], axis=-1)
    srgb8 = np.round(moon_albedo.linear_to_srgb(lin) * 255).astype(np.uint8)
    w_map = np.where((np.arange(360)[None, :] + 180) % 360 < 180, 0.22, 0.39) * np.ones((140, 1))
    out, stats = moon_albedo.tie_to_w_map(srgb8, w_map, HAPKE)
    expected = float(moon_albedo.reflectance(np.array(0.22), HAPKE) / moon_albedo.reflectance(np.array(0.39), HAPKE))
    rgb = moon_albedo.srgb_to_linear(out.astype(np.float64) / 255)
    out_lum = 0.2126 * rgb[..., 0] + 0.7152 * rgb[..., 1] + 0.0722 * rgb[..., 2]
    band = slice(h // 2 - 40, h // 2 + 40)  # within +-20 deg, away from the half boundary and the wrap
    dark = out_lum[band, w // 8: 3 * w // 8].mean()
    bright = out_lum[band, 5 * w // 8: 7 * w // 8].mean()
    assert abs(dark / bright / expected - 1) < 0.03
    assert abs(stats["out_mean"] / stats["mosaic_mean"] - 1) < 0.01
    hue = rgb[band, 5 * w // 8: 7 * w // 8]
    assert abs((hue[..., 0] / hue[..., 1]).mean() - 1.05) < 0.03
    detail = out_lum[band, 5 * w // 8: 7 * w // 8]
    assert detail.std() / detail.mean() > 0.02  # the 10 % noise, compressed by gamma, not removed
