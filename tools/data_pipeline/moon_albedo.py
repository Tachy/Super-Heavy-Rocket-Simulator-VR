"""Large-scale lunar albedo from the LROC WAC Hapke w map (Sato et al. 2014) for the Moon's colour texture.

The CGI Moon Kit colour mosaic (sources.toml "moon_color") is a visualisation: its sRGB-decoded luminance puts Mare
Tranquillitatis at 0.069 against a global mean of 0.30 (ratio 0.23), far darker than the photometry (Task 11 fix B,
ruling T11-C4). Here the texture's large-scale brightness follows the measured single-scattering albedo instead:

  A(lat, lon) = I/F(w(lat, lon)) / I/F(w_ref)    (Hapke 2012 with the config's b, c, B_S0, h_S, K = 1; i 30, e 0, g 30 deg,
                                                  the mosaics' standard geometry; smooth surface: the roughness factor
                                                  does not depend on w and cancels in the ratio)

w bilinear from the 1-degree map (+-70 deg; fill values and the poles: from the mosaic, below). The mosaic keeps the hue
(rgb / luminance) and the detail below the map's 1 degree, with its contrast compressed by the exponent gamma that maps
the mosaic's 1-degree luminance onto A (least squares of log A on log L over the valid cells): out luminance

  L_out = s x A_1deg x (L / L_1deg)^gamma,   s: the global area-weighted mean of L_out equals the mosaic's (the
                                             regolith scale R is re-derived from the texture anyway:
                                             tools/validation/moon_disc_calibration.py).

Beyond +-70 deg (no map) and in fill cells A_1deg = exp(c) L_1deg^gamma (the same fit), blended over 65-70 deg.
Pure numpy + Pillow; deterministic. Used by prepare.py; the numbers are printed (and asserted in test_moon_albedo.py).
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from PIL import Image

# [S] PDS3 label of WAC_HAPKEPARAMMAP_643NM.IMG: 4 label records of 1440 bytes, BAND_SEQUENTIAL 9 x 140 x 360
# IEEE754 LSB float32 (W, B, C, BC0, HC, BS0, HS, THETA, PHI), 1 deg, lat 70 .. -70 (line 0 centre 69.5), lon 0 .. 360 E
# (sample 0 centre 0.5), missing constant 0xFF7FFFFB.
LABEL_BYTES = 4 * 1440
BANDS, LINES, SAMPLES = 9, 140, 360
MAP_LAT_MAX_DEG = 70.0
MISSING_LIMIT = -1.0e30
BLEND_START_DEG = 65.0  # [E] the fit takes over between 65 and 70 deg
# [D] The mosaics' standard geometry (also the statistical rock layer's rho, unresolved_shading.cpp).
STANDARD_I_DEG, STANDARD_E_DEG, STANDARD_G_DEG = 30.0, 0.0, 30.0


def read_w_map(path: Path) -> np.ndarray:
    """The w band (140 x 360, lat 69.5 .. -69.5, lon 0.5 .. 359.5 E); NaN where missing."""
    data = np.fromfile(path, dtype="<f4", offset=LABEL_BYTES, count=BANDS * LINES * SAMPLES)
    w = data.reshape(BANDS, LINES, SAMPLES)[0].astype(np.float64)
    w[(w < MISSING_LIMIT) | ~np.isfinite(w) | (w <= 0.0) | (w >= 1.0)] = np.nan
    return w


def h_function(x: np.ndarray, w: np.ndarray) -> np.ndarray:
    """Hapke 2002 H function (the core's hapke_h_function, default form)."""
    x = np.maximum(x, 1e-12)
    gamma = np.sqrt(1.0 - w)
    r0 = (1.0 - gamma) / (1.0 + gamma)
    return 1.0 / (1.0 - w * x * (r0 + 0.5 * (1.0 - 2.0 * r0 * x) * np.log((1.0 + x) / x)))


def reflectance(w: np.ndarray, hapke: dict) -> np.ndarray:
    """Hapke I/F (smooth, K = 1) at the standard geometry for single-scattering albedo w."""
    mu0, mu = math.cos(math.radians(STANDARD_I_DEG)), math.cos(math.radians(STANDARD_E_DEG))
    g = math.radians(STANDARD_G_DEG)
    b, c = hapke["b"], hapke["c"]
    p = 0.5 * (1 + c) * (1 - b * b) / (1 - 2 * b * math.cos(g) + b * b) ** 1.5 \
        + 0.5 * (1 - c) * (1 - b * b) / (1 + 2 * b * math.cos(g) + b * b) ** 1.5
    shoe = 1.0 + hapke["bs0"] / (1.0 + math.tan(0.5 * g) / hapke["hs"])
    return 0.25 * w * mu0 / (mu0 + mu) * (p * shoe + h_function(mu0, w) * h_function(mu, w) - 1.0)


def srgb_to_linear(v: np.ndarray) -> np.ndarray:
    return np.where(v <= 0.04045, v / 12.92, ((v + 0.055) / 1.055) ** 2.4)


def linear_to_srgb(v: np.ndarray) -> np.ndarray:
    v = np.clip(v, 0.0, 1.0)
    return np.where(v <= 0.0031308, 12.92 * v, 1.055 * v ** (1.0 / 2.4) - 0.055)


def _cell_means(lum: np.ndarray) -> np.ndarray:
    """The texture's luminance averaged over 1-degree cells (180 x 360, lat 89.5 .. -89.5, lon -179.5 .. 179.5)."""
    h, w = lum.shape
    rows = (np.arange(h) * 180) // h
    cols = (np.arange(w) * 360) // w
    sums = np.zeros((180, 360))
    counts = np.zeros((180, 360))
    np.add.at(sums, (rows[:, None], cols[None, :]), lum)
    np.add.at(counts, (rows[:, None], cols[None, :]), 1.0)
    return sums / counts


def _bilinear_cells(cells: np.ndarray, h: int, w: int) -> np.ndarray:
    """A 180 x 360 cell field (centres) bilinear on the texture's pixel centres (lon wraps, lat clamps)."""
    lat_idx = (np.arange(h) + 0.5) * 180.0 / h - 0.5
    lon_idx = (np.arange(w) + 0.5) * 360.0 / w - 0.5
    i0 = np.clip(np.floor(lat_idx).astype(int), 0, 178)
    fi = np.clip(lat_idx - i0, 0.0, 1.0)
    j0 = np.floor(lon_idx).astype(int)
    fj = lon_idx - j0
    j0 %= 360
    j1 = (j0 + 1) % 360
    top = cells[i0][:, j0] * (1 - fj) + cells[i0][:, j1] * fj
    bottom = cells[i0 + 1][:, j0] * (1 - fj) + cells[i0 + 1][:, j1] * fj
    return top * (1 - fi)[:, None] + bottom * fi[:, None]


def tie_to_w_map(rgb_srgb8: np.ndarray, w_map: np.ndarray, hapke: dict) -> tuple[np.ndarray, dict]:
    """The colour texture (uint8 sRGB, lon -180..180, lat 90..-90) with its large-scale albedo from the w map."""
    rgb = srgb_to_linear(rgb_srgb8.astype(np.float64) / 255.0)
    lum = 0.2126 * rgb[..., 0] + 0.7152 * rgb[..., 1] + 0.0722 * rgb[..., 2]
    h, w = lum.shape
    cell_lat = 89.5 - np.arange(180)
    area = np.cos(np.radians(cell_lat))[:, None] * np.ones((1, 360))
    l_cells = _cell_means(lum)
    # The w map on the texture's cell grid: rows 20 .. 159 are lat 69.5 .. -69.5; texture column j (lon -179.5 + j)
    # is map sample (j + 180) mod 360.
    w_cells = np.full((180, 360), np.nan)
    w_cells[20:160] = np.roll(w_map, 180, axis=1)
    a_ref = reflectance(np.array(hapke["w"]), hapke)
    a_cells = reflectance(w_cells, hapke) / a_ref
    valid = np.isfinite(a_cells) & (l_cells > 0.0)
    x, y = np.log(l_cells[valid]), np.log(a_cells[valid])
    gamma, c = np.polyfit(x, y, 1, w=area[valid])
    fit_cells = np.exp(c) * np.maximum(l_cells, 1e-6) ** gamma
    weight = np.clip((MAP_LAT_MAX_DEG - np.abs(cell_lat)) / (MAP_LAT_MAX_DEG - BLEND_START_DEG), 0.0, 1.0)[:, None]
    weight = np.where(np.isfinite(a_cells), weight, 0.0)
    target_cells = np.where(weight > 0.0, weight * np.nan_to_num(a_cells) + (1 - weight) * fit_cells, fit_cells)
    target = _bilinear_cells(target_cells, h, w)
    l_low = _bilinear_cells(l_cells, h, w)
    detail = (np.maximum(lum, 1e-6) / np.maximum(l_low, 1e-6)) ** gamma
    l_out = target * detail
    pixel_area = np.cos(np.radians(90.0 - (np.arange(h) + 0.5) * 180.0 / h))[:, None]
    mean_in = float((lum * pixel_area).sum() / (pixel_area.sum() * w))
    scale = mean_in / float((l_out * pixel_area).sum() / (pixel_area.sum() * w))
    l_out *= scale
    out = rgb * (l_out / np.maximum(lum, 1e-6))[..., None]
    clipped = float((out > 1.0).any(axis=-1).mean())
    out_srgb8 = np.round(linear_to_srgb(out) * 255.0).astype(np.uint8)

    def region(lat0: float, lat1: float, lon0: float, lon1: float, field: np.ndarray) -> float:
        rows = slice(int((90 - lat1) * h / 180), int((90 - lat0) * h / 180))
        cols = slice(int((lon0 + 180) * w / 360), int((lon1 + 180) * w / 360))
        return float(field[rows, cols].mean())

    out_lum = srgb_to_linear(out_srgb8.astype(np.float64) / 255.0)
    out_lum = 0.2126 * out_lum[..., 0] + 0.7152 * out_lum[..., 1] + 0.0722 * out_lum[..., 2]
    # [S] regions: Mare Tranquillitatis around the Apollo 11 site (0-8 N, 20-30 E); highland south of Descartes
    # (12-20 S, 10-20 E); global means area-weighted.
    stats = {
        "gamma": float(gamma), "fit_c": float(c), "scale": scale, "clipped_share": clipped,
        "w_mare": float(np.nanmean(w_cells[82:90, 200:210])), "w_highland": float(np.nanmean(w_cells[102:110, 190:200])),
        "mosaic_mare": region(0, 8, 20, 30, lum), "mosaic_highland": region(-20, -12, 10, 20, lum),
        "out_mare": region(0, 8, 20, 30, out_lum), "out_highland": region(-20, -12, 10, 20, out_lum),
        "mosaic_mean": mean_in, "out_mean": float((out_lum * pixel_area).sum() / (pixel_area.sum() * w)),
    }
    stats["mosaic_ratio"] = stats["mosaic_mare"] / stats["mosaic_highland"]
    stats["out_ratio"] = stats["out_mare"] / stats["out_highland"]
    stats["reflectance_ratio"] = float(reflectance(np.array(stats["w_mare"]), hapke) /
                                       reflectance(np.array(stats["w_highland"]), hapke))
    return out_srgb8, stats


def load_hapke(config_path: Path) -> dict:
    import tomllib

    with config_path.open("rb") as f:
        return tomllib.load(f)["hapke"]


def prepare(mosaic_tif: Path, w_map_path: Path, config_path: Path, out_png: Path) -> dict:
    """moon_color.png from the colour mosaic, tied to the w map (the mosaic as it is when the map is absent)."""
    with Image.open(mosaic_tif) as moon:
        rgb = np.asarray(moon.convert("RGB"))
    if not w_map_path.exists():
        Image.fromarray(rgb).save(out_png, optimize=True)
        return {}
    out, stats = tie_to_w_map(rgb, read_w_map(w_map_path), load_hapke(config_path))
    Image.fromarray(out).save(out_png, optimize=True)
    return stats
