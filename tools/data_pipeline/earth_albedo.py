"""Physically calibrated Earth surface albedo: the one transfer of every Earth ground texel, and the global level's
calibration (Task 15 of the Moon procedural surface plan).

Transfer (all Earth imagery: data/textures/earth_day.jpg, the Sentinel-2 and the NAIP tile pyramids of imagery.py):
a texel holds the surface reflectance per channel (R, G, B ~ MODIS bands 1 / 4 / 3 = Sentinel-2 B04 / B03 / B02)
encoded with the sRGB transfer function (IEC 61966-2-1, `encode_srgb`); the shaders decode it once, with the GPU's sRGB
decode of their `source_color` samplers. Reflectance 1 = a white Lambertian surface.

Global level (data/textures/earth_day.jpg, 5400 x 2700, equirectangular): land from a MODIS MCD43A4 v061 nadir
BRDF-adjusted reflectance (NBAR) climatology - per 3.7 km MODIS pixel the median over the twelve mid-month products of
CLIMATOLOGY_YEAR (the Sentinel-2 composites' year, so the levels meet; a month the Planetary Computer lacks - 2025-07 to
2025-11 on 2026-11-05 - from the year before), at least CLIMATOLOGY_MIN_DATES valid, averaged into the texture's pixels
(`build_climatology`, network: Microsoft Planetary Computer, data/raw/earth_albedo_modis_<year>.npz). Where it has no
samples (sea, sea ice, polar night, persistent cloud) the Blue Marble Next Generation (Dec 2004, NASA Earth Observatory)
fills in: a display product (MODIS surface reflectance stretched for appearance, Stoeckli et al. 2005), mapped back to
reflectance by per-channel monotone look-up tables fitted to the MCD43A4 NBAR of its own month (`fit`, stored in
config/earth_albedo.toml). prepare.py writes the texture (`write_day_texture`, offline).

Usage: python tools/data_pipeline/earth_albedo.py fit           (re-fits config/earth_albedo.toml)
       python tools/data_pipeline/earth_albedo.py climatology   (= fetch.py earth_albedo --force)
"""

from __future__ import annotations

import json
import math
import sys
import time
import tomllib
import urllib.error
import urllib.parse
import urllib.request
import warnings
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_FILE = REPO_ROOT / "config" / "earth_albedo.toml"
CHANNELS = ("red", "green", "blue")
LUMA_WEIGHTS = np.array([0.2126, 0.7152, 0.0722])  # [S] ITU-R BT.709 / sRGB luminance
DAY_JPEG_QUALITY = 95  # [E] the calibrated day texture

# MCD43A4 v061 on the Planetary Computer: sinusoidal 10 x 10 deg tiles, 2400 x 2400 px of 463.3 m, int16 x 1e-4
PC_STAC = "https://planetarycomputer.microsoft.com/api/stac/v1"
PC_SIGN = "https://planetarycomputer.microsoft.com/api/sas/v1/sign?href="
MODIS_COLLECTION = "modis-43A4-061"
MODIS_BANDS = ("Nadir_Reflectance_Band1", "Nadir_Reflectance_Band4", "Nadir_Reflectance_Band3")  # [S] red, green, blue
MODIS_SCALE = 1e-4  # [S] MCD43A4 v061 scale factor
MODIS_FILL = 32767  # [S]
MODIS_SPHERE_M = 6371007.181  # [S] MODIS sinusoidal grid sphere
FIT_DATE = "2004-12-16"  # [E] mid-month of the Blue Marble's December 2004 (NBAR window 16 days around it)
FIT_GRID_PX = 300  # MODIS read at 1/8 resolution (3.7 km, the COG's overview) - finer than a 7.4 km day-texture pixel
FIT_MIN_SAMPLES = 2  # MODIS samples per day-texture pixel for a pair
MODIS_MAX_VALID = 16000  # [S] MCD43A4 valid range 0..32766; above 1.6 only artefacts
CLIMATOLOGY_YEAR = 2025  # [E] = imagery.S2_YEAR
CLIMATOLOGY_MIN_DATES = 3  # [E] valid monthly products per MODIS pixel for a median
CLIMATOLOGY_MIN_SAMPLES = 1  # MODIS pixels per day-texture pixel for the climatology to replace the tables
EMBED_FEATHER_M = 50_000.0  # [E] fade of the Sentinel-2 / global ratio outside a Sentinel-2 box
EMBED_SIGMA_PX = 2.0  # [E] Gaussian extension of the box-edge ratio (texture pixels, ~7 km)
CLIMATOLOGY_MIN_LAND_FRACTION = 0.5  # [E] land share of a pixel (ETOPO + geoid) for the climatology
WATER_THRESHOLD_M = 0.05  # [S] = core EarthTerrain::water_threshold_m: orthometric height at or below is sea
OPEN_WATER_DIST_M = 2000.0  # [E] water farther than this from any non-water pixel is open water (no visible bottom)
OPEN_WATER_PERCENTILE = 90.0  # [E] of the imagery over open water: the pedestal above which the brightness is bottom / surf
OPEN_WATER_LEVEL_PERCENTILE = 50.0  # [E] the median of the L2A open water is the calibrated sea level of a Sentinel-2 box
SEA_FEATHER_M = 100_000.0  # [E] fade of the calibrated sea level back to the global sea outside a Sentinel-2 box (cosine)
OPEN_WATER_DECIMATION = 8  # the pedestal is read from every 8th pixel (80 m) of the Sentinel-2 file
OPEN_WATER_MIN_PIXELS = 100  # [E] fewer open-water samples than this give no pedestal (zeros, logged)
LAND_BLOCK_PX = 512  # block of the land-only Sentinel-2 file
LAND_LATTICE_PX = 64  # lat / lon of a block's pixels from a lattice of this spacing (UTM to geographic is smooth: sub-mm)
CLIMATOLOGY_FILE = REPO_ROOT / "data" / "raw" / f"earth_albedo_modis_{CLIMATOLOGY_YEAR}.npz"


# ----------------------------------------------------------------------------------------------- transfer

def encode_srgb(reflectance) -> np.ndarray:
    """Reflectance (any shape, clipped to [0, 1]) -> sRGB-encoded 8-bit counts (IEC 61966-2-1)."""
    r = np.clip(np.asarray(reflectance, dtype=np.float64), 0.0, 1.0)
    v = np.where(r <= 0.0031308, 12.92 * r, 1.055 * np.power(r, 1.0 / 2.4) - 0.055)
    return np.rint(v * 255.0).astype(np.uint8)


def decode_srgb(counts) -> np.ndarray:
    """sRGB-encoded 8-bit counts -> reflectance (float64), the GPU's sRGB decode."""
    c = np.asarray(counts, dtype=np.float64) / 255.0
    return np.where(c <= 0.04045, c / 12.92, np.power((c + 0.055) / 1.055, 2.4))


# ----------------------------------------------------------------------------------------------- look-up tables

def isotonic(values: np.ndarray, weights: np.ndarray) -> np.ndarray:
    """Weighted isotonic (non-decreasing) regression, pool adjacent violators."""
    blocks: list[list[float]] = []  # [mean, weight, count]
    for v, w in zip(values, weights):
        blocks.append([float(v), float(w), 1])
        while len(blocks) > 1 and blocks[-2][0] > blocks[-1][0]:
            m2, w2, n2 = blocks.pop()
            m1, w1, n1 = blocks.pop()
            blocks.append([(m1 * w1 + m2 * w2) / (w1 + w2), w1 + w2, n1 + n2])
    return np.concatenate([np.full(n, m) for m, _, n in blocks])


def monotone_lut(counts: np.ndarray, reflectance: np.ndarray, min_pairs: int = 20) -> np.ndarray:
    """A non-decreasing 256-entry table count -> reflectance from (count, reflectance) pairs: the median per count (counts
    with at least `min_pairs` pairs), isotonic regression weighted by the pair count, linear interpolation between the
    supported counts, proportional to the count below the lowest one (count 0 -> 0) and with the last supported slope
    above the highest one (capped at 1)."""
    counts = np.rint(np.asarray(counts, dtype=np.float64)).astype(int)
    reflectance = np.asarray(reflectance, dtype=np.float64)
    order = np.argsort(counts, kind="stable")
    c_sorted, r_sorted = counts[order], reflectance[order]
    support, medians, weights = [], [], []
    bounds = np.searchsorted(c_sorted, np.arange(257))
    for v in range(256):
        a, b = bounds[v], bounds[v + 1]
        if b - a >= min_pairs:
            support.append(v)
            medians.append(float(np.median(r_sorted[a:b])))
            weights.append(b - a)
    if len(support) < 2:
        raise ValueError("monotone_lut: fewer than two supported counts")
    support = np.array(support, dtype=np.float64)
    fitted = np.maximum(isotonic(np.array(medians), np.array(weights, dtype=np.float64)), 0.0)
    x = np.arange(256, dtype=np.float64)
    lut = np.interp(x, support, fitted)
    lo = support[0]
    if lo > 0:
        lut[x < lo] = fitted[0] * x[x < lo] / lo
    hi = int(support[-1])
    k = max(len(support) - 8, 0)  # slope over the last supported counts
    slope = max((fitted[-1] - fitted[k]) / max(support[-1] - support[k], 1.0), 0.0)
    lut[hi + 1:] = fitted[-1] + slope * (x[hi + 1:] - hi)
    return np.clip(lut, 0.0, 1.0)


def apply_lut(rgb_counts: np.ndarray, luts: np.ndarray) -> np.ndarray:
    """(..., 3) uint8 counts -> (..., 3) reflectance via the (3, 256) per-channel tables."""
    rgb_counts = np.asarray(rgb_counts)
    return np.stack([luts[c][rgb_counts[..., c]] for c in range(3)], axis=-1)


def load_luts(path: Path = CONFIG_FILE) -> np.ndarray:
    with path.open("rb") as f:
        cfg = tomllib.load(f)
    luts = np.array([cfg[f"lut_{c}"] for c in CHANNELS], dtype=np.float64)
    if luts.shape != (3, 256):
        raise ValueError(f"{path}: expected three 256-entry tables")
    return luts


# ----------------------------------------------------------------------------------------------- MODIS (network)

_SAS: dict[str, str] = {}


def pc_sign(href: str) -> str:
    """A Planetary Computer SAS-signed href (one signature per storage container, reused; 429 waits)."""
    key = "/".join(href.split("/")[2:4])
    if key not in _SAS:
        for attempt in range(20):
            try:
                with urllib.request.urlopen(PC_SIGN + urllib.parse.quote(href, safe=""), timeout=60) as r:
                    _SAS[key] = json.load(r)["href"].split("?", 1)[1]
                break
            except urllib.error.HTTPError as e:
                if e.code != 429 or attempt == 19:
                    raise
                time.sleep(60)
    return href + "?" + _SAS[key]


def sinusoidal_to_latlon(x_m: np.ndarray, y_m: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """MODIS sinusoidal grid metres -> (lat, lon) degrees; lon NaN outside the projection's domain."""
    lat = np.degrees(y_m / MODIS_SPHERE_M)
    with np.errstate(divide="ignore", invalid="ignore"):
        lon = np.degrees(x_m / (MODIS_SPHERE_M * np.cos(np.radians(lat))))
    lon[np.abs(lon) > 180.0] = np.nan
    return lat, lon


def modis_items(date: str) -> dict[str, object]:
    """{tile "hXXvYY": item} of the MCD43A4 product dated `date` (YYYY-MM-DD). A datetime search returns every product
    whose 16-day window contains the date, so the items are filtered by their product date (id "A<year><day of year>")."""
    import datetime as dt
    from pystac_client import Client

    d = dt.date.fromisoformat(date)
    tag = f"A{d.year}{d.timetuple().tm_yday:03d}"
    items = Client.open(PC_STAC).search(collections=[MODIS_COLLECTION], datetime=date).items()
    return {it.id.split(".")[2]: it for it in items if it.id.split(".")[1] == tag}


def _read_bands(item) -> tuple[np.ndarray, object] | None:
    """One MCD43A4 tile's red / green / blue NBAR at FIT_GRID_PX (the COG overview) -> (reflectance (n, n, 3), NaN where
    invalid; grid transform), None after repeated read errors."""
    import rasterio
    from rasterio.enums import Resampling

    bands, transform = [], None
    for band in MODIS_BANDS:
        for attempt in range(4):
            try:
                with rasterio.open(pc_sign(item.assets[band].href)) as ds:
                    a = ds.read(1, out_shape=(FIT_GRID_PX, FIT_GRID_PX), resampling=Resampling.nearest)
                    transform = ds.transform * ds.transform.scale(ds.width / FIT_GRID_PX, ds.height / FIT_GRID_PX)
                break
            except Exception:  # noqa: BLE001 - network hiccup, retried
                if attempt == 3:
                    return None
                time.sleep(5 * (attempt + 1))
        bands.append(a)
    stack = np.stack(bands, axis=-1).astype(np.float64)
    bad = ((stack < 0) | (stack > MODIS_MAX_VALID) | (stack == MODIS_FILL)).any(axis=-1)
    rho = stack * MODIS_SCALE
    rho[bad] = np.nan
    return rho, transform


def _to_points(rho: np.ndarray, transform) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    """Valid grid pixels -> (lat, lon, reflectance (m, 3))."""
    valid = np.isfinite(rho).all(axis=-1)
    if not valid.any():
        return None
    rows, cols = np.nonzero(valid)
    lat, lon = sinusoidal_to_latlon(transform.c + (cols + 0.5) * transform.a, transform.f + (rows + 0.5) * transform.e)
    ok = np.isfinite(lon)
    return lat[ok], lon[ok], rho[valid][ok]


class _Bins:
    """Sums of reflectance samples per pixel of an equirectangular h x w grid (lon -180..180, lat 90..-90)."""

    def __init__(self, h: int, w: int):
        self.h, self.w = h, w
        self.sums = np.zeros((h * w, 3))
        self.hits = np.zeros(h * w)

    def add(self, lat, lon, rho) -> None:
        row = np.clip(((90.0 - lat) / 180.0 * self.h).astype(int), 0, self.h - 1)
        col = np.clip(((lon + 180.0) / 360.0 * self.w).astype(int), 0, self.w - 1)
        idx = row * self.w + col
        lo = int(idx.min())
        idx = idx - lo
        n = int(idx.max()) + 1
        self.hits[lo:lo + n] += np.bincount(idx, minlength=n)
        for c in range(3):
            self.sums[lo:lo + n, c] += np.bincount(idx, weights=rho[:, c], minlength=n)

    def mean(self) -> tuple[np.ndarray, np.ndarray]:
        """(mean reflectance (h, w, 3), NaN without samples; sample count (h, w))."""
        with np.errstate(invalid="ignore", divide="ignore"):
            m = self.sums / self.hits[:, None]
        return m.reshape(self.h, self.w, 3), self.hits.reshape(self.h, self.w)


def fit(raw_day: Path, out: Path = CONFIG_FILE, log=print, workers: int = 16) -> dict:
    """Fit the tables: every land tile of the MCD43A4 product of FIT_DATE, averaged into the day texture's pixels."""
    from concurrent.futures import ThreadPoolExecutor
    from PIL import Image

    with Image.open(raw_day) as img:
        day = np.asarray(img.convert("RGB"))
    h, w, _ = day.shape
    items = modis_items(FIT_DATE)
    log(f"  MCD43A4 {FIT_DATE}: {len(items)} tiles", flush=True)
    bins, used = _Bins(h, w), 0
    with ThreadPoolExecutor(workers) as pool:
        for res in pool.map(_read_bands, items.values()):
            pts = _to_points(*res) if res is not None else None
            if pts is not None:
                bins.add(*pts)
                used += 1
    mean, hits = bins.mean()
    pairs = hits.ravel() >= FIT_MIN_SAMPLES
    rho_px = mean.reshape(-1, 3)[pairs]
    counts_px = day.reshape(-1, 3)[pairs]
    luts = np.stack([monotone_lut(counts_px[:, c], rho_px[:, c]) for c in range(3)])
    pred = apply_lut(counts_px, luts)
    y_ref, y_pred = rho_px @ LUMA_WEIGHTS, pred @ LUMA_WEIGHTS
    rel = np.abs(y_pred - y_ref) / np.maximum(y_ref, 1e-3)
    stats = {"tiles": used, "pairs": int(pairs.sum()), "median_rel_luma_error": float(np.median(rel)),
             "p90_rel_luma_error": float(np.percentile(rel, 90)),
             "mean_luma_ref": float(y_ref.mean()), "mean_luma_fit": float(y_pred.mean())}
    log(f"  fit: {stats}", flush=True)
    write_config(out, luts, stats)
    return stats


def _tile_median(tile_items: list) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    """Worker: one tile's per-pixel median over the dates with a valid NBAR (at least CLIMATOLOGY_MIN_DATES)."""
    stacks, transform = [], None
    for item in tile_items:
        res = _read_bands(item)
        if res is not None:
            stacks.append(res[0])
            transform = res[1]
    if not stacks:
        return None
    st = np.stack(stacks)
    n = np.isfinite(st).all(axis=-1).sum(axis=0)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        med = np.nanmedian(st, axis=0)
    med[n < CLIMATOLOGY_MIN_DATES] = np.nan
    return _to_points(med, transform)


def build_climatology(out: Path = CLIMATOLOGY_FILE, size: tuple[int, int] = (2700, 5400), log=print,
                      workers: int = 16) -> dict:
    """The global land reflectance: per MODIS pixel (3.7 km) the median NBAR over the mid-month products of
    CLIMATOLOGY_YEAR, averaged into an equirectangular grid of `size` (the day texture's) -> .npz (rho float32, hits)."""
    from concurrent.futures import ThreadPoolExecutor

    by_tile: dict[str, list] = {}
    for month in range(1, 13):
        for year in (CLIMATOLOGY_YEAR, CLIMATOLOGY_YEAR - 1):  # the Planetary Computer lacks some months: the year before
            date = f"{year}-{month:02d}-15"
            items = modis_items(date)
            log(f"  MCD43A4 {date}: {len(items)} tiles", flush=True)
            if items:
                break
        for tile, item in items.items():
            by_tile.setdefault(tile, []).append(item)
    bins, used = _Bins(*size), 0
    t0 = time.time()
    with ThreadPoolExecutor(workers) as pool:
        for k, pts in enumerate(pool.map(_tile_median, by_tile.values())):
            if pts is not None:
                bins.add(*pts)
                used += 1
            if k % 25 == 0:
                log(f"    tile {k + 1}/{len(by_tile)} ({time.time() - t0:.0f} s)", flush=True)
    mean, hits = bins.mean()
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, rho=mean.astype(np.float32), hits=hits.astype(np.uint16))
    stats = {"tiles": used, "pixels": int((hits > 0).sum())}
    log(f"  climatology: {stats} -> {out}", flush=True)
    return stats


def _lattice_bilinear(nodes: np.ndarray, rows: np.ndarray, cols: np.ndarray, fr: np.ndarray, fc: np.ndarray) -> np.ndarray:
    """Bilinear interpolation of lattice values (nodes at rows x cols) to every pixel row fr x column fc."""
    ri = np.clip(np.searchsorted(rows, fr, side="right") - 1, 0, len(rows) - 2)
    ci = np.clip(np.searchsorted(cols, fc, side="right") - 1, 0, len(cols) - 2)
    wr = ((fr - rows[ri]) / (rows[ri + 1] - rows[ri]))[:, None]
    wc = ((fc - cols[ci]) / (cols[ci + 1] - cols[ci]))[None, :]
    r0, r1, c0, c1 = ri[:, None], (ri + 1)[:, None], ci[None, :], (ci + 1)[None, :]
    return (1 - wr) * ((1 - wc) * nodes[r0, c0] + wc * nodes[r0, c1]) + wr * ((1 - wc) * nodes[r1, c0] + wc * nodes[r1, c1])


def open_water_levels(reflectance: Path, mask: "WaterMask", log=print) -> tuple[np.ndarray, np.ndarray | None]:
    """Per channel (R, G, B) of the Sentinel-2 reflectance over open water (water by the renderer's rule, > `OPEN_WATER_DIST_M`
    from any non-water pixel): (pedestal, level). The pedestal is the `OPEN_WATER_PERCENTILE`: what Sen2Cor L2A reports over
    the sea of this scene (turbidity, glint, scene-wise steps) - not water-leaving reflectance of the calibrated ocean level.
    Water pixels of the tiles keep only the brightness above it (shallow bottoms, flats, surf) on top of the sea. The level
    is the median (`OPEN_WATER_LEVEL_PERCENTILE`): the sea the day texture takes inside the box (`embed_sentinel2`). With
    fewer than OPEN_WATER_MIN_PIXELS open-water samples (logged): pedestal zeros and level None - the embedding then adds the
    imagery's full water brightness and leaves the global sea."""
    import rasterio
    from pyproj import Transformer
    from rasterio.enums import Resampling
    from scipy.ndimage import distance_transform_edt

    f = OPEN_WATER_DECIMATION
    with rasterio.open(reflectance) as ds:
        h, w = ds.height // f, ds.width // f
        a = ds.read(out_shape=(3, h, w), resampling=Resampling.nearest).astype(np.float64) * 1e-4
        t = ds.transform  # north-up
        cols, rows = np.meshgrid((np.arange(w) + 0.5) * f, (np.arange(h) + 0.5) * f)
        lon, lat = Transformer.from_crs(ds.crs, "EPSG:4326", always_xy=True).transform(t.c + t.a * cols, t.f + t.e * rows)
        pixel_m = abs(t.a) * f * (111320.0 if ds.crs.is_geographic else 1.0)  # metres (degrees: at the equator)
    water = mask.water(np.asarray(lat), np.asarray(lon))
    open_water = water & (a > 0).all(axis=0) & (distance_transform_edt(water) * pixel_m > OPEN_WATER_DIST_M)
    if open_water.sum() < OPEN_WATER_MIN_PIXELS:
        log(f"open_water_pedestal: only {int(open_water.sum())} open-water samples in {reflectance.name} (< {OPEN_WATER_MIN_PIXELS}): "
            "pedestal zero, water pixels keep their full imagery brightness on top of the global sea (no calibrated sea level)")
        return np.zeros(3), None
    return (np.percentile(a[:, open_water], OPEN_WATER_PERCENTILE, axis=1),
            np.percentile(a[:, open_water], OPEN_WATER_LEVEL_PERCENTILE, axis=1))


def open_water_pedestal(reflectance: Path, mask: "WaterMask", log=print) -> np.ndarray:
    """The pedestal of `open_water_levels` (per channel 90th percentile of the L2A open water; zeros without samples)."""
    return open_water_levels(reflectance, mask, log)[0]


def build_land_reflectance(reflectance: Path, out: Path, mask: "WaterMask", pedestal: np.ndarray, log=print) -> Path:
    """The Sentinel-2 level split by the renderer's water rule at every 10 m pixel, for the global texture's embedding
    (uint16, reflectance x 1e4; cached beside the source, raw, not shipped): bands 1-3 the land reflectance (water = 0, no
    data), bands 4-6 the brightness above the open-water `pedestal` on water pixels (+ 1 count so that 0 stays no data:
    the excess is the value minus 1) - what the tiles keep on top of the global sea on water pixels."""
    import rasterio
    from pyproj import Transformer
    from rasterio.windows import Window

    if out.exists() and out.stat().st_mtime >= reflectance.stat().st_mtime:
        return out
    partial = out.with_suffix(".partial.tif")
    ped_dn = np.asarray(pedestal, dtype=np.float64)[:, None] * 1e4
    with rasterio.open(reflectance) as src:
        to_geo = Transformer.from_crs(src.crs, "EPSG:4326", always_xy=True)
        profile = dict(src.profile, BIGTIFF="YES", count=6)
        step = LAND_LATTICE_PX
        with rasterio.open(partial, "w", **profile) as dst:
            for r0 in range(0, src.height, LAND_BLOCK_PX):
                for c0 in range(0, src.width, LAND_BLOCK_PX):
                    win = Window(c0, r0, min(LAND_BLOCK_PX, src.width - c0), min(LAND_BLOCK_PX, src.height - r0))
                    data = src.read(window=win)
                    excess = np.zeros_like(data)
                    if data.any():
                        h, w = int(win.height), int(win.width)
                        rows, cols = np.unique(np.append(np.arange(0, h, step), h - 1)), np.unique(np.append(np.arange(0, w, step), w - 1))
                        cg, rg_ = np.meshgrid(cols, rows)
                        t = src.transform  # north-up
                        x, y = t.c + t.a * (c0 + cg + 0.5), t.f + t.e * (r0 + rg_ + 0.5)
                        lon_n, lat_n = to_geo.transform(np.asarray(x), np.asarray(y))
                        fr, fc = np.arange(h), np.arange(w)
                        lat = _lattice_bilinear(lat_n, rows, cols, fr, fc)
                        lon = _lattice_bilinear(lon_n, rows, cols, fr, fc)
                        wet = mask.water(lat, lon)
                        excess[:, wet] = np.minimum(np.maximum(data[:, wet] - ped_dn, 0.0), 65534.0).astype(np.uint16) + 1
                        excess[:, ~(data > 0).all(axis=0)] = 0  # no imagery: no excess either
                        data[:, wet] = 0
                    dst.write(np.concatenate([data, excess]), window=win)
    partial.replace(out)
    log(f"  earth_albedo: land / water-excess Sentinel-2 {out.name}")
    return out


def embed_sentinel2(rho: np.ndarray, repo_root: Path, log=print, land: np.ndarray | None = None) -> np.ndarray:
    """The Sentinel-2 level inside the global one, land only: in every Sentinel-2 box (config/imagery.toml) the land texture
    pixels (`land`) whose centre lies inside take the mean of the Sentinel-2 reflectance of the land pixels (renderer's water
    rule, `build_land_reflectance`), so the two levels draw the same mean on land where the renderer fades between them;
    outside, the log ratio Sentinel-2 / global of the land pixels at the box edge (Gaussian-extended, `EMBED_SIGMA_PX`) fades
    out linearly over `EMBED_FEATHER_M` on land pixels only - the land products' difference (MODIS NBAR climatology vs the
    2025 Sentinel-2 median, ~10-20 %) spread over the feather instead of a step. A sea texture pixel inside a box keeps the
    global sea level plus the mean brightness of its water pixels above the open-water pedestal (what the tiles draw there:
    shallow bottoms, flats, surf), never the Sentinel-2 sea itself (no calibrated ocean product at 10 m)."""
    import cv2
    import rasterio
    from rasterio.transform import from_bounds
    from rasterio.warp import Resampling, reproject

    import earth_terrain as et
    import imagery as im

    if not (repo_root / "config" / "imagery.toml").exists():
        return rho
    cfg = im.load_config(repo_root / "config" / "imagery.toml")
    areas = {a["id"]: a for a in et.load_config(repo_root / "config" / "earth_terrain.toml")["area"]}
    h, w, _ = rho.shape
    for src in cfg["source"]:
        if src["product"] != "sentinel2_median":
            continue
        path = repo_root / "data" / "raw" / "imagery" / src["id"] / "reflectance.tif"
        if not path.exists():
            log(f"  earth_albedo: {path} missing, {src['id']} not embedded")
            continue
        split, sea_level = False, None
        if WaterMask.available(repo_root, src["area"]):
            mask = WaterMask(repo_root, src["area"])
            pedestal, sea_level = open_water_levels(path, mask, log)
            path = build_land_reflectance(path, path.with_name("reflectance_split.tif"), mask, pedestal, log)
            split = True
        else:
            log(f"  earth_albedo: no height grids / geoid for {src['area']}: {src['id']} embedded with its sea")
        south, north, west, east = im.source_box(src, areas)
        lat_c = (south + north) / 2.0
        m_per_px_y = math.pi * 6371000.0 / h
        m_per_px_x = m_per_px_y * 2.0 * h / w * math.cos(math.radians(lat_c))
        reach_m = max(EMBED_FEATHER_M, SEA_FEATHER_M if sea_level is not None else 0.0)
        pad_y, pad_x = int(reach_m / m_per_px_y) + 2, int(reach_m / m_per_px_x) + 2
        r0 = max(int(math.floor((90.0 - north) / 180.0 * h)) - pad_y, 0)
        r1 = min(int(math.ceil((90.0 - south) / 180.0 * h)) + pad_y, h)
        c0 = max(int(math.floor((west + 180.0) / 360.0 * w)) - pad_x, 0)
        c1 = min(int(math.ceil((east + 180.0) / 360.0 * w)) + pad_x, w)
        transform = from_bounds(-180.0 + c0 * 360.0 / w, 90.0 - r1 * 180.0 / h, -180.0 + c1 * 360.0 / w,
                                90.0 - r0 * 180.0 / h, c1 - c0, r1 - r0)
        s2 = np.zeros((3, r1 - r0, c1 - c0), dtype=np.float32)
        with rasterio.open(path) as ds:
            reproject(source=rasterio.band(ds, [1, 2, 3]), destination=s2, dst_transform=transform, dst_crs="EPSG:4326",
                      resampling=Resampling.average, src_nodata=0, dst_nodata=0)
            wet = np.zeros_like(s2)
            if split:
                reproject(source=rasterio.band(ds, [4, 5, 6]), destination=wet, dst_transform=transform, dst_crs="EPSG:4326",
                          resampling=Resampling.average, src_nodata=0, dst_nodata=0)
        s2 = np.moveaxis(s2, 0, -1).astype(np.float64) * im.S2_REFLECTANCE_SCALE
        wet = np.moveaxis(np.where(wet > 0, wet - 1.0, 0.0), 0, -1).astype(np.float64) * im.S2_REFLECTANCE_SCALE  # excess
        lat = 90.0 - (np.arange(r0, r1) + 0.5) * 180.0 / h
        lon = -180.0 + (np.arange(c0, c1) + 0.5) * 360.0 / w
        lon_g, lat_g = np.meshgrid(lon, lat)
        inside = (lat_g > south) & (lat_g < north) & (lon_g > west) & (lon_g < east)
        win = rho[r0:r1, c0:c1].copy()
        land_win = np.ones(inside.shape, dtype=bool) if land is None else land[r0:r1, c0:c1]
        valid = inside & land_win & (s2 > 0).all(axis=-1) & (win > 0).all(axis=-1)
        log_ratio = np.where(valid[..., None], np.log(np.where(valid[..., None], s2 / np.maximum(win, 1e-6), 1.0)), 0.0)
        num = cv2.GaussianBlur((log_ratio * valid[..., None]).astype(np.float32), (0, 0), EMBED_SIGMA_PX)
        den = cv2.GaussianBlur(valid.astype(np.float32), (0, 0), EMBED_SIGMA_PX)
        ext = np.where(den[..., None] > 1e-3, num / np.maximum(den[..., None], 1e-3), 0.0)
        dy = np.maximum(np.maximum(lat_g - north, south - lat_g), 0.0) * 111320.0
        dx = np.maximum(np.maximum(lon_g - east, west - lon_g), 0.0) * 111320.0 * math.cos(math.radians(lat_c))
        fade = np.clip(1.0 - np.hypot(dx, dy) / EMBED_FEATHER_M, 0.0, 1.0)
        sea = win + (wet if split else 0.0) * inside[..., None]
        if sea_level is not None:  # the calibrated sea of the box, fading back to the global sea outside it (cosine)
            reach = np.clip(1.0 - np.hypot(dx, dy) / SEA_FEATHER_M, 0.0, 1.0)
            w_sea = (0.5 - 0.5 * np.cos(np.pi * reach))[..., None]
            sea = win * (1.0 - w_sea) + sea_level * w_sea + (wet if split else 0.0) * inside[..., None]
        out = np.where(land_win[..., None], win * np.exp(ext * fade[..., None]), sea)
        out[valid] = s2[valid]
        rho[r0:r1, c0:c1] = out
        y = LUMA_WEIGHTS
        log(f"  earth_albedo: {src['id']} embedded ({int(valid.sum())} pixels; box mean luminance Sentinel-2 "
            f"{float((s2[valid] @ y).mean()):.4f} vs global {float((win[valid] @ y).mean()):.4f})")
    return rho


def read_grid_toml(toml_path: Path) -> tuple[np.ndarray, dict]:
    """A height / undulation grid of data/terrain, data/geoid (header toml + int16 file, memory-mapped) -> (metres, header)."""
    g = tomllib.loads(toml_path.read_text(encoding="utf-8"))["grid"]
    a = np.memmap(toml_path.with_name(g["heights"]), dtype="<i2", mode="r", shape=(g["lines"], g["samples"]))
    return a, g


def _bilinear_points(array: np.ndarray, g: dict, lat: np.ndarray, lon: np.ndarray, wrap: bool = False) -> np.ndarray:
    """Bilinear metres of a grid file at the points (lat, lon) (same shape); outside the grid clamps to the edge."""
    fi = np.clip((g["north_latitude_deg"] - lat) / g["step_latitude_deg"], 0.0, g["lines"] - 1.0)
    fj = (lon - g["west_longitude_deg"]) / g["step_longitude_deg"]
    if wrap:
        fj = fj % g["samples"]
    fj = np.clip(fj, 0.0, g["samples"] - 1.0)
    i0 = np.minimum(fi.astype(np.int64), g["lines"] - 2)
    j0 = np.minimum(fj.astype(np.int64), g["samples"] - 2)
    wi, wj = fi - i0, fj - j0
    v = ((1 - wi) * ((1 - wj) * array[i0, j0] + wj * array[i0, j0 + 1])
         + wi * ((1 - wj) * array[i0 + 1, j0] + wj * array[i0 + 1, j0 + 1]))
    return v * float(g["dn_scale_m"])


class WaterMask:
    """The renderer's sea rule (orthometric height <= WATER_THRESHOLD_M, core EarthTerrain::water_threshold_m) at any point
    of an imagery area, from the height grids the game itself loads: the 1 m local grid where it covers the point, else the
    regional GLO-30 grid, minus the EGM2008 undulation. Used for the imagery tiles and the land-only Sentinel-2 embedding."""

    @staticmethod
    def available(repo_root: Path, area: str) -> bool:
        return ((repo_root / "data" / "terrain" / "earth" / f"{area}_regional.toml").exists()
                and (repo_root / "data" / "geoid" / "egm2008_25.toml").exists())

    def __init__(self, repo_root: Path, area: str):
        terrain = repo_root / "data" / "terrain" / "earth"
        self.regional, self.rg = read_grid_toml(terrain / f"{area}_regional.toml")
        local_toml = terrain / f"{area}_local.toml"
        self.local, self.lg = read_grid_toml(local_toml) if local_toml.exists() else (None, None)
        self.geoid, self.gg = read_grid_toml(repo_root / "data" / "geoid" / "egm2008_25.toml")

    def water(self, lat: np.ndarray, lon: np.ndarray) -> np.ndarray:
        """True where the point is sea (arrays of one shape)."""
        h = _bilinear_points(self.regional, self.rg, lat, lon)
        if self.local is not None:
            g = self.lg
            south = g["north_latitude_deg"] - (g["lines"] - 1) * g["step_latitude_deg"]
            east = g["west_longitude_deg"] + (g["samples"] - 1) * g["step_longitude_deg"]
            inside = (lat <= g["north_latitude_deg"]) & (lat >= south) & (lon >= g["west_longitude_deg"]) & (lon <= east)
            if inside.any():
                h = np.where(inside, _bilinear_points(self.local, g, lat, lon), h)
        n = _bilinear_points(self.geoid, self.gg, lat, lon, wrap=True)
        return (h - n) <= WATER_THRESHOLD_M


def land_fraction(repo_root: Path, h: int, w: int) -> np.ndarray | None:
    """Per pixel of an equirectangular h x w grid the fraction of ETOPO 2022 relief nodes (data/terrain/earth/
    global_relief, ellipsoidal) above the geoid (data/geoid/egm2008_25) by more than the core's water threshold - the
    renderer's sea rule. The 3 x 3 nodes around each pixel of a grid at half the node spacing; None without the data."""
    import tomllib as tl

    relief_toml = repo_root / "data" / "terrain" / "earth" / "global_relief.toml"
    geoid_toml = repo_root / "data" / "geoid" / "egm2008_25.toml"
    if not (relief_toml.exists() and geoid_toml.exists()):
        return None

    def grid(toml_path: Path) -> tuple[np.ndarray, dict]:
        g = tl.loads(toml_path.read_text(encoding="utf-8"))["grid"]
        a = np.fromfile(toml_path.with_name(g["heights"]), dtype="<i2").reshape(g["lines"], g["samples"])
        return a.astype(np.float32) * float(g["dn_scale_m"]), g

    relief, rg = grid(relief_toml)
    geoid, gg = grid(geoid_toml)
    lines, samples = relief.shape
    if lines != 2 * h + 1 or samples != 2 * w:
        return None
    lat = rg["north_latitude_deg"] - np.arange(lines) * rg["step_latitude_deg"]
    lon = rg["west_longitude_deg"] + np.arange(samples) * rg["step_longitude_deg"]
    gi = np.clip(np.rint((gg["north_latitude_deg"] - lat) / gg["step_latitude_deg"]).astype(int), 0, geoid.shape[0] - 1)
    gj = np.rint((lon - gg["west_longitude_deg"]) / gg["step_longitude_deg"]).astype(int) % geoid.shape[1]
    land = (relief - geoid[gi][:, gj] > WATER_THRESHOLD_M).astype(np.float32)
    land = np.concatenate([land, land[:, :1]], axis=1)  # wrap in longitude
    frac = np.zeros((h, w), dtype=np.float32)
    for dr in range(3):
        for dc in range(3):
            frac += land[dr:dr + 2 * h:2, dc:dc + 2 * w:2]
    return frac / 9.0


def sea_only(texture: np.ndarray, repo_root: Path) -> np.ndarray:
    """The day texture (reflectance) with every land pixel replaced by the nearest sea pixel (the sea level of the global
    texture continued under the coast): the 10 m tiles take their sea pixels from it, so the 7 km pixels of land do not
    bleed into the sea by bilinear sampling. Unchanged without the land mask."""
    from scipy.ndimage import distance_transform_edt

    frac = land_fraction(repo_root, *texture.shape[:2])
    if frac is None:
        return texture
    land = frac >= CLIMATOLOGY_MIN_LAND_FRACTION
    if land.all() or not land.any():
        return texture
    idx = distance_transform_edt(land, return_distances=False, return_indices=True)
    return texture[idx[0], idx[1]]


def build_day_texture(repo_root: Path = REPO_ROOT, embed: bool = True) -> np.ndarray:
    """The calibrated day texture as sRGB-encoded counts (h, w, 3): the MODIS climatology where it has samples
    (`CLIMATOLOGY_MIN_SAMPLES`), elsewhere (sea, sea ice, polar night, persistent cloud) the Blue Marble through the
    tables (a pixel takes the climatology only where ETOPO says it is mostly land); with `embed` the land-only Sentinel-2
    level inside it (`embed_sentinel2`, where its reflectance.tif exists). Without the climatology file only the tables
    (with a warning)."""
    from PIL import Image

    with Image.open(repo_root / "data" / "raw" / "earth_day_5400.jpg") as img:
        counts = np.asarray(img.convert("RGB"))
    rho = apply_lut(counts, load_luts(repo_root / "config" / "earth_albedo.toml"))
    clim = repo_root / CLIMATOLOGY_FILE.relative_to(REPO_ROOT)
    frac = land_fraction(repo_root, *rho.shape[:2])
    if frac is None:
        print("  earth_albedo: no ETOPO relief / geoid (fetch.py earth_terrain geoid_egm2008): no land mask")
    land_mask = None if frac is None else frac >= CLIMATOLOGY_MIN_LAND_FRACTION
    if clim.exists():
        with np.load(clim) as z:
            land, hits = z["rho"].astype(np.float64), z["hits"]
        if land.shape != rho.shape:
            raise ValueError(f"{clim}: grid {land.shape} differs from the day texture {rho.shape}")
        take = hits >= CLIMATOLOGY_MIN_SAMPLES
        if land_mask is not None:  # MODIS NBAR of water pixels in land tiles (cloud, glint) is no sea albedo
            take &= land_mask
        rho[take] = land[take]
    else:
        print(f"  earth_albedo: {clim} missing (fetch.py earth_albedo): the Blue Marble through the tables only")
    if embed:
        rho = embed_sentinel2(rho, repo_root, land=land_mask)
    return encode_srgb(rho)


def write_day_texture(repo_root: Path, out_path: Path) -> np.ndarray:
    from PIL import Image

    counts = build_day_texture(repo_root)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(counts, "RGB").save(out_path, "JPEG", quality=DAY_JPEG_QUALITY)
    return counts


def fetch(source: dict, repo_root: Path, download, force: bool = False) -> None:
    """fetch.py step: the MODIS climatology (network, ~10 min)."""
    target = repo_root / source["target"]
    if target.exists() and not force:
        print("  already present")
        return
    build_climatology(target)


def write_config(out: Path, luts: np.ndarray, stats: dict) -> None:
    lines = [
        "# Earth surface albedo: the Blue Marble Next Generation (Dec 2004) day texture's 8-bit counts -> surface",
        "# reflectance per channel. Generated by tools/data_pipeline/earth_albedo.py fit (do not edit by hand).",
        f"# [D] fitted to MODIS MCD43A4 v061 NBAR (bands 1 / 4 / 3) of {FIT_DATE}, {stats['tiles']} land tiles read at",
        f"# 1/8 resolution, averaged into {stats['pairs']} day-texture pixels (>= {FIT_MIN_SAMPLES} samples each); per",
        "# channel the median reflectance per count, isotonic (non-decreasing), interpolated between supported counts.",
        f"# Luminance error per pixel: median {stats['median_rel_luma_error']:.3f}, 90th percentile "
        f"{stats['p90_rel_luma_error']:.3f}; mean luminance reference {stats['mean_luma_ref']:.4f}, fit "
        f"{stats['mean_luma_fit']:.4f}.",
        "",
    ]
    for c, name in enumerate(CHANNELS):
        values = ", ".join(f"{v:.5f}" for v in luts[c])
        lines.append(f"lut_{name} = [{values}]")
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    if len(sys.argv) > 1 and sys.argv[1] == "fit":
        fit(REPO_ROOT / "data" / "raw" / "earth_day_5400.jpg")
        return 0
    if len(sys.argv) > 1 and sys.argv[1] == "climatology":
        build_climatology()
        return 0
    print(__doc__)
    return 1


if __name__ == "__main__":
    sys.exit(main())
