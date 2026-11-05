"""Offline bake of tileable procedural regolith and rock textures for the Moon's near-ground surface.

Usage: python tools/moon_textures/bake_moon_detail.py [--out data/textures/moon_detail]
(also: python tools/data_pipeline/fetch.py moon_detail). Procedural only, no downloaded imagery, deterministic:
numpy Generator with the fixed seeds below (the integer-hash rule applies to the runtime core, not to this offline bake)
and Pillow with a fixed PNG compression level give byte-identical files on the same machine and library versions.

Band split (no double counting, controller ruling 2026-11-05)
-------------------------------------------------------------
The core's relief (craters + regolith roughness, `environment::LunarSurfaceDetail`) is already in the terrain mesh
down to the node vertex spacing. The regolith textures carry ONLY the wavelengths below that spacing, from the same
self-affine spectrum: octave variance V(lambda) = a^2 (lambda / baseline_m)^(2 H) per octave (H = roughness.hurst_exponent,
baseline_m = 0.125 m, both from config/moon_surface.toml), i.e. the law the core's roughness uses. The amplitude
constant a is calibrated here with Guo's statistic exactly as the core does: the median plane-detrended standard
deviation of 25 x 25 sample windows (spacing 5 mm, 125 mm side) of a full-spectrum field equals
roughness.rms_height_at_baseline_m (3.7 mm). The T10 shader blends each scale in only where the mesh no longer
resolves it (the mesh resolves wavelengths >= 2 x its vertex spacing: 2 cm at the finest level 22, 1.02 cm spacing).

Scales (1024 x 1024 px each, the bands are disjoint and contiguous, 3 octaves each, tile / lambda_hi = 16 - 2 x 2 of
the former 512 px tiles since the Task 11 fix round, so each map holds four tiles' worth of craters and pebbles):
    s0: tile 0.64 m,   pixel 0.625 mm, wavelengths 5 mm .. 40 mm    (flat 5.95 .. 36.7 mm, content to 43.6 mm)
    s1: tile 5.12 m,   pixel 5 mm,     wavelengths 40 mm .. 320 mm  (flat 43.6 .. 293 mm, content 36.7 .. 349 mm)
    s2: tile 40.96 m,  pixel 40 mm,    wavelengths 320 mm .. 2.56 m (flat 349 mm .. 2.35 m, content 293 mm .. 2.79 m)
    s3: tile 327.68 m, pixel 320 mm,   wavelengths 2.56 .. 20.48 m  (flat 2.79 .. 17.2 m, content from 2.35 m)
(wavelength_min_m / wavelength_max_m in moon_detail.toml). The outer edges (5 mm, 20.48 m) carry a raised-cosine taper
of a quarter octave inside the band. The shared edges are complementary: cos / sin amplitude ramps straddling the edge
over a quarter octave (an eighth on each side), so the two neighbours' spectral variance sums to exactly the law at
every wavelength (the Task 10 review found the earlier inside tapers on both neighbours lost ~0.9 mm^2 of Guo's
statistic).

Craters (Task 10 fix round): every scale also carries the small craters of the core's law (config [craters]: N(>D) =
density (D / ref)^-b, fresh / degraded depth per D, rim, degraded share) whose diameter lies in its band (D from 1 cm,
each crater in exactly one scale): Poisson count of the law's expectation on the periodic tile, uniform positions, D by
the law's inverse cumulative, the core's crater_profile_m (bowl to R, (r / R)^-3 rim flank to 3 R), zero mean. They are
kept as a separate component, so the shader weights each by the share the mesh does not draw (regolith: the band's
spectral transfer; craters: the core's crater_drawn_fraction) - one texture fetch for both.

Pebbles (Task 11 fix round, review I3): the rocks below MoonRockLayer.PEBBLE_GEOMETRY_LIMIT_M (2 cm, from the law's
1 cm) are never instances; they sit in the crater component of the scale whose band holds them (s0) as discrete
half-spheroid bumps (plan radius D / 2, height PEBBLE_ETA x D: the statistical rock layer's shape), count and sizes
from config [rocks.sfd] at abundance 1 (stratified quantiles, uniform positions). Their radiance mean stays in the
statistical layer: the texture relief only modulates (mean-preserving), and the rocks' slopes are outside the
regolith's theta_bar budget (unresolved_shading), so nothing is counted twice. MoonDetailShading weights the s0 crater
component by variance: sqrt((w_c^2 V_craters + V_pebbles) / (V_craters + V_pebbles)) (pebbles never mesh-drawn).

Files per scale: regolith_height_<s>.png and crater_height_<s>.png (16-bit greyscale, value = 32768 + h /
height_scale_m * 32767, scales in moon_detail.toml; the tests integrate them), regolith_slope_<s>.rgbah (the shader's
map: raw little-endian half floats, regolith dh/du, dh/dv, craters dh/du, dh/dv; u along the image row = +x, v down the
image = +row; periodic central differences of the float heights, loaded without a per-pixel decode),
regolith_normal_<s>.png (8-bit RGB tangent space of the sum, OpenGL convention: red = +x right, green = +y up = towards
the image top, blue = +z; for previews). Test for T10: mesh + textures at the 125 mm baseline reproduce Guo's 4 mm
+- 10 % (no double roughness across the handoff).

Rock textures (triplanar set, 4 variants, 512 x 512 px, tile 1.0 m, 1.95 mm pixel): relative albedo (8-bit grey, value
128 = 1.0, mean exactly 1.0 within quantisation: the shader multiplies by rocks.albedo_to_regolith, so the WAC mean is
never shifted) and tangent-space normals. Variants: 0 fine grain, 1 vesicular (pits), 2 breccia (clast patches),
3 fractured (crack grooves). All amplitudes of the rock textures are [E] (appearance only; the config holds no
rock-surface texture data): spectrum H 0.8 with 0.5 x the regolith amplitude constant a (ROCK_AMPLITUDE_FACTOR, [E]), relative albedo sd 0.10.
"""

from __future__ import annotations

import argparse
import sys
import tomllib
from pathlib import Path

import numpy as np
from PIL import Image

REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_FILE = REPO_ROOT / "config" / "moon_surface.toml"
DEFAULT_OUT = REPO_ROOT / "data" / "textures" / "moon_detail"

SIZE_PX = 512                       # [E] the rock maps; mipmapped by Godot
REGOLITH_SIZE_PX = 1024             # [E] the regolith / crater maps: 2 x 2 of the former 512 px tiles per scale (Task 11
#   fix B, Task 10 re-review N3): four times the crater (and pebble) population per realisation, so the summed-roughness
#   margin no longer hangs on the few top-octave craters of one small tile
PEBBLE_ETA = 0.444                  # [D] exposed height / D of the rocks: the core's measured_eta of the resting angular templates (fix C)
#   (Task 11 fix A report; unresolved_shading's rock_exposed_height_per_diameter, printed by the [unresolved_shading]
#   test) - the statistical layer's half-spheroid shape, which the pebble bumps take
PEBBLE_MAX_DIAMETER_M = 0.02        # MoonRockLayer.PEBBLE_GEOMETRY_LIMIT_M: rocks below it are never instances
PEBBLE_SEED_OFFSET = 70             # pebble draws of scale <seed> use seed + PEBBLE_SEED_OFFSET
TAPER_OCTAVES = 0.25                # [E] raised-cosine taper inside the outer band edges; straddle width of the shared ones
GUO_WINDOW_SAMPLES = 25             # [S] Guo et al. 2021: 25 x 25 samples (5 mm DTM, 125 mm window)
GUO_PIXEL_M = 0.005                 # [S] Guo et al. 2021 DTM pixel
CALIBRATION_SIZE_PX = 4096          # field for the amplitude calibration
CALIBRATION_PIXEL_M = 0.0025        # 10.24 m field, finest wavelength 5 mm
CALIBRATION_WINDOWS = 1536          # [S] as the core's roughness normalisation
CALIBRATION_SEED = 20261105
PNG_COMPRESS_LEVEL = 9
ROCK_TILE_M = 1.0                   # [E]
ROCK_HURST = 0.8                    # [E] appearance only
ROCK_ALBEDO_SD = 0.10               # [E] relative albedo variation of rock surfaces
ROCK_AMPLITUDE_FACTOR = 0.5         # [E] rock-surface height amplitude / regolith octave amplitude a: bare rock has no loose
#   grains, so its mm-scale relief is taken as half the regolith's (rms slope 0.21 instead of 0.43, which looked
#   over-rough in the normal maps); no lunar rock-surface roughness data in the config
ROCK_LAMBDA_MIN_PX = 4.0            # [E] finest wavelength of the rock height field in pixels

# name, tile edge [m], lambda_lo [m], lambda_hi [m], seed, lower edge shared, upper edge shared
REGOLITH_SCALES = (
    ("s0", 0.64, 0.005, 0.040, 1101, False, True),
    ("s1", 5.12, 0.040, 0.320, 1102, True, True),
    ("s2", 40.96, 0.320, 2.560, 1103, True, True),
    ("s3", 327.68, 2.560, 20.48, 1104, True, False),
)
CRATER_SEED_OFFSET = 50             # crater draws of scale <seed> use seed + CRATER_SEED_OFFSET
CRATER_REACH_RADII = 3.0            # crater_profile_m is zero from 3 R (core lunar_surface_detail.cpp)
ROCK_SEEDS = (2201, 2202, 2203, 2204)


def load_roughness(path: Path = CONFIG_FILE) -> dict:
    with path.open("rb") as f:
        return tomllib.load(f)["roughness"]


def load_craters(path: Path = CONFIG_FILE) -> dict:
    with path.open("rb") as f:
        return tomllib.load(f)["craters"]


def load_rocks(path: Path = CONFIG_FILE) -> dict:
    with path.open("rb") as f:
        return tomllib.load(f)["rocks"]


def pebble_population(rocks: dict, tile_m: float, d_lo: float, d_hi: float, seed: int) -> dict:
    """Pebbles of the config rock law (rocks.sfd, N(>D) = density (D / ref)^-b, abundance 1: the mean population) with D in
    [d_lo, d_hi) on a periodic tile: the expected count rounded, uniform positions, D at stratified quantiles of the
    law's inverse cumulative (as crater_population)."""
    sfd = rocks["sfd"]
    if sfd["form"] != "power":
        raise ValueError("bake_moon_detail: rocks.sfd must be a power law")
    d_lo, d_hi = max(d_lo, sfd["min_diameter_m"]), min(d_hi, sfd["max_diameter_m"])
    rng = np.random.default_rng(seed + PEBBLE_SEED_OFFSET)
    if d_hi <= d_lo:
        return {"x": np.zeros(0), "y": np.zeros(0), "d": np.zeros(0), "expected": 0.0, "d_lo": 0.0, "d_hi": 0.0}
    b = sfd["exponent"]
    cumulative = lambda d: sfd["density_per_m2"] * (d / sfd["reference_diameter_m"]) ** -b  # noqa: E731
    expected = (cumulative(d_lo) - cumulative(d_hi)) * tile_m**2
    count = int(round(expected))
    x, y = rng.uniform(0.0, tile_m, count), rng.uniform(0.0, tile_m, count)
    u = (np.arange(count) + rng.uniform(0.0, 1.0, count)) / max(count, 1)
    d = d_lo * (1.0 - u * (1.0 - (d_hi / d_lo) ** -b)) ** (-1.0 / b)
    return {"x": x, "y": y, "d": d, "expected": expected, "d_lo": d_lo, "d_hi": d_hi}


def pebble_field(population: dict, size_px: int, pixel_m: float) -> np.ndarray:
    """Periodic height field of the pebbles: half-spheroid bumps (plan radius D / 2, height PEBBLE_ETA x D - the
    statistical rock layer's shape) at the pixel centres, overlaps by the maximum (one surface), zero mean."""
    field = np.zeros((size_px, size_px))
    for x, y, d in zip(population["x"], population["y"], population["d"]):
        radius = 0.5 * d
        reach = int(np.ceil(radius / pixel_m)) + 1
        cx, cy = x / pixel_m - 0.5, y / pixel_m - 0.5
        cols = np.arange(int(np.floor(cx)) - reach, int(np.floor(cx)) + reach + 2)
        rows = np.arange(int(np.floor(cy)) - reach, int(np.floor(cy)) + reach + 2)
        r = np.hypot((cols[None, :] - cx) * pixel_m, (rows[:, None] - cy) * pixel_m)
        bump = PEBBLE_ETA * d * np.sqrt(np.clip(1.0 - (r / radius) ** 2, 0.0, 1.0))
        index = np.ix_(rows % size_px, cols % size_px)
        field[index] = np.maximum(field[index], bump)
    return field - field.mean()


def crater_cumulative(craters: dict, diameter_m: float) -> float:
    """N(>D) per m^2 of the config law (power form, as SizeFrequency::cumulative_density_per_m2)."""
    sfd = craters["sfd"]
    if sfd["form"] != "power":
        raise ValueError("bake_moon_detail: craters.sfd must be a power law")
    return sfd["density_per_m2"] * (diameter_m / sfd["reference_diameter_m"]) ** -sfd["exponent"]


def crater_band_range(craters: dict, lam_lo: float, lam_hi: float) -> tuple[float, float]:
    """Diameters a scale carries: the band [lam_lo, lam_hi) clipped to the law's [min, max] (each crater in one band)."""
    sfd = craters["sfd"]
    return max(lam_lo, sfd["min_diameter_m"]), min(lam_hi, sfd["max_diameter_m"])


def crater_profile(depth: np.ndarray, rim: np.ndarray, radius: np.ndarray, r: np.ndarray) -> np.ndarray:
    """crater_profile_m of the core: parabolic bowl to R, rim (r / R)^-3 law to 3 R (heights in m)."""
    rho = r / radius
    bowl = rim - depth + depth * rho**2
    flank = rim * (1.0 / np.maximum(rho, 1.0) ** 3 - 1.0 / 27.0) * 27.0 / 26.0
    return np.where(rho < 1.0, bowl, np.where(rho < CRATER_REACH_RADII, flank, 0.0))


def crater_population(craters: dict, tile_m: float, lam_lo: float, lam_hi: float, seed: int) -> dict:
    """Craters of the config law with D in the scale's band on a periodic tile: the expected count
    (N(>D_lo) - N(>D_hi)) x tile^2 rounded, uniform positions, D by the law's inverse cumulative at stratified quantiles
    ((k + jitter) / count), exactly the configured fresh / degraded share in random order (depth per D fresh /
    degraded, rim = rim_height_to_diameter / depth_fresh x depth). Stratified, because a tile holds only ~6 craters in
    its top octave, which carry most of the height variance: independent draws left the first bake's tiles at 0.59-1.24
    x the law's crater variance."""
    d_lo, d_hi = crater_band_range(craters, lam_lo, lam_hi)
    rng = np.random.default_rng(seed + CRATER_SEED_OFFSET)
    if d_hi <= d_lo:
        return {"x": np.zeros(0), "y": np.zeros(0), "d": np.zeros(0), "depth": np.zeros(0), "rim": np.zeros(0),
                "expected": 0.0}
    n_lo, n_hi = crater_cumulative(craters, d_lo), crater_cumulative(craters, d_hi)
    expected = (n_lo - n_hi) * tile_m**2
    count = int(round(expected))
    x, y = rng.uniform(0.0, tile_m, count), rng.uniform(0.0, tile_m, count)
    b = craters["sfd"]["exponent"]
    u = (np.arange(count) + rng.uniform(0.0, 1.0, count)) / max(count, 1)
    d = d_lo * (1.0 - u * (1.0 - (d_hi / d_lo) ** -b)) ** (-1.0 / b)      # inverse of N(>D) inside [d_lo, d_hi)
    fresh = np.zeros(count, dtype=bool)
    fresh[:int(round(count * (1.0 - craters["degraded_fraction"])))] = True
    rng.shuffle(fresh)
    depth = d * np.where(fresh, craters["depth_to_diameter_fresh"], craters["depth_to_diameter_degraded"])
    rim = craters["rim_height_to_diameter"] / craters["depth_to_diameter_fresh"] * depth
    return {"x": x, "y": y, "d": d, "depth": depth, "rim": rim, "expected": expected}


def crater_field(population: dict, size_px: int, pixel_m: float) -> np.ndarray:
    """Periodic height field of the craters at the pixel centres, zero mean (LOLA heights are area means)."""
    field = np.zeros((size_px, size_px))
    for x, y, d, depth, rim in zip(population["x"], population["y"], population["d"], population["depth"],
                                   population["rim"]):
        radius = 0.5 * d
        reach = int(np.ceil(CRATER_REACH_RADII * radius / pixel_m)) + 1
        cx, cy = x / pixel_m - 0.5, y / pixel_m - 0.5                   # pixel centres at (i + 0.5) pixel
        cols = np.arange(int(np.floor(cx)) - reach, int(np.floor(cx)) + reach + 2)
        rows = np.arange(int(np.floor(cy)) - reach, int(np.floor(cy)) + reach + 2)
        r = np.hypot((cols[None, :] - cx) * pixel_m, (rows[:, None] - cy) * pixel_m)
        field[np.ix_(rows % size_px, cols % size_px)] += crater_profile(depth, rim, radius, r)
    return field - field.mean()


def octave_psd_constant(amplitude_m: float, hurst: float, baseline_m: float) -> float:
    """C of the spectral density S(k) = C k^-(2H+2) [m^2 per (cycle/m)^2] whose octave variance (k .. 2k) is
    a^2 (1 / (k baseline))^(2H): the self-affine roughness law of the core (octave labelled by its longest wavelength)."""
    return amplitude_m**2 * baseline_m ** (-2.0 * hurst) * 2.0 * hurst / (2.0 * np.pi * (1.0 - 2.0 ** (-2.0 * hurst)))


def band_weight(wavelength_m: np.ndarray, lam_lo: float, lam_hi: float, shared_lo: bool = False,
                shared_hi: bool = False) -> np.ndarray:
    """Amplitude weight: 1 inside the band. An outer edge: raised-cosine to 0 over TAPER_OCTAVES inside the band. A
    shared edge: sin (upper band) / cos (lower band) of a quarter period over TAPER_OCTAVES centred on the edge, so the
    squared weights of the two neighbours sum to 1."""
    with np.errstate(divide="ignore"):
        x = np.log2(wavelength_m)
    lo, hi = np.log2(lam_lo), np.log2(lam_hi)
    half = 0.5 * TAPER_OCTAVES
    if shared_lo:   # rises from lo - half (0) to lo + half (1)
        w_lo = np.sin(0.5 * np.pi * np.clip((x - lo + half) / TAPER_OCTAVES, 0.0, 1.0))
    else:
        w_lo = 0.5 - 0.5 * np.cos(np.pi * np.clip((x - lo) / TAPER_OCTAVES, 0.0, 1.0))
    if shared_hi:   # falls from hi - half (1) to hi + half (0)
        w_hi = np.cos(0.5 * np.pi * np.clip((x - hi + half) / TAPER_OCTAVES, 0.0, 1.0))
    else:
        w_hi = 0.5 - 0.5 * np.cos(np.pi * np.clip((hi - x) / TAPER_OCTAVES, 0.0, 1.0))
    w = w_lo * w_hi
    w[~np.isfinite(x)] = 0.0
    return w


def synthesise(size_px: int, pixel_m: float, lam_lo: float, lam_hi: float, amplitude_m: float, hurst: float,
               baseline_m: float, seed: int, shared_lo: bool = False, shared_hi: bool = False) -> np.ndarray:
    """Periodic Gaussian field with the self-affine spectrum inside [lam_lo, lam_hi] (heights in metres)."""
    length = size_px * pixel_m
    fy = np.fft.fftfreq(size_px, d=pixel_m)[:, None]            # cycles / m
    fx = np.fft.rfftfreq(size_px, d=pixel_m)[None, :]
    k = np.hypot(fx, fy)
    k[0, 0] = 1.0
    c = octave_psd_constant(amplitude_m, hurst, baseline_m)
    density = c * k ** (-(2.0 * hurst + 2.0)) * band_weight(1.0 / k, lam_lo, lam_hi, shared_lo, shared_hi) ** 2
    density[0, 0] = 0.0
    dk = 1.0 / length
    gain = size_px * np.sqrt(density) * dk                       # |G|^2 = N^2 S dk^2 (variance per mode)
    white = np.random.default_rng(seed).standard_normal((size_px, size_px))
    return np.fft.irfft2(np.fft.rfft2(white) * gain, s=(size_px, size_px))


def window_rms(field: np.ndarray, stride: int, windows: int, seed: int, samples: int = GUO_WINDOW_SAMPLES) -> np.ndarray:
    """Plane-detrended standard deviation of `windows` windows of samples x samples points at `stride` pixels apart."""
    n = field.shape[0]
    span = (samples - 1) * stride
    rng = np.random.default_rng(seed)
    y0 = rng.integers(0, n - span, windows)
    x0 = rng.integers(0, n - span, windows)
    idx = np.arange(samples) * stride
    h = field[(y0[:, None, None] + idx[None, :, None]), (x0[:, None, None] + idx[None, None, :])]
    c = np.arange(samples) - (samples - 1) / 2.0
    h = h - h.mean(axis=(1, 2), keepdims=True)
    ax = (h * c[None, None, :]).sum(axis=(1, 2)) / (samples * (c**2).sum())
    ay = (h * c[None, :, None]).sum(axis=(1, 2)) / (samples * (c**2).sum())
    res = h - ax[:, None, None] * c[None, None, :] - ay[:, None, None] * c[None, :, None]
    return np.sqrt((res**2).mean(axis=(1, 2)))


def guo_statistic(field: np.ndarray, stride: int, windows: int, seed: int) -> float:
    """Median plane-detrended window std (Guo's statistic); the window side is 25 x stride x pixel."""
    return float(np.median(window_rms(field, stride, windows, seed)))


def calibrated_amplitude(roughness: dict) -> float:
    """Octave amplitude constant a such that the full-spectrum field has Guo's statistic = rms_height_at_baseline_m."""
    target = roughness["rms_height_at_baseline_m"]
    hurst, baseline = roughness["hurst_exponent"], roughness["baseline_m"]
    trial = target
    length = CALIBRATION_SIZE_PX * CALIBRATION_PIXEL_M
    field = synthesise(CALIBRATION_SIZE_PX, CALIBRATION_PIXEL_M, 2.0 * CALIBRATION_PIXEL_M, length, trial, hurst,
                       baseline, CALIBRATION_SEED)
    sub = field[::2, ::2]                                        # 5 mm samples as Guo's DTM
    measured = guo_statistic(sub, 1, CALIBRATION_WINDOWS, CALIBRATION_SEED + 1)   # 25 x 5 mm = 125 mm
    return trial * target / measured                             # the statistic is linear in the amplitude


def slopes_from_height(height_m: np.ndarray, pixel_m: float) -> tuple[np.ndarray, np.ndarray]:
    """(dh/dx along a row, dh/drow down the image): periodic central differences (zero mean exactly)."""
    dx = (np.roll(height_m, -1, axis=1) - np.roll(height_m, 1, axis=1)) / (2.0 * pixel_m)
    drow = (np.roll(height_m, -1, axis=0) - np.roll(height_m, 1, axis=0)) / (2.0 * pixel_m)
    return dx, drow


def save_slopes(regolith_m: np.ndarray, craters_m: np.ndarray, pixel_m: float, path: Path) -> None:
    """Raw little-endian half floats (regolith dh/dx, dh/drow, craters dh/dx, dh/drow), row-major (Godot
    Image.FORMAT_RGBAH): the shader weights the two components separately."""
    rx, rrow = slopes_from_height(regolith_m, pixel_m)
    cx, crow = slopes_from_height(craters_m, pixel_m)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(np.stack([rx, rrow, cx, crow], axis=-1).astype("<f2").tobytes())


def normals_from_height(height_m: np.ndarray, pixel_m: float) -> np.ndarray:
    """Tangent-space normals (OpenGL: +y up = image top) as uint8 RGB, periodic central differences."""
    dx, drow = slopes_from_height(height_m, pixel_m)
    n = np.stack([-dx, drow, np.ones_like(dx)], axis=-1)         # +y up = decreasing row: n_y = -dh/dy = +dh/drow
    n /= np.linalg.norm(n, axis=-1, keepdims=True)
    return np.clip(np.rint((n * 0.5 + 0.5) * 255.0), 0, 255).astype(np.uint8)


def encode_height(height_m: np.ndarray) -> tuple[np.ndarray, float]:
    scale = float(np.abs(height_m).max())
    return np.clip(np.rint(32768.0 + height_m / scale * 32767.0), 0, 65535).astype(np.uint16), scale


def save_png(array: np.ndarray, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(array).save(path, format="PNG", compress_level=PNG_COMPRESS_LEVEL, optimize=False)


def wrapped_blob(shape: int, cy: float, cx: float, radius: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Row / column index arrays (wrapped) and normalised squared distance for a blob of `radius` px at (cy, cx)."""
    r = int(np.ceil(radius)) + 1
    rows = (np.arange(int(cy) - r, int(cy) + r + 1)) % shape
    cols = (np.arange(int(cx) - r, int(cx) + r + 1)) % shape
    yy = np.arange(int(cy) - r, int(cy) + r + 1)[:, None] - cy
    xx = np.arange(int(cx) - r, int(cx) + r + 1)[None, :] - cx
    return rows, cols, (yy**2 + xx**2) / radius**2


def rock_variant(variant: int, amplitude_m: float, baseline_m: float) -> tuple[np.ndarray, np.ndarray]:
    """(height in metres, relative albedo, mean 1) of rock variant 0..3 (all [E], appearance only)."""
    size, pixel = SIZE_PX, ROCK_TILE_M / SIZE_PX
    seed = ROCK_SEEDS[variant]
    rng = np.random.default_rng(seed + 100)
    height = synthesise(size, pixel, ROCK_LAMBDA_MIN_PX * pixel, ROCK_TILE_M, ROCK_AMPLITUDE_FACTOR * amplitude_m,
                         ROCK_HURST, baseline_m, seed)
    albedo_noise = synthesise(size, pixel, 4.0 * pixel, ROCK_TILE_M / 2.0, 1.0, 0.6, baseline_m, seed + 50)
    albedo_noise /= albedo_noise.std()
    albedo = ROCK_ALBEDO_SD * albedo_noise
    if variant == 1:                                              # vesicular: parabolic pits, darker inside
        for _ in range(450):
            cy, cx = rng.uniform(0, size, 2)
            radius = rng.uniform(2.0, 9.0)
            depth = 0.35 * radius * pixel * rng.uniform(0.6, 1.0)
            rows, cols, q = wrapped_blob(size, cy, cx, radius)
            bowl = np.where(q < 1.0, 1.0 - q, 0.0)
            height[np.ix_(rows, cols)] -= depth * bowl
            albedo[np.ix_(rows, cols)] -= 0.15 * bowl
    elif variant == 2:                                            # breccia: clast patches with height and albedo steps
        patch = synthesise(size, pixel, 0.05, 0.4, 1.0, 0.3, baseline_m, seed + 70)
        clasts = patch > np.quantile(patch, 0.65)
        height += clasts * 0.0015
        albedo += np.where(clasts, 0.12, -0.05)
    elif variant == 3:                                            # fractured: grooves along the zero lines of two fields
        for i in range(2):
            line = synthesise(size, pixel, 0.06, 0.5, 1.0, 0.3, baseline_m, seed + 80 + i)
            width = 0.12 * line.std()
            groove = np.exp(-((line / width) ** 2))
            height -= 0.002 * groove
            albedo -= 0.10 * groove
    albedo = albedo - albedo.mean()
    return height, albedo


def encode_albedo(relative: np.ndarray) -> np.ndarray:
    """Relative albedo (1 + variation) -> 8-bit, 128 = 1.0; the variation is shifted so the decoded mean is 1.0 (+- 0.2 %)."""
    code = np.clip(np.rint(128.0 * (1.0 + relative)), 0, 255)
    for _ in range(4):                                            # exact-mean correction after quantisation
        shift = 128.0 - code.mean()
        relative = relative + shift / 128.0
        code = np.clip(np.rint(128.0 * (1.0 + relative)), 0, 255)
    return code.astype(np.uint8)


def bake(out_dir: Path, config_file: Path = CONFIG_FILE) -> dict:
    roughness = load_roughness(config_file)
    hurst, baseline = roughness["hurst_exponent"], roughness["baseline_m"]
    amplitude = calibrated_amplitude(roughness)
    meta = {"hurst_exponent": hurst, "baseline_m": baseline, "rms_height_at_baseline_m": roughness["rms_height_at_baseline_m"],
            "octave_amplitude_m": amplitude, "scales": []}
    craters = load_craters(config_file)
    rocks_config = load_rocks(config_file)
    for name, tile, lam_lo, lam_hi, seed, shared_lo, shared_hi in REGOLITH_SCALES:
        pixel = tile / REGOLITH_SIZE_PX
        height = synthesise(REGOLITH_SIZE_PX, pixel, lam_lo, lam_hi, amplitude, hurst, baseline, seed, shared_lo,
                            shared_hi)
        population = crater_population(craters, tile, lam_lo, lam_hi, seed)
        crater_only = crater_field(population, REGOLITH_SIZE_PX, pixel)
        # Pebbles (rocks below PEBBLE_MAX_DIAMETER_M, never instances) as discrete bumps in the scale whose band holds
        # them, in the crater component (the shader's weights combine the two by variance: MoonDetailShading).
        pebbles = pebble_population(rocks_config, tile, lam_lo, min(lam_hi, PEBBLE_MAX_DIAMETER_M), seed)
        pebble_height = pebble_field(pebbles, REGOLITH_SIZE_PX, pixel)
        crater_height = crater_only + pebble_height
        code, scale = encode_height(height)
        save_png(code, out_dir / f"regolith_height_{name}.png")
        crater_code, crater_scale = encode_height(crater_height)
        save_png(crater_code, out_dir / f"crater_height_{name}.png")
        save_png(normals_from_height(height + crater_height, pixel), out_dir / f"regolith_normal_{name}.png")
        save_slopes(height, crater_height, pixel, out_dir / f"regolith_slope_{name}.rgbah")
        d_lo, d_hi = crater_band_range(craters, lam_lo, lam_hi)
        meta["scales"].append({"name": name, "tile_m": tile, "pixel_m": pixel, "wavelength_min_m": lam_lo,
                               "wavelength_max_m": lam_hi, "height_scale_m": scale, "rms_m": float(height.std()),
                               "crater_min_diameter_m": d_lo, "crater_max_diameter_m": d_hi,
                               "crater_count": int(population["d"].size),
                               "crater_expected_count": float(population["expected"]),
                               "crater_height_scale_m": crater_scale, "crater_rms_m": float(crater_height.std()),
                               "crater_only_rms_m": float(crater_only.std()),
                               "pebble_min_diameter_m": pebbles["d_lo"], "pebble_max_diameter_m": pebbles["d_hi"],
                               "pebble_count": int(pebbles["d"].size),
                               "pebble_expected_count": float(pebbles["expected"]),
                               "pebble_rms_m": float(pebble_height.std())})
    rocks = []
    for variant in range(4):
        height, albedo = rock_variant(variant, amplitude, baseline)
        save_png(encode_albedo(albedo), out_dir / f"rock_albedo_{variant}.png")
        save_png(normals_from_height(height, ROCK_TILE_M / SIZE_PX), out_dir / f"rock_normal_{variant}.png")
        rocks.append({"variant": variant, "tile_m": ROCK_TILE_M, "rms_height_m": float(height.std())})
    meta["rocks"] = rocks
    write_meta(meta, out_dir / "moon_detail.toml")
    return meta


def write_meta(meta: dict, path: Path) -> None:
    lines = ["# Generated by tools/moon_textures/bake_moon_detail.py - do not edit.",
             f"hurst_exponent = {meta['hurst_exponent']!r}", f"baseline_m = {meta['baseline_m']!r}",
             f"rms_height_at_baseline_m = {meta['rms_height_at_baseline_m']!r}",
             f"octave_amplitude_m = {meta['octave_amplitude_m']!r}",
             "height_encoding = \"value = 32768 + h / height_scale_m * 32767\"",
             "rock_albedo_encoding = \"relative albedo = value / 128 (mean 1.0)\"", ""]
    for s in meta["scales"]:
        lines += ["[[regolith]]"] + [f"{key} = {value!r}" if not isinstance(value, str) else f'{key} = "{value}"'
                                     for key, value in s.items()] + [""]
    for r in meta["rocks"]:
        lines += ["[[rock]]"] + [f"{key} = {value!r}" for key, value in r.items()] + [""]
    path.write_text("\n".join(lines), encoding="utf-8", newline="\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    meta = bake(args.out)
    print(f"octave amplitude constant a = {meta['octave_amplitude_m'] * 1000:.3f} mm (calibrated to "
          f"{meta['rms_height_at_baseline_m'] * 1000:.2f} mm at 125 mm)")
    total = 0
    for path in sorted(args.out.glob("*")):
        total += path.stat().st_size
        print(f"  {path.name} {path.stat().st_size / 1024:.0f} kB")
    print(f"total {total / 1048576:.2f} MB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
