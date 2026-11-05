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

Scales (512 x 512 px each, the bands are disjoint and contiguous, 3 octaves each, tile / lambda_hi = 8):
    s0: tile 0.32 m,  pixel 0.625 mm, wavelengths 5 mm .. 40 mm
    s1: tile 2.56 m,  pixel 5 mm,     wavelengths 40 mm .. 320 mm
    s2: tile 20.48 m, pixel 40 mm,    wavelengths 320 mm .. 2.56 m
The band edges carry a raised-cosine taper of a quarter octave inside the band (the flat part is where the spectrum
is tested). Heights: 16-bit greyscale PNG, value = 32768 + h / height_scale_m * 32767 (scale per map in
moon_detail.toml); normals: 8-bit RGB tangent space, OpenGL convention (red = +x right, green = +y up = towards the
image top, blue = +z), slopes from the metric pixel size, periodic differences (seamless).

T10 handoff: exact band edges per scale (wavelength, flat part inside the quarter-octave taper), for the fade by node
vertex spacing d (the mesh resolves lambda >= 2 d, so scale i carries weight 0 once 2 d <= its lambda_min and 1 once
2 d >= its lambda_max; blend log-linearly between):
    s0: lambda_min 5 mm,   lambda_max 40 mm   (flat 5.95 .. 33.6 mm)  -> weight 0 for d <= 2.5 mm, 1 for d >= 20 mm
    s1: lambda_min 40 mm,  lambda_max 320 mm  (flat 47.6 .. 269 mm)   -> weight 0 for d <= 20 mm,  1 for d >= 160 mm
    s2: lambda_min 320 mm, lambda_max 2.56 m  (flat 381 mm .. 2.15 m) -> weight 0 for d <= 160 mm, 1 for d >= 1.28 m
(the same values are in moon_detail.toml as wavelength_min_m / wavelength_max_m). Test for T10: mesh + textures at the
125 mm baseline reproduce 4 mm +- 10 % (no double roughness across the handoff).

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

SIZE_PX = 512                       # [E] every map; mipmapped by Godot
TAPER_OCTAVES = 0.25                # [E] raised-cosine taper inside the band edges
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

# name, tile edge [m], lambda_lo [m], lambda_hi [m], seed
REGOLITH_SCALES = (
    ("s0", 0.32, 0.005, 0.040, 1101),
    ("s1", 2.56, 0.040, 0.320, 1102),
    ("s2", 20.48, 0.320, 2.560, 1103),
)
ROCK_SEEDS = (2201, 2202, 2203, 2204)


def load_roughness(path: Path = CONFIG_FILE) -> dict:
    with path.open("rb") as f:
        return tomllib.load(f)["roughness"]


def octave_psd_constant(amplitude_m: float, hurst: float, baseline_m: float) -> float:
    """C of the spectral density S(k) = C k^-(2H+2) [m^2 per (cycle/m)^2] whose octave variance (k .. 2k) is
    a^2 (1 / (k baseline))^(2H): the self-affine roughness law of the core (octave labelled by its longest wavelength)."""
    return amplitude_m**2 * baseline_m ** (-2.0 * hurst) * 2.0 * hurst / (2.0 * np.pi * (1.0 - 2.0 ** (-2.0 * hurst)))


def band_weight(wavelength_m: np.ndarray, lam_lo: float, lam_hi: float) -> np.ndarray:
    """1 inside the band, raised-cosine to 0 over TAPER_OCTAVES inside both edges, 0 outside."""
    with np.errstate(divide="ignore"):
        x = np.log2(wavelength_m)
    lo, hi = np.log2(lam_lo), np.log2(lam_hi)
    w = np.zeros_like(x)
    inside = (x >= lo) & (x <= hi)
    up = np.clip((x - lo) / TAPER_OCTAVES, 0.0, 1.0)
    down = np.clip((hi - x) / TAPER_OCTAVES, 0.0, 1.0)
    taper = 0.5 - 0.5 * np.cos(np.pi * np.minimum(up, down))
    w[inside] = taper[inside]
    return w


def synthesise(size_px: int, pixel_m: float, lam_lo: float, lam_hi: float, amplitude_m: float, hurst: float,
               baseline_m: float, seed: int) -> np.ndarray:
    """Periodic Gaussian field with the self-affine spectrum inside [lam_lo, lam_hi] (heights in metres)."""
    length = size_px * pixel_m
    fy = np.fft.fftfreq(size_px, d=pixel_m)[:, None]            # cycles / m
    fx = np.fft.rfftfreq(size_px, d=pixel_m)[None, :]
    k = np.hypot(fx, fy)
    k[0, 0] = 1.0
    c = octave_psd_constant(amplitude_m, hurst, baseline_m)
    density = c * k ** (-(2.0 * hurst + 2.0)) * band_weight(1.0 / k, lam_lo, lam_hi) ** 2
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


def normals_from_height(height_m: np.ndarray, pixel_m: float) -> np.ndarray:
    """Tangent-space normals (OpenGL: +y up = image top) as uint8 RGB, periodic central differences."""
    dx = (np.roll(height_m, -1, axis=1) - np.roll(height_m, 1, axis=1)) / (2.0 * pixel_m)
    drow = (np.roll(height_m, -1, axis=0) - np.roll(height_m, 1, axis=0)) / (2.0 * pixel_m)
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
    for name, tile, lam_lo, lam_hi, seed in REGOLITH_SCALES:
        pixel = tile / SIZE_PX
        height = synthesise(SIZE_PX, pixel, lam_lo, lam_hi, amplitude, hurst, baseline, seed)
        code, scale = encode_height(height)
        save_png(code, out_dir / f"regolith_height_{name}.png")
        save_png(normals_from_height(height, pixel), out_dir / f"regolith_normal_{name}.png")
        meta["scales"].append({"name": name, "tile_m": tile, "pixel_m": pixel, "wavelength_min_m": lam_lo,
                               "wavelength_max_m": lam_hi, "height_scale_m": scale, "rms_m": float(height.std())})
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
