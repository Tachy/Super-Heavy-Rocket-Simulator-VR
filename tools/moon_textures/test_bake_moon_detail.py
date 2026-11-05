"""Tests of the procedural Moon texture bake (python -m pytest tools/moon_textures)."""

from __future__ import annotations

import ast
import sys
import tomllib
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bake_moon_detail as bake  # noqa: E402


@pytest.fixture(scope="module")
def baked(tmp_path_factory):
    first = tmp_path_factory.mktemp("bake_a")
    second = tmp_path_factory.mktemp("bake_b")
    meta = bake.bake(first)
    bake.bake(second)
    return first, second, meta


@pytest.fixture(scope="module")
def roughness():
    return bake.load_roughness()


def load(path: Path) -> np.ndarray:
    return np.array(Image.open(path))


def decoded_heights(folder: Path, scale: dict) -> np.ndarray:
    raw = load(folder / f"regolith_height_{scale['name']}.png").astype(np.float64)
    return (raw - 32768.0) / 32767.0 * scale["height_scale_m"]


def test_bake_is_byte_identical(baked):
    first, second, _ = baked
    files = sorted(p.name for p in first.iterdir())
    assert files == sorted(p.name for p in second.iterdir())
    assert len(files) == 3 * 2 + 4 * 2 + 1
    for name in files:
        assert (first / name).read_bytes() == (second / name).read_bytes(), name


def seam_ratio(image: np.ndarray) -> tuple[float, float, float]:
    """RMS of the wrap-around neighbour difference over the RMS of the interior one (both axes), and the maxima."""
    img = image.astype(np.float64)
    if img.ndim == 3:
        img = img.reshape(img.shape[0], img.shape[1], -1)
    seam = np.concatenate([(img[:, 0] - img[:, -1]).ravel(), (img[0] - img[-1]).ravel()])
    inner = np.concatenate([(img[:, 1:] - img[:, :-1]).ravel(), (img[1:] - img[:-1]).ravel()])
    return float(np.sqrt((seam**2).mean() / (inner**2).mean())), float(np.abs(seam).max()), float(np.abs(inner).max())


def test_maps_tile_seamlessly(baked):
    # a texture of nonzero slope cannot have an edge difference <= 1 LSB; seamless means the wrap-around neighbours
    # differ like any interior neighbours (rms ratio ~1) and no seam step exceeds the interior maximum by more than 1 LSB
    first, _, _ = baked
    for path in sorted(first.glob("*.png")):
        ratio, seam_max, inner_max = seam_ratio(load(path))
        assert ratio < 1.1, (path.name, ratio)    # a seam would raise it; step-edged maps (breccia) may sit below 1
        assert seam_max <= inner_max + 1, (path.name, seam_max, inner_max)


def radial_variance(field: np.ndarray, pixel_m: float, k_lo: float, k_hi: float) -> float:
    """Variance of the modes with k in [k_lo, k_hi) (cycles / m) of a periodic field."""
    n = field.shape[0]
    power = np.abs(np.fft.fft2(field)) ** 2 / n**4
    k = np.hypot(np.fft.fftfreq(n, d=pixel_m)[:, None], np.fft.fftfreq(n, d=pixel_m)[None, :])
    return float(power[(k >= k_lo) & (k < k_hi)].sum())


def law_variance(amplitude: float, hurst: float, baseline: float, k_lo: float, k_hi: float) -> float:
    c = bake.octave_psd_constant(amplitude, hurst, baseline)
    return 2.0 * np.pi * c * (k_lo ** (-2 * hurst) - k_hi ** (-2 * hurst)) / (2.0 * hurst)


def test_regolith_spectrum_matches_config_law(baked, roughness):
    first, _, meta = baked
    hurst, baseline = roughness["hurst_exponent"], roughness["baseline_m"]
    a = meta["octave_amplitude_m"]
    for scale in meta["scales"]:
        field = decoded_heights(first, scale)
        lam_lo = scale["wavelength_min_m"] * 2 ** bake.TAPER_OCTAVES
        lam_hi = scale["wavelength_max_m"] * 2 ** -bake.TAPER_OCTAVES
        edges = np.geomspace(1.0 / lam_hi, 1.0 / lam_lo, 4)           # three equal log bands over the flat part
        measured = [radial_variance(field, scale["pixel_m"], edges[i], edges[i + 1]) for i in range(3)]
        expected = [law_variance(a, hurst, baseline, edges[i], edges[i + 1]) for i in range(3)]
        for m, e in zip(measured, expected):
            assert abs(np.sqrt(m / e) - 1.0) < 0.15, (scale["name"], np.sqrt(m / e))
        # slope: variance ratio of neighbouring log bands -> Hurst exponent (variance per band ~ k^-2H)
        band_ratio = (edges[1] / edges[0])
        fitted_h = -0.5 * np.log(np.array(measured[2]) / np.array(measured[0])) / np.log(band_ratio**2)
        assert abs(fitted_h - hurst) < 0.15, (scale["name"], fitted_h)
        # nothing outside the band
        outside = radial_variance(field, scale["pixel_m"], 0.0, 1.0 / (scale["wavelength_max_m"] * 1.05))
        assert outside < 1e-6 * sum(measured)


VALIDATION_SCRIPT = Path(__file__).resolve().parents[1] / "validation" / "moon_roughness_residual.py"


def load_validation_functions() -> dict:
    """Constants and the `fbm` / `window_rms` functions of tools/validation/moon_roughness_residual.py (the Python mirror of
    the core's Guo statistic and roughness field), executed without running the script's crater / rock synthesis."""
    tree = ast.parse(VALIDATION_SCRIPT.read_text(encoding="utf-8"))
    keep = {"PX", "L", "N", "W_PX", "H", "rng", "fbm", "window_rms"}
    body = [n for n in tree.body if (isinstance(n, ast.FunctionDef) and n.name in keep)
            or (isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id in keep for t in n.targets))]
    body = [n for n in body if not (isinstance(n, ast.Assign) and "sys.argv" in ast.dump(n))]
    namespace = {"np": np, "SEED": 3}
    exec(compile(ast.Module(body=[ast.parse("rng = np.random.default_rng(SEED)").body[0]] + body, type_ignores=[]),
                 str(VALIDATION_SCRIPT), "exec"), namespace)
    return namespace


def test_baked_amplitude_agrees_with_the_core_roughness_law_independently(baked, roughness):
    # The validation script's field (spectrum k^-(H+1), 8 m patch, 5 mm samples, no bake code) is scaled so Guo's statistic
    # (median window std, same code as the core's derive_roughness_scale mirrored there) is exactly the config 3.7 mm. Its
    # octave variances must then equal the law with the baked amplitude a (calibrated separately, 10.24 m field).
    v = load_validation_functions()
    field = v["fbm"]()
    field = field / np.median(v["window_rms"](field)) * roughness["rms_height_at_baseline_m"]
    hurst, baseline = roughness["hurst_exponent"], roughness["baseline_m"]
    a = baked[2]["octave_amplitude_m"]
    for lam_hi, lam_lo in ((0.8, 0.4), (0.4, 0.2), (0.2, 0.1), (0.1, 0.05), (0.05, 0.025)):   # octaves, 25 mm .. 0.8 m
        measured = radial_variance(field, v["PX"], 1.0 / lam_hi, 1.0 / lam_lo)
        expected = law_variance(a, hurst, baseline, 1.0 / lam_hi, 1.0 / lam_lo)
        assert abs(np.sqrt(measured / expected) - 1.0) < 0.10, (lam_hi, np.sqrt(measured / expected))


def test_calibration_reproduces_the_config_roughness_at_several_windows(baked, roughness):
    # an independent full-spectrum field (other seed) measured with Guo's statistic at 125 mm / 0.5 m / 1 m windows
    _, _, meta = baked
    hurst, baseline = roughness["hurst_exponent"], roughness["baseline_m"]
    a_target = roughness["rms_height_at_baseline_m"]
    n, px = bake.CALIBRATION_SIZE_PX, bake.CALIBRATION_PIXEL_M
    field = bake.synthesise(n, px, 2 * px, n * px, meta["octave_amplitude_m"], hurst, baseline, 777)[::2, ::2]
    for stride, window_m in ((1, 0.125), (4, 0.5), (8, 1.0)):
        measured = bake.guo_statistic(field, stride, 1536, 99)
        law = a_target * (window_m / baseline) ** hurst
        assert abs(measured / law - 1.0) < 0.15, (window_m, measured, law)


def test_rock_albedo_mean_is_one(baked):
    first, _, _ = baked
    for variant in range(4):
        code = load(first / f"rock_albedo_{variant}.png").astype(np.float64)
        assert abs(code.mean() / 128.0 - 1.0) < 0.01, variant
        assert code.std() / 128.0 > 0.03, variant                     # it does vary
        assert 0 < code.min() and code.max() < 255                    # not clipped to the range


def test_normals_are_unit_and_upward(baked):
    first, _, meta = baked
    for path in list(first.glob("*_normal_*.png")):
        n = load(path).astype(np.float64) / 255.0 * 2.0 - 1.0
        assert (n[..., 2] > 0).all(), path.name
        assert abs(np.linalg.norm(n, axis=-1).mean() - 1.0) < 0.01, path.name


def test_metadata_matches_the_files(baked):
    first, _, meta = baked
    parsed = tomllib.loads((first / "moon_detail.toml").read_text(encoding="utf-8"))
    assert [r["name"] for r in parsed["regolith"]] == ["s0", "s1", "s2"]
    assert parsed["hurst_exponent"] == meta["hurst_exponent"]
    for r in parsed["regolith"]:
        assert r["tile_m"] / 512 == pytest.approx(r["pixel_m"])
    # bands are contiguous and disjoint
    assert parsed["regolith"][0]["wavelength_max_m"] == parsed["regolith"][1]["wavelength_min_m"]
    assert parsed["regolith"][1]["wavelength_max_m"] == parsed["regolith"][2]["wavelength_min_m"]
