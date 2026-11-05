"""Unit tests of the Earth surface albedo transfer and calibration (earth_albedo.py). No network.

Run: python -m pytest tools/data_pipeline/test_earth_albedo.py
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent))
import earth_albedo as ea  # noqa: E402


def test_srgb_round_trip_and_reference_points():
    counts = np.arange(256, dtype=np.uint8)
    assert (ea.encode_srgb(ea.decode_srgb(counts)) == counts).all()
    assert ea.decode_srgb(np.array([255]))[0] == pytest.approx(1.0)
    assert ea.decode_srgb(np.array([188]))[0] == pytest.approx(0.5029, abs=1e-3)  # IEC 61966-2-1
    assert ea.encode_srgb(np.array([-0.1, 1.7])).tolist() == [0, 255]  # clipped


def test_isotonic_pools_violators():
    out = ea.isotonic(np.array([1.0, 3.0, 2.0, 4.0]), np.ones(4))
    assert out.tolist() == pytest.approx([1.0, 2.5, 2.5, 4.0])
    out = ea.isotonic(np.array([1.0, 3.0, 2.0]), np.array([1.0, 1.0, 3.0]))
    assert out.tolist() == pytest.approx([1.0, 2.25, 2.25])


def test_monotone_lut_recovers_a_stretch():
    """A display stretch (gamma-like, the Blue Marble's kind) with noise is inverted to within the noise."""
    rng = np.random.default_rng(1)
    rho = rng.uniform(0.0, 0.6, 200_000)
    counts = np.clip(np.rint(255.0 * (rho / 0.6) ** 0.45 + rng.normal(0, 2.0, rho.size)), 0, 255)
    lut = ea.monotone_lut(counts, rho)
    assert (np.diff(lut) >= 0).all() and lut[0] == pytest.approx(0.0, abs=1e-3)
    probe = np.array([64, 128, 200])
    truth = 0.6 * (probe / 255.0) ** (1 / 0.45)
    assert lut[probe] == pytest.approx(truth, rel=0.08)


def test_monotone_lut_extrapolates_unsupported_counts():
    counts = np.repeat(np.arange(50, 101), 30)
    rho = counts / 500.0
    lut = ea.monotone_lut(counts, rho)
    assert lut[25] == pytest.approx(0.05, rel=0.01)  # proportional below the lowest supported count
    assert lut[150] == pytest.approx(0.3, rel=0.02)  # the last slope above the highest
    assert lut.max() <= 1.0


def test_apply_lut_per_channel():
    luts = np.stack([np.linspace(0, 1, 256), np.linspace(0, 0.5, 256), np.zeros(256)])
    out = ea.apply_lut(np.array([[255, 255, 255]], dtype=np.uint8), luts)
    assert out[0].tolist() == pytest.approx([1.0, 0.5, 0.0])


def test_sinusoidal_inverse():
    lat, lon = ea.sinusoidal_to_latlon(np.array([0.0, ea.MODIS_SPHERE_M * np.radians(30.0) * np.cos(np.radians(45.0))]),
                                       np.array([0.0, ea.MODIS_SPHERE_M * np.radians(45.0)]))
    assert lat.tolist() == pytest.approx([0.0, 45.0]) and lon.tolist() == pytest.approx([0.0, 30.0])
    _, lon = ea.sinusoidal_to_latlon(np.array([ea.MODIS_SPHERE_M * np.pi]), np.array([ea.MODIS_SPHERE_M * np.radians(60.0)]))
    assert np.isnan(lon[0])  # outside the projection's domain


def test_bins_average_samples_per_pixel():
    bins = ea._Bins(180, 360)
    bins.add(np.array([0.5, 0.5, -45.5]), np.array([10.5, 10.5, 100.5]),
             np.array([[0.1, 0.2, 0.3], [0.3, 0.4, 0.5], [0.9, 0.9, 0.9]]))
    mean, hits = bins.mean()
    assert hits[89, 190] == 2 and mean[89, 190].tolist() == pytest.approx([0.2, 0.3, 0.4])
    assert hits[135, 280] == 1 and np.isnan(mean[0, 0]).all()


def test_build_day_texture_prefers_the_climatology(tmp_path, monkeypatch):
    from PIL import Image

    (tmp_path / "data" / "raw").mkdir(parents=True)
    (tmp_path / "config").mkdir()
    Image.fromarray(np.full((4, 8, 3), 100, dtype=np.uint8), "RGB").save(tmp_path / "data/raw/earth_day_5400.jpg",
                                                                         quality=100)
    ea.write_config(tmp_path / "config" / "earth_albedo.toml", np.tile(np.linspace(0, 0.51, 256), (3, 1)),
                    {"tiles": 1, "pairs": 1, "median_rel_luma_error": 0, "p90_rel_luma_error": 0,
                     "mean_luma_ref": 0, "mean_luma_fit": 0})
    rho = np.full((4, 8, 3), np.nan, dtype=np.float32)
    hits = np.zeros((4, 8), dtype=np.uint16)
    rho[0, 0], hits[0, 0] = (0.3, 0.3, 0.3), 2
    clim = tmp_path / ea.CLIMATOLOGY_FILE.relative_to(ea.REPO_ROOT)
    np.savez_compressed(clim, rho=rho, hits=hits)
    counts = ea.build_day_texture(tmp_path)
    assert ea.decode_srgb(counts[0, 0]).tolist() == pytest.approx([0.3] * 3, rel=0.02)  # the climatology
    assert ea.decode_srgb(counts[2, 5]).tolist() == pytest.approx([0.2] * 3, rel=0.03)  # the table: 100 / 255 x 0.51


def test_embed_sentinel2_takes_the_box_and_feathers_outside(tmp_path):
    """Inside a Sentinel-2 box the global pixels carry the Sentinel-2 box mean; outside, the edge ratio fades out over
    EMBED_FEATHER_M; far away the global level is untouched."""
    import rasterio
    from rasterio.transform import from_bounds

    (tmp_path / "config").mkdir()
    (tmp_path / "config" / "imagery.toml").write_text(
        '[[source]]\nid = "s2_t"\nproduct = "sentinel2_median"\narea = "t"\nbox = "regional"\nmin_level = 8\n'
        'max_level = 13\n', encoding="utf-8")
    (tmp_path / "config" / "earth_terrain.toml").write_text(
        '[[area]]\nid = "t"\nlatitude_deg = 26.0\nlongitude_deg = -97.0\nlocal_half_size_m = 5000.0\n'
        'regional_half_size_m = 100000.0\n', encoding="utf-8")
    raw = tmp_path / "data" / "raw" / "imagery" / "s2_t"
    raw.mkdir(parents=True)
    with rasterio.open(raw / "reflectance.tif", "w", driver="GTiff", width=200, height=200, count=3, dtype="uint16",
                       crs="EPSG:4326", transform=from_bounds(-98.5, 24.5, -95.5, 27.5, 200, 200), nodata=0) as d:
        d.write(np.full((3, 200, 200), 2000, dtype=np.uint16))  # reflectance 0.2
    rho = np.full((2700, 5400, 3), 0.1)
    out = ea.embed_sentinel2(rho.copy(), tmp_path, log=lambda *_: None)
    h, w = 2700, 5400
    px = lambda lat, lon: (int((90.0 - lat) / 180.0 * h), int((lon + 180.0) / 360.0 * w))  # noqa: E731
    assert out[px(26.0, -97.0)] == pytest.approx([0.2] * 3, rel=1e-3)  # inside: the Sentinel-2 mean
    edge = out[px(26.0, -97.0 + 1.0 + 0.1)][0]  # ~10 km east of the box edge: most of the ratio 2
    far = out[px(26.0, -97.0 + 1.0 + 0.45)][0]  # ~45 km out: nearly faded
    assert 0.15 < edge < 0.2 and 0.1 <= far < edge
    assert out[px(26.0, -90.0)] == pytest.approx([0.1] * 3)  # beyond the feather: untouched
