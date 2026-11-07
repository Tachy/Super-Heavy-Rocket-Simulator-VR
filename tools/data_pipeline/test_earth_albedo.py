"""Unit tests of the Earth surface albedo transfer and calibration (earth_albedo.py). No network.

Run: python -m pytest tools/data_pipeline/test_earth_albedo.py
"""

import sys
from pathlib import Path

import math

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


def _write_relief(root: Path, h: int, w: int) -> None:
    """West half 20 m ellipsoidal (land over a 10 m geoid), east half -100 m (sea)."""
    (root / "data" / "terrain" / "earth").mkdir(parents=True)
    (root / "data" / "geoid").mkdir(parents=True)
    relief = np.full((2 * h + 1, 2 * w), -100, dtype="<i2")
    relief[:, :w] = 20
    relief.tofile(root / "data" / "terrain" / "earth" / "r.i16")
    geoid = np.full((5, 10), 1000, dtype="<i2")  # 10 m undulation (dn 0.01 m)
    geoid.tofile(root / "data" / "geoid" / "g.i16")

    def header(name, lines, samples):
        return (f'[grid]\nheights = "{name}"\nlines = {lines}\nsamples = {samples}\n'
                f'dn_scale_m = {1.0 if name == "r.i16" else 0.01}\nnorth_latitude_deg = 90.0\nwest_longitude_deg = -180.0\n'
                f'step_latitude_deg = {180.0 / (lines - 1)}\nstep_longitude_deg = {360.0 / samples}\n')
    (root / "data" / "terrain" / "earth" / "global_relief.toml").write_text(header("r.i16", 2 * h + 1, 2 * w))
    (root / "data" / "geoid" / "egm2008_25.toml").write_text(header("g.i16", 5, 10))


def test_land_fraction_from_relief_above_the_geoid(tmp_path):
    """3 x 3 relief nodes per pixel; land = ellipsoidal height minus the geoid above the core's water threshold."""
    h, w = 4, 8
    _write_relief(tmp_path, h, w)
    frac = ea.land_fraction(tmp_path, h, w)
    assert frac[:, 1].tolist() == pytest.approx([1.0] * h)  # 20 m - 10 m: land
    assert frac[:, 5].tolist() == pytest.approx([0.0] * h)
    assert frac[0, 3] == pytest.approx(2.0 / 3.0)  # the pixel at the edge sees one sea column of three
    assert ea.land_fraction(tmp_path, h + 1, w) is None  # a grid that does not match


def _write_area_grids(root: Path, area: str = "t") -> None:
    """Regional grid 0.1 deg steps over lat 25..27, lon -98..-96: 0 m ellipsoidal west of -97 (sea over a 0 m geoid),
    +10 m east of it; geoid file all 0."""
    (root / "data" / "terrain" / "earth").mkdir(parents=True, exist_ok=True)
    (root / "data" / "geoid").mkdir(parents=True, exist_ok=True)
    lines, samples = 21, 21
    heights = np.zeros((lines, samples), dtype="<i2")
    heights[:, 11:] = 100  # 0.1 m scale: 10 m
    heights.tofile(root / "data" / "terrain" / "earth" / f"{area}_regional.i16")
    np.zeros((181, 360), dtype="<i2").tofile(root / "data" / "geoid" / "g.i16")
    (root / "data" / "terrain" / "earth" / f"{area}_regional.toml").write_text(
        f'[grid]\nheights = "{area}_regional.i16"\nlines = {lines}\nsamples = {samples}\ndn_scale_m = 0.1\n'
        'north_latitude_deg = 27.0\nwest_longitude_deg = -98.0\nstep_latitude_deg = 0.1\nstep_longitude_deg = 0.1\n')
    (root / "data" / "geoid" / "egm2008_25.toml").write_text(
        '[grid]\nheights = "g.i16"\nlines = 181\nsamples = 360\ndn_scale_m = 0.01\nnorth_latitude_deg = 90.0\n'
        'west_longitude_deg = -180.0\nstep_latitude_deg = 1.0\nstep_longitude_deg = 1.0\n')


def test_water_mask_follows_the_renderers_sea_rule(tmp_path):
    _write_area_grids(tmp_path)
    mask = ea.WaterMask(tmp_path, "t")
    lat = np.array([[26.0, 26.0, 26.0]])
    lon = np.array([[-97.5, -96.5, -96.9]])  # sea, land (10 m), inside the ramp between the two
    assert mask.water(lat, lon).tolist() == [[True, False, False]]  # 0 m is sea (<= 0.05 m); the ramp is above 0.05 m


def test_land_reflectance_removes_the_sea_only(tmp_path):
    import rasterio
    from rasterio.transform import from_bounds

    _write_area_grids(tmp_path)
    src = tmp_path / "reflectance.tif"
    with rasterio.open(src, "w", driver="GTiff", width=100, height=100, count=3, dtype="uint16", crs="EPSG:4326",
                       transform=from_bounds(-98.0, 25.0, -96.0, 27.0, 100, 100), nodata=0) as d:
        d.write(np.full((3, 100, 100), 2000, dtype=np.uint16))
        d.write(np.full((100, 10), 3000, dtype=np.uint16), 1, window=rasterio.windows.Window(0, 0, 10, 100))  # bright sea strip
    out = ea.build_land_reflectance(src, tmp_path / "land.tif", ea.WaterMask(tmp_path, "t"), np.array([0.25, 0.25, 0.25]),
                                    log=lambda *_: None)
    with rasterio.open(out) as d:
        a = d.read()
    assert (a[:3, :, :5] == 0).all() and (a[:3, :, 95:] == 2000).all()  # west sea gone from the land bands, east land kept
    assert (a[3:, :, 95:] == 0).all()  # no water excess on land
    assert (a[3, :, 20:25] == 1).all()  # sea below the pedestal: excess 0 (+ 1 count)
    assert (a[3, :, :5] == 1 + 500).all() and (a[4, :, :5] == 1).all()  # red of the strip: 0.3 - 0.25 above the pedestal


def test_embed_sentinel2_is_land_only(tmp_path):
    """Land pixels take Sentinel-2; the feather ratio ignores sea pixels; sea pixels take the calibrated open-water level of
    the box (the median of the L2A open water), fading back to the global sea over SEA_FEATHER_M."""
    import rasterio
    from rasterio.transform import from_bounds

    _write_area_grids(tmp_path)  # sea west of -97, land east: lon -97.0 is the coast
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
                       crs="EPSG:4326", transform=from_bounds(-98.0, 25.0, -96.0, 27.0, 200, 200), nodata=0) as d:
        d.write(np.full((3, 200, 200), 2000, dtype=np.uint16))
    h, w = 2700, 5400
    land = np.zeros((h, w), dtype=bool)
    land[:, int((-97.0 + 180.0) / 360.0 * w):] = True
    out = ea.embed_sentinel2(np.full((h, w, 3), 0.1), tmp_path, log=lambda *_: None, land=land)
    px = lambda lat, lon: (int((90.0 - lat) / 180.0 * h), int((lon + 180.0) / 360.0 * w))  # noqa: E731
    assert out[px(26.0, -96.5)] == pytest.approx([0.2] * 3, rel=1e-3)  # land inside: Sentinel-2
    assert out[px(26.0, -97.5)] == pytest.approx([0.2] * 3, rel=1e-3)  # sea inside the box: the calibrated L2A open water
    assert out[px(26.0, -95.8)][0] > 0.1  # land beyond the box edge: the feathered ratio
    # sea beyond the box edge: cosine fade from the calibrated level to the global one over SEA_FEATHER_M
    dist_m = 0.15 * 111320.0 * math.cos(math.radians(26.0))
    w_sea = 0.5 - 0.5 * math.cos(math.pi * (1.0 - dist_m / ea.SEA_FEATHER_M))
    assert out[px(26.0, -98.15)][0] == pytest.approx(0.1 + 0.1 * w_sea, abs=0.01)
    assert 0.1 < out[px(26.0, -98.15)][0] < 0.2
    assert out[px(26.0, -99.5)] == pytest.approx([0.1] * 3)  # > 100 km beyond the box: the global sea


def test_embed_sentinel2_without_open_water_leaves_the_global_sea(tmp_path, monkeypatch):
    """Review M-4 (follow-ups): too few open-water samples give no calibrated sea level (open_water_levels -> None): the sea
    inside the box is the global sea plus the imagery's full water brightness (pedestal zero), the sea outside stays global."""
    import rasterio
    from rasterio.transform import from_bounds

    _write_area_grids(tmp_path)
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
                       crs="EPSG:4326", transform=from_bounds(-98.0, 25.0, -96.0, 27.0, 200, 200), nodata=0) as d:
        d.write(np.full((3, 200, 200), 2000, dtype=np.uint16))
    monkeypatch.setattr(ea, "OPEN_WATER_MIN_PIXELS", 10 ** 9)  # no scene has that much open water: level None
    logs = []
    h, w = 2700, 5400
    land = np.zeros((h, w), dtype=bool)
    land[:, int((-97.0 + 180.0) / 360.0 * w):] = True
    out = ea.embed_sentinel2(np.full((h, w, 3), 0.1), tmp_path, log=logs.append, land=land)
    px = lambda lat, lon: (int((90.0 - lat) / 180.0 * h), int((lon + 180.0) / 360.0 * w))  # noqa: E731
    assert any("open-water samples" in m for m in logs)
    assert out[px(26.0, -97.5)] == pytest.approx([0.3] * 3, rel=1e-3)  # inside: global 0.1 + the water's 0.2 above pedestal 0
    assert out[px(26.0, -98.15)] == pytest.approx([0.1] * 3)  # outside the box: the global sea, no calibrated fade
    assert out[px(26.0, -96.5)] == pytest.approx([0.2] * 3, rel=1e-3)  # land inside: Sentinel-2 as before


def test_sea_only_continues_the_sea_under_land(tmp_path):
    h, w = 4, 8
    (tmp_path / "data" / "terrain" / "earth").mkdir(parents=True)
    (tmp_path / "data" / "geoid").mkdir(parents=True)
    relief = np.full((2 * h + 1, 2 * w), -100, dtype="<i2")
    relief[:, :w] = 20  # west half land
    relief.tofile(tmp_path / "data" / "terrain" / "earth" / "r.i16")
    np.full((5, 10), 1000, dtype="<i2").tofile(tmp_path / "data" / "geoid" / "g.i16")

    def header(name, lines, samples, scale):
        return (f'[grid]\nheights = "{name}"\nlines = {lines}\nsamples = {samples}\ndn_scale_m = {scale}\n'
                f'north_latitude_deg = 90.0\nwest_longitude_deg = -180.0\nstep_latitude_deg = {180.0 / (lines - 1)}\n'
                f'step_longitude_deg = {360.0 / samples}\n')
    (tmp_path / "data" / "terrain" / "earth" / "global_relief.toml").write_text(header("r.i16", 2 * h + 1, 2 * w, 1.0))
    (tmp_path / "data" / "geoid" / "egm2008_25.toml").write_text(header("g.i16", 5, 10, 0.01))
    tex = np.zeros((h, w, 3))
    tex[:, :w // 2] = 0.5  # land texels
    tex[:, w // 2:] = 0.02  # sea texels
    out = ea.sea_only(tex, tmp_path)
    assert out[:, :w // 2].max() == pytest.approx(0.02)  # land texels carry the neighbouring sea value
    assert out[:, w // 2:].max() == pytest.approx(0.02)


def test_open_water_pedestal_is_read_over_open_water_only(tmp_path):
    import rasterio
    from rasterio.transform import from_bounds

    _write_area_grids(tmp_path)  # sea west of -97, land east
    n = 800
    a = np.full((3, n, n), 2000, dtype=np.uint16)  # land 0.2
    a[:, :, : n // 2] = 500  # sea 0.05
    a[:, :, 20:30] = 1500  # a bright surf strip near the west edge of the file is still open water (> 2 km from land)
    src = tmp_path / "r.tif"
    with rasterio.open(src, "w", driver="GTiff", width=n, height=n, count=3, dtype="uint16", crs="EPSG:4326",
                       transform=from_bounds(-98.0, 25.0, -96.0, 27.0, n, n), nodata=0) as d:
        d.write(a)
    ped = ea.open_water_pedestal(src, ea.WaterMask(tmp_path, "t"))
    # file pixel ~ 0.0025 deg ~ 275 m: the land edge is at column 400, the open water starts ~2 km (8 px x 8) before it
    assert ped.tolist() == pytest.approx([0.05] * 3, abs=0.01)  # the strip is a few % of the open water: below p90


def test_open_water_levels_median_and_pedestal(tmp_path):
    import rasterio
    from rasterio.transform import from_bounds

    _write_area_grids(tmp_path)
    n = 800
    a = np.full((3, n, n), 2000, dtype=np.uint16)
    a[:, :, : n // 2] = 500  # open sea 0.05 ...
    a[:, :, 20:60] = 1500  # ... with a bright 25 % of it
    src = tmp_path / "r.tif"
    with rasterio.open(src, "w", driver="GTiff", width=n, height=n, count=3, dtype="uint16", crs="EPSG:4326",
                       transform=from_bounds(-98.0, 25.0, -96.0, 27.0, n, n), nodata=0) as d:
        d.write(a)
    ped, level = ea.open_water_levels(src, ea.WaterMask(tmp_path, "t"))
    assert level.tolist() == pytest.approx([0.05] * 3, abs=0.005)  # the median ignores the bright quarter
    assert (ped > level).all()  # the pedestal (p90) sits above it


def test_open_water_pedestal_logs_when_there_is_no_open_water(tmp_path):
    """Task 15 minor M3: no open water in the file gives zeros, and the run says so (the embedding then adds the full water
    brightness)."""
    from rasterio.transform import from_bounds
    import rasterio
    _write_area_grids(tmp_path)  # sea west of -97, land east
    n = 100
    src = tmp_path / "land.tif"
    with rasterio.open(src, "w", driver="GTiff", width=n, height=n, count=3, dtype="uint16", crs="EPSG:4326",
                       transform=from_bounds(-96.5, 25.0, -96.0, 27.0, n, n), nodata=0) as d:  # land only
        d.write(np.full((3, n, n), 2000, dtype=np.uint16))
    lines = []
    ped = ea.open_water_pedestal(src, ea.WaterMask(tmp_path, "t"), log=lines.append)
    assert ped.tolist() == [0.0, 0.0, 0.0]
    assert len(lines) == 1 and "open-water samples" in lines[0] and "land.tif" in lines[0]


def test_open_ocean_reflectance_is_clear_case1_water():
    """pi Rrs at the channels' band centres; the Rrs set is oligotrophic Case-1 water by the NASA OC3M band ratio
    (chlorophyll 0.05-0.2 mg m^-3) and its 443 nm value matches the SeaWiFS clear-water nLw (0.0093-0.0096 sr^-1)."""
    rho = ea.open_ocean_reflectance()
    rrs = dict(zip(ea.OCEAN_RRS_WAVELENGTHS_NM, ea.OCEAN_RRS_SR))
    assert rho.tolist() == pytest.approx([math.pi * 0.00025, math.pi * 0.0019, math.pi * 0.0080], rel=1e-9)
    assert 0.05 <= ea.oc3m_chlorophyll(rrs[443.0], rrs[488.0], rrs[547.0]) <= 0.2
    assert 0.0093 <= rrs[443.0] <= 0.0096
    assert float(rho @ ea.LUMA_WEIGHTS) == pytest.approx(0.0064, abs=0.0005)  # luminance, vs 0.019 of the fill


def test_water_optics_sea_replaces_the_fill_and_keeps_the_excess():
    luts = np.tile(np.linspace(0.0, 1.0, 256), (3, 1))  # count / 255
    counts = np.array([[ea.BMNG_OCEAN_FILL_COUNTS, (2, 5, 60), (0, 0, 8), (200, 200, 200)]], dtype=np.uint8)
    rho = ea.apply_lut(counts, luts)
    sea = np.array([[True, True, True, False]])
    out = ea.water_optics_sea(rho.copy(), sea, luts)
    clear = ea.open_ocean_reflectance()
    assert out[0, 0].tolist() == pytest.approx(clear.tolist())  # the fill: clear water
    assert out[0, 1].tolist() == pytest.approx((clear + [0.0, 0.0, 40 / 255]).tolist())  # brighter: excess kept
    assert out[0, 2].tolist() == pytest.approx(clear.tolist())  # darker than the fill: clear water
    assert out[0, 3].tolist() == pytest.approx(rho[0, 3].tolist())  # land untouched


def test_shader_fallback_is_the_open_ocean():
    """water.gdshaderinc's deep_water (the sea body without a day texture) is the pipeline's open-ocean value (0.5 %)."""
    import re

    text = (Path(__file__).resolve().parents[2] / "game" / "shaders" / "water.gdshaderinc").read_text(encoding="utf-8")
    m = re.search(r"uniform vec3 deep_water = vec3\(([^)]*)\);", text)
    values = [float(v) for v in m.group(1).split(",")]
    assert values == pytest.approx(ea.open_ocean_reflectance().tolist(), rel=5e-3)


def test_sea_ice_is_not_open_ocean_water():
    """Pack ice (excess 0.3+) keeps the imagery and has water fraction 0; a bright bank (excess ~0.08) stays water."""
    luts = np.tile(np.linspace(0.0, 1.0, 256), (3, 1))
    counts = np.array([[ea.BMNG_OCEAN_FILL_COUNTS, (10, 30, 90), (200, 210, 220)]], dtype=np.uint8)
    rho = ea.apply_lut(counts, luts)
    assert ea.sea_ice(rho, luts).tolist() == [[False, False, True]]
    sea = np.array([[True, True, True]])
    out = ea.water_optics_sea(rho.copy(), sea, luts)
    assert out[0, 2].tolist() == pytest.approx(rho[0, 2].tolist())  # ice: untouched
    assert out[0, 1, 2] > ea.open_ocean_reflectance()[2]  # bank: water body plus its excess


def test_water_texture_masks_land_and_ice(tmp_path):
    """255 = ocean, land pixels (>= 0.5 land) 0, ice 0; no relief -> False and no file."""
    from PIL import Image

    h, w = 4, 8
    _write_relief(tmp_path, h, w)
    ice = np.full((h, w, 3), 8, dtype=np.uint8)
    ice[:] = ea.BMNG_OCEAN_FILL_COUNTS
    ice[0, 5] = (230, 235, 240)
    (tmp_path / "data" / "raw").mkdir(parents=True)
    Image.fromarray(ice, "RGB").save(tmp_path / "data" / "raw" / "earth_day_5400.jpg", quality=100, subsampling=0)
    (tmp_path / "config").mkdir()
    (tmp_path / "config" / "earth_albedo.toml").write_text(ea.CONFIG_FILE.read_text(encoding="utf-8"), encoding="utf-8")
    out = tmp_path / "t" / "water.png"
    assert ea.write_water_texture(tmp_path, out, (h, w))
    mask = np.asarray(Image.open(out))
    assert mask[:, 1].tolist() == [0] * h  # land
    assert mask[1:, 5].tolist() == [255] * (h - 1)  # ocean
    assert mask[0, 5] == 0  # ice
    assert not ea.write_water_texture(tmp_path / "none", tmp_path / "x.png", (h, w))
    assert not (tmp_path / "x.png").exists()
