"""Unit tests of the Earth terrain pipeline (datum conversion, area boxes, resampling, gap filling). No network.

Run: python -m pytest tools/data_pipeline/test_earth_terrain.py
"""

import math
import sys
import tomllib
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent))
import earth_terrain as et  # noqa: E402


def test_orthometric_to_ellipsoidal_adds_geoid():
    # a synthetic 2.5' geoid grid whose undulation at node (i, j) is known: N = (i * 7 + j * 3 % 11 - 40) cm
    lines, samples = 4321, 8640
    cm = ((np.arange(lines)[:, None] * 7 + (np.arange(samples)[None, :] * 3) % 11) - 40).astype(np.int32)
    # a target grid that sits exactly on geoid nodes: 5 x 4 nodes, geoid steps 1/24 deg
    i0, j0 = 1000, 2000
    geo = et.Geo(north=90.0 - i0 / 24.0, west=-180.0 + j0 / 24.0, step_lat=1 / 24.0, step_lon=1 / 24.0, lines=5, samples=4)
    h_ortho = np.linspace(-20.0, 150.0, 20).reshape(5, 4)
    h_ell = et.orthometric_to_ellipsoidal(h_ortho, geo.lats(), geo.lons(), cm)
    for i in range(5):
        for j in range(4):
            n_m = cm[i0 + i, j0 + j] * 0.01
            assert h_ell[i, j] == pytest.approx(h_ortho[i, j] + n_m, abs=1e-9)


def test_navd88_converted_with_geoid18(tmp_path):
    import rasterio
    from rasterio.transform import from_origin

    # synthetic GEOID18-like grid (pixel-centred, EPSG:4326): N = -26 + 0.5 * (lat - 26) + 2 * (lon + 97) [m]; linear, so
    # the bilinear grid shift reproduces it exactly
    path = tmp_path / "us_noaa_g2018u0.tif"
    step = 1.0 / 60.0
    lons = -98.0 + step * (np.arange(180) + 0.5)
    lats = 27.0 - step * (np.arange(120) + 0.5)
    n_grid = (-26.0 + 0.5 * (lats[:, None] - 26.0) + 2.0 * (lons[None, :] + 97.0)).astype(np.float32)
    with rasterio.open(path, "w", driver="GTiff", height=120, width=180, count=1, dtype="float32", crs="EPSG:4326",
                       transform=from_origin(-98.0, 27.0, step, step)) as ds:
        ds.write(n_grid, 1)
    lat = np.array([25.9972, 26.4, 25.5])
    lon = np.array([-97.1561, -96.7, -97.9])
    h_navd = np.array([2.0, 10.5, -3.0])
    h_ell = et.navd88_to_ellipsoidal(h_navd, lat, lon, path)
    expected = h_navd + (-26.0 + 0.5 * (lat - 26.0) + 2.0 * (lon + 97.0))
    assert np.max(np.abs(h_ell - expected)) < 1e-3  # within 1 mm of the grid shift (N is added to the orthometric height)


def test_area_bounds():
    south, north, west, east = et.area_bounds(25.99686, -97.15394, 5000.0)
    lat_c = math.radians(25.99686)
    # WGS84 radii of curvature at the latitude
    a, e2 = 6378137.0, 0.00669437999014
    w = math.sqrt(1 - e2 * math.sin(lat_c) ** 2)
    m, n = a * (1 - e2) / w**3, a / w
    assert north - 25.99686 == pytest.approx(math.degrees(5000.0 / m), rel=1e-9)
    assert 25.99686 - south == pytest.approx(north - 25.99686, rel=1e-9)
    assert east - (-97.15394) == pytest.approx(math.degrees(5000.0 / (n * math.cos(lat_c))), rel=1e-9)
    assert (-97.15394) - west == pytest.approx(east + 97.15394, rel=1e-9)
    # the box is ~10 km on a side
    assert (north - south) * 110574.0 == pytest.approx(10000.0, rel=5e-3)
    assert (east - west) * 111320.0 * math.cos(lat_c) == pytest.approx(10000.0, rel=5e-3)


def test_grid_for_box_is_centred_with_exact_extent():
    geo = et.grid_for_box(28.585, -80.590, 6000.0, 1 / 108000.0, 1 / 108000.0 / math.cos(math.radians(28.585)))
    assert geo.lines % 2 == 1 and geo.samples % 2 == 1
    assert (geo.north + geo.south) / 2 == pytest.approx(28.585, abs=1e-9)
    assert (geo.west + geo.east) / 2 == pytest.approx(-80.590, abs=1e-9)
    s, n, w, e = et.area_bounds(28.585, -80.590, 6000.0)
    assert geo.north >= n and geo.north - n < 2 * geo.step_lat
    assert geo.east >= e and geo.east - e < 2 * geo.step_lon


def test_resample_global_wraps():
    # 1' pixels (ratio 2 per 2' node): 4 lines x 6 columns -> 3 node lines x 3 node columns; columns wrap at the antimeridian
    raster = np.arange(24, dtype=np.float64).reshape(4, 6)
    nodes = et.resample_global(raster, 2)
    assert nodes.shape == (3, 3)
    # node (1, 1): block of rows 1..2, columns 1..2
    assert nodes[1, 1] == pytest.approx(raster[1:3, 1:3].mean())
    # node (1, 0) at the antimeridian: columns -1 and 0 (wrapped)
    assert nodes[1, 0] == pytest.approx(raster[1:3, [5, 0]].mean())
    # pole lines: only the single pixel row on the map side
    assert nodes[0, 1] == pytest.approx(raster[0, 1:3].mean())
    assert nodes[2, 2] == pytest.approx(raster[3, 3:5].mean())


def test_local_tile_nodata_filled_from_regional():
    regional_geo = et.Geo(north=1.0, west=0.0, step_lat=0.1, step_lon=0.1, lines=11, samples=11)
    lat_r, lon_r = regional_geo.lats()[:, None], regional_geo.lons()[None, :]
    regional = 100.0 + 30.0 * lat_r + 10.0 * lon_r  # linear: bilinear sampling is exact
    local_geo = et.Geo(north=0.65, west=0.25, step_lat=0.01, step_lon=0.01, lines=31, samples=41)
    lat_l, lon_l = local_geo.lats()[:, None], local_geo.lons()[None, :]
    local = np.full((31, 41), 5.0, dtype=np.float32)
    mask = np.zeros((31, 41), dtype=bool)
    mask[3:9, 10:20] = True
    mask[:, 0] = True
    local[mask] = np.nan
    filled, count = et.fill_nodata_from_regional(local, local_geo, regional, regional_geo)
    assert count == int(mask.sum())
    assert not np.isnan(filled).any()
    assert np.all(filled[~mask] == 5.0)  # valid 3DEP values untouched
    expected = 100.0 + 30.0 * lat_l + 10.0 * lon_l
    assert np.max(np.abs(filled[mask] - np.broadcast_to(expected, filled.shape)[mask])) < 1e-3


def test_config_parses_areas():
    cfg = et.load_config(Path(__file__).resolve().parents[2] / "config" / "earth_terrain.toml")
    ids = [a["id"] for a in cfg["area"]]
    assert ids == ["starbase", "cape"]
    assert cfg["global"]["relief_step_arcmin"] == 2.0
    for a in cfg["area"]:
        assert a["local_half_size_m"] < a["regional_half_size_m"]
    with (Path(__file__).resolve().parents[2] / "config" / "earth_terrain.toml").open("rb") as f:
        tomllib.load(f)
