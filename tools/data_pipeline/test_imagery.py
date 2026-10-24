"""Unit tests of the imagery pipeline (tile scheme, median composite, colour match, pyramid, NAIP datum shift).
Synthetic rasters only, no network.

Run: python -m pytest tools/data_pipeline/test_imagery.py
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent))
import imagery as im  # noqa: E402
import earth_terrain as et  # noqa: E402


def test_tile_bounds_level0():
    assert im.tiles_for_box(0, -90.0, 90.0, -180.0, 180.0) == [(0, 0), (1, 0)]
    south, north, west, east = im.tile_bounds(0, 1, 0)
    assert (south, north, west, east) == (-90.0, 90.0, 0.0, 180.0)


def test_tile_bounds_level17():
    size = 180.0 / 2**17
    south, north, west, east = im.tile_bounds(17, 5, 7)
    assert north - south == pytest.approx(size, rel=1e-12)
    assert east - west == pytest.approx(size, rel=1e-12)
    assert west == pytest.approx(-180.0 + 5 * size, rel=1e-12)
    assert north == pytest.approx(90.0 - 7 * size, rel=1e-12)


def test_tiles_for_box():
    z = 3  # 22.5 deg tiles
    # a box strictly inside tile (x=4, y=2): lon -90..-67.5, lat 45..22.5
    assert im.tiles_for_box(z, 30.0, 40.0, -80.0, -70.0) == [(4, 2)]
    # a box straddling the x boundary at -67.5 and the y boundary at 22.5
    got = im.tiles_for_box(z, 20.0, 30.0, -70.0, -60.0)
    assert sorted(got) == [(4, 2), (4, 3), (5, 2), (5, 3)]
    # a box that only touches the neighbouring tiles along an edge does not include them
    assert im.tiles_for_box(z, 22.5, 45.0, -90.0, -67.5) == [(4, 2)]


def test_median_ignores_clouds():
    rgb = np.zeros((3, 2, 2, 3), dtype=np.uint8)
    scl = np.full((3, 2, 2), 4, dtype=np.uint8)
    rgb[0], rgb[1], rgb[2] = 100, 110, 250  # third scene bright
    scl[2] = 9  # cloud (high probability)
    scl[1, 0, 0] = 3  # cloud shadow in the second scene at pixel (0, 0)
    out = im.clear_median(rgb, scl)
    assert out[0, 0, 0] == 100  # only scene 0 clear
    assert out[1, 1, 0] in (105, 106)  # median of 100 and 110
    # a pixel with no clear scene at all falls back to the valid pixels (no hole); nodata everywhere stays 0
    scl[:, 1, 1] = 9
    rgb[0, 1, 1] = 0
    out = im.clear_median(rgb, scl)
    assert out[1, 1, 0] in (180, 181)  # median of 110 and 250 (scene 0 is nodata there)
    rgb[:, 1, 1] = 0
    assert im.clear_median(rgb, scl)[1, 1, 0] == 0


def test_colour_match_recovers_gain():
    rng = np.random.default_rng(3)
    src = rng.uniform(20.0, 200.0, size=(500, 3))
    gain_true = np.array([1.3, 1.1, 0.9])
    offset_true = np.array([-10.0, 4.0, 12.0])
    ref = src * gain_true + offset_true + rng.normal(0.0, 0.3, size=src.shape)
    gain, offset = im.fit_gain_offset(src, ref)
    assert gain == pytest.approx(gain_true, rel=0.01)
    assert offset == pytest.approx(offset_true, abs=1.0)
    # a reference without variation (the 12 km day texture over a small box): mean match, gain 1
    g, o = im.fit_gain_offset(src, np.full_like(src, 90.0))
    assert g == pytest.approx([1.0, 1.0, 1.0])
    assert o == pytest.approx(90.0 - src.mean(axis=0))


def test_apply_gain_offset_keeps_nodata():
    rgb = np.array([[[0, 0, 0], [100, 100, 100]]], dtype=np.uint8)
    out = im.apply_gain_offset(rgb, np.array([1.3, 1.3, 1.3]), np.array([-10.0, -10.0, -10.0]))
    assert out[0, 0].tolist() == [0, 0, 0]
    assert out[0, 1].tolist() == [120, 120, 120]


def test_downsample_pyramid():
    rng = np.random.default_rng(5)
    kids = [rng.integers(0, 256, size=(256, 256, 3), dtype=np.uint8) for _ in range(4)]  # tl, tr, bl, br
    parent = im.downsample_children(*kids)
    assert parent.shape == (256, 256, 3) and parent.dtype == np.uint8
    big = np.zeros((512, 512, 3))
    big[:256, :256], big[:256, 256:], big[256:, :256], big[256:, 256:] = kids
    expected = big.reshape(256, 2, 256, 2, 3).mean(axis=(1, 3))
    assert np.abs(parent.astype(float) - expected).max() <= 0.5 + 1e-9
    # a missing child counts as nodata (zeros)
    parent = im.downsample_children(kids[0], None, None, None)
    assert parent[:128, :128].mean() == pytest.approx(kids[0].reshape(128, 2, 128, 2, 3).mean(axis=(1, 3)).mean(), abs=0.5)
    assert parent[:, 128:].max() == 0


def _spot_dataset(path: Path, bounds, size=512, spot=(300, 200)):
    import rasterio
    from rasterio.transform import from_bounds

    data = np.full((3, size, size), 40, dtype=np.uint8)
    r, c = spot
    data[:, r - 8:r + 8, c - 8:c + 8] = 250
    west, south, east, north = bounds
    with rasterio.open(path, "w", driver="GTiff", width=size, height=size, count=3, dtype="uint8", crs="EPSG:4326",
                       transform=from_bounds(west, south, east, north, size, size)) as dst:
        dst.write(data)
    return rasterio.open(path)


def _centroid(tile: np.ndarray) -> tuple[float, float]:
    mask = tile[..., 0] > 150
    rows, cols = np.nonzero(mask)
    return rows.mean(), cols.mean()


def test_naip_shift_applied(tmp_path):
    tile_bounds = im.tile_bounds(14, 3000, 4000)  # (south, north, west, east)
    south, north, west, east = tile_bounds
    # the source covers exactly this tile, 2 source pixels per tile pixel
    ds = _spot_dataset(tmp_path / "spot.tif", (west, south, east, north))
    px = (east - west) / im.TILE_PX
    base, _ = im.render_tile([ds], tile_bounds, (0.0, 0.0))
    shifted, _ = im.render_tile([ds], tile_bounds, (3.0 * px, -2.0 * px))  # +3 px north, -2 px west (dlat, dlon)
    r0, c0 = _centroid(base)
    r1, c1 = _centroid(shifted)
    # content labelled NAD83 moves by the ITRF-minus-NAD83 shift: north is up (rows decrease), west is left
    assert r1 - r0 == pytest.approx(-3.0, abs=0.3)
    assert c1 - c0 == pytest.approx(-2.0, abs=0.3)


def test_naip_shift_matches_task3_function():
    south, north, west, east = et.area_bounds(25.99686, -97.15394, 5500.0)
    dlat, dlon = im.naip_shift_deg((south, north, west, east))
    ref = et.nad83_horizontal_shift_deg(25.99686, -97.15394)
    assert (dlat, dlon) == pytest.approx(ref, abs=1e-9)
    assert 0.5 < abs(et.horizontal_shift_m(25.99686, -97.15394)[1]) + abs(et.horizontal_shift_m(25.99686, -97.15394)[0]) < 3.0


def test_choose_naip_year_prefers_newest_covering():
    box = (10.0, 11.0, 20.0, 21.0)  # south, north, west, east
    full = lambda y: {"year": y, "bbox": (20.0, 10.0, 21.0, 11.0)}  # noqa: E731
    part = lambda y: {"year": y, "bbox": (20.0, 10.0, 20.4, 11.0)}  # noqa: E731
    assert im.choose_naip_year([full(2020), full(2022), part(2024)], box) == 2022
    assert im.choose_naip_year([part(2020), part(2024)], box) == 2024  # nothing covers: the newest of the best
