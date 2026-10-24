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
    """NAIP-style per-channel fit recovers a synthetic per-channel gain / offset."""
    rng = np.random.default_rng(3)
    src = rng.uniform(20.0, 200.0, size=(500, 3))
    gain_true = np.array([1.3, 1.1, 0.9])
    offset_true = np.array([-10.0, 4.0, 12.0])
    ref = src * gain_true + offset_true + rng.normal(0.0, 0.3, size=src.shape)
    gain, offset = im.fit_gain_offset(src, ref)
    assert gain == pytest.approx(gain_true, rel=0.01)
    assert offset == pytest.approx(offset_true, abs=1.0)


def test_luminance_fit_gains_equal_across_channels():
    rng = np.random.default_rng(4)
    src = rng.uniform(20.0, 200.0, size=(500, 3))
    ref = np.clip(src[:, [0]] * 0 + (src @ np.array(im.LUMA_WEIGHTS))[:, None] * 0.8 - 12.0, 0, 255) + np.array([0.0, 0.0, 0.0])
    gain, offset = im.fit_gain_offset(src, ref, luminance=True)
    assert gain[0] == gain[1] == gain[2] and offset[0] == offset[1] == offset[2]
    assert gain[0] == pytest.approx(0.8, rel=0.01) and offset[0] == pytest.approx(-12.0, abs=1.0)


def test_fit_fallback_is_all_or_nothing():
    rng = np.random.default_rng(6)
    src = rng.uniform(20.0, 200.0, size=(300, 3))
    ref = src * 1.2 + 5.0
    ref[:, 2] = 90.0  # no variation in ONE channel: every channel falls back to a mean match with gain 1
    gain, offset = im.fit_gain_offset(src, ref)
    assert gain == pytest.approx([1.0, 1.0, 1.0])
    assert offset == pytest.approx(ref.mean(axis=0) - src.mean(axis=0))
    # a single channel with an implausible gain is an error naming the source, not a silent substitute
    ref = src * np.array([1.0, 1.0, 8.0])
    with pytest.raises(ValueError, match="s2_test"):
        im.fit_gain_offset(src, ref, name="s2_test")


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


def test_pyramid_identical_for_any_worker_count(tmp_path):
    import rasterio
    from rasterio.transform import from_bounds

    size = 0.03  # deg: ~ 5 x 5 tiles at z15 (0.0055 deg)
    west, south = -97.2, 26.0
    rng = np.random.default_rng(11)
    data = rng.integers(1, 256, size=(3, 600, 600), dtype=np.uint8)
    tif = tmp_path / "src.tif"
    with rasterio.open(tif, "w", driver="GTiff", width=600, height=600, count=3, dtype="uint8", crs="EPSG:4326",
                       transform=from_bounds(west, south, west + size, south + size, 600, 600)) as dst:
        dst.write(data)
    box = (south + 0.002, south + size - 0.002, west + 0.002, west + size - 0.002)
    gain, offset = np.array([0.8, 0.9, 1.0]), np.array([-5.0, 0.0, 3.0])
    counts = {}
    for workers in (1, 4):
        counts[workers] = im.write_pyramid([tif], box, (1e-5, 2e-5), gain, offset, 13, 15, tmp_path / f"w{workers}",
                                           log=lambda *_: None, workers=workers)
    assert counts[1] == counts[4] and counts[1][15] > 4
    files1 = sorted(p.relative_to(tmp_path / "w1") for p in (tmp_path / "w1").rglob("*.jpg"))
    files4 = sorted(p.relative_to(tmp_path / "w4") for p in (tmp_path / "w4").rglob("*.jpg"))
    assert files1 == files4 and len(files1) == sum(counts[1].values())
    for f in files1:
        assert (tmp_path / "w1" / f).read_bytes() == (tmp_path / "w4" / f).read_bytes()


def test_resolve_workers_override_and_memory_cap():
    msgs = []
    assert im.resolve_workers(3, 1, msgs.append) == 3
    mem = im.available_memory_bytes()
    if mem:
        capped = im.resolve_workers(10_000, mem // 4, msgs.append, "t")
        assert capped <= 3 and any("memory cap" in m for m in msgs)
    assert im.resolve_workers(None, 1, msgs.append) >= 1


def test_s2_block_worker_median_from_local_scenes(tmp_path):
    """The composite block worker (scene windows, SCL at half resolution, median) on synthetic local rasters."""
    import rasterio
    from rasterio.transform import from_origin

    def scene(name, value, scl_value, row_off):
        vis, scl = tmp_path / f"{name}_vis.tif", tmp_path / f"{name}_scl.tif"
        with rasterio.open(vis, "w", driver="GTiff", width=64, height=64, count=3, dtype="uint8", crs="EPSG:32614",
                           transform=from_origin(500000, 2900000, 10, 10)) as d:
            d.write(np.full((3, 64, 64), value, dtype=np.uint8))
        with rasterio.open(scl, "w", driver="GTiff", width=32, height=32, count=1, dtype="uint8", crs="EPSG:32614",
                           transform=from_origin(500000, 2900000, 20, 20)) as d:
            d.write(np.full((1, 32, 32), scl_value, dtype=np.uint8))
        return {"id": name, "visual": str(vis), "scl": str(scl), "row_off": row_off, "col_off": 0, "width": 64, "height": 64}

    scenes = [scene("a", 100, 4, 0), scene("b", 120, 4, 0), scene("c", 250, 9, 0)]  # c is cloud
    r0, c0, out = im._s2_block((scenes, 0, 0, 64, 64))
    assert (r0, c0) == (0, 0) and out.shape == (64, 64, 3)
    assert (out == 110).all()
    # a block entirely outside every scene
    assert im._s2_block((scenes, 640, 0, 64, 64))[2] is None
