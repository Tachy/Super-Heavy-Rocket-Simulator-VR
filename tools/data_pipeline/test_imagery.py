"""Unit tests of the imagery pipeline (tile scheme, median composite, colour match, pyramid, NAIP datum shift).
Synthetic rasters only, no network.

Run: python -m pytest tools/data_pipeline/test_imagery.py
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent))
import earth_albedo as ea  # noqa: E402
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


def test_median_keeps_uint16():
    rgb = np.zeros((3, 1, 1, 3), dtype=np.uint16)
    rgb[0], rgb[1], rgb[2] = 2600, 3000, 9000
    scl = np.full((3, 1, 1), 4, dtype=np.uint8)
    out = im.clear_median(rgb, scl)
    assert out.dtype == np.uint16 and out[0, 0, 0] == 3000


def test_tci_decode_is_linear_reflectance():
    """TCI = floor(1 + DN x 254 / 2500): count t decodes to the centre of its reflectance bin; 0 stays no data."""
    dn = np.arange(0, 2500, 7, dtype=np.float64)
    t = np.floor(1 + dn * 254.0 / 2500.0)
    rho = im.tci_to_reflectance(t)
    assert np.abs(rho - dn * 1e-4).max() <= 0.5 * 2500.0 / 254.0 * 1e-4 + 1e-12
    assert im.tci_to_reflectance(np.array([0]))[0] == 0.0
    assert im.tci_to_reflectance(np.array([255]))[0] == pytest.approx(0.2505, abs=1e-3)  # saturated: >= 0.25


def test_to_reflectance_scale_and_naip_table(tmp_path):
    raw = np.array([[[1000, 2000, 3000]]], dtype=np.uint16)
    assert im.to_reflectance(raw, {"kind": "scale", "scale": 1e-4})[0, 0].tolist() == pytest.approx([0.1, 0.2, 0.3])
    luts = np.tile(np.linspace(0.0, 0.5, 256), (3, 1))
    ratio = np.full((4, 4, 3), 2.0, dtype=np.float32)
    path = tmp_path / "gain.npy"
    np.save(path, ratio)
    transfer = {"kind": "naip", "luts": luts, "ratio": {"path": str(path), "box": (0.0, 1.0, 0.0, 1.0)}}
    counts = np.array([[[255, 0, 51]]], dtype=np.uint8)
    out = im.to_reflectance(counts, transfer, np.array([[0.5]]), np.array([[0.5]]))
    assert out[0, 0].tolist() == pytest.approx([1.0, 0.0, 0.2])


def test_local_ratio_follows_reference_means():
    """The sliding-window gain makes the source's local means follow the reference: a bright flat (reference twice the
    source over one half) keeps its brightness instead of being pulled to the box mean."""
    src = np.full((200, 200, 3), 0.1)
    ref = src.copy()
    ref[:, :100] = 0.2
    valid = np.ones((200, 200), dtype=bool)
    ratio = im.local_ratio(ref, src, valid, sigma_px=5.0)
    assert ratio[:, :80].mean() == pytest.approx(2.0, rel=1e-3)
    assert ratio[:, 120:].mean() == pytest.approx(1.0, rel=1e-3)
    # cells without data inside the window do not bias it; without any data it is 1; the gain is clipped
    valid[:, :] = False
    assert (im.local_ratio(ref, src, valid, sigma_px=5.0) == 1.0).all()
    assert im.local_ratio(ref * 10, src, np.ones((200, 200), dtype=bool), 5.0).max() == im.NAIP_RATIO_LIMITS[1]


def test_sample_ratio_bilinear_on_cell_centres(tmp_path):
    grid = np.zeros((2, 2, 3), dtype=np.float32)
    grid[:, 1] = 1.0  # east column 1
    path = tmp_path / "r.npy"
    np.save(path, grid)
    r = {"path": str(path), "box": (0.0, 2.0, 0.0, 2.0)}
    assert im.sample_ratio(r, np.array([1.0]), np.array([1.0]))[0, 0] == pytest.approx(0.5)  # between the centres
    assert im.sample_ratio(r, np.array([1.0]), np.array([1.8]))[0, 0] == pytest.approx(1.0)  # clamped at the edge


def test_downsample_pyramid():
    rng = np.random.default_rng(5)
    kids = [rng.integers(0, 256, size=(256, 256, 3), dtype=np.uint8) for _ in range(4)]  # tl, tr, bl, br
    parent = im.downsample_children(*kids)
    assert parent.shape == (256, 256, 3) and parent.dtype == np.uint8
    big = np.zeros((512, 512, 3), dtype=np.uint8)
    big[:256, :256], big[:256, 256:], big[256:, :256], big[256:, 256:] = kids
    valid = big.any(axis=-1)[..., None]
    lin = (ea.decode_srgb(big) * valid).reshape(256, 2, 256, 2, 3).sum(axis=(1, 3))
    n = valid.reshape(256, 2, 256, 2, 1).sum(axis=(1, 3))
    expected = ea.encode_srgb(lin / np.maximum(n, 1))  # the mean in reflectance (linear), encoded again
    assert np.abs(parent.astype(int) - np.maximum(expected, 1).astype(int)).max() <= 1
    # a missing child counts as nodata (zeros)
    parent = im.downsample_children(kids[0], None, None, None)
    assert (parent[:128, :128] > 0).all()
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
    transfer = {"kind": "scale", "scale": 1.0 / 300.0}
    counts = {}
    for workers in (1, 4):
        counts[workers] = im.write_pyramid([tif], box, (1e-5, 2e-5), transfer, 13, 15, tmp_path / f"w{workers}",
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


def test_s2_reflectance_decodes_tci_and_recomputes_saturated_pixels(tmp_path):
    """reflectance.tif: the TCI composite decoded; TCI-saturated pixels from the median of the reflectance bands."""
    import rasterio
    from rasterio.transform import from_origin

    def write(path, data, res, dtype):
        with rasterio.open(path, "w", driver="GTiff", width=data.shape[2], height=data.shape[1], count=data.shape[0],
                           dtype=dtype, crs="EPSG:32614", transform=from_origin(500000, 2900000, res, res)) as d:
            d.write(data)

    items = []
    for name, dn, scl in (("a", 3000, 4), ("b", 3400, 4), ("c", 9000, 9)):  # c is cloud
        d = tmp_path / name
        d.mkdir()
        write(d / "TCI.tif", np.full((3, 64, 64), 255, dtype=np.uint8), 10, "uint8")
        for band in im.S2_REFLECTANCE_BANDS:
            write(d / f"{band}.tif", np.full((1, 64, 64), dn, dtype=np.uint16), 10, "uint16")
        write(d / "SCL.tif", np.full((1, 32, 32), scl, dtype=np.uint8), 20, "uint8")
        items.append({"id": name, "visual": str(d / "TCI.tif"), "scl": str(d / "SCL.tif")})
    comp = np.full((3, 64, 64), 101, dtype=np.uint8)
    comp[:, 10:12, 20:22] = 255  # saturated in the TCI median
    comp[:, 40, 40] = 0  # no data
    write(tmp_path / "composite.tif", comp, 10, "uint8")
    path, saturated = im.build_s2_reflectance(items, tmp_path / "composite.tif", tmp_path, False, lambda *_: None, 1)
    assert saturated == 4
    with rasterio.open(path) as ds:
        out = ds.read()
    assert out[0, 0, 0] == round((101 - 0.5) * im.S2_TCI_REFLECTANCE_PER_COUNT / im.S2_REFLECTANCE_SCALE)
    assert (out[:, 10:12, 20:22] == 3200).all()  # median of the two clear scenes, not 0.25
    assert (out[:, 40, 40] == 0).all()


def test_downsample_never_averages_nodata():
    child = np.full((256, 256, 3), 100, dtype=np.uint8)
    child[0, 0] = 0  # one no-data pixel of the 2 x 2 block of parent pixel (0, 0)
    child[2:4, 2:4] = 0  # a whole 2 x 2 block of no data -> parent pixel (1, 1) stays no data
    parent = im.downsample_children(child, child, child, child)
    assert parent[0, 0].tolist() == [100, 100, 100]
    assert parent[1, 1].tolist() == [0, 0, 0]


def _write_raster(path: Path, data: np.ndarray, bounds) -> Path:
    import rasterio
    from rasterio.transform import from_bounds

    west, south, east, north = bounds
    with rasterio.open(path, "w", driver="GTiff", width=data.shape[2], height=data.shape[1], count=3, dtype="uint8",
                       crs="EPSG:4326", transform=from_bounds(west, south, east, north, data.shape[2], data.shape[1])) as d:
        d.write(data)
    return path


def _tile_pixels_inside(root: Path, z: int, box):
    """{(x, y): (decoded rgb or None when missing, inside-box mask)} of the tiles of the box at level z."""
    south, north, west, east = box
    out = {}
    for x, y in im.tiles_for_box(z, south, north, west, east):
        rgb = im.decode_jpeg(im.tile_path(root, z, x, y))
        ts, tn, tw, te = im.tile_bounds(z, x, y)
        c = (np.arange(im.TILE_PX) + 0.5) / im.TILE_PX
        lon_g, lat_g = np.meshgrid(tw + c * (te - tw), tn - c * (tn - ts))
        out[(x, y)] = (rgb, (lon_g > west) & (lon_g < east) & (lat_g > south) & (lat_g < north))
    return out


@pytest.mark.parametrize("with_reference", [True, False])
def test_pyramid_fill_leaves_no_nodata_inside_coverage(tmp_path, with_reference):
    """No data inside the coverage box is filled before writing (NAIP: from the fitted Sentinel-2 reference, Sentinel-2:
    from the day texture), every tile of the box is written and no level holds an all-zero pixel inside the box."""
    size = 0.03
    west, south = -97.2, 26.0
    data = np.full((3, 600, 600), 120, dtype=np.uint8)
    data[:, :300, 300:] = 0  # the north-east quarter is no data (sea beyond the scenes)
    src = _write_raster(tmp_path / "src.tif", data, (west, south, west + size, south + size))
    box = (south + 0.002, south + size - 0.002, west + 0.002, west + size - 0.002)
    texture = np.zeros((180, 360, 3))
    texture[...] = (0.03, 0.06, 0.09)  # the day texture in reflectance
    fill = {"texture": im.texture_window(texture, box)}
    if with_reference:
        ref = _write_raster(tmp_path / "ref.tif", np.full((3, 100, 100), 100, dtype=np.uint8),
                            (west - 0.01, south - 0.01, west + size + 0.01, south + size + 0.01))
        fill["reference"] = ([ref], {"kind": "scale", "scale": 0.002})
    expected_fill = ea.encode_srgb((0.2, 0.2, 0.2) if with_reference else (0.03, 0.06, 0.09)).astype(float)
    messages = []
    counts = im.write_pyramid([src], box, (0.0, 0.0), {"kind": "scale", "scale": 0.001}, 13, 15, tmp_path / "out",
                              log=messages.append, workers=1, fill=fill)
    assert any("no-data pixels left inside the box: 0" in m for m in messages)  # counted before the JPEG encoding
    for z in (13, 14, 15):
        tiles = _tile_pixels_inside(tmp_path / "out", z, box)
        assert counts[z] == len(tiles)
        for (x, y), (rgb, inside) in tiles.items():
            assert rgb is not None, f"z{z} tile {x}/{y} missing"
            assert not (rgb[inside].astype(int).sum(axis=-1) <= 6).any(), f"no data inside the box in z{z} {x}/{y}"
    # the filled quarter carries the fill colour (the source itself is 120 grey)
    probe = (south + size - 0.006, south + size - 0.003, west + size - 0.006, west + size - 0.003)
    for rgb, inside in _tile_pixels_inside(tmp_path / "out", 15, probe).values():
        assert rgb[inside].mean(axis=0) == pytest.approx(expected_fill, abs=3.0)


def test_fill_nodata_takes_the_sea_from_the_global_level_only():
    """Sea pixels (water rule) are no data whatever the imagery and the reference show: they read the sea texture, land
    pixels keep the imagery."""
    box = (0.0, 1.0, 0.0, 1.0)
    n = 4
    rho = np.full((n, n, 3), 0.2)
    valid = np.ones((n, n), dtype=bool)
    water = np.zeros((n, n), dtype=bool)
    water[:, :2] = True
    land_tex = np.full((8, 8, 3), 0.3)
    sea_tex = np.full((8, 8, 3), 0.02)
    out, ok = im.fill_nodata(rho, valid, box, None, im.texture_window(land_tex, box), water, im.texture_window(sea_tex, box))
    assert ok.all()
    assert out[:, :2] == pytest.approx(0.02)
    assert out[:, 2:] == pytest.approx(0.2)


def test_fill_nodata_keeps_the_brightness_above_the_open_water_pedestal():
    """Sea pixels read the sea texture plus the imagery's excess over the open-water pedestal (shallow bottoms, surf);
    imagery at or below the pedestal is dropped."""
    box = (0.0, 1.0, 0.0, 1.0)
    rho = np.full((4, 4, 3), 0.05)
    rho[:, 1] = 0.15  # bright shallow column
    water = np.ones((4, 4), dtype=bool)
    tex = im.texture_window(np.full((8, 8, 3), 0.02), box)
    out, ok = im.fill_nodata(rho, np.ones((4, 4), dtype=bool), box, None, tex, water, tex, pedestal=np.array([0.06, 0.06, 0.06]))
    assert ok.all()
    assert out[:, 0] == pytest.approx(0.02)  # 0.05 is below the pedestal 0.06
    assert out[:, 1] == pytest.approx(0.02 + 0.09)
