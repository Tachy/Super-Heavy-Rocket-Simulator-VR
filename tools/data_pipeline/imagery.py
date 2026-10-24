"""Earth imagery tile pyramid (M8 Plan B): NAIP 0.6 m around the pads, an own Sentinel-2 L2A median composite at 10 m.

Sources (config/imagery.toml, one entry per product and area):
  naip              Microsoft Planetary Computer STAC, collection `naip` (USDA FSA, public domain), newest year covering the
                    box, 0.6 m RGB in NAD83 UTM: clipped to the box, then shifted into ITRF (earth_terrain.nad83_horizontal_
                    shift_deg: content labelled NAD83 lies that much further north / east in ITRF).
  sentinel2_median  Element84 Earth Search, `sentinel-2-l2a` (Copernicus Sentinel data 2025, free and open), scenes of 2025
                    with eo:cloud_cover < 20, up to 8 per MGRS tile; per-pixel median of the clear pixels (SCL classes
                    4, 5, 6, 7), WGS84 UTM (no datum shift), kept as a GeoTIFF composite in data/raw/imagery/<id>/.
Both are fitted per channel (gain, offset; least squares on a 1 km lattice) to data/textures/earth_day.jpg so that the imagery
meets the day texture without a colour step; the fit is stored in the index and applied when the tiles are written.

Output (EPSG:4326 TMS, level z has 2^(z+1) x 2^z tiles of 256 x 256 px, tile size 180 / 2^z deg, x counts east from 180 W,
y counts south from 90 N, pixel centres at (i + 0.5) / 256 of the tile):
  data/imagery/earth/<source id>/<z>/<x>/<y>.jpg     (JPEG quality 88 [E])
  data/imagery/earth/<source id>/meta.toml           per source, compiled into
  data/imagery/earth/index.toml                      id, product, year, min/max level, exact coverage, fade_m, gain/offset
Lower levels are 2 x 2 box means of the 4 decoded children. Tiles entirely outside the box are not written.

Usage: python tools/data_pipeline/fetch.py [--force] imagery
"""

from __future__ import annotations

import io
import json
import math
import os
import shutil
import time
import tomllib
import urllib.request
import warnings
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

import earth_terrain as et

REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_FILE = REPO_ROOT / "config" / "imagery.toml"

TILE_PX = 256
JPEG_QUALITY = 88  # [E]
NAIP_MARGIN_M = 500.0  # [E] local box margin: the NAIP tiles cover the 3DEP plateau area plus this ring
FADE_M = {"naip": 500.0, "sentinel2_median": 5000.0}  # [E] blend width at the coverage edge
SCL_CLEAR = (4, 5, 6, 7)  # [S] ESA L2A scene classification: vegetation, not vegetated, water, unclassified
S2_MAX_CLOUD_PERCENT = 20  # [E]
S2_SCENES_PER_TILE = 8  # [E]
S2_YEAR = 2025
S2_PIXEL_M = 10.0  # [S] Sentinel-2 visual (TCI) band spacing
S2_BLOCK_PX = 1024  # composite block (read + median + write unit); multiple of the GTiff block of 512
LATTICE_M = 1000.0  # [E] colour-fit lattice spacing
MIN_REF_STD = 3.0  # 8-bit counts: below this the reference carries no gain information (12 km day texture over a small box)
GAIN_LIMITS = (0.5, 2.0)  # [E] sanity bounds of the fitted gain
NAIP_STAC = "https://planetarycomputer.microsoft.com/api/stac/v1"
NAIP_TOKEN_URL = "https://planetarycomputer.microsoft.com/api/sas/v1/token/naip"
S2_STAC = "https://earth-search.aws.element84.com/v1"
M_PER_DEG_LAT = 111_320.0  # only for lattice sizing
READ_RETRIES = 4

Box = tuple[float, float, float, float]  # south, north, west, east [deg]


# ----------------------------------------------------------------------------------------------- tile scheme

def tile_bounds(z: int, x: int, y: int) -> Box:
    """(south, north, west, east) of tile (z, x, y)."""
    size = 180.0 / 2**z
    return 90.0 - (y + 1) * size, 90.0 - y * size, -180.0 + x * size, -180.0 + (x + 1) * size


def tiles_for_box(z: int, south: float, north: float, west: float, east: float) -> list[tuple[int, int]]:
    """(x, y) of exactly the tiles whose interior intersects the box, row-major from the north-west."""
    size = 180.0 / 2**z
    eps = 1e-9
    x0, x1 = math.floor((west + 180.0) / size + eps), math.ceil((east + 180.0) / size - eps) - 1
    y0, y1 = math.floor((90.0 - north) / size + eps), math.ceil((90.0 - south) / size - eps) - 1
    x0, y0 = max(x0, 0), max(y0, 0)
    x1, y1 = min(x1, 2 ** (z + 1) - 1), min(y1, 2**z - 1)
    return [(x, y) for y in range(y0, y1 + 1) for x in range(x0, x1 + 1)]


def tile_path(root: Path, z: int, x: int, y: int) -> Path:
    return root / str(z) / str(x) / f"{y}.jpg"


# ----------------------------------------------------------------------------------------------- pixel operations

def clear_median(rgb: np.ndarray, scl: np.ndarray) -> np.ndarray:
    """Per-pixel, per-channel median over the scenes whose SCL class is clear. rgb (n, h, w, 3) uint8 (0 = no data), scl
    (n, h, w) uint8. A pixel without any clear scene takes the median of its valid (non-zero) scenes instead of a hole;
    without valid scenes it stays 0."""
    valid = rgb.any(axis=-1)
    clear = valid & np.isin(scl, SCL_CLEAR)
    use = np.where(clear.any(axis=0)[None], clear, valid)
    stack = np.where(use[..., None], rgb.astype(np.float32), np.nan)
    stack.sort(axis=0)  # NaN last
    count = use.sum(axis=0)
    lo = np.maximum(count - 1, 0) // 2
    hi = count // 2
    idx_lo = np.broadcast_to(lo[None, ..., None], (1,) + lo.shape + (3,))
    idx_hi = np.broadcast_to(hi[None, ..., None], (1,) + hi.shape + (3,))
    median = 0.5 * (np.take_along_axis(stack, idx_lo, axis=0)[0] + np.take_along_axis(stack, idx_hi, axis=0)[0])
    median[count == 0] = 0.0
    return np.rint(np.nan_to_num(median)).astype(np.uint8)


def fit_gain_offset(src: np.ndarray, ref: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Per-channel least squares ref ~ gain * src + offset on (N, 3) samples. A reference without variation
    (std < MIN_REF_STD) or a gain outside GAIN_LIMITS falls back to a mean match (gain 1)."""
    gain, offset = np.ones(3), np.zeros(3)
    for c in range(3):
        s, r = src[:, c].astype(np.float64), ref[:, c].astype(np.float64)
        g = 1.0
        if r.std() >= MIN_REF_STD and s.std() > 1e-9:
            a = np.stack([s, np.ones_like(s)], axis=1)
            g = float(np.linalg.lstsq(a, r, rcond=None)[0][0])
            if not GAIN_LIMITS[0] <= g <= GAIN_LIMITS[1]:
                g = 1.0  # the texture does not resolve this box: a mean match is safer than a clamped extreme
        gain[c] = g
        offset[c] = float(r.mean() - g * s.mean())
    return gain, offset


def apply_gain_offset(rgb: np.ndarray, gain: np.ndarray, offset: np.ndarray) -> np.ndarray:
    """gain * rgb + offset clipped to 8 bit; no-data pixels (all channels 0) stay 0."""
    out = np.clip(np.rint(rgb.astype(np.float64) * gain + offset), 1, 255).astype(np.uint8)  # 1: only no data is 0
    out[~rgb.any(axis=-1)] = 0
    return out


def downsample_children(tl, tr, bl, br) -> np.ndarray:
    """The parent tile: 2 x 2 box mean of the four (256, 256, 3) children; a missing child (None) is no data (zeros)."""
    zero = np.zeros((TILE_PX, TILE_PX, 3), dtype=np.uint8)
    rows = [np.concatenate([c if c is not None else zero for c in row], axis=1) for row in ((tl, tr), (bl, br))]
    big = np.concatenate(rows, axis=0).astype(np.float64)
    mean = big.reshape(TILE_PX, 2, TILE_PX, 2, 3).mean(axis=(1, 3))
    return np.rint(mean).astype(np.uint8)


# ----------------------------------------------------------------------------------------------- rendering

def naip_shift_deg(box: Box) -> tuple[float, float]:
    """(dlat, dlon) of ITRF minus NAD83 at the box centre."""
    south, north, west, east = box
    return et.nad83_horizontal_shift_deg((south + north) / 2.0, (west + east) / 2.0)


def render_box(datasets, box: Box, shift_deg: tuple[float, float], width: int, height: int,
               resampling=None) -> tuple[np.ndarray, np.ndarray]:
    """Reproject the first three bands of every dataset into the geographic box (width x height pixels, edges at the box
    bounds) -> (rgb (h, w, 3) uint8, valid (h, w)). `shift_deg` = (dlat, dlon) moves the content north / east (datum
    shift); a source value of 0 is no data."""
    import rasterio
    from rasterio.transform import from_bounds
    from rasterio.warp import Resampling, reproject, transform_bounds

    resampling = resampling or Resampling.bilinear
    south, north, west, east = box
    dlat, dlon = shift_deg
    # destination grid labelled in the source datum: the pixel at ITRF position P shows the source at P - shift
    dst_transform = from_bounds(west - dlon, south - dlat, east - dlon, north - dlat, width, height)
    out = np.zeros((3, height, width), dtype=np.uint8)
    for ds in datasets:
        b = transform_bounds(ds.crs, "EPSG:4326", *ds.bounds)
        if b[2] <= west - dlon or b[0] >= east - dlon or b[3] <= south - dlat or b[1] >= north - dlat:
            continue
        reproject(source=rasterio.band(ds, [1, 2, 3]), destination=out, dst_transform=dst_transform,
                  dst_crs="EPSG:4326", resampling=resampling, src_nodata=0, dst_nodata=0, init_dest_nodata=False)
    rgb = np.moveaxis(out, 0, -1)
    return np.ascontiguousarray(rgb), rgb.any(axis=-1)


def render_tile(datasets, bounds: Box, shift_deg: tuple[float, float]) -> tuple[np.ndarray, np.ndarray]:
    return render_box(datasets, bounds, shift_deg, TILE_PX, TILE_PX)


def encode_jpeg(rgb: np.ndarray) -> bytes:
    from PIL import Image

    buf = io.BytesIO()
    Image.fromarray(rgb, "RGB").save(buf, "JPEG", quality=JPEG_QUALITY)
    return buf.getvalue()


def decode_jpeg(path: Path) -> np.ndarray | None:
    from PIL import Image

    if not path.exists():
        return None
    with Image.open(path) as img:
        return np.asarray(img.convert("RGB"))


def write_tile(root: Path, z: int, x: int, y: int, rgb: np.ndarray) -> int:
    path = tile_path(root, z, x, y)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = encode_jpeg(rgb)
    path.write_bytes(data)
    return len(data)


# ----------------------------------------------------------------------------------------------- colour match

def load_day_texture(path: Path) -> np.ndarray:
    from PIL import Image

    with Image.open(path) as img:
        return np.asarray(img.convert("RGB"), dtype=np.float64)


def sample_day_texture(texture: np.ndarray, lats: np.ndarray, lons: np.ndarray) -> np.ndarray:
    """Bilinear samples (N, 3) of the equirectangular day texture (u = (lon + 180) / 360, v = (90 - lat) / 180)."""
    h, w, _ = texture.shape
    fx = ((np.asarray(lons) + 180.0) / 360.0 * w - 0.5) % w
    fy = np.clip((90.0 - np.asarray(lats)) / 180.0 * h - 0.5, 0.0, h - 1.0)
    x0 = np.floor(fx).astype(int)
    y0 = np.minimum(np.floor(fy).astype(int), h - 2)
    wx, wy = (fx - x0)[:, None], (fy - y0)[:, None]
    x1 = (x0 + 1) % w
    top = (1 - wx) * texture[y0, x0] + wx * texture[y0, x1]
    bottom = (1 - wx) * texture[y0 + 1, x0] + wx * texture[y0 + 1, x1]
    return (1 - wy) * top + wy * bottom


def fit_to_reference(datasets, box: Box, shift_deg, texture: np.ndarray, reference=None) -> tuple[np.ndarray, np.ndarray]:
    """gain/offset of the imagery (averaged on a 1 km lattice over the box) against the day texture; or, where the 12 km
    texture cannot resolve the box (NAIP), against `reference` = (datasets, gain, offset) of an already fitted source
    (the Sentinel-2 composite of the same area, which itself is matched to the texture)."""
    from rasterio.warp import Resampling

    south, north, west, east = box
    lat_c = (south + north) / 2.0
    ny = max(2, round((north - south) * M_PER_DEG_LAT / LATTICE_M))
    nx = max(2, round((east - west) * M_PER_DEG_LAT * math.cos(math.radians(lat_c)) / LATTICE_M))
    rgb, valid = render_box(datasets, box, shift_deg, nx, ny, Resampling.average)
    if reference is None:
        lats = north - (np.arange(ny) + 0.5) * (north - south) / ny
        lons = west + (np.arange(nx) + 0.5) * (east - west) / nx
        lon_g, lat_g = np.meshgrid(lons, lats)
        ref = sample_day_texture(texture, lat_g[valid], lon_g[valid])
    else:
        ref_datasets, ref_gain, ref_offset = reference
        ref_rgb, ref_valid = render_box(ref_datasets, box, (0.0, 0.0), nx, ny, Resampling.average)
        valid = valid & ref_valid
        ref = apply_gain_offset(ref_rgb, ref_gain, ref_offset)[valid].astype(np.float64)
    src = rgb[valid].astype(np.float64)
    if len(src) < 4:
        raise RuntimeError("Keine gueltigen Bildpunkte fuer den Farbabgleich")
    return fit_gain_offset(src, ref)


# ----------------------------------------------------------------------------------------------- pyramid

def write_pyramid(datasets, box: Box, shift_deg, gain, offset, min_level: int, max_level: int, root: Path,
                  log=print) -> dict[int, int]:
    """Finest level rendered from the datasets (colour fit applied), lower levels as box means of the decoded children.
    Returns {level: tile count}."""
    counts: dict[int, int] = {}
    south, north, west, east = box
    tiles = tiles_for_box(max_level, south, north, west, east)
    n = 0
    for i, (x, y) in enumerate(tiles):
        rgb, valid = render_tile(datasets, tile_bounds(max_level, x, y), shift_deg)
        if not valid.any():
            continue
        write_tile(root, max_level, x, y, apply_gain_offset(rgb, gain, offset))
        n += 1
        if i % 500 == 0:
            log(f"    z{max_level}: {i}/{len(tiles)}")
    counts[max_level] = n
    for z in range(max_level - 1, min_level - 1, -1):
        n = 0
        for x, y in tiles_for_box(z, south, north, west, east):
            kids = [decode_jpeg(tile_path(root, z + 1, 2 * x + dx, 2 * y + dy)) for dy in (0, 1) for dx in (0, 1)]
            if all(k is None for k in kids):
                continue
            write_tile(root, z, x, y, downsample_children(*kids))
            n += 1
        counts[z] = n
        log(f"    z{z}: {n} tiles")
    return counts


# ----------------------------------------------------------------------------------------------- network helpers

def _retry(fn, what: str, log=print):
    for attempt in range(READ_RETRIES):
        try:
            return fn()
        except Exception as err:  # network and GDAL errors vary in type
            if attempt == READ_RETRIES - 1:
                raise
            log(f"    {what}: {err!r}, retry {attempt + 1}")
            time.sleep(5.0 * (attempt + 1))


def _http_json(url: str) -> dict:
    request = urllib.request.Request(url, headers={"User-Agent": "starsim-data-pipeline"})
    with urllib.request.urlopen(request, timeout=60) as response:
        return json.load(response)


def _gdal_http_env() -> None:
    os.environ.setdefault("GDAL_HTTP_MAX_RETRY", "5")
    os.environ.setdefault("GDAL_HTTP_RETRY_DELAY", "3")
    os.environ.setdefault("CPL_VSIL_CURL_ALLOWED_EXTENSIONS", ".tif,.tiff")
    os.environ.setdefault("GDAL_DISABLE_READDIR_ON_OPEN", "EMPTY_DIR")
    os.environ.setdefault("VSI_CACHE", "TRUE")
    os.environ.setdefault("GDAL_HTTP_TIMEOUT", "120")


def choose_naip_year(items: list[dict], box: Box) -> int:
    """The newest year whose items (dicts with `year`, `bbox` = (west, south, east, north)) cover the whole box (lattice
    test); when none does, the newest among the years with the best coverage."""
    south, north, west, east = box
    lat = np.linspace(south, north, 25)
    lon = np.linspace(west, east, 25)
    lon_g, lat_g = np.meshgrid(lon, lat)
    best: dict[int, float] = {}
    for year in {i["year"] for i in items}:
        covered = np.zeros(lat_g.shape, dtype=bool)
        for item in (i for i in items if i["year"] == year):
            w, s, e, n = item["bbox"]
            covered |= (lon_g >= w) & (lon_g <= e) & (lat_g >= s) & (lat_g <= n)
        best[year] = float(covered.mean())
    top = max(best.values())
    return max(y for y, c in best.items() if c >= top - 1e-12)


# ----------------------------------------------------------------------------------------------- NAIP

def naip_clips(src_id: str, box: Box, raw_dir: Path, force: bool, log=print) -> tuple[list[Path], int, Box]:
    """Clip the newest covering NAIP year to the box (NAD83 UTM GeoTIFFs in raw_dir) -> (paths, year, covered box)."""
    import rasterio
    from pystac_client import Client
    from rasterio.transform import array_bounds
    from rasterio.warp import transform_bounds
    from rasterio.windows import Window, from_bounds

    _gdal_http_env()
    south, north, west, east = box
    items_file = raw_dir / "items.json"
    if items_file.exists() and not force:
        cached = json.loads(items_file.read_text())
    else:
        client = Client.open(NAIP_STAC)
        found = list(client.search(collections=["naip"], bbox=(west, south, east, north)).items())
        entries = [{"id": it.id, "year": int(it.properties.get("naip:year") or it.datetime.year),
                    "bbox": list(it.bbox), "href": it.assets["image"].href} for it in found]
        if not entries:
            raise RuntimeError(f"{src_id}: keine NAIP-Szenen ueber der Box")
        year = choose_naip_year(entries, box)
        cached = {"year": year, "items": [e for e in entries if e["year"] == year]}
        raw_dir.mkdir(parents=True, exist_ok=True)
        items_file.write_text(json.dumps(cached, indent=1))
    year, entries = cached["year"], cached["items"]
    log(f"  {src_id}: NAIP {year}, {len(entries)} scenes: {[e['id'] for e in entries]}")
    dlat, dlon = naip_shift_deg(box)
    paths: list[Path] = []
    covered = [90.0, -90.0, 180.0, -180.0]
    for e in entries:
        path = raw_dir / f"{e['id']}.tif"
        paths.append(path)
        w, s, ee, n = e["bbox"]
        covered = [min(covered[0], max(s, south)), max(covered[1], min(n, north)),
                   min(covered[2], max(w, west)), max(covered[3], min(ee, east))]
        if path.exists() and not force:
            continue

        def clip(e=e, path=path):
            token = _http_json(NAIP_TOKEN_URL)["token"]
            with rasterio.open(f"{e['href']}?{token}") as ds:
                b = transform_bounds("EPSG:4326", ds.crs, west - dlon, south - dlat, east - dlon, north - dlat,
                                     densify_pts=21)
                win = from_bounds(*b, transform=ds.transform).round_offsets(op="floor").round_lengths(op="ceil")
                win = win.intersection(Window(0, 0, ds.width, ds.height))
                data = ds.read([1, 2, 3], window=win)
                profile = dict(driver="GTiff", width=win.width, height=win.height, count=3, dtype="uint8", crs=ds.crs,
                               transform=ds.window_transform(win), tiled=True, blockxsize=512, blockysize=512,
                               compress="deflate", predictor=2, BIGTIFF="IF_SAFER")
            partial = path.with_suffix(".part")
            with rasterio.open(partial, "w", **profile) as out:
                out.write(data)
            partial.replace(path)

        log(f"    clipping {e['id']}")
        _retry(clip, e["id"], log)
    return paths, year, (covered[0], covered[1], covered[2], covered[3])


# ----------------------------------------------------------------------------------------------- Sentinel-2

def s2_items(box: Box, raw_dir: Path, force: bool, log=print) -> list[dict]:
    """Up to S2_SCENES_PER_TILE least cloudy 2025 scenes per MGRS tile (cached as items.json)."""
    from pystac_client import Client

    items_file = raw_dir / "items.json"
    if items_file.exists() and not force:
        return json.loads(items_file.read_text())
    south, north, west, east = box
    client = Client.open(S2_STAC)
    search = client.search(collections=["sentinel-2-l2a"], bbox=(west, south, east, north),
                           datetime=f"{S2_YEAR}-01-01/{S2_YEAR}-12-31",
                           query={"eo:cloud_cover": {"lt": S2_MAX_CLOUD_PERCENT}}, max_items=2000)
    by_tile: dict[str, dict[str, dict]] = {}
    for it in search.items():
        parts = it.id.split("_")
        tile, date = parts[1], parts[2]  # S2B_14RPQ_20250105_0_L2A
        entry = {"id": it.id, "tile": tile, "date": date, "cloud": float(it.properties["eo:cloud_cover"]),
                 "visual": it.assets["visual"].href, "scl": it.assets["scl"].href}
        old = by_tile.setdefault(tile, {}).get(date)
        if old is None or entry["cloud"] < old["cloud"]:
            by_tile[tile][date] = entry
    chosen = []
    for tile, dates in sorted(by_tile.items()):
        best = sorted(dates.values(), key=lambda e: e["cloud"])[:S2_SCENES_PER_TILE]
        log(f"    MGRS {tile}: {len(dates)} candidates, {len(best)} used (cloud {[round(b['cloud'], 1) for b in best]})")
        chosen.extend(best)
    if not chosen:
        raise RuntimeError("Keine Sentinel-2-Szenen gefunden")
    raw_dir.mkdir(parents=True, exist_ok=True)
    items_file.write_text(json.dumps(chosen, indent=1))
    return chosen


def build_s2_composite(items: list[dict], box: Box, raw_dir: Path, force: bool, log=print) -> Path:
    """The median composite over the box as a tiled deflate GeoTIFF in the dominant UTM zone (resumable by block)."""
    import rasterio
    from rasterio.transform import from_origin
    from rasterio.warp import transform_bounds
    from rasterio.windows import Window

    _gdal_http_env()
    final = raw_dir / "composite.tif"
    partial = raw_dir / "composite.partial.tif"
    progress_file = raw_dir / "composite.progress.json"
    if final.exists() and not force:
        return final
    if force:
        for f in (partial, progress_file):
            f.unlink(missing_ok=True)

    def probe(item):
        with rasterio.open(item["visual"]) as ds:
            return item, ds.crs.to_string(), ds.transform, ds.width, ds.height

    with ThreadPoolExecutor(8) as pool:
        infos = list(pool.map(lambda it: _retry(lambda: probe(it), it["id"], log), items))
    crs_names = [i[1] for i in infos]
    crs_name = max(set(crs_names), key=crs_names.count)
    infos = [i for i in infos if i[1] == crs_name]
    log(f"    composite in {crs_name}: {len(infos)} of {len(items)} scenes")
    south, north, west, east = box
    minx, miny, maxx, maxy = transform_bounds("EPSG:4326", crs_name, west, south, east, north, densify_pts=41)
    minx, miny = math.floor(minx / S2_PIXEL_M) * S2_PIXEL_M, math.floor(miny / S2_PIXEL_M) * S2_PIXEL_M
    maxx, maxy = math.ceil(maxx / S2_PIXEL_M) * S2_PIXEL_M, math.ceil(maxy / S2_PIXEL_M) * S2_PIXEL_M
    width, height = round((maxx - minx) / S2_PIXEL_M), round((maxy - miny) / S2_PIXEL_M)
    transform = from_origin(minx, maxy, S2_PIXEL_M, S2_PIXEL_M)
    offsets = []  # item pixel = composite pixel + (row_off, col_off)
    for item, _, t, w, h in infos:
        col_off, row_off = (minx - t.c) / S2_PIXEL_M, (t.f - maxy) / S2_PIXEL_M
        if abs(col_off - round(col_off)) > 1e-3 or abs(row_off - round(row_off)) > 1e-3:
            raise RuntimeError(f"{item['id']}: Szenengitter nicht auf dem 10-m-Gitter des Komposits")
        offsets.append((round(row_off), round(col_off), w, h))

    done: set[str] = set(json.loads(progress_file.read_text())) if progress_file.exists() and partial.exists() else set()
    if not partial.exists():
        done = set()
        raw_dir.mkdir(parents=True, exist_ok=True)
        with rasterio.open(partial, "w", driver="GTiff", width=width, height=height, count=3, dtype="uint8",
                           crs=crs_name, transform=transform, tiled=True, blockxsize=512, blockysize=512,
                           compress="deflate", predictor=2, BIGTIFF="YES"):
            pass

    def read_item(k, r0, c0, h, w):
        item = infos[k][0]
        row_off, col_off, iw, ih = offsets[k]
        ra, ca = max(r0 + row_off, 0), max(c0 + col_off, 0)
        rb, cb = min(r0 + row_off + h, ih), min(c0 + col_off + w, iw)
        if rb <= ra or cb <= ca:
            return None
        win = Window(ca, ra, cb - ca, rb - ra)
        scl_win = Window(ca / 2.0, ra / 2.0, (cb - ca) / 2.0, (rb - ra) / 2.0)
        from rasterio.enums import Resampling

        def go():
            with rasterio.open(item["visual"]) as v, rasterio.open(item["scl"]) as s:
                return (v.read([1, 2, 3], window=win),
                        s.read(1, window=scl_win, out_shape=(rb - ra, cb - ca), resampling=Resampling.nearest))

        rgb, scl = _retry(go, item["id"], log)
        return ra - row_off - r0, ca - col_off - c0, rgb, scl

    blocks = [(r, c) for r in range(0, height, S2_BLOCK_PX) for c in range(0, width, S2_BLOCK_PX)]
    t0 = time.time()
    with rasterio.open(partial, "r+") as dst, ThreadPoolExecutor(8) as pool:
        for b, (r0, c0) in enumerate(blocks):
            key = f"{r0},{c0}"
            if key in done:
                continue
            h, w = min(S2_BLOCK_PX, height - r0), min(S2_BLOCK_PX, width - c0)
            parts = [p for p in pool.map(lambda k: read_item(k, r0, c0, h, w), range(len(infos))) if p is not None]
            if parts:
                rgb = np.zeros((len(parts), h, w, 3), dtype=np.uint8)
                scl = np.zeros((len(parts), h, w), dtype=np.uint8)
                for n, (dr, dc, v, s) in enumerate(parts):
                    rgb[n, dr:dr + v.shape[1], dc:dc + v.shape[2]] = np.moveaxis(v, 0, -1)
                    scl[n, dr:dr + s.shape[0], dc:dc + s.shape[1]] = s
                dst.write(np.moveaxis(clear_median(rgb, scl), -1, 0), window=Window(c0, r0, w, h))
            done.add(key)
            progress_file.write_text(json.dumps(sorted(done)))
            log(f"    composite block {len(done)}/{len(blocks)} ({time.time() - t0:.0f} s)")
    partial.replace(final)
    progress_file.unlink(missing_ok=True)
    return final


# ----------------------------------------------------------------------------------------------- driver

def load_config(path: Path = CONFIG_FILE) -> dict:
    with path.open("rb") as f:
        return tomllib.load(f)


def source_box(src: dict, areas: dict[str, dict]) -> Box:
    area = areas[src["area"]]
    half = area["local_half_size_m"] + NAIP_MARGIN_M if src["box"] == "local" else area["regional_half_size_m"]
    return et.area_bounds(area["latitude_deg"], area["longitude_deg"], half)


def _fmt_list(values) -> str:
    return "[" + ", ".join(f"{v:.6f}" for v in values) + "]"


def meta_text(m: dict) -> str:
    lines = ["[[source]]", f'id = "{m["id"]}"', f'product = "{m["product"]}"', f'year = {m["year"]}',
             f'scenes = {m["scenes"]}', f'min_level = {m["min_level"]}', f'max_level = {m["max_level"]}',
             f'north_deg = {m["north_deg"]:.7f}', f'south_deg = {m["south_deg"]:.7f}',
             f'west_deg = {m["west_deg"]:.7f}', f'east_deg = {m["east_deg"]:.7f}', f'fade_m = {m["fade_m"]:.1f}',
             f'gain_rgb = {_fmt_list(m["gain_rgb"])}', f'offset_rgb = {_fmt_list(m["offset_rgb"])}',
             f'tile_count = {m["tile_count"]}', ""]
    return "\n".join(lines)


def write_index(out_dir: Path, source_ids: list[str]) -> None:
    """Compile the per-source meta.toml files (config order) into index.toml; the first lines are comments."""
    parts = ["# Earth imagery tile pyramids (tools/data_pipeline/imagery.py): EPSG:4326 TMS, 256 px JPEG tiles,",
             "# path <id>/<z>/<x>/<y>.jpg, tile size 180 / 2^z deg, x east from 180 W, y south from 90 N.", ""]
    for sid in source_ids:
        meta = out_dir / sid / "meta.toml"
        if meta.exists():
            parts.append(meta.read_text(encoding="utf-8"))
    (out_dir / "index.toml").write_text("\n".join(parts), encoding="utf-8")


def _dir_size(path: Path) -> int:
    return sum(f.stat().st_size for f in path.rglob("*") if f.is_file())


def s2_reference(sid: str, src: dict, areas: dict, repo_root: Path):
    """(composite dataset list, gain, offset) of the fitted Sentinel-2 source of the same area, None when not built yet."""
    import rasterio

    cfg = load_config(repo_root / "config" / "imagery.toml")
    for other in cfg["source"]:
        if other["product"] == "sentinel2_median" and other["area"] == src["area"]:
            meta = repo_root / "data" / "imagery" / "earth" / other["id"] / "meta.toml"
            composite = repo_root / "data" / "raw" / "imagery" / other["id"] / "composite.tif"
            if meta.exists() and composite.exists():
                m = tomllib.loads(meta.read_text(encoding="utf-8"))["source"][0]
                return [rasterio.open(composite)], np.array(m["gain_rgb"]), np.array(m["offset_rgb"])
    print(f"  {sid}: kein Sentinel-2-Abgleich verfuegbar, Farbabgleich gegen die Tagestextur")
    return None


def build_source(src: dict, areas: dict[str, dict], repo_root: Path, texture: np.ndarray, force: bool, log=print) -> dict:
    import rasterio

    sid, product = src["id"], src["product"]
    box = source_box(src, areas)
    raw_dir = repo_root / "data" / "raw" / "imagery" / sid
    root = repo_root / "data" / "imagery" / "earth" / sid
    if root.exists():
        shutil.rmtree(root)
    if product == "naip":
        paths, year, covered = naip_clips(sid, box, raw_dir, force, log)
        shift, scenes = naip_shift_deg(box), len(paths)
    elif product == "sentinel2_median":
        items = s2_items(box, raw_dir, force, log)
        paths, year, covered, shift, scenes = [build_s2_composite(items, box, raw_dir, force, log)], S2_YEAR, box, (0.0, 0.0), len(items)
    else:
        raise ValueError(f"unbekanntes Produkt {product}")
    datasets = [rasterio.open(p) for p in paths]
    try:
        reference = None
        if product == "naip":  # the day texture is ~12 km per pixel: far too coarse for an 11 km box
            reference = s2_reference(sid, src, areas, repo_root)
        gain, offset = fit_to_reference(datasets, covered, shift, texture, reference)
        log(f"  {sid}: gain {np.round(gain, 3).tolist()}, offset {np.round(offset, 2).tolist()}")
        counts = write_pyramid(datasets, covered, shift, gain, offset, src["min_level"], src["max_level"], root, log)
    finally:
        for ds in datasets:
            ds.close()
    south, north, west, east = covered
    meta = {"id": sid, "product": product, "year": year, "scenes": scenes, "min_level": src["min_level"],
            "max_level": src["max_level"], "north_deg": north, "south_deg": south, "west_deg": west, "east_deg": east,
            "fade_m": FADE_M[product], "gain_rgb": gain, "offset_rgb": offset, "tile_count": sum(counts.values())}
    (root / "meta.toml").write_text(meta_text(meta), encoding="utf-8")
    log(f"  {sid}: {sum(counts.values())} tiles {counts}, {_dir_size(root) / 1e6:.1f} MB")
    return meta


def fetch(source: dict, repo_root: Path, download, force: bool = False) -> None:
    cfg = load_config(repo_root / "config" / "imagery.toml")
    areas = {a["id"]: a for a in et.load_config(repo_root / "config" / "earth_terrain.toml")["area"]}
    texture = load_day_texture(repo_root / "data" / "textures" / "earth_day.jpg")
    out_dir = repo_root / "data" / "imagery" / "earth"
    out_dir.mkdir(parents=True, exist_ok=True)
    warnings.filterwarnings("ignore", category=RuntimeWarning)
    # Sentinel-2 first: the NAIP colour fit uses the fitted composite of its area as the reference
    for src in sorted(cfg["source"], key=lambda s: s["product"] == "naip"):
        if (out_dir / src["id"] / "meta.toml").exists() and not force:
            print(f"  {src['id']}: already present")
            continue
        print(f"  {src['id']}: {src['product']} over {src['area']} ({src['box']} box)", flush=True)
        build_source(src, areas, repo_root, texture, force, log=lambda s: print(s, flush=True))
        write_index(out_dir, [s["id"] for s in cfg["source"]])
    write_index(out_dir, [s["id"] for s in cfg["source"]])
