"""Earth imagery tile pyramid (M8 Plan B): NAIP 0.6 m around the pads, an own Sentinel-2 L2A median composite at 10 m.

Sources (config/imagery.toml, one entry per product and area):
  naip              Microsoft Planetary Computer STAC, collection `naip` (USDA FSA, public domain), newest year covering the
                    box, 0.6 m RGB in NAD83 UTM: clipped to the box, then shifted into ITRF (earth_terrain.nad83_horizontal_
                    shift_deg: content labelled NAD83 lies that much further north / east in ITRF).
  sentinel2_median  Element84 Earth Search, `sentinel-2-l2a` (Copernicus Sentinel data 2025, free and open), scenes of 2025
                    with eo:cloud_cover < 20, up to 8 per MGRS tile; per-pixel median of the clear pixels (SCL classes
                    4, 5, 6, 7), WGS84 UTM (no datum shift), kept as a GeoTIFF composite in data/raw/imagery/<id>/.
Every tile holds surface reflectance, sRGB-encoded (earth_albedo.py: the one transfer of all Earth imagery):
  Sentinel-2  L2A surface reflectance without display stretch: the median of the visual (TCI) bands decoded with their
              linear quantisation (`S2_TCI_REFLECTANCE_PER_COUNT`); pixels saturated in the TCI (reflectance >= 0.25:
              beaches, flats, roofs) are recomputed from the B04 / B03 / B02 reflectance bands (`build_s2_reflectance`,
              reflectance.tif next to the composite).
  NAIP        not calibrated: a per-channel monotone table count -> reflectance fitted to the area's Sentinel-2
              reflectance on a 10 m lattice, then a local gain (Gaussian sliding window, `NAIP_RATIO_SIGMA_M`) so the
              NAIP means follow the Sentinel-2 reflectance under them everywhere (bright local flats keep their brightness).

Output (EPSG:4326 TMS, level z has 2^(z+1) x 2^z tiles of 256 x 256 px, tile size 180 / 2^z deg, x counts east from 180 W,
y counts south from 90 N, pixel centres at (i + 0.5) / 256 of the tile):
  data/imagery/earth/<source id>/<z>/<x>/<y>.jpg     (JPEG quality 88 [E])
  data/imagery/earth/<source id>/meta.toml           per source, compiled into
  data/imagery/earth/index.toml                      id, product, year, min/max level, exact coverage, fade_m, transfer
No data (sea beyond the NAIP scenes, Sentinel-2 composite gaps) is filled before a tile is written, in reflectance:
NAIP from its area's Sentinel-2 reflectance, then (and Sentinel-2 directly) from the calibrated day texture. Every tile of
the box is written; tiles entirely outside the box are not. Lower levels are 2 x 2 means of the valid pixels of the 4
decoded children in reflectance (linear; no data never enters a mean).

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

import earth_albedo as ea
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
S2_READ_THREADS = 8  # scene reads per block worker
S2_BLOCK_PX = 1024  # composite block (read + median + write unit); multiple of the GTiff block of 512
# [D] Sentinel-2 L2A TCI quantisation, measured on S2B_14RNN_20251116 (1000 x 1000 px, Earth Search COGs whose DN =
# reflectance x 1e4, BOA offset applied): TCI = 0.101604 DN + 0.497 (rms 0.4 counts) = floor(1 + DN x 254 / 2500) in
# all three bands, so a count t decodes to reflectance (t - 0.5) x 2500 / 254 x 1e-4; t = 255 means >= 0.25 (saturated).
S2_TCI_REFLECTANCE_PER_COUNT = 2500.0 / 254.0 * 1e-4
S2_TCI_SATURATED = 255
S2_REFLECTANCE_SCALE = 1e-4  # [S] Earth Search sentinel-2-l2a band COGs (earthsearch:boa_offset_applied): DN x 1e-4
S2_REFLECTANCE_BANDS = ("B04", "B03", "B02")  # [S] red, green, blue (10 m)
NAIP_FIT_CELL_M = 10.0  # NAIP is fitted against the Sentinel-2 reflectance at its resolution
NAIP_FIT_SUBCELLS = 4  # NAIP rendered at 2.5 m, the table applied, then averaged into the 10 m cells (no Jensen bias)
NAIP_RATIO_SIGMA_M = 50.0  # [E] Gaussian sliding window of the local NAIP gain (5 Sentinel-2 pixels)
NAIP_RATIO_LIMITS = (0.5, 2.0)  # [E] bounds of the local gain (NAIP 2021/22 vs Sentinel-2 2025: changed ground)
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
    """Per-pixel, per-channel median over the scenes whose SCL class is clear. rgb (n, h, w, 3) uint8 or uint16 (0 = no
    data), scl (n, h, w) uint8. A pixel without any clear scene takes the median of its valid (non-zero) scenes instead of
    a hole; without valid scenes it stays 0. The result keeps the input's dtype."""
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
    return np.rint(np.nan_to_num(median)).astype(rgb.dtype)


def tci_to_reflectance(tci: np.ndarray) -> np.ndarray:
    """Sentinel-2 L2A TCI counts -> surface reflectance (float64); count 0 (no data) -> 0."""
    t = np.asarray(tci, dtype=np.float64)
    return np.where(t > 0, (t - 0.5) * S2_TCI_REFLECTANCE_PER_COUNT, 0.0)


def to_reflectance(raw: np.ndarray, transfer: dict, lat=None, lon=None) -> np.ndarray:
    """A source's raw pixels (h, w, 3) -> surface reflectance (h, w, 3) float64. `transfer`: {"kind": "scale", "scale"}
    (the Sentinel-2 reflectance composite: DN x scale) or {"kind": "naip", "luts": (3, 256), "ratio": local gain window
    dict (`sample_ratio`)} with the pixel-centre grids `lat` / `lon` (degrees)."""
    if transfer["kind"] == "scale":
        return np.asarray(raw, dtype=np.float64) * transfer["scale"]
    if transfer["kind"] == "naip":
        rho = ea.apply_lut(raw, np.asarray(transfer["luts"]))
        if transfer.get("ratio") is not None:
            rho = rho * sample_ratio(transfer["ratio"], lat, lon)
        return rho
    raise ValueError(f"unknown transfer {transfer['kind']}")


def encode_tile(rho: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """Reflectance -> sRGB-encoded 8-bit tile; valid pixels at least 1 (0 in all channels marks no data)."""
    out = np.maximum(ea.encode_srgb(rho), 1).astype(np.uint8)
    out[~valid] = 0
    return out


def downsample_children(tl, tr, bl, br) -> np.ndarray:
    """The parent tile: 2 x 2 mean in reflectance (linear, sRGB-decoded) of the VALID pixels (not all channels 0) of the
    four (256, 256, 3) children, encoded again; no data never enters a mean. A parent pixel without any valid child pixel
    (a missing child (None) is no data) stays 0."""
    zero = np.zeros((TILE_PX, TILE_PX, 3), dtype=np.uint8)
    rows = [np.concatenate([c if c is not None else zero for c in row], axis=1) for row in ((tl, tr), (bl, br))]
    big = np.concatenate(rows, axis=0)
    valid = big.any(axis=-1).reshape(TILE_PX, 2, TILE_PX, 2, 1)
    total = (ea.decode_srgb(big).reshape(TILE_PX, 2, TILE_PX, 2, 3) * valid).sum(axis=(1, 3))
    count = valid.sum(axis=(1, 3))
    mean = np.divide(total, count, out=np.zeros_like(total), where=count > 0)
    return encode_tile(mean, count[..., 0] > 0)


# ----------------------------------------------------------------------------------------------- rendering

def naip_shift_deg(box: Box) -> tuple[float, float]:
    """(dlat, dlon) of ITRF minus NAD83 at the box centre."""
    south, north, west, east = box
    return et.nad83_horizontal_shift_deg((south + north) / 2.0, (west + east) / 2.0)


def render_box(datasets, box: Box, shift_deg: tuple[float, float], width: int, height: int,
               resampling=None) -> tuple[np.ndarray, np.ndarray]:
    """Reproject the first three bands of every dataset into the geographic box (width x height pixels, edges at the box
    bounds) -> (rgb (h, w, 3) in the datasets' dtype, valid (h, w)). `shift_deg` = (dlat, dlon) moves the content north /
    east (datum shift); a source value of 0 is no data."""
    import rasterio
    from rasterio.transform import from_bounds
    from rasterio.warp import Resampling, reproject, transform_bounds

    resampling = resampling or Resampling.bilinear
    south, north, west, east = box
    dlat, dlon = shift_deg
    # destination grid labelled in the source datum: the pixel at ITRF position P shows the source at P - shift
    dst_transform = from_bounds(west - dlon, south - dlat, east - dlon, north - dlat, width, height)
    out = np.zeros((3, height, width), dtype=np.dtype(datasets[0].dtypes[0]) if datasets else np.uint8)
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
    """The calibrated day texture (earth_albedo.py transfer) as reflectance (h, w, 3) float64."""
    from PIL import Image

    with Image.open(path) as img:
        return ea.decode_srgb(np.asarray(img.convert("RGB")))


def sample_day_texture(texture: np.ndarray, lats: np.ndarray, lons: np.ndarray) -> np.ndarray:
    """Bilinear samples (N, 3) of the equirectangular day texture (u = (lon + 180) / 360, v = (90 - lat) / 180), in
    the texture's values (reflectance from `load_day_texture`: filtered after the decode, as on the GPU)."""
    h, w, _ = texture.shape
    return _sample_texture(texture, 0, 0, h, w, lats, lons)


def _sample_texture(rgb: np.ndarray, row0: int, col0: int, full_h: int, full_w: int, lats, lons) -> np.ndarray:
    """Bilinear samples of a texture window `rgb` whose pixel (0, 0) is pixel (row0, col0) of the full full_h x full_w
    equirectangular texture (a full texture wraps in longitude; a window is clamped to its edge)."""
    fx = (np.asarray(lons, dtype=np.float64) + 180.0) / 360.0 * full_w - 0.5
    fy = np.clip((90.0 - np.asarray(lats, dtype=np.float64)) / 180.0 * full_h - 0.5, 0.0, full_h - 1.0)
    h, w, _ = rgb.shape
    wraps = w == full_w
    if wraps:
        fx = fx % full_w
        x0 = np.floor(fx).astype(int)
        x1 = (x0 + 1) % w
    else:
        fx = np.clip(fx - col0, 0.0, w - 1.0)
        x0 = np.minimum(np.floor(fx).astype(int), max(w - 2, 0))
        x1 = np.minimum(x0 + 1, w - 1)
    fy = np.clip(fy - row0, 0.0, h - 1.0)
    y0 = np.minimum(np.floor(fy).astype(int), max(h - 2, 0))
    y1 = np.minimum(y0 + 1, h - 1)
    wx, wy = (fx - x0)[..., None], (fy - y0)[..., None]
    top = (1 - wx) * rgb[y0, x0] + wx * rgb[y0, x1]
    bottom = (1 - wx) * rgb[y1, x0] + wx * rgb[y1, x1]
    return (1 - wy) * top + wy * bottom


TEXTURE_WINDOW_MARGIN_PX = 3  # beyond the box: covers the finest tiles overhanging the box edge (< 1 texel at z >= 8)


def texture_window(texture: np.ndarray, box: Box, margin_px: int = TEXTURE_WINDOW_MARGIN_PX) -> dict:
    """The part of the day texture around the box (small enough to hand to every tile worker) for `sample_texture_window`."""
    south, north, west, east = box
    h, w, _ = texture.shape
    r0 = max(int(math.floor((90.0 - north) / 180.0 * h)) - margin_px, 0)
    r1 = min(int(math.ceil((90.0 - south) / 180.0 * h)) + margin_px, h)
    c0 = max(int(math.floor((west + 180.0) / 360.0 * w)) - margin_px, 0)
    c1 = min(int(math.ceil((east + 180.0) / 360.0 * w)) + margin_px, w)
    return {"rgb": np.ascontiguousarray(texture[r0:r1, c0:c1]), "row0": r0, "col0": c0, "full_h": h, "full_w": w}


def sample_texture_window(win: dict, lats, lons) -> np.ndarray:
    return _sample_texture(win["rgb"], win["row0"], win["col0"], win["full_h"], win["full_w"], lats, lons)


def gaussian_mean(values: np.ndarray, weight: np.ndarray, sigma_px: float) -> np.ndarray:
    """Normalised Gaussian convolution: the weighted local mean of (h, w, c) values with (h, w) weights (0 = no data);
    NaN where no weight lies within reach."""
    import cv2

    w = weight.astype(np.float32)
    num = cv2.GaussianBlur(values.astype(np.float32) * w[..., None], (0, 0), sigma_px, borderType=cv2.BORDER_REFLECT)
    den = cv2.GaussianBlur(w, (0, 0), sigma_px, borderType=cv2.BORDER_REFLECT)
    if num.ndim == 2:
        num = num[..., None]
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(den[..., None] > 1e-6, num / den[..., None], np.nan)


def local_ratio(reference: np.ndarray, source: np.ndarray, valid: np.ndarray, sigma_px: float,
                limits: tuple[float, float] = NAIP_RATIO_LIMITS) -> np.ndarray:
    """Per-channel local gain reference / source over a Gaussian sliding window (both means over the cells valid in
    both), clipped to `limits`; 1 where the window holds no valid cell."""
    ref_mean = gaussian_mean(reference, valid, sigma_px)
    src_mean = gaussian_mean(source, valid, sigma_px)
    with np.errstate(invalid="ignore", divide="ignore"):
        ratio = ref_mean / src_mean
    ratio = np.where(np.isfinite(ratio), ratio, 1.0)
    return np.clip(ratio, *limits).astype(np.float32)


_RATIO_CACHE: dict[str, np.ndarray] = {}


def sample_ratio(ratio: dict, lat: np.ndarray, lon: np.ndarray) -> np.ndarray:
    """Bilinear samples of a local gain grid {"path": .npy (ny, nx, 3) on the lattice of "box" (cell centres), "box"} at
    the given latitudes / longitudes (degrees); clamped at the grid edge."""
    grid = _RATIO_CACHE.get(ratio["path"])
    if grid is None:
        grid = _RATIO_CACHE[ratio["path"]] = np.load(ratio["path"])
    south, north, west, east = ratio["box"]
    ny, nx, _ = grid.shape
    fx = np.clip((np.asarray(lon) - west) / (east - west) * nx - 0.5, 0.0, nx - 1.0)
    fy = np.clip((north - np.asarray(lat)) / (north - south) * ny - 0.5, 0.0, ny - 1.0)
    x0 = np.minimum(np.floor(fx).astype(int), max(nx - 2, 0))
    y0 = np.minimum(np.floor(fy).astype(int), max(ny - 2, 0))
    x1, y1 = np.minimum(x0 + 1, nx - 1), np.minimum(y0 + 1, ny - 1)
    wx, wy = (fx - x0)[..., None], (fy - y0)[..., None]
    top = (1 - wx) * grid[y0, x0] + wx * grid[y0, x1]
    bottom = (1 - wx) * grid[y1, x0] + wx * grid[y1, x1]
    return (1 - wy) * top + wy * bottom


def fit_naip(datasets, box: Box, shift_deg, ref_datasets, ref_transfer: dict, ratio_path: Path,
             log=print) -> dict:
    """The NAIP transfer against the area's Sentinel-2 reflectance on a 10 m lattice over the box: per channel a monotone
    table count -> reflectance (the 10 m mean counts against the reflectance, `earth_albedo.monotone_lut`), then the local
    gain: the reflectance over the tabled NAIP, both rendered on the lattice (NAIP at 2.5 m, the table applied, averaged
    into the cells), as Gaussian sliding-window means (`NAIP_RATIO_SIGMA_M`), saved to `ratio_path` (.npy).
    Returns the transfer dict for `to_reflectance` plus fit statistics."""
    from rasterio.warp import Resampling

    south, north, west, east = box
    lat_c = (south + north) / 2.0
    ny = max(2, round((north - south) * M_PER_DEG_LAT / NAIP_FIT_CELL_M))
    nx = max(2, round((east - west) * M_PER_DEG_LAT * math.cos(math.radians(lat_c)) / NAIP_FIT_CELL_M))
    counts, valid = render_box(datasets, box, shift_deg, nx, ny, Resampling.average)
    ref_raw, ref_valid = render_box(ref_datasets, box, (0.0, 0.0), nx, ny, Resampling.average)
    ref = to_reflectance(ref_raw, ref_transfer)
    both = valid & ref_valid
    if both.sum() < 100:
        raise RuntimeError("Keine gueltigen Bildpunkte fuer den NAIP-Abgleich")
    luts = np.stack([ea.monotone_lut(counts[both][:, c], ref[both][:, c]) for c in range(3)])
    k = NAIP_FIT_SUBCELLS
    fine, fine_valid = render_box(datasets, box, shift_deg, nx * k, ny * k, Resampling.bilinear)
    tabled = ea.apply_lut(fine, luts.astype(np.float32)) * fine_valid[..., None]
    cell_sum = tabled.reshape(ny, k, nx, k, 3).sum(axis=(1, 3))
    cell_n = fine_valid.reshape(ny, k, nx, k).sum(axis=(1, 3))
    naip = np.divide(cell_sum, cell_n[..., None], out=np.zeros_like(cell_sum), where=cell_n[..., None] > 0)
    both &= cell_n > 0
    ratio = local_ratio(ref, naip, both, NAIP_RATIO_SIGMA_M / NAIP_FIT_CELL_M)
    ratio_path.parent.mkdir(parents=True, exist_ok=True)
    np.save(ratio_path, ratio)
    y = ea.LUMA_WEIGHTS
    stats = {"cells": int(both.sum()), "ratio_p05": np.percentile(ratio[both] @ y, 5).item(),
             "ratio_median": np.median(ratio[both] @ y).item(), "ratio_p95": np.percentile(ratio[both] @ y, 95).item(),
             "ratio_clipped": float(((ratio[both] <= NAIP_RATIO_LIMITS[0]) | (ratio[both] >= NAIP_RATIO_LIMITS[1]))
                                    .any(axis=-1).mean()),
             "luma_ref_mean": float((ref[both] @ y).mean()), "luma_table_mean": float((naip[both] @ y).mean())}
    log(f"    NAIP fit: {stats}")
    return {"kind": "naip", "luts": luts, "ratio": {"path": str(ratio_path), "box": box}, "stats": stats}


# ----------------------------------------------------------------------------------------------- pyramid

# ----------------------------------------------------------------------------------------------- parallel execution

TILE_WORKER_BYTES = 400_000_000  # [E] peak memory of one tile worker (open datasets, GDAL cache, a chunk of tiles)
S2_WORKER_BYTES = 1_200_000_000  # [E] peak of one composite block worker (up to ~16 scenes x 1024^2 x 3 + float32 sort stack)
TILE_CHUNK = 64  # tiles per task


def available_memory_bytes() -> int | None:
    try:
        import psutil

        return int(psutil.virtual_memory().available)
    except ImportError:
        pass
    if os.name == "nt":
        import ctypes

        class Status(ctypes.Structure):
            _fields_ = [("length", ctypes.c_ulong), ("load", ctypes.c_ulong), ("total_phys", ctypes.c_ulonglong),
                        ("avail_phys", ctypes.c_ulonglong), ("total_page", ctypes.c_ulonglong),
                        ("avail_page", ctypes.c_ulonglong), ("total_virt", ctypes.c_ulonglong),
                        ("avail_virt", ctypes.c_ulonglong), ("avail_ext", ctypes.c_ulonglong)]

        st = Status()
        st.length = ctypes.sizeof(Status)
        if ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(st)):
            return int(st.avail_phys)
        return None
    try:
        return os.sysconf("SC_AVPHYS_PAGES") * os.sysconf("SC_PAGE_SIZE")
    except (ValueError, OSError, AttributeError):
        return None


def resolve_workers(requested: int | None, worker_bytes: int, log=print, what: str = "") -> int:
    """Worker count: `requested` (--workers), else the CPUs usable by this process (os.process_cpu_count on Python 3.13+,
    else os.cpu_count, else 1); capped so that workers x `worker_bytes` fits 80 % of the available memory."""
    cpu_count = getattr(os, "process_cpu_count", os.cpu_count)
    workers = max(1, requested if requested else (cpu_count() or 1))
    mem = available_memory_bytes()
    if mem is not None and workers * worker_bytes > 0.8 * mem:
        capped = max(1, int(0.8 * mem // worker_bytes))
        log(f"  {what}: workers {workers} -> {capped} (memory cap: {worker_bytes / 1e6:.0f} MB per worker, "
            f"{mem / 1e9:.1f} GB available)")
        workers = capped
    return workers


def iter_tasks(fn, tasks: list, workers: int):
    """Results of fn(task) as they complete: in-process and in order for workers == 1, else on a process pool."""
    if workers <= 1 or len(tasks) <= 1:
        for t in tasks:
            yield fn(t)
        return
    from concurrent.futures import ProcessPoolExecutor, as_completed

    with ProcessPoolExecutor(min(workers, len(tasks))) as pool:
        futures = [pool.submit(fn, t) for t in tasks]
        for f in as_completed(futures):
            yield f.result()


def _chunks(items: list, size: int) -> list[list]:
    return [items[i:i + size] for i in range(0, len(items), size)]


def pixel_centres(bounds: Box, n: int = TILE_PX) -> tuple[np.ndarray, np.ndarray]:
    """(lat, lon) grids (n, n) of the pixel centres of a tile (row 0 at the north edge)."""
    south, north, west, east = bounds
    c = (np.arange(n) + 0.5) / n
    lon_g, lat_g = np.meshgrid(west + c * (east - west), north - c * (north - south))
    return lat_g, lon_g


def fill_nodata(rho: np.ndarray, valid: np.ndarray, bounds: Box, reference=None, texture=None) -> tuple[np.ndarray, np.ndarray]:
    """Fill the no-data pixels of a tile in reflectance: first from `reference` = (datasets, transfer), the Sentinel-2
    reflectance of the area (for NAIP; rendered without datum shift), then from `texture` (a `texture_window` of the
    calibrated day texture in reflectance). Returns (reflectance, valid); pixels neither layer covers stay invalid."""
    out = rho.copy()
    valid = valid.copy()
    hole = ~valid
    if hole.any() and reference is not None:
        ref_datasets, ref_transfer = reference
        ref_raw, ref_valid = render_box(ref_datasets, bounds, (0.0, 0.0), rho.shape[1], rho.shape[0])
        take = hole & ref_valid
        out[take] = to_reflectance(ref_raw, ref_transfer)[take]
        valid |= take
        hole &= ~ref_valid
    if hole.any() and texture is not None:
        lat_g, lon_g = pixel_centres(bounds, rho.shape[0])
        out[hole] = sample_texture_window(texture, lat_g[hole], lon_g[hole])
        valid |= hole
    return out, valid


def _nodata_inside(tile: np.ndarray, bounds: Box, box: Box) -> int:
    """Count of no-data pixels (all channels 0) of a tile whose centres lie inside the box."""
    south, north, west, east = box
    lat_g, lon_g = pixel_centres(bounds, tile.shape[0])
    inside = (lat_g > south) & (lat_g < north) & (lon_g > west) & (lon_g < east)
    return int((inside & ~tile.any(axis=-1)).sum())


def _render_chunk(task) -> tuple[int, int]:
    """Worker: render + transfer to reflectance (+ no-data fill) + encode + write the finest-level tiles of one chunk ->
    (written count, no-data pixels left inside the box). Without a fill, tiles without any valid pixel are skipped."""
    import rasterio

    paths, root, z, tiles, shift_deg, transfer, fill, box = task
    datasets = [rasterio.open(p) for p in paths]
    ref_datasets = [rasterio.open(p) for p in fill["reference"][0]] if fill and fill.get("reference") else []
    try:
        reference = (ref_datasets, fill["reference"][1]) if ref_datasets else None
        texture = fill.get("texture") if fill else None
        n = left = 0
        for x, y in tiles:
            bounds = tile_bounds(z, x, y)
            raw, valid = render_tile(datasets, bounds, shift_deg)
            if not fill and not valid.any():
                continue
            lat_g, lon_g = pixel_centres(bounds, TILE_PX)
            rho = to_reflectance(raw, transfer, lat_g, lon_g)
            if fill:
                rho, valid = fill_nodata(rho, valid, bounds, reference, texture)
            tile = encode_tile(rho, valid)
            left += _nodata_inside(tile, bounds, box)
            write_tile(Path(root), z, x, y, tile)
            n += 1
        return n, left
    finally:
        for ds in datasets + ref_datasets:
            ds.close()


def _downsample_chunk(task) -> int:
    """Worker: parent tiles of one chunk from the decoded children on disk; returns the written count."""
    root, z, tiles = task
    root, n = Path(root), 0
    for x, y in tiles:
        kids = [decode_jpeg(tile_path(root, z + 1, 2 * x + dx, 2 * y + dy)) for dy in (0, 1) for dx in (0, 1)]
        if all(k is None for k in kids):
            continue
        write_tile(root, z, x, y, downsample_children(*kids))
        n += 1
    return n


def write_pyramid(paths: list[Path], box: Box, shift_deg, transfer: dict, min_level: int, max_level: int, root: Path,
                  log=print, workers: int = 1, fill: dict | None = None) -> dict[int, int]:
    """Finest level rendered from the raster files (`transfer` to reflectance, `to_reflectance`, sRGB-encoded), lower
    levels as reflectance box means of the decoded children, tiles in parallel (`workers` processes; the files do not
    depend on the count). `fill` = {"reference": (paths, transfer) (optional), "texture": texture_window}: no data is
    filled before writing (`fill_nodata`) and every tile of the box is written. Returns {level: count}."""
    counts: dict[int, int] = {}
    south, north, west, east = box
    paths = [str(p) for p in paths]
    if fill and fill.get("reference"):
        ref_paths, ref_transfer = fill["reference"]
        fill = dict(fill, reference=([str(p) for p in ref_paths], ref_transfer))
    tiles = tiles_for_box(max_level, south, north, west, east)
    tasks = [(paths, str(root), max_level, c, shift_deg, transfer, fill, box) for c in _chunks(tiles, TILE_CHUNK)]
    results = list(iter_tasks(_render_chunk, tasks, workers))
    counts[max_level] = sum(r[0] for r in results)
    log(f"    z{max_level}: {counts[max_level]} tiles, no-data pixels left inside the box: {sum(r[1] for r in results)}")
    for z in range(max_level - 1, min_level - 1, -1):
        tasks = [(str(root), z, c) for c in _chunks(tiles_for_box(z, south, north, west, east), TILE_CHUNK)]
        counts[z] = sum(iter_tasks(_downsample_chunk, tasks, workers))
        log(f"    z{z}: {counts[z]} tiles")
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


def _s2_block(task):
    """Worker: the median of one composite block -> (r0, c0, (h, w, 3) uint8 or None when no scene covers it).
    Reads the intersecting scenes on a few threads (network latency). Task (scenes, r0, c0, h, w[, bands]): with `bands`
    true the reflectance bands B04 / B03 / B02 (uint16, next to each scene's visual file) instead of the visual bands."""
    import rasterio
    from rasterio.enums import Resampling
    from rasterio.windows import Window

    _gdal_http_env()
    scenes, r0, c0, h, w = task[:5]
    bands = len(task) > 5 and task[5]

    def read_item(scene):
        ra, ca = max(r0 + scene["row_off"], 0), max(c0 + scene["col_off"], 0)
        rb, cb = min(r0 + scene["row_off"] + h, scene["height"]), min(c0 + scene["col_off"] + w, scene["width"])
        if rb <= ra or cb <= ca:
            return None
        win = Window(ca, ra, cb - ca, rb - ra)
        scl_win = Window(ca / 2.0, ra / 2.0, (cb - ca) / 2.0, (rb - ra) / 2.0)

        def go():
            with rasterio.open(scene["scl"]) as s:
                scl = s.read(1, window=scl_win, out_shape=(rb - ra, cb - ca), resampling=Resampling.nearest)
            if not bands:
                with rasterio.open(scene["visual"]) as v:
                    return v.read([1, 2, 3], window=win), scl
            visual = scene["visual"]
            base = visual[:max(visual.rfind("/"), visual.rfind("\\"))]  # the band files lie next to the visual file
            rgb = []
            for band in S2_REFLECTANCE_BANDS:
                with rasterio.open(f"{base}/{band}.tif") as b:
                    rgb.append(b.read(1, window=win))
            return np.stack(rgb), scl

        rgb, scl = _retry(go, scene["id"], lambda *_: None)
        return ra - scene["row_off"] - r0, ca - scene["col_off"] - c0, rgb, scl

    with ThreadPoolExecutor(S2_READ_THREADS) as pool:
        parts = [p for p in pool.map(read_item, scenes) if p is not None]
    if not parts:
        return r0, c0, None
    rgb = np.zeros((len(parts), h, w, 3), dtype=np.uint16 if bands else np.uint8)
    scl = np.zeros((len(parts), h, w), dtype=np.uint8)
    for n, (dr, dc, v, sc) in enumerate(parts):
        rgb[n, dr:dr + v.shape[1], dc:dc + v.shape[2]] = np.moveaxis(v, 0, -1)
        scl[n, dr:dr + sc.shape[0], dc:dc + sc.shape[1]] = sc
    return r0, c0, clear_median(rgb, scl)


def build_s2_composite(items: list[dict], box: Box, raw_dir: Path, force: bool, log=print,
                       workers: int | None = None) -> Path:
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

    scenes = [{"id": infos[k][0]["id"], "visual": infos[k][0]["visual"], "scl": infos[k][0]["scl"],
               "row_off": offsets[k][0], "col_off": offsets[k][1], "width": offsets[k][2], "height": offsets[k][3]}
              for k in range(len(infos))]
    blocks = [(r, c) for r in range(0, height, S2_BLOCK_PX) for c in range(0, width, S2_BLOCK_PX)]
    todo = [(scenes, r0, c0, min(S2_BLOCK_PX, height - r0), min(S2_BLOCK_PX, width - c0))
            for r0, c0 in blocks if f"{r0},{c0}" not in done]
    t0 = time.time()
    workers = resolve_workers(workers, S2_WORKER_BYTES, log, "composite")
    log(f"    composite: {len(todo)} of {len(blocks)} blocks to compute, {workers} workers")
    with rasterio.open(partial, "r+") as dst:
        # the parent writes the blocks and the progress file (resumable); workers only compute
        for r0, c0, out in iter_tasks(_s2_block, todo, workers):
            if out is not None:
                h, w = out.shape[:2]
                dst.write(np.moveaxis(out, -1, 0), window=Window(c0, r0, w, h))
            done.add(f"{r0},{c0}")
            progress_file.write_text(json.dumps(sorted(done)))
            log(f"    composite block {len(done)}/{len(blocks)} ({time.time() - t0:.0f} s)")
    partial.replace(final)
    progress_file.unlink(missing_ok=True)
    return final


def build_s2_reflectance(items: list[dict], composite: Path, raw_dir: Path, force: bool, log=print,
                         workers: int | None = None) -> tuple[Path, int]:
    """The composite as surface reflectance (uint16, reflectance x 1e4, 0 = no data; reflectance.tif on the composite's
    grid): the TCI median decoded (`tci_to_reflectance`); pixels saturated in any TCI channel (>= 0.25) take the median
    of the B04 / B03 / B02 reflectance bands with the same clear-pixel rule, recomputed for the blocks holding them
    (network). Returns (path, saturated pixel count)."""
    import rasterio
    from rasterio.windows import Window

    _gdal_http_env()
    final = raw_dir / "reflectance.tif"
    meta_file = raw_dir / "reflectance.json"
    if final.exists() and meta_file.exists() and not force:
        return final, int(json.loads(meta_file.read_text())["saturated_pixels"])
    partial = raw_dir / "reflectance.partial.tif"
    with rasterio.open(composite) as src:
        crs, transform, width, height = src.crs, src.transform, src.width, src.height
        profile = dict(driver="GTiff", width=width, height=height, count=3, dtype="uint16", crs=crs,
                       transform=transform, tiled=True, blockxsize=512, blockysize=512, compress="deflate",
                       predictor=2, BIGTIFF="YES", nodata=0)
        saturated_blocks, saturated = [], 0
        with rasterio.open(partial, "w", **profile) as dst:
            for r0 in range(0, height, S2_BLOCK_PX):
                for c0 in range(0, width, S2_BLOCK_PX):
                    win = Window(c0, r0, min(S2_BLOCK_PX, width - c0), min(S2_BLOCK_PX, height - r0))
                    tci = np.moveaxis(src.read(window=win), 0, -1)
                    sat = (tci == S2_TCI_SATURATED).any(axis=-1)
                    if sat.any():
                        saturated_blocks.append((r0, c0, int(win.height), int(win.width)))
                        saturated += int(sat.sum())
                    rho = np.rint(tci_to_reflectance(tci) / S2_REFLECTANCE_SCALE)
                    out = np.where(tci.any(axis=-1)[..., None], np.maximum(rho, 1), 0).astype(np.uint16)
                    dst.write(np.moveaxis(out, -1, 0), window=win)
    log(f"    reflectance: {saturated} TCI-saturated pixels in {len(saturated_blocks)} blocks -> reflectance bands")
    if saturated_blocks:
        def probe(item):
            with rasterio.open(item["visual"]) as ds:
                return item, ds.crs, ds.transform, ds.width, ds.height

        with ThreadPoolExecutor(8) as pool:
            infos = list(pool.map(lambda it: _retry(lambda: probe(it), it["id"], log), items))
        scenes = []
        for item, scrs, t, w, h in infos:
            if scrs != crs:
                continue
            col_off, row_off = (transform.c - t.c) / S2_PIXEL_M, (t.f - transform.f) / S2_PIXEL_M
            scenes.append({"id": item["id"], "visual": item["visual"], "scl": item["scl"], "row_off": round(row_off),
                           "col_off": round(col_off), "width": w, "height": h})
        tasks = [(scenes, r0, c0, h, w, True) for r0, c0, h, w in saturated_blocks]
        t0 = time.time()
        n = 0
        with rasterio.open(composite) as src, rasterio.open(partial, "r+") as dst:
            for r0, c0, med in iter_tasks(_s2_block, tasks, resolve_workers(workers, S2_WORKER_BYTES, log, "bands")):
                n += 1
                if med is None:
                    continue
                h, w = med.shape[:2]
                win = Window(c0, r0, w, h)
                sat = (np.moveaxis(src.read(window=win), 0, -1) == S2_TCI_SATURATED).any(axis=-1) & med.any(axis=-1)
                cur = np.moveaxis(dst.read(window=win), 0, -1)
                cur[sat] = np.maximum(med[sat], 1)
                dst.write(np.moveaxis(cur, -1, 0), window=win)
                log(f"    reflectance bands block {n}/{len(tasks)} ({time.time() - t0:.0f} s)")
    partial.replace(final)
    meta_file.write_text(json.dumps({"saturated_pixels": saturated, "saturated_blocks": len(saturated_blocks)}))
    return final, saturated


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
             f'tile_count = {m["tile_count"]}']
    for k, v in m.get("extra", {}).items():
        lines.append(f'{k} = "{v}"' if isinstance(v, str) else f"{k} = {v}" if isinstance(v, int) else f"{k} = {v:.6f}")
    lines.append("")
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
    """(reflectance path list, transfer) of the Sentinel-2 source of the same area, None when not built yet."""
    cfg = load_config(repo_root / "config" / "imagery.toml")
    for other in cfg["source"]:
        if other["product"] == "sentinel2_median" and other["area"] == src["area"]:
            reflectance = repo_root / "data" / "raw" / "imagery" / other["id"] / "reflectance.tif"
            if reflectance.exists():
                return [reflectance], S2_TRANSFER
    raise RuntimeError(f"{sid}: keine Sentinel-2-Reflektanz des Gebiets {src['area']} (fetch.py imagery baut sie zuerst)")


S2_TRANSFER = {"kind": "scale", "scale": S2_REFLECTANCE_SCALE}


def s2_reflectance(src: dict, areas: dict[str, dict], repo_root: Path, force: bool, log=print,
                   workers: int | None = None) -> tuple[Path, int, int]:
    """A Sentinel-2 source's reflectance.tif (items, composite and reflectance built when missing or forced) ->
    (path, TCI-saturated pixel count, scene count)."""
    raw_dir = repo_root / "data" / "raw" / "imagery" / src["id"]
    items = s2_items(source_box(src, areas), raw_dir, force, log)
    composite = build_s2_composite(items, source_box(src, areas), raw_dir, force, log, workers)
    path, saturated = build_s2_reflectance(items, composite, raw_dir, force, log, workers)
    return path, saturated, len(items)


def build_source(src: dict, areas: dict[str, dict], repo_root: Path, texture: np.ndarray, force: bool, log=print,
                 workers: int | None = None) -> dict:
    """One source's tile pyramid. `texture`: the calibrated day texture in reflectance (`load_day_texture`)."""
    import rasterio

    sid, product = src["id"], src["product"]
    box = source_box(src, areas)
    raw_dir = repo_root / "data" / "raw" / "imagery" / sid
    root = repo_root / "data" / "imagery" / "earth" / sid
    if root.exists():
        shutil.rmtree(root)
    reference, extra = None, {}
    if product == "naip":
        paths, year, covered = naip_clips(sid, box, raw_dir, force, log)
        shift, scenes = naip_shift_deg(box), len(paths)
        reference = s2_reference(sid, src, areas, repo_root)
        datasets = [rasterio.open(p) for p in paths]
        ref_datasets = [rasterio.open(p) for p in reference[0]]
        try:
            transfer = fit_naip(datasets, covered, shift, ref_datasets, reference[1], raw_dir / "local_gain.npy", log)
        finally:
            for ds in datasets + ref_datasets:
                ds.close()
        extra = {"transfer": "naip_table_local_gain", **{f"fit_{k}": v for k, v in transfer.pop("stats").items()}}
    elif product == "sentinel2_median":
        # built (or forced) by fetch() before the day texture: here only read from the cache
        reflectance, saturated, scenes = s2_reflectance(src, areas, repo_root, False, log, workers)
        paths, year, covered, shift = [reflectance], S2_YEAR, box, (0.0, 0.0)
        transfer = S2_TRANSFER
        extra = {"transfer": "sentinel2_l2a_reflectance", "tci_saturated_pixels": saturated}
    else:
        raise ValueError(f"unbekanntes Produkt {product}")
    # no data inside the box (sea beyond the scenes, composite gaps) is filled in reflectance before writing
    fill = {"reference": reference, "texture": texture_window(texture, covered)}
    counts = write_pyramid(paths, covered, shift, transfer, src["min_level"], src["max_level"], root, log,
                           resolve_workers(workers, TILE_WORKER_BYTES, log, "tiles"), fill)
    south, north, west, east = covered
    meta = {"id": sid, "product": product, "year": year, "scenes": scenes, "min_level": src["min_level"],
            "max_level": src["max_level"], "north_deg": north, "south_deg": south, "west_deg": west, "east_deg": east,
            "fade_m": FADE_M[product], "extra": extra, "tile_count": sum(counts.values())}
    (root / "meta.toml").write_text(meta_text(meta), encoding="utf-8")
    log(f"  {sid}: {sum(counts.values())} tiles {counts}, {_dir_size(root) / 1e6:.1f} MB")
    return meta


def fetch(source: dict, repo_root: Path, download, force: bool = False, retile: bool = False,
          workers: int | None = None) -> None:
    """`retile`: rebuild the fits and all tiles from the cached clips / composites / reflectance files (no network once
    reflectance.tif exists), keeping the raw caches."""
    cfg = load_config(repo_root / "config" / "imagery.toml")
    areas = {a["id"]: a for a in et.load_config(repo_root / "config" / "earth_terrain.toml")["area"]}
    # the Sentinel-2 reflectance first: the calibrated day texture embeds it (earth_albedo.py, as prepare.py writes it)
    for src in cfg["source"]:
        if src["product"] == "sentinel2_median":
            s2_reflectance(src, areas, repo_root, force, lambda s: print(s, flush=True), workers)
    texture = ea.decode_srgb(ea.build_day_texture(repo_root))
    out_dir = repo_root / "data" / "imagery" / "earth"
    out_dir.mkdir(parents=True, exist_ok=True)
    warnings.filterwarnings("ignore", category=RuntimeWarning)
    print(f"  workers: {resolve_workers(workers, TILE_WORKER_BYTES, print, 'tiles')} (CPUs usable: "
          f"{getattr(os, 'process_cpu_count', os.cpu_count)() or 1}{', --workers ' + str(workers) if workers else ''})", flush=True)
    # Sentinel-2 first: the NAIP fit uses the Sentinel-2 reflectance of its area as the reference
    rebuilt_areas: set[str] = set()  # a rebuilt Sentinel-2 source invalidates the NAIP fitted against it
    for src in sorted(cfg["source"], key=lambda s: s["product"] == "naip"):
        if (out_dir / src["id"] / "meta.toml").exists() and not force and not retile                 and not (src["product"] == "naip" and src["area"] in rebuilt_areas):
            print(f"  {src['id']}: already present")
            continue
        print(f"  {src['id']}: {src['product']} over {src['area']} ({src['box']} box)", flush=True)
        build_source(src, areas, repo_root, texture, force, log=lambda s: print(s, flush=True), workers=workers)
        if src["product"] == "sentinel2_median":
            rebuilt_areas.add(src["area"])
        write_index(out_dir, [s["id"] for s in cfg["source"]])
    write_index(out_dir, [s["id"] for s in cfg["source"]])


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force", action="store_true", help="rebuild clips / composites and tiles")
    parser.add_argument("--retile", action="store_true", help="redo fit and tiles from the cached clips / composites")
    parser.add_argument("--workers", type=int, default=None, help="worker processes (default: CPUs usable by the process)")
    args = parser.parse_args()
    fetch({}, REPO_ROOT, None, args.force, args.retile, args.workers)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
