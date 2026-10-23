"""Earth height tiles (M8 part 1): global relief, regional and local terrain around the launch areas.

Sources (all heights are written as WGS84 ellipsoidal heights, `height_reference = "wgs84_ellipsoid"`):
  global   NOAA NCEI ETOPO 2022 60" surface (ice surface and ocean bathymetry; orthometric to EGM2008, public domain),
           box-averaged to a 2' node grid, + EGM2008 undulation          -> global_relief.toml/.i16     (dn 1 m)
  regional Copernicus DEM GLO-30 (AWS open data bucket copernicus-dem-30m, 1" COG tiles; a SURFACE model: vegetation and
           buildings included; orthometric to EGM2008) + EGM2008        -> <area>_regional.toml/.i16  (dn 0.1 m)
  local    USGS 3DEP 1 m DEM (TNM Access API; BARE EARTH; NAVD88 orthometric, UTM NAD83) reprojected to a geographic grid of
           1/30" latitude step, + GEOID18 (NOAA, via cdn.proj.org us_noaa_g2018u0.tif) -> <area>_local.toml/.i16 (dn 0.01 m)
           Gaps of the 3DEP coverage take the regional value; an area without 3DEP uses GLO-30 for its local level.
Output directory data/terrain/earth/; raw downloads are cached in data/raw/earth/. The header format is the `[grid]` of
grid_file.py (node-registered, line 0 = north edge); extra keys: area, level, source, area centre, half size, notes.

Usage: python tools/data_pipeline/fetch.py [--force] earth_terrain   (after the geoid source)
"""

from __future__ import annotations

import json
import math
import tomllib
import urllib.parse
import urllib.request
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import grid_file

REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_FILE = REPO_ROOT / "config" / "earth_terrain.toml"

# WGS84 (core/include/starsim/math/constants.hpp): semi-major axis and first eccentricity squared
WGS84_A_M = 6378137.0
WGS84_E2 = 0.00669437999014

GLO30_STEP_DEG = 1.0 / 3600.0
LOCAL_STEP_LAT_DEG = 1.0 / (30.0 * 3600.0)  # 1/30 arcsec = 1.03 m
DN_GLOBAL_M, DN_REGIONAL_M, DN_LOCAL_M = 1.0, 0.1, 0.01
ETOPO_PIXELS_PER_NODE = 2  # 60" pixels -> 2' nodes
GLO30_URL = "https://copernicus-dem-30m.s3.amazonaws.com/{name}/{name}.tif"
TNM_API = "https://tnmaccess.nationalmap.gov/api/v1/products"
TNM_DATASET = "Digital Elevation Model (DEM) 1 meter"
MIN_3DEP_COVERAGE = 0.01  # below this fraction of valid local nodes the area falls back to GLO-30
RESAMPLE_ROWS = 512  # rows per chunk of the separable bilinear resampling


@dataclass(frozen=True)
class Geo:
    """A node-registered geographic grid: node (i, j) at (north - i * step_lat, west + j * step_lon)."""

    north: float
    west: float
    step_lat: float
    step_lon: float
    lines: int
    samples: int

    @property
    def south(self) -> float:
        return self.north - (self.lines - 1) * self.step_lat

    @property
    def east(self) -> float:
        return self.west + (self.samples - 1) * self.step_lon

    def lats(self) -> np.ndarray:
        return self.north - np.arange(self.lines) * self.step_lat

    def lons(self) -> np.ndarray:
        return self.west + np.arange(self.samples) * self.step_lon


GEOID_GEO = Geo(north=90.0, west=-180.0, step_lat=1.0 / 24.0, step_lon=1.0 / 24.0, lines=4321, samples=8640)


# ----------------------------------------------------------------------------------------------- pure functions

def load_config(path: Path = CONFIG_FILE) -> dict:
    with path.open("rb") as f:
        return tomllib.load(f)


def area_bounds(latitude_deg: float, longitude_deg: float, half_size_m: float) -> tuple[float, float, float, float]:
    """(south, north, west, east) in degrees of the square of half size `half_size_m` around the centre, from the WGS84
    meridian and prime-vertical radii of curvature at the centre latitude."""
    lat = math.radians(latitude_deg)
    w = math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
    meridian_m = WGS84_A_M * (1.0 - WGS84_E2) / w**3
    prime_vertical_m = WGS84_A_M / w
    d_lat = math.degrees(half_size_m / meridian_m)
    d_lon = math.degrees(half_size_m / (prime_vertical_m * math.cos(lat)))
    return latitude_deg - d_lat, latitude_deg + d_lat, longitude_deg - d_lon, longitude_deg + d_lon


def grid_for_box(latitude_deg: float, longitude_deg: float, half_size_m: float, step_lat: float, step_lon: float) -> Geo:
    """A grid centred on a node at the area centre that covers the whole box (odd node counts)."""
    south, north, west, east = area_bounds(latitude_deg, longitude_deg, half_size_m)
    n_lat = math.ceil((north - latitude_deg) / step_lat - 1e-9)
    n_lon = math.ceil((east - longitude_deg) / step_lon - 1e-9)
    return Geo(north=latitude_deg + n_lat * step_lat, west=longitude_deg - n_lon * step_lon, step_lat=step_lat,
               step_lon=step_lon, lines=2 * n_lat + 1, samples=2 * n_lon + 1)


def resample_grid(array: np.ndarray, src: Geo, lats: np.ndarray, lons: np.ndarray, *, wrap: bool = False,
                  dtype=np.float64, scale: float = 1.0) -> np.ndarray:
    """Separable bilinear values of `array` (on `src`) at the outer product lats x lons; positions outside clamp to the
    edge (longitude wraps instead when `wrap`). NaN in the source propagates to the neighbouring results."""
    lats, lons = np.asarray(lats, dtype=np.float64), np.asarray(lons, dtype=np.float64)
    fj = (lons - src.west) / src.step_lon
    if wrap:
        fj = fj % src.samples
        j0 = np.floor(fj).astype(np.int64)
        wj = fj - j0
        j0 %= src.samples
        j1 = (j0 + 1) % src.samples
    else:
        fj = np.clip(fj, 0.0, src.samples - 1.0)
        j0 = np.minimum(np.floor(fj).astype(np.int64), max(src.samples - 2, 0))
        wj = fj - j0
        j1 = np.minimum(j0 + 1, src.samples - 1)
    fi = np.clip((src.north - lats) / src.step_lat, 0.0, src.lines - 1.0)
    i0 = np.minimum(np.floor(fi).astype(np.int64), max(src.lines - 2, 0))
    wi = fi - i0
    i1 = np.minimum(i0 + 1, src.lines - 1)
    out = np.empty((lats.size, lons.size), dtype=dtype)
    for a in range(0, lats.size, RESAMPLE_ROWS):
        b = min(a + RESAMPLE_ROWS, lats.size)
        r0, r1 = i0[a:b, None], i1[a:b, None]
        w_i = wi[a:b, None]
        top = (1 - wj) * array[r0, j0[None, :]] + wj * array[r0, j1[None, :]]
        bottom = (1 - wj) * array[r1, j0[None, :]] + wj * array[r1, j1[None, :]]
        out[a:b] = ((1 - w_i) * top + w_i * bottom) * scale
    return out


def orthometric_to_ellipsoidal(h_orthometric_m: np.ndarray, lats: np.ndarray, lons: np.ndarray,
                               geoid_cm: np.ndarray) -> np.ndarray:
    """h_ellipsoidal = H + N with the EGM2008 undulation N (2.5' grid in cm) at every node of lats x lons."""
    n = resample_grid(geoid_cm, GEOID_GEO, lats, lons, wrap=True, dtype=np.float64, scale=0.01)
    return (h_orthometric_m + n).astype(h_orthometric_m.dtype if h_orthometric_m.dtype.kind == "f" else np.float64)


def _navd88_transformer(geoid18_path: Path):
    from pyproj import Transformer

    grids = str(Path(geoid18_path).resolve()).replace("\\", "/")
    pipeline = ("+proj=pipeline +step +proj=unitconvert +xy_in=deg +xy_out=rad "
                f"+step +proj=vgridshift +grids={grids} +multiplier=1 "
                "+step +proj=unitconvert +xy_in=rad +xy_out=deg")
    return Transformer.from_pipeline(pipeline)


def navd88_to_ellipsoidal(h_navd88_m, lat_deg, lon_deg, geoid18_path: Path) -> np.ndarray:
    """NAVD88 orthometric height -> ellipsoidal height with the NOAA GEOID18 grid, through PROJ (pyproj)."""
    t = _navd88_transformer(geoid18_path)
    h = np.asarray(h_navd88_m, dtype=np.float64)
    # the PROJ vgridshift step with +multiplier=1 run forward adds the grid value: h = H + N
    _, _, h_out = t.transform(np.asarray(lon_deg, dtype=np.float64), np.asarray(lat_deg, dtype=np.float64), h)
    return np.asarray(h_out)


def navd88_grid_to_ellipsoidal(h_navd88_m: np.ndarray, geo: Geo, geoid18_path: Path, lattice_nodes: int = 64) -> np.ndarray:
    """The same for a whole tile: GEOID18 (1' cells) is evaluated through PROJ on a lattice every `lattice_nodes` nodes
    and interpolated bilinearly to the tile nodes (the geoid varies by < 1 mm per 64 m), then added."""
    li = np.unique(np.append(np.arange(0, geo.lines, lattice_nodes), geo.lines - 1))
    lj = np.unique(np.append(np.arange(0, geo.samples, lattice_nodes), geo.samples - 1))
    lat_l, lon_l = geo.lats()[li], geo.lons()[lj]
    lon_m, lat_m = np.meshgrid(lon_l, lat_l)
    n = navd88_to_ellipsoidal(np.zeros(lat_m.shape), lat_m.ravel(), lon_m.ravel(), geoid18_path).reshape(lat_m.shape)
    # the lattice is not uniform at the last row/column: interpolate by index position
    fi = np.interp(np.arange(geo.lines), li, np.arange(li.size))
    fj = np.interp(np.arange(geo.samples), lj, np.arange(lj.size))
    i0 = np.minimum(np.floor(fi).astype(np.int64), max(li.size - 2, 0))
    j0 = np.minimum(np.floor(fj).astype(np.int64), max(lj.size - 2, 0))
    i1, j1 = np.minimum(i0 + 1, li.size - 1), np.minimum(j0 + 1, lj.size - 1)
    wi, wj = (fi - i0)[:, None], (fj - j0)[None, :]
    out = np.empty(h_navd88_m.shape, dtype=np.float32)
    for a in range(0, geo.lines, RESAMPLE_ROWS):
        b = min(a + RESAMPLE_ROWS, geo.lines)
        r0, r1, w = i0[a:b, None], i1[a:b, None], wi[a:b]
        nn = ((1 - w) * ((1 - wj) * n[r0, j0[None, :]] + wj * n[r0, j1[None, :]])
              + w * ((1 - wj) * n[r1, j0[None, :]] + wj * n[r1, j1[None, :]]))
        out[a:b] = h_navd88_m[a:b] + nn
    return out


def resample_global(raster: np.ndarray, ratio: int) -> np.ndarray:
    """Pixel-registered global raster (lines x columns, pixel centres half a pixel inside the poles / antimeridian) ->
    node-registered grid with nodes `ratio` pixels apart (nodes at the pole lines and at the antimeridian, no +180
    column): every node is the box average of the ratio x ratio pixels around it. Columns wrap; at the pole lines only the
    pixel row on the map side exists."""
    if ratio % 2:
        raise ValueError("ratio must be even (nodes sit on pixel boundaries)")
    lines, columns = raster.shape
    if lines % ratio or columns % ratio:
        raise ValueError(f"raster {raster.shape} is not a multiple of {ratio}")
    half = ratio // 2
    node_lines, node_cols = lines // ratio + 1, columns // ratio
    # column sums over the wrapped blocks [j * ratio - half, j * ratio + half)
    rolled = np.roll(raster.astype(np.float64), half, axis=1)
    cols = rolled.reshape(lines, node_cols, ratio).sum(axis=2)  # block j covers original columns j*ratio-half .. +half-1
    out = np.empty((node_lines, node_cols), dtype=np.float64)
    out[0] = cols[:half].sum(axis=0) / (half * ratio)
    out[-1] = cols[lines - half:].sum(axis=0) / (half * ratio)
    for k in range(1, node_lines - 1):
        out[k] = cols[k * ratio - half:k * ratio + half].sum(axis=0) / (ratio * ratio)
    return out


def fill_nodata_from_regional(local: np.ndarray, local_geo: Geo, regional: np.ndarray,
                              regional_geo: Geo) -> tuple[np.ndarray, int]:
    """Replace the NaN nodes of the local tile (3DEP gaps) with the regional heights sampled at them."""
    missing = np.isnan(local)
    count = int(missing.sum())
    if count == 0:
        return local, 0
    filled = local.copy()
    rows = np.flatnonzero(missing.any(axis=1))
    lats, lons = local_geo.lats(), local_geo.lons()
    for a in range(0, rows.size, RESAMPLE_ROWS):
        r = rows[a:a + RESAMPLE_ROWS]
        values = resample_grid(regional, regional_geo, lats[r], lons, dtype=np.float32)
        m = missing[r]
        sub = filled[r]
        sub[m] = values[m]
        filled[r] = sub
    return filled, count


def to_dn(heights_m: np.ndarray, dn_scale_m: float) -> np.ndarray:
    if np.isnan(heights_m).any():
        raise RuntimeError("nodata left in a tile")
    dn = np.rint(heights_m / dn_scale_m)
    if dn.min() < -32768 or dn.max() > 32767:
        raise RuntimeError(f"heights [{dn.min() * dn_scale_m}, {dn.max() * dn_scale_m}] m do not fit int16 at {dn_scale_m} m")
    return dn.astype("<i2")


def check_points(geo: Geo, area_lat: float, area_lon: float) -> list[tuple[float, float]]:
    """Centre and four points at 30 % of the extent, all inside the grid."""
    south, east = geo.south, geo.east
    lat_c, lon_c = (geo.north + south) / 2.0, (geo.west + east) / 2.0
    dl, dn = 0.3 * (geo.north - south), 0.3 * (east - geo.west)
    pts = [(lat_c, lon_c), (lat_c + dl, lon_c + dn), (lat_c + dl, lon_c - dn), (lat_c - dl, lon_c + dn),
           (lat_c - dl, lon_c - dn)]
    return pts


def tile_header(name: str, dn: np.ndarray, geo: Geo, dn_scale_m: float, *, global_longitude: bool, extra: dict) -> str:
    checks = []
    for lat, lon in check_points(geo, 0.0, 0.0) if not global_longitude else GLOBAL_CHECKS:
        h = resample_grid(dn, geo, np.array([lat]), np.array([lon]), wrap=global_longitude, scale=dn_scale_m)[0, 0]
        checks.append((lat, lon, float(h)))
    return grid_file.header_text(
        f"Earth height tile {name} - written by tools/data_pipeline/earth_terrain.py",
        product=extra["product"], height_reference="wgs84_ellipsoid", dn_scale_m=dn_scale_m, heights=f"{name}.i16",
        lines=geo.lines, samples=geo.samples, north_latitude_deg=geo.north, west_longitude_deg=geo.west,
        step_latitude_deg=geo.step_lat, step_longitude_deg=geo.step_lon, global_longitude=global_longitude,
        checks=checks, extra={k: v for k, v in extra.items() if k != "product"})


GLOBAL_CHECKS = [(25.9972, -97.1561), (28.6082, -80.6041), (28.5619, -80.5772), (31.35, -75.45), (28.45, -77.55),
                 (31.2, -121.3), (10.4, -150.6), (-30.3, 80.6), (45.5, -30.4), (-45.45, -120.35), (0.0, 179.99),
                 (0.0, -179.99), (46.0, 8.0), (27.99, 86.93)]


def write_tile(out_dir: Path, name: str, heights_m: np.ndarray, geo: Geo, dn_scale_m: float, *, global_longitude: bool,
               extra: dict) -> int:
    out_dir.mkdir(parents=True, exist_ok=True)
    dn = to_dn(heights_m, dn_scale_m)
    (out_dir / f"{name}.i16").write_bytes(dn.tobytes())
    (out_dir / f"{name}.toml").write_text(
        tile_header(name, dn, geo, dn_scale_m, global_longitude=global_longitude, extra=extra), encoding="utf-8")
    return dn.nbytes


# ----------------------------------------------------------------------------------------------- sources

def _download_cached(url: str, target: Path, download, force: bool) -> bool:
    """Download unless cached; False when the server has no such file (HTTP 403/404)."""
    if target.exists() and not force:
        return True
    try:
        download(url, target)
    except urllib.error.HTTPError as err:
        if err.code in (403, 404):
            return False
        raise
    return True


def read_etopo_nodes(tif: Path, geoid_cm: np.ndarray, step_arcmin: float) -> tuple[np.ndarray, Geo]:
    import rasterio

    if step_arcmin != 2.0:
        raise ValueError("only the 2' relief step is implemented")
    with rasterio.open(tif) as ds:
        t = ds.transform
        if (ds.shape != (10800, 21600) or abs(t.a - 1 / 60) > 1e-12 or abs(t.e + 1 / 60) > 1e-12
                or abs(t.c + 180.0) > 1e-9 or abs(t.f - 90.0) > 1e-9):
            raise RuntimeError(f"{tif}: unexpected ETOPO grid {ds.shape} {t}")
        raster = ds.read(1)
    if not np.isfinite(raster).all():
        raise RuntimeError("ETOPO: non-finite values")
    nodes = resample_global(raster, ETOPO_PIXELS_PER_NODE)
    geo = Geo(north=90.0, west=-180.0, step_lat=1 / 30.0, step_lon=1 / 30.0, lines=nodes.shape[0], samples=nodes.shape[1])
    return orthometric_to_ellipsoidal(nodes, geo.lats(), geo.lons(), geoid_cm), geo


def glo30_tile_name(lat_floor: int, lon_floor: int) -> str:
    ns, ew = ("N" if lat_floor >= 0 else "S"), ("E" if lon_floor >= 0 else "W")
    return f"Copernicus_DSM_COG_10_{ns}{abs(lat_floor):02d}_00_{ew}{abs(lon_floor):03d}_00_DEM"


def read_glo30_mosaic(geo: Geo, raw_dir: Path, download, force: bool) -> tuple[np.ndarray, Geo, list[str]]:
    """GLO-30 tiles covering `geo` (+ a pixel margin) as one orthometric mosaic (pixel centres = its nodes). Tiles the
    bucket does not hold (open ocean) are 0 m, i.e. the geoid."""
    import rasterio

    margin = 2 * GLO30_STEP_DEG
    lat_range = range(math.floor(geo.south - margin), math.floor(geo.north + margin) + 1)
    lon_range = range(math.floor(geo.west - margin), math.floor(geo.east + margin) + 1)
    top, left = max(lat_range) + 1, min(lon_range)
    px = round(1.0 / GLO30_STEP_DEG)
    mosaic = np.zeros((len(lat_range) * px, len(lon_range) * px), dtype=np.float32)
    mosaic_geo = Geo(north=top - GLO30_STEP_DEG / 2, west=left + GLO30_STEP_DEG / 2, step_lat=GLO30_STEP_DEG,
                     step_lon=GLO30_STEP_DEG, lines=mosaic.shape[0], samples=mosaic.shape[1])
    names, missing = [], []
    for lat in lat_range:
        for lon in lon_range:
            name = glo30_tile_name(lat, lon)
            tif = raw_dir / f"{name}.tif"
            if not _download_cached(GLO30_URL.format(name=name), tif, download, force):
                missing.append(name)
                continue
            with rasterio.open(tif) as ds:
                if ds.shape != (px, px):
                    raise RuntimeError(f"{tif}: shape {ds.shape}, expected {(px, px)} (tile north of 50 N not supported)")
                data = ds.read(1)
                if ds.nodata is not None:
                    data = np.where(data == ds.nodata, 0.0, data)
            i, j = top - 1 - lat, lon - left
            mosaic[i * px:(i + 1) * px, j * px:(j + 1) * px] = np.nan_to_num(data, nan=0.0)
            names.append(name)
    if missing:
        print(f"  {len(missing)} GLO-30 tile(s) absent from the bucket (open sea, taken as 0 m geoid): {', '.join(missing)}")
    return mosaic, mosaic_geo, names


def tnm_1m_items(geo: Geo) -> list[dict]:
    """3DEP 1 m tiles intersecting the grid extent, newest first, one per download URL."""
    items, offset, total = [], 0, None
    bbox = f"{geo.west},{geo.south},{geo.east},{geo.north}"
    while total is None or offset < total:
        query = urllib.parse.urlencode({"datasets": TNM_DATASET, "bbox": bbox, "max": 100, "offset": offset,
                                        "outputFormat": "JSON"})
        request = urllib.request.Request(f"{TNM_API}?{query}", headers={"User-Agent": "starsim-data-pipeline"})
        with urllib.request.urlopen(request, timeout=120) as response:
            page = json.load(response)
        total = page.get("total", 0)
        items += page.get("items", [])
        offset += 100
        if not page.get("items"):
            break
    unique = {it["downloadURL"]: it for it in items if it.get("downloadURL", "").lower().endswith(".tif")}
    return sorted(unique.values(), key=lambda it: it.get("lastUpdated", ""), reverse=True)


def read_3dep_local(geo: Geo, items: list[dict], raw_dir: Path, download, force: bool) -> np.ndarray:
    """3DEP tiles reprojected (bilinear) onto the local geographic grid, NAVD88 heights, NaN where no tile has data.
    Newer tiles win; older ones only fill what remains."""
    import rasterio
    from rasterio.transform import from_origin
    from rasterio.warp import Resampling, reproject

    acc = np.full((geo.lines, geo.samples), np.nan, dtype=np.float32)
    dst_transform = from_origin(geo.west - geo.step_lon / 2, geo.north + geo.step_lat / 2, geo.step_lon, geo.step_lat)
    for it in items:
        tif = raw_dir / Path(urllib.parse.urlparse(it["downloadURL"]).path).name
        _download_cached(it["downloadURL"], tif, download, force)
        tmp = np.full(acc.shape, np.nan, dtype=np.float32)
        with rasterio.open(tif) as ds:
            nodata = ds.nodata if ds.nodata is not None else -999999.0
            reproject(source=rasterio.band(ds, 1), destination=tmp, src_nodata=nodata, dst_nodata=np.nan,
                      dst_transform=dst_transform, dst_crs="EPSG:4326", resampling=Resampling.bilinear)
        gain = np.isnan(acc) & ~np.isnan(tmp)
        acc[gain] = tmp[gain]
        print(f"  {tif.name}: {int(gain.sum())} nodes taken")
        del tmp
    return acc


# ----------------------------------------------------------------------------------------------- orchestration

def fetch(source: dict, repo_root: Path, download, force: bool = False) -> None:
    import geoid

    cfg = load_config(repo_root / "config" / "earth_terrain.toml")
    out_dir = repo_root / "data" / "terrain" / "earth"
    raw_dir = repo_root / "data" / "raw" / "earth"
    geoid_dir = repo_root / "data" / "geoid"
    if not (geoid_dir / "egm2008_25.i16").exists():
        raise RuntimeError("fetch the geoid source first: python tools/data_pipeline/fetch.py geoid_egm2008")
    geoid_cm = geoid.load_fine_cm(geoid_dir)
    sizes: dict[str, int] = {}

    name = "global_relief"
    if (out_dir / f"{name}.toml").exists() and not force:
        print(f"  {name}: already present")
    else:
        tif = raw_dir / "etopo_2022_60s_surface.tif"
        _download_cached(source["url"], tif, download, force)
        h, geo = read_etopo_nodes(tif, geoid_cm, cfg["global"]["relief_step_arcmin"])
        sizes[name] = write_tile(out_dir, name, h, geo, DN_GLOBAL_M, global_longitude=True, extra={
            "product": "ETOPO_2022_surface_2min", "level": "global", "area": "global",
            "source": "NOAA NCEI ETOPO 2022 60s surface, 2x2 box average + EGM2008",
            "note": "ice surface and ocean bathymetry; heights ellipsoidal (EGM2008 undulation added)"})
        del h
        print(f"  {name}: {sizes[name] / 1e6:.1f} MB, {geo.lines} x {geo.samples}")

    geoid18 = raw_dir / "us_noaa_g2018u0.tif"
    for area in cfg["area"]:
        aid = area["id"]
        lat, lon = area["latitude_deg"], area["longitude_deg"]
        regional_name, local_name = f"{aid}_regional", f"{aid}_local"
        if all((out_dir / f"{n}.toml").exists() for n in (regional_name, local_name)) and not force:
            print(f"  {aid}: already present")
            continue
        # regional: GLO-30 surface
        rgeo = grid_for_box(lat, lon, area["regional_half_size_m"], GLO30_STEP_DEG, GLO30_STEP_DEG)
        mosaic, mgeo, names = read_glo30_mosaic(rgeo, raw_dir, download, force)
        regional = resample_grid(mosaic, mgeo, rgeo.lats(), rgeo.lons(), dtype=np.float32)
        del mosaic
        regional = orthometric_to_ellipsoidal(regional, rgeo.lats(), rgeo.lons(), geoid_cm)
        common = {"area": aid, "area_latitude_deg": lat, "area_longitude_deg": lon}
        sizes[regional_name] = write_tile(out_dir, regional_name, regional, rgeo, DN_REGIONAL_M, global_longitude=False,
                                          extra={**common, "product": "Copernicus_GLO-30", "level": "regional",
                                                 "half_size_m": area["regional_half_size_m"],
                                                 "source": "Copernicus DEM GLO-30 + EGM2008",
                                                 "note": "surface model: vegetation and buildings included",
                                                 "tiles": ",".join(names)})
        print(f"  {regional_name}: {sizes[regional_name] / 1e6:.1f} MB, {rgeo.lines} x {rgeo.samples}")
        # local: 3DEP bare earth, GLO-30 where 3DEP has no data or no coverage
        lgeo = grid_for_box(lat, lon, area["local_half_size_m"], LOCAL_STEP_LAT_DEG,
                            LOCAL_STEP_LAT_DEG / math.cos(math.radians(lat)))
        items = tnm_1m_items(lgeo)
        print(f"  {aid}: {len(items)} 3DEP 1 m tile(s) in the local box")
        extra = {**common, "level": "local", "half_size_m": area["local_half_size_m"]}
        local = None
        if items:
            _download_cached(source["geoid18_url"], geoid18, download, force)
            local = read_3dep_local(lgeo, items, raw_dir, download, force)
            valid = float(np.mean(~np.isnan(local)))
            print(f"  {aid}: 3DEP covers {valid * 100:.2f} % of the local nodes")
            if valid < MIN_3DEP_COVERAGE:
                local = None
        if local is None:
            print(f"  {aid}: no usable 3DEP 1 m coverage, local level from GLO-30 (fallback)")
            h = resample_grid(regional, rgeo, lgeo.lats(), lgeo.lons(), dtype=np.float32)
            extra.update(product="Copernicus_GLO-30", source="Copernicus DEM GLO-30 + EGM2008 (fallback: no 3DEP 1 m coverage)",
                         note="surface model: vegetation and buildings included")
            seam = None
        else:
            valid_mask = ~np.isnan(local)
            local = navd88_grid_to_ellipsoidal(local, lgeo, geoid18)
            reg_at_local = resample_grid(regional, rgeo, lgeo.lats(), lgeo.lons(), dtype=np.float32)
            diff = (local - reg_at_local)[valid_mask]
            seam = (float(diff.mean()), float(np.sqrt(np.mean(diff**2))), float(np.median(diff)))
            h, filled = fill_nodata_from_regional(local, lgeo, regional, rgeo)
            print(f"  {aid}: {filled} nodes ({filled / h.size * 100:.2f} %) filled from GLO-30; seam local - regional over "
                  f"the 3DEP nodes: mean {seam[0]:+.3f} m, median {seam[2]:+.3f} m, rms {seam[1]:.3f} m")
            extra.update(product="USGS_3DEP_1m", source="USGS 3DEP 1 m DEM (NAVD88, GEOID18) + GLO-30 for gaps",
                         note="bare earth; gaps filled from the regional GLO-30 surface",
                         coverage_fraction=round(float(valid_mask.mean()), 6), seam_mean_m=round(seam[0], 4),
                         seam_median_m=round(seam[2], 4), seam_rms_m=round(seam[1], 4), tiles=",".join(
                             Path(urllib.parse.urlparse(it["downloadURL"]).path).name for it in items))
            del reg_at_local, diff
        sizes[local_name] = write_tile(out_dir, local_name, h, lgeo, DN_LOCAL_M, global_longitude=False, extra=extra)
        print(f"  {local_name}: {sizes[local_name] / 1e6:.1f} MB, {lgeo.lines} x {lgeo.samples}")
    print("  Earth terrain tiles ->", out_dir)
