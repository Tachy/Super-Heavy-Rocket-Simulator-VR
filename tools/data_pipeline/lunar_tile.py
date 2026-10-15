"""LOLA terrain for the lunar landing sites (lunar l2).

Site tiles: crop of the south polar stereographic LDEM_80S_20M (20 m/px, 80-90 S) around each site of
config/lunar_sites.toml, fetched with one HTTP range request over the rows it needs (the full product is 1.8 GB).
Global grid: LDEM_16 (16 px/deg, 33 MB) for the terrain outside the tiles. Output in data/terrain/: a TOML header and
int16 heights (DN; height above the 1737.4 km sphere = DN x dn_scale_m), read by core/src/config/lunar_site_config.cpp.

Projections (LOLA dsmap_polar.cat [S], spherical R = 1737.4 km; 0-based indices):
  south polar stereographic: rho = 2 R tan(pi/4 + lat/2), X = rho sin(lon), Y = -rho cos(lon),
  sample = pole_sample + X / scale, line = pole_line + Y / scale (longitude 0 up from the south pole, east clockwise
  as seen from below);
  simple cylindrical (LDEM_16): line = zero_line - ppd lat_deg, sample = zero_sample + ppd lon_deg (lon 0..360 east).
"""

from __future__ import annotations

import math
import re
import struct
import sys
import tomllib
import urllib.request
from pathlib import Path

SPHERE_RADIUS_M = 1737400.0  # LOLA reference sphere [S: label OFFSET, A_AXIS_RADIUS]
ORIENTATION_RMS_LIMIT_M = 150.0  # [E] tile vs LDEM_16 (1.9 km pixels in rough polar terrain)
USER_AGENT = {"User-Agent": "starsim-data-pipeline"}


# ----------------------------------------------------------------------------------------------- pure functions

def parse_label(text: str) -> dict:
    """PDS3 label keywords: numbers (unit suffix dropped) -> float, quoted / bare words -> str. First occurrence wins."""
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    label: dict = {}
    for line in text.splitlines():
        m = re.match(r"^\s*([A-Z0-9_^]+)\s*=\s*(.*?)\s*$", line)
        if not m:
            continue
        key, value = m.group(1), m.group(2)
        if key in label:
            continue
        number = re.match(r"^([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)\s*(<.*>)?$", value)
        if number:
            label[key] = float(number.group(1))
        else:
            label[key] = value.strip('"').strip("'")
    return label


def polar_south_line_sample(lat: float, lon: float, pole_line: float, pole_sample: float,
                            scale_m: float, radius_m: float = SPHERE_RADIUS_M) -> tuple[float, float]:
    rho = 2.0 * radius_m * math.tan(math.pi / 4.0 + lat / 2.0)
    x = rho * math.sin(lon)
    y = -rho * math.cos(lon)
    return pole_line + y / scale_m, pole_sample + x / scale_m


def polar_south_lat_lon(line: float, sample: float, pole_line: float, pole_sample: float,
                        scale_m: float, radius_m: float = SPHERE_RADIUS_M) -> tuple[float, float]:
    x = (sample - pole_sample) * scale_m
    y = (line - pole_line) * scale_m
    rho = math.hypot(x, y)
    return -math.pi / 2.0 + 2.0 * math.atan(rho / (2.0 * radius_m)), math.atan2(x, -y)


def cylindrical_line_sample(lat: float, lon: float, ppd: float, zero_line: float,
                            zero_sample: float) -> tuple[float, float]:
    return zero_line - ppd * math.degrees(lat), zero_sample + ppd * (math.degrees(lon) % 360.0)


def crop_rows(rows: bytes, samples: int, first_sample: int, count: int) -> list[list[int]]:
    """Rows of int16 LE (`samples` each) cropped to columns first_sample .. first_sample + count - 1."""
    row_bytes = 2 * samples
    out = []
    for k in range(len(rows) // row_bytes):
        start = k * row_bytes + 2 * first_sample
        out.append(list(struct.unpack(f"<{count}h", rows[start:start + 2 * count])))
    return out


def bilinear(tile: list[list[int]], line: float, sample: float) -> float:
    """Bilinear value (DN) inside the tile; nodes exactly."""
    l0 = min(int(math.floor(line)), len(tile) - 2)
    s0 = min(int(math.floor(sample)), len(tile[0]) - 2)
    fl, fs = line - l0, sample - s0
    return ((1 - fl) * ((1 - fs) * tile[l0][s0] + fs * tile[l0][s0 + 1])
            + fl * ((1 - fs) * tile[l0 + 1][s0] + fs * tile[l0 + 1][s0 + 1]))


# ----------------------------------------------------------------------------------------------- downloads

def _get(url: str, byte_range: tuple[int, int] | None = None) -> bytes:
    headers = dict(USER_AGENT)
    if byte_range:
        headers["Range"] = f"bytes={byte_range[0]}-{byte_range[1]}"
    with urllib.request.urlopen(urllib.request.Request(url, headers=headers)) as response:
        if byte_range and response.status != 206:
            raise RuntimeError(f"{url}: server ignored the range request (HTTP {response.status}); "
                               "refusing to download the whole product")
        return response.read()


def _label_url(img_url: str) -> str:
    return img_url[: -len(".img")] + ".lbl"


def _require(label: dict, key: str, expected) -> None:
    if label.get(key) != expected:
        raise RuntimeError(f"label {key} = {label.get(key)!r}, expected {expected!r}")


def _write_i16(path: Path, values: list[int]) -> None:
    path.write_bytes(struct.pack(f"<{len(values)}h", *values))


def fetch_global(source: dict, out_dir: Path) -> None:
    label = parse_label(_get(_label_url(source["url"])).decode("latin-1"))
    _require(label, "SAMPLE_TYPE", "LSB_INTEGER")
    _require(label, "SAMPLE_BITS", 16.0)
    _require(label, "MAP_PROJECTION_TYPE", "SIMPLE CYLINDRICAL")
    _require(label, "OFFSET", SPHERE_RADIUS_M)
    lines, samples = int(label["LINES"]), int(label["LINE_SAMPLES"])
    ppd = label["MAP_RESOLUTION"]
    data = _get(source["url"])
    if len(data) != 2 * lines * samples:
        raise RuntimeError(f"{source['url']}: {len(data)} bytes, expected {2 * lines * samples}")
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "ldem_16.i16").write_bytes(data)
    # SAMPLE_PROJECTION_OFFSET is the sample of CENTER_LONGITUDE: sample of longitude 0 = offset - ppd x centre
    zero_sample = label["SAMPLE_PROJECTION_OFFSET"] - ppd * label["CENTER_LONGITUDE"]
    (out_dir / "ldem_16.toml").write_text(
        "# LOLA LDEM_16 (global, simple cylindrical) - written by tools/data_pipeline/lunar_tile.py\n"
        "[grid]\n"
        'product = "LDEM_16"\n'
        'projection = "simple_cylindrical"\n'
        f"sphere_radius_m = {SPHERE_RADIUS_M}\n"
        f"dn_scale_m = {label['SCALING_FACTOR']}\n"
        'heights = "ldem_16.i16"\n'
        f"lines = {lines}\n"
        f"samples = {samples}\n"
        f"pixels_per_degree = {ppd}\n"
        f"zero_line = {label['LINE_PROJECTION_OFFSET']}\n"
        f"zero_sample = {zero_sample}\n", encoding="utf-8")
    print(f"  LDEM_16 {lines} x {samples} -> {out_dir / 'ldem_16.i16'}")


def fetch_site_tile(site: dict, source: dict, out_dir: Path) -> None:
    label = parse_label(_get(_label_url(source["url"])).decode("latin-1"))
    _require(label, "SAMPLE_TYPE", "LSB_INTEGER")
    _require(label, "SAMPLE_BITS", 16.0)
    _require(label, "MAP_PROJECTION_TYPE", "POLAR STEREOGRAPHIC")
    _require(label, "CENTER_LATITUDE", -90.0)
    _require(label, "A_AXIS_RADIUS", SPHERE_RADIUS_M / 1000.0)
    _require(label, "OFFSET", SPHERE_RADIUS_M)
    full_lines, full_samples = int(label["LINES"]), int(label["LINE_SAMPLES"])
    scale = label["MAP_SCALE"]
    pole_line, pole_sample = label["LINE_PROJECTION_OFFSET"], label["SAMPLE_PROJECTION_OFFSET"]
    lat, lon = math.radians(site["latitude_deg"]), math.radians(site["longitude_deg"])
    line, sample = polar_south_line_sample(lat, lon, pole_line, pole_sample, scale)
    half = round(site["tile_half_size_m"] / scale)
    first_line, first_sample = round(line) - half, round(sample) - half
    count = 2 * half + 1
    if first_line < 0 or first_sample < 0 or first_line + count > full_lines or first_sample + count > full_samples:
        raise RuntimeError(f"site {site['id']}: tile outside {source['id']}")
    row_bytes = 2 * full_samples
    print(f"  {site['id']}: rows {first_line}..{first_line + count - 1} ({count * row_bytes / 1e6:.1f} MB)")
    rows = _get(source["url"], (first_line * row_bytes, (first_line + count) * row_bytes - 1))
    tile = crop_rows(rows, full_samples, first_sample, count)
    out_dir.mkdir(parents=True, exist_ok=True)
    _write_i16(out_dir / f"{site['id']}.i16", [v for row in tile for v in row])
    dn_scale = label["SCALING_FACTOR"]
    checks = []
    for fl, fs in [(0, 0), (-0.4, -0.4), (-0.4, 0), (-0.4, 0.4), (0, -0.4), (0, 0.4), (0.4, -0.4), (0.4, 0), (0.4, 0.37)]:
        cl, cs = line + fl * half, sample + fs * half
        c_lat, c_lon = polar_south_lat_lon(cl, cs, pole_line, pole_sample, scale)
        checks.append((math.degrees(c_lat), math.degrees(c_lon),
                       dn_scale * bilinear(tile, cl - first_line, cs - first_sample)))
    text = (f"# LOLA {source['id']} tile around {site['name']} - written by tools/data_pipeline/lunar_tile.py\n"
            "[grid]\n"
            f'product = "{site["product"]}"\n'
            'projection = "polar_stereographic_south"\n'
            f"sphere_radius_m = {SPHERE_RADIUS_M}\n"
            f"dn_scale_m = {dn_scale}\n"
            f'heights = "{site["id"]}.i16"\n'
            f"lines = {count}\n"
            f"samples = {count}\n"
            f"pole_line = {pole_line}\n"
            f"pole_sample = {pole_sample}\n"
            f"scale_m = {scale}\n"
            f"first_line = {first_line}\n"
            f"first_sample = {first_sample}\n")
    for c_lat, c_lon, h in checks:
        text += f"\n[[check]]\nlatitude_deg = {c_lat!r}\nlongitude_deg = {c_lon!r}\nheight_m = {h!r}\n"
    (out_dir / f"{site['id']}.toml").write_text(text, encoding="utf-8")
    finish_tile(site["id"], out_dir)


def finish_tile(site_id: str, out_dir: Path) -> None:
    """Orientation check of a written tile; a rejected tile is removed so fetch.py does not take it as present."""
    try:
        check_orientation(site_id, out_dir)
    except Exception:
        for suffix in (".toml", ".i16"):
            (out_dir / f"{site_id}{suffix}").unlink(missing_ok=True)
        raise


# ----------------------------------------------------------------------------------------------- orientation check

def _load_grid(out_dir: Path, name: str) -> tuple[dict, list[list[int]]]:
    with (out_dir / f"{name}.toml").open("rb") as f:
        grid = tomllib.load(f)["grid"]
    data = (out_dir / grid["heights"]).read_bytes()
    n = grid["samples"]
    values = struct.unpack(f"<{grid['lines'] * n}h", data)
    return grid, [list(values[k * n:(k + 1) * n]) for k in range(grid["lines"])]


def check_orientation(site_id: str, out_dir: Path) -> float:
    """rms (m) of the tile against LDEM_16 over a 21 x 21 lattice; raises if a mirrored tile would fit as well."""
    tile_grid, tile = _load_grid(out_dir, site_id)
    glob_grid, glob = _load_grid(out_dir, "ldem_16")
    count = tile_grid["lines"]

    def global_height(lat: float, lon: float) -> float:
        line, sample = cylindrical_line_sample(lat, lon, glob_grid["pixels_per_degree"], glob_grid["zero_line"],
                                               glob_grid["zero_sample"])
        line = min(max(line, 0.0), glob_grid["lines"] - 1.0)
        s0 = math.floor(sample)
        fs = sample - s0
        l0 = min(int(math.floor(line)), glob_grid["lines"] - 2)
        fl = line - l0
        n = glob_grid["samples"]
        a, b = s0 % n, (s0 + 1) % n
        v = ((1 - fl) * ((1 - fs) * glob[l0][a] + fs * glob[l0][b])
             + fl * ((1 - fs) * glob[l0 + 1][a] + fs * glob[l0 + 1][b]))
        return glob_grid["dn_scale_m"] * v

    def rms(transform) -> float:
        total, n = 0.0, 0
        for i in range(21):
            for j in range(21):
                tl, ts = i * (count - 1) / 20.0, j * (count - 1) / 20.0
                sl, ss = transform(tl, ts)
                lat, lon = polar_south_lat_lon(tile_grid["first_line"] + tl, tile_grid["first_sample"] + ts,
                                               tile_grid["pole_line"], tile_grid["pole_sample"], tile_grid["scale_m"])
                d = tile_grid["dn_scale_m"] * bilinear(tile, sl, ss) - global_height(lat, lon)
                total += d * d
                n += 1
        return math.sqrt(total / n)

    last = count - 1
    correct = rms(lambda l, s: (l, s))
    alternatives = {"lines flipped": rms(lambda l, s: (last - l, s)),
                    "samples flipped": rms(lambda l, s: (l, last - s)),
                    "transposed": rms(lambda l, s: (s, l))}
    print(f"  orientation vs LDEM_16: rms {correct:.1f} m; "
          + ", ".join(f"{k} {v:.1f} m" for k, v in alternatives.items()))
    if correct > ORIENTATION_RMS_LIMIT_M or correct > 0.5 * min(alternatives.values()):
        raise RuntimeError(f"tile {site_id} does not match LDEM_16 (rms {correct:.1f} m, alternatives {alternatives})")
    return correct


# ----------------------------------------------------------------------------------------------- entry point

_done = False


def fetch_all(sources: list[dict], repo_root: Path, force: bool = False) -> None:
    """Global grid first, then every site tile of config/lunar_sites.toml that is missing (or all with `force`)."""
    global _done
    if _done:
        return
    _done = True
    out_dir = repo_root / "data" / "terrain"
    by_id = {s["id"]: s for s in sources}
    if force or not (out_dir / "ldem_16.toml").exists():
        fetch_global(by_id["lola_ldem_16"], out_dir)
    else:
        print("  LDEM_16 already present")
    with (repo_root / "config" / "lunar_sites.toml").open("rb") as f:
        sites = tomllib.load(f)["site"]
    product_source = {"LDEM_80S_20M": by_id["lola_ldem_80s_20m"]}
    for site in sites:
        if not force and (out_dir / f"{site['id']}.toml").exists():
            print(f"  {site['id']} already present")
            continue
        fetch_site_tile(site, product_source[site["product"]], out_dir)


if __name__ == "__main__":
    sys.exit("use tools/data_pipeline/fetch.py")
