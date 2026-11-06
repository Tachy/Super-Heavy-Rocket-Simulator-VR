"""LOLA terrain for the lunar landing sites.

Global grid: LDEM_64 (64 px/deg, 473 m, 531 MB, fetched whole) for the terrain outside the tiles -> data/terrain/ldem_64.*.
Site tiles: the core CLI build/core/lunar_site_tile.exe plans the HTTP byte ranges of a site's height and albedo tile
from config/lunar_terrain_products.toml (`plan`), this module downloads them (<= 32 parallel connections, HTTP 429
Retry-After honoured by stopping) and the CLI assembles the tile (`assemble`) -> data/terrain/sites/<cache key>.{toml,i16}.
Output: a TOML header and int16 DN (height above the 1737.4 km sphere = DN x dn_scale_m; albedo = relative factor), read
by core/src/config/lunar_site_config.cpp.

Projections (LOLA dsmap_polar.cat [S], spherical R = 1737.4 km; 0-based indices):
  south polar stereographic: rho = 2 R tan(pi/4 + lat/2), X = rho sin(lon), Y = -rho cos(lon),
  sample = pole_sample + X / scale, line = pole_line + Y / scale (longitude 0 up from the south pole, east clockwise
  as seen from below);
  simple cylindrical (LDEM_64): line = zero_line - ppd lat_deg, sample = zero_sample + ppd lon_deg (lon 0..360 east).
"""

from __future__ import annotations

import json
import math
import mmap
import re
import struct
import subprocess
import sys
import tomllib
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

SPHERE_RADIUS_M = 1737400.0  # LOLA reference sphere [S: label OFFSET, A_AXIS_RADIUS]
ORIENTATION_RMS_LIMIT_M = 150.0  # [E] tile vs LDEM_64 (0.47 km pixels in rough polar terrain)
CLI_EXE = Path("build/core/lunar_site_tile.exe")
MAX_CONNECTIONS = 16  # PDS Geosciences: 403 at 64 connections (task 1) and, measured 2026-11-06, after ~300 small ranges at 32; 16 ran 600 ranges clean
GET_ATTEMPTS = 3
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

class RateLimited(RuntimeError):
    """HTTP 429: the host asks to wait `retry_after_s` seconds (None when it sent no Retry-After)."""

    def __init__(self, url: str, retry_after_s: int | None):
        super().__init__(f"{url}: HTTP 429, Retry-After {retry_after_s} s")
        self.url = url
        self.retry_after_s = retry_after_s


def _open(url: str, byte_range: tuple[int, int] | None):
    headers = dict(USER_AGENT)
    if byte_range:
        headers["Range"] = f"bytes={byte_range[0]}-{byte_range[1]}"
    try:
        return urllib.request.urlopen(urllib.request.Request(url, headers=headers), timeout=120)
    except urllib.error.HTTPError as error:
        if error.code == 429:  # never retried: the caller reports it and stops
            retry_after = error.headers.get("Retry-After") if error.headers else None
            raise RateLimited(url, int(retry_after) if retry_after and retry_after.isdigit() else None) from error
        raise


def _get(url: str, byte_range: tuple[int, int] | None = None) -> bytes:
    """The body of a URL (or of an inclusive byte range, which must come back as HTTP 206 and complete).
    Transient network errors are retried GET_ATTEMPTS times; HTTP 429 raises RateLimited at once, other HTTP errors too."""
    for attempt in range(GET_ATTEMPTS):
        try:
            with _open(url, byte_range) as response:
                if byte_range and response.status != 206:
                    raise RuntimeError(f"{url}: server ignored the range request (HTTP {response.status}); "
                                       "refusing to download the whole product")
                data = response.read()
            if byte_range and len(data) != byte_range[1] - byte_range[0] + 1:
                raise ConnectionError(f"{url}: short range body ({len(data)} bytes)")
            return data
        except (urllib.error.URLError, TimeoutError, ConnectionError) as error:
            if isinstance(error, urllib.error.HTTPError) or attempt == GET_ATTEMPTS - 1:
                raise
            print(f"  retry {attempt + 1}: {error}")
    raise AssertionError("unreachable")


def _download(url: str, target: Path) -> None:
    """Streams a whole file to `target` (through a .part file)."""
    target.parent.mkdir(parents=True, exist_ok=True)
    partial = target.with_name(target.name + ".part")
    with _open(url, None) as response, partial.open("wb") as out:
        while chunk := response.read(1 << 20):
            out.write(chunk)
    partial.replace(target)


def _label_url(img_url: str) -> str:
    return img_url[: -len(".img")] + ".lbl"


def _require(label: dict, key: str, expected) -> None:
    if label.get(key) != expected:
        raise RuntimeError(f"label {key} = {label.get(key)!r}, expected {expected!r}")


def fetch_global(source: dict, out_dir: Path) -> None:
    """The global grid of `source` whole -> out_dir/<name>.{i16,toml} (name = the .img file's stem, e.g. ldem_64)."""
    img_url = source["url"]
    name = img_url.rsplit("/", 1)[1][: -len(".img")]
    label = parse_label(_get(_label_url(img_url)).decode("latin-1"))
    _require(label, "SAMPLE_TYPE", "LSB_INTEGER")
    _require(label, "SAMPLE_BITS", 16.0)
    _require(label, "MAP_PROJECTION_TYPE", "SIMPLE CYLINDRICAL")
    _require(label, "OFFSET", SPHERE_RADIUS_M)
    lines, samples = int(label["LINES"]), int(label["LINE_SAMPLES"])
    ppd = label["MAP_RESOLUTION"]
    heights = out_dir / f"{name}.i16"
    _download(img_url, heights)
    if heights.stat().st_size != 2 * lines * samples:
        size = heights.stat().st_size
        heights.unlink()
        raise RuntimeError(f"{img_url}: {size} bytes, expected {2 * lines * samples}")
    # SAMPLE_PROJECTION_OFFSET is the sample of CENTER_LONGITUDE: sample of longitude 0 = offset - ppd x centre
    zero_sample = label["SAMPLE_PROJECTION_OFFSET"] - ppd * label["CENTER_LONGITUDE"]
    (out_dir / f"{name}.toml").write_text(
        f"# LOLA {name.upper()} (global, simple cylindrical) - written by tools/data_pipeline/lunar_tile.py\n"
        "[grid]\n"
        f'product = "{name.upper()}"\n'
        'projection = "simple_cylindrical"\n'
        f"sphere_radius_m = {SPHERE_RADIUS_M}\n"
        f"dn_scale_m = {label['SCALING_FACTOR']}\n"
        f'heights = "{name}.i16"\n'
        f"lines = {lines}\n"
        f"samples = {samples}\n"
        f"pixels_per_degree = {ppd}\n"
        f"zero_line = {label['LINE_PROJECTION_OFFSET']}\n"
        f"zero_sample = {zero_sample}\n", encoding="utf-8")
    print(f"  {name.upper()} {lines} x {samples} -> {heights}")


# ----------------------------------------------------------------------------------------------- orientation check

def _load_tile(tile_dir: Path, name: str) -> tuple[dict, list[list[int]]]:
    with (tile_dir / f"{name}.toml").open("rb") as f:
        grid = tomllib.load(f)["grid"]
    data = (tile_dir / grid["heights"]).read_bytes()
    n = grid["samples"]
    values = struct.unpack(f"<{grid['lines'] * n}h", data)
    return grid, [list(values[k * n:(k + 1) * n]) for k in range(grid["lines"])]


def check_orientation(tile_name: str, tile_dir: Path, global_dir: Path, global_name: str = "ldem_64") -> float | None:
    """rms (m) of a south polar height tile against the global grid over a 21 x 21 lattice; raises if a mirrored tile
    would fit as well. None for every other tile (north polar tiles: the core test "north polar tiles agree with
    LDEM_64"; cylindrical tiles are no reprojection of anything)."""
    with (tile_dir / f"{tile_name}.toml").open("rb") as f:
        header = tomllib.load(f)["grid"]
    if header["projection"] != "polar_stereographic_south" or header.get("kind", "height") != "height":
        return None
    tile_grid, tile = _load_tile(tile_dir, tile_name)
    with (global_dir / f"{global_name}.toml").open("rb") as f:
        glob_grid = tomllib.load(f)["grid"]
    count = tile_grid["lines"]
    n, glob_lines = glob_grid["samples"], glob_grid["lines"]
    with (global_dir / glob_grid["heights"]).open("rb") as gf, mmap.mmap(gf.fileno(), 0, access=mmap.ACCESS_READ) as glob:

        def node(line: int, sample: int) -> int:
            return struct.unpack_from("<h", glob, 2 * (line * n + sample % n))[0]

        def global_height(lat: float, lon: float) -> float:
            line, sample = cylindrical_line_sample(lat, lon, glob_grid["pixels_per_degree"], glob_grid["zero_line"],
                                                   glob_grid["zero_sample"])
            line = min(max(line, 0.0), glob_lines - 1.0)
            s0 = math.floor(sample)
            fs = sample - s0
            l0 = min(int(math.floor(line)), glob_lines - 2)
            fl = line - l0
            v = ((1 - fl) * ((1 - fs) * node(l0, s0) + fs * node(l0, s0 + 1))
                 + fl * ((1 - fs) * node(l0 + 1, s0) + fs * node(l0 + 1, s0 + 1)))
            return glob_grid["dn_scale_m"] * v

        def rms(transform) -> float:
            total, k = 0.0, 0
            for i in range(21):
                for j in range(21):
                    tl, ts = i * (count - 1) / 20.0, j * (count - 1) / 20.0
                    sl, ss = transform(tl, ts)
                    lat, lon = polar_south_lat_lon(tile_grid["first_line"] + tl, tile_grid["first_sample"] + ts,
                                                   tile_grid["pole_line"], tile_grid["pole_sample"], tile_grid["scale_m"])
                    d = tile_grid["dn_scale_m"] * bilinear(tile, sl, ss) - global_height(lat, lon)
                    total += d * d
                    k += 1
            return math.sqrt(total / k)

        last = count - 1
        correct = rms(lambda l, s: (l, s))
        alternatives = {"lines flipped": rms(lambda l, s: (last - l, s)),
                        "samples flipped": rms(lambda l, s: (l, last - s)),
                        "transposed": rms(lambda l, s: (s, l))}
    print(f"  orientation vs {global_name.upper()}: rms {correct:.1f} m; "
          + ", ".join(f"{k} {v:.1f} m" for k, v in alternatives.items()))
    if correct > ORIENTATION_RMS_LIMIT_M or correct > 0.5 * min(alternatives.values()):
        raise RuntimeError(f"tile {tile_name} does not match {global_name} (rms {correct:.1f} m, alternatives {alternatives})")
    return correct


def finish_tile(tile_name: str, tile_dir: Path, global_dir: Path) -> None:
    """Orientation check of a written tile; a rejected tile is removed so fetch.py does not take it as present."""
    try:
        check_orientation(tile_name, tile_dir, global_dir)
    except Exception:
        for suffix in (".toml", ".i16"):
            (tile_dir / f"{tile_name}{suffix}").unlink(missing_ok=True)
        raise


# ----------------------------------------------------------------------------------------------- site tiles through the CLI

def _run_cli(args: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(args, capture_output=True, text=True)


def _download_ranges(ranges: list[dict], work: Path) -> int:
    """Every range of the request -> work/<index>.bin, MAX_CONNECTIONS at a time; stops at the first error
    (RateLimited included). Returns the bytes loaded."""
    total = 0
    with ThreadPoolExecutor(max_workers=MAX_CONNECTIONS) as pool:
        futures = [(r, pool.submit(_get, r["url"], (r["first"], r["last"]))) for r in ranges]
        try:
            for r, future in futures:
                body = future.result()
                (work / f"{r['index']}.bin").write_bytes(body)
                total += len(body)
        except BaseException:
            pool.shutdown(wait=False, cancel_futures=True)
            raise
    return total


def fetch_site_tile(site: dict, kind: str, exe: Path, config_dir: Path, tile_dir: Path, global_dir: Path,
                    force: bool = False) -> str:
    """One tile (kind "height" | "albedo") of a site through the CLI: plan, download, assemble. Returns "ok", "present"
    or "refused" (the CLI found no product for the tile; ledger ruling R6: fatal for a height tile only, the caller
    decides). Raises RateLimited on HTTP 429."""
    plan = _run_cli([str(exe), "plan", str(config_dir), kind, repr(site["latitude_deg"]), repr(site["longitude_deg"])])
    if plan.returncode == 2:
        print(f"  {site['id']} {kind}: no product ({plan.stderr.strip()})")
        return "refused"
    if plan.returncode != 0:
        raise RuntimeError(f"lunar_site_tile plan failed for {site['id']} {kind}: {plan.stderr.strip()}")
    request = json.loads(plan.stdout)
    key = request["cache_key"]
    if not force and (tile_dir / f"{key}.toml").exists():
        print(f"  {site['id']} {kind}: {key} present")
        return "present"
    megabytes = sum(r["last"] - r["first"] + 1 for r in request["ranges"]) / 1e6
    print(f"  {site['id']} {kind}: {key}, {len(request['ranges'])} ranges, {megabytes:.1f} MB")
    work = tile_dir / f".{key}.work"  # unique per tile: parallel fetches never share it
    work.mkdir(parents=True, exist_ok=True)
    try:
        (work / "request.json").write_text(plan.stdout, encoding="utf-8")
        _download_ranges(request["ranges"], work)
        built = _run_cli([str(exe), "assemble", str(config_dir), str(work / "request.json"), str(work),
                          str(work / f"{key}.toml")])
        if built.returncode != 0:
            raise RuntimeError(f"lunar_site_tile assemble failed for {site['id']} {kind}: {built.stderr.strip()}")
        (work / f"{key}.i16").replace(tile_dir / f"{key}.i16")
        (work / f"{key}.toml").replace(tile_dir / f"{key}.toml")  # the header last: it marks the tile as present
    finally:
        for leftover in work.iterdir():
            leftover.unlink()
        work.rmdir()
    finish_tile(key, tile_dir, global_dir)
    return "ok"


def fetch_site_tiles(sites: list[dict], repo_root: Path, force: bool = False) -> None:
    """Height and albedo tile of every site. A missing height product is an error, an albedo refusal is not (the
    global albedo is used there); HTTP 429 reports the Retry-After and stops without retrying."""
    exe = repo_root / CLI_EXE
    if not exe.exists():
        raise SystemExit(f"{CLI_EXE} is missing: build the core first (tools/build.ps1)")
    tile_dir = repo_root / "data" / "terrain" / "sites"
    tile_dir.mkdir(parents=True, exist_ok=True)
    for site in sites:
        for kind in ("height", "albedo"):
            try:
                outcome = fetch_site_tile(site, kind, exe, repo_root / "config", tile_dir, repo_root / "data" / "terrain",
                                          force)
            except RateLimited as limited:
                wait = f"{limited.retry_after_s} s" if limited.retry_after_s is not None else "an unknown time"
                raise SystemExit(f"{limited.url} answered HTTP 429: the host asks to wait {wait} (Retry-After). "
                                 "Stopped; run fetch.py again after that time.") from limited
            if outcome == "refused" and kind == "height":
                raise SystemExit(f"site {site['id']}: no height product holds its tile")


# ----------------------------------------------------------------------------------------------- entry points

def fetch_global_grid(source: dict, repo_root: Path, force: bool = False) -> None:
    out_dir = repo_root / "data" / "terrain"
    name = source["url"].rsplit("/", 1)[1][: -len(".img")]
    if force or not (out_dir / f"{name}.toml").exists():
        fetch_global(source, out_dir)
    else:
        print(f"  {name.upper()} already present")


def fetch_sites(source: dict, repo_root: Path, force: bool = False) -> None:
    """The preset sites of config/lunar_sites.toml, or the source's own `sites` list (the opt-in test sites)."""
    if "sites" in source:
        sites = source["sites"]
    else:
        with (repo_root / "config" / "lunar_sites.toml").open("rb") as f:
            sites = tomllib.load(f)["site"]
    fetch_site_tiles(sites, repo_root, force)


if __name__ == "__main__":
    sys.exit("use tools/data_pipeline/fetch.py")
