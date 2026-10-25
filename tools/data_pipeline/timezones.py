"""Time zones at a position: zone grid + UTC-offset transitions (local time under the webcast UTC clock).

Sources:
  boundaries  timezone-boundary-builder "timezones-with-oceans" GeoJSON release (ODbL 1.0; land zones from
              OpenStreetMap, the oceans as the nautical zones Etc/GMT+-N)
  rules       IANA tzdata (public domain) through the Python `tzdata` package (its IANA_VERSION is recorded)

Output (git-ignored, data/timezones/):
  zone_grid.bin   the zone index of every cell of a global lat/lon grid, run-length encoded per row
                  (little-endian):
                    char[4] "STZG", u32 version (1), u32 columns, u32 rows,
                    f64 west_longitude_deg, f64 north_latitude_deg, f64 step_deg,
                    u32 row_start[rows + 1]   index of each row's first run (runs of row r: row_start[r] ..
                                              row_start[r + 1] - 1),
                    runs: u16 first_column, u16 zone_index   (columns first_column .. next run's first - 1)
                  Cell (r, c) covers latitudes north - (r + 1) * step .. north - r * step and longitudes
                  west + c * step .. west + (c + 1) * step; its zone is the one containing the cell centre
                  (nearest-cell lookup: a point is up to step / 2 from its cell centre, ~1.1 km at 0.02 deg).
  zones.json      {"tzdata": "<IANA version>", "boundaries": "<release>", "grid_step_deg", "from_unix", "to_unix",
                   "zones": [{"id", "transitions": [[unix_s, offset_s, abbreviation], ...]}, ...]}
                  zone index = position in "zones"; the first transition of every zone is at from_unix (the state
                  then), the others are exact (to the second) up to to_unix. abbreviation "" where tzdata has only
                  a numeric designation ("+0530", "-03"): the game then shows the UTC offset alone.

Transitions: the TZif file's explicit transitions (exact instants) and, past its last one, the POSIX rule in its
footer evaluated through zoneinfo (daily sampling + bisection to the second; rule-based changes are never closer than
a day). Usage: python tools/data_pipeline/fetch.py timezones (after the download), or
python tools/data_pipeline/timezones.py [--force]
"""

from __future__ import annotations

import argparse
import datetime as dt
import importlib.resources
import json
import re
import struct
import sys
import zipfile
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
RAW_DIR = Path("data/raw/timezones")
OUT_DIR = Path("data/timezones")
GRID_FILE = "zone_grid.bin"
ZONES_FILE = "zones.json"
GRID_MAGIC = b"STZG"
GRID_VERSION = 1
# 0.02 deg = 2.2 km north-south: the borders near the sites (Texas / Mexico, Florida's Eastern / Central line) resolve
# to ~1 km; 18000 x 9000 cells, a few MB run-length encoded.
STEP_DEG = 0.02
WEST_LONGITUDE_DEG = -180.0
NORTH_LATITUDE_DEG = 90.0
FROM_UTC = dt.datetime(1990, 1, 1, tzinfo=dt.timezone.utc)
TO_UTC = dt.datetime(2061, 1, 1, tzinfo=dt.timezone.utc)
DAY_S = 86400
BAND_ROWS = 1500  # rasterised in latitude bands (memory)
NUMERIC_DESIGNATION = re.compile(r"^[+-]\d{2,4}$")


def release_tag(source: dict) -> str:
    """The boundary release tag from the download URL (.../releases/download/<tag>/...)."""
    match = re.search(r"/download/([^/]+)/", source["url"])
    return match.group(1) if match else "unknown"


def raw_zip(source: dict) -> Path:
    return RAW_DIR / f"timezones-with-oceans-{release_tag(source)}.geojson.zip"


def fetch(source: dict, repo_root: Path, download, force: bool) -> None:
    archive = repo_root / raw_zip(source)
    if force or not archive.exists():
        download(source["url"], archive)
    out = repo_root / OUT_DIR
    if not force and (out / GRID_FILE).exists() and (out / ZONES_FILE).exists():
        print("  already present")
        return
    build(archive, out, release_tag(source))


# ------------------------------------------------------------------ transitions

def abbreviation(designation: str) -> str:
    """The tzdata designation unless it is only a numeric offset ("+0530", "-03")."""
    return "" if NUMERIC_DESIGNATION.match(designation) else designation


def tzif_transitions(data: bytes) -> tuple[list[tuple[int, int, str]], tuple[int, str]]:
    """Explicit transitions [(unix_s, utoff_s, designation)] of a TZif (v2+) file and the type before the first one."""
    if data[:4] != b"TZif":
        raise ValueError("not a TZif file")
    counts = struct.unpack(">6l", data[20:44])
    isutcnt, isstdcnt, leapcnt, timecnt, typecnt, charcnt = counts
    if data[4] >= ord("2"):  # skip the 32-bit block, read the 64-bit one
        v1 = timecnt * 5 + typecnt * 6 + charcnt + leapcnt * 8 + isstdcnt + isutcnt
        base = 44 + v1
        isutcnt, isstdcnt, leapcnt, timecnt, typecnt, charcnt = struct.unpack(">6l", data[base + 20:base + 44])
        pos, time_size = base + 44, 8
    else:
        pos, time_size = 44, 4
    times = struct.unpack(f">{timecnt}{'q' if time_size == 8 else 'l'}", data[pos:pos + timecnt * time_size])
    pos += timecnt * time_size
    indices = data[pos:pos + timecnt]
    pos += timecnt
    types = [struct.unpack(">lBB", data[pos + 6 * i:pos + 6 * i + 6]) for i in range(typecnt)]
    pos += typecnt * 6
    chars = data[pos:pos + charcnt]

    def designation(index: int) -> str:
        start = types[index][2]
        return chars[start:chars.index(b"\0", start)].decode("ascii")

    explicit = [(times[i], types[indices[i]][0], designation(indices[i])) for i in range(timecnt)]
    return explicit, (types[0][0], designation(0))


def _state(zone: ZoneInfo, unix_s: int) -> tuple[int, str]:
    local = dt.datetime.fromtimestamp(unix_s, zone)
    return int(local.utcoffset().total_seconds()), local.tzname()


def zone_transitions(zone_id: str, from_unix: int, to_unix: int) -> list[list]:
    """[[unix_s, offset_s, abbreviation]]: the state at from_unix, then every change of offset or abbreviation."""
    data = importlib.resources.files("tzdata.zoneinfo").joinpath(*zone_id.split("/")).read_bytes()
    explicit, initial = tzif_transitions(data)
    zone = ZoneInfo(zone_id)
    changes: list[tuple[int, int, str]] = []
    state = initial
    for t, offset, name in explicit:
        if t <= from_unix:
            state = (offset, name)
        elif t < to_unix:
            changes.append((t, offset, name))
    # past the explicit list: the footer rule through zoneinfo, daily samples + bisection
    t = max(from_unix, explicit[-1][0] if explicit else from_unix)
    current = _state(zone, t)
    while t < to_unix:
        nxt = min(t + DAY_S, to_unix)
        later = _state(zone, nxt)
        if later != current:
            lo, hi = t, nxt  # state(lo) == current, state(hi) != current
            while hi - lo > 1:
                mid = (lo + hi) // 2
                if _state(zone, mid) == current:
                    lo = mid
                else:
                    hi = mid
            changes.append((hi, *_state(zone, hi)))
            current = _state(zone, hi)
        t = nxt
    out = [[from_unix, state[0], abbreviation(state[1])]]
    for t, offset, name in changes:
        if [offset, abbreviation(name)] != out[-1][1:]:
            out.append([t, offset, abbreviation(name)])
    return out


# ------------------------------------------------------------------ grid

def grid_shape() -> tuple[int, int]:
    return round(180.0 / STEP_DEG), round(360.0 / STEP_DEG)


def nautical_zone(longitude_deg: float) -> str:
    """The nautical zone of a meridian (Etc/GMT+N is UTC-N), for cells no boundary polygon covers."""
    hours = int(np.floor((longitude_deg + 7.5) / 15.0))
    hours = max(-12, min(12, hours))
    return "Etc/GMT" if hours == 0 else f"Etc/GMT{'-' if hours > 0 else '+'}{abs(hours)}"


def encode_rows(grid: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Run-length encoding: (row_start u32[rows + 1], runs u16[n, 2] of (first_column, zone))."""
    row_start = [0]
    runs = []
    for row in grid:
        edges = np.flatnonzero(np.diff(row)) + 1
        starts = np.concatenate(([0], edges))
        runs.append(np.stack([starts, row[starts]], axis=1))
        row_start.append(row_start[-1] + len(starts))
    return np.array(row_start, dtype="<u4"), np.concatenate(runs).astype("<u2")


def decode_cell(row_start: np.ndarray, runs: np.ndarray, row: int, column: int) -> int:
    first, last = int(row_start[row]), int(row_start[row + 1])
    index = first + int(np.searchsorted(runs[first:last, 0], column, side="right")) - 1
    return int(runs[index, 1])


def write_grid(path: Path, grid: np.ndarray) -> None:
    row_start, runs = encode_rows(grid)
    rows, columns = grid.shape
    with path.open("wb") as f:
        f.write(GRID_MAGIC)
        f.write(struct.pack("<3I3d", GRID_VERSION, columns, rows, WEST_LONGITUDE_DEG, NORTH_LATITUDE_DEG, STEP_DEG))
        f.write(row_start.tobytes())
        f.write(runs.tobytes())


def read_grid(path: Path) -> tuple[dict, np.ndarray, np.ndarray]:
    data = path.read_bytes()
    if data[:4] != GRID_MAGIC:
        raise ValueError("not a zone grid")
    version, columns, rows, west, north, step = struct.unpack("<3I3d", data[4:40])
    row_start = np.frombuffer(data, dtype="<u4", count=rows + 1, offset=40)
    runs = np.frombuffer(data, dtype="<u2", offset=40 + 4 * (rows + 1)).reshape(-1, 2)
    header = {"version": version, "columns": columns, "rows": rows, "west": west, "north": north, "step": step}
    return header, row_start, runs


def cell_of(header: dict, latitude_deg: float, longitude_deg: float) -> tuple[int, int]:
    row = int(np.floor((header["north"] - latitude_deg) / header["step"]))
    column = int(np.floor((longitude_deg - header["west"]) / header["step"])) % header["columns"]
    return min(max(row, 0), header["rows"] - 1), column


def rasterize(features: list[dict], zone_index: dict[str, int]) -> np.ndarray:
    import rasterio.features  # lazy: only the build needs rasterio
    from rasterio.transform import from_origin

    rows, columns = grid_shape()
    grid = np.zeros((rows, columns), dtype=np.uint16)
    unset = np.uint16(0xFFFF)
    # oceans first, land on top (the release's land and ocean polygons meet at the coast)
    ordered = sorted(features, key=lambda f: (not f["properties"]["tzid"].startswith("Etc/"), f["properties"]["tzid"]))
    shapes = [(f["geometry"], zone_index[f["properties"]["tzid"]]) for f in ordered]
    for band in range(0, rows, BAND_ROWS):
        band_rows = min(BAND_ROWS, rows - band)
        transform = from_origin(WEST_LONGITUDE_DEG, NORTH_LATITUDE_DEG - band * STEP_DEG, STEP_DEG, STEP_DEG)
        grid[band:band + band_rows] = rasterio.features.rasterize(
            shapes, out_shape=(band_rows, columns), transform=transform, fill=unset, dtype="uint16",
            merge_alg=rasterio.enums.MergeAlg.replace)
        print(f"    rows {band}..{band + band_rows - 1}")
    holes = grid == unset
    if holes.any():  # slivers between polygons: the nautical zone of the cell's meridian
        longitudes = WEST_LONGITUDE_DEG + (np.arange(columns) + 0.5) * STEP_DEG
        column_zone = np.array([zone_index[nautical_zone(lon)] for lon in longitudes], dtype=np.uint16)
        r, c = np.nonzero(holes)
        grid[r, c] = column_zone[c]
        print(f"    {len(r)} uncovered cells filled with the nautical zone")
    return grid


def build(archive: Path, out: Path, release: str) -> None:
    import tzdata

    with zipfile.ZipFile(archive) as zf:
        name = next(n for n in zf.namelist() if n.endswith((".geojson", ".json")))
        features = json.loads(zf.read(name))["features"]
    ids = sorted({f["properties"]["tzid"] for f in features} | {nautical_zone(lon) for lon in range(-180, 181, 15)})
    zone_index = {zone_id: i for i, zone_id in enumerate(ids)}
    from_unix, to_unix = int(FROM_UTC.timestamp()), int(TO_UTC.timestamp())
    print(f"  {len(ids)} zones, transitions {FROM_UTC.year}-{TO_UTC.year} (tzdata {tzdata.IANA_VERSION})")
    zones = [{"id": zone_id, "transitions": zone_transitions(zone_id, from_unix, to_unix)} for zone_id in ids]
    print(f"  rasterising {grid_shape()[1]} x {grid_shape()[0]} cells at {STEP_DEG} deg")
    grid = rasterize(features, zone_index)
    out.mkdir(parents=True, exist_ok=True)
    write_grid(out / GRID_FILE, grid)
    table = {"tzdata": tzdata.IANA_VERSION, "boundaries": f"timezone-boundary-builder {release} (with oceans)",
             "grid_step_deg": STEP_DEG, "from_unix": from_unix, "to_unix": to_unix, "zones": zones}
    (out / ZONES_FILE).write_text(json.dumps(table, separators=(",", ":")), encoding="utf-8")
    for f in (GRID_FILE, ZONES_FILE):
        print(f"  -> {out / f} ({(out / f).stat().st_size / 1e6:.2f} MB)")


def main() -> int:
    import tomllib

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force", action="store_true", help="rebuild even if the outputs exist")
    args = parser.parse_args()
    with Path(__file__).with_name("sources.toml").open("rb") as f:
        source = next(s for s in tomllib.load(f)["source"] if s["id"] == "timezones")
    archive = REPO_ROOT / raw_zip(source)
    if not archive.exists():
        print(f"missing {archive}: run python tools/data_pipeline/fetch.py timezones", file=sys.stderr)
        return 1
    if args.force or not (REPO_ROOT / OUT_DIR / GRID_FILE).exists():
        build(archive, REPO_ROOT / OUT_DIR, release_tag(source))
    return 0


if __name__ == "__main__":
    sys.exit(main())
