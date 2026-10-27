"""ERA5 analog weather set: the typical cloud situation of a calendar month, offline (M8 weather and clouds).

For the reference year (2025) the 15th of each month at 00 / 06 / 12 / 18 UTC (48 hours) is downloaded from the
ARCO-ERA5 Zarr store (six timed variables + the static orography, ~150 MB raw per hour, blosc chunks), turned into a
downsampled weather cache file by build/core/weather_snapshot.exe (--downsample 4: 1 deg, ~8.6 MB) and the raw files are
deleted again (the static orography file is kept once).

URL and file-name rules mirror core/src/environment/weather_source.cpp (era5_requests / era5_file_name) exactly; the
pytest pins them. Output (git-ignored):
  data/weather/analog/era5_<hours since 1900>.wthr
  data/weather/analog/index.json   {"reference_year", "downsample", "entries": [{"month", "hour_utc", "key", "file"}]}
Raw downloads: data/raw/weather/. Usage: python tools/data_pipeline/fetch.py weather_analog
"""

from __future__ import annotations

import datetime as dt
import http.client
import json
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
STORE_URL = "https://storage.googleapis.com/gcp-public-data-arco-era5/ar/full_37-1h-0p25deg-chunk-1.zarr-v3/"
EPOCH_1900 = dt.datetime(1900, 1, 1, tzinfo=dt.timezone.utc)
REFERENCE_YEAR = 2025
REFERENCE_DAY = 15
SLOT_HOURS = (0, 6, 12, 18)
DOWNSAMPLE = 4
RAW_DIR = Path("data/raw/weather")
OUT_DIR = Path("data/weather/analog")
SNAPSHOT_EXE = Path("build/core/weather_snapshot.exe")
STATIC_OROGRAPHY_FILE = "era5_geopotential_at_surface.blosc"

# (variable, four-dimensional (pressure levels), time independent) - the order of the core's era5_variables
VARIABLES = (
    ("fraction_of_cloud_cover", True, False),
    ("specific_cloud_liquid_water_content", True, False),
    ("specific_cloud_ice_water_content", True, False),
    ("temperature", True, False),
    ("surface_pressure", False, False),
    ("total_column_water_vapour", False, False),
    ("geopotential_at_surface", False, True),
    ("2m_temperature", False, False),
    ("2m_dewpoint_temperature", False, False),
    ("total_cloud_cover", False, False),
)


def hours_since_1900(when: dt.datetime) -> int:
    """Whole hours since 1900-01-01 00:00 UTC: the ARCO-ERA5 `time` index (offset 0)."""
    return int((when - EPOCH_1900).total_seconds() // 3600)


def cache_key(hour: int) -> str:
    return f"era5_{hour}"


def era5_file_name(key: str, variable: str) -> str:
    return f"{key}_{variable}.blosc"


def era5_requests(hour: int) -> list[tuple[str, str]]:
    """(url, file name) per file of one ERA5 hour; the static orography has one file name for every hour."""
    key = cache_key(hour)
    out = []
    for name, four_d, static in VARIABLES:
        url = f"{STORE_URL}{name}/{hour}" + (".0.0.0" if four_d else ".0.0")
        out.append((url, STATIC_OROGRAPHY_FILE if static else era5_file_name(key, name)))
    return out


def analog_hours(year: int = REFERENCE_YEAR) -> list[tuple[int, int, int]]:
    """(month, slot hour UTC, hours since 1900) of the 48 reference hours."""
    out = []
    for month in range(1, 13):
        for slot in SLOT_HOURS:
            when = dt.datetime(year, month, REFERENCE_DAY, slot, tzinfo=dt.timezone.utc)
            out.append((month, slot, hours_since_1900(when)))
    return out


def analog_index(year: int = REFERENCE_YEAR) -> dict:
    entries = [{"month": m, "hour_utc": s, "key": cache_key(h), "file": cache_key(h) + ".wthr"}
               for m, s, h in analog_hours(year)]
    return {"reference_year": year, "downsample": DOWNSAMPLE, "entries": entries}


def download_resumable(url: str, target: Path, retries: int = 5) -> None:
    """Download to <target>.part (continued with a Range request after an interruption), then rename into place."""
    target.parent.mkdir(parents=True, exist_ok=True)
    partial = target.with_suffix(target.suffix + ".part")
    for attempt in range(retries):
        have = partial.stat().st_size if partial.exists() else 0
        headers = {"User-Agent": "starsim-data-pipeline"}
        if have:
            headers["Range"] = f"bytes={have}-"
        try:
            with urllib.request.urlopen(urllib.request.Request(url, headers=headers), timeout=60) as response:
                mode = "ab" if have and response.status == 206 else "wb"
                with partial.open(mode) as out:
                    while chunk := response.read(1 << 20):
                        out.write(chunk)
            partial.replace(target)
            return
        except urllib.error.HTTPError as error:
            if error.code == 416 and have:  # the part file is already complete
                partial.replace(target)
                return
            if error.code in (403, 404):
                raise
            print(f"  retry {attempt + 1}: {error}")
        except (urllib.error.URLError, TimeoutError, ConnectionError, http.client.IncompleteRead) as error:
            print(f"  retry {attempt + 1}: {error}")
            time.sleep(min(2 ** attempt, 30))  # short backoff; the .part file is continued by a Range request
    raise RuntimeError(f"download failed: {url}")


def fetch(source: dict, root: Path, force: bool = False) -> None:
    exe = root / SNAPSHOT_EXE
    if not exe.exists():
        raise SystemExit(f"{SNAPSHOT_EXE} is missing: build the core first (tools/build.ps1)")
    raw, out_dir = root / RAW_DIR, root / OUT_DIR
    out_dir.mkdir(parents=True, exist_ok=True)
    year = int(source.get("reference_year", REFERENCE_YEAR))
    for month, slot, hour in analog_hours(year):
        key = cache_key(hour)
        target = out_dir / (key + ".wthr")
        if target.exists() and not force:
            print(f"  {key}: present")
            continue
        print(f"  {key} ({year}-{month:02d}-{REFERENCE_DAY} {slot:02d} UTC)")
        requests = era5_requests(hour)
        for url, file_name in requests:
            if not (raw / file_name).exists():
                print(f"    downloading {file_name}")
                download_resumable(url, raw / file_name)
        partial = target.with_suffix(".wthr.part")
        subprocess.run([str(exe), "--era5", str(raw), "--id", key, "--config", str(root / "config"),
                        "--out", str(partial), "--downsample", str(DOWNSAMPLE)], check=True)
        partial.replace(target)
        for _, file_name in requests:  # raw files are deletable; the static orography stays (shared by every hour)
            if file_name != STATIC_OROGRAPHY_FILE:
                (raw / file_name).unlink(missing_ok=True)
    index = analog_index(year)
    (out_dir / "index.json").write_text(json.dumps(index, indent=1), encoding="utf-8")
    total = sum(p.stat().st_size for p in out_dir.glob("*.wthr"))
    print(f"  {len(index['entries'])} analog files, {total / 1e6:.0f} MB in {OUT_DIR}")


if __name__ == "__main__":
    sys.exit("run: python tools/data_pipeline/fetch.py weather_analog")
