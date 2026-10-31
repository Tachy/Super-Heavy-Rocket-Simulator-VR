"""Download all external data sources listed in sources.toml.

Usage: python tools/data_pipeline/fetch.py [--force] [source_id ...]
Files that already exist are skipped unless --force is given.
"""

from __future__ import annotations

import argparse
import shutil
import sys
import tomllib
import urllib.request
import zipfile
from pathlib import Path

import iss_oem
import lunar_tile

REPO_ROOT = Path(__file__).resolve().parents[2]
SOURCES_FILE = Path(__file__).with_name("sources.toml")


def load_sources() -> list[dict]:
    with SOURCES_FILE.open("rb") as f:
        return tomllib.load(f)["source"]


def download(url: str, target: Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    partial = target.with_suffix(target.suffix + ".part")
    print(f"  downloading {url}")
    request = urllib.request.Request(url, headers={"User-Agent": "starsim-data-pipeline"})
    with urllib.request.urlopen(request) as response, partial.open("wb") as out:
        shutil.copyfileobj(response, out, length=1 << 20)
    partial.replace(target)


def extract(archive: Path, destination: Path) -> None:
    print(f"  extracting {archive.name} -> {destination}")
    with zipfile.ZipFile(archive) as zf:
        zf.extractall(destination)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ids", nargs="*", help="only fetch these source ids")
    parser.add_argument("--force", action="store_true", help="re-download existing files")
    parser.add_argument("--retile", action="store_true",
                        help="imagery: redo the colour fit and all tiles from the cached clips / composites (no network)")
    parser.add_argument("--workers", type=int, default=None,
                        help="imagery: worker processes (default: CPUs usable by the process, memory-capped)")
    args = parser.parse_args()

    sources = load_sources()
    if args.ids:
        unknown = set(args.ids) - {s["id"] for s in sources}
        if unknown:
            print(f"unknown source ids: {', '.join(sorted(unknown))}", file=sys.stderr)
            return 1
        sources = [s for s in sources if s["id"] in args.ids]

    for source in sources:
        target = REPO_ROOT / source["target"]
        print(f"[{source['id']}] {source['name']}")
        if source.get("kind") == "lunar_tiles":  # LOLA: range-cropped site tiles + the global grid (lunar_tile.py)
            lunar_tile.fetch_all(load_sources(), REPO_ROOT, args.force)
            continue
        if source.get("kind") == "iss_oem":  # ISS OEM: always fetched, every version archived (iss_oem.py)
            iss_oem.fetch(source, REPO_ROOT, download)
            continue
        if source.get("kind") == "geoid":  # EGM2008: PROJ CDN GeoTIFF -> int16 grid (geoid.py)
            import geoid  # lazy: needs numpy (and rasterio), the other sources do not

            geoid.fetch(source, REPO_ROOT, download, args.force)
            continue
        if source.get("kind") == "earth_terrain":  # ETOPO + GLO-30 + 3DEP height tiles (earth_terrain.py)
            import earth_terrain  # lazy: needs numpy, rasterio and pyproj

            earth_terrain.fetch(source, REPO_ROOT, download, args.force)
            continue
        if source.get("kind") == "imagery":  # NAIP + Sentinel-2 median composite -> EPSG:4326 tile pyramid (imagery.py)
            import imagery  # lazy: needs numpy, rasterio, pyproj, Pillow and pystac-client

            imagery.fetch(source, REPO_ROOT, download, args.force, args.retile, args.workers)
            continue
        if source.get("kind") == "timezones":  # zone boundaries -> zone grid + tzdata transitions (timezones.py)
            import timezones  # lazy: needs numpy, rasterio and tzdata

            timezones.fetch(source, REPO_ROOT, download, args.force)
            continue
        if source.get("kind") == "weather_analog":  # ERA5 reference-year hours -> downsampled .wthr files (weather.py)
            if "weather_analog" not in args.ids:  # opt-in: gigabytes of downloads and a built core are needed
                print("  skipped (opt-in): build the core, then run `python tools/data_pipeline/fetch.py weather_analog`")
                continue
            import weather  # needs build/core/weather_snapshot.exe

            weather.fetch(source, REPO_ROOT, args.force)
            continue
        if source.get("kind") in ("weather_licence", "licence_only"):  # licence-only entries (GFS, GRIB2, OpenCV): no download
            continue
        if source.get("kind") == "timezones_rules":  # read from the tzdata package by the "timezones" step
            continue
        if target.exists() and not args.force:
            print("  already present")
        else:
            download(source["url"], target)
            if source.get("extract"):
                extract(target, REPO_ROOT / source["extract_to"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
