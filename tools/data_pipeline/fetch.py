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
        if target.exists() and not args.force:
            print("  already present")
        else:
            download(source["url"], target)
            if source.get("extract"):
                extract(target, REPO_ROOT / source["extract_to"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
