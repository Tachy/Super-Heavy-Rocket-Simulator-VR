"""Convert downloaded raw data (data/raw) into runtime formats Godot can load directly.

Usage: python tools/data_pipeline/prepare.py

Outputs (git-ignored):
  data/textures/earth_day.jpg     equirectangular, lon -180..180, lat 90..-90
  data/textures/earth_night.jpg   Black Marble, resampled to 8192x4096
  data/textures/moon_color.png    LROC colour mosaic (equirectangular, lon -180..180)
  data/stars/stars.csv            stars to magnitude 7.5: ra_deg, dec_deg, mag, color_index (ICRS / J2000)
"""

from __future__ import annotations

import csv
import gzip
import shutil
import sys
from pathlib import Path

from PIL import Image

REPO_ROOT = Path(__file__).resolve().parents[2]
RAW = REPO_ROOT / "data" / "raw"
TEXTURES = REPO_ROOT / "data" / "textures"
STARS = REPO_ROOT / "data" / "stars"
STAR_MAGNITUDE_LIMIT = 7.5

Image.MAX_IMAGE_PIXELS = 200_000_000  # trusted NASA mosaics exceed Pillow's default guard


def prepare_textures() -> None:
    TEXTURES.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(RAW / "earth_day_5400.jpg", TEXTURES / "earth_day.jpg")
    with Image.open(RAW / "earth_night_3km.jpg") as night:
        night.convert("RGB").resize((8192, 4096), Image.Resampling.LANCZOS).save(
            TEXTURES / "earth_night.jpg", quality=92)
    with Image.open(RAW / "moon_color_4k.tif") as moon:
        moon.convert("RGB").save(TEXTURES / "moon_color.png", optimize=True)
    print(f"textures -> {TEXTURES}")


def prepare_stars() -> None:
    STARS.mkdir(parents=True, exist_ok=True)
    count = 0
    with gzip.open(RAW / "hyg_v44.csv.gz", "rt", encoding="utf-8", newline="") as src, \
            (STARS / "stars.csv").open("w", encoding="utf-8", newline="") as dst:
        writer = csv.writer(dst)
        writer.writerow(["ra_deg", "dec_deg", "mag", "color_index"])
        for row in csv.DictReader(src):
            if row["proper"] == "Sol" or not row["mag"]:
                continue
            mag = float(row["mag"])
            if mag > STAR_MAGNITUDE_LIMIT:
                continue
            ci = row["ci"] or "0.65"
            writer.writerow([f"{float(row['ra']) * 15.0:.6f}", f"{float(row['dec']):.6f}", f"{mag:.2f}", ci])
            count += 1
    print(f"{count} stars (mag <= {STAR_MAGNITUDE_LIMIT}) -> {STARS / 'stars.csv'}")


def main() -> int:
    prepare_textures()
    prepare_stars()
    return 0


if __name__ == "__main__":
    sys.exit(main())
