"""Convert downloaded raw data (data/raw) into runtime formats Godot can load directly.

Usage: python tools/data_pipeline/prepare.py

Outputs (git-ignored):
  data/textures/earth_day.jpg     surface reflectance, sRGB-encoded (earth_albedo.py); equirectangular, lon -180..180,
                                  lat 90..-90
  data/textures/earth_water.png   water fraction (255 = sea) on the same grid (earth_albedo.write_water_texture)
  data/textures/moon_color.png    LROC colour mosaic, large-scale albedo from the WAC Hapke w map (moon_albedo.py;
                                  equirectangular, lon -180..180)
  data/stars/stars.csv            stars to magnitude 7.5: ra_deg, dec_deg, mag, color_index (ICRS / J2000)
"""

from __future__ import annotations

import csv
import gzip
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
    if (RAW / "earth_day_5400.jpg").exists():
        # surface reflectance, sRGB-encoded: MODIS NBAR climatology on land, the Blue Marble through the fitted tables
        # elsewhere (earth_albedo.py)
        import earth_albedo

        earth_albedo.write_day_texture(REPO_ROOT, TEXTURES / "earth_day.jpg")
        if not earth_albedo.write_water_texture(REPO_ROOT, TEXTURES / "earth_water.png"):
            # the scaled-space sea needs it (world.gd draws the sea as land without it, tools/package.ps1 refuses to package)
            raise SystemExit("earth_water: no ETOPO relief / geoid (python tools/data_pipeline/fetch.py earth_terrain "
                             "geoid_egm2008), data/textures/earth_water.png cannot be written")
    else:
        print("earth_day: raw file missing (fetch.py earth_day), skipped")
    if (RAW / "moon_color_4k.tif").exists():
        # The large-scale albedo from the WAC Hapke w map when present (moon_albedo.py; Task 11 fix C).
        import moon_albedo

        stats = moon_albedo.prepare(RAW / "moon_color_4k.tif", RAW / "WAC_HAPKEPARAMMAP_643NM.IMG",
                                    REPO_ROOT / "config" / "moon_surface.toml", TEXTURES / "moon_color.png")
        if stats:
            print(f"moon_color: tied to the WAC w map: mare / highland {stats['mosaic_ratio']:.3f} -> "
                  f"{stats['out_ratio']:.3f} (Hapke I/F ratio of w {stats['w_mare']:.3f} / {stats['w_highland']:.3f}: "
                  f"{stats['reflectance_ratio']:.3f}), fine-detail exponent {stats['gamma']:.3f}")
        else:
            print("moon_color: WAC w map missing (fetch.py lroc_wac_hapke_parameter_map): the mosaic as it is")
    else:
        print("moon_color: raw file missing (fetch.py moon_color), skipped")
    print(f"textures -> {TEXTURES}")


def prepare_stars() -> None:
    if not (RAW / "hyg_v44.csv.gz").exists():
        print("stars: raw file missing (fetch.py hyg_stars), skipped")
        return
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
