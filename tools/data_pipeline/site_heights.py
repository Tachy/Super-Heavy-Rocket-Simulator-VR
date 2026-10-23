"""Launch site heights derived from the Earth terrain data (M8a Task 6).

Per `[[site]]` of config/launch_sites.toml: the median ellipsoidal height of the 1 m local grid inside the site's
`plateau_radius_m`, its spread (max - min inside the radius), the orthometric height (ellipsoidal - EGM2008
undulation) and the old config value. `--write` replaces only the site-level `height_m = ...` line of each block
(the tower's `height_m` is untouched), keeping every other line and comment.

Run: python tools/data_pipeline/site_heights.py [--write]
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import tomllib
from datetime import date
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import geoid  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
SITES_FILE = REPO_ROOT / "config" / "launch_sites.toml"
TERRAIN_DIR = REPO_ROOT / "data" / "terrain" / "earth"
GEOID_DIR = REPO_ROOT / "data" / "geoid"
LOCAL_GRIDS = ("starbase_local", "cape_local")
EARTH_RADIUS_M = 6378137.0  # sphere_radius_m of the grid headers: metres per radian of the disc test
ORTHOMETRIC_RANGE_M = (-2.0, 25.0)  # plausibility gate for coastal pads
MAX_SPREAD_M = 5.0


class LocalGrid:
    """A node-registered geographic i16 grid (header of grid_file.py)."""

    def __init__(self, header: dict, dn: np.ndarray):
        self.north = header["north_latitude_deg"]
        self.west = header["west_longitude_deg"]
        self.step_lat = header["step_latitude_deg"]
        self.step_lon = header["step_longitude_deg"]
        self.scale = header["dn_scale_m"]
        self.dn = dn
        self.lines, self.samples = dn.shape

    def contains(self, lat_deg: float, lon_deg: float, margin_m: float = 0.0) -> bool:
        m_deg = math.degrees(margin_m / EARTH_RADIUS_M)
        return (self.north - (self.lines - 1) * self.step_lat + m_deg <= lat_deg <= self.north - m_deg
                and self.west + m_deg <= lon_deg <= self.west + (self.samples - 1) * self.step_lon - m_deg)


def load_grid(toml_path: Path) -> LocalGrid:
    header = tomllib.loads(toml_path.read_text(encoding="utf-8"))["grid"]
    dn = np.fromfile(toml_path.parent / header["heights"], dtype="<i2").reshape(header["lines"], header["samples"])
    return LocalGrid(header, dn)


def disc_heights(grid: LocalGrid, lat_deg: float, lon_deg: float, radius_m: float) -> np.ndarray:
    """Heights (m) of all nodes within radius_m of the position (equirectangular distance on the grid sphere)."""
    cos_lat = math.cos(math.radians(lat_deg))
    r_lat_deg = math.degrees(radius_m / EARTH_RADIUS_M)
    r_lon_deg = r_lat_deg / cos_lat
    i0 = max(int(math.floor((grid.north - (lat_deg + r_lat_deg)) / grid.step_lat)), 0)
    i1 = min(int(math.ceil((grid.north - (lat_deg - r_lat_deg)) / grid.step_lat)), grid.lines - 1)
    j0 = max(int(math.floor((lon_deg - r_lon_deg - grid.west) / grid.step_lon)), 0)
    j1 = min(int(math.ceil((lon_deg + r_lon_deg - grid.west) / grid.step_lon)), grid.samples - 1)
    lats = grid.north - np.arange(i0, i1 + 1) * grid.step_lat
    lons = grid.west + np.arange(j0, j1 + 1) * grid.step_lon
    dy = np.radians(lats - lat_deg)[:, None] * EARTH_RADIUS_M
    dx = np.radians(lons - lon_deg)[None, :] * EARTH_RADIUS_M * cos_lat
    inside = dx * dx + dy * dy <= radius_m * radius_m
    return grid.dn[i0:i1 + 1, j0:j1 + 1][inside].astype(np.float64) * grid.scale


def median_and_spread(grid: LocalGrid, lat_deg: float, lon_deg: float, radius_m: float) -> tuple[float, float]:
    h = disc_heights(grid, lat_deg, lon_deg, radius_m)
    if h.size == 0:
        raise ValueError("no grid node inside the radius")
    return float(np.median(h)), float(h.max() - h.min())


def height_line(height_m: float, source: str, radius_m: float, day: str) -> str:
    return f"height_m = {height_m:.2f}  # [D] median of {source} within {radius_m:g} m, {day}"


def replace_site_heights(text: str, heights_m: dict[str, tuple[float, str, float]], day: str) -> str:
    """Replace the site-level `height_m = ...` line of each `[[site]]` block listed in heights_m
    (id -> (height, source, radius)). Sub-table lines (`[site.tower]` ...) are never touched."""
    out: list[str] = []
    site_id: str | None = None
    in_site_level = False
    for line in text.split("\n"):
        stripped = line.strip()
        if stripped.startswith("["):
            in_site_level = stripped == "[[site]]"
            if in_site_level:
                site_id = None
        elif in_site_level:
            m = re.match(r'id\s*=\s*"([^"]+)"', stripped)
            if m:
                site_id = m.group(1)
            elif re.match(r"height_m\s*=", stripped) and site_id in heights_m:
                h, source, radius = heights_m[site_id]
                line = height_line(h, source, radius, day)
        out.append(line)
    return "\n".join(out)


def evaluate(sites_file: Path, terrain_dir: Path, geoid_dir: Path) -> list[dict]:
    sites = tomllib.loads(sites_file.read_text(encoding="utf-8"))["site"]
    grids = {name: load_grid(terrain_dir / f"{name}.toml") for name in LOCAL_GRIDS}
    undulation_cm = geoid.load_fine_cm(geoid_dir)
    rows = []
    for site in sites:
        lat, lon, radius = site["latitude_deg"], site["longitude_deg"], site["plateau_radius_m"]
        name, grid = next(((n, g) for n, g in grids.items() if g.contains(lat, lon, radius)), (None, None))
        if grid is None:
            raise ValueError(f"{site['id']}: no local grid covers the plateau")
        median, spread = median_and_spread(grid, lat, lon, radius)
        n = geoid.bilinear_m(undulation_cm, lat, lon)
        rows.append({"id": site["id"], "grid": name, "radius_m": radius, "old_m": site["height_m"],
                     "ellipsoidal_m": median, "spread_m": spread, "undulation_m": n, "orthometric_m": median - n})
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--write", action="store_true", help="write the heights into config/launch_sites.toml")
    args = parser.parse_args()
    rows = evaluate(SITES_FILE, TERRAIN_DIR, GEOID_DIR)
    print(f"{'site':14} {'grid':14} {'r [m]':>6} {'old':>8} {'ellips.':>8} {'spread':>7} {'N':>7} {'ortho.':>7}")
    for r in rows:
        print(f"{r['id']:14} {r['grid']:14} {r['radius_m']:6.0f} {r['old_m']:8.2f} {r['ellipsoidal_m']:8.2f} "
              f"{r['spread_m']:7.2f} {r['undulation_m']:7.2f} {r['orthometric_m']:7.2f}")
    if args.write:
        day = date.today().isoformat()
        values = {r["id"]: (r["ellipsoidal_m"], "3DEP 1 m + GLO-30", r["radius_m"]) for r in rows}
        text = SITES_FILE.read_text(encoding="utf-8")
        SITES_FILE.write_text(replace_site_heights(text, values, day), encoding="utf-8", newline="\n")
        print(f"written: {SITES_FILE}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
