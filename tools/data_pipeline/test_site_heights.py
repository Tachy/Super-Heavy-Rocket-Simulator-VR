"""Unit tests of the launch site height tool. No network, no data files.

Run: python -m pytest tools/data_pipeline/test_site_heights.py
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import site_heights  # noqa: E402

SITES_FILE = Path(__file__).resolve().parents[2] / "config" / "launch_sites.toml"


def synthetic_grid(dn: np.ndarray) -> site_heights.LocalGrid:
    header = {"north_latitude_deg": 26.01, "west_longitude_deg": -97.17, "step_latitude_deg": 1.0e-5,
              "step_longitude_deg": 1.0e-5, "dn_scale_m": 0.01}
    return site_heights.LocalGrid(header, dn)


def test_median_inside_radius():
    n = 801  # 1e-5 deg ~ 1.1 m: an ~890 m square
    dn = np.zeros((n, n), dtype=np.int16)  # 0 m everywhere
    centre = n // 2
    lat = 26.01 - centre * 1.0e-5
    lon = -97.17 + centre * 1.0e-5
    # A 10 m mound inside r = 100 m, plus a 50 m spike outside it that a square window would catch.
    yy, xx = np.mgrid[0:n, 0:n]
    dist_m = np.hypot((yy - centre) * 1.0e-5 * 111194.9, (xx - centre) * 1.0e-5 * 111194.9 * np.cos(np.radians(lat)))
    dn[dist_m <= 60.0] = 1000  # +10 m disc of 60 m radius: about 36 % of the 100 m disc
    dn[(dist_m > 100.0) & (dist_m < 110.0)] = 5000  # outside the radius: must not count
    grid = synthetic_grid(dn)
    median, spread = site_heights.median_and_spread(grid, lat, lon, 100.0)
    assert median == 0.0  # the 36 % mound is below the median of the disc
    assert abs(spread - 10.0) < 1e-9  # the +50 m ring is outside
    median_small, spread_small = site_heights.median_and_spread(grid, lat, lon, 50.0)
    assert abs(median_small - 10.0) < 1e-9  # inside the mound only
    assert spread_small == 0.0


def test_write_keeps_comments():
    text = SITES_FILE.read_text(encoding="utf-8")
    new = {"starbase_olp1": (-19.87, "3DEP", 150.0), "ksc_lc39a": (-14.1, "3DEP", 150.0),
           "ccsfs_slc40": (-24.2, "3DEP", 100.0), "ccsfs_lz40": (-25.0, "3DEP", 61.0)}
    out = site_heights.replace_site_heights(text, new, "2026-10-23")
    old_lines, new_lines = text.split("\n"), out.split("\n")
    assert len(old_lines) == len(new_lines)
    changed = [(a, b) for a, b in zip(old_lines, new_lines) if a != b]
    assert len(changed) == 4  # exactly the four site-level height lines; the tower height_m lines are untouched
    for a, b in changed:
        assert a.startswith("height_m = ")
        assert b.startswith("height_m = ") and "[D] median of 3DEP within" in b and "2026-10-23" in b
    assert "height_m = 146.0" in out  # tower
