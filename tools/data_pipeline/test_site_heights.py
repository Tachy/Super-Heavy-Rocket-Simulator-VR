"""Unit tests of the launch site height tool. No network, no data files.

Run: python -m pytest tools/data_pipeline/test_site_heights.py
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import site_heights  # noqa: E402

# The layout of config/launch_sites.toml: site-level height_m lines, a tower sub-table with its own height_m, comments.
SITES_TEXT = """# Launch and landing sites.

[[site]]
id = "starbase_olp1"
name = "Starbase OLP-1"
height_m = 10.0               # [E]
plateau_radius_m = 150.0      # [E]

[site.tower]                  # catch tower
height_m = 146.0              # [E]
mount_height_m = 20.0         # [E]

[site.sea]                    # coastline
direction_deg = 90.0          # [E]

[[site]]
id = "ksc_lc39a"
height_m = 15.0               # [E]

[site.tower]
height_m = 146.0              # [E]

[[site]]
id = "ccsfs_slc40"
height_m = -20.0              # [E]

[[site]]
id = "ccsfs_lz40"
kind = "landing_zone"
height_m = -20.0              # [E]
"""


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
    new = {"starbase_olp1": (-19.87, "3DEP", 150.0), "ksc_lc39a": (-14.1, "3DEP", 150.0),
           "ccsfs_slc40": (-24.2, "3DEP", 100.0), "ccsfs_lz40": (-25.0, "3DEP", 61.0)}
    out = site_heights.replace_site_heights(SITES_TEXT, new, "2026-10-23")
    old_lines, new_lines = SITES_TEXT.split("\n"), out.split("\n")
    assert len(old_lines) == len(new_lines)
    changed = [(a, b) for a, b in zip(old_lines, new_lines) if a != b]
    assert len(changed) == 4  # exactly the four site-level height lines; the tower height_m lines are untouched
    for a, b in changed:
        assert a.startswith("height_m = ")
        assert b.startswith("height_m = ") and "[D] median of 3DEP within" in b and "2026-10-23" in b
    assert out.count("height_m = 146.0") == 2  # towers
    assert "height_m = -19.87  # [D] median of 3DEP within 150 m, 2026-10-23" in out


def row(site_id: str, orthometric_m: float, spread_m: float) -> dict:
    return {"id": site_id, "orthometric_m": orthometric_m, "spread_m": spread_m}


def test_gate_passes_plausible_sites():
    failures, waived = site_heights.gate([row("a", 1.4, 3.1), row("b", -2.0, 5.0), row("c", 25.0, 0.0)], {})
    assert failures == [] and waived == []


def test_gate_refuses_outside_range_or_spread():
    failures, waived = site_heights.gate([row("low", -2.5, 1.0), row("high", 30.0, 1.0), row("rough", 5.0, 7.0)], {})
    assert len(failures) == 3 and waived == []
    assert failures[0].startswith("low: orthometric -2.50 m")
    assert failures[1].startswith("high: orthometric 30.00 m")
    assert failures[2].startswith("rough: spread 7.00 m")


def test_gate_waiver_covers_only_its_check():
    waivers = {"pad": {"spread": "trench in the mound"}}
    failures, waived = site_heights.gate([row("pad", 13.8, 12.9)], waivers)
    assert failures == [] and len(waived) == 1 and "trench in the mound" in waived[0]
    failures, waived = site_heights.gate([row("pad", 40.0, 12.9)], waivers)  # the range check is not waived
    assert len(failures) == 1 and failures[0].startswith("pad: orthometric") and len(waived) == 1


def test_lc39a_waiver_is_ruling_6():
    reason = site_heights.GATE_WAIVERS["ksc_lc39a"]["spread"]
    assert "Ruling 6" in reason
    failures, waived = site_heights.gate([row("ksc_lc39a", 13.84, 12.9), row("starbase_olp1", 1.41, 3.1)])
    assert failures == [] and len(waived) == 1
    failures, _ = site_heights.gate([row("starbase_olp1", 1.41, 12.9)])  # the waiver is per site
    assert len(failures) == 1
