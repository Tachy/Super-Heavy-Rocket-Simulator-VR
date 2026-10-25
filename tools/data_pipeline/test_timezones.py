"""Tests of the time zone pipeline: run-length grid round trip, transitions against zoneinfo, and (with data/timezones
built) the zones of known places. No network.

Run: python -m pytest tools/data_pipeline/test_timezones.py
"""

import bisect
import datetime as dt
import json
import random
import sys
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent))
import timezones  # noqa: E402

OUT = timezones.REPO_ROOT / timezones.OUT_DIR
FROM_UNIX = int(timezones.FROM_UTC.timestamp())
TO_UNIX = int(timezones.TO_UTC.timestamp())


def test_grid_round_trip(tmp_path):
    rng = np.random.default_rng(7)
    grid = np.repeat(rng.integers(0, 500, size=(40, 30)), 9, axis=1).astype(np.uint16)  # runs of 9 cells
    grid[3, :] = 17  # a single-run row
    path = tmp_path / "grid.bin"
    timezones.write_grid(path, grid)
    header, row_start, runs = timezones.read_grid(path)
    assert (header["rows"], header["columns"]) == grid.shape
    for r in range(grid.shape[0]):
        for c in range(grid.shape[1]):
            assert timezones.decode_cell(row_start, runs, r, c) == grid[r, c]
    assert row_start[4] - row_start[3] == 1


def test_abbreviation_drops_numeric_designations():
    assert timezones.abbreviation("CDT") == "CDT"
    assert timezones.abbreviation("+0530") == ""
    assert timezones.abbreviation("-03") == ""
    assert timezones.abbreviation("GMT") == "GMT"


def test_nautical_zone():
    assert timezones.nautical_zone(0.0) == "Etc/GMT"
    assert timezones.nautical_zone(-150.6) == "Etc/GMT+10"
    assert timezones.nautical_zone(80.6) == "Etc/GMT-5"
    assert timezones.nautical_zone(179.9) == "Etc/GMT-12"
    assert timezones.nautical_zone(-179.9) == "Etc/GMT+12"


def state_at(transitions, unix_s):
    index = bisect.bisect_right([t[0] for t in transitions], unix_s) - 1
    return transitions[index][1], transitions[index][2]


@pytest.mark.parametrize("zone_id", ["America/Chicago", "America/New_York", "Asia/Kolkata", "Africa/Casablanca",
                                     "Australia/Lord_Howe", "Europe/Berlin", "America/Sao_Paulo", "Pacific/Apia",
                                     "Etc/GMT+10"])
def test_transitions_match_zoneinfo(zone_id):
    transitions = timezones.zone_transitions(zone_id, FROM_UNIX, TO_UNIX)
    zone = ZoneInfo(zone_id)
    assert transitions[0][0] == FROM_UNIX
    rng = random.Random(zone_id)
    samples = [rng.randrange(FROM_UNIX, TO_UNIX) for _ in range(3000)]
    for t, _, _ in transitions[1:]:  # exactly at, and one second before, every transition
        samples += [t, t - 1]
    for t in samples:
        local = dt.datetime.fromtimestamp(t, zone)
        expected = (int(local.utcoffset().total_seconds()), timezones.abbreviation(local.tzname()))
        assert state_at(transitions, t) == expected, (zone_id, t)


def test_chicago_dst_start_2024():
    transitions = timezones.zone_transitions("America/Chicago", FROM_UNIX, TO_UNIX)
    t = int(dt.datetime(2024, 3, 10, 8, 0, tzinfo=dt.timezone.utc).timestamp())
    assert state_at(transitions, t - 1) == (-6 * 3600, "CST")
    assert state_at(transitions, t) == (-5 * 3600, "CDT")


needs_data = pytest.mark.skipif(not (OUT / timezones.GRID_FILE).exists(),
                                reason="data/timezones not built (python tools/data_pipeline/fetch.py timezones)")


@needs_data
@pytest.mark.parametrize("name, latitude_deg, longitude_deg, zone_id", [
    ("starbase", 25.9972, -97.1561, "America/Chicago"),
    ("lc39a", 28.6082, -80.6041, "America/New_York"),
    ("matamoros", 25.87, -97.50, "America/Matamoros"),
    ("new_delhi", 28.61, 77.21, "Asia/Kolkata"),
    ("pacific_open", 10.4, -150.6, "Etc/GMT+10"),
    ("indian_ocean", -30.3, 80.6, "Etc/GMT-5"),
    ("berlin", 52.52, 13.40, "Europe/Berlin"),
    ("vandenberg", 34.63, -120.61, "America/Los_Angeles"),
])
def test_grid_zones_of_known_places(name, latitude_deg, longitude_deg, zone_id):
    header, row_start, runs = timezones.read_grid(OUT / timezones.GRID_FILE)
    table = json.loads((OUT / timezones.ZONES_FILE).read_text(encoding="utf-8"))
    row, column = timezones.cell_of(header, latitude_deg, longitude_deg)
    assert table["zones"][timezones.decode_cell(row_start, runs, row, column)]["id"] == zone_id, name


@needs_data
def test_table_covers_every_grid_index():
    header, row_start, runs = timezones.read_grid(OUT / timezones.GRID_FILE)
    table = json.loads((OUT / timezones.ZONES_FILE).read_text(encoding="utf-8"))
    assert header["step"] == pytest.approx(timezones.STEP_DEG)
    assert int(runs[:, 1].max()) < len(table["zones"])
    assert table["from_unix"] == FROM_UNIX and table["to_unix"] == TO_UNIX


@needs_data
def test_table_matches_zoneinfo_for_every_zone():
    table = json.loads((OUT / timezones.ZONES_FILE).read_text(encoding="utf-8"))
    rng = random.Random(11)
    for zone in table["zones"]:
        tz = ZoneInfo(zone["id"])
        samples = [rng.randrange(FROM_UNIX, TO_UNIX) for _ in range(200)]
        samples += [t for t, _, _ in zone["transitions"][1:]] + [t - 1 for t, _, _ in zone["transitions"][1:]]
        for t in samples:
            local = dt.datetime.fromtimestamp(t, tz)
            expected = (int(local.utcoffset().total_seconds()), timezones.abbreviation(local.tzname()))
            assert state_at(zone["transitions"], t) == expected, (zone["id"], t)
