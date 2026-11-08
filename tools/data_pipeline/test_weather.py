"""Tests of the ERA5 analog weather pipeline: URL / file-name rules (pinned against the core's rule), the analog index.
No network.

Run: python -m pytest tools/data_pipeline/test_weather.py
"""

import datetime as dt
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import weather  # noqa: E402

CORE_SOURCE = weather.REPO_ROOT / "core/src/environment/weather_source.cpp"


def test_temperature_url_for_2025_04_15_12_utc():
    when = dt.datetime(2025, 4, 15, 12, tzinfo=dt.timezone.utc)
    t = weather.hours_since_1900(when)
    # independent day count: 1900-01-01 .. 2025-01-01 = 125 years, 31 leap days (1904..2024; 1900 is not a leap year)
    days = 125 * 365 + 31 + (31 + 28 + 31 + 14)
    assert t == days * 24 + 12
    urls = dict((name, url) for url, name in weather.era5_requests(t))
    assert urls[f"era5_{t}_temperature.blosc"] == f"{weather.STORE_URL}temperature/{t}.0.0.0"
    assert urls[f"era5_{t}_surface_pressure.blosc"] == f"{weather.STORE_URL}surface_pressure/{t}.0.0"
    assert urls[weather.STATIC_OROGRAPHY_FILE].endswith(f"geopotential_at_surface/{t}.0.0")
    assert urls[f"era5_{t}_2m_dewpoint_temperature.blosc"] == f"{weather.STORE_URL}2m_dewpoint_temperature/{t}.0.0"
    assert urls[f"era5_{t}_2m_temperature.blosc"] == f"{weather.STORE_URL}2m_temperature/{t}.0.0"
    assert urls[f"era5_{t}_total_cloud_cover.blosc"] == f"{weather.STORE_URL}total_cloud_cover/{t}.0.0"
    assert len(urls) == 10


def test_rules_match_the_core_source():
    text = CORE_SOURCE.read_text(encoding="utf-8")
    assert weather.STORE_URL in text
    for name, four_d, static in weather.VARIABLES:
        assert re.search(r'\{"%s", %s, %s\}' % (name, str(four_d).lower(), str(static).lower()), text), name
    assert weather.STATIC_OROGRAPHY_FILE in (weather.REPO_ROOT / "core/include/starsim/environment/weather_source.hpp"
                                             ).read_text(encoding="utf-8")


def test_analog_index_lists_48_unique_files():
    index = weather.analog_index()
    entries = index["entries"]
    assert len(entries) == 48
    assert len({e["file"] for e in entries}) == 48
    assert {e["month"] for e in entries} == set(range(1, 13))
    assert {e["hour_utc"] for e in entries} == {0, 6, 12, 18}
    first = entries[0]
    assert first["key"] == "era5_" + str(weather.hours_since_1900(dt.datetime(2025, 1, 15, tzinfo=dt.timezone.utc)))


def test_bare_fetch_skips_weather_analog_without_network(monkeypatch, capsys):
    import fetch

    source = {"id": "weather_analog", "name": "x", "kind": "weather_analog", "target": "data/weather/analog/index.json"}
    monkeypatch.setattr(fetch, "load_sources", lambda: [source])
    calls = []
    monkeypatch.setattr(weather, "fetch", lambda *args, **kwargs: calls.append(args))
    monkeypatch.setattr(fetch, "download", lambda *args, **kwargs: calls.append("network"))

    monkeypatch.setattr(sys, "argv", ["fetch.py"])
    assert fetch.main() == 0
    assert calls == [] and "opt-in" in capsys.readouterr().out

    monkeypatch.setattr(sys, "argv", ["fetch.py", "weather_analog"])
    assert fetch.main() == 0
    assert len(calls) == 1


def test_format_version_matches_the_core_header():
    # T7b (review M5): the version is read from the core's header, files of another version are rebuilt
    text = (weather.REPO_ROOT / weather.FORMAT_HEADER).read_text(encoding="utf-8")
    expected = int(re.search(r"weather_snapshot_file_version = (\d+);", text).group(1))
    assert weather.current_format_version(weather.REPO_ROOT) == expected


def test_file_format_version_reads_the_header(tmp_path):
    good = tmp_path / "a.wthr"
    good.write_bytes(weather.FILE_MAGIC + (8).to_bytes(4, "little") + bytes(8))
    assert weather.file_format_version(good) == 8
    bad = tmp_path / "b.wthr"
    bad.write_bytes(b"NOTWTHR!" + bytes(8))
    assert weather.file_format_version(bad) is None
    short = tmp_path / "c.wthr"
    short.write_bytes(b"STAR")
    assert weather.file_format_version(short) is None


def test_fetch_rebuilds_a_stale_analog_file(tmp_path, monkeypatch):
    # a present file of the current version is kept, one of an older version is rebuilt (no network: raw files present)
    (tmp_path / weather.SNAPSHOT_EXE).parent.mkdir(parents=True)
    (tmp_path / weather.SNAPSHOT_EXE).write_bytes(b"")
    (tmp_path / weather.FORMAT_HEADER).parent.mkdir(parents=True)
    (tmp_path / weather.FORMAT_HEADER).write_text("inline constexpr std::uint32_t weather_snapshot_file_version = 9;",
                                                  encoding="utf-8")
    out = tmp_path / weather.OUT_DIR
    out.mkdir(parents=True)
    hours = list(weather.analog_hours(weather.REFERENCE_YEAR))
    for i, (_, _, hour) in enumerate(hours):
        version = 8 if i == 0 else 9
        (out / (weather.cache_key(hour) + ".wthr")).write_bytes(weather.FILE_MAGIC + version.to_bytes(4, "little"))
    raw = tmp_path / weather.RAW_DIR
    raw.mkdir(parents=True)
    for _, file_name in weather.era5_requests(hours[0][2]):
        (raw / file_name).write_bytes(b"")
    built = []

    def fake_run(cmd, check):
        built.append(cmd[cmd.index("--id") + 1])
        Path(cmd[cmd.index("--out") + 1]).write_bytes(weather.FILE_MAGIC + (9).to_bytes(4, "little"))

    monkeypatch.setattr(weather.subprocess, "run", fake_run)
    weather.fetch({}, tmp_path)
    assert built == [weather.cache_key(hours[0][2])]
    assert weather.file_format_version(out / (weather.cache_key(hours[0][2]) + ".wthr")) == 9
