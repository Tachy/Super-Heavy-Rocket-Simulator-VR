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
    assert len(urls) == 7


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
