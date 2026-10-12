"""Fetch reference vectors from the JPL Horizons API and write test fixtures.

Usage: python tools/validation/fetch_horizons.py

Writes tests/validation/fixtures/horizons_*.csv (committed to git, so tests run offline):
  - geocentric Moon and Sun states (ICRF, TDB) at fixed epochs
  - topocentric Moon positions for launch sites -> validates Earth orientation end to end
    (site_gcrf = moon_geocentric - moon_topocentric)

Note: Horizons uses DE441 and its own EOP series; differences to DE440 / NAIF EOP kernels
are expected at the decimetre-to-metre level and are reflected in the test tolerances.
"""

from __future__ import annotations

import csv
import json
import sys
import urllib.parse
import urllib.request
from pathlib import Path

API_URL = "https://ssd.jpl.nasa.gov/api/horizons.api"
FIXTURE_DIR = Path(__file__).resolve().parents[2] / "tests" / "validation" / "fixtures"

# Julian dates (TDB) spread over 1990-2040, including the 2024-04-08 total solar eclipse.
EPOCHS_JD_TDB = [
    2447892.5, 2449718.5, 2451545.0, 2452275.75, 2453371.25, 2454466.5, 2455562.125, 2456658.5,
    2457754.875, 2458849.5, 2459580.5, 2460310.5, 2460409.25, 2460676.5, 2461041.5, 2461326.0,
    2461406.5, 2462137.5, 2463233.5, 2466154.5,
]
# Topocentric checks need EOP history: only past epochs (2000-2026).
TOPO_EPOCHS_JD_TDB = [2451545.0, 2455562.125, 2458849.5, 2460409.25, 2460676.5, 2461041.5]

SITES = {
    # name: (east longitude deg, geodetic latitude deg, height above WGS84 km)
    "starbase_olp1": (-97.15394, 25.99686, 0.010),
    "ksc_lc39a": (-80.60410, 28.60822, 0.015),
}


def horizons_vectors(command: str, center: str, epochs: list[float], extra: dict | None = None) -> list[list[str]]:
    params = {
        "format": "json",
        "COMMAND": f"'{command}'",
        "EPHEM_TYPE": "VECTORS",
        "CENTER": f"'{center}'",
        "REF_PLANE": "FRAME",
        "REF_SYSTEM": "ICRF",
        "VEC_TABLE": "2",
        "VEC_CORR": "NONE",
        "OUT_UNITS": "KM-S",
        "CSV_FORMAT": "YES",
        "TIME_TYPE": "TDB",
        "TLIST_TYPE": "JD",
        "TLIST": " ".join(f"'{jd:.6f}'" for jd in epochs),
        "OBJ_DATA": "NO",
    }
    params.update(extra or {})
    url = API_URL + "?" + urllib.parse.urlencode(params)
    with urllib.request.urlopen(url) as response:
        payload = json.load(response)
    text = payload["result"]
    if "$$SOE" not in text:
        raise RuntimeError(f"Horizons error:\n{text}")
    body = text.split("$$SOE")[1].split("$$EOE")[0]
    rows = []
    for line in body.strip().splitlines():
        fields = [f.strip() for f in line.split(",") if f.strip()]
        # JDTDB, calendar, X, Y, Z, VX, VY, VZ
        rows.append([fields[0]] + fields[2:8])
    return rows


def write_csv(name: str, header: list[str], rows: list[list[str]]) -> None:
    FIXTURE_DIR.mkdir(parents=True, exist_ok=True)
    path = FIXTURE_DIR / name
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)
    print(f"wrote {path} ({len(rows)} rows)")


def main() -> int:
    header = ["jd_tdb", "x_km", "y_km", "z_km", "vx_kmps", "vy_kmps", "vz_kmps"]
    write_csv("horizons_moon_geocentric.csv", header, horizons_vectors("301", "500@399", EPOCHS_JD_TDB))
    write_csv("horizons_sun_geocentric.csv", header, horizons_vectors("10", "500@399", EPOCHS_JD_TDB))
    for site, (lon, lat, h) in SITES.items():
        rows = horizons_vectors(
            "301", "coord@399", TOPO_EPOCHS_JD_TDB,
            {"COORD_TYPE": "GEODETIC", "SITE_COORD": f"'{lon},{lat},{h}'"},
        )
        write_csv(f"horizons_moon_topocentric_{site}.csv",
                  header + ["site_lon_deg", "site_lat_deg", "site_h_km"],
                  [r + [str(lon), str(lat), str(h)] for r in rows])
    return 0


if __name__ == "__main__":
    sys.exit(main())
