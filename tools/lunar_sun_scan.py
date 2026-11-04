"""Sun elevation at a lunar landing site against the terrain horizon, and the mission epochs that land in daylight.

Runs game/tests/lunar_sun_scan.gd headless (core ephemeris DE440, Moon mean-Earth orientation, LOLA terrain horizon in
the Sun's azimuth, the same MoonSunHorizon data the game renders with) and evaluates the margin
    margin = Sun elevation (disc centre) - terrain horizon elevation in the Sun's azimuth.
For every candidate mission epoch (the mission's [orbit] epoch_utc) the surface stay is [epoch + touchdown_offset,
epoch + touchdown_offset + stay]; the table lists the minimum margin over that stay. The touchdown offset is not
computed here: take it from the flown mission (STARSIM_DEBUG_LUNAR_SESSION=1 on the "Lunar mission epoch probe" case,
status line "Landed", days after the epoch) and pass it with --landing-offset-d.

Usage (repo root):
  python tools/lunar_sun_scan.py --start 2026-11-02T00:00:00 --days 60 --landing-offset-d 9.7 --stay-h 24 \
      --epochs 2026-11-06,2026-11-07,...  [--site mons_mouton_im2] [--min-margin 5]
Without --epochs the table covers every full day of the scan as epoch candidates.
"""
import argparse
import datetime as dt
import subprocess
import sys
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
GODOT = ROOT / "external/godot/bin/godot.windows.editor.double.x86_64.console.exe"


def site_position(site_id: str) -> tuple[float, float]:
    with open(ROOT / "config/lunar_sites.toml", "rb") as f:
        for s in tomllib.load(f)["site"]:
            if s["id"] == site_id:
                return s["latitude_deg"], s["longitude_deg"]
    raise SystemExit(f"unknown lunar site {site_id}")


def run_scan(lat: float, lon: float, start: str, days: float, step_min: float, godot: Path) -> list[tuple[float, float, float, float]]:
    cmd = [str(godot), "--headless", "--path", str(ROOT / "game"), "--script", "res://tests/lunar_sun_scan.gd", "--",
           f"--lat={lat}", f"--lon={lon}", f"--start={start}", f"--days={days}", f"--step_min={step_min}"]
    out = subprocess.run(cmd, capture_output=True, text=True, check=True).stdout
    rows = [tuple(float(x) for x in line.split()[1:5]) for line in out.splitlines() if line.startswith("scan: ")]
    if not rows:
        raise SystemExit("the scan printed no rows:\n" + out[-2000:])
    return rows  # (hours since start, sun elevation, sun azimuth, horizon) in deg


def min_margin(rows, t0_h: float, t1_h: float) -> tuple[float, float, float, float]:
    """(minimum margin, margin at t0, margin at t1, minimum Sun elevation) over the samples in [t0, t1], deg."""
    inside = [(r[0], r[1] - r[3], r[1]) for r in rows if t0_h - 1e-9 <= r[0] <= t1_h + 1e-9]
    if not inside:
        raise SystemExit("the stay lies outside the scanned span")
    return min(m for _, m, _ in inside), inside[0][1], inside[-1][1], min(e for _, _, e in inside)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--site", default="mons_mouton_im2")
    p.add_argument("--start", default="2026-11-02T00:00:00", help="UTC start of the scan (ISO, no zone)")
    p.add_argument("--days", type=float, default=60.0)
    p.add_argument("--step-min", type=float, default=60.0)
    p.add_argument("--landing-offset-d", type=float, default=9.7, help="touchdown, days after the mission epoch")
    p.add_argument("--stay-h", type=float, default=24.0, help="surface stay (the mission's [surface] stay_hours)")
    p.add_argument("--epochs", default="", help="comma-separated mission epochs (ISO UTC), each optionally ISO@touchdown_offset_d "
                   "(the flown touchdown differs per epoch); default: every day of the scan")
    p.add_argument("--min-margin", type=float, default=5.0, help="requirement in deg (marks the OK column)")
    p.add_argument("--godot", default=str(GODOT))
    a = p.parse_args()

    lat, lon = site_position(a.site)
    rows = run_scan(lat, lon, a.start, a.days, a.step_min, Path(a.godot))
    start = dt.datetime.fromisoformat(a.start)
    epochs = []
    for item in (a.epochs.split(",") if a.epochs else []):
        iso, _, off = item.partition("@")
        epochs.append((dt.datetime.fromisoformat(iso), float(off) if off else a.landing_offset_d))
    if not epochs:
        epochs = [(start + dt.timedelta(days=k), a.landing_offset_d)
                  for k in range(int(a.days - a.landing_offset_d - a.stay_h / 24.0))]
    print(f"site {a.site} lat {lat} lon {lon}; touchdown = epoch + {a.landing_offset_d} d, stay {a.stay_h} h, "
          f"requirement margin >= {a.min_margin} deg (margin = Sun elevation - terrain horizon in the Sun's azimuth)")
    print(f"{'epoch (UTC)':<20} {'touchdown (UTC)':<17} {'margin touchdown':>16} {'margin liftoff':>14} {'min over stay':>13} {'min Sun el':>10}  OK")
    for e, offset_d in epochs:
        t0 = (e - start).total_seconds() / 3600.0 + offset_d * 24.0
        t1 = t0 + a.stay_h
        lo, m0, m1, el = min_margin(rows, t0, t1)
        td = start + dt.timedelta(hours=t0)
        print(f"{e.isoformat():<20} {td.strftime('%Y-%m-%dT%H:%M'):<17} {m0:16.2f} {m1:14.2f} {lo:13.2f} {el:10.2f}  {'yes' if lo >= a.min_margin else '-'}")


if __name__ == "__main__":
    sys.exit(main())
