"""Time base of a webcast telemetry table: when the displayed state leaves the pad.

The tables' t_s is the webcast clock (T+0 = the announced liftoff). A rocket released at full thrust accelerates at
once (T/W - 1) g, but the displayed speed of every Block 3 / Falcon 9 table stays at 0 until T+2-3 s and its early
rise extrapolates to zero only at T+3.24-3.50 s (measured: flight 13 3.24, Starlink 15-25 3.26, CRS-35 3.42, flight 14
3.50 s): the telemetry graphics show the state ~3.4 s late (or the clock leads the release). A simulation released at its T+0 is compared at t_s - delta.

  python tools/telemetry/liftoff_time.py <table.csv> [--simulated sim.csv] [--from-mps 8] [--to-mps 40]

Estimates of delta from the readings between --from-mps and --to-mps (drag-free, near-vertical: the acceleration is
thrust / mass - g and grows slowly as the propellant burns):
  quadratic  the root of a least-squares parabola v(t) through the readings (no model);
  model      with --simulated (t_s,speed_kmh columns of a simulated ascent, T+0 = release; STARSIM_ASCENT_DUMP of the
             ascent validation tests): the median of t_table(v) - t_sim(v) over the readings - the model's own curve
             shape. A thrust-to-mass error e shifts it by about e * t_sim(v) (~0.3 s for 3 % at 40 m/s).
The acceleration of both (slope at the window's centre) is printed: a thrust-to-mass difference shows there, a time
base difference does not.

The altitude dial runs on its own, later time base: the displayed speed integrated through the near-vertical rise (from
the pad's displayed altitude) is matched to the altitude readings (shown to 0.1 km) between --alt-from and --alt-to s;
the shift with the least mean error is the altitude dial's lag behind the speed dial.
"""
import argparse
import csv

import numpy as np


def readings(path, column="booster_speed_kmh"):
    rows = list(csv.DictReader(line for line in open(path) if not line.startswith("#")))
    return np.array([(float(r["t_s"]), float(r[column]) / 3.6) for r in rows if r.get(column)])


def window(data, lo, hi):
    t, v = data[:, 0], data[:, 1]
    m = (t > 0.0) & (t < 60.0) & (v >= lo) & (v <= hi)
    return t[m], v[m]


def quadratic_root(t, v):
    p2, p1, p0 = np.polyfit(t, v, 2)
    roots = [r.real for r in np.roots([p2, p1, p0]) if abs(r.imag) < 1e-9 and r.real < t.min()]
    root = max(roots) if roots else float("nan")
    centre = 0.5 * (t.min() + t.max())
    return root, 2.0 * p2 * centre + p1


def altitude_lag(path, t_from, t_to):
    rows = list(csv.DictReader(line for line in open(path) if not line.startswith("#")))
    d = np.array([(float(r["t_s"]), float(r["booster_speed_kmh"]) / 3.6, float(r["booster_altitude_km"]) * 1e3)
                  for r in rows if r["booster_speed_kmh"] and r["booster_altitude_km"] and -5.0 < float(r["t_s"]) < t_to + 10])
    t, v, h = d.T
    pad = h[t < 0.0].mean()
    climbed = pad + np.concatenate([[0.0], np.cumsum(0.5 * (v[1:] + v[:-1]) * np.diff(t))])
    window = (t >= t_from) & (t <= t_to)
    best = None
    for shift in np.arange(0.0, 5.0001, 0.05):
        shown = np.round(np.interp(t[window] - shift, t, climbed) / 100.0) * 100.0
        error = float(np.mean(np.abs(shown - h[window])))
        if best is None or error < best[1] - 1e-9:
            best = (shift, error)
    return best


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("table")
    p.add_argument("--simulated", help="simulated ascent (t_s, speed_kmh), T+0 = release")
    p.add_argument("--from-mps", type=float, default=8.0)
    p.add_argument("--to-mps", type=float, default=40.0)
    p.add_argument("--alt-from", type=float, default=12.0)
    p.add_argument("--alt-to", type=float, default=32.0)
    a = p.parse_args()
    t, v = window(readings(a.table), a.from_mps, a.to_mps)
    if len(t) < 4:
        raise SystemExit("too few readings in the speed window")
    root, accel = quadratic_root(t, v)
    print(f"{a.table}: {len(t)} readings T+{t.min():.2f}..{t.max():.2f} s ({a.from_mps:.0f}-{a.to_mps:.0f} m/s)")
    print(f"  quadratic: speed zero at T+{root:.2f} s, acceleration {accel:.2f} m/s^2 at T+{0.5 * (t.min() + t.max()):.1f} s")
    lag, error = altitude_lag(a.table, a.alt_from, a.alt_to)
    print(f"  altitude dial: {lag:.2f} s behind the speed dial (mean error {error:.0f} m, readings "
          f"T+{a.alt_from:.0f}..{a.alt_to:.0f} s); its time base T+{root + lag:.2f} s")
    if a.simulated:
        sim = readings(a.simulated, "speed_kmh")
        st, sv = sim[:, 0], sim[:, 1]
        rising = np.maximum.accumulate(sv)
        shifts = [ti - np.interp(vi, rising, st) for ti, vi in zip(t, v)]
        delta = float(np.median(shifts))
        st_w, sv_w = window(sim, a.from_mps, a.to_mps)
        _, sim_accel = quadratic_root(st_w, sv_w)
        print(f"  model: delta {delta:.2f} s (readings {min(shifts):.2f}..{max(shifts):.2f} s); simulated acceleration "
              f"{sim_accel:.2f} m/s^2 at {0.5 * (a.from_mps + a.to_mps):.0f} m/s window centre")


if __name__ == "__main__":
    main()
