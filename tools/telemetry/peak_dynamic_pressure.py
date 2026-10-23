"""Dynamic pressure of a recorded ascent from webcast telemetry (tables written by webcast_telemetry.py).

  q = rho(h) v^2 / 2   with v the displayed speed (relative to the ground: no wind, the air turns with the Earth)
                       and rho the US Standard Atmosphere 1976 density at the displayed altitude.

  python tools/telemetry/peak_dynamic_pressure.py <table.csv> [--to 180] [--vehicle booster]
      [--capacity-t 425 --rated-flow-tps 2.75]   (throttled phases from the propellant gauge)

Speed and altitude are smoothed by local least-squares lines (speed +-2 s, altitude +-3 s: the dials show 0.1 km)
and evaluated every --step seconds up to --to (before the staging: the stack). Air data: drag_estimate.atmosphere
(US76 layer equations below 86 km, the model the core's environment::Us76Atmosphere implements). Prints the q
profile around the peak and the peak itself (time, altitude, speed, Mach, q).

With --capacity-t and --rated-flow-tps (all engines at their rating) the gauge's flow share (gauge slope over +-4 s,
times the capacity, over the rated flow) is printed too, and the phases throttled below --throttled-below: their
Mach range and the highest q held in them (a q limit the guidance flew to), against the highest q flown at full
thrust (no evidence of a limit there).
"""
import argparse
import csv

from drag_estimate import atmosphere, load, slope

SPEED_HALF_WINDOW_S = 2.0
ALTITUDE_HALF_WINDOW_S = 3.0
GAUGE_HALF_WINDOW_S = 4.0


def profile(speed, alt, t_from, t_to, step):
    """[(t, h, v, mach, q)] where both smoothed readings exist."""
    st, sv = zip(*speed)
    at, ah = zip(*alt)
    rows = []
    t = t_from
    while t <= t_to + 1e-9:
        _, v = slope(st, sv, t, SPEED_HALF_WINDOW_S)
        _, h = slope(at, ah, t, ALTITUDE_HALF_WINDOW_S)
        if v is not None and h is not None:
            rho, sound = atmosphere(max(h, 0.0))
            rows.append((t, h, v, v / sound, 0.5 * rho * v * v))
        t += step
    return rows


def gauge(path, vehicle):
    rows = list(csv.DictReader(line for line in open(path) if not line.startswith("#")))
    col = f"{vehicle}_propellant_pct"
    return [(float(r["t_s"]), float(r[col]) * 0.01) for r in rows if r.get(col)]


def flow_share(prop, t, capacity_kg, rated_kgps):
    pt, pv = zip(*prop)
    dp, _ = slope(pt, pv, t, GAUGE_HALF_WINDOW_S)
    return None if dp is None else -dp * capacity_kg / rated_kgps


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("table")
    p.add_argument("--from", dest="t_from", type=float, default=10.0)
    p.add_argument("--to", dest="t_to", type=float, default=150.0, help="before the staging")
    p.add_argument("--step", type=float, default=1.0)
    p.add_argument("--vehicle", choices=["booster", "ship"], default="booster")
    p.add_argument("--around", type=float, default=20.0, help="print the profile +- this around the peak [s]")
    p.add_argument("--capacity-t", type=float, default=0.0, help="propellant at 100 %% gauge [t]")
    p.add_argument("--rated-flow-tps", type=float, default=0.0, help="all engines' flow at their rating [t/s]")
    p.add_argument("--throttled-below", type=float, default=0.95, help="flow share of a throttled phase")
    a = p.parse_args()
    speed, alt = load(a.table, a.vehicle)
    rows = profile(speed, alt, a.t_from, a.t_to, a.step)
    if not rows:
        raise SystemExit("no speed and altitude readings in the interval")
    prop = gauge(a.table, a.vehicle) if a.capacity_t and a.rated_flow_tps else []
    shares = [flow_share(prop, r[0], a.capacity_t * 1e3, a.rated_flow_tps * 1e3) if prop else None for r in rows]
    peak = max(rows, key=lambda r: r[4])
    print(" t[s]   h[km]  v[m/s]  Mach   q[kPa]" + ("  flow" if prop else ""))
    for (t, h, v, mach, q), share in zip(rows, shares):
        if abs(t - peak[0]) <= a.around:
            extra = f"  {share:4.2f}" if share is not None else ""
            print(f"{t:6.1f} {h / 1e3:6.2f} {v:7.1f} {mach:5.2f} {q / 1e3:8.2f}{extra}")
    t, h, v, mach, q = peak
    print(f"peak q {q:.0f} Pa at T+{t:.1f} s (h {h / 1e3:.2f} km, v {v:.1f} m/s, Mach {mach:.2f})")
    if not prop:
        return
    # throttled phases after the engines are at thrust (t > 15 s): contiguous rows below the share
    phases, current = [], []
    for row, share in zip(rows, shares):
        if share is not None and share < a.throttled_below and row[0] > 15.0:
            current.append(row)
        elif current:
            phases.append(current)
            current = []
    if current:
        phases.append(current)
    throttled = set()
    for ph in phases:
        if len(ph) < 3:
            continue
        top = max(ph, key=lambda r: r[4])
        throttled.update(r[0] for r in ph)
        print(f"throttled T+{ph[0][0]:.0f}..{ph[-1][0]:.0f} s: Mach {ph[0][3]:.2f}..{ph[-1][3]:.2f}, q held up to "
              f"{top[4]:.0f} Pa (Mach {top[3]:.2f})")
    full = [r for r in rows if r[0] not in throttled]
    if full:
        top = max(full, key=lambda r: r[4])
        print(f"full thrust: highest q {top[4]:.0f} Pa at T+{top[0]:.0f} s (Mach {top[3]:.2f})")


if __name__ == "__main__":
    main()
