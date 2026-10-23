"""Peak dynamic pressure of a recorded ascent from webcast telemetry (tables written by webcast_telemetry.py).

  q = rho(h) v^2 / 2   with v the displayed speed (relative to the ground: no wind, the air turns with the Earth)
                       and rho the US Standard Atmosphere 1976 density at the displayed altitude.

  python tools/telemetry/peak_dynamic_pressure.py <table.csv> [--to 180] [--vehicle booster]

Speed and altitude are smoothed by local least-squares lines (speed +-2 s, altitude +-3 s: the dials show 0.1 km)
and evaluated every --step seconds up to --to (before the staging: the stack). Air data: drag_estimate.atmosphere
(US76 layer equations below 86 km, the model the core's environment::Us76Atmosphere implements). Prints the q
profile around the peak and the peak itself (time, altitude, speed, Mach, q).
"""
import argparse

from drag_estimate import atmosphere, load, slope

SPEED_HALF_WINDOW_S = 2.0
ALTITUDE_HALF_WINDOW_S = 3.0


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


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("table")
    p.add_argument("--from", dest="t_from", type=float, default=10.0)
    p.add_argument("--to", dest="t_to", type=float, default=150.0, help="before the staging")
    p.add_argument("--step", type=float, default=1.0)
    p.add_argument("--vehicle", choices=["booster", "ship"], default="booster")
    p.add_argument("--around", type=float, default=20.0, help="print the profile +- this around the peak [s]")
    a = p.parse_args()
    speed, alt = load(a.table, a.vehicle)
    rows = profile(speed, alt, a.t_from, a.t_to, a.step)
    if not rows:
        raise SystemExit("no speed and altitude readings in the interval")
    peak = max(rows, key=lambda r: r[4])
    print(" t[s]   h[km]  v[m/s]  Mach   q[kPa]")
    for t, h, v, mach, q in rows:
        if abs(t - peak[0]) <= a.around:
            print(f"{t:6.1f} {h / 1e3:6.2f} {v:7.1f} {mach:5.2f} {q / 1e3:8.2f}")
    t, h, v, mach, q = peak
    print(f"peak q {q:.0f} Pa at T+{t:.1f} s (h {h / 1e3:.2f} km, v {v:.1f} m/s, Mach {mach:.2f})")


if __name__ == "__main__":
    main()
