"""Thrust vector of a Falcon 9 first stage around boostback ignition, from the webcast speed and altitude dials.

In the flight plane: a_along = dV/dt + g sin(gamma); a_normal = V dgamma/dt + (g - V^2/r) cos(gamma). The angle from the
velocity (0 along it, 180 against it; positive towards local up) is the body axis while the thrust dominates, so it shows
whether the turn goes on under the engines, and the magnitude shows the engine build-up. Rounded dials: central differences
over +-window s. The dials carry their own time bases (liftoff_time.py): only differences of the bases matter here.

  python tools/telemetry/flip_thrust_direction.py tests/validation/telemetry/f9_crs35_2026-10-13.csv --from 145 --to 200
"""
import argparse
import csv
import math

MU = 3.986004418e14
R_LOCAL = 6373000.0  # [E] Earth radius near the Cape (28.5 deg), as boostback_fit.py
ALTITUDE_MINUS_SPEED_BASE_S = 2.45  # [D] CRS-35 dial time bases 5.87 - 3.42 s (boostback_fit.py)


def load(path):
    rows = [r for r in open(path) if not r.startswith("#")]
    speed, altitude = [], []
    for r in csv.DictReader(rows):
        t = float(r["t_s"])
        if r["booster_speed_kmh"]:
            speed.append((t, float(r["booster_speed_kmh"]) / 3.6))
        if r["booster_altitude_km"]:
            altitude.append((t - ALTITUDE_MINUS_SPEED_BASE_S, float(r["booster_altitude_km"]) * 1e3))
    return speed, altitude


def slope(series, t, window):
    points = [(a, b) for a, b in series if abs(a - t) <= window]
    n = len(points)
    if n < 3:
        return None, None
    mean_t = sum(p[0] for p in points) / n
    mean_y = sum(p[1] for p in points) / n
    denominator = sum((p[0] - mean_t) ** 2 for p in points)
    rate = sum((p[0] - mean_t) * (p[1] - mean_y) for p in points) / denominator
    return mean_y + rate * (t - mean_t), rate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("table")
    parser.add_argument("--from", dest="start", type=float, default=145.0)
    parser.add_argument("--to", type=float, default=200.0)
    parser.add_argument("--step", type=float, default=1.0)
    parser.add_argument("--window", type=float, default=1.5)
    args = parser.parse_args()
    speed, altitude = load(args.table)
    print("t_s  V_mps gamma_deg  dV_mps2 a_along a_normal |a|  from_velocity_deg axis_elev_downrange_deg")
    t = args.start
    while t <= args.to:
        v, v_rate = slope(speed, t, args.window)
        h, h_rate = slope(altitude, t, args.window)
        v1, _ = slope(speed, t + args.step, args.window)
        h1, h1_rate = slope(altitude, t + args.step, args.window)
        v0, _ = slope(speed, t - args.step, args.window)
        h0, h0_rate = slope(altitude, t - args.step, args.window)
        if None in (v, h, v1, h1, v0, h0):
            t += args.step
            continue
        gamma = math.asin(max(-1.0, min(1.0, h_rate / v)))
        gamma_1 = math.asin(max(-1.0, min(1.0, h1_rate / v1)))
        gamma_0 = math.asin(max(-1.0, min(1.0, h0_rate / v0)))
        gamma_rate = (gamma_1 - gamma_0) / (2.0 * args.step)
        r = R_LOCAL + h
        g = MU / r**2
        along = v_rate + g * math.sin(gamma)
        normal = v * gamma_rate + (g - v**2 / r) * math.cos(gamma)
        off = math.degrees(math.atan2(normal, along))
        print(f"{t:6.1f} {v:7.1f} {math.degrees(gamma):6.1f} {v_rate:7.1f} {along:7.1f} {normal:7.1f} "
              f"{math.hypot(along, normal):6.1f} {off:7.1f} {math.degrees(gamma) + off:7.1f}")
        t += args.step


if __name__ == "__main__":
    main()
