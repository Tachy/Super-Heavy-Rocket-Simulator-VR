"""Drag and lift of the entering ship from webcast telemetry (tables written by webcast_telemetry.py).

Along the path D/m = -dv/dt - g sin(gamma). Across it, in the vertical plane, the lift component follows from the
curvature of the altitude: L_v/m = v dgamma/dt + (g - v^2/r) cos(gamma) - 2 Omega v cos(lat) (Coriolis of
eastward flight; the displayed speed is relative to the rotating Earth). Bank is unknown, so L_v/D is a lower
bound of the vehicle's L/D.

  python tools/telemetry/entry_estimate.py <table.csv> --from <t_s> --to <t_s> [--step 40] [--latitude-deg 10]

Speed slope over +-3 s, altitude parabola over +-20 s (0.1 km dials). Air data: US76 (drag_estimate.atmosphere).
"""
import argparse
import math

from drag_estimate import atmosphere, load, quadratic, slope

EARTH_ROTATION_RADPS = 7.2921159e-5  # IERS
EARTH_RADIUS_M = 6371.0e3            # mean radius (local curvature of the path)
G0 = 9.80665


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("table")
    p.add_argument("--from", dest="t_from", type=float, required=True)
    p.add_argument("--to", dest="t_to", type=float, required=True)
    p.add_argument("--step", type=float, default=40.0)
    p.add_argument("--latitude-deg", type=float, default=10.0, help="mean latitude of the entry ground track")
    a = p.parse_args()
    speed, alt = load(a.table, "ship")
    st, sv = zip(*speed)
    at, ah = zip(*alt)
    print("  t[s]  h[km]  v[m/s]  Mach   D/m   Lv/m  Lv/D  beta[kg/m2]")
    t = a.t_from
    while t <= a.t_to:
        dv, v = slope(st, sv, t, 3.0)
        fit = quadratic(at, ah, t, 20.0)
        if dv is not None and fit is not None and v > 50.0:
            h, hd, hdd = fit
            g = G0 * (EARTH_RADIUS_M / (EARTH_RADIUS_M + h)) ** 2
            sin_g = max(-1.0, min(1.0, hd / v))
            cos_g = math.sqrt(1.0 - sin_g * sin_g)
            drag = -dv - g * sin_g
            gamma_rate = (hdd - dv * sin_g) / (v * cos_g)
            lift_v = (v * gamma_rate + (g - v * v / (EARTH_RADIUS_M + h)) * cos_g
                      - 2.0 * EARTH_ROTATION_RADPS * v * math.cos(math.radians(a.latitude_deg)))
            rho, sound = atmosphere(max(h, 0.0))
            ok = drag > 0.3
            print(f"{t:7.1f} {h / 1e3:5.1f} {v:7.0f} {v / sound:5.1f} {drag:6.2f} {lift_v:6.2f} "
                  f"{lift_v / drag if ok else float('nan'):5.2f} {0.5 * rho * v * v / drag if ok else float('nan'):8.0f}")
        t += a.step


if __name__ == "__main__":
    main()
