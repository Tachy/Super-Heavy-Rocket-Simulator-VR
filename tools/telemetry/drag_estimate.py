"""Drag of the falling booster from webcast telemetry (tables written by webcast_telemetry.py).

In unpowered flight along the path:  D/m = -dv/dt - g sin(gamma),  sin(gamma) = (dh/dt) / v,
so the ballistic coefficient beta = m / (C_D A) = rho v^2 / (2 D/m) follows from speed and altitude alone;
C_D needs the (non-public) mass: C_D = 2 (D/m) m / (rho v^2 A).

  python tools/telemetry/drag_estimate.py <table.csv> --from <t_s> --to <t_s> [--mass-t 347] [--area-m2 63.6]
      [--vehicle ship]   (the ship's entry: drag along the path, whatever its attitude; lift acts across it)

Derivatives come from local least-squares lines (speed +-2 s, altitude +-6 s: the webcast rounds altitude to whole
km). Air data: US Standard Atmosphere 1976 below 86 km. Assumptions: displayed speed relative to the ground, no
wind, flat Earth gravity 9.81 m/s^2 scaled with altitude. Prints t, h, v, Mach, rho, D/m, beta, C_D.
"""
import argparse
import csv
import math

R_AIR = 287.053  # J/(kg K)
G0 = 9.80665
EARTH_RADIUS_M = 6356766.0  # US76 geopotential reference
GAMMA_AIR = 1.4
# US76 layers: geopotential base height [m], base temperature [K], lapse rate [K/m], base pressure [Pa]
LAYERS = [(0.0, 288.15, -0.0065, 101325.0), (11000.0, 216.65, 0.0, 22632.06), (20000.0, 216.65, 0.001, 5474.889),
          (32000.0, 228.65, 0.0028, 868.0187), (47000.0, 270.65, 0.0, 110.9063), (51000.0, 270.65, -0.0028, 66.93887),
          (71000.0, 214.65, -0.002, 3.956420), (84852.0, 186.946, 0.0, 0.3733836)]


def atmosphere(h_m: float):
    """Density [kg/m^3] and speed of sound [m/s] at geometric height h (US76, below 86 km)."""
    z = EARTH_RADIUS_M * h_m / (EARTH_RADIUS_M + h_m)
    base = max(layer for layer in LAYERS if layer[0] <= z)
    hb, tb, lapse, pb = base
    t = tb + lapse * (z - hb)
    if lapse == 0.0:
        p = pb * math.exp(-G0 * (z - hb) / (R_AIR * tb))
    else:
        p = pb * (t / tb) ** (-G0 / (R_AIR * lapse))
    return p / (R_AIR * t), math.sqrt(GAMMA_AIR * R_AIR * t)


def slope(ts, ys, t, half):
    pts = [(a, b) for a, b in zip(ts, ys) if abs(a - t) <= half]
    if len(pts) < 3:
        return None, None
    n = len(pts)
    mt = sum(a for a, _ in pts) / n
    my = sum(b for _, b in pts) / n
    stt = sum((a - mt) ** 2 for a, _ in pts)
    if stt == 0.0:
        return None, None
    k = sum((a - mt) * (b - my) for a, b in pts) / stt
    return k, my + k * (t - mt)


def quadratic(ts, ys, t, half):
    """Altitude, rate and second derivative at t from a least-squares parabola over +-half (None: too few)."""
    pts = [(a - t, b) for a, b in zip(ts, ys) if abs(a - t) <= half]
    if len(pts) < 6:
        return None
    S = [[sum(x ** (i + j) for x, _ in pts) for j in range(3)] for i in range(3)]
    b = [sum(y * x ** i for x, y in pts) for i in range(3)]
    for i in range(3):
        for k in range(i + 1, 3):
            f = S[k][i] / S[i][i]
            for j in range(3):
                S[k][j] -= f * S[i][j]
            b[k] -= f * b[i]
    c = [0.0] * 3
    for i in (2, 1, 0):
        c[i] = (b[i] - sum(S[i][j] * c[j] for j in range(i + 1, 3))) / S[i][i]
    return c[0], c[1], 2.0 * c[2]


def load(path, vehicle="booster"):
    rows = list(csv.DictReader(line for line in open(path) if not line.startswith("#")))
    speed = [(float(r["t_s"]), float(r[f"{vehicle}_speed_kmh"]) / 3.6) for r in rows if r[f"{vehicle}_speed_kmh"]]
    alt = [(float(r["t_s"]), float(r[f"{vehicle}_altitude_km"]) * 1e3) for r in rows if r[f"{vehicle}_altitude_km"]]
    return speed, alt


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("table")
    p.add_argument("--from", dest="t_from", type=float, required=True)
    p.add_argument("--to", dest="t_to", type=float, required=True)
    p.add_argument("--mass-t", type=float, default=347.0, help="assumed booster mass [t] for C_D")
    p.add_argument("--area-m2", type=float, default=63.62, help="reference area (9 m diameter)")
    p.add_argument("--step", type=float, default=2.0)
    p.add_argument("--vehicle", choices=["booster", "ship"], default="booster")
    p.add_argument("--alt-window", type=float, default=6.0, help="half window of the altitude slope [s]")
    p.add_argument("--vertical-fallback", action="store_true",
                   help="without a speed reading, assume a vertical fall and use the altitude (0.1 km dials)")
    a = p.parse_args()
    speed, alt = load(a.table, a.vehicle)
    st, sv = zip(*speed)
    at, ah = zip(*alt)
    print(" t[s]   h[km]  v[m/s]  Mach  rho[kg/m3]  D/m[m/s2]  beta[kg/m2]  C_D")
    t = a.t_from
    while t <= a.t_to:
        dv, v = slope(st, sv, t, 2.0)
        dh, h = slope(at, ah, t, a.alt_window)
        if dv is None and a.vertical_fallback:
            # No speed reading (round dials: the booster's speed gives way to its engine icon below ~9 km): falling
            # nearly vertically, speed and its rate come from a quadratic fit of the altitude (+-4 s).
            fit = quadratic(at, ah, t, 4.0)
            if fit is not None:
                h, rate, accel = fit
                v, dv, dh = -rate, -accel, rate
        if dv is not None and dh is not None and v and v > 50.0:
            rho, sound = atmosphere(max(h, 0.0))
            g = G0 * (EARTH_RADIUS_M / (EARTH_RADIUS_M + h)) ** 2
            sin_gamma = max(-1.0, min(1.0, dh / v))
            drag = -dv - g * sin_gamma
            q = 0.5 * rho * v * v
            beta = q / drag if drag > 0 else float("nan")
            cd = drag * a.mass_t * 1e3 / (q * a.area_m2)
            print(f"{t:6.1f} {h / 1e3:6.1f} {v:7.1f} {v / sound:5.2f}  {rho:9.4f}  {drag:9.2f}  {beta:10.0f}  {cd:5.2f}")
        t += a.step


if __name__ == "__main__":
    main()
