"""Vehicle masses and thrust from webcast telemetry with the propellant gauge (round-dial layout, flight 10 on).

During a powered segment of the stack (booster burning) or of the ship alone (--vehicle ship):
  mass       m(t) = M_rest + C p(t)                  p: gauge fraction, C: propellant capacity (100 %)
  flow       mdot = -C dp/dt
  thrust     F    = Isp(h) g0 mdot                   Isp between sea level and vacuum by ambient pressure
  motion     F    = m (dv/dt + g sin(gamma) + D/m)
Telemetry alone fixes only M_rest / C (both sides scale with C); C comes from a source (--capacity-t). The fit
gives M_rest (booster dry + whole ship + anything not in the gauge), the liftoff mass and the thrust per running
engine, to compare with the engine rating.

  python tools/telemetry/mass_estimate.py <table.csv> --capacity-t 3650 --isp-sl 330 --isp-vac 350 \\
      --from 20 --to 140 [--engine-rating-tf 280] [--cd 0.5]
"""
import argparse
import csv
import math
import statistics

from drag_estimate import EARTH_RADIUS_M, G0, atmosphere, slope

P0 = 101325.0
AREA_M2 = 63.62  # 9 m diameter


def load(path, vehicle):
    rows = list(csv.DictReader(line for line in open(path) if not line.startswith("#")))

    def series(col, scale):
        return [(float(r["t_s"]), float(r[col]) * scale) for r in rows if r.get(col)]

    return (series(f"{vehicle}_speed_kmh", 1 / 3.6), series(f"{vehicle}_altitude_km", 1e3),
            series(f"{vehicle}_propellant_pct", 0.01), series(f"{vehicle}_engines_lit", 1.0))


def pressure_ratio(h):
    rho, a = atmosphere(max(h, 0.0))
    t = a * a / (1.4 * 287.053)
    return rho * 287.053 * t / P0


def polyfit2(pts):
    """Least-squares h = c0 + c1 t + c2 t^2 (normal equations, t centred for conditioning)."""
    tm = sum(t for t, _ in pts) / len(pts)
    S = [[0.0] * 3 for _ in range(3)]
    b = [0.0] * 3
    for t, h in pts:
        x = [1.0, t - tm, (t - tm) ** 2]
        for i in range(3):
            b[i] += x[i] * h
            for j in range(3):
                S[i][j] += x[i] * x[j]
    # Gaussian elimination
    for i in range(3):
        for k in range(i + 1, 3):
            f = S[k][i] / S[i][i]
            for j in range(3):
                S[k][j] -= f * S[i][j]
            b[k] -= f * b[i]
    c = [0.0] * 3
    for i in (2, 1, 0):
        c[i] = (b[i] - sum(S[i][j] * c[j] for j in range(i + 1, 3))) / S[i][i]
    # back to uncentred: h = c0 + c1 (t-tm) + c2 (t-tm)^2
    return [c[0] - c[1] * tm + c[2] * tm * tm, c[1] - 2.0 * c[2] * tm, c[2]]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("table")
    ap.add_argument("--vehicle", choices=["booster", "ship"], default="booster")
    ap.add_argument("--capacity-t", type=float, required=True, help="propellant at 100 %% gauge [t]")
    ap.add_argument("--isp-sl", type=float, required=True)
    ap.add_argument("--isp-vac", type=float, required=True)
    ap.add_argument("--from", dest="t_from", type=float, default=20.0)
    ap.add_argument("--to", dest="t_to", type=float, default=140.0)
    ap.add_argument("--cd", type=float, default=0.5, help="stack drag coefficient on the 9 m disc (ascent)")
    ap.add_argument("--engine-rating-tf", type=float, default=0.0, help="sea-level rating per engine, to compare")
    ap.add_argument("--step", type=float, default=2.0)
    a = ap.parse_args()
    speed, alt, prop, eng = load(a.table, a.vehicle)
    st, sv = zip(*speed)
    at, ah = zip(*alt)
    pt, pv = zip(*prop)
    C = a.capacity_t * 1e3

    # Ship: quadratic fit of the altitude (window -100 s .. +30 s, anchored by the stack altitude before staging),
    # iterated with 3-sigma rejection of misreadings.
    pts = [(tt, hh) for tt, hh in alt if a.t_from - 100.0 <= tt <= a.t_to + 30.0]  # incl. the stack before staging
    coef = None
    for _ in range(5):
        if len(pts) < 6:
            break
        coef = polyfit2(pts)
        res = [hh - (coef[0] + coef[1] * tt + coef[2] * tt * tt) for tt, hh in pts]
        sigma = math.sqrt(sum(r * r for r in res) / len(res))
        pts = [p for p, r in zip(pts, res) if abs(r) <= max(3.0 * sigma, 500.0)]

    def altitude_poly(t):
        if coef is None:
            return None, None
        return coef[1] + 2.0 * coef[2] * t, coef[0] + coef[1] * t + coef[2] * t * t

    # Samples: x = p (gauge), y = a_T (thrust acceleration without drag), f = flow per capacity (-dp/dt), isp.
    samples = []
    t = a.t_from
    while t <= a.t_to:
        dv, v = slope(st, sv, t, 2.0)
        if a.vehicle == "ship":
            dh, h = altitude_poly(t)  # the ship's altitude is missing while the booster is shown: smooth fit
        else:
            dh, h = slope(at, ah, t, 3.0)
        dp, p = slope(pt, pv, t, 4.0)
        if None not in (dv, dh, dp) and v > 20.0 and dp < 0.0:
            g = G0 * (EARTH_RADIUS_M / (EARTH_RADIUS_M + h)) ** 2
            sin_g = max(-1.0, min(1.0, dh / v))
            ratio = pressure_ratio(h)
            isp = a.isp_vac - (a.isp_vac - a.isp_sl) * ratio
            rho, _ = atmosphere(max(h, 0.0))
            lit = [n for tt, n in eng if abs(tt - t) <= 1.0]
            samples.append((t, h, v, p, dv + g * sin_g, -dp, isp, 0.5 * rho * v * v, lit[0] if lit else None))
        t += a.step
    # m a_T + D = F:  (M + C p) a_T + q Cd A = Isp g0 C f   ->  M a_T = C (Isp g0 f - p a_T) - q Cd A
    # least squares for M (one unknown).
    # Robust: M from every sample, median (single misreadings would dominate a least-squares fit).
    samples = [x for x in samples if 0.0 < x[3] <= 1.0 and 0.5 < x[4] < 50.0 and 0.0 < x[5] < 0.02]
    per_sample = sorted((C * (isp * G0 * f - p * aT) - q * a.cd * AREA_M2) / aT
                        for _, _, _, p, aT, f, isp, q, _ in samples)
    M = statistics.median(per_sample)
    quart = (per_sample[len(per_sample) // 4], per_sample[(3 * len(per_sample)) // 4])
    print(f"{len(samples)} samples T+{a.t_from:.0f}..{a.t_to:.0f} s; capacity {a.capacity_t:.0f} t")
    print(f"M_rest ({'booster dry + ship + unmetered' if a.vehicle == 'booster' else 'ship dry + payload + unmetered'})"
          f" = {M / 1e3:.0f} t (median; quartiles {quart[0] / 1e3:.0f}..{quart[1] / 1e3:.0f} t)")
    print(" t[s]  h[km]  v[m/s]  gauge%   mass[t]  thrust[MN]  engines  per engine [tf]  rating share  resid%")
    for t, h, v, p, aT, f, isp, q, lit in samples:
        m = M + C * p
        thrust_motion = m * aT + q * a.cd * AREA_M2
        thrust_flow = isp * G0 * C * f
        per = thrust_flow / lit / 9806.65 if lit else float("nan")
        # sea-level-equivalent rating share: per-engine thrust scaled by Isp_sl/Isp(h)
        share = per * a.isp_sl / isp / a.engine_rating_tf if a.engine_rating_tf and lit else float("nan")
        print(f"{t:5.0f} {h / 1e3:6.1f} {v:7.1f}  {100 * p:6.1f}  {m / 1e3:8.0f}  {thrust_flow / 1e6:9.1f}"
              f"  {lit if lit else '-':>7}  {per:14.0f}  {share:11.2f}  {100 * (thrust_motion / thrust_flow - 1):6.1f}")
    p0 = next((v for tt, v in prop if tt >= 0.0), None)
    if p0 is not None:
        print(f"liftoff: gauge {100 * p0:.1f} % -> propellant {C * p0 / 1e3:.0f} t, total mass {(M + C * p0) / 1e3:.0f} t")


if __name__ == "__main__":
    main()
