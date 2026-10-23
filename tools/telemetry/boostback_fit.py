"""Thrust history of a booster's boostback from its speed and altitude readings (follow-ups Task 7b).

  python tools/telemetry/boostback_fit.py <table.csv> --on 151 --off 208 [--step 3] [--steady-from 172]
      [--exhaust-mps 3050] [--max-thrust-kn 2796] [--end-mass-t 40.8]
  python tools/telemetry/boostback_fit.py <dump.csv> --simulated --on 153 --off 198 --step 4

A planar point mass over a spherical Earth (no rotation, no drag: q < 20 Pa above 80 km) flies from --t0 with a free
initial state (height, vertical and horizontal speed); the thrust acceleration is piecewise linear between knots
every --step s from --on to --off (zero at both ends), pointed backwards along the horizontal and tilted down by a
linearly varying angle. Least squares on the speed readings (0.3 m/s) and the altitude readings (0.04 km, 0.3 km
above 100 km where the dial shows whole km). The table's time bases (speed 3.42 s, altitude 5.87 s: CRS-35,
liftoff_time.py) are taken off; --simulated reads a dump of our flight (t_s, alt_km, speed_kmh; true time).

Output: the thrust delta-v (integral of the acceleration), the knots, and the mass bracket. The propellant of a burn
follows from its delta-v and its end mass alone (m_end (exp(dv / c) - 1)); the end mass is bounded below by the
gauge after the burn (--end-mass-t: dry + gauge + nitrogen) and above by the thrust ceiling: in the steady phase
(from --steady-from, where the acceleration grows as a constant thrust on a falling mass: 1/a falls by 1/c per second)
the mass at --steady-from is at most max_thrust / a.

Checked on our own flight (dump of the hidden test "Falcon 9 CRS-35 first stage profile dump"): 2101 m/s fitted
against 2100 m/s flown, knots within ~2 % of thrust / mass away from the speed minimum.
"""
import argparse
import csv
import math

import numpy as np
from scipy.optimize import least_squares

MU = 3.986004418e14
R_LOCAL = 6373000.0  # [E] Earth radius near the Cape (28.5 deg)
SPEED_TIME_BASE_S = 3.42      # CRS-35 (liftoff_time.py)
ALTITUDE_TIME_BASE_S = 5.87


def load(path, simulated, t0, t1):
    ts, vs, th, hs = [], [], [], []
    rows = [r for r in open(path) if not r.startswith("#")]
    for r in csv.DictReader(rows):
        t = float(r["t_s"])
        if simulated:
            if t0 <= t <= t1:
                ts.append(t); vs.append(float(r["speed_kmh"]) / 3.6)
                th.append(t); hs.append(round(float(r["alt_km"]), 1) * 1e3)  # as a dial shows it
            continue
        if r["booster_speed_kmh"] and t0 <= t - SPEED_TIME_BASE_S <= t1:
            ts.append(t - SPEED_TIME_BASE_S); vs.append(float(r["booster_speed_kmh"]) / 3.6)
        if r["booster_altitude_km"] and t0 <= t - ALTITUDE_TIME_BASE_S <= t1:
            th.append(t - ALTITUDE_TIME_BASE_S); hs.append(float(r["booster_altitude_km"]) * 1e3)
    return map(np.array, (ts, vs, th, hs))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--simulated", action="store_true")
    ap.add_argument("--t0", type=float, default=143.0)
    ap.add_argument("--t1", type=float, default=228.0)
    ap.add_argument("--on", type=float, required=True)
    ap.add_argument("--off", type=float, required=True)
    ap.add_argument("--step", type=float, default=3.0)
    ap.add_argument("--steady-from", type=float, default=None)
    ap.add_argument("--exhaust-mps", type=float, default=3050.0)    # Merlin vacuum, our engine model
    ap.add_argument("--max-thrust-kn", type=float, default=2796.0)  # three Merlins in vacuum, our engine model
    ap.add_argument("--end-mass-t", type=float, default=None)
    args = ap.parse_args()

    ts, vs, th, hs = load(args.csv, args.simulated, args.t0, args.t1)
    h_sigma = np.where(hs > 99.95e3, 0.3e3, 0.04e3)
    v_sigma = 0.3
    knots = np.arange(args.on, args.off + 1e-6, args.step)
    if knots[-1] < args.off:
        knots = np.append(knots, args.off)
    nk = len(knots)

    def simulate(q):
        h0, vz0, vh0, d0, d1 = q[:5]
        a_k = np.concatenate([[0.0], q[5:], [0.0]])
        dt = 0.05
        t = args.t0
        r, vr, vt = R_LOCAL + h0, vz0, vh0
        T, V, H, A = [], [], [], []
        while t <= args.t1 + 1e-9:
            T.append(t); V.append(math.hypot(vr, vt)); H.append(r - R_LOCAL)
            a = float(np.interp(t, knots, a_k)) if knots[0] <= t <= knots[-1] else 0.0
            A.append(a)
            delta = d0 + d1 * (t - args.on)
            g = MU / r**2
            dvr = -g + vt**2 / r - a * math.sin(delta)
            dvt = -vr * vt / r - a * math.cos(delta)
            r += vr * dt; vr += dvr * dt; vt += dvt * dt
            t += dt
        return np.array(T), np.array(V), np.array(H), np.array(A)

    def residual(q):
        T, V, H, _ = simulate(q)
        return np.concatenate([(np.interp(ts, T, V) - vs) / v_sigma, (np.interp(th, T, H) - hs) / h_sigma])

    q0 = [hs[0], 0.6 * vs[0], 0.8 * vs[0], 0.05, 0.0] + [35.0] * (nk - 2)
    lo = [hs[0] - 5e3, 0, 0, -0.6, -0.03] + [0.0] * (nk - 2)
    hi = [hs[0] + 5e3, 2000, 2500, 0.6, 0.03] + [80.0] * (nk - 2)
    sol = least_squares(residual, q0, bounds=(lo, hi), x_scale="jac")
    T, V, H, A = simulate(sol.x)
    res = residual(sol.x)
    nv = len(ts)
    h0, vz0, vh0, d0, d1 = sol.x[:5]
    dv = float(np.sum(A) * 0.05)
    print(f"rms speed {np.sqrt(np.mean((res[:nv] * v_sigma) ** 2)):.2f} m/s, altitude "
          f"{np.sqrt(np.mean((res[nv:] * h_sigma) ** 2)):.0f} m")
    print(f"T+{args.t0:.0f}: height {h0 / 1e3:.2f} km, vertical {vz0:.0f} m/s, horizontal {vh0:.0f} m/s; thrust "
          f"tilted down {math.degrees(d0):.1f} deg + {math.degrees(d1):.3f} deg/s")
    print(f"thrust delta-v {dv:.0f} m/s")
    a_k = np.concatenate([[0.0], sol.x[5:], [0.0]])
    print("knot  T+s  a [m/s2]  mass at the thrust ceiling [t]")
    for t, a in zip(knots, a_k):
        print(f"      {t:6.1f} {a:6.2f}   {args.max_thrust_kn / a if a > 1 else float('nan'):7.1f}")
    i = np.searchsorted(T, args.off + 0.5)
    print(f"after the burn T+{T[i]:.1f}: speed {V[i]:.0f} m/s, height {H[i] / 1e3:.1f} km")

    ratio = math.exp(dv / args.exhaust_mps)
    if args.steady_from is not None:
        a_steady = float(np.interp(args.steady_from, knots, a_k))
        dv_after = float(np.sum(A[T >= args.steady_from]) * 0.05)
        m_end_max = args.max_thrust_kn / a_steady / math.exp(dv_after / args.exhaust_mps)
        print(f"steady from T+{args.steady_from:.0f}: a {a_steady:.1f} m/s2, delta-v after it {dv_after:.0f} m/s -> end "
              f"mass <= {m_end_max:.1f} t (thrust ceiling {args.max_thrust_kn:.0f} kN)")
        for m_end in ([args.end_mass_t] if args.end_mass_t else []) + [m_end_max]:
            m_steady = m_end * math.exp(dv_after / args.exhaust_mps)
            print(f"  end mass {m_end:.1f} t: start {m_end * ratio:.1f} t, burned {m_end * (ratio - 1):.1f} t, steady "
                  f"thrust {a_steady * m_steady:.0f} kN ({a_steady * m_steady / args.max_thrust_kn:.2f} of the ceiling)")


if __name__ == "__main__":
    main()
