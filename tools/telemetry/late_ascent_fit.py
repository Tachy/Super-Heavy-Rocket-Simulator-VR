"""Late first-stage burn: thrust per mass from the speed and altitude dials (follow-ups Task 7c).

With a constant thrust F and mass flow mdot the sensed acceleration along the path a = F / m gives
    1 / a(t) = m(t) / F = m0 / F - t / c,        c = F / mdot (effective exhaust speed)
so a straight line through 1/a over the late burn (drag < 0.5 % of the thrust above ~30 km) separates the two:
its slope gives c (thrust per mass flow - an Isp or thrust error moves it), its level gives m / F (a mass error or a
thrust/throttle error with the rated Isp moves it; the propellant gauge's flow tells the two apart).

a is taken from the dials: dv/dt (least-squares slope over +-half window) + g(h) sin(gamma), gamma from the altitude
dial's slope over the speed. The same estimator runs on a simulated dump (STARSIM_ASCENT_DUMP) so its bias cancels;
the dump's own thrust and mass give the truth for the simulated stack.

  python tools/telemetry/late_ascent_fit.py tests/validation/telemetry/f9_crs35_2026-10-13.csv \\
      --speed-base 3.42 --altitude-base 5.87 --simulated <dump>/crs35_ascent.csv --from 100 --to 132
"""
import argparse
import csv
import math

MU_EARTH = 3.986004418e14
R_EARTH = 6.378137e6
G0 = 9.80665


def recorded(path, speed_base, altitude_base):
    rows = list(csv.DictReader(line for line in open(path) if not line.startswith("#")))
    v = [(float(r["t_s"]) - speed_base, float(r["booster_speed_kmh"]) / 3.6) for r in rows if r.get("booster_speed_kmh")]
    h = [(float(r["t_s"]) - altitude_base, float(r["booster_altitude_km"]) * 1e3) for r in rows
         if r.get("booster_altitude_km")]
    g = [(float(r["t_s"]), float(r["g_force"])) for r in rows if r.get("g_force")]
    p = [(float(r["t_s"]), float(r["booster_propellant_pct"])) for r in rows if r.get("booster_propellant_pct")]
    return v, h, g, p


def simulated(path):
    rows = list(csv.DictReader(open(path)))
    out = {k: [] for k in ("v", "h", "m", "f", "g")}
    last = None
    for r in rows:
        t = float(r["t_s"])
        if t == last:
            continue
        last = t
        out["v"].append((t, float(r["speed_kmh"]) / 3.6))
        out["h"].append((t, float(r["altitude_km"]) * 1e3))
        out["m"].append((t, float(r["mass_t"]) * 1e3))
        out["f"].append((t, float(r["thrust_mn"]) * 1e6))
        out["g"].append((t, float(r["sensed_g"])))
    return out


def line(series, t0, half):
    pts = [(t - t0, y) for t, y in series if abs(t - t0) <= half]
    n = len(pts)
    if n < 2:
        return float("nan"), float("nan")
    st = sum(u for u, _ in pts)
    sy = sum(y for _, y in pts)
    stt = sum(u * u for u, _ in pts)
    sty = sum(u * y for u, y in pts)
    slope = (n * sty - st * sy) / (n * stt - st * st)
    return (sy - slope * st) / n, slope


def interp(series, t):
    for (t0, y0), (t1, y1) in zip(series, series[1:]):
        if t0 <= t <= t1:
            return y0 + (y1 - y0) * (t - t0) / (t1 - t0) if t1 > t0 else y0
    return float("nan")


def path_acceleration(v, h, t, half_v, half_h):
    speed, dvdt = line(v, t, half_v)
    alt, dhdt = line(h, t, half_h)
    gamma = math.asin(max(-1.0, min(1.0, dhdt / speed)))
    g = MU_EARTH / (R_EARTH + alt) ** 2
    return dvdt + g * math.sin(gamma), speed, alt, gamma


def fit(ts, inv_a):
    n = len(ts)
    tm = sum(ts) / n
    sxx = sum((t - tm) ** 2 for t in ts)
    slope = sum((t - tm) * y for t, y in zip(ts, inv_a)) / sxx
    level = sum(inv_a) / n
    rms = math.sqrt(sum((level + slope * (t - tm) - y) ** 2 for t, y in zip(ts, inv_a)) / n)
    return tm, level, slope, rms


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("table")
    ap.add_argument("--speed-base", type=float, required=True, help="speed dial lag behind the release [s]")
    ap.add_argument("--altitude-base", type=float, required=True, help="altitude dial lag behind the release [s]")
    ap.add_argument("--simulated", help="STARSIM_ASCENT_DUMP csv (t_s from the release)")
    ap.add_argument("--from", dest="t_from", type=float, default=100.0, help="true time [s]")
    ap.add_argument("--to", dest="t_to", type=float, default=132.0, help="true time [s]")
    ap.add_argument("--step", type=float, default=2.0)
    ap.add_argument("--half-speed", type=float, default=2.0)
    ap.add_argument("--half-altitude", type=float, default=3.0)
    args = ap.parse_args()

    v, h, gdial, gauge = recorded(args.table, args.speed_base, args.altitude_base)
    sim = simulated(args.simulated) if args.simulated else None
    ts = []
    t = args.t_from
    while t <= args.t_to + 1e-9:
        ts.append(t)
        t += args.step

    print("true t   flight: a[m/s2]  v[m/s]  h[km] gam[deg] | sim: a(dials) a(F/m)   m[t]   F[MN] | a ratio sim/flight")
    rec_inv, sim_inv, sim_true_inv = [], [], []
    for t in ts:
        a, s, alt, gam = path_acceleration(v, h, t, args.half_speed, args.half_altitude)
        rec_inv.append(1.0 / a)
        line_txt = f"{t:6.1f}   {a:14.2f} {s:7.1f} {alt / 1e3:6.1f} {math.degrees(gam):7.2f}"
        if sim:
            sa, *_ = path_acceleration(sim["v"], sim["h"], t, args.half_speed, args.half_altitude)
            m = interp(sim["m"], t)
            f = interp(sim["f"], t)
            sim_inv.append(1.0 / sa)
            sim_true_inv.append(m / f)
            line_txt += f" | {sa:12.2f} {f / m:7.2f} {m / 1e3:7.1f} {f / 1e6:6.3f} | {sa / a:6.3f}"
        print(line_txt)

    def report(name, inv):
        tm, level, slope, rms = fit(ts, inv)
        print(f"{name:28s} 1/a at t={tm:5.1f} s: {level * 1e3:7.3f} ms2/m  (a {1 / level:5.2f} m/s2), "
              f"c = {-1 / slope:7.0f} m/s (Isp {-1 / slope / G0:5.1f} s), rms {rms * 1e3:.3f} ms2/m")
        return tm, level, slope

    print()
    rec = report("flight (dials)", rec_inv)
    if sim:
        sd = report("sim (same estimator)", sim_inv)
        st = report("sim (F/m truth)", sim_true_inv)
        tm = rec[0]
        m_sim = interp(sim["m"], tm)
        f_sim = interp(sim["f"], tm)
        bias = sd[1] / st[1]
        print(f"\nestimator bias on the sim: level x{bias:.4f}, c x{(1 / sd[2]) / (1 / st[2]):.4f}")
        ratio = rec[1] / sd[1]
        print(f"at t={tm:.1f} s the flight's m/F is x{ratio:.4f} ours (ours m {m_sim / 1e3:.1f} t, F {f_sim / 1e6:.3f} MN):")
        print(f"  same thrust -> stack {m_sim * ratio / 1e3:.1f} t ({m_sim * (ratio - 1) / 1e3:+.1f} t)")
        print(f"  same mass   -> thrust {f_sim / ratio / 1e6:.3f} MN ({(1 / ratio - 1) * 100:+.1f} %)")
    # G dial and gauge flow for reference (display times)
    print("\nG dial (display time): " + ", ".join(f"{t:.2f}:{g:.1f}" for t, g in gdial if 126 <= t <= 137))
    gp = [(t, p) for t, p in gauge if args.t_from + args.speed_base <= t <= args.t_to + args.speed_base + 4]
    if len(gp) > 2:
        _, dp = line(gp, sum(t for t, _ in gp) / len(gp), 1e9)
        print(f"gauge flow over display {gp[0][0]:.2f}-{gp[-1][0]:.2f} s: {dp:.3f} %/s")


if __name__ == "__main__":
    main()
