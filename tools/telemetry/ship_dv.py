"""Ship dry mass from the delta-v of its ascent burn (ignition to engine cutoff), via the rocket equation.

  m0 / m1 = exp(sum_i dv_i / (Isp_i g0)),  m0 = M + Cm p0,  m1 = M + Cm p1,  M = D + P + H
dv_i is the thrust delta-v of segment i (piecewise-constant engine set): the along-path acceleration plus g sin(gamma)
(gravity loss) and the normal acceleration needed to turn the path (steering), integrated over the burn; the altitude
comes from one polynomial fit over the burn, anchored by the stack's altitude before staging (the ship's altitude dial
is hidden for parts of the burn). Drag is ignored (above ~60 km).
The webcast's propellant gauge shows the main tanks only (`config/README.md`): Cm = capacity - header load is the
metered capacity, and the header load H (the landing propellant, full through the ascent) is not burned in it, so it
sits in M with the dry mass D and the payload P:  D = M - P - H.  (Before 2026-11-03 this tool took Cm = capacity and
H = 0, used only the samples where the ship's altitude was shown for the gravity loss, ignored steering and took the
gauge's last table value at cutoff: 248-260 t "dry", which contained the headers.)
p0: the table's gauge over T+160-260 s extrapolated to ignition (the arc clips at ~98.2 % at the top); p1: the gauge
after cutoff read from frames (--p1; the table's late values sit ~0.45 % above the arc and drop out). Gauge readings
are corrected by the table's bias (--gauge-bias).

  python tools/telemetry/ship_dv.py <table.csv> --ignition-s 146 --p1 2.5 --payload-t 50 [--header-t 35]
      [--capacity-t 1600] [--rvac-out-s 438] [--isp-sl 346.2 --thrust-sl-mn 2.880 --isp-rvac 380 --thrust-rvac-mn 2.98]

The default engines are the config's: Raptor 3 in vacuum (1.30 m nozzle: 2.880 MN, 346.2 s) and RVac 3 (2.98 MN,
380 s); --rvac-out-s drops one RVac from that time on (flight 12 T+183 s, flight 14 T+438 s).
"""
import argparse
import csv
import math

import numpy as np

from drag_estimate import G0

EARTH_MEAN_RADIUS_M = 6371.0e3
EARTH_ROTATION_RAD_S = 7.2921159e-5
STARBASE_LATITUDE_RAD = math.radians(26.0)  # the launch site; the ascent track stays within a few degrees of it


def load(path):
    rows = list(csv.DictReader(line for line in open(path) if not line.startswith("#")))

    def column(name, scale=1.0):
        return np.array([(float(r["t_s"]), float(r[name]) * scale) for r in rows if r.get(name)])

    return (column("ship_speed_kmh", 1 / 3.6), column("ship_altitude_km", 1e3), column("ship_propellant_pct"),
            column("booster_altitude_km", 1e3))


def altitude_fit(alt, stack, t0, t1, degree=4):
    """Polynomial altitude over the burn with outlier rejection; returns h(t), dh/dt, d2h/dt2."""
    pts = np.vstack([stack[(stack[:, 0] > t0 - 60) & (stack[:, 0] < t0)],
                     alt[(alt[:, 0] >= t0) & (alt[:, 0] <= t1 + 40)]])
    tm, scale = 0.5 * (t0 + t1), 300.0
    x = (pts[:, 0] - tm) / scale
    keep = np.ones(len(x), bool)
    for _ in range(4):
        c = np.polyfit(x[keep], pts[keep, 1], degree)
        res = pts[:, 1] - np.polyval(c, x)
        keep = np.abs(res) < max(3 * res[keep].std(), 600.0)
    d1, d2 = np.polyder(c), np.polyder(c, 2)
    return (lambda t: np.polyval(c, (t - tm) / scale), lambda t: np.polyval(d1, (t - tm) / scale) / scale,
            lambda t: np.polyval(d2, (t - tm) / scale) / scale ** 2)


def local_slope(series, t, half):
    m = np.abs(series[:, 0] - t) <= half
    if m.sum() < 4:
        return None, None
    c = np.polyfit(series[m, 0] - t, series[m, 1], 1)
    return c[0], c[1]


def thrust_dv(speed, alt, stack, t0, t1):
    """Per 1 s step: time, along-path and normal thrust acceleration [m/s^2], gravity term [m/s^2]."""
    h, hd, hdd = altitude_fit(alt, stack, t0, t1)
    out = []
    for t in np.arange(t0, t1, 1.0):
        dv, v = local_slope(speed, t, 2.0)
        if dv is None:
            continue
        r = EARTH_MEAN_RADIUS_M + h(t)
        g = G0 * (EARTH_MEAN_RADIUS_M / r) ** 2
        sg = float(np.clip(hd(t) / v, -1.0, 1.0))
        cg = math.sqrt(1.0 - sg * sg)
        gamma_rate = (hdd(t) - dv * sg) / (v * cg)
        along = dv + g * sg
        normal = v * gamma_rate + (g - v * v / r) * cg - 2 * EARTH_ROTATION_RAD_S * v * math.cos(STARBASE_LATITUDE_RAD)
        out.append((t, along, normal, g * sg))
    return np.array(out)


def isp_mix(n_sl, n_rvac, a):
    f_sl, f_rv = n_sl * a.thrust_sl_mn, n_rvac * a.thrust_rvac_mn
    return (f_sl + f_rv) / (f_sl / a.isp_sl + f_rv / a.isp_rvac)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("table")
    ap.add_argument("--ignition-s", type=float, required=True, help="ship ignition (hot staging) [s after liftoff]")
    ap.add_argument("--p1", type=float, required=True, help="gauge after cutoff [%%], read from frames")
    ap.add_argument("--p0", type=float, default=None, help="gauge at ignition [%%] (default: extrapolated)")
    ap.add_argument("--payload-t", type=float, default=0.0)
    ap.add_argument("--header-t", type=float, default=35.0, help="header (landing) propellant, not on the gauge [t]")
    ap.add_argument("--capacity-t", type=float, default=1600.0, help="total propellant capacity incl. headers [t]")
    ap.add_argument("--gauge-bias", type=float, default=0.45, help="table gauge minus the frames' arc [%%]")
    ap.add_argument("--rvac-out-s", type=float, default=None, help="one RVac out from this time on")
    ap.add_argument("--isp-sl", type=float, default=346.2)
    ap.add_argument("--thrust-sl-mn", type=float, default=2.880)
    ap.add_argument("--isp-rvac", type=float, default=380.0)
    ap.add_argument("--thrust-rvac-mn", type=float, default=2.98)
    a = ap.parse_args()

    speed, alt, gauge, stack = load(a.table)
    speed = speed[speed[:, 1] > 100.0]
    early = speed[speed[:, 0] < 700.0]
    t1 = early[np.argmax(early[:, 1]), 0]
    t0 = a.ignition_s
    s = thrust_dv(speed, alt, stack, t0, t1)
    T, AL, AN, GV = s[:, 0], s[:, 1], s[:, 2], s[:, 3]
    AT = np.hypot(AL, AN)
    v0 = float(np.interp(t0, speed[:, 0], speed[:, 1]))
    v1 = float(early[np.argmax(early[:, 1]), 1])

    if a.p0 is None:
        m = (gauge[:, 0] > 160) & (gauge[:, 0] < 260) & (gauge[:, 1] > 50)
        c = np.polyfit(gauge[m, 0], gauge[m, 1] - a.gauge_bias, 1)
        p0 = float(np.polyval(c, t0))
    else:
        p0 = a.p0
    p1 = a.p1

    # Segments of constant engine set: exponent = sum dv_i / (Isp_i g0)
    t_out = a.rvac_out_s if a.rvac_out_s is not None else math.inf
    before, after = T < t_out, T >= t_out
    dv_before = float(np.trapezoid(AT[before], T[before])) if before.sum() > 1 else 0.0
    dv_after = float(np.trapezoid(AT[after], T[after])) if after.sum() > 1 else 0.0
    isp_full, isp_less = isp_mix(3, 3, a), isp_mix(3, 2, a)
    ln_r = dv_before / (isp_full * G0) + dv_after / (isp_less * G0)
    r = math.exp(ln_r)
    metered = (a.capacity_t - a.header_t) * 1e3
    M = metered * (p0 / 100 - r * p1 / 100) / (r - 1.0)
    D = M - (a.payload_t + a.header_t) * 1e3
    print(f"burn T+{t0:.0f}-{t1:.0f} s, {v0:.0f} -> {v1:.0f} m/s; gravity {np.trapezoid(GV, T):.0f}, along "
          f"{np.trapezoid(AL, T):.0f}, thrust (with steering) {dv_before + dv_after:.0f} m/s; Isp {isp_full:.1f} s"
          + (f" / {isp_less:.1f} s after T+{t_out:.0f} s" if a.rvac_out_s is not None else ""))
    print(f"gauge {p0:.1f} -> {p1:.2f} % of {metered / 1e3:.0f} t metered; mass ratio {r:.3f}; dry + payload + header "
          f"{M / 1e3:.1f} t, at cutoff {(M + metered * p1 / 100) / 1e3:.1f} t; dry {D / 1e3:.1f} t "
          f"(payload {a.payload_t:.1f} t, header {a.header_t:.1f} t)")


if __name__ == "__main__":
    main()
