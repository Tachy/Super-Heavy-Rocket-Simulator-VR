"""Ship dry mass from the delta-v of its burn (staging to engine cutoff), via the rocket equation.

  dv = Isp g0 ln(m0 / m1),  m0 = D + P + C p0,  m1 = D + P + C p1
dv = speed gain + gravity loss (integral of g sin(gamma) dt, gamma from the altitude profile) + drag (ignored:
above ~60 km). D = dry mass, P = payload, C = propellant capacity, p0/p1 = gauge fraction at ignition/cutoff
(measured with the round-dial layout, else assumed). Solves for D.

  python tools/telemetry/ship_dv.py <table.csv> --capacity-t 1600 --isp 365 --payload-t 50 [--p0 0.98 --p1 0.04]
"""
import argparse
import csv
import math

from drag_estimate import EARTH_RADIUS_M, G0, slope


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("table")
    ap.add_argument("--capacity-t", type=float, required=True)
    ap.add_argument("--isp", type=float, required=True, help="effective vacuum Isp of the ship's engines [s]")
    ap.add_argument("--payload-t", type=float, default=0.0)
    ap.add_argument("--p0", type=float, default=None, help="gauge at ignition (default: measured, else 0.98)")
    ap.add_argument("--p1", type=float, default=None, help="gauge at cutoff (default: measured, else 0.05)")
    a = ap.parse_args()
    rows = list(csv.DictReader(line for line in open(a.table) if not line.startswith("#")))
    sp = [(float(r["t_s"]), float(r["ship_speed_kmh"]) / 3.6) for r in rows if r["ship_speed_kmh"]]
    al = [(float(r["t_s"]), float(r["ship_altitude_km"]) * 1e3) for r in rows if r["ship_altitude_km"]]
    pr = [(float(r["t_s"]), float(r["ship_propellant_pct"]) / 100) for r in rows if r.get("ship_propellant_pct")]
    # Staging: where the ship speed separates from the booster's
    # cutoff: maximum ship speed before T+700 s.
    # (round dials: the booster's speed leaves the shared dial; strip: both shown, they start to differ).
    bs = {round(float(r["t_s"]) * 4): r["booster_speed_kmh"] for r in rows}

    def separate(t, v):
        b = bs.get(round(t * 4))
        return not b or abs(float(b) / 3.6 - v) > 10.0

    t_stage = next(t for t, v in sp if t > 100.0 and separate(t, v))
    t_cut, v_cut = max(((t, v) for t, v in sp if t < 700.0), key=lambda x: x[1])
    v_stage = next(v for t, v in sp if t >= t_stage)
    st, sv = zip(*sp)
    at, ah = zip(*al)
    loss, t = 0.0, t_stage
    while t < t_cut:
        dh, h = slope(at, ah, t, 30.0)
        _, v = slope(st, sv, t, 2.0)
        if dh is not None and v:
            g = G0 * (EARTH_RADIUS_M / (EARTH_RADIUS_M + h)) ** 2
            loss += g * max(-1.0, min(1.0, dh / v)) * 1.0
        t += 1.0

    def gauge(t, default):
        near = [p for tt, p in pr if abs(tt - t) <= 5.0]
        return sorted(near)[len(near) // 2] if near else default

    p0 = a.p0 if a.p0 is not None else gauge(t_stage + 5.0, 0.98)
    p1 = a.p1 if a.p1 is not None else gauge(t_cut - 3.0, 0.05)
    dv = v_cut - v_stage + loss
    r = math.exp(dv / (a.isp * G0))  # m0 / m1
    C = a.capacity_t * 1e3
    # (D + P + C p0) = r (D + P + C p1)  ->  D = (C p0 - r C p1) / (r - 1) - P
    D = (C * p0 - r * C * p1) / (r - 1.0) - a.payload_t * 1e3
    print(f"staging T+{t_stage:.0f} s {v_stage:.0f} m/s, cutoff T+{t_cut:.0f} s {v_cut:.0f} m/s, gravity loss {loss:.0f} m/s,"
          f" dv {dv:.0f} m/s; gauge {100 * p0:.1f} -> {100 * p1:.1f} %; mass ratio {r:.2f}; dry mass {D / 1e3:.0f} t")


if __name__ == "__main__":
    main()
