"""Calibrates the boil-off wall resistances on Centaur flight reconstructions (refuelling r2).

Source: De Kruif & Kutter, AIAA 2007-5845, Table 2 - LO2 sidewall flux in sunlit coasts, broadside to the Sun, rolling.
Mirrors core/tests/test_boil_off.cpp (centaur_wall_flux_w_m2): Earth of mean radius, 36 strips, 60 time samples.
Prints the white-foam and 3-layer MLI resistances [D], the predictions for AC-167 / AV-003, and the depot value
(30 effective layers, leak ~ 1/(n+1)).
"""
import math

SIGMA = 5.670374419e-8
S = 1361.0
ALBEDO = 0.30
IR = 239.0
RE = 6371000.0
MU = 3.986004418e14
TB = 90.19
BTU = 3.15459  # W/m^2 per Btu/hr-ft^2
STRIPS = 36
SAMPLES = 60


def plate_view_factor(angle, h):
    full = math.acos(1.0 / h)
    if angle <= full:
        return math.cos(angle) / h**2
    if angle >= math.pi - full:
        return 0.0
    s = math.sqrt(h * h - 1.0)
    rim = math.sqrt(max(0.0, 1.0 - h * h * math.cos(angle) ** 2))
    return (0.5 - math.asin(min(1.0, s / (h * math.sin(angle)))) / math.pi
            + (math.cos(angle) * math.acos(max(-1.0, min(1.0, -s / math.tan(angle)))) - s * rim) / (math.pi * h * h))


def strip_leak(absorbed, eps, r):
    lo, hi = 1.0, 3000.0
    for _ in range(200):
        t = 0.5 * (lo + hi)
        if eps * SIGMA * t**4 + (t - TB) / r - absorbed > 0.0:
            hi = t
        else:
            lo = t
    return (t - TB) / r


def orbit_at(rp, ra, t):
    a = 0.5 * (rp + ra)
    e = (ra - rp) / (ra + rp)
    n = math.sqrt(MU / a**3)
    ecc = n * t
    for _ in range(50):
        ecc -= (ecc - e * math.sin(ecc) - n * t) / (1.0 - e * math.cos(ecc))
    nu = 2.0 * math.atan2(math.sqrt(1 + e) * math.sin(ecc / 2), math.sqrt(1 - e) * math.cos(ecc / 2))
    return a * (1.0 - e * math.cos(ecc)), nu


def wall_flux(alpha, eps, r, perigee_km, apogee_km, coast_min, earth=True):
    rp, ra = RE + perigee_km * 1e3, RE + apogee_km * 1e3
    _, nu_mid = orbit_at(rp, ra, 0.5 * coast_min * 60.0)
    total = 0.0
    for k in range(SAMPLES):
        radius, nu = orbit_at(rp, ra, (k + 0.5) / SAMPLES * coast_min * 60.0)
        h = radius / RE if earth else 1.0e6
        phase = max(0.0, math.cos(nu - nu_mid)) if earth else 0.0
        acc = 0.0
        for i in range(STRIPS):
            phi = (i + 0.5) / STRIPS * 2.0 * math.pi  # strip normal in the orbit plane
            to_nadir = math.atan2(math.sin(phi - nu - math.pi), math.cos(phi - nu - math.pi))
            view = plate_view_factor(abs(to_nadir), h)
            absorbed = alpha * (S * max(0.0, math.cos(phi - nu_mid)) + ALBEDO * S * view * phase) + eps * IR * view
            acc += strip_leak(absorbed, eps, r)
        total += acc / STRIPS
    return total / SAMPLES


def solve_resistance(target, flux):
    lo, hi = 1e-3, 1e4
    for _ in range(200):
        mid = math.sqrt(lo * hi)
        if flux(mid) > target:
            lo = mid
        else:
            hi = mid
    return mid


if __name__ == "__main__":
    paint = (0.25, 0.88)
    foam = solve_resistance(23 * BTU, lambda r: wall_flux(*paint, r, 167, 1132, 58))
    print(f"white foam resistance (AC-206) = {foam:.3f} m^2K/W")
    for name, hp, ha, minutes, measured in [("AC-167", 185, 267, 59, 25), ("AV-003", 167, 4165, 78, 25)]:
        q = wall_flux(*paint, foam, hp, ha, minutes)
        print(f"{name}: model {q:.1f} W/m^2, flight {measured * BTU:.1f} ({q / (measured * BTU) - 1:+.1%})")
    mli3 = solve_resistance(3 * BTU, lambda r: wall_flux(0.30, 0.80, r, 0, 0, 1, earth=False))
    print(f"3-layer MLI resistance (TC average) = {mli3:.2f} m^2K/W")
    print(f"depot, 30 effective layers = {mli3 * 31 / 4:.1f} m^2K/W")
