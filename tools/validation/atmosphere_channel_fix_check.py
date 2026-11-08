"""Look-fixes T9: candidate fixes for the three-channel direct beam (see atmosphere_spectrum_check.py).

Option 1: move the transmittance table's channel wavelengths from 680 / 550 / 440 nm to the equivalent wavelengths of
the sRGB channels under sunlight (the colour's optical depth matches); solar_illuminance_rgb and the luminance factors
follow automatically (both read the same constants).
Option 2: store per channel the broadband transmittance T_c = sum_l w_c(l) T(l) / sum_l w_c(l), w_c = (XYZ->sRGB x
CMF)_c x S_TOA: exact for the direct beam at every air mass and height; segment transmittance by the table ratio
T(p) / T(q) is then approximate (checked here for a near-horizontal path).

Usage: python tools/validation/atmosphere_channel_fix_check.py
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import atmosphere_spectrum_check as a  # noqa: E402

WL = np.arange(360.0, 830.0, 1.0)


def weights():
    w = (a.XYZ_TO_SRGB @ a.cmf(WL).T) * a.bin_value(a.SOLAR_BINS, WL)  # 3 x n
    return w / w.sum(1, keepdims=True)


W = weights()


def opt1_xyz(air, elev, lc):
    kf = a.k_factors_at(lc)
    rgb = kf * air.solar(lc) * air.T_direct(lc, elev)
    return a.SRGB_TO_XYZ @ rgb


def opt2_xyz(air, elev):
    teff = W @ air.T_direct(WL, elev)
    rgb_toa = a.XYZ_TO_SRGB @ a.xyz_of(a.bin_value(a.SOLAR_BINS, WL), WL, 1.0)
    return a.SRGB_TO_XYZ @ (rgb_toa * teff)


def main():
    lc1 = np.array([628.0, 546.0, 444.0])
    print("direct beam CCT (K): full 1 nm | core now 680/550/440 | option 1 628/546/444 | option 2 broadband")
    for kw, label in [({}, "model optics"), ({"aod": 0.3}, "AOD 0.3"), ({"aod": 0.05, "alpha": 0.5}, "AOD 0.05 alpha 0.5")]:
        air = a.Air(**kw)
        print(f" {label}")
        for e in (90, 60, 48, 30, 15, 10, 5, 2):
            full = air.direct_xyz_full(e)
            now = air.direct_xyz_3(e)
            o1 = opt1_xyz(air, e, lc1)
            o2 = opt2_xyz(air, e)
            def m(x):
                return 1e6 / a.cct(x)[0] - 1e6 / a.cct(full)[0]
            print(f"   {e:3d} deg: {a.cct(full)[0]:6.0f} | {a.cct(now)[0]:6.0f} ({m(now):+5.1f} mired)"
                  f" | {a.cct(o1)[0]:6.0f} ({m(o1):+5.1f}) | {a.cct(o2)[0]:6.0f} ({m(o2):+5.1f})"
                  f"   Ev full {full[1]:7.0f} now {now[1]:7.0f} o1 {o1[1]:7.0f} o2 {o2[1]:7.0f}")

    # global horizontal (direct x sin e + the SPCTRAL2 sky at 15 wavelengths as the core's sky)
    print("\nglobal horizontal CCT (K): full | now | option 1 | option 2   (sky: SPCTRAL2 at the 15 precompute wavelengths)")
    air = a.Air()
    for e in (90, 60, 48, 30, 15, 5):
        cz = math.sin(math.radians(e))
        sky_full = air.sky_xyz(e)
        sky15 = air.sky_xyz(e, a.L15, a.DL15)
        g = [air.direct_xyz_full(e) * cz + sky_full, air.direct_xyz_3(e) * cz + sky15,
             opt1_xyz(air, e, lc1) * cz + sky15, opt2_xyz(air, e) * cz + sky15]
        print(f"   {e:3d} deg: " + " | ".join(f"{a.cct(x)[0]:6.0f}" for x in g))

    # option 2 segment error: object at height h on a ray of elevation e from the ground (plane parallel layers)
    print("\nsegment transmittance by the table ratio T(ground)/T(h), grey object lit by the local Sun (Sun 30 deg):")
    for e, h in ((2.0, 500.0), (5.0, 1000.0), (0.5, 100.0)):
        m = 1 / math.sin(math.radians(e))
        frac_r = 1 - math.exp(-h / 8000.0)
        frac_a = 1 - math.exp(-h / 1200.0)
        tau_seg = m * (frac_r * a.tau_rayleigh(WL) + frac_a * a.tau_aerosol(WL))
        t_seg = np.exp(-tau_seg)
        light = a.bin_value(a.SOLAR_BINS, WL) * air.T_direct(WL, 30)
        exact_rgb = a.XYZ_TO_SRGB @ a.xyz_of(light * t_seg, WL, 1.0)
        lit_rgb = a.XYZ_TO_SRGB @ a.xyz_of(light, WL, 1.0)
        # table: T to the top along the ray from the ground and from h (plane parallel: the full column minus the part)
        tau_g = m * (a.tau_rayleigh(WL) + a.tau_aerosol(WL) + a.tau_ozone(WL))
        tau_h = tau_g - tau_seg
        t2 = (W @ np.exp(-tau_g)) / (W @ np.exp(-tau_h))
        t_now = np.exp(-(m * (frac_r * a.tau_rayleigh(lc_now := np.array([680.0, 550.0, 440.0])) + frac_a * a.tau_aerosol(lc_now))))
        t_o1 = np.exp(-(m * (frac_r * a.tau_rayleigh(lc1) + frac_a * a.tau_aerosol(lc1))))
        print(f"   ray {e} deg, h {h:.0f} m: exact T_rgb {(exact_rgb / lit_rgb).round(4)} | now {t_now.round(4)} | opt1 {t_o1.round(4)} | opt2 {t2.round(4)}")


if __name__ == "__main__":
    main()
