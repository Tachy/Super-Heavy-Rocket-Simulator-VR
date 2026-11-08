"""Atmosphere colour check (look-fixes T9): how much of the model's warm daylight comes from the spectral representation,
the aerosol, the ozone and the solar spectrum.

Offline, CPU only. Rebuilds the core's optics (core/src/environment/atmosphere_optics.cpp, the tables of
core/include/starsim/environment/atmosphere_spectra.hpp, config/atmosphere/earth_optics.toml) in numpy and computes, for a
clear sky at sea level:
  - the direct beam at 1 nm (reference), at the core's three channel wavelengths 680 / 550 / 440 nm (what the core's
    direct Sun is: TOA RGB x T(lambda_c)), and at the 15 precompute wavelengths;
  - a spectral diffuse sky by SPCTRAL2 (Bird & Riordan 1986, J. Clim. Appl. Meteor. 25, 87; NREL/TP-215-2436), used
    only for differences (the core's sky is Bruneton's multiple scattering at 15 wavelengths);
  - CCT (Robertson-type nearest point of the Planckian locus in CIE 1960 uv) and McCamy, Duv, linear Rec.709 B/R.
Optional reference: ASTM G173-03 (pvlib's ASTMG173.csv) direct and global tilt spectra.

Usage: python tools/validation/atmosphere_spectrum_check.py [--g173 path/to/ASTMG173.csv]
"""
import argparse
import math
import re
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SPECTRA_HPP = ROOT / "core/include/starsim/environment/atmosphere_spectra.hpp"

# Constants as in core/include/starsim/math/constants.hpp.
N_STD = 2.54743e25          # standard air number density, 15 C, 101325 Pa (Peck & Reeder's refractivity reference)
DU = 2.6867e20              # molecules m^-2 per Dobson unit
KM = 683.0                  # lm/W
XYZ_TO_SRGB = np.array([[3.2406, -1.5372, -0.4986], [-0.9689, 1.8758, 0.0415], [0.0557, -0.2040, 1.0570]])
SRGB_TO_XYZ = np.linalg.inv(XYZ_TO_SRGB)
# Sea-level air column: p0 / (m_air g) with the column-mean gravity (US76: g falls ~0.3 % over the mass-weighted
# column) [D]; checked below against Bucholtz 1995 (tau_R(550) = 0.0973 for the US standard atmosphere).
AIR_COLUMN_PER_M2 = 101325.0 / (28.9644e-3 / 6.02214076e23 * 9.786)

DEPOL = [(0.200, 4.545), (0.205, 4.384), (0.210, 4.221), (0.215, 4.113), (0.220, 4.004), (0.225, 3.895),
         (0.230, 3.785), (0.240, 3.675), (0.250, 3.565), (0.260, 3.455), (0.270, 3.400), (0.280, 3.289),
         (0.290, 3.233), (0.300, 3.178), (0.310, 3.178), (0.320, 3.122), (0.330, 3.066), (0.340, 3.066),
         (0.350, 3.010), (0.360, 3.010), (0.370, 3.010), (0.380, 2.955), (0.390, 2.955), (0.400, 2.955),
         (0.450, 2.899), (0.500, 2.842), (0.550, 2.842), (0.600, 2.786), (0.650, 2.786), (0.700, 2.786),
         (0.750, 2.786), (0.800, 2.730), (0.850, 2.730), (0.900, 2.730), (0.950, 2.730), (1.000, 2.730)]


def parse_array(text, name):
    m = re.search(name + r"(?:\[[^\]]*\])+\s*=\s*\{(.*?)\};", text, re.S)
    return np.array([float(v) for v in re.findall(r"[-+]?\d+\.?\d*(?:e[-+]?\d+)?", m.group(1))])


SPEC = SPECTRA_HPP.read_text(encoding="utf-8")
SOLAR_BINS = parse_array(SPEC, "solar_irradiance_w_m2_nm")
OZONE_BINS = parse_array(SPEC, "ozone_cross_section_m2")
CMF = parse_array(SPEC, "cie_1931_2deg_cmf").reshape(-1, 4)


def bin_value(table, wl_nm):
    pos = (np.asarray(wl_nm, float) - 360.0) / 10.0 - 0.5
    return np.interp(pos, np.arange(len(table)), table)


def cmf(wl_nm):
    wl = np.asarray(wl_nm, float)
    return np.stack([np.interp(wl, CMF[:, 0], CMF[:, k], left=0, right=0) for k in (1, 2, 3)], -1)


def rayleigh_sigma(wl_nm):
    l = np.asarray(wl_nm, float) * 1e-9
    s2 = (1e-6 / l) ** 2
    nm1 = 1e-8 * (8060.51 + 2480990.0 / (132.274 - s2) + 17455.7 / (39.32957 - s2))
    n2m1 = 2 * nm1 + nm1 * nm1
    rho = np.interp(l * 1e6, [d[0] for d in DEPOL], [d[1] for d in DEPOL]) * 0.01
    king = (6 + 3 * rho) / (6 - 7 * rho)
    return 24 * math.pi ** 3 * n2m1 ** 2 / (l ** 4 * N_STD ** 2 * (n2m1 + 3) ** 2) * king


def tau_rayleigh(wl):
    return rayleigh_sigma(wl) * AIR_COLUMN_PER_M2


def tau_ozone(wl, du=300.0):
    return bin_value(OZONE_BINS, wl) * du * DU


def tau_aerosol(wl, aod550=0.1, alpha=1.3):
    return aod550 * (np.asarray(wl, float) / 550.0) ** (-alpha)


def air_mass(elev_deg, scale_height_m=None, layer_km=None):
    """Relative optical air mass of a spherical shell atmosphere (R = 6371 km): exponential profile or, for ozone, a
    thin layer at layer_km."""
    z = math.radians(90.0 - elev_deg)
    R = 6371e3
    if layer_km is not None:
        h = layer_km * 1e3
        return (R + h) / math.sqrt((R + h) ** 2 - (R * math.sin(z)) ** 2)
    H = scale_height_m
    s = np.linspace(0, 400e3 if elev_deg < 10 else 120e3 / max(math.cos(z), 0.05), 200001)
    r = np.sqrt(R * R + s * s + 2 * R * s * math.cos(z))
    return np.trapezoid(np.exp(-(r - R) / H), s) / H


def xyz_of(spec, wl, dl):
    return KM * (cmf(wl) * spec[:, None]).sum(0) * dl


# --- colorimetry
WL1 = np.arange(360.0, 830.0 + 0.5, 1.0)


def planck(T, wl):
    l = wl * 1e-9
    return 1 / (l ** 5 * (np.exp(1.438777e-2 / (l * T)) - 1))


def uv(xyz):
    X, Y, Z = xyz
    d = X + 15 * Y + 3 * Z
    return np.array([4 * X / d, 6 * Y / d])


_LOCUS_T = np.concatenate([np.arange(1500, 4000, 10), np.arange(4000, 30001, 5)]).astype(float)
_LOCUS = np.array([uv(xyz_of(planck(T, WL1), WL1, 1.0)) for T in _LOCUS_T])


def cct(xyz):
    p = uv(xyz)
    d = np.hypot(*(_LOCUS - p).T)
    i = int(np.argmin(d))
    j = max(1, min(i, len(_LOCUS_T) - 2))
    # parabolic refinement
    a, b, c = d[j - 1], d[j], d[j + 1]
    den = a - 2 * b + c
    off = 0.5 * (a - c) / den if den > 0 else 0.0
    T = _LOCUS_T[j] + off * (_LOCUS_T[j + 1] - _LOCUS_T[j - 1]) / 2
    duv = d[i] * (1 if p[1] > _LOCUS[i][1] else -1)
    return T, duv


def mccamy(xyz):
    X, Y, Z = xyz
    x, y = X / (X + Y + Z), Y / (X + Y + Z)
    n = (x - 0.3320) / (0.1858 - y)
    return 449 * n ** 3 + 3525 * n ** 2 + 6823.3 * n + 5520.33


def br(xyz):
    rgb = XYZ_TO_SRGB @ xyz
    return rgb[2] / rgb[0]


def describe(xyz):
    T, duv = cct(xyz)
    return f"{T:6.0f} K (McCamy {mccamy(xyz):5.0f}, Duv {duv:+.4f}), B/R {br(xyz):.3f}, Ev {xyz[1]:8.0f} lx"


# --- the model's three channels and the 15 precompute wavelengths
LC = np.array([680.0, 550.0, 440.0])
L15 = 360.0 + (np.arange(15) + 0.5) * (470.0 / 15)
DL15 = 470.0 / 15


def k_factors(solar=None):
    """sun_radiance_to_luminance (atmosphere_tables.cpp): linear sRGB of the TOA spectrum per unit S(lambda_c)."""
    wl = np.arange(360.0, 830.0, 1.0)
    S = bin_value(SOLAR_BINS, wl) if solar is None else solar(wl)
    rgb = XYZ_TO_SRGB @ (KM * (cmf(wl) * S[:, None]).sum(0) * 1.0)
    Sc = bin_value(SOLAR_BINS, LC) if solar is None else solar(LC)
    return rgb / Sc


def k_factors_at(lc):
    """The same factors for other channel wavelengths lc (nm)."""
    wl = np.arange(360.0, 830.0, 1.0)
    rgb = XYZ_TO_SRGB @ (KM * (cmf(wl) * bin_value(SOLAR_BINS, wl)[:, None]).sum(0))
    return rgb / bin_value(SOLAR_BINS, np.asarray(lc, float))


class Air:
    def __init__(self, aod=0.1, alpha=1.3, ssa=0.9, g=0.8, du=300.0, albedo=0.1, solar=None, gas=None):
        self.aod, self.alpha, self.ssa, self.g, self.du, self.albedo = aod, alpha, ssa, g, du, albedo
        self.solar = solar if solar is not None else (lambda wl: bin_value(SOLAR_BINS, wl))
        self.gas = gas  # optional callable wl, elev -> extra transmittance (water vapour, O2)

    def T_direct(self, wl, elev):
        mR = air_mass(elev, 8000.0)
        mA = air_mass(elev, 1200.0)
        mO = air_mass(elev, layer_km=22.0)
        t = np.exp(-mR * tau_rayleigh(wl) - mA * tau_aerosol(wl, self.aod, self.alpha) - mO * tau_ozone(wl, self.du))
        if self.gas is not None:
            t = t * self.gas(wl, elev)
        return t

    def direct_normal(self, wl, elev):
        return self.solar(wl) * self.T_direct(wl, elev)

    def diffuse_horizontal(self, wl, elev):
        """SPCTRAL2 diffuse on a horizontal plate (Rayleigh + aerosol + ground-sky reflection)."""
        wl = np.asarray(wl, float)
        um = wl / 1000.0
        cz = math.sin(math.radians(elev))
        mR, mA, mO = air_mass(elev, 8000.0), air_mass(elev, 1200.0), air_mass(elev, layer_km=22.0)
        tR, tA, tO = tau_rayleigh(wl), tau_aerosol(wl, self.aod, self.alpha), tau_ozone(wl, self.du)
        Tr, To = np.exp(-mR * tR), np.exp(-mO * tO)
        Tas = np.exp(-self.ssa * tA * mA)
        Taa = np.exp(-(1 - self.ssa) * tA * mA)
        Tg = self.gas(wl, elev) if self.gas is not None else 1.0
        alg = math.log(1 - self.g)
        afs = alg * (1.459 + alg * (0.1595 + alg * 0.4129))
        bfs = alg * (0.0783 + alg * (-0.3824 - alg * 0.5874))
        Fs = 1 - 0.5 * math.exp((afs + bfs * cz) * cz)
        Fsp = 1 - 0.5 * math.exp((afs + bfs / 1.8) / 1.8)
        H0 = self.solar(wl)
        common = H0 * cz * To * Tg * Taa
        Ir = common * (1 - Tr ** 0.95) * 0.5
        Ia = common * Tr ** 1.5 * (1 - Tas) * Fs
        m18 = 1.8
        Trp = np.exp(-m18 * tR)
        Tasp = np.exp(-self.ssa * tA * m18)
        Taap = np.exp(-(1 - self.ssa) * tA * m18)
        Top = np.exp(-m18 * tO)
        rho_s = Top * Taap * (0.5 * (1 - Trp) + (1 - Fsp) * Trp * (1 - Tasp))
        Id = self.direct_normal(wl, elev)
        Ig = (Id * cz + Ir + Ia) * rho_s * self.albedo / (1 - rho_s * self.albedo)
        Cs = np.where(um <= 0.45, (um + 0.55) ** 1.8, 1.0)
        return (Ir + Ia + Ig) * Cs

    # colour of the direct beam in three representations
    def direct_xyz_full(self, elev):
        return xyz_of(self.direct_normal(WL1, elev), WL1, 1.0)

    def direct_xyz_3(self, elev, kf=None):
        kf = k_factors() if kf is None else kf
        rgb = kf * self.solar(LC) * self.T_direct(LC, elev)
        return SRGB_TO_XYZ @ rgb

    def direct_xyz_15(self, elev):
        return xyz_of(self.direct_normal(L15, elev), L15, DL15)

    def sky_xyz(self, elev, wl=WL1, dl=1.0):
        return xyz_of(self.diffuse_horizontal(wl, elev), wl, dl)


def g173_spectra(path):
    d = np.genfromtxt(path, delimiter=",", skip_header=2)
    return d[:, 0], d[:, 1], d[:, 2], d[:, 3]


def g173_xyz(wl, spec):
    sel = (wl >= 360) & (wl <= 830)
    w, s = wl[sel], spec[sel]
    return KM * np.trapezoid(cmf(w) * s[:, None], w, axis=0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--g173", type=Path)
    args = ap.parse_args()
    print(f"tau_R 440/550/680: {tau_rayleigh(440):.4f} {tau_rayleigh(550):.4f} {tau_rayleigh(680):.4f}"
          f"  (Bucholtz 1995 US std: 0.2446 / 0.0973 / 0.0423)")
    print(f"tau_O3 440/550/600/680 at 300 DU: {tau_ozone(440):.4f} {tau_ozone(550):.4f} {tau_ozone(600):.4f} {tau_ozone(680):.4f}")
    toa = xyz_of(bin_value(SOLAR_BINS, WL1), WL1, 1.0)
    print("TOA (G173 ETR, 10 nm bins):", describe(toa))
    base = Air()
    elevs = [90, 60, 48, 30, 15, 5]

    print("\n(a) spectral representation, model optics (AOD 0.1, alpha 1.3, SSA 0.9, 300 DU)")
    print(" elev | direct full 1 nm | direct 3-channel (core) | direct 15 wl | sky full (SPCTRAL2) | sky 15 wl | global full | global core-like (3ch sun + 15 wl sky)")
    rows = {}
    for e in elevs:
        cz = math.sin(math.radians(e))
        df, d3, d15 = base.direct_xyz_full(e), base.direct_xyz_3(e), base.direct_xyz_15(e)
        sf, s15 = base.sky_xyz(e), base.sky_xyz(e, L15, DL15)
        gf, gc = df * cz + sf, d3 * cz + s15
        rows[e] = (df, d3, d15, sf, s15, gf, gc)
        print(f" {e:3d}  | {cct(df)[0]:6.0f} | {cct(d3)[0]:6.0f} (B/R {br(d3):.3f} vs {br(df):.3f}) | {cct(d15)[0]:6.0f}"
              f" | {cct(sf)[0]:6.0f} | {cct(s15)[0]:6.0f} | {cct(gf)[0]:6.0f} | {cct(gc)[0]:6.0f}"
              f"   sky share {sf[1] / gf[1]:.3f}")
    e = 48
    print(" detail Sun 48 deg:")
    for name, x in zip(["direct full", "direct 3ch", "direct 15", "sky full", "sky 15", "global full", "global core-like"], rows[e]):
        print(f"   {name:17s} {describe(x)}")

    # spectrally exact channel coefficients for the direct beam: effective transmittance per channel
    print("\n(a') effective channel transmittance: T_eff,c = RGB_c(full beam) / RGB_c(TOA) vs T(lambda_c)")
    for e in elevs:
        rgb_full = XYZ_TO_SRGB @ base.direct_xyz_full(e)
        rgb_toa = XYZ_TO_SRGB @ toa
        teff = rgb_full / rgb_toa
        tc = base.T_direct(LC, e)
        print(f"   {e:3d} deg: T_eff {teff.round(4)}  T(680/550/440) {tc.round(4)}  ratio {(teff / tc).round(4)}")
    # equivalent wavelengths: lambda where T(lambda) at AM1 reproduces the channel's effective optical depth
    print("   equivalent channel wavelengths (optical depth of the effective transmittance, Sun 90 / 48 / 15 deg):")
    for e in (90, 48, 15):
        rgb_full = XYZ_TO_SRGB @ base.direct_xyz_full(e)
        teff = rgb_full / (XYZ_TO_SRGB @ toa)
        wl = np.arange(380.0, 760.0, 0.1)
        tw = base.T_direct(wl, e)
        eq = []
        for c in range(3):
            # T is monotone enough per channel band; pick the closest in the channel's region
            band = {0: (580, 760), 1: (480, 620), 2: (380, 500)}[c]
            sel = (wl >= band[0]) & (wl <= band[1])
            eq.append(wl[sel][np.argmin(abs(tw[sel] - teff[c]))])
        print(f"     {e:3d} deg: {np.round(eq, 1)} nm")

    print("\n(b) aerosol sensitivity at Sun 48 deg (direct full / sky / global full; core-like direct 3ch)")
    for label, kw in [("model 0.10 a1.3 w0.90", {}),
                      ("AOD 0.05", {"aod": 0.05}), ("AOD 0.15", {"aod": 0.15}), ("AOD 0.20", {"aod": 0.2}),
                      ("AOD 0.30", {"aod": 0.3}),
                      ("alpha 0.5 (maritime)", {"alpha": 0.5}), ("alpha 1.0", {"alpha": 1.0}), ("alpha 1.6", {"alpha": 1.6}),
                      ("SSA 0.95", {"ssa": 0.95}), ("SSA 0.97", {"ssa": 0.97}),
                      ("g 0.7", {"g": 0.7}),
                      ("AERONET-like coastal 0.12 a1.0 w0.95", {"aod": 0.12, "alpha": 1.0, "ssa": 0.95}),
                      ("AERONET-like rural 0.08 a1.3 w0.95", {"aod": 0.08, "alpha": 1.3, "ssa": 0.95}),
                      ("no aerosol", {"aod": 0.0})]:
        a = Air(**kw)
        cz = math.sin(math.radians(48))
        df, sf = a.direct_xyz_full(48), a.sky_xyz(48)
        print(f"   {label:38s} direct {cct(df)[0]:6.0f} (3ch {cct(a.direct_xyz_3(48))[0]:6.0f}) sky {cct(sf)[0]:6.0f}"
              f" global {cct(df * cz + sf)[0]:6.0f}  sky share {sf[1] / (df[1] * cz + sf[1]):.3f}")

    print("\n(c) ozone at Sun 48 deg")
    for du in (0.0, 250.0, 300.0, 350.0):
        a = Air(du=du)
        cz = math.sin(math.radians(48))
        df, sf = a.direct_xyz_full(48), a.sky_xyz(48)
        print(f"   {du:5.0f} DU: direct {cct(df)[0]:6.0f} sky {cct(sf)[0]:6.0f} global {cct(df * cz + sf)[0]:6.0f}")

    print("\n(d) solar spectrum: TOA CCT and its leverage")
    for tilt in (-0.02, 0.0, 0.02):
        sol = (lambda t: (lambda wl: bin_value(SOLAR_BINS, wl) * (1 + t * (np.asarray(wl) - 550) / 100)))(tilt)
        a = Air(solar=sol)
        cz = math.sin(math.radians(48))
        t = xyz_of(sol(WL1), WL1, 1.0)
        df, sf = a.direct_xyz_full(48), a.sky_xyz(48)
        print(f"   tilt {tilt * 100:+.0f} %/100 nm: TOA {cct(t)[0]:6.0f}, direct {cct(df)[0]:6.0f}, global {cct(df * cz + sf)[0]:6.0f}")

    if args.g173:
        wl, etr, glob, direct = g173_spectra(args.g173)
        print("\n(e) ASTM G173-03 reference (SMARTS 2.9.2: AM 1.5, Sun 41.8 deg, AOD500 0.084 rural, 1.42 cm H2O, 0.34 atm-cm O3)")
        print("   ETR 0.5-1 nm:   ", describe(g173_xyz(wl, etr)))
        print("   direct+circums.:", describe(g173_xyz(wl, direct)))
        print("   global 37 tilt: ", describe(g173_xyz(wl, glob)))
        # water vapour + O2 + mixed gas: ratio of G173 direct to a SMARTS-like run without them is not available; use
        # our model at the G173 conditions and compare the shapes.
        e = 41.81
        g = Air(aod=0.084 * (500 / 550) ** 1.14, alpha=1.14, du=343.8, ssa=0.93)
        print("   model optics at G173 conditions, direct full:", describe(g.direct_xyz_full(e)))
        print("   model optics at G173 conditions, direct 3ch :", describe(g.direct_xyz_3(e)))
        # gas absorption read from G173: direct / (ETR x our continuum T), smoothed, as a multiplicative band term
        sel = (wl >= 360) & (wl <= 830)
        cont = bin_value(SOLAR_BINS, wl[sel]) * g.T_direct(wl[sel], e)
        ratio = direct[sel] / (etr[sel] * g.T_direct(wl[sel], e))
        print("   G173 direct / (ETR x model continuum T): mean 400-700 nm"
              f" {ratio[(wl[sel] >= 400) & (wl[sel] <= 700)].mean():.3f}, 680-740 {ratio[(wl[sel] >= 680) & (wl[sel] <= 740)].mean():.3f},"
              f" 750-770 {ratio[(wl[sel] >= 750) & (wl[sel] <= 770)].mean():.3f}")
        gas_wl, gas_ratio = wl[sel], ratio

        def gas(w, elev, _w=gas_wl, _r=gas_ratio):
            # band absorption only (ratio clipped to <= 1, continuum taken as ours); air-mass scaling by Beer-Lambert
            r = np.clip(np.interp(w, _w, _r), 1e-6, 1.0)
            # keep only narrow features: divide by a 60 nm running mean of the clipped ratio's upper envelope
            return r ** (air_mass(elev, 2000.0) / air_mass(41.81, 2000.0))

        # crude: apply only beyond 680 nm, where water vapour and O2 bands live; shorter wavelengths take the
        # continuum (Chappuis is ozone, already modelled)
        def gas_red(w, elev):
            w = np.asarray(w, float)
            return np.where(w >= 680.0, gas(w, elev), 1.0)

        print("   with the G173 water-vapour / O2 bands (>= 680 nm) added to the model optics:")
        for elev in (90, 48, 15):
            a0, a1 = Air(), Air(gas=gas_red)
            cz = math.sin(math.radians(elev))
            d0, d1 = a0.direct_xyz_full(elev), a1.direct_xyz_full(elev)
            s0, s1 = a0.sky_xyz(elev), a1.sky_xyz(elev)
            print(f"     {elev:3d} deg: direct {cct(d0)[0]:6.0f} -> {cct(d1)[0]:6.0f}, global {cct(d0 * cz + s0)[0]:6.0f}"
                  f" -> {cct(d1 * cz + s1)[0]:6.0f}")


if __name__ == "__main__":
    main()
