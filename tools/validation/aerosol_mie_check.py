"""Look-fixes T9b: aerosol phase functions and spectral optics from the AERONET size distributions.

Lorenz-Mie (Bohren & Huffman 1983, BHMIE recursion) over the bimodal lognormal volume distributions of Dubovik et al.
2002 (J. Atmos. Sci. 59, 590, Table 1) for an urban-industrial / continental site (GSFC, Greenbelt) and an oceanic site
(Lanai, Hawaii), at 440 / 550 / 670 / 870 nm. Prints AOD shape (Angstrom 440-870), single-scattering albedo, asymmetry
g and the phase function ratios p(theta) / p(25 deg) next to the Cornette-Shanks function the atmosphere model uses, and
fits the model's phase representation (a two-term Henyey-Greenstein, w HG(g1) + (1 - w) HG(g2), with the Cornette-
Shanks (1 + cos^2) factor dropped) to the Mie phase function.

Usage: python -I tools/validation/aerosol_mie_check.py
"""
import math

import numpy as np

# Dubovik et al. 2002 Table 1 (per site): refractive index n, k; fine / coarse volume median radius (um) and width sigma
# (ln), volume concentrations (um^3 / um^2) as functions of the AOD (tau440 for GSFC, tau1020 for Lanai).
SITES = {
    # GSFC: r_Vf = 0.12 + 0.11 tau440, sigma_f 0.38; r_Vc = 3.03 + 0.49 tau440, sigma_c 0.75; C_Vf 0.15 tau440,
    # C_Vc 0.01 + 0.04 tau440; n 1.41, k 0.003. Evaluated at tau440 0.15 (a clean continental background).
    "continental (GSFC, tau440 0.15)": dict(n=1.41, k=0.003, tau=0.15,
                                              fine=lambda t: (0.12 + 0.11 * t, 0.38, 0.15 * t),
                                              coarse=lambda t: (3.03 + 0.49 * t, 0.75, 0.01 + 0.04 * t)),
    # Lanai: r_Vf 0.16, sigma_f 0.48; r_Vc 2.70, sigma_c 0.68; C_Vf 0.40 tau1020, C_Vc 0.80 tau1020; n 1.36, k 0.0015.
    "maritime (Lanai, tau1020 0.04)": dict(n=1.36, k=0.0015, tau=0.04,
                                             fine=lambda t: (0.16, 0.48, 0.40 * t),
                                             coarse=lambda t: (2.70, 0.68, 0.80 * t)),
}
WAVELENGTHS_UM = (0.44, 0.55, 0.67, 0.87)
ANGLES_DEG = np.concatenate([np.linspace(0.0, 30.0, 301), np.linspace(30.5, 180.0, 300)])


def bhmie(x, m, mu):
    """Efficiencies Q_ext, Q_sca and the amplitude |S1|^2 + |S2|^2 at cos(theta) = mu (Bohren & Huffman, appendix A)."""
    nstop = int(x + 4.0 * x ** (1.0 / 3.0) + 2.0)
    y = m * x
    nmx = int(max(nstop, abs(y)) + 15)
    d = np.zeros(nmx + 1, dtype=complex)
    for n in range(nmx, 0, -1):
        d[n - 1] = n / y - 1.0 / (d[n] + n / y)
    pi0, pi1 = np.zeros_like(mu), np.ones_like(mu)
    psi0, psi1 = math.cos(x), math.sin(x)
    chi0, chi1 = -math.sin(x), math.cos(x)
    xi1 = complex(psi1, -chi1)
    s1 = np.zeros_like(mu, dtype=complex)
    s2 = np.zeros_like(mu, dtype=complex)
    qsca = 0.0
    qext = 0.0
    for n in range(1, nstop + 1):
        fn = (2.0 * n + 1.0) / (n * (n + 1.0))
        psi = (2.0 * n - 1.0) * psi1 / x - psi0
        chi = (2.0 * n - 1.0) * chi1 / x - chi0
        xi = complex(psi, -chi)
        an = ((d[n] / m + n / x) * psi - psi1) / ((d[n] / m + n / x) * xi - xi1)
        bn = ((m * d[n] + n / x) * psi - psi1) / ((m * d[n] + n / x) * xi - xi1)
        qsca += (2.0 * n + 1.0) * (abs(an) ** 2 + abs(bn) ** 2)
        qext += (2.0 * n + 1.0) * (an.real + bn.real)
        pi = pi1
        tau = n * mu * pi - (n + 1.0) * pi0
        s1 += fn * (an * pi + bn * tau)
        s2 += fn * (an * tau + bn * pi)
        psi0, psi1 = psi1, psi
        chi0, chi1 = chi1, chi
        xi1 = complex(psi1, -chi1)
        pi1 = ((2.0 * n + 1.0) * mu * pi - (n + 1.0) * pi0) / n
        pi0 = pi
    return 2.0 / x ** 2 * qext, 2.0 / x ** 2 * qsca, np.abs(s1) ** 2 + np.abs(s2) ** 2


def population(site, wl_um, radii_um):
    """Extinction / scattering per unit column (um^2 / um^2 = AOD) and the phase function P (mean over 4 pi = 1)."""
    m = complex(site["n"], site["k"])
    mu = np.cos(np.radians(ANGLES_DEG))
    lnr = np.log(radii_um)
    dlnr = lnr[1] - lnr[0]
    dv = np.zeros_like(radii_um)
    for mode in (site["fine"], site["coarse"]):
        rv, s, cv = mode(site["tau"])
        dv += cv / (math.sqrt(2.0 * math.pi) * s) * np.exp(-((lnr - math.log(rv)) ** 2) / (2.0 * s * s))
    ext = sca = 0.0
    s11 = np.zeros_like(mu)
    for r, v in zip(radii_um, dv):
        x = 2.0 * math.pi * r / wl_um
        qe, qs, s = bhmie(x, m, mu)
        number = v * dlnr / (4.0 / 3.0 * math.pi * r ** 3)
        area = math.pi * r * r
        ext += number * area * qe
        sca += number * area * qs
        s11 += number * s * (wl_um / (2.0 * math.pi)) ** 2 / 2.0
    # normalise: integral of P over the sphere / 4 pi = 1
    th = np.radians(ANGLES_DEG)
    norm = np.trapezoid(s11 * np.sin(th), th) / 2.0
    p = s11 / norm
    g = np.trapezoid(p * np.cos(th) * np.sin(th), th) / 2.0
    return ext, sca, p, g


def hg(g, mu):
    return (1.0 - g * g) / (1.0 + g * g - 2.0 * g * mu) ** 1.5


def cornette_shanks(g, mu):
    return 1.5 * (1.0 - g * g) / (2.0 + g * g) * (1.0 + mu * mu) / (1.0 + g * g - 2.0 * g * mu) ** 1.5


def fit_tthg(p, mu, th, g_mie):
    """Least squares in log P for w, g1, g2 on a grid: the aureole 0.5-30 deg at full weight (the visible part around the
    Sun and the Moon), the rest of the sphere by sin(theta); the fit's asymmetry within 0.02 of the Mie value."""
    sel = np.radians(ANGLES_DEG) >= np.radians(0.5)
    weight = np.where(ANGLES_DEG <= 30.0, 1.0, 0.2 + np.sin(th))[sel]
    best = None
    for g1 in np.linspace(0.80, 0.995, 40):
        for g2 in np.linspace(0.0, 0.80, 41):
            h1, h2 = hg(g1, mu), hg(g2, mu)
            for w in np.linspace(0.01, 0.6, 60):
                if abs(w * g1 + (1 - w) * g2 - g_mie) > 0.02:
                    continue
                q = w * h1 + (1 - w) * h2
                e = np.sum((np.log(q[sel]) - np.log(p[sel])) ** 2 * weight)
                if best is None or e < best[0]:
                    best = (e, w, g1, g2)
    return best[1:]


def main():
    radii = np.exp(np.linspace(math.log(0.02), math.log(20.0), 220))
    mu = np.cos(np.radians(ANGLES_DEG))
    th = np.radians(ANGLES_DEG)
    idx = {a: int(np.argmin(np.abs(ANGLES_DEG - a))) for a in (0.5, 1, 2, 3, 5, 10, 15, 25, 45, 90, 180)}
    for name, site in SITES.items():
        print(name)
        res = {wl: population(site, wl, radii) for wl in WAVELENGTHS_UM}
        e440, e870 = res[0.44][0], res[0.87][0]
        print(f"  AOD 440 {e440:.4f}  550 {res[0.55][0]:.4f}  870 {e870:.4f}  Angstrom 440-870 "
              f"{-math.log(e440 / e870) / math.log(0.44 / 0.87):.2f}  (440-670 "
              f"{-math.log(e440 / res[0.67][0]) / math.log(0.44 / 0.67):.2f})")
        for wl in WAVELENGTHS_UM:
            ext, sca, p, g = res[wl]
            print(f"  {wl * 1000:.0f} nm: SSA {sca / ext:.3f}  g {g:.3f}")
        ext, sca, p, g = res[0.55]
        p25 = p[idx[25]]
        print("  550 nm P(theta) / P(25):  " + "  ".join(f"{a:g}: {p[i] / p25:6.2f}" for a, i in idx.items()))
        for gg in (0.8, round(g, 2)):
            c = cornette_shanks(gg, mu)
            print(f"  Cornette-Shanks g {gg:.2f}:  " + "  ".join(f"{a:g}: {c[i] / c[idx[25]]:6.2f}" for a, i in idx.items()))
        w, g1, g2 = fit_tthg(p, mu, th, g)
        q = w * hg(g1, mu) + (1 - w) * hg(g2, mu)
        gq = w * g1 + (1 - w) * g2
        print(f"  TTHG fit w {w:.3f} g1 {g1:.3f} g2 {g2:.3f} (g {gq:.3f}):  "
              + "  ".join(f"{a:g}: {q[i] / q[idx[25]]:6.2f}" for a, i in idx.items()))
        print("  TTHG / Mie at the angles: " + "  ".join(f"{a:g}: {q[i] / p[i]:5.2f}" for a, i in idx.items()))
        print("  Mie P absolute:           " + "  ".join(f"{a:g}: {p[i]:7.2f}" for a, i in idx.items()))


if __name__ == "__main__":
    main()
