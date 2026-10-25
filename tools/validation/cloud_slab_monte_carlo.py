"""Monte Carlo reflectance and reflected radiance of a conservative cloud slab with the core's droplet phase function.

Independent check of the delta-Eddington two-stream reference and of the shader's source terms in
core/include/starsim/environment/cloud_optics.hpp: photons enter a plane-parallel slab (optical depth tau, black
surface below) with the Sun at cos zenith mu0 (or isotropic incidence), scatter without absorption with the Jendersie &
d'Eon 2023 phase function for 20 um droplets (Henyey-Greenstein + Draine mixture, sampled exactly: HG by inversion,
Draine by rejection from HG), and the escaping fractions are counted. Offline tool only, not part of the build; fixed
seed.

    python tools/validation/cloud_slab_monte_carlo.py [photons]            # R and T (hemispheric)
    python tools/validation/cloud_slab_monte_carlo.py --radiance [photons] > tests/validation/weather/cloud_slab_radiance_mc.csv

--radiance writes the reflected radiance per unit Sun irradiance (normal to the beam) [1/sr], averaged over solid-angle
bins around view zenith angles 0, 30, 60 deg and relative azimuths 0, 90, 180 deg, for tau 1, 10, 30 and Sun zenith 0,
60 deg. Bin: view zenith within +-5 deg (0..8 deg for the zenith view, all azimuths), relative azimuth within +-15 deg
(both sides of the Sun's plane). Relative azimuth = angle between the horizontal propagation directions of the sunlight
and the reflected light: 0 = forward side (camera opposite the Sun), 180 = backscatter side (Sun behind the camera).
A bin's radiance = mu0 (photons in the bin / photons in) / integral of cos(view zenith) over the bin's solid angle; its
1-sigma statistical error is the relative error 1 / sqrt(photons in the bin).
"""
import sys

import numpy as np

D_UM = 20.0
G_HG = np.exp(-0.0990567 / (D_UM - 1.67154))
G_D = np.exp(-2.20679 / (D_UM + 3.91029) - 0.428934)
ALPHA = np.exp(3.62489 - 8.29288 / (D_UM + 5.52825))
W_D = np.exp(-0.599085 / (D_UM - 0.641583) - 0.665888)
RNG = np.random.default_rng(1)

VIEW_ZENITHS_DEG = (0.0, 30.0, 60.0)
REL_AZIMUTHS_DEG = (0.0, 90.0, 180.0)
ZENITH_HALF_WIDTH_DEG = 5.0
ZENITH_CAP_DEG = 8.0
AZIMUTH_HALF_WIDTH_DEG = 15.0


def sample_hg(g, n):
    u = RNG.random(n)
    s = (1 - g * g) / (1 - g + 2 * g * u)
    return (1 + g * g - s * s) / (2 * g)


def sample_draine(n):
    out = np.empty(n)
    todo = np.arange(n)
    while todo.size:
        m = sample_hg(G_D, todo.size)
        accept = RNG.random(todo.size) < (1 + ALPHA * m * m) / (1 + ALPHA)
        out[todo[accept]] = m[accept]
        todo = todo[~accept]
    return out


def sample_cos(n):
    m = np.empty(n)
    pick = RNG.random(n) < W_D
    m[pick] = sample_draine(pick.sum())
    m[~pick] = sample_hg(G_HG, (~pick).sum())
    return m


def rotate(ux, uy, uz, ct):
    st = np.sqrt(np.maximum(0, 1 - ct * ct))
    ph = 2 * np.pi * RNG.random(ct.size)
    cp, sp = np.cos(ph), np.sin(ph)
    nx, ny, nz = np.empty_like(ux), np.empty_like(uy), np.empty_like(uz)
    pole = np.abs(uz) > 0.99999
    a = ~pole
    den = np.sqrt(1 - uz[a] ** 2)
    nx[a] = st[a] * (ux[a] * uz[a] * cp[a] - uy[a] * sp[a]) / den + ux[a] * ct[a]
    ny[a] = st[a] * (uy[a] * uz[a] * cp[a] + ux[a] * sp[a]) / den + uy[a] * ct[a]
    nz[a] = -st[a] * cp[a] * den + uz[a] * ct[a]
    nx[pole] = st[pole] * cp[pole]
    ny[pole] = st[pole] * sp[pole]
    nz[pole] = np.sign(uz[pole]) * ct[pole]
    return nx, ny, nz


def slab(tau, mu0, n, diffuse=False, escapes=None):
    """Reflected and transmitted fractions; z = optical depth below the top, uz > 0 downward. The Sun's beam travels
    toward +x. escapes (a list) collects the directions (ux, uy, uz) of the reflected photons."""
    uz = np.sqrt(RNG.random(n)) if diffuse else np.full(n, mu0)
    ux, uy, z = np.sqrt(1 - uz * uz), np.zeros(n), np.zeros(n)
    reflected = transmitted = 0
    while z.size:
        z = z + uz * (-np.log(RNG.random(z.size)))
        up, down = z < 0, z > tau
        reflected += up.sum()
        transmitted += down.sum()
        if escapes is not None:
            escapes.append((ux[up], uy[up], uz[up]))
        keep = ~(up | down)
        z, ux, uy, uz = z[keep], ux[keep], uy[keep], uz[keep]
        ux, uy, uz = rotate(ux, uy, uz, sample_cos(z.size))
    return reflected / n, transmitted / n


def radiance_bins(tau, mu0, n, batch=500000):
    """Bin-averaged reflected radiance [1/sr] per unit Sun irradiance: list of (zenith, azimuth, L, rel_sigma, count)."""
    counts = {}
    done = 0
    while done < n:
        m = min(batch, n - done)
        escapes = []
        slab(tau, mu0, m, escapes=escapes)
        ux = np.concatenate([e[0] for e in escapes])
        uy = np.concatenate([e[1] for e in escapes])
        uz = np.concatenate([e[2] for e in escapes])
        zen = np.degrees(np.arccos(np.clip(-uz, -1, 1)))
        azi = np.degrees(np.arctan2(np.abs(uy), ux))
        for vz in VIEW_ZENITHS_DEG:
            for va in REL_AZIMUTHS_DEG if vz > 0 else (0.0,):
                if vz == 0:
                    sel = zen <= ZENITH_CAP_DEG
                else:
                    sel = (np.abs(zen - vz) <= ZENITH_HALF_WIDTH_DEG) & (np.abs(azi - va) <= AZIMUTH_HALF_WIDTH_DEG)
                counts[(vz, va)] = counts.get((vz, va), 0) + int(sel.sum())
        done += m
    rows = []
    for (vz, va), k in counts.items():
        if vz == 0:
            lo, hi, dphi = 0.0, ZENITH_CAP_DEG, 2 * np.pi
        else:
            lo, hi = vz - ZENITH_HALF_WIDTH_DEG, vz + ZENITH_HALF_WIDTH_DEG
            # Both sides of the Sun's plane; at 0 and 180 deg the two half-bins meet in one bin of 2 half-widths.
            sides = 1 if va in (0.0, 180.0) else 2
            dphi = sides * np.radians(2 * AZIMUTH_HALF_WIDTH_DEG)
        mu_hi, mu_lo = np.cos(np.radians(lo)), np.cos(np.radians(hi))
        projected = dphi * (mu_hi ** 2 - mu_lo ** 2) / 2
        rows.append((vz, va, mu0 * k / n / projected, 1 / np.sqrt(max(k, 1)), k))
    return rows


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if "--radiance" in sys.argv:
        n = int(args[0]) if args else 4000000
        print("# Reflected radiance of a conservative cloud slab (black surface), Monte Carlo, "
              "tools/validation/cloud_slab_monte_carlo.py --radiance " + str(n))
        print("# Per unit Sun irradiance normal to the beam [1/sr], bin-averaged; bins and azimuth convention in the "
              "tool's docstring. rel_sigma = 1/sqrt(count).")
        print("sun_zenith_deg,optical_depth,view_zenith_deg,rel_azimuth_deg,radiance_per_sr,rel_sigma,count")
        for sun_zenith in (0.0, 60.0):
            mu0 = np.cos(np.radians(sun_zenith))
            for tau in (1, 10, 30):
                for vz, va, rad, sig, k in radiance_bins(tau, mu0, n):
                    print(f"{sun_zenith:.0f},{tau},{vz:.0f},{va:.0f},{rad:.6f},{sig:.4f},{k}", flush=True)
        return
    n = int(args[0]) if args else 100000
    for mu0 in (1.0, 0.5):
        for tau in (1, 3, 10, 30, 50):
            r, t = slab(tau, mu0, n)
            print(f"mu0 {mu0} tau {tau:3d} R {r:.4f} T {t:.4f}")
    r, t = slab(20, 0.0, n, diffuse=True)
    print(f"isotropic incidence tau 20 R {r:.4f} T {t:.4f}")


if __name__ == "__main__":
    main()
