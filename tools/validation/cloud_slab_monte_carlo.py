"""Monte Carlo reflectance of a conservative cloud slab with the core's droplet phase function.

Independent check of the delta-Eddington two-stream reference in core/include/starsim/environment/cloud_optics.hpp:
photons enter a plane-parallel slab (optical depth tau, black surface below) with the Sun at cos zenith mu0 (or
isotropic incidence), scatter without absorption with the Jendersie & d'Eon 2023 phase function for 20 um droplets
(Henyey-Greenstein + Draine mixture, sampled exactly: HG by inversion, Draine by rejection from HG), and the escaping
fractions are counted. Offline tool only, not part of the build; fixed seed.

    python tools/validation/cloud_slab_monte_carlo.py [photons]
"""
import sys

import numpy as np

D_UM = 20.0
G_HG = np.exp(-0.0990567 / (D_UM - 1.67154))
G_D = np.exp(-2.20679 / (D_UM + 3.91029) - 0.428934)
ALPHA = np.exp(3.62489 - 8.29288 / (D_UM + 5.52825))
W_D = np.exp(-0.599085 / (D_UM - 0.641583) - 0.665888)
RNG = np.random.default_rng(1)


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


def slab(tau, mu0, n, diffuse=False):
    """Reflected and transmitted fractions; z = optical depth below the top, uz > 0 downward."""
    uz = np.sqrt(RNG.random(n)) if diffuse else np.full(n, mu0)
    ux, uy, z = np.sqrt(1 - uz * uz), np.zeros(n), np.zeros(n)
    reflected = transmitted = 0
    while z.size:
        z = z + uz * (-np.log(RNG.random(z.size)))
        up, down = z < 0, z > tau
        reflected += up.sum()
        transmitted += down.sum()
        keep = ~(up | down)
        z, ux, uy, uz = z[keep], ux[keep], uy[keep], uz[keep]
        ux, uy, uz = rotate(ux, uy, uz, sample_cos(z.size))
    return reflected / n, transmitted / n


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 100000
    for mu0 in (1.0, 0.5):
        for tau in (1, 3, 10, 30, 50):
            r, t = slab(tau, mu0, n)
            print(f"mu0 {mu0} tau {tau:3d} R {r:.4f} T {t:.4f}")
    r, t = slab(20, 0.0, n, diffuse=True)
    print(f"isotropic incidence tau 20 R {r:.4f} T {t:.4f}")


if __name__ == "__main__":
    main()
