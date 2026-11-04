"""Residual regolith roughness after removing the generated crater and rock populations.

Synthesises a patch of the generated surface at Guo et al. 2021's DTM resolution (5 mm) and evaluates their statistic:
RMS height = standard deviation of the plane-detrended elevation in square windows of side 125 mm, median over windows.
Craters: equilibrium SFD, parabolic bowl (rim-to-floor depth d), rim h, exterior rim falloff h (r/R)^-3.
Rocks: half-ellipsoid caps a:b:c, burial fraction of the full height below the surface, added on top.
Constants mirror config/moon_surface.toml (keep in sync). Result recorded in docs/subsystems/moon-surface.md
(Regolith roughness): residual 3.72-3.76 mm over seeds 1-8.
Usage: python tools/validation/moon_roughness_residual.py [seed]
"""
import sys
import numpy as np

SEED = int(sys.argv[1]) if len(sys.argv) > 1 else 1
rng = np.random.default_rng(SEED)

PX = 0.005          # Guo DTM 5 mm
L = 8.0             # patch side, m
N = int(L / PX)
W_PX = 25           # 125 mm window
# config/moon_surface.toml
C_K, C_B, C_DMIN, C_DMAX = 0.0336, 2.0, 0.01, 4.0
DD_FRESH, DD_DEG, RIM_FRESH, DEG_FRAC = 0.13, 0.07, 0.024, 0.70
R_K, R_B, R_DMIN, R_DMAX = 0.00154, 2.286, 0.01, 4.0
BA, CA, BURIAL = 0.71, 0.50, 0.3
H = 0.78

def sample_power(k, b, dmin, dmax, area):
    n_exp = k * (dmin ** -b - dmax ** -b) * area
    n = rng.poisson(n_exp)
    u = rng.random(n)
    # inverse CDF of truncated power law
    return (dmin ** -b - u * (dmin ** -b - dmax ** -b)) ** (-1.0 / b)

def stamp(z, cx, cy, half, fn):
    i0 = max(int((cx - half) / PX), 0); i1 = min(int((cx + half) / PX) + 1, N)
    j0 = max(int((cy - half) / PX), 0); j1 = min(int((cy + half) / PX) + 1, N)
    if i0 >= i1 or j0 >= j1:
        return
    xs = (np.arange(i0, i1) + 0.5) * PX - cx
    ys = (np.arange(j0, j1) + 0.5) * PX - cy
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    z[i0:i1, j0:j1] += fn(X, Y)

def craters():
    z = np.zeros((N, N))
    margin = 3 * C_DMAX
    area = (L + 2 * margin) ** 2
    ds = sample_power(C_K, C_B, C_DMIN, C_DMAX, area)
    for D in ds:
        cx, cy = rng.uniform(-margin, L + margin, 2)
        R = D / 2
        fresh = rng.random() >= DEG_FRAC
        d = (DD_FRESH if fresh else DD_DEG) * D
        h = RIM_FRESH * D * (d / (DD_FRESH * D))   # degraded rim scaled with depth
        ext = 3 * R
        def f(X, Y, R=R, d=d, h=h):
            r = np.hypot(X, Y)
            inner = h - d + d * (r / R) ** 2
            outer = h * (np.maximum(r, R) / R) ** -3
            out = np.where(r < R, inner, outer)
            return np.where(r < ext, out, 0.0)
        stamp(z, cx, cy, ext, f)
    return z

def rocks():
    z = np.zeros((N, N))
    margin = R_DMAX
    area = (L + 2 * margin) ** 2
    ds = sample_power(R_K, R_B, R_DMIN, R_DMAX, area)
    for D in ds:
        cx, cy = rng.uniform(-margin, L + margin, 2)
        a, b, c = D / 2, BA * D / 2, CA * D / 2       # semi-axes, D = long axis
        th = rng.uniform(0, np.pi)
        ct, st = np.cos(th), np.sin(th)
        bur = BURIAL * 2 * c                           # share of full height below the surface
        def f(X, Y, a=a, b=b, c=c, bur=bur):
            u = (ct * X + st * Y) / a; v = (-st * X + ct * Y) / b
            q = 1 - u * u - v * v
            top = (c - bur) + c * np.sqrt(np.clip(q, 0, None))   # centre at c - bur above the ground
            return np.where(q > 0, np.clip(top, 0, None), 0.0)
        stamp(z, cx, cy, a, f)
    return z

def fbm():
    k = np.fft.fftfreq(N, d=PX)
    KX, KY = np.meshgrid(k, k, indexing="ij")
    kk = np.hypot(KX, KY); kk[0, 0] = 1.0
    amp = kk ** (-(H + 1.0))
    amp[0, 0] = 0.0
    ph = rng.standard_normal((N, N)) + 1j * rng.standard_normal((N, N))
    z = np.real(np.fft.ifft2(amp * ph))
    return z

def window_rms(z):
    n = N // W_PX
    xs = (np.arange(W_PX) - (W_PX - 1) / 2) * PX
    X, Y = np.meshgrid(xs, xs, indexing="ij")
    A = np.stack([np.ones(W_PX * W_PX), X.ravel(), Y.ravel()], 1)
    P = A @ np.linalg.pinv(A)
    blocks = z[: n * W_PX, : n * W_PX].reshape(n, W_PX, n, W_PX).transpose(0, 2, 1, 3).reshape(n * n, -1)
    res = blocks - blocks @ P.T
    return np.sqrt((res ** 2).mean(1))

zc = craters(); zr = rocks(); zf = fbm()
rc, rr, rf = window_rms(zc), window_rms(zr), window_rms(zf)
zf /= np.median(rf)   # unit median window RMS
rcr = window_rms(zc + zr)
print(f"seed {SEED}  windows {rc.size}")
print(f"craters: median {np.median(rc)*1e3:.3f} mm  rms-of-var {np.sqrt((rc**2).mean())*1e3:.3f} mm")
print(f"rocks:   median {np.median(rr)*1e3:.3f} mm  rms-of-var {np.sqrt((rr**2).mean())*1e3:.3f} mm")
print(f"cr+rk:   median {np.median(rcr)*1e3:.3f} mm  rms-of-var {np.sqrt((rcr**2).mean())*1e3:.3f} mm  p90 {np.percentile(rcr,90)*1e3:.3f}")
# solve residual r so that median window RMS of craters+rocks+r*fbm = 4 mm
target = 0.004
lo, hi = 0.0, target
for _ in range(40):
    mid = 0.5 * (lo + hi)
    m = np.median(window_rms(zc + zr + mid * zf))
    if m > target: hi = mid
    else: lo = mid
r_med = 0.5 * (lo + hi)
tot = window_rms(zc + zr + r_med * zf)
print(f"residual (median-matched): {r_med*1e3:.3f} mm  -> total median {np.median(tot)*1e3:.3f}  range p5-p95 {np.percentile(tot,5)*1e3:.2f}-{np.percentile(tot,95)*1e3:.2f} mm")
cv = (rcr ** 2).mean()
print(f"residual (variance-subtracted): {np.sqrt(max(target**2 - cv, 0))*1e3:.3f} mm")
