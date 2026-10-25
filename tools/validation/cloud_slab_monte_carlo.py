"""Monte Carlo reflectance and reflected radiance of a conservative cloud slab with the core's droplet phase function.

Independent check of the delta-Eddington two-stream reference and of the shader's source terms in
core/include/starsim/environment/cloud_optics.hpp: photons enter a plane-parallel slab (optical depth tau, black
surface below) with the Sun at cos zenith mu0 (or isotropic incidence), scatter without absorption with the Jendersie &
d'Eon 2023 phase function for 20 um droplets (Henyey-Greenstein + Draine mixture, sampled exactly: HG by inversion,
Draine by rejection from HG), and the escaping fractions are counted. Offline tool only, not part of the build; fixed
seed.

    python tools/validation/cloud_slab_monte_carlo.py [photons]            # R and T (hemispheric)
    python tools/validation/cloud_slab_monte_carlo.py --radiance [photons] > tests/validation/weather/cloud_slab_radiance_mc.csv
    python tools/validation/cloud_slab_monte_carlo.py --albedo [photons] > tests/validation/weather/cloud_slab_albedo_mc.csv
    python -I tools/validation/cloud_slab_monte_carlo.py --table-runs <run dir> [workers]     # 16 x 27 slabs, ~30 min / 24
    python -I tools/validation/cloud_slab_monte_carlo.py --table-build <run dir> config/clouds/cloud_transfer_table.bin \
        tests/validation/weather/cloud_transfer_table_sigma.bin

--table-runs / --table-build make the transfer table of the multiply scattered light (Task 8b; quantity, layout and
reductions in core/include/starsim/environment/cloud_transfer_table.hpp): fixed seeds per run (run_seed), photons per
run by slab depth (run_photons: 1e6 up to tau 64, 4e5 up to 256, 2e5 beyond; 3.424e8 in all), 1-sigma per entry from 8
batches (the sigma file).

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


def sample_hg(g, n, rng=RNG):
    u = rng.random(n)
    s = (1 - g * g) / (1 - g + 2 * g * u)
    return (1 + g * g - s * s) / (2 * g)


def sample_draine(n, rng=RNG):
    out = np.empty(n)
    todo = np.arange(n)
    while todo.size:
        m = sample_hg(G_D, todo.size, rng)
        accept = rng.random(todo.size) < (1 + ALPHA * m * m) / (1 + ALPHA)
        out[todo[accept]] = m[accept]
        todo = todo[~accept]
    return out


def sample_cos(n, rng=RNG):
    m = np.empty(n)
    pick = rng.random(n) < W_D
    m[pick] = sample_draine(pick.sum(), rng)
    m[~pick] = sample_hg(G_HG, (~pick).sum(), rng)
    return m


def rotate(ux, uy, uz, ct, rng=RNG):
    st = np.sqrt(np.maximum(0, 1 - ct * ct))
    ph = 2 * np.pi * rng.random(ct.size)
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


# ---- Transfer table of the multiply scattered light (orders >= 2), Task 8b; layout in cloud_transfer_table.hpp ----

G_MEAN = (1 - W_D) * G_HG + W_D * G_D * (1 + ALPHA * (3 + 2 * G_D ** 2) / 5) / (1 + ALPHA * (1 + 2 * G_D ** 2) / 3)
TABLE_DIFFUSION_K = 0.75 * (1 - G_MEAN)  # x = 1 / (1 + K tau): the two-stream diffuse transmittance
TABLE_DEPTH_SCALE = 0.25                # depth nodes a_i = scale (ratio^i - 1), i < count - 1; the last node infinity
TABLE_DEPTH_RATIO = np.sqrt(2.0)
TABLE_DEPTH_NODES = 22
TABLE_MU_NODES = 16                     # mu0 = ((i + 1) / count)^2; the table holds J / mu0
TABLE_THETA_NODES = 48                  # u(theta) = c1 ln(1 + theta / theta0) + c2 theta, u(pi) = count - 1
TABLE_THETA0 = 0.002
TABLE_THETA_LINEAR = 12.0 / np.pi
TABLE_PSI_NODES = 13                    # psi = pi i / (count - 1)
# Histogram of the photons' directions before a collision (sun frame): finer than the table's view directions.
IN_THETA_BINS = 128
IN_THETA0 = 0.001
IN_THETA_LINEAR = 32.0 / np.pi
IN_PSI_BINS = 12
# Depth bins of a run, mirrored about the middle of the slab (optical depth from the nearer surface).
DEPTH_EDGES = np.array([0, .05, .1, .15, .2, .3, .4, .5, .6, .8, 1, 1.25, 1.5, 1.75, 2, 2.5, 3, 3.5, 4, 5, 6, 7, 8, 10,
                        12, 14, 16, 20, 24, 28, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 384,
                        448, 512])
# Slabs simulated per mu0 node; table nodes in between are interpolated in x = 1 / (1 + K tau).
RUN_TAUS = (0.1, 0.15, 0.25, 0.35, 0.5, 0.75, 1, 1.5, 2, 3, 4, 6, 8, 12, 16, 24, 32, 48, 64, 96, 128, 192, 256, 384,
            512, 768, 1024)
SEMI_INFINITE_MIN_TAU = 256
TABLE_BATCHES = 8
KERNEL_SAMPLES = 1000000
TABLE_MAGIC = b"STCLDTT2"


def theta_map_c1(count, theta0, linear):
    return (count - 1 - linear * np.pi) / np.log(1 + np.pi / theta0)


def theta_to_u(theta, count, theta0, linear):
    return theta_map_c1(count, theta0, linear) * np.log1p(theta / theta0) + linear * theta


def u_to_theta(u, count, theta0, linear):
    lo, hi = np.zeros_like(u, dtype=float), np.full_like(u, np.pi, dtype=float)
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        below = theta_to_u(mid, count, theta0, linear) < u
        lo, hi = np.where(below, mid, lo), np.where(below, hi, mid)
    return 0.5 * (lo + hi)


def table_depth_nodes():
    i = np.arange(TABLE_DEPTH_NODES - 1)
    return np.append(TABLE_DEPTH_SCALE * (TABLE_DEPTH_RATIO ** i - 1), np.inf)


def table_mu_nodes():
    return ((np.arange(TABLE_MU_NODES) + 1) / TABLE_MU_NODES) ** 2


def table_theta_nodes():
    return u_to_theta(np.arange(TABLE_THETA_NODES, dtype=float), TABLE_THETA_NODES, TABLE_THETA0, TABLE_THETA_LINEAR)


def table_psi_nodes():
    return np.pi * np.arange(TABLE_PSI_NODES) / (TABLE_PSI_NODES - 1)


def in_theta_edges():
    return u_to_theta(np.arange(IN_THETA_BINS + 1, dtype=float), IN_THETA_BINS + 1, IN_THETA0, IN_THETA_LINEAR)


def run_depth_edges(tau):
    top = DEPTH_EDGES[DEPTH_EDGES < tau / 2]
    return np.concatenate([top, [tau / 2], tau - top[::-1]])


def sun_frame(ux, uy, uz, mu0):
    """Angle theta to the Sun's propagation direction and folded azimuth psi about it (0 = tilted toward up).
    Coordinates of slab(): z down, the beam (sqrt(1 - mu0^2), 0, mu0); e1 = (mu0, 0, -sqrt(1 - mu0^2)) is up's part
    normal to the beam."""
    s0 = np.sqrt(1 - mu0 * mu0)
    chord = np.sqrt((ux - s0) ** 2 + uy ** 2 + (uz - mu0) ** 2)
    theta = 2 * np.arcsin(np.minimum(chord / 2, 1))
    psi = np.arctan2(np.abs(uy), ux * mu0 - uz * s0)
    return theta, psi


def in_bin_index(theta, psi):
    t = np.floor(theta_to_u(theta, IN_THETA_BINS + 1, IN_THETA0, IN_THETA_LINEAR)).astype(np.int64)
    p = np.floor(psi / np.pi * IN_PSI_BINS).astype(np.int64)
    return np.clip(t, 0, IN_THETA_BINS - 1) * IN_PSI_BINS + np.clip(p, 0, IN_PSI_BINS - 1)


def in_bin_solid_angles():
    edges = in_theta_edges()
    ring = 2 * np.sin((edges[1:] - edges[:-1]) / 2) * np.sin((edges[1:] + edges[:-1]) / 2)  # cos a - cos b
    return np.repeat(ring, IN_PSI_BINS) * 2 * np.pi / IN_PSI_BINS  # both mirror halves of a folded psi bin


def direction_kernel(seed=7):
    """K[o, i] = (w_D / dOmega_i) * integral over in-bin i of p_D(Omega . Omega_o): the mass of the phase function's
    Draine part around the table direction o that falls into bin i (sampled exactly like the photons), per steradian of
    the bin, times its weight w_D. The HG part (g 0.995: a ~0.3 deg peak) is not in the table: the view integration
    treats it as forward (cloud_transfer_table.hpp, "HG peak")."""
    rng = np.random.default_rng(seed)
    theta_o, psi_o = np.meshgrid(table_theta_nodes(), table_psi_nodes(), indexing="ij")
    theta_o, psi_o = theta_o.ravel(), psi_o.ravel()
    kernel = np.zeros((theta_o.size, IN_THETA_BINS * IN_PSI_BINS))
    # Sun frame as slab() coordinates with mu0 = 1: the beam (0, 0, 1), e1 = (1, 0, 0), e2 = (0, 1, 0).
    for o in range(theta_o.size):
        n = KERNEL_SAMPLES
        ux = np.full(n, np.sin(theta_o[o]) * np.cos(psi_o[o]))
        uy = np.full(n, np.sin(theta_o[o]) * np.sin(psi_o[o]))
        uz = np.full(n, np.cos(theta_o[o]))
        ux, uy, uz = rotate(ux, uy, uz, sample_draine(n, rng), rng)
        theta, psi = sun_frame(ux, uy, uz, 1.0)
        kernel[o] = np.bincount(in_bin_index(theta, psi), minlength=kernel.shape[1]) / n
    return W_D * kernel / in_bin_solid_angles()


def transfer_run(job):
    """One slab: histograms (batch, depth bin, in-direction bin) of the collisions of photons that scattered before
    (their emission is the source of orders >= 2), plus the escaping fractions."""
    tau, mu0, photons, seed, out_file = job
    rng = np.random.default_rng(seed)
    edges = run_depth_edges(tau)
    bins = (edges.size - 1) * IN_THETA_BINS * IN_PSI_BINS
    hist = np.zeros(TABLE_BATCHES * bins, np.int64)
    reflected = transmitted = 0
    s0 = np.sqrt(1 - mu0 * mu0)
    per = photons // TABLE_BATCHES
    n = per * TABLE_BATCHES
    # All batches in one loop (photon i belongs to batch i % batches): the long tails of thick slabs cost per loop.
    ux, uy, uz, z = np.full(n, s0), np.zeros(n), np.full(n, mu0), np.zeros(n)
    batch = np.arange(n) % TABLE_BATCHES
    scattered = False
    pending, pending_n = [], 0
    while z.size:
        z = z + uz * (-np.log(1 - rng.random(z.size)))
        up, down = z < 0, z > tau
        reflected += int(up.sum())
        transmitted += int(down.sum())
        keep = ~(up | down)
        z, ux, uy, uz, batch = z[keep], ux[keep], uy[keep], uz[keep], batch[keep]
        if scattered and z.size:
            depth = np.clip(np.searchsorted(edges, z, side="right") - 1, 0, edges.size - 2)
            theta, psi = sun_frame(ux, uy, uz, mu0)
            pending.append(batch * bins + depth * (IN_THETA_BINS * IN_PSI_BINS) + in_bin_index(theta, psi))
            pending_n += z.size
            if pending_n > 4000000:
                hist += np.bincount(np.concatenate(pending), minlength=hist.size)
                pending, pending_n = [], 0
        ux, uy, uz = rotate(ux, uy, uz, sample_cos(z.size, rng), rng)
        scattered = True
    if pending:
        hist += np.bincount(np.concatenate(pending), minlength=hist.size)
    hist = hist.reshape(TABLE_BATCHES, bins)
    np.savez_compressed(out_file, hist=hist.astype(np.uint32), tau=tau, mu0=mu0, photons=n,
                        edges=edges, reflected=reflected, transmitted=transmitted)
    return out_file


def run_seed(m, t):
    """Seed of run m (1-based mu0 node), t (tau index): the even nodes keep the seeds of the first 8-node set."""
    return 1000 + 100 * (m // 2) + t if m % 2 == 0 else 5000 + 100 * m + t


def run_photons(tau):
    return 1000000 if tau <= 64 else 400000 if tau <= 256 else 200000


def table_runs(run_dir, workers):
    import multiprocessing
    import os
    os.makedirs(run_dir, exist_ok=True)
    jobs = []
    for m, mu0 in enumerate(table_mu_nodes(), start=1):
        for t, tau in enumerate(RUN_TAUS):
            out = os.path.join(run_dir, f"run_mu{m}_tau{t:02d}.npz")
            if not os.path.exists(out):
                jobs.append((tau, mu0, run_photons(tau), run_seed(m, t), out))
    jobs.sort(key=lambda j: -j[0])  # thick slabs first: they take longest
    with multiprocessing.Pool(workers) as pool:
        for out in pool.imap_unordered(transfer_run, jobs):
            print("done", out, flush=True)


def run_source(run, kernel):
    """J [1/sr per unit Sun irradiance and unit optical depth] at the depth-bin centres: mean and 1-sigma over batches."""
    hist = run["hist"].astype(float).reshape(TABLE_BATCHES, run["edges"].size - 1, -1)
    width = np.diff(run["edges"])
    per = float(run["photons"]) / TABLE_BATCHES
    j = np.einsum("bdi,oi->bdo", hist, kernel) * float(run["mu0"]) / (per * width[None, :, None])
    return j.mean(0), j.std(0, ddof=1) / np.sqrt(TABLE_BATCHES)


def profile_at(centres, values, sigmas, depth):
    """Linear in depth between bin centres (linear extrapolation beyond the first / last centre)."""
    i = int(np.clip(np.searchsorted(centres, depth) - 1, 0, centres.size - 2))
    f = (depth - centres[i]) / (centres[i + 1] - centres[i])
    return (1 - f) * values[i] + f * values[i + 1], np.sqrt(((1 - f) * sigmas[i]) ** 2 + (f * sigmas[i + 1]) ** 2)


def table_build(run_dir, out_values, out_sigma):
    import os
    kernel = direction_kernel()
    nodes = table_depth_nodes()
    x_of = lambda tau: 1 / (1 + TABLE_DIFFUSION_K * tau)
    shape = (TABLE_DEPTH_NODES, TABLE_DEPTH_NODES, TABLE_MU_NODES, TABLE_THETA_NODES * TABLE_PSI_NODES)
    values, sigmas = np.zeros(shape), np.zeros(shape)
    photons = 0
    for m in range(1, TABLE_MU_NODES + 1):
        mi = m - 1  # run files count from 1
        runs = []
        for t in range(len(RUN_TAUS)):
            run = np.load(os.path.join(run_dir, f"run_mu{m}_tau{t:02d}.npz"))
            photons += int(run["photons"])
            mean, sigma = run_source(run, kernel)
            mean, sigma = mean / float(run["mu0"]), sigma / float(run["mu0"])  # per unit flux through the top
            edges = run["edges"]
            runs.append((float(run["tau"]), 0.5 * (edges[1:] + edges[:-1]), mean, sigma))

        def at(run, depth_top, depth_bottom):
            tau, centres, mean, sigma = run
            depth = depth_top if depth_top is not None else tau - depth_bottom
            return profile_at(centres, mean, sigma, depth)

        for ia, a in enumerate(nodes):
            for ib, b in enumerate(nodes):
                if np.isinf(a):
                    if np.isinf(b):  # deep in a semi-infinite medium: the deepest finite node's value
                        values[ia, ib, mi], sigmas[ia, ib, mi] = values[ia - 1, ib, mi], sigmas[ia - 1, ib, mi]
                    continue  # infinitely far below the top of a finite slab: no light
                from_top = a <= b
                depth_top, depth_bottom = (a, None) if from_top else (None, b)
                if np.isinf(b):
                    # Semi-infinite: at a fixed depth the diffusion field is linear in 1 / (tau + 2 z0) and
                    # 1 / K ~ 2 z0, so linear in x; least-squares line through the thick runs (tau >= 256, the
                    # depth in their upper half), its value at x = 0.
                    thick = [r for r in runs if r[0] >= SEMI_INFINITE_MIN_TAU and r[0] >= 2 * a]
                    xs = np.array([x_of(r[0]) for r in thick])
                    weights = 1 / len(thick) - xs.mean() * (xs - xs.mean()) / np.sum((xs - xs.mean()) ** 2)
                    at_depth = [at(r, a, None) for r in thick]
                    values[ia, ib, mi] = sum(w * v for w, (v, _) in zip(weights, at_depth))
                    sigmas[ia, ib, mi] = np.sqrt(sum((w * s) ** 2 for w, (_, s) in zip(weights, at_depth)))
                    continue
                tau = a + b
                if tau <= RUN_TAUS[0]:  # thinner than the thinnest run: the source scales with the slab
                    v, s = at(runs[0], a * RUN_TAUS[0] / tau, None) if tau > 0 else (0.0, 0.0)
                    values[ia, ib, mi], sigmas[ia, ib, mi] = v * tau / RUN_TAUS[0], s * tau / RUN_TAUS[0]
                    continue
                k = int(np.searchsorted(RUN_TAUS, tau) - 1)
                k = min(max(k, 0), len(RUN_TAUS) - 2)
                (v1, s1), (v2, s2) = at(runs[k], depth_top, depth_bottom), at(runs[k + 1], depth_top, depth_bottom)
                f = (x_of(tau) - x_of(RUN_TAUS[k])) / (x_of(RUN_TAUS[k + 1]) - x_of(RUN_TAUS[k]))
                values[ia, ib, mi] = (1 - f) * v1 + f * v2
                sigmas[ia, ib, mi] = np.sqrt(((1 - f) * s1) ** 2 + (f * s2) ** 2)
    values = np.maximum(values, 0.0)
    header = table_header(photons, " ".join(["python -I tools/validation/cloud_slab_monte_carlo.py --table-build"] +
                                             [os.path.basename(p) for p in (out_values, out_sigma)]))
    for path, data in ((out_values, values), (out_sigma, sigmas)):
        with open(path, "wb") as f:
            f.write(header)
            f.write(data.astype("<f2").tobytes())
    print(f"table {shape} from {photons} photons; max relative sigma (J > 1e-3) "
          f"{np.max(np.where(values > 1e-3, sigmas / np.maximum(values, 1e-30), 0)):.4f}")


def table_header(photons, command):
    """256 bytes: magic, the node maps and the Draine weight (cloud_transfer_table.hpp reads them), the photon count, the command."""
    import struct
    head = TABLE_MAGIC + struct.pack("<4I", TABLE_DEPTH_NODES, TABLE_MU_NODES, TABLE_THETA_NODES, TABLE_PSI_NODES)
    head += struct.pack("<7d", TABLE_DEPTH_SCALE, TABLE_DEPTH_RATIO, TABLE_DIFFUSION_K, TABLE_THETA0,
                        theta_map_c1(TABLE_THETA_NODES, TABLE_THETA0, TABLE_THETA_LINEAR), TABLE_THETA_LINEAR, W_D)
    head += struct.pack("<Q", photons)
    text = command.encode("ascii")[: 256 - len(head) - 1]
    return head + text + b"\0" * (256 - len(head) - len(text))


def albedo_rows(n):
    rows = []
    for mu0 in (1.0, 0.5):
        for tau in (1, 3, 10, 30, 50):
            r, _ = slab(tau, mu0, n)
            rows.append((mu0, tau, r, np.sqrt(r * (1 - r) / n)))
    return rows


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if "--table-runs" in sys.argv:
        table_runs(args[0], int(args[1]) if len(args) > 1 else 20)
        return
    if "--table-build" in sys.argv:
        table_build(*args[:3])
        return
    if "--albedo" in sys.argv:
        n = int(args[0]) if args else 2000000
        print("# Albedo of a conservative cloud slab (black surface), Monte Carlo, "
              "tools/validation/cloud_slab_monte_carlo.py --albedo " + str(n))
        print("sun_cos_zenith,optical_depth,albedo,sigma")
        for mu0, tau, r, s in albedo_rows(n):
            print(f"{mu0},{tau},{r:.6f},{s:.6f}", flush=True)
        return
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
