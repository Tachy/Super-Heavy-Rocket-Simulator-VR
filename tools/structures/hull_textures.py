"""Procedural steel map of the vehicle hulls (weld seams + oil-canning dents), baked from the detail file's numbers.

One RGBA8 PNG, `hull_steel.png`, tiling (seamless in both axes), no photo pixels, deterministic (fixed lattice seed,
fixed zlib level). The shader maps it with the hull's UV (arc metres at the barrel radius, station metres):

    u period = circumference / U_TILES (an integer count: the map closes around the hull), v period = ring pitch with a
    weld bead at v = 0 (so a weld at every multiple of the pitch). Texel = period / size.

Channels (linear data, no sRGB; mipmaps are plain box averages made by the engine, which is why the data are chosen
so that averaging stays correct):
    R, G  slope of the surface along u and v (dh/du, dh/dv), stored as 0.5 + slope / SLOPE_RANGE / 2
    B     slope energy sx^2 + sy^2 over SLOPE_ENERGY_RANGE (0.1). The shader's variance term is B - (mean slope)^2: zero on
          mip 0, the slope variance lost to filtering on the coarser mips (feeds the roughness: Toksvig-like, so
          glints do not shimmer)
    A     heat-affected band coverage of the weld (shader: darker, rougher)
"""
from __future__ import annotations

import math
import struct
import zlib

import numpy as np

U_TILES = 6               # [E] the map repeats this often around the hull
SIZE_U = 1024             # [E] texels along u: 4.6 mm at R = 4.5 m
SIZE_V = 512              # [E] texels along v: 3.5 mm at the 1.8 m ring pitch
SLOPE_RANGE = 0.6         # slopes are stored over +-SLOPE_RANGE / 2 (the bead's 0.24 plus dents fit)
SLOPE_ENERGY_RANGE = 0.1
SEAM_FRACTION = 0.2       # [E] longitudinal seam bead height as a fraction of the ring weld's (photos: fainter than the rings)
SEAM_HAZ_SHARE = 0.3      # [E] heat-affected darkening of the longitudinal seam relative to the ring weld's
HAZ_FACTOR = 1.2          # [E] heat-affected band half width / bead half width
DENT_OCTAVES = ((4, 2, 1.0), (12, 5, 1.085))  # [E] (cells along u, cells along v, weight): ~1.2 m and ~0.38 m waves
SEED = 20241031
FILE_NAME = "hull_steel.png"
STRINGERS_PER_TILE = 12   # [E] 0.39 m pitch: NSF photos: dotted vertical lines ~0.35 +-0.1 m apart on booster and ship
DOT_PITCH_M = 0.09        # [E] +-0.02 m along the line, same photo (3.7 px at 41 px/m)
DOT_RADIUS_M = 0.018      # [E] ~4 texels: not resolved in the frames, reads as a faint line beyond ~30 m
DOT_LINE_SHARE = 0.6     # [E] share of the stringer lines that carry dots: frames (booster and ship, f12 / NSF photos): bands, not a grid
DOT_DROPOUT = 0.2        # [E] share of dots missing along a line (same frames: broken rows)
DOT_COVERAGE = 0.8        # [E] darkening share of a dot in the A channel
FLAP_FILE_NAME = "flap_lee.png"
FLAP_SIZE = (1024, 342)   # texels along the hinge, span: the aft flap's 12 x 4 m
FLAP_PERIOD_M = (12.0, 4.0)
FLAP_COLUMN_M = 0.95      # [E] panel width: f14 file 5099 s: 115 px against the 7.2 m outer edge of 875 px
FLAP_ROW_M = 0.74         # [E] five rows over the 3.69 m span, same frame
FLAP_DIAGONALS_M = (((4.7, 0.0), (1.2, 3.69)), ((4.7, 0.0), (6.3, 3.69)))  # [E] +-0.5 m: the two stiffeners, same frame
FLAP_LINE_HALF_M = 0.025  # [E]
FLAP_SLOPE = 0.08         # [E] rise of the panel lines


def _lattice(rng, nu, nv):
    return rng.random((nv, nu))


def _periodic_noise(rng, cells_u, cells_v, size_u, size_v):
    """Smooth periodic value noise (smoothstep interpolation) on a size_v x size_u grid."""
    lat = _lattice(rng, cells_u, cells_v)
    u = np.arange(size_u) * cells_u / size_u
    v = np.arange(size_v) * cells_v / size_v
    iu, iv = np.floor(u).astype(int), np.floor(v).astype(int)
    fu, fv = u - iu, v - iv
    fu, fv = fu * fu * (3.0 - 2.0 * fu), fv * fv * (3.0 - 2.0 * fv)
    iu1, iv1 = (iu + 1) % cells_u, (iv + 1) % cells_v
    a = lat[np.ix_(iv, iu)] * (1 - fu)[None, :] + lat[np.ix_(iv, iu1)] * fu[None, :]
    b = lat[np.ix_(iv1, iu)] * (1 - fu)[None, :] + lat[np.ix_(iv1, iu1)] * fu[None, :]
    return a * (1 - fv)[:, None] + b * fv[:, None]


def periods_m(radius_m: float, ring_pitch_m: float):
    return 2.0 * math.pi * radius_m / U_TILES, ring_pitch_m


def bake(radius_m: float, hull: dict) -> np.ndarray:
    """(SIZE_V, SIZE_U, 4) uint8 array; row index = v, column = u."""
    pu, pv = periods_m(radius_m, hull["ring_pitch_m"])
    du, dv = pu / SIZE_U, pv / SIZE_V
    rng = np.random.default_rng(SEED)
    h = np.zeros((SIZE_V, SIZE_U))
    for cu, cv, w in DENT_OCTAVES:
        h += w * (_periodic_noise(rng, cu, cv, SIZE_U, SIZE_V) - 0.5) * (pu / cu + pv / cv) * 0.5
    sx = (np.roll(h, -1, axis=1) - np.roll(h, 1, axis=1)) / (2.0 * du)
    sy = (np.roll(h, -1, axis=0) - np.roll(h, 1, axis=0)) / (2.0 * dv)
    rms = math.sqrt(float(np.mean(sx * sx + sy * sy)))
    gain = hull["dent_slope_rms"] / rms
    sx, sy = sx * gain, sy * gain
    # weld bead: a triangle at v = 0 (wrapping), height weld_height_m, half width weld_half_width_m
    v = ((np.arange(SIZE_V) + 0.5) * dv + 0.5 * pv) % pv - 0.5 * pv  # signed distance to the nearest weld (m), texel centres
    half, height = hull["weld_half_width_m"], hull["weld_height_m"]
    bead_slope = -np.sign(v) * (np.abs(v) < half) * (height / half)
    sy = sy + bead_slope[:, None]
    # vertical (longitudinal) seam at u = 0: the stainless sheets of a ring meet there (one per period = 4.71 m of arc);
    # the shader shifts u per ring, so the seams stagger from ring to ring like the real panels (photos: faint vertical lines)
    u = ((np.arange(SIZE_U) + 0.5) * du + 0.5 * pu) % pu - 0.5 * pu
    seam_half, seam_height = half, SEAM_FRACTION * height
    sx = sx - (np.sign(u) * (np.abs(u) < seam_half) * (seam_height / seam_half))[None, :]
    haz_half = HAZ_FACTOR * half
    haz_v = np.clip((haz_half - np.abs(v)) / dv + 0.5, 0.0, 1.0)  # box-filtered at the texel
    haz_u = np.clip((HAZ_FACTOR * seam_half - np.abs(u)) / du + 0.5, 0.0, 1.0)
    haz = np.maximum(haz_v[:, None], SEAM_HAZ_SHARE * haz_u[None, :])
    # dotted stringer lines (spot-weld rows): at every multiple of period / STRINGERS_PER_TILE; the shader shifts rings by
    # whole stringers, so the lines run on across the rings as in the photos
    s_pitch = pu / STRINGERS_PER_TILE
    rng = np.random.default_rng(SEED + 7)
    dots = np.zeros((SIZE_V, SIZE_U))
    u_all = (np.arange(SIZE_U) + 0.5) * du
    v_all = (np.arange(SIZE_V) + 0.5) * dv
    # frames: dotted rows come in irregular groups with bare sheet between them
    for line in rng.choice(STRINGERS_PER_TILE, round(DOT_LINE_SHARE * STRINGERS_PER_TILE), replace=False):
        n_dots = max(1, round(pv / (DOT_PITCH_M * rng.uniform(0.8, 1.25))))
        d_pitch = pv / n_dots
        phase = rng.uniform(0.0, d_pitch)
        keep = rng.random(n_dots) > DOT_DROPOUT
        du_line = (u_all - (line + 0.5) * s_pitch + 0.5 * pu) % pu - 0.5 * pu
        columns = np.abs(du_line) < 2.0 * DOT_RADIUS_M
        k = np.floor((v_all - phase) / d_pitch + 0.5).astype(int)
        vs = v_all - phase - k * d_pitch
        field = np.clip((DOT_RADIUS_M - np.hypot(du_line[columns][None, :], vs[:, None])) / du + 0.5, 0.0, 1.0)
        dots[:, columns] = np.maximum(dots[:, columns], field * keep[k % n_dots][:, None])
    haz = np.maximum(haz, DOT_COVERAGE * dots)
    energy = sx * sx + sy * sy
    out = np.empty((SIZE_V, SIZE_U, 4))
    out[..., 0] = 0.5 + sx / SLOPE_RANGE
    out[..., 1] = 0.5 + sy / SLOPE_RANGE
    out[..., 2] = energy / SLOPE_ENERGY_RANGE
    out[..., 3] = haz
    if out[..., :3].min() < 0.0 or out[..., :3].max() > 1.0:
        raise ValueError("steel map: slope exceeds the stored range")
    return np.rint(out * 255.0).astype(np.uint8)


def bake_flap() -> np.ndarray:
    """Panel lines of the flaps' bare lee face, same channels as the hull map, not tiling: columns, rows, two stiffeners."""
    w, h = FLAP_SIZE
    du, dv = FLAP_PERIOD_M[0] / w, FLAP_PERIOD_M[1] / h
    u = (np.arange(w) + 0.5) * du
    v = (np.arange(h) + 0.5) * dv
    uu, vv = np.meshgrid(u, v)
    col = (uu + FLAP_COLUMN_M / 2.0) % FLAP_COLUMN_M - FLAP_COLUMN_M / 2.0
    row = (vv + FLAP_ROW_M / 2.0) % FLAP_ROW_M - FLAP_ROW_M / 2.0
    on_col, on_row = np.abs(col) < FLAP_LINE_HALF_M, np.abs(row) < FLAP_LINE_HALF_M
    sx = -np.sign(col) * on_col * FLAP_SLOPE
    sy = -np.sign(row) * on_row * FLAP_SLOPE
    line = np.maximum(on_col, on_row).astype(float) * 0.6
    for (u0, v0), (u1, v1) in FLAP_DIAGONALS_M:
        eu, ev = u1 - u0, v1 - v0
        length = math.hypot(eu, ev)
        dist = ((uu - u0) * ev - (vv - v0) * eu) / length
        along = ((uu - u0) * eu + (vv - v0) * ev) / length
        on = (np.abs(dist) < 2.0 * FLAP_LINE_HALF_M) & (along > 0.0) & (along < length)
        sx = sx + np.sign(dist) * on * FLAP_SLOPE * ev / length
        sy = sy - np.sign(dist) * on * FLAP_SLOPE * eu / length
        line = np.maximum(line, on * 0.8)
    out = np.empty((h, w, 4))
    out[..., 0] = 0.5 + sx / SLOPE_RANGE
    out[..., 1] = 0.5 + sy / SLOPE_RANGE
    out[..., 2] = (sx * sx + sy * sy) / SLOPE_ENERGY_RANGE
    out[..., 3] = line
    return np.rint(np.clip(out, 0.0, 1.0) * 255.0).astype(np.uint8)


def png_bytes(rgba: np.ndarray) -> bytes:
    """8-bit RGBA PNG (filter 0, zlib level 9): byte-identical for identical pixels."""
    h, w, _ = rgba.shape
    raw = b"".join(b"\x00" + rgba[y].tobytes() for y in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def write(out_dir, spec: dict) -> dict:
    """Bakes the map for the vehicle's spec into `out_dir`; returns the manifest's texture entry."""
    radii = {st["diameter_m"] / 2.0 for st in spec["stages"].values()}
    if len(radii) != 1:
        raise ValueError(f"steel map: one hull radius expected (the map closes around it), got {sorted(radii)}")
    radius = radii.pop()
    hull = spec["detail"]["hull"]
    (out_dir / FILE_NAME).write_bytes(png_bytes(bake(radius, hull)))
    pu, pv = periods_m(radius, hull["ring_pitch_m"])
    (out_dir / FLAP_FILE_NAME).write_bytes(png_bytes(bake_flap()))
    return {"file": FILE_NAME, "flap_file": FLAP_FILE_NAME, "flap_period_m": list(FLAP_PERIOD_M),
            "shift_steps": STRINGERS_PER_TILE, "period_u_m": pu, "period_v_m": pv, "size": [SIZE_U, SIZE_V], "slope_range": SLOPE_RANGE,
            "slope_energy_range": SLOPE_ENERGY_RANGE, "texel_m": [pu / SIZE_U, pv / SIZE_V]}
