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
HAZ_FACTOR = 1.2          # [E] heat-affected band half width / bead half width
DENT_OCTAVES = ((4, 2, 1.0), (12, 5, 1.085))  # [E] (cells along u, cells along v, weight): ~1.2 m and ~0.38 m waves
SEED = 20241031
FILE_NAME = "hull_steel.png"


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
    haz_half = HAZ_FACTOR * half
    haz = np.clip((haz_half - np.abs(v)) / dv + 0.5, 0.0, 1.0)  # box-filtered at the texel
    energy = sx * sx + sy * sy
    out = np.empty((SIZE_V, SIZE_U, 4))
    out[..., 0] = 0.5 + sx / SLOPE_RANGE
    out[..., 1] = 0.5 + sy / SLOPE_RANGE
    out[..., 2] = energy / SLOPE_ENERGY_RANGE
    out[..., 3] = np.broadcast_to(haz[:, None], (SIZE_V, SIZE_U))
    if out[..., :3].min() < 0.0 or out[..., :3].max() > 1.0:
        raise ValueError("steel map: slope exceeds the stored range")
    return np.rint(out * 255.0).astype(np.uint8)


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
    return {"file": FILE_NAME, "period_u_m": pu, "period_v_m": pv, "size": [SIZE_U, SIZE_V], "slope_range": SLOPE_RANGE,
            "slope_energy_range": SLOPE_ENERGY_RANGE, "texel_m": [pu / SIZE_U, pv / SIZE_V]}
