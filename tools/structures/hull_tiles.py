"""Heat shield tile maps of the vehicle hulls (flat-topped hexagon tiles, a gap filler between them, a rounded edge), baked
from the detail file's numbers; procedural, no photo pixels, deterministic (fixed seed, PNG filter 0 / zlib level 9).

The shader decides WHERE tiles are (the coverage table around the belly, the flaps' outer faces); these maps decide how they
look. Three RGBA8 PNGs, all in units of the lattice period (1 x sqrt3 across-flats = two tiles):
    tile_patch  TILE_SIZE_U x TILE_SIZE_V, tiling, MIPMAPPED (the game builds the chain: box averages, which the channels are
                designed for) and sampled anisotropically:
                R, G  bevel tilt of the normal along the arc / the station axis, 0.5 + tilt (+-0.5)
                B     tilt energy tx^2 + ty^2 over TILE_ENERGY_RANGE; the shader's variance term B - |mean tilt|^2 widens
                      the roughness on the coarser mips (as for the steel map)
                A     coverage of the gap filler (box filtered by TILE_SUPERSAMPLE^2 samples per texel)
    tile_sel    half the patch size, NEAREST, no mips: which tile a texel belongs to (R, G: the id cell's offset from the
                period cell, B: 255 = the second lattice)
    tile_ids    IDS_U x IDS_V period cells, NEAREST, no mips: R / G the shade of the first / second tile of a cell
                (shader: 0.75 + 0.5 x value), B / A its "repaired" flag. Ids cannot be averaged: the shader fades the
                per-tile variation to its mean by the pixel footprint; the tile shape itself is mipmapped.
"""
from __future__ import annotations

import math

import numpy as np

TILE_SIZE_U = 256          # [E] texels across a tile's flats: 1.17 mm at 0.30 m, finer than a VR pixel at 5 m (1.7 mm)
TILE_SIZE_V = 512          # [E] texels along sqrt3 x across flats (0.52 m): 1.01 mm
TILE_SUPERSAMPLE = 4
TILE_BEVEL_M = 0.012       # [E] width of the rounded edge of a tile
TILE_BEVEL_TILT = 0.4      # [E] normal tilt at the edge of a tile
TILE_ENERGY_RANGE = TILE_BEVEL_TILT ** 2
IDS_U, IDS_V = 128, 64     # [E] period cells of the id map: 38 m x 33 m before the shade pattern repeats
REPAIRED_FRACTION = 0.006  # [E] share of lighter, repaired tiles
SEED = 20241101
FILES = {"patch": "tile_patch.png", "sel": "tile_sel.png", "ids": "tile_ids.png"}
SQRT3 = math.sqrt(3.0)
FLAT_NORMALS = ((1.0, 0.0), (0.5, SQRT3 / 2.0), (-0.5, SQRT3 / 2.0))


def hex_cells(x, y):
    """Points (x, y) of the period cell (units of across flats): the nearest tile's offset (gx, gy) from its centre, the id
    cell's offset (dx, dy) from the period cell and the second lattice's flag. The first tile's centre is the cell's
    middle, the second's its corners (shared by four cells: the id cell is the one the corner is the lower left of)."""
    hx, hy = 0.5, SQRT3 / 2.0
    ax, ay = x % 1.0 - hx, y % SQRT3 - hy
    cell_x, cell_y = np.floor(x - hx), np.floor((y - hy) / SQRT3)
    bx, by = x - hx - cell_x - hx, y - hy - cell_y * SQRT3 - hy
    second = ax * ax + ay * ay >= bx * bx + by * by
    gx, gy = np.where(second, bx, ax), np.where(second, by, ay)
    dx = np.where(second, cell_x + 1.0, 0.0).astype(int)
    dy = np.where(second, cell_y + 1.0, 0.0).astype(int)
    return gx, gy, dx, dy, second


def bake_patch(across_flats_m: float, gap_m: float) -> np.ndarray:
    """(TILE_SIZE_V, TILE_SIZE_U, 4) uint8, row = station axis, column = arc axis."""
    n, w, h = TILE_SUPERSAMPLE, TILE_SIZE_U, TILE_SIZE_V
    xx, yy = np.meshgrid((np.arange(w * n) + 0.5) / (w * n), (np.arange(h * n) + 0.5) / (h * n) * SQRT3)
    gx, gy, _, _, _ = hex_cells(xx, yy)
    dots = np.stack([gx * a + gy * b for a, b in FLAT_NORMALS])
    k = np.argmax(np.abs(dots), axis=0)
    best = np.take_along_axis(dots, k[None], 0)[0]
    flats, sign = np.abs(best), np.sign(best)
    nx = np.array([a for a, _ in FLAT_NORMALS])[k] * sign
    ny = np.array([b for _, b in FLAT_NORMALS])[k] * sign
    edge_m = (0.5 - flats) * across_flats_m
    gap_half = 0.5 * gap_m
    gap = (edge_m < gap_half).astype(float)
    t = np.clip((edge_m - gap_half) / TILE_BEVEL_M, 0.0, 1.0)
    bevel = (1.0 - t * t * (3.0 - 2.0 * t)) * (1.0 - gap)
    tx, ty = TILE_BEVEL_TILT * bevel * nx, TILE_BEVEL_TILT * bevel * ny

    def box(a):
        return a.reshape(h, n, w, n).mean(axis=(1, 3))

    out = np.empty((h, w, 4))
    out[..., 0] = 0.5 + box(tx)
    out[..., 1] = 0.5 + box(ty)
    out[..., 2] = box(tx * tx + ty * ty) / TILE_ENERGY_RANGE
    out[..., 3] = box(gap)
    return np.rint(out * 255.0).astype(np.uint8)


def bake_sel() -> np.ndarray:
    w, h = TILE_SIZE_U // 2, TILE_SIZE_V // 2
    xx, yy = np.meshgrid((np.arange(w) + 0.5) / w, (np.arange(h) + 0.5) / h * SQRT3)
    _, _, dx, dy, second = hex_cells(xx, yy)
    out = np.zeros((h, w, 4), dtype=np.uint8)
    out[..., 0], out[..., 1], out[..., 2], out[..., 3] = dx * 255, dy * 255, second * 255, 255
    return out


def bake_ids() -> np.ndarray:
    rng = np.random.default_rng(SEED)
    out = np.empty((IDS_V, IDS_U, 4), dtype=np.uint8)
    out[..., 0] = rng.integers(0, 256, (IDS_V, IDS_U))
    out[..., 1] = rng.integers(0, 256, (IDS_V, IDS_U))
    out[..., 2] = (rng.random((IDS_V, IDS_U)) < REPAIRED_FRACTION) * 255
    out[..., 3] = (rng.random((IDS_V, IDS_U)) < REPAIRED_FRACTION) * 255
    return out


def write(out_dir, spec: dict, png_bytes) -> dict:
    """Bakes the maps for the vehicle's spec into `out_dir`; returns the manifest's entry."""
    tiles = spec["detail"]["ship"]["tiles"]
    af = tiles["hex_across_flats_m"]
    (out_dir / FILES["patch"]).write_bytes(png_bytes(bake_patch(af, tiles["gap_m"])))
    (out_dir / FILES["sel"]).write_bytes(png_bytes(bake_sel()))
    (out_dir / FILES["ids"]).write_bytes(png_bytes(bake_ids()))
    return {"files": dict(FILES), "period_m": [af, af * SQRT3], "ids_size": [IDS_U, IDS_V],
            "tilt_energy_range": TILE_ENERGY_RANGE, "patch_size": [TILE_SIZE_U, TILE_SIZE_V],
            "texel_m": [af / TILE_SIZE_U, af * SQRT3 / TILE_SIZE_V]}
