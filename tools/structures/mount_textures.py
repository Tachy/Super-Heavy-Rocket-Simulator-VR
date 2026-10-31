"""Procedural maps of the Pad B site, no photo pixels; one RGB8 PNG each, tiling in both axes.

`mount_panels.png`: the mount's plated faces (`tile_m` x `tile_m` of UV): large flat plates, `course_m` high and `panel_m`
wide, stacked without an offset (no brick bond), rust-tan seams, per-plate tone, staining and streaks below the seams.
`pad_concrete.png`: the hardstand and ramps (`[trench]`): cast slabs with dark joints, tyre marks and stains.
Deterministic (fixed seed, fixed zlib level). sRGB data (the shader samples `source_color`).
"""
from __future__ import annotations

import numpy as np

import hull_textures

SEED = 20261031
TAN = np.array([0.62, 0.58, 0.50])       # [E] sRGB plate, grey to tan: between the sunlit person-scale frame and images.jpeg
SEAM = np.array([0.47, 0.40, 0.32])      # [E] sRGB rust-tan seam line, person-scale frame
TONE_SPREAD = 0.09                       # [E] plate-to-plate brightness spread (+-)
GREY_SHIFT = 0.10                        # [E] share of plates with the tan pulled to grey, images.jpeg
STREAK_DARKEN = 0.20                     # [E] strongest dirt streak
STAIN_DARKEN = 0.18                      # [E] strongest blotch
CONCRETE = np.array([0.73, 0.73, 0.71])  # [E] sRGB sunlit hardstand, images.jpeg and padb_aerial_ship39_on_mount_2026-04
JOINT_DARKEN = 0.86                      # [E] slab joint as a share of the slab
SLAB_SPREAD = 0.05                       # [E] slab-to-slab brightness spread (+-)
FILE_NAME = "mount_panels.png"
CONCRETE_FILE_NAME = "pad_concrete.png"


def bake(m: dict) -> np.ndarray:
    size = int(m["map_size"])
    tile, course, panel, seam = m["tile_m"], m["course_m"], m["panel_m"], m["seam_m"]
    rng = np.random.default_rng(SEED)
    px = tile / size
    u = (np.arange(size) + 0.5) * px  # along the face (columns)
    v = (np.arange(size) + 0.5) * px  # along the slope (rows)
    courses, panels = int(round(tile / course)), int(round(tile / panel))
    row, col = np.floor(v / course).astype(int) % courses, np.floor(u / panel).astype(int) % panels
    tone = 1.0 + TONE_SPREAD * (2.0 * rng.random((courses, panels)) - 1.0)
    grey = rng.random((courses, panels)) < GREY_SHIFT
    img = np.empty((size, size, 3))
    img[:] = TAN
    img[grey[row[:, None], col[None, :]]] = 0.5 * (TAN + TAN.mean())
    img *= tone[row[:, None], col[None, :]][:, :, None]
    # staining first, so that the seams stay on top
    stain = hull_textures._periodic_noise(rng, 5, 5, size, size) * 0.6 + hull_textures._periodic_noise(rng, 13, 13, size, size) * 0.4
    img *= (1.0 - STAIN_DARKEN * stain)[:, :, None]
    streak = hull_textures._periodic_noise(rng, 48, 3, size, size)
    below = np.exp(-((v % course) / (0.45 * course))[:, None])
    img *= (1.0 - STREAK_DARKEN * np.clip(streak * 1.6 - 0.5, 0.0, 1.0) * below)[:, :, None]
    # rust-tan seams: distance to the course boundary and to the plate edge
    dv = np.minimum(v % course, course - v % course)[:, None] * np.ones((1, size))
    du = np.minimum(u % panel, panel - u % panel)[None, :] * np.ones((size, 1))
    line = np.clip((0.5 * seam - np.minimum(dv, du)) / px + 0.5, 0.0, 1.0)[:, :, None]
    img = img * (1.0 - line) + SEAM * line
    return (np.clip(img, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8)


def bake_concrete(t: dict) -> np.ndarray:
    size = int(t["concrete_map_size"])
    tile, slab, joint = t["concrete_tile_m"], t["slab_m"], t["joint_m"]
    rng = np.random.default_rng(SEED + 1)
    px = tile / size
    c = (np.arange(size) + 0.5) * px
    n = int(round(tile / slab))
    idx = np.floor(c / slab).astype(int) % n
    tone = 1.0 + SLAB_SPREAD * (2.0 * rng.random((n, n)) - 1.0)
    img = np.empty((size, size, 3))
    img[:] = CONCRETE
    img *= tone[idx[:, None], idx[None, :]][:, :, None]
    stain = hull_textures._periodic_noise(rng, 7, 7, size, size) * 0.6 + hull_textures._periodic_noise(rng, 23, 23, size, size) * 0.4
    img *= (1.0 - STAIN_DARKEN * stain)[:, :, None]
    d = np.minimum(c % slab, slab - c % slab)
    near = np.minimum(d[:, None], d[None, :])
    line = np.clip((0.5 * joint - near) / px + 0.5, 0.0, 1.0)[:, :, None]
    img = img * (1.0 - line) + img * JOINT_DARKEN * line
    return (np.clip(img, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8)


def _write_png(path, rgb: np.ndarray) -> None:
    rgba = np.concatenate([rgb, np.full(rgb.shape[:2] + (1,), 255, np.uint8)], axis=2)
    path.write_bytes(hull_textures.png_bytes(rgba))


def write(out_dir, detail: dict) -> dict:
    _write_png(out_dir / FILE_NAME, bake(detail["mount"]))
    _write_png(out_dir / CONCRETE_FILE_NAME, bake_concrete(detail["trench"]))
    return {"file": FILE_NAME, "tile_m": detail["mount"]["tile_m"], "concrete_file": CONCRETE_FILE_NAME,
            "concrete_tile_m": detail["trench"]["concrete_tile_m"]}
