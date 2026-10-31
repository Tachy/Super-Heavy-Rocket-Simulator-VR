"""Procedural panel map of the Pad B mount's shell (tan steel panels in horizontal courses), no photo pixels.

One RGB8 PNG, `mount_panels.png`, tiling in both axes: `tile_m` x `tile_m` metres of the shell's UV (the shell shader
repeats it). Courses of `course_m` run horizontally, panels `panel_m` wide in running bond (every other course offset by
half a panel), `seam_m` dark seams, a per-panel tone, weld haze beside the seams, vertical dirt streaks below the seams
and low-frequency stains. Deterministic (fixed seed, fixed zlib level). sRGB data (the shader samples `source_color`).
"""
from __future__ import annotations

import numpy as np

import hull_textures

SEED = 20261031
TAN = np.array([0.50, 0.44, 0.35])      # [E] sRGB of the clean panel: tan / gold-beige steel (pyramid photos 2026-04, 2025-09)
SEAM_DARKEN = 0.70                       # [E] seam colour as a share of the panel
TONE_SPREAD = 0.07                       # [E] panel-to-panel brightness spread (+-)
STREAK_DARKEN = 0.22                     # [E] strongest dirt streak
STAIN_DARKEN = 0.12                      # [E] strongest blotch
FILE_NAME = "mount_panels.png"


def bake(m: dict) -> np.ndarray:
    size = int(m["map_size"])
    tile, course, panel, seam = m["tile_m"], m["course_m"], m["panel_m"], m["seam_m"]
    rng = np.random.default_rng(SEED)
    px = tile / size
    u = (np.arange(size) + 0.5) * px  # along the face (columns)
    v = (np.arange(size) + 0.5) * px  # along the slope (rows)
    row = np.floor(v / course).astype(int)
    stagger = (row % 2) * 0.5 * panel
    uu = (u[None, :] + stagger[:, None]) % tile
    col = np.floor(uu / panel).astype(int)
    courses = int(round(tile / course))
    panels = int(round(tile / panel))
    tone = 1.0 + TONE_SPREAD * (2.0 * rng.random((courses, panels)) - 1.0)
    img = np.empty((size, size, 3))
    img[:] = TAN
    img *= tone[row[:, None] % courses, col % panels][:, :, None]
    # seams: distance to the course boundary and to the panel edge
    dv = np.minimum(v % course, course - v % course)[:, None]
    du = np.minimum(uu % panel, panel - uu % panel)
    half = 0.5 * seam
    line = np.clip((half - np.minimum(dv, du)) / px + 0.5, 0.0, 1.0)
    haze = np.clip((3.0 * half - np.minimum(dv, du)) / (3.0 * half), 0.0, 1.0)
    img *= (1.0 - 0.06 * haze)[:, :, None]  # heat-affected band beside the seam
    img = img * (1.0 - line[:, :, None]) + img * SEAM_DARKEN * line[:, :, None]
    # dirt streaks below the horizontal seams: stretched noise along v, strongest just under a course boundary
    streak = hull_textures._periodic_noise(rng, 48, 3, size, size)
    below = np.exp(-((v % course) / (0.35 * course))[:, None])
    img *= (1.0 - STREAK_DARKEN * np.clip(streak * 1.6 - 0.5, 0.0, 1.0) * below)[:, :, None]
    stain = hull_textures._periodic_noise(rng, 5, 5, size, size) * 0.6 + hull_textures._periodic_noise(rng, 13, 13, size, size) * 0.4
    img *= (1.0 - STAIN_DARKEN * stain)[:, :, None]
    return (np.clip(img, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8)


def write(out_dir, detail: dict) -> dict:
    rgb = bake(detail["mount"])
    rgba = np.concatenate([rgb, np.full(rgb.shape[:2] + (1,), 255, np.uint8)], axis=2)
    (out_dir / FILE_NAME).write_bytes(hull_textures.png_bytes(rgba))
    return {"file": FILE_NAME, "tile_m": detail["mount"]["tile_m"]}
