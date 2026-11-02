"""Baked albedo maps of the ISS (numpy, deterministic, no photo pixels): the silhouette is the mesh, the fine structure is here. Written by the driver into data/models/iss/.
The game builds the mip chains (box averages) and samples with trilinear + anisotropic filtering; UVs are metres, each map repeats over its period (manifest `look.textured`).

array_cells.png   legacy blanket: one bay of cell strings (light frame lines, per-string shade)
irosa_cells.png   iROSA: finer, darker cells
panel_skin.png    module skin: panel lines and small shade differences (Whipple shields, MLI patches)
radiator_ribs.png radiator: the panels' cross seams
truss? no: bare paint.
"""
from __future__ import annotations

import struct
import zlib
from pathlib import Path

import numpy as np

ARRAY_PERIOD_M = (2.8, 2.14)  # [E] I_The_International...: about 12 bays over the 35 m blanket, 2 cell columns over its 4.28 m width
IROSA_PERIOD_M = (2.4, 1.5)  # [E] D_ photos: the roll-out blanket's strings; no close photo of an iROSA in the repo
PANEL_PERIOD_M = (3.46, 3.0)  # [E] D_Crew_Dragon_Endeavour: Whipple panels about 1.1 m around, 1.5 m long, here two by two on one map
RADIATOR_PERIOD_M = (2.0, 2.0)  # [E] station photos: seams about every 2 m along the panel
SEED = 20261102


def png_bytes(image: np.ndarray) -> bytes:
    h, w, c = image.shape
    raw = b"".join(b"\x00" + image[y].tobytes() for y in range(h))
    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
    color = {1: 0, 3: 2, 4: 6}[c]
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, color, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")


def _lines(n, pitch, width, offset=0.0):
    """1 at a line (period `pitch` texels), 0 else."""
    x = (np.arange(n) + 0.5 - offset) % pitch
    return (np.minimum(x, pitch - x) < width).astype(float)


def array_cells(rng) -> np.ndarray:
    w, h = 512, 512
    shade = rng.uniform(0.82, 1.15, size=(8, 8))
    img = np.kron(shade, np.ones((h // 8, w // 8)))
    img *= rng.uniform(0.96, 1.04, size=(h, w)) ** 0.5
    cols, rows = _lines(w, w / 16, 0.9), _lines(h, h / 16, 0.9)
    img = img * (1.0 - 0.25 * np.maximum(cols[None, :], rows[:, None]))
    bay, frame = _lines(w, w, 3.0), _lines(h, h, 3.0)  # the bay's frame lines: gold-coloured in the photos
    rgb = np.dstack([img, img, img])
    gold = np.maximum(bay[None, :], frame[:, None])[..., None]
    rgb = rgb * (1.0 - gold) + gold * np.array([1.9, 1.5, 0.9])
    return np.clip(rgb * 128.0, 0, 255).astype(np.uint8)


def irosa_cells(rng) -> np.ndarray:
    w, h = 512, 256
    shade = rng.uniform(0.85, 1.12, size=(16, 32))
    img = np.kron(shade, np.ones((h // 16, w // 32)))
    cols, rows = _lines(w, w / 32, 0.8), _lines(h, h / 16, 0.8)
    img = img * (1.0 - 0.45 * np.maximum(cols[None, :], rows[:, None]))
    return np.clip(np.dstack([img, img, img]) * 150.0, 0, 255).astype(np.uint8)


def panel_skin(rng) -> np.ndarray:
    w, h = 512, 512
    shade = rng.uniform(0.97, 1.03, size=(4, 4))
    img = np.kron(shade, np.ones((h // 4, w // 4)))
    img *= 1.0 + 0.012 * rng.standard_normal(size=(h, w))
    cols, rows = _lines(w, w / 4, 1.0), _lines(h, h / 4, 1.0)
    img = img * (1.0 - 0.16 * np.maximum(cols[None, :], rows[:, None]))
    return np.clip(np.dstack([img, img, img]) * 200.0, 0, 255).astype(np.uint8)


def radiator_ribs(rng) -> np.ndarray:
    w, h = 64, 256
    img = np.ones((h, w))
    img *= 1.0 - 0.3 * _lines(h, h / 2, 1.2)[:, None]
    return np.clip(np.dstack([img, img, img]) * 240.0, 0, 255).astype(np.uint8)


def write(out: Path) -> dict:
    """Writes the maps; returns {paint key: {file, period_m}} for the manifest."""
    rng = np.random.default_rng(SEED)
    maps = {"paint_array": ("array_cells.png", array_cells(rng), ARRAY_PERIOD_M), "paint_irosa": ("irosa_cells.png", irosa_cells(rng), IROSA_PERIOD_M),
            "paint_us": ("panel_skin.png", panel_skin(rng), PANEL_PERIOD_M), "paint_radiator": ("radiator_ribs.png", radiator_ribs(rng), RADIATOR_PERIOD_M)}
    result = {}
    for key, (name, image, period) in maps.items():
        (Path(out) / name).write_bytes(png_bytes(image))
        result[key] = {"file": name, "period_m": list(period)}
    result["paint_ru"] = {"file": "panel_skin.png", "period_m": [p * 0.8 for p in PANEL_PERIOD_M]}  # [E] the same map, MLI patches a little smaller
    return result
