"""Per-photo annotations: tools/structures/cabin_fit/annotations/<photo_id>.toml.

    sigma_px = 1.5                      # optional, default 1.0
    [[point]]
    label = "top_left_corner"
    px = [412.0, 230.5]
    [[edge]]
    label = "front_top"                 # an edge the PartModel lists in edges()
    px = [[400.0, 100.0], [500.0, 104.0], [600.0, 110.0]]   # points along the visible edge
"""
from __future__ import annotations

import tomllib
from dataclasses import dataclass, field
from pathlib import Path

from .camera import Camera

HERE = Path(__file__).resolve().parent
ANNOTATION_DIR = HERE / "annotations"
CAMERA_FILE = HERE.parents[2] / "config" / "cockpits" / "photo_cameras.toml"


@dataclass
class PhotoObs:
    id: str
    camera: Camera                     # initial guess (a fitted copy comes back in FitResult)
    points: dict = field(default_factory=dict)   # label -> (u, v)
    edges: dict = field(default_factory=dict)    # label -> [(u, v), ...]
    sigma_px: float = 1.0
    fit_k: bool = False                # fisheye only: also fit k1
    camera_sigma: tuple | None = None  # optional prior (position_m, angle_rad, f_rel) around the initial camera


def camera_row(photo_id: str, camera_file: Path | None = None) -> dict:
    with open(camera_file or CAMERA_FILE, "rb") as fh:
        rows = tomllib.load(fh).get("photo", [])
    for row in rows:
        if row["id"] == photo_id:
            return row
    raise KeyError(f"photo {photo_id!r} not in {camera_file or CAMERA_FILE}")


def load(photo_id: str, annotation_dir: Path | None = None, camera_file: Path | None = None) -> PhotoObs:
    path = (annotation_dir or ANNOTATION_DIR) / f"{photo_id}.toml"
    with open(path, "rb") as fh:
        data = tomllib.load(fh)
    cam = Camera.from_row(camera_row(photo_id, camera_file))
    pts = {p["label"]: (float(p["px"][0]), float(p["px"][1])) for p in data.get("point", [])}
    edges = {e["label"]: [(float(u), float(v)) for u, v in e["px"]] for e in data.get("edge", [])}
    return PhotoObs(photo_id, cam, pts, edges, float(data.get("sigma_px", 1.0)))
