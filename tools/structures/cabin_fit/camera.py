"""Photo cameras for the cabin fit: pinhole and equidistant fisheye, in the cabin frame.

Convention (same as config/cockpits/photo_cameras.toml): `forward` and `up` are unit vectors in the cabin frame,
image right = forward x up (right-handed). `rot` maps cabin-frame vectors to camera axes (rows: right, down, forward);
pixel u runs right, v runs down. Pinhole: u = cx + f x/z. Fisheye: r = f (theta + k1 theta^3 + k2 theta^5 ...).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

import numpy as np


def rot_from_forward_up(forward, up) -> np.ndarray:
    f = np.asarray(forward, float)
    f = f / np.linalg.norm(f)
    u = np.asarray(up, float)
    u = u - f * np.dot(u, f)  # make up orthogonal to forward
    u = u / np.linalg.norm(u)
    right = np.cross(f, u)
    return np.vstack([right, -u, f])


def rotvec_to_matrix(v) -> np.ndarray:
    v = np.asarray(v, float)
    th = np.linalg.norm(v)
    if th < 1e-14:
        return np.eye(3) + _skew(v)
    k = v / th
    kx = _skew(k)
    return np.eye(3) + math.sin(th) * kx + (1 - math.cos(th)) * (kx @ kx)


def _skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


@dataclass(frozen=True)
class Camera:
    position_m: np.ndarray
    rot: np.ndarray
    f_px: float
    cx: float
    cy: float
    model: str = "pinhole"  # "pinhole" | "fisheye_equidistant"
    k: tuple = ()
    size_px: tuple = (0, 0)
    id: str = ""
    file: str = ""
    extra: dict = field(default_factory=dict, compare=False)

    @property
    def forward(self) -> np.ndarray:
        return self.rot[2]

    @property
    def up(self) -> np.ndarray:
        return -self.rot[1]

    @staticmethod
    def from_row(row: dict) -> "Camera":
        w, h = row["size_px"]
        fov = math.radians(float(row["fov_h_deg"]))
        model = row.get("model", "pinhole")
        if model == "pinhole":
            f = (w / 2) / math.tan(fov / 2)
        elif model == "fisheye_equidistant":
            f = (w / 2) / (fov / 2)
        else:
            raise ValueError(f"unknown camera model {model!r}")
        return Camera(np.asarray(row["position_m"], float), rot_from_forward_up(row["forward"], row["up"]), f,
                      w / 2, h / 2, model, tuple(row.get("k", ())), (int(w), int(h)), row.get("id", ""), row.get("file", ""))

    def to_row(self) -> dict:
        w, h = self.size_px
        if not w:
            w, h = int(round(2 * self.cx)), int(round(2 * self.cy))
        if self.model == "pinhole":
            fov = 2 * math.atan((w / 2) / self.f_px)
        else:
            fov = 2 * (w / 2) / self.f_px
        row = {"id": self.id, "file": self.file, "size_px": [int(w), int(h)],
               "position_m": [float(x) for x in self.position_m],
               "forward": [float(x) for x in self.forward], "up": [float(x) for x in self.up],
               "fov_h_deg": math.degrees(fov), "model": self.model}
        if self.k:
            row["k"] = [float(x) for x in self.k]
        return row

    def with_(self, **kw) -> "Camera":
        return replace(self, **kw)


def _theta_poly(theta, k):
    s = theta.copy()
    for i, ki in enumerate(k):
        s = s + ki * theta ** (3 + 2 * i)
    return s


def project(points_m: np.ndarray, cam: Camera) -> np.ndarray:
    """Project (N,3) cabin-frame points to (N,2) pixels. Points behind a pinhole camera give nan."""
    p = (np.atleast_2d(np.asarray(points_m, float)) - cam.position_m) @ cam.rot.T
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    if cam.model == "pinhole":
        with np.errstate(divide="ignore", invalid="ignore"):
            zz = np.where(z > 1e-9, z, np.nan)
            return np.stack([cam.cx + cam.f_px * x / zz, cam.cy + cam.f_px * y / zz], axis=1)
    rho = np.hypot(x, y)
    theta = np.arctan2(rho, z)
    r = cam.f_px * _theta_poly(theta, cam.k)
    with np.errstate(divide="ignore", invalid="ignore"):
        s = np.where(rho > 1e-15, r / rho, cam.f_px)  # on the axis r/rho -> f (theta/rho -> 1/z)
    return np.stack([cam.cx + s * x, cam.cy + s * y], axis=1)


def unproject(px: np.ndarray, cam: Camera) -> np.ndarray:
    """Pixels (N,2) -> unit ray directions (N,3) in the cabin frame."""
    px = np.atleast_2d(np.asarray(px, float))
    dx, dy = px[:, 0] - cam.cx, px[:, 1] - cam.cy
    if cam.model == "pinhole":
        d = np.stack([dx / cam.f_px, dy / cam.f_px, np.ones_like(dx)], axis=1)
    else:
        r = np.hypot(dx, dy)
        target = r / cam.f_px
        theta = target.copy()
        for _ in range(50):  # Newton on theta + k1 theta^3 + ... = r / f
            g = _theta_poly(theta, cam.k) - target
            dg = np.ones_like(theta)
            for i, ki in enumerate(cam.k):
                dg = dg + ki * (3 + 2 * i) * theta ** (2 + 2 * i)
            step = g / dg
            theta = theta - step
            if np.max(np.abs(step)) < 1e-15:
                break
        with np.errstate(invalid="ignore", divide="ignore"):
            ux = np.where(r > 1e-15, dx / r, 0.0)
            uy = np.where(r > 1e-15, dy / r, 0.0)
        d = np.stack([np.sin(theta) * ux, np.sin(theta) * uy, np.cos(theta)], axis=1)
    d = d / np.linalg.norm(d, axis=1, keepdims=True)
    return d @ cam.rot  # camera -> cabin frame (rot is orthonormal)
