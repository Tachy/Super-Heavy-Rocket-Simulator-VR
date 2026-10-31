"""Camera solve of a reference image (spec A): pose from identified points of known 3D position (OpenCV solvePnP).

Points file (TOML, next to the image in the git-ignored reference folder), 3D frame chosen by the measurer
(x right, y toward the viewer, z up, metres; e.g. origin at the booster axis on the mount deck):

    image = "padb_full_stack_on_mount_2026-05.jpg"
    [camera]
    width_px = 2560
    height_px = 1708
    focal_px = 5000.0              # known, or scanned with "focal_scan_px"
    [[point]]
    name = "booster_left_base"
    xyz_m = [-4.5, 0.0, 0.0]
    uv_px = [1270.0, 1290.0]       # ORIGINAL image pixel coordinates
    ...
    [[measure]]                    # optional: a 3D length / point recovered from a pixel pair on a known plane
    name = "tower_top_z_m"
    uv_px = [1100.0, 95.0]
    plane = "y"                    # intersect the pixel ray with the plane y = 0 (or "x" / "z")
    value = 0.0

Usage: python camera_solve.py <points.toml>   -> pose, reprojection error, every [[measure]] result; with
"focal_scan_px = [lo, hi]" the focal length with the smallest reprojection error is used and the spread is reported.
Principal point is the image centre (photos are not cropped off-centre as far as known; the error budget says so).
"""
from __future__ import annotations

import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass
class Pose:
    rvec: np.ndarray
    tvec: np.ndarray
    camera_matrix: np.ndarray
    reprojection_rms_px: float

    @property
    def rotation(self) -> np.ndarray:
        return cv2.Rodrigues(self.rvec)[0]

    @property
    def camera_position_m(self) -> np.ndarray:
        """Camera centre in the 3D frame of the points."""
        return (-self.rotation.T @ self.tvec).reshape(3)


def camera_matrix(width_px: float, height_px: float, focal_px: float) -> np.ndarray:
    return np.array([[focal_px, 0.0, width_px / 2.0], [0.0, focal_px, height_px / 2.0], [0.0, 0.0, 1.0]])


def solve_pose(xyz_m, uv_px, k: np.ndarray) -> Pose:
    """solvePnP (SQPNP for >= 4 points of any layout, IPPE-free) with a reprojection RMS."""
    obj = np.asarray(xyz_m, dtype=np.float64).reshape(-1, 3)
    img = np.asarray(uv_px, dtype=np.float64).reshape(-1, 2)
    if len(obj) < 4 or len(obj) != len(img):
        raise ValueError("need at least 4 matching points")
    ok, rvec, tvec = cv2.solvePnP(obj, img, k, None, flags=cv2.SOLVEPNP_SQPNP)
    if not ok:
        raise ValueError("solvePnP failed")
    ok = True
    proj, _ = cv2.projectPoints(obj, rvec, tvec, k, None)
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
    return Pose(rvec, tvec, k, rms)


def project(pose: Pose, xyz_m) -> np.ndarray:
    pts = np.asarray(xyz_m, dtype=np.float64).reshape(-1, 3)
    out, _ = cv2.projectPoints(pts, pose.rvec, pose.tvec, pose.camera_matrix, None)
    return out.reshape(-1, 2)


def pixel_ray(pose: Pose, uv_px) -> tuple[np.ndarray, np.ndarray]:
    """(origin, unit direction) of the viewing ray of a pixel, in the 3D frame of the points."""
    k_inv = np.linalg.inv(pose.camera_matrix)
    d_cam = k_inv @ np.array([uv_px[0], uv_px[1], 1.0])
    d = pose.rotation.T @ d_cam
    return pose.camera_position_m, d / np.linalg.norm(d)


def intersect_plane(pose: Pose, uv_px, axis: str, value_m: float) -> np.ndarray:
    """3D point where the pixel ray meets the plane <axis> = value_m (axis "x", "y" or "z")."""
    idx = "xyz".index(axis)
    o, d = pixel_ray(pose, uv_px)
    if abs(d[idx]) < 1e-9:
        raise ValueError("ray parallel to the plane")
    t = (value_m - o[idx]) / d[idx]
    if t <= 0.0:
        raise ValueError("plane behind the camera")
    return o + t * d


def solve_file(points: dict) -> dict:
    cam = points["camera"]
    pts = points["point"]
    xyz = [p["xyz_m"] for p in pts]
    uv = [p["uv_px"] for p in pts]
    scan = cam.get("focal_scan_px")
    if scan:
        cands = np.linspace(scan[0], scan[1], 41)
    else:
        cands = [float(cam["focal_px"])]
    best = None
    for f in cands:
        pose = solve_pose(xyz, uv, camera_matrix(cam["width_px"], cam["height_px"], f))
        if best is None or pose.reprojection_rms_px < best[0].reprojection_rms_px:
            best = (pose, f)
    pose, f = best
    out = {"focal_px": float(f), "rms_px": pose.reprojection_rms_px, "camera_position_m": pose.camera_position_m,
           "measures": {}}
    for m in points.get("measure", []):
        out["measures"][m["name"]] = intersect_plane(pose, m["uv_px"], m["plane"], float(m["value"]))
    return out


def main(argv: list[str]) -> int:
    with open(Path(argv[1]), "rb") as f:
        points = tomllib.load(f)
    res = solve_file(points)
    print(f"# {points.get('image', '?')}: f = {res['focal_px']:.0f} px, reprojection RMS {res['rms_px']:.2f} px, "
          f"camera at {np.round(res['camera_position_m'], 1).tolist()} m")
    for name, p in res["measures"].items():
        print(f"{name} = {np.round(p, 2).tolist()}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
