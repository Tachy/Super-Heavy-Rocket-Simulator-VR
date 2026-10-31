"""Camera solve: synthetic views with a known pose must be recovered (no photo data involved)."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "measure"))
import camera_solve as cs  # noqa: E402

BOX = [(-6, -6, 0), (6, -6, 0), (6, 6, 0), (-6, 6, 0), (-6, -6, 40), (6, -6, 40), (6, 6, 40), (-6, 6, 40)]


def _synthetic_pose():
    k = cs.camera_matrix(2000, 1400, 3000.0)
    cam = np.array([150.0, -400.0, 30.0])
    fwd = np.array([-150.0, 400.0, 10.0]) / np.linalg.norm([-150.0, 400.0, 10.0])
    right = np.cross(fwd, [0, 0, 1.0])
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    rot = np.vstack([right, down, fwd])
    rvec = cs.cv2.Rodrigues(rot)[0]
    tvec = -rot @ cam
    return cs.Pose(rvec, tvec.reshape(3, 1), k, 0.0), cam


def test_pose_recovered_from_projected_points():
    true, cam = _synthetic_pose()
    uv = cs.project(true, BOX)
    pose = cs.solve_pose(BOX, uv, true.camera_matrix)
    assert pose.reprojection_rms_px < 1e-3
    assert np.allclose(pose.camera_position_m, cam, atol=0.05)


def test_noise_gives_bounded_position_error():
    true, cam = _synthetic_pose()
    rng = np.random.default_rng(1)
    uv = cs.project(true, BOX) + rng.normal(0.0, 1.0, (8, 2))
    pose = cs.solve_pose(BOX, uv, true.camera_matrix)
    assert 0.3 < pose.reprojection_rms_px < 3.0
    assert np.linalg.norm(pose.camera_position_m - cam) < 8.0  # 427 m away, 1 px noise


def test_plane_intersection_recovers_height():
    true, _ = _synthetic_pose()
    pose = cs.solve_pose(BOX, cs.project(true, BOX), true.camera_matrix)
    target = np.array([6.0, -6.0, 25.0])
    uv = cs.project(true, [target])[0]
    got = cs.intersect_plane(pose, uv, "y", -6.0)
    assert np.allclose(got, target, atol=0.05)


def test_focal_scan_finds_true_focal_length():
    true, _ = _synthetic_pose()
    uv = cs.project(true, BOX)
    pts = {"camera": {"width_px": 2000, "height_px": 1400, "focal_scan_px": [2000, 4000]},
           "point": [{"xyz_m": list(p), "uv_px": u.tolist()} for p, u in zip(BOX, uv)]}
    res = cs.solve_file(pts)
    assert res["focal_px"] == pytest.approx(3000.0, abs=1.0)
    assert res["rms_px"] < 0.01


def test_too_few_points_rejected():
    with pytest.raises(ValueError):
        cs.solve_pose(BOX[:3], [(0, 0)] * 3, cs.camera_matrix(100, 100, 100.0))
