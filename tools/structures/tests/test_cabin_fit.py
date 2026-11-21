"""Cabin fit toolkit: synthetic recovery, unconstrained parameters, camera round trips, rows, annotations, anchors."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cabin_fit import annotations, anchors  # noqa: E402
from cabin_fit.camera import Camera, project, unproject  # noqa: E402
from cabin_fit.fit import Anchor, PartModel, fit  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
POSITIONS = [(1.4, -1.7, 1.2), (-1.5, -1.3, 0.6), (0.2, 1.9, 1.3), (0.1, -0.4, 2.0)][:3]


class Box(PartModel):
    def __init__(self, prior=(1.0, 0.6, 0.2), sigma=0.5, extra=False):
        self.params = {"w": (prior[0], sigma), "d": (prior[1], sigma), "h": (prior[2], sigma)}
        if extra:
            self.params["unused"] = (0.37, 0.12)

    def points(self, p):
        w, d, h = p["w"], p["d"], p["h"]
        return {f"c{i}{j}{k}": np.array([(i - .5) * w, (j - .5) * d, (k - .5) * h]) for i in (0, 1) for j in (0, 1) for k in (0, 1)}

    def edges(self):
        return {"bottom_front": ("c000", "c100")}


def _cam(pos, target, f=900.0, size=(1200, 800), model="pinhole", k=()):
    fwd = np.asarray(target, float) - pos
    return Camera.from_row({"id": "x", "size_px": list(size), "position_m": list(pos), "forward": list(fwd), "up": [0, 0, 1],
                            "fov_h_deg": 60.0, "model": model}).with_(f_px=f, k=k)


def _photos(rng, noise=0.5, perturb=True):
    truth = Box((1.2, 0.5, 0.3)).points({"w": 1.2, "d": 0.5, "h": 0.3})
    labels = list(truth)
    arr = np.array([truth[l] for l in labels])
    out = []
    for i, pos in enumerate(POSITIONS):
        cam = _cam(np.array(pos), (0, 0, 0))
        uv = project(arr, cam) + rng.normal(0, noise, (8, 2))
        guess = cam
        if perturb:
            guess = cam.with_(position_m=cam.position_m + rng.normal(0, 0.05, 3), f_px=cam.f_px * 1.03)
        out.append(annotations.PhotoObs(f"p{i}", guess, {l: tuple(u) for l, u in zip(labels, uv)}, sigma_px=0.5))
    return out


def test_fit_recovers_synthetic_box():
    rng = np.random.default_rng(0)
    photos = _photos(rng)
    res = fit(photos, Box((1.1, 0.6, 0.25)), [Anchor("c000", "c100", 1.2, 0.001, "synthetic")])
    assert res.params["w"] == pytest.approx(1.2, abs=0.002)
    assert res.params["d"] == pytest.approx(0.5, abs=0.002)
    assert res.params["h"] == pytest.approx(0.3, abs=0.002)
    assert max(res.residual_px.values()) < 1.0
    assert 0 < res.sigma["d"] < 0.003  # the 2 mm tolerance is about 1 sigma of 0.5 px noise at this geometry


@pytest.mark.parametrize("seed", range(1, 6))
def test_fit_error_consistent_with_sigma(seed):
    res = fit(_photos(np.random.default_rng(seed)), Box((1.1, 0.6, 0.25)), [Anchor("c000", "c100", 1.2, 0.001, "synthetic")])
    for name, true in (("w", 1.2), ("d", 0.5), ("h", 0.3)):
        assert abs(res.params[name] - true) < 4 * res.sigma[name]


def test_edge_residuals_constrain_dimension():
    rng = np.random.default_rng(5)
    photos = _photos(rng, noise=0.0, perturb=False)
    for ph in photos:  # replace point 'c100' by an edge trace of the bottom front edge
        ph.points.pop("c100")
        ph.points.pop("c000")
    truth = Box().points({"w": 1.2, "d": 0.5, "h": 0.3})
    for ph, pos in zip(photos, POSITIONS):
        t = np.linspace(0.1, 0.9, 5)[:, None]
        seg = truth["c000"] * (1 - t) + truth["c100"] * t
        ph.edges["bottom_front"] = [tuple(u) for u in project(seg, _cam(np.array(pos), (0, 0, 0)))]
    res = fit(photos, Box((1.2, 0.5, 0.3)), [Anchor("c010", "c110", 1.2, 0.001, "synthetic")])
    assert res.params["w"] == pytest.approx(1.2, abs=0.002)
    assert max(res.residual_px.values()) < 0.1


def test_fit_reports_unconstrained_parameters():
    rng = np.random.default_rng(4)
    res = fit(_photos(rng), Box((1.1, 0.6, 0.25), extra=True), [Anchor("c000", "c100", 1.2, 0.001, "synthetic")])
    assert res.params["unused"] == pytest.approx(0.37, abs=1e-9)
    assert res.sigma["unused"] == pytest.approx(0.12, rel=1e-6)


@pytest.mark.parametrize("k", [(), (0.02, -0.003)])
def test_fisheye_round_trip(k):
    cam = _cam(np.array([0.1, 0.2, 0.3]), (1, 1, 0.5), f=700.0, size=(2000, 1500), model="fisheye_equidistant", k=k)
    rng = np.random.default_rng(0)
    d = rng.normal(size=(200, 3))
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    d = d[(d @ cam.forward) > np.cos(np.radians(80))]
    pts = cam.position_m + 3.0 * d
    back = unproject(project(pts, cam), cam)
    assert np.abs(back - d).max() < 1e-9


def test_pinhole_round_trip():
    cam = _cam(np.array([0.0, 0, 0]), (1, 0.2, 0.1))
    d = np.array([[1, 0.25, 0.1], [1, 0.1, 0.15]])
    d = d / np.linalg.norm(d, axis=1, keepdims=True)
    assert np.abs(unproject(project(d * 2, cam), cam) - d).max() < 1e-12


@pytest.mark.parametrize("model", ["pinhole", "fisheye_equidistant"])
def test_camera_row_round_trip(model):
    row = {"id": "a", "file": "f.jpg", "size_px": [1200, 800], "position_m": [1.0, 2.0, 3.0], "forward": [0.14263, -0.98978, 0.0],
           "up": [0.98978, 0.14263, 0.0], "fov_h_deg": 96.449, "model": model}
    cam = Camera.from_row(row)
    back = cam.to_row()
    assert back["fov_h_deg"] == pytest.approx(96.449, abs=1e-9)
    assert np.allclose(back["forward"], row["forward"], atol=1e-4) and np.allclose(back["up"], row["up"], atol=1e-4)
    assert back["model"] == model and back["size_px"] == [1200, 800]
    # image right = forward x up, as in photo_cameras.toml
    assert np.allclose(cam.rot[0], np.cross(row["forward"], row["up"]), atol=1e-4)


def test_photo_cameras_file_rows_load():
    path = ROOT / "config" / "cockpits" / "photo_cameras.toml"
    if not path.exists():
        pytest.skip("photo_cameras.toml not present")
    import tomllib
    for row in tomllib.loads(path.read_text(encoding="utf-8"))["photo"]:
        cam = Camera.from_row(row)
        assert cam.to_row()["fov_h_deg"] == pytest.approx(row["fov_h_deg"], abs=1e-6)


def test_annotation_load(tmp_path):
    (tmp_path / "ph.toml").write_text('sigma_px = 2.0\n[[point]]\nlabel = "a"\npx = [1.0, 2.0]\n[[edge]]\nlabel = "e"\npx = [[0,0],[1,1]]\n')
    cams = tmp_path / "cams.toml"
    cams.write_text('[[photo]]\nid = "ph"\nfile = "x"\nsize_px = [100, 50]\nposition_m = [0,0,0]\nforward = [1,0,0]\nup = [0,0,1]\n'
                    'fov_h_deg = 90.0\nmodel = "pinhole"\n')
    obs = annotations.load("ph", tmp_path, cams)
    assert obs.points == {"a": (1.0, 2.0)} and obs.edges["e"] == [(0.0, 0.0), (1.0, 1.0)] and obs.sigma_px == 2.0
    assert obs.camera.f_px == pytest.approx(50.0)


def test_anchor_table_matches_ansur_csv():
    csv = ROOT / "data" / "anthropometry" / "ANSUR_II_MALE_Public.csv"
    if not csv.exists():
        pytest.skip("ANSUR csv not present")
    import csv as csvmod
    import statistics
    with open(csv, encoding="latin-1", newline="") as fh:
        rows = list(csvmod.DictReader(fh))
    for name, col in anchors.ANSUR_COLUMNS.items():
        med = statistics.median(float(r[col]) for r in rows) / 1000.0
        assert anchors.KNOWN[name].length_m == pytest.approx(med, abs=1e-9)
    a = anchors.anchor("idss_passage_m", "p", "q")
    assert a.length_m == 0.8 and a.source.startswith("[S]")


def _unseen_box(prior_sigma):
    box = Box((1.1, 0.6, 0.25), extra=True)
    box.params["unused"] = (0.37, prior_sigma)
    return box


def test_unseen_parameter_wide_prior_keeps_prior_sigma_and_flag():
    res = fit(_photos(np.random.default_rng(4)), _unseen_box(1e4), [Anchor("c000", "c100", 1.2, 0.001, "synthetic")])
    assert res.sigma["unused"] == pytest.approx(1e4, rel=1e-6)
    assert res.constrained_by_data["unused"] is False
    assert all(res.constrained_by_data[k] for k in ("w", "d", "h"))


def test_unseen_parameter_not_inflated_by_overdispersed_data():
    rng = np.random.default_rng(4)
    photos = _photos(rng, noise=2.0)  # actual 2 px noise, claimed 0.5 px
    res = fit(photos, _unseen_box(0.12), [Anchor("c000", "c100", 1.2, 0.001, "synthetic")])
    assert res.chi2 / res.dof > 4
    assert res.sigma["unused"] == pytest.approx(0.12, rel=1e-6)
    assert res.constrained_by_data["unused"] is False
    assert res.sigma["d"] > 0


def test_photo_without_observations_has_no_confident_camera():
    rng = np.random.default_rng(4)
    photos = _photos(rng)
    photos.append(annotations.PhotoObs("empty", photos[0].camera, {}, sigma_px=0.5))
    res = fit(photos, Box((1.1, 0.6, 0.25)), [Anchor("c000", "c100", 1.2, 0.001, "synthetic")])
    assert res.sigma["cam.empty.x"] == float("inf")
    assert res.constrained_by_data["cam.empty.x"] is False


def test_anchor_row_stays_quadratic_under_robust_loss():
    # an anchor 5 sigma off the points' scale must pull with its full (quadratic) weight, not as L1
    rng = np.random.default_rng(0)
    photos = _photos(rng, noise=0.0, perturb=False)
    res = fit(photos, Box((1.2, 0.5, 0.3), sigma=10.0), [Anchor("c000", "c100", 1.2, 0.0005, "synthetic")])
    assert res.params["w"] == pytest.approx(1.2, abs=2e-4)
    (_, rv), = res.anchor_residuals
    assert abs(rv) < 0.5


def test_annotation_load_optional_fit_k_and_camera_sigma(tmp_path):
    (tmp_path / "ph.toml").write_text('fit_k = true\ncamera_sigma = [0.05, 0.02, 0.01]\n')
    cams = tmp_path / "cams.toml"
    cams.write_text('[[photo]]\nid = "ph"\nfile = "x"\nsize_px = [100, 50]\nposition_m = [0,0,0]\nforward = [1,0,0]\nup = [0,0,1]\n'
                    'fov_h_deg = 90.0\nmodel = "fisheye_equidistant"\n')
    obs = annotations.load("ph", tmp_path, cams)
    assert obs.fit_k is True and obs.camera_sigma == (0.05, 0.02, 0.01)


def _one_photo_three_points():
    ph = _photos(np.random.default_rng(4))[0]
    keep = ("c000", "c111", "c101")
    return annotations.PhotoObs("p0", ph.camera, {k: ph.points[k] for k in keep}, sigma_px=0.5)


def test_underdetermined_fit_reports_null_directions_and_empty_camera_inf():
    ph = _one_photo_three_points()
    empty = annotations.PhotoObs("empty", ph.camera, {}, sigma_px=0.5)
    res = fit([ph, empty], Box((1.1, 0.6, 0.25)), [])
    assert all(res.sigma[f"cam.empty.{k}"] == float("inf") for k in ("x", "y", "z", "rx", "ry", "rz", "lnf"))
    assert not any(res.constrained_by_data[f"cam.empty.{k}"] for k in ("x", "y", "z", "rx", "ry", "rz", "lnf"))
    assert any(np.isinf(v) for k, v in res.sigma.items() if k.startswith("cam.p0."))  # 9 rows < 10 camera+part unknowns


def test_constrained_means_data_outweigh_prior():
    res = fit([_one_photo_three_points()], Box((1.1, 0.6, 0.25)), [])  # no anchor: no absolute scale
    assert res.constrained_by_data["d"] is False and res.constrained_by_data["h"] is False
    assert res.sigma["d"] >= 0.5 * 0.5 and res.sigma["h"] >= 0.5 * 0.5


def test_unknown_loss_raises():
    with pytest.raises(ValueError):
        fit(_photos(np.random.default_rng(1)), Box(), [], loss="huber")
