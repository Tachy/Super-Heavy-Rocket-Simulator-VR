"""Joint multi-view fit of part parameters and photo cameras.

Unknowns: the part parameters (shared by all photos) and per photo the camera pose (position 3 + rotation vector 3
relative to the initial camera), ln(focal) and, for fisheye photos with fit_k, k1. Residuals: reprojection of annotated
points and edges (px / sigma_px), anchors ((length - length_m) / sigma_m) and parameter priors ((p - prior) / prior_sigma).
Robust loss (soft_l1) acts on the image rows only; anchor and prior rows stay quadratic.
Sigma: sqrt(diag((J^T J)^-1)) from a rank-aware SVD of the column-scaled Jacobian, times max(1, sqrt(chi2 / dof)) for
parameters the image / anchor data constrain (dof counts data rows only). A parameter no data row sees comes back as its
prior with sigma == prior_sigma (never inflated), or inf without a prior; `FitResult.constrained_by_data` flags it.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import least_squares

from .annotations import PhotoObs
from .camera import Camera, project, rotvec_to_matrix

BEHIND_PX = 1.0e4  # residual for a point behind the camera
EDGE_SAMPLES = 24


class PartModel:
    """Subclass: set `self.params = {name: (prior, prior_sigma)}`, implement points(params); edges() is optional."""

    params: dict[str, tuple[float, float]]

    def points(self, params: dict[str, float]) -> dict[str, np.ndarray]:
        raise NotImplementedError

    def edges(self) -> dict[str, tuple[str, str]]:
        """label -> (point label a, point label b): straight 3D edges that annotations may trace."""
        return {}


@dataclass(frozen=True)
class Anchor:
    label_a: str
    label_b: str
    length_m: float
    sigma_m: float
    source: str = ""


@dataclass
class FitResult:
    params: dict
    sigma: dict
    residual_px: dict
    cameras: dict
    chi2: float = 0.0   # data rows only (image + anchors)
    dof: int = 0        # data rows - rank of the data Jacobian (priors excluded); may be <= 0
    constrained_by_data: dict = field(default_factory=dict)  # name -> False: only a prior (or nothing) fixes it: tag [E]
    anchor_residuals: list = field(default_factory=list)  # (source, (length - length_m) / sigma_m) per anchor


def _apply(cam0: Camera, x: np.ndarray, fit_k: bool) -> Camera:
    rot = rotvec_to_matrix(x[3:6]) @ cam0.rot
    k = (x[7],) + tuple(cam0.k[1:]) if fit_k else cam0.k
    return cam0.with_(position_m=cam0.position_m + x[0:3], rot=rot, f_px=cam0.f_px * np.exp(x[6]), k=k)


def _polyline_distance(pts: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Signed distance of each observed point to the projected polyline (nearest segment)."""
    a, b = poly[:-1], poly[1:]
    ab = b - a
    L2 = np.maximum((ab ** 2).sum(1), 1e-12)
    out = np.empty(len(pts))
    for i, p in enumerate(pts):
        t = np.clip(((p - a) * ab).sum(1) / L2, 0, 1)
        d = p - (a + ab * t[:, None])
        j = int(np.argmin((d ** 2).sum(1)))
        cross = ab[j, 0] * (p[1] - a[j, 1]) - ab[j, 1] * (p[0] - a[j, 0])
        out[i] = np.hypot(*d[j]) * (1.0 if cross >= 0 else -1.0)
    return out


def fit(photos: list[PhotoObs], model: PartModel, anchors: list[Anchor], loss: str = "soft_l1", f_scale: float = 2.0) -> FitResult:
    photos = list(photos)
    names = list(model.params)
    nP = len(names)
    prior = np.array([model.params[n][0] for n in names], float)
    psig = np.array([model.params[n][1] for n in names], float)
    edge_defs = model.edges()
    blocks = []  # (offset, n) per photo
    off = nP
    for ph in photos:
        n = 8 if ph.fit_k and ph.camera.model != "pinhole" else 7
        blocks.append((off, n))
        off += n
    nX = off
    x0 = np.zeros(nX)
    x0[:nP] = prior

    def split(x):
        params = dict(zip(names, x[:nP]))
        cams = []
        for ph, (o, n) in zip(photos, blocks):
            xc = x[o:o + n].copy()
            cams.append(_apply(ph.camera, xc, n == 8))
        return params, cams

    # fisheye k1: x holds the absolute k1 (not an offset); start from the initial value, k padded to length 1
    for i, ph in enumerate(photos):
        if blocks[i][1] == 8 and not ph.camera.k:
            photos[i] = PhotoObs(ph.id, ph.camera.with_(k=(0.0,)), ph.points, ph.edges, ph.sigma_px, ph.fit_k, ph.camera_sigma)
        if blocks[i][1] == 8:
            x0[blocks[i][0] + 7] = photos[i].camera.k[0]

    def residuals(x, detail=False, robust=False):
        params, cams = split(x)
        pts = model.points(params)
        res = []
        per_photo = {}
        for ph, cam in zip(photos, cams):
            r = []
            for lab, uv in ph.points.items():
                p = project(np.asarray(pts[lab], float)[None, :], cam)[0]
                r.extend((p - np.asarray(uv)) if np.all(np.isfinite(p)) else [BEHIND_PX, BEHIND_PX])
            for lab, poly_obs in ph.edges.items():
                la, lb = edge_defs[lab]
                t = np.linspace(0, 1, EDGE_SAMPLES)[:, None]
                seg = np.asarray(pts[la], float) * (1 - t) + np.asarray(pts[lb], float) * t
                poly = project(seg, cam)
                if not np.all(np.isfinite(poly)):
                    r.extend([BEHIND_PX] * len(poly_obs))
                else:
                    r.extend(_polyline_distance(np.asarray(poly_obs, float), poly))
            r = np.asarray(r, float)
            per_photo[ph.id] = float(np.sqrt(np.mean(r ** 2))) if len(r) else 0.0
            rn = r / ph.sigma_px
            if robust and loss == "soft_l1":  # soft_l1 as a residual transform: image rows only
                rn = np.sign(rn) * np.sqrt(2.0 * f_scale ** 2 * (np.sqrt(1.0 + (rn / f_scale) ** 2) - 1.0))
            res.append(rn)
        arow = []
        for a in anchors:
            arow.append((np.linalg.norm(np.asarray(pts[a.label_a]) - np.asarray(pts[a.label_b])) - a.length_m) / a.sigma_m)
        crow = []
        for ph, (o, n) in zip(photos, blocks):
            if ph.camera_sigma is not None:
                sp, sa, sf = ph.camera_sigma
                crow.extend(x[o:o + 3] / sp)
                crow.extend(x[o + 3:o + 6] / sa)
                crow.append(x[o + 6] / sf)
        prow = (x[:nP] - prior) / psig
        out = np.concatenate(res + [np.asarray(arow, float), np.asarray(crow, float), prow])
        if detail:
            return out, per_photo, arow, sum(len(q) for q in res) + len(arow)
        return out

    sol = least_squares(lambda x: residuals(x, robust=True), x0, loss="linear", x_scale="jac", method="trf", max_nfev=400,
                        xtol=1e-12, ftol=1e-12, gtol=1e-12)
    x = sol.x
    r, per_photo, arow, nd = residuals(x, detail=True)
    J = _jacobian(residuals, x)
    m, n = J.shape
    sd, data_flag, rank_d = _sigma(J, nd, psig)
    dof = nd - rank_d
    chi2 = float(r[:nd] @ r[:nd])
    scale = max(1.0, chi2 / dof) if dof > 0 else 1.0
    sd = np.where(data_flag, sd * np.sqrt(scale), sd)
    params, cams = split(x)
    sigma = {nm: float(sd[i]) for i, nm in enumerate(names)}
    flags = {nm: bool(data_flag[i]) for i, nm in enumerate(names)}
    for ph, (o, nb) in zip(photos, blocks):
        for j, key in enumerate(["x", "y", "z", "rx", "ry", "rz", "lnf", "k1"][:nb]):
            sigma[f"cam.{ph.id}.{key}"] = float(sd[o + j])
            flags[f"cam.{ph.id}.{key}"] = bool(data_flag[o + j])
    cameras = {ph.id: c for ph, c in zip(photos, cams)}
    return FitResult({k: float(v) for k, v in params.items()}, sigma, per_photo, cameras, chi2, dof,
                     anchor_residuals=[(a.source, float(v)) for a, v in zip(anchors, arow)], constrained_by_data=flags)


def _sigma(J: np.ndarray, nd: int, psig: np.ndarray, rel_tol: float = 1e-9):
    """Rank-aware sigma per unknown: (sd, constrained_by_data, rank of the data Jacobian). nd = number of data rows (first rows of J)."""
    cn = np.linalg.norm(J, axis=0)
    cn[cn == 0] = 1.0
    Js = J / cn
    _, s, vt = np.linalg.svd(Js, full_matrices=False)
    keep = s > rel_tol * s[0]
    var_s = np.zeros(J.shape[1])
    for sj, vj, k in zip(s, vt, keep):
        if k:
            var_s += vj ** 2 / sj ** 2
        else:
            var_s[vj ** 2 > 1e-8] = np.inf
    sd = np.sqrt(var_s) / cn
    _, sd_, vd = np.linalg.svd(J[:nd] / cn, full_matrices=False) if nd > 0 else (None, np.zeros(1), np.zeros((1, J.shape[1])))
    kd = sd_ > rel_tol * max(sd_[0], 1e-300)
    P = (vd[kd] ** 2).sum(0) if kd.any() else np.zeros(J.shape[1])  # diag of the row-space projector of the data Jacobian
    return sd, P > 0.5, int(kd.sum())


def _jacobian(fun, x, eps=1e-7):
    f0 = fun(x)
    J = np.empty((len(f0), len(x)))
    for i in range(len(x)):
        h = eps * max(1.0, abs(x[i]))
        xp = x.copy()
        xp[i] += h
        xm = x.copy()
        xm[i] -= h
        J[:, i] = (fun(xp) - fun(xm)) / (2 * h)
    return J
