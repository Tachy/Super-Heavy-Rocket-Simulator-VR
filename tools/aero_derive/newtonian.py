"""Modified Newtonian pressure force on triangle meshes, with occlusion by first hit along the flow.

Lees' modified Newtonian: Cp = cp_max cos^2(phi), phi between the surface normal and the flow, cp_max the stagnation
pressure coefficient behind a normal shock. Rays parallel to the flow sample the projected silhouette on a regular
grid; each ray loads only the first triangle it meets, on whichever side that is (the meshes are open shells and
partly mirrored, so winding is not used). Lee sides, hidden faces and interiors are therefore never loaded.

A sample of projected area dA on a face with cos(phi) = c carries the force cp_max c^2 (dA / c) along the inward
normal, so its share of the force along the flow is cp_max c^2 dA. This makes the result linear in cp_max:
Ca = cp_max * k with k = sum(c^2 dA) / A_ref, which `first_hit` returns once per geometry.

Known limits (see docs/subsystems/aero-derivation.md): local inclination knows nothing about separation, shock
interaction or stagnated cavities; on a flat face it gives the stagnation value everywhere, above the real mean.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

GAMMA_AIR = 1.4  # calorically perfect air; real-gas effects raise cp_max by a few % above Mach ~8 (not modelled)
DEFAULT_GRID = 500  # samples across the larger silhouette extent


def cp_max(mach: float, gamma: float = GAMMA_AIR) -> float:
    """Stagnation pressure coefficient: isentropic below Mach 1, Rayleigh pitot above (NACA 1135 eqs. 44, 100)."""
    m2 = mach * mach
    if m2 < 1e-12:
        return 1.0
    if mach <= 1.0:
        ratio = (1.0 + 0.5 * (gamma - 1.0) * m2) ** (gamma / (gamma - 1.0))
    else:
        ratio = (((gamma + 1.0) ** 2 * m2) / (4.0 * gamma * m2 - 2.0 * (gamma - 1.0))) ** (gamma / (gamma - 1.0)) \
            * (1.0 - gamma + 2.0 * gamma * m2) / (gamma + 1.0)
    return (ratio - 1.0) / (0.5 * gamma * m2)


@dataclass
class FirstHit:
    """Per-sample result of the ray test; `k_*` are per unit cp_max and reference area."""
    tri_index: np.ndarray  # first triangle per grid sample, -1 = none
    cell_m: float
    cos2: np.ndarray  # cos^2(phi) per triangle
    force_perp: np.ndarray  # per triangle, per unit projected area and cp_max: normal force vector
    part_ids: np.ndarray
    part_names: list

    def k_total(self, a_ref_m2: float) -> float:
        hit = self.tri_index[self.tri_index >= 0]
        return float(self.cos2[hit].sum()) * self.cell_m ** 2 / a_ref_m2

    def k_normal(self, a_ref_m2: float) -> float:
        hit = self.tri_index[self.tri_index >= 0]
        return float(np.linalg.norm(self.force_perp[hit].sum(axis=0))) * self.cell_m ** 2 / a_ref_m2

    def k_by_part(self, a_ref_m2: float) -> dict:
        """Share of k per part; only parts that are actually hit appear."""
        hit = self.tri_index[self.tri_index >= 0]
        sums = np.bincount(self.part_ids[hit], weights=self.cos2[hit], minlength=len(self.part_names))
        return {n: float(s) * self.cell_m ** 2 / a_ref_m2 for n, s in zip(self.part_names, sums) if s > 0.0}

    def frontal_by_part_m2(self) -> dict:
        """Projected area first hit, per part [m^2]."""
        hit = self.tri_index[self.tri_index >= 0]
        counts = np.bincount(self.part_ids[hit], minlength=len(self.part_names))
        return {n: float(c) * self.cell_m ** 2 for n, c in zip(self.part_names, counts) if c > 0}


def _basis(flow) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    f = np.asarray(flow, float)
    f = f / np.linalg.norm(f)
    helper = np.array([0.0, 1.0, 0.0]) if abs(f[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(f, helper)
    u /= np.linalg.norm(u)
    return f, u, np.cross(f, u)


def first_hit(tris: np.ndarray, flow_dir_body, cell_m: float | None = None, part_ids=None, part_names=None) -> FirstHit:
    """Z-buffer equivalent of one ray per grid cell along `flow_dir_body` (the direction the air moves in body axes).

    `tris`: (n, 3, 3) metres. Every triangle is tested against every ray that falls inside its projection.
    """
    tris = np.asarray(tris, float)
    f, u, v = _basis(flow_dir_body)
    pu, pv, depth = tris @ u, tris @ v, tris @ f
    u0, u1, v0, v1 = pu.min(), pu.max(), pv.min(), pv.max()
    if cell_m is None:
        cell_m = max(u1 - u0, v1 - v0) / DEFAULT_GRID
    nu, nv = int((u1 - u0) / cell_m) + 2, int((v1 - v0) / cell_m) + 2
    zbuf = np.full((nu, nv), np.inf)
    index = np.full((nu, nv), -1, dtype=np.int32)
    # sample centres: u0 + (i + 0.5) cell
    iu0 = np.maximum(np.ceil((pu.min(axis=1) - u0) / cell_m - 0.5).astype(int), 0)
    iu1 = np.minimum(np.floor((pu.max(axis=1) - u0) / cell_m - 0.5).astype(int), nu - 1)
    iv0 = np.maximum(np.ceil((pv.min(axis=1) - v0) / cell_m - 0.5).astype(int), 0)
    iv1 = np.minimum(np.floor((pv.max(axis=1) - v0) / cell_m - 0.5).astype(int), nv - 1)
    denom = (pv[:, 1] - pv[:, 2]) * (pu[:, 0] - pu[:, 2]) + (pu[:, 2] - pu[:, 1]) * (pv[:, 0] - pv[:, 2])
    for t in np.nonzero((iu1 >= iu0) & (iv1 >= iv0) & (np.abs(denom) > 1e-14))[0]:
        us = u0 + (np.arange(iu0[t], iu1[t] + 1) + 0.5) * cell_m
        vs = v0 + (np.arange(iv0[t], iv1[t] + 1) + 0.5) * cell_m
        du, dv = us[:, None] - pu[t, 2], vs[None, :] - pv[t, 2]
        w0 = ((pv[t, 1] - pv[t, 2]) * du + (pu[t, 2] - pu[t, 1]) * dv) / denom[t]
        w1 = ((pv[t, 2] - pv[t, 0]) * du + (pu[t, 0] - pu[t, 2]) * dv) / denom[t]
        w2 = 1.0 - w0 - w1
        inside = (w0 >= 0.0) & (w1 >= 0.0) & (w2 >= 0.0)
        if not inside.any():
            continue
        z = w0 * depth[t, 0] + w1 * depth[t, 1] + w2 * depth[t, 2]
        window = zbuf[iu0[t]:iu1[t] + 1, iv0[t]:iv1[t] + 1]
        nearer = inside & (z < window)
        window[nearer] = z[nearer]
        index[iu0[t]:iu1[t] + 1, iv0[t]:iv1[t] + 1][nearer] = t

    normal = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    length = np.linalg.norm(normal, axis=1)
    length[length == 0.0] = 1.0
    normal = normal / length[:, None]
    c = normal @ f
    inward = normal * np.sign(c)[:, None]  # along the push of the air on the exposed side
    cabs = np.abs(c)
    # per unit projected area and cp_max: cos^2 / cos along the inward normal; minus its part along the flow
    perp = (inward - cabs[:, None] * f) * cabs[:, None]
    ids = np.zeros(len(tris), dtype=np.int64) if part_ids is None else np.asarray(part_ids, dtype=np.int64)
    names = ["all"] if part_names is None else list(part_names)
    return FirstHit(index, cell_m, cabs * cabs, perp, ids, names)


def axial_force_coefficient(tris: np.ndarray, flow_dir_body, cp_max_value: float, a_ref_m2: float,
                            cell_m: float | None = None) -> tuple[float, float]:
    """(ca, cn): force along the flow and magnitude across it, per q A_ref, with first-hit occlusion."""
    hit = first_hit(tris, flow_dir_body, cell_m)
    return cp_max_value * hit.k_total(a_ref_m2), cp_max_value * hit.k_normal(a_ref_m2)
