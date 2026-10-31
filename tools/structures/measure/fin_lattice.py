"""Lattice homography of a grid-fin image from points that sit on a regular grid (cell centres or wall crossings).

Points -> integer lattice indices (BFS with locally predicted neighbour vectors) -> DLT homography image -> lattice plane
(unit = one grid step of the point set) -> residual in px and in grid steps. Used by fin_rectify_all.py.
"""
from __future__ import annotations

import cv2
import numpy as np


def assign_indices(pts, seed_idx=None, tol=0.33):
    """BFS over the points. The four neighbour vectors are taken from the two dominant nearest-neighbour directions."""
    pts = np.asarray(pts, float)
    n = len(pts)
    d = np.linalg.norm(pts[:, None] - pts[None], axis=2)
    np.fill_diagonal(d, 1e9)
    nn = np.sort(d, axis=1)[:, 0]
    step = float(np.median(nn))
    vecs = []
    for i in range(n):
        for j in np.where(d[i] < 1.5 * step)[0]:
            vecs.append(pts[j] - pts[i])
    vecs = np.array(vecs)
    ang = np.arctan2(vecs[:, 1], vecs[:, 0]) % np.pi
    # two SHORTEST independent lattice vectors (the nearest neighbours): mean length per angle bin, shortest bins first
    nbin = 36
    bins = np.minimum((ang / np.pi * nbin).astype(int), nbin - 1)
    ln = np.linalg.norm(vecs, axis=1)
    mean_len = np.array([ln[bins == k].mean() if (bins == k).sum() >= 3 else 1e9 for k in range(nbin)])
    k1 = int(np.argmin(mean_len))
    far = [k for k in range(nbin) if min((k - k1) % nbin, (k1 - k) % nbin) >= 6]
    k2 = far[int(np.argmin(mean_len[far]))]
    a1, a2 = (k1 + 0.5) * np.pi / nbin, (k2 + 0.5) * np.pi / nbin

    def mean_vec(a):
        sel = vecs[(np.abs(((ang - a + np.pi / 2) % np.pi) - np.pi / 2) < 0.25)]
        sel = np.array([v if v @ [np.cos(a), np.sin(a)] > 0 else -v for v in sel])
        return sel.mean(axis=0)

    v1, v2 = mean_vec(a1), mean_vec(a2)
    start = int(np.argmin(np.linalg.norm(pts - pts.mean(axis=0), axis=1))) if seed_idx is None else seed_idx
    idx = {start: (0, 0)}
    queue = [start]
    while queue:
        cur = queue.pop(0)
        i0, j0 = idx[cur]
        for (di, dj) in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ni = (i0 + di, j0 + dj)
            if ni in idx.values():
                continue
            # local vectors from already assigned neighbours of cur
            loc = {}
            for k2, (ii, jj) in idx.items():
                if (ii, jj) in ((i0 + 1, j0), (i0 - 1, j0), (i0, j0 + 1), (i0, j0 - 1)) and k2 != cur:
                    loc[(ii - i0, jj - j0)] = pts[k2] - pts[cur]
            if (di, dj) in loc:
                continue
            if (-di, -dj) in loc:
                pred = pts[cur] - loc[(-di, -dj)]
            else:
                pred = pts[cur] + di * v1 + dj * v2
            dist = np.linalg.norm(pts - pred, axis=1)
            dist[list(idx.keys())] = 1e9
            m = int(np.argmin(dist))
            if dist[m] < tol * min(np.linalg.norm(v1), np.linalg.norm(v2)):
                idx[m] = ni
                queue.append(m)
    return idx, step


def fit(pts, idx):
    """DLT homography (lattice (i, j) -> image) by cv2 with all points; returns H_img_from_lat and residuals."""
    keys = sorted(idx)
    img = np.array([pts[k] for k in keys], np.float32)
    lat = np.array([idx[k] for k in keys], np.float32)
    h, _ = cv2.findHomography(lat, img, 0)
    proj = cv2.perspectiveTransform(lat[None], h)[0]
    res_px = np.linalg.norm(proj - img, axis=1)
    hinv = np.linalg.inv(h)
    back = cv2.perspectiveTransform(img[None], hinv.astype(np.float64))[0]
    res_lat = np.linalg.norm(back - lat, axis=1)
    return h, res_px, res_lat, lat
