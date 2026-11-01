"""Nose-first load on the parts: which samples see the flow, and at what share of the free-stream dynamic pressure.

The plain first hit (`newtonian.first_hit`) lets a ring hide a skirt 70 m behind it. Flow reattaches a few step heights
behind a part, so here a part shields a later one on the same ray only within its wake range. Used for the nose-first
cases only: the descent's comparison keeps the plain first hit (its tables are not derived from this tool).

Rules per ray parallel to the axis, nose to tail:
- The first hit is exposed.
- The hull ends the ray: what lies behind it is inside the body.
- A later hit on a part already met on this ray never loads (back shell, or a row of the same part): winding cannot
  tell a back face from a new front in these open, partly mirrored shells.
- A later hit on another part is exposed if it lies more than WAKE_STEP_HEIGHTS x the height of the part just passed
  behind that part's last hit. Height = how far that part stands out of the hull at most.

Limits: a wake is taken as all (shielded) or nothing; a part's span counts as its height, which overstates a thin fin's
or flap's wake (only the strip behind its edge is affected).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import newtonian

WAKE_STEP_HEIGHTS = 10.0  # [E] turbulent backward-facing step reattaches at about 6-8 heights (Eaton & Johnston, AIAA J. 19, 1981, recalled); rounded up


def group_of(part_name: str) -> str:
    """A fin or flap is one body whatever its shells are called."""
    pieces = part_name.split("/")
    return "/".join(pieces[:2])


def is_hull(part_name: str) -> bool:
    return part_name.endswith("/hull")


@dataclass
class ExposedLoad:
    """Exposed samples: part id, c^2 dA [m^2], station [m], height out of the hull [m]."""
    part_ids: np.ndarray
    load_m2: np.ndarray
    x_m: np.ndarray
    height_m: np.ndarray
    names: list

    def k_by_part(self, a_ref_m2: float) -> dict:
        sums = np.bincount(self.part_ids, weights=self.load_m2, minlength=len(self.names))
        return {n: float(s) / a_ref_m2 for n, s in zip(self.names, sums) if s > 0.0}

    def credited_k_by_part(self, a_ref_m2: float, q_share) -> dict:
        """As `k_by_part`, each sample weighted by `q_share(x_m, height_m)`."""
        sums = np.bincount(self.part_ids, weights=self.load_m2 * q_share(self.x_m, self.height_m), minlength=len(self.names))
        return {n: float(s) / a_ref_m2 for n, s in zip(self.names, sums) if s > 0.0}


def exposed_hits(tris, part_ids, part_names, hull_radius_m, cell_m: float, wake_step_heights: float = WAKE_STEP_HEIGHTS) -> ExposedLoad:
    """Nose first (air moving along -X). `hull_radius_m(x_array)` gives the bare hull's radius at a station."""
    hits = newtonian.all_hits(tris, (-1.0, 0.0, 0.0), cell_m)
    tri = hits.tri
    part = np.asarray(part_ids)[tri]
    x = -hits.depth  # depth grows along the flow, the flow runs along -X
    height = np.maximum(hits.radial_m - hull_radius_m(x), 0.0)

    groups = sorted({group_of(n) for n in part_names})
    group_index = {g: i for i, g in enumerate(groups)}
    group_of_part = np.array([group_index[group_of(n)] for n in part_names])
    hull_of_part = np.array([is_hull(n) for n in part_names])
    group = group_of_part[part]
    group_height = np.zeros(len(groups))
    np.maximum.at(group_height, group, height)

    order = np.lexsort((hits.depth, hits.ray))
    ray_s, depth_s, group_s, hull_s = hits.ray[order].tolist(), hits.depth[order].tolist(), group[order].tolist(), hull_of_part[part][order].tolist()
    wake = (group_height * wake_step_heights).tolist()
    exposed_s = np.zeros(len(order), dtype=bool)
    current_ray, stopped = -1, False
    met: set = set()
    last_depth, last_group = 0.0, -1
    for i, ray in enumerate(ray_s):
        g = group_s[i]
        if ray != current_ray:
            current_ray, stopped = ray, False
            met = set()
            exposed_s[i] = True
        elif not stopped and g not in met and depth_s[i] - last_depth > wake[last_group]:
            exposed_s[i] = True
        if stopped:
            continue
        met.add(g)
        last_depth, last_group = depth_s[i], g
        if hull_s[i]:
            stopped = True
    keep = order[exposed_s]
    return ExposedLoad(part[keep], hits.cos2[tri[keep]] * cell_m ** 2, x[keep], height[keep], list(part_names))
