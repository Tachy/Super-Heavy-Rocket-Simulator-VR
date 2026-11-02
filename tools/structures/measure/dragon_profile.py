"""Crew Dragon: our generated glTF's outline against the typed, measured numbers of config/vehicles/detail/crew_dragon.toml.

    python tools/structures/measure/dragon_profile.py [--models data/models/crew_dragon]

Stations of the body profile (azimuths outside the cowls), the crown of the cowl lobes, the nose's seam plane and tip, the hinge point, the trunk's fin outline.
Prints each table's largest deviation in metres. Reads glTF + bin only (no Blender, no third-party file).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import tomllib
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
DETAIL = ROOT / "config" / "vehicles" / "detail" / "crew_dragon.toml"
VEHICLE = ROOT / "config" / "vehicles" / "crew_dragon.toml"
MODELS = ROOT / "data" / "models" / "crew_dragon"
PLAIN_AZIMUTHS_DEG = (0.0, 15.0, 180.0, 345.0)  # azimuths free of cowls and panels
NOSE_NODES = {"nose_cone"}
COMPONENT = {5126: ("<f4", 1), 5123: ("<u2", 1), 5125: ("<u4", 1), 5121: ("u1", 1)}
COUNT = {"SCALAR": 1, "VEC2": 2, "VEC3": 3}


def triangles(path: Path, names: set[str] | None = None, skip: set[str] | None = None) -> np.ndarray:
    """World triangles (n, 3, 3) of the named mesh nodes. Our models carry no node transforms below the root (tested in test_dragon_spec)."""
    g = json.loads(path.read_text(encoding="utf-8"))
    data = (path.parent / g["buffers"][0]["uri"]).read_bytes()

    def accessor(i):
        a = g["accessors"][i]
        v = g["bufferViews"][a["bufferView"]]
        kind, _ = COMPONENT[a["componentType"]]
        count = COUNT[a["type"]]
        off = v.get("byteOffset", 0) + a.get("byteOffset", 0)
        return np.frombuffer(data, kind, a["count"] * count, off).reshape(-1, count)

    out = []
    for node in g["nodes"]:
        name = node.get("name")
        if "mesh" not in node or (names and name not in names) or (skip and name in skip):
            continue
        for prim in g["meshes"][node["mesh"]]["primitives"]:
            pos = accessor(prim["attributes"]["POSITION"]).astype(float)
            idx = accessor(prim["indices"]).astype(int).ravel()
            out.append(pos[idx].reshape(-1, 3, 3))
    return np.vstack(out)


def section(tris: np.ndarray, x: float) -> np.ndarray:
    """Segments (n, 2, 2) in (y, z) of the triangles with the plane X = x."""
    k = (tris[:, :, 0].min(1) <= x) & (tris[:, :, 0].max(1) >= x)
    segs = []
    for tri in tris[k]:
        d = tri[:, 0] - x
        pts = []
        for a, b in ((0, 1), (1, 2), (2, 0)):
            if (d[a] <= 0.0) != (d[b] <= 0.0):
                pts.append(tri[a, 1:] + d[a] / (d[a] - d[b]) * (tri[b, 1:] - tri[a, 1:]))
        if len(pts) == 2:
            segs.append(pts)
    return np.array(segs) if segs else np.zeros((0, 2, 2))


def outer_radius(tris: np.ndarray, x: float, az_deg: float) -> float:
    """Largest r of the surface along the ray at (x, az); NaN when nothing is hit."""
    s = section(tris, x)
    if len(s) == 0:
        return math.nan
    a = math.radians(az_deg)
    e, n = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
    d0, d1 = s[:, 0] @ n, s[:, 1] @ n
    m = (d0 <= 0.0) != (d1 <= 0.0)
    if not m.any():
        return math.nan
    hit = s[m, 0] + (s[m, 1] - s[m, 0]) * (d0[m] / (d0[m] - d1[m]))[:, None]
    r = hit @ e
    r = r[r > 0.0]
    return float(r.max()) if len(r) else math.nan


def lerp(table, x):
    return float(np.interp(x, [p[0] for p in table], [p[1] for p in table]))


def lobes_deg() -> list[float]:
    with open(VEHICLE, "rb") as f:
        v = tomllib.load(f)
    cap = next(s for s in v["stage"] if s["id"] == "capsule")
    group = next(g for g in cap["engine_group"] if g["id"] == "superdraco")
    a = group["angles_deg"]
    return [0.5 * (a[i] + a[i + 1]) for i in range(0, len(a), 2)]


def deviations(models: Path = MODELS, detail_path: Path = DETAIL) -> dict:
    """{table: (largest deviation m, where)}; a missing surface counts as infinite."""
    with open(detail_path, "rb") as f:
        d = tomllib.load(f)
    c, t = d["capsule"], d["trunk"]
    with open(VEHICLE, "rb") as f:
        vehicle = tomllib.load(f)
    trunk_length = next(s for s in vehicle["stage"] if s["id"] == "trunk")["length_m"]
    cap_file = models / "dragon_capsule.gltf"
    closed = triangles(cap_file, skip={"window", "hatch", "seams", "draco_pod", "hinge_fairing", "deck", "recess", "superdraco_nozzle"})
    hinge = np.array([c["nose_hinge_x_m"], c["nose_hinge_r_m"] * math.cos(math.radians(c["nose_hinge_azimuth_deg"])),
                      c["nose_hinge_r_m"] * math.sin(math.radians(c["nose_hinge_azimuth_deg"]))])
    nose = triangles(models / "dragon_nose.gltf", NOSE_NODES) + hinge
    out = {}

    def worst(key, items):
        dev, where = 0.0, None
        for here, expected, got in items:
            e = abs(got - expected) if not math.isnan(got) else math.inf
            if e > dev:
                dev, where = e, here
        out[key] = (dev, where)

    body_points = []
    for x, r in c["profile_m"][1:-1]:
        for az in PLAIN_AZIMUTHS_DEG:
            seam_x = c["seam_axis_x_m"] + c["seam_slope"] * r * math.cos(math.radians(az))
            if abs(x - seam_x) < 0.03:  # the seam's gap
                continue
            tris = closed if x < seam_x else nose
            body_points.append(((x, az), r, outer_radius(tris, x, az)))
    worst("body_profile", body_points)
    lobes = lobes_deg()
    crown = []
    for x, r in c["hood_outer_m"]:
        if x <= c["bay_top_x_m"] + 0.05 or x > 3.5:
            continue
        for pod in lobes:
            crown.append(((x, pod), r, outer_radius(closed, x, pod)))
    worst("cowl_crown", crown)
    floor = []
    for x, r in c["bay_floor_m"]:
        if 0.8 < x < c["bay_top_x_m"] - c["bay_corner_drop_m"]:
            for pod in lobes:
                floor.append(((x, pod), r, outer_radius(closed, x, pod)))
    worst("bay_floor", floor)
    # The nose's seam: its outer rim lies on the typed plane, and the tip on the typed length.
    verts = nose.reshape(-1, 3)
    seam = []
    for az in range(0, 360, 15):
        a = np.degrees(np.arctan2(verts[:, 2], verts[:, 1]))
        m = (np.abs((a - az + 180.0) % 360.0 - 180.0) < 2.0) & (np.hypot(verts[:, 1], verts[:, 2]) > 0.3) & (verts[:, 0] > c["seam_axis_x_m"] - 0.4)
        m &= np.abs(np.hypot(verts[:, 1], verts[:, 2]) - np.interp(verts[:, 0], [q[0] for q in c["profile_m"]], [q[1] for q in c["profile_m"]])) < 0.005  # the outer skin only (not the lip, not the arm)
        shell = verts[m]
        if len(shell) == 0:
            seam.append(((az,), 0.0, math.nan))
            continue
        low = shell[np.argmin(shell[:, 0] - c["seam_slope"] * shell[:, 1])]
        plane = c["seam_axis_x_m"] + c["seam_slope"] * low[1]
        seam.append(((az,), plane, float(low[0])))
    worst("nose_seam", seam)
    out["nose_tip"] = (abs(float(verts[:, 0].max()) - c["profile_m"][-1][0]), None)
    nearest = float(np.min(np.linalg.norm(verts - hinge, axis=1)))
    out["hinge_point"] = (nearest, None)
    trunk = triangles(models / "dragon_trunk.gltf", {"trunk"})
    fin = []
    for (x, r) in t["fin_outline_m"][2:-2]:
        for k in range(int(t["fin_count"])):
            az = t["fin_first_azimuth_deg"] + k * 360.0 / t["fin_count"]
            fin.append(((x, az), r, outer_radius(trunk, x + trunk_length, az)))
    worst("fin_outline", fin)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--models", default=str(MODELS))
    args = ap.parse_args()
    dev = deviations(Path(args.models))
    for key, (value, where) in dev.items():
        print(f"{key:14s} {value:7.4f} m  at {where}")
    print(f"largest {max(v for v, _ in dev.values()):.4f} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
