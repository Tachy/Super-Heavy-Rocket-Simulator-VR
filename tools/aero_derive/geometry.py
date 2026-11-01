"""The built vehicle models as triangles in body coordinates, and the secured areas read from them.

Body frame = the manifest's: X station axis (nose +X), (Y, Z) = r (cos a, sin a). Stack coordinates: booster station 0 at
X = 0, the ship's stage-local stations shifted by the booster's length. Placement ports `game/vehicle/vehicle_models.gd`
(`flap_transform`, `fin_transform`, `engine_transform`).

Left out on purpose: the grid fin's `plate` (the renderer's far stand-in, one opaque polygon: not a physical part).

    python tools/aero_derive/geometry.py [vehicle_id]    prints the secured-area table, writes
                                                         screenshots/vehicle-physics/secured_areas.csv
"""
from __future__ import annotations

import csv
import json
import math
import struct
import sys
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

import newtonian

ROOT = Path(__file__).resolve().parents[2]
MODEL_ROOT = ROOT / "data" / "models"
CONFIG_ROOT = ROOT / "config" / "vehicles"
OUT_DIR = ROOT / "screenshots" / "vehicle-physics"
sys.path.insert(0, str(ROOT / "tools" / "structures" / "blender"))
from hull_profile import Profile  # noqa: E402

NON_PHYSICAL_PARTS = {"plate"}  # far-distance render stand-in
HULL_LATERAL_MIN_RADIAL_NORMAL = 0.2  # a mesh face with more radial normal than this counts as lateral


def _horn(u: float, soft: float) -> float:
    """Copy of `horn` in tools/structures/blender/gen_grid_fin.py (that module needs Blender)."""
    return (1.0 - soft) * math.sin(math.pi * u) + soft * 0.5 * (1.0 - math.cos(2.0 * math.pi * u))


def read_gltf(path: Path) -> dict:
    """{node name: (n, 3, 3) float64 triangles}; the files carry no node transforms."""
    gltf = json.loads(path.read_text(encoding="utf-8"))
    blob = (path.parent / gltf["buffers"][0]["uri"]).read_bytes()
    parts = {}
    for node in gltf["nodes"]:
        if "mesh" not in node:
            continue
        assert "matrix" not in node and "translation" not in node and "rotation" not in node, path
        chunks = []
        for prim in gltf["meshes"][node["mesh"]]["primitives"]:
            acc = gltf["accessors"][prim["attributes"]["POSITION"]]
            view = gltf["bufferViews"][acc["bufferView"]]
            start = view.get("byteOffset", 0) + acc.get("byteOffset", 0)
            pos = np.frombuffer(blob, dtype="<f4", count=acc["count"] * 3, offset=start).reshape(-1, 3).astype(float)
            iacc = gltf["accessors"][prim["indices"]]
            iview = gltf["bufferViews"][iacc["bufferView"]]
            kind = {5123: "<u2", 5125: "<u4", 5121: "u1"}[iacc["componentType"]]
            idx = np.frombuffer(blob, dtype=kind, count=iacc["count"],
                                offset=iview.get("byteOffset", 0) + iacc.get("byteOffset", 0))
            chunks.append(pos[idx.reshape(-1, 3)])
        parts[node["name"]] = np.concatenate(chunks)
    return parts


def shoelace(u, v) -> float:
    u, v = np.asarray(u, float), np.asarray(v, float)
    return 0.5 * abs(float(np.dot(u, np.roll(v, -1)) - np.dot(v, np.roll(u, -1))))


def rotation(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * k @ k


@dataclass
class Vehicle:
    vehicle_id: str
    config: dict
    detail: dict
    manifest: dict
    hulls: dict  # stage id -> {part: tris}
    flap_models: dict  # kind -> {part: tris}
    fin_model: dict
    engine_models: dict
    booster_length_m: float = field(init=False)

    def __post_init__(self):
        self.booster_length_m = self.stage("booster")["length_m"]

    def stage(self, stage_id: str) -> dict:
        index = {"booster": 0, "ship": 1}[stage_id]
        return self.config["stage"][index]

    def stage_base_m(self, stage_id: str) -> float:
        return 0.0 if stage_id == "booster" else self.booster_length_m

    # ---------------------------------------------------------------- posed parts
    def flap_triangles(self, entry: dict, deflection_deg: float) -> dict:
        """Port of vehicle_models.gd flap_transform; stage-local to the ship."""
        sign = int(entry["rotation_sign_about_plus_x"])
        azimuth = math.radians(entry["hinge_azimuth_deg"])
        lean = math.radians(entry["drawn_hinge_lean_deg"])
        x = np.array([1.0, 0.0, 0.0])
        e_r = np.array([0.0, math.cos(azimuth), math.sin(azimuth)])
        hinge_axis = x * math.cos(lean) - e_r * math.sin(lean)
        normal = x * math.sin(lean) + e_r * math.cos(lean)
        stow = np.cross(x, e_r) * (-float(sign))
        rest = np.column_stack([hinge_axis, normal, stow])
        turn = rotation(hinge_axis, sign * math.radians(deflection_deg))
        pivot = x * entry["drawn_hinge_station_aft_m"] + e_r * entry["drawn_hinge_radius_aft_m"]
        matrix = turn @ rest
        return {part: tris @ matrix.T + pivot for part, tris in self.flap_models[entry["kind"]].items()}

    def fin_triangles(self, azimuth_deg: float, deflection_deg: float) -> dict:
        """Port of vehicle_models.gd fin_transform; booster coordinates."""
        azimuth = math.radians(azimuth_deg)
        hull_radius = self.stage("booster")["diameter_m"] / 2.0
        station = self.stage("booster")["grid_fins"]["station_m"]
        pivot = np.array([station, math.cos(azimuth) * hull_radius, math.sin(azimuth) * hull_radius])
        matrix = rotation((1.0, 0.0, 0.0), azimuth) @ rotation((0.0, 1.0, 0.0), math.radians(deflection_deg))
        catch = self.manifest["look"]["catch_fin_azimuths_deg"]
        out = {}
        for part, tris in self.fin_model.items():
            if part in NON_PHYSICAL_PARTS:
                continue
            if part == "bearing" and not any(abs((azimuth_deg - c + 180.0) % 360.0 - 180.0) < 1e-6 for c in catch):
                continue
            out[part] = tris @ matrix.T + pivot
        return out

    def booster_engine_positions(self) -> list:
        """(y, z) of each booster engine: `angles_deg`, or evenly spaced from 0 (as the groups without angles)."""
        out = []
        for group in self.stage("booster")["engine_group"]:
            angles = group.get("angles_deg") or [360.0 * i / group["count"] for i in range(group["count"])]
            for a in angles:
                out.append((group["ring_radius_m"] * math.cos(math.radians(a)), group["ring_radius_m"] * math.sin(math.radians(a))))
        return out

    def triangles(self, stages=("booster", "ship"), flap_deg: float = 90.0, fin_deg: float = 0.0,
                  engines: bool = False, flaps: bool = True, fins: bool = True):
        """All physical triangles of the stages, with part labels: (tris, (part names, part id per triangle))."""
        chunks, names, ids = [], [], []

        def add(name: str, tris: np.ndarray) -> None:
            names.append(name)
            chunks.append(tris)
            ids.append(np.full(len(tris), len(names) - 1, dtype=np.int64))

        for stage_id in stages:
            base = np.array([self.stage_base_m(stage_id), 0.0, 0.0])
            for part, tris in self.hulls[stage_id].items():
                add(f"{stage_id}/{part}", tris + base)
            if stage_id == "ship" and flaps:
                for entry in self.manifest["flaps"]["ship"]:
                    for part, tris in self.flap_triangles(entry, flap_deg).items():
                        add(f"flap/{entry['id']}/{part}", tris + base)
            if stage_id == "booster":
                if fins:
                    for az in self.stage("booster")["grid_fins"]["azimuths_deg"]:
                        for part, tris in self.fin_triangles(az, fin_deg).items():
                            add(f"fin/{az:g}/{part}", tris)
                if engines:
                    for i, (y, z) in enumerate(self.booster_engine_positions()):
                        for part, tris in self.engine_models["raptor3"].items():
                            add(f"engine/{i}/{part}", tris + np.array([0.0, y, z]))
        return np.concatenate(chunks), (names, np.concatenate(ids))

    # ---------------------------------------------------------------- secured areas
    def _profile(self, stage_id: str) -> Profile:
        stage = self.stage(stage_id)
        hull = {**self.detail["hull"]}
        chamfer = self.detail["ship"]["engine_bay"]["aft_chamfer_m"] if stage_id == "ship" else None
        return Profile({"length_m": stage["length_m"], "diameter_m": stage["diameter_m"],
                        "nose_length_m": stage["aero"].get("nose_length_m", 0.0)}, hull, chamfer)

    def wetted_end_station_m(self, stage_id: str) -> float:
        """Booster: the barrel ends at the dome under the truss (the dome is not on the outer skin)."""
        profile = self._profile(stage_id)
        return profile.x_barrel_end if stage_id == "booster" else profile.L

    def hull_wetted_area_m2(self, stage_id: str) -> float:
        """Lateral area of the drawn lathe: 2 pi r ds along the profile (chamfer included, end caps not)."""
        profile = self._profile(stage_id)
        xs = np.linspace(0.0, self.wetted_end_station_m(stage_id), 20001)
        rs = np.array([profile.base(x) for x in xs])
        return float(np.sum(2.0 * math.pi * 0.5 * (rs[1:] + rs[:-1]) * np.hypot(np.diff(xs), np.diff(rs))))

    def hull_mesh_wetted_area_m2(self, stage_id: str) -> float:
        """The same from the mesh: lateral faces up to the wetted end, as a cross-check of the profile."""
        tris = self.hulls[stage_id]["hull"]
        normal = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
        area = 0.5 * np.linalg.norm(normal, axis=1)
        unit = normal / np.maximum(2.0 * area, 1e-12)[:, None]
        centre = tris.mean(axis=1)
        keep = (np.hypot(unit[:, 1], unit[:, 2]) > HULL_LATERAL_MIN_RADIAL_NORMAL) & (centre[:, 0] <= self.wetted_end_station_m(stage_id) + 1e-6)
        return float(area[keep].sum())

    def flap_area_m2(self, kind: str) -> float:
        """Planform from the detail file's outline polygon (the generator's input)."""
        flap = self.detail["flap"][kind]
        return shoelace(flap["outline_u_m"], flap["outline_v_m"])

    def flap_mesh_area_m2(self, kind: str) -> float:
        """Planform from the built panel: half the area of all its faces projected on the hinge plane (X, Z)."""
        tris = self.flap_models[kind]["panel"][:, :, [0, 2]]
        a, b = tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]
        return 0.25 * float(np.abs(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]).sum())

    def flap_span_m(self, kind: str) -> float:
        return max(self.detail["flap"][kind]["outline_v_m"])

    def grid_fin_gross_area_m2(self) -> float:
        fin = self.detail["grid_fin"]
        return shoelace(fin["outline_z_m"], fin["outline_y_m"])

    def grid_fin_blocked_area_m2(self, cell_m: float = 0.004) -> dict:
        """Area blocked looking along the cells at d = 0, per part (first hit, render plate excluded)."""
        names, chunks, ids = [], [], []
        for part, tris in self.fin_model.items():
            if part in NON_PHYSICAL_PARTS or part in ("bearing", "shaft"):
                continue  # the bearing is on two fins only, the shaft stands inside the pylon
            names.append(part)
            chunks.append(tris)
            ids.append(np.full(len(tris), len(names) - 1, dtype=np.int64))
        hit = newtonian.first_hit(np.concatenate(chunks), (1.0, 0.0, 0.0), cell_m, np.concatenate(ids), names)
        return hit.frontal_by_part_m2()

    def grid_fin_wall_share(self) -> float:
        """Share of the fin's area between its solid parts that the lattice walls block: 1 - (1 - w/p)^2.

        The lattice walls are zero-thickness double-sided sheets in the mesh (`_saddle_wall` in gen_grid_fin.py), so no
        ray test can see `wall_m`; only the cross rib and the walls' cusp slopes are hit. The thickness comes from the
        detail file (an upper bound, not resolved in any frame)."""
        fin = self.detail["grid_fin"]
        return 1.0 - (1.0 - fin["wall_m"] / fin["cell_pitch_m"]) ** 2

    def grid_fin_open_fraction(self, cell_m: float = 0.004) -> float:
        """Open share of the gross outline along the cells at d = 0: solid parts from the mesh, walls from wall_m / pitch."""
        gross = self.grid_fin_gross_area_m2()
        solid = sum(self.grid_fin_blocked_area_m2(cell_m).values())
        return (gross - solid) * (1.0 - self.grid_fin_wall_share()) / gross

    def grid_fin_mean_depth_m(self, face: str) -> float:
        """Mean depth of a cell wall when that face is met first: the thickness less the scoop's mean (drawn `horn`)."""
        fin = self.detail["grid_fin"]
        dip = fin["cusp_dip_aft_m"] if face == "aft" else fin["cusp_dip_fwd_m"]
        mean = float(np.mean([_horn(u, fin["cusp_soft"]) for u in np.linspace(0.0, 1.0, 2001)]))
        return fin["thickness_m"] - dip * mean

    def hot_stage_open_height_m(self) -> float:
        """Open band of the hot-stage: barrel end (station L - dome_start_m) to the closed ring under the ship."""
        return self.detail["hull"]["dome_start_m"] - self.detail["booster"]["hot_stage_ring"]["height_m"]

    def outer_radius_m(self, x_m) -> np.ndarray:
        """Radius of the bare hull at a stack station: the booster's barrel (no dome: the band's outer line), the ship's lathe."""
        booster, ship = self._profile("booster"), self._profile("ship")
        base = self.booster_length_m
        grid = np.arange(0.0, base + ship.L + 0.05, 0.05)
        radius = [booster.base(min(x, booster.x_barrel_end)) if x < base else ship.base(min(x - base, ship.L)) for x in grid]
        return np.interp(x_m, grid, radius)

    def grid_fin_wall_edge_cos2(self, face: str) -> float:
        """Mean cos^2(phi) of a wall's end face along its length. Forward face flat at the drawn dip 0 -> 1; aft face
        scooped by `cusp_dip_aft_m` with a saddle per pitch (the drawn `horn` profile, gen_grid_fin.py)."""
        fin = self.detail["grid_fin"]
        dip = fin["cusp_dip_aft_m"] if face == "aft" else fin["cusp_dip_fwd_m"]
        if dip <= 0.0:
            return 1.0
        s = np.linspace(0.0, fin["cell_pitch_m"], 2001)
        x = dip * np.array([_horn(v, fin["cusp_soft"]) for v in s / fin["cell_pitch_m"]])
        slope = np.gradient(x, s)
        return float(np.mean(1.0 / (1.0 + slope * slope)))


def load_vehicle(vehicle_id: str) -> Vehicle:
    with open(CONFIG_ROOT / f"{vehicle_id}.toml", "rb") as f:
        config = tomllib.load(f)
    with open(CONFIG_ROOT / "detail" / f"{vehicle_id}.toml", "rb") as f:
        detail = tomllib.load(f)
    folder = MODEL_ROOT / vehicle_id
    manifest = json.loads((folder / "manifest.json").read_text(encoding="utf-8"))
    return Vehicle(vehicle_id, config, detail, manifest,
                   {"booster": read_gltf(folder / "booster.gltf"), "ship": read_gltf(folder / "ship.gltf")},
                   {"aft": read_gltf(folder / "flap_aft.gltf"), "fwd": read_gltf(folder / "flap_fwd.gltf")},
                   read_gltf(folder / "grid_fin.gltf"),
                   {"raptor3": read_gltf(folder / "raptor.gltf")})


def secured_areas(vehicle: Vehicle) -> list:
    """Rows: quantity, drawn value, its error, config value, drawn vs config, source of the drawn value."""
    ship, booster = vehicle.stage("ship"), vehicle.stage("booster")
    flap_cfg = {f["id"]: f for f in ship["flap"]}
    aft, fwd = flap_cfg["aft_right"], flap_cfg["forward_right"]
    fin = vehicle.detail["grid_fin"]
    gross = vehicle.grid_fin_gross_area_m2()
    open_fraction = vehicle.grid_fin_open_fraction()
    analytic_open = (1.0 - fin["wall_m"] / fin["cell_pitch_m"]) ** 2
    rows = []

    def add(name, value, err_abs, config_value, source):
        rows.append({"quantity": name, "drawn": value, "error_abs": err_abs, "config": config_value,
                     "drawn_over_config": (value / config_value) if config_value else None, "source": source})

    # Flap errors [E]: the mesh-outline difference plus the 1:100 model's own scale uncertainty (1 % in length = 2 % in area).
    for kind, cfg in (("aft", aft), ("fwd", fwd)):
        area, mesh = vehicle.flap_area_m2(kind), vehicle.flap_mesh_area_m2(kind)
        add(f"flap {kind} area [m^2]", area, abs(area - mesh) + 0.02 * area, cfg["area_m2"],
            f"detail outline, STL 1:100 [E]; mesh projection {mesh:.2f}")
        add(f"flap {kind} span [m]", vehicle.flap_span_m(kind), 0.01 * vehicle.flap_span_m(kind), cfg["span_m"], "detail outline [E]")
    add("grid fin gross area [m^2]", gross, 0.02 * gross, booster["grid_fins"]["area_m2"],
        "detail outline, STL 1:100 [E]; 102.9 cells x 0.35^2 = 12.60")
    blocked = vehicle.grid_fin_blocked_area_m2()
    add("grid fin open fraction [-]", open_fraction, 0.04, None,
        f"solid parts by first hit along the cells at d = 0, then the walls (1 - w/p)^2 = {analytic_open:.3f} (mesh walls are zero-thick sheets; wall 0.04 m is an upper bound [E]); "
        + ", ".join(f"{k} {v:.2f}" for k, v in sorted(blocked.items())) + " m^2")
    add("grid fin open area [m^2]", open_fraction * gross, 0.04 * gross, None, "open fraction x gross")
    for stage_id in ("booster", "ship"):
        area, mesh = vehicle.hull_wetted_area_m2(stage_id), vehicle.hull_mesh_wetted_area_m2(stage_id)
        cyl = math.pi * vehicle.stage(stage_id)["diameter_m"] * vehicle.stage(stage_id)["length_m"]
        add(f"wetted hull {stage_id} [m^2]", area, abs(area - mesh) + 0.01 * area, None,
            f"lathe integral to station {vehicle.wetted_end_station_m(stage_id):.2f}; mesh {mesh:.1f}; plain cylinder pi D L {cyl:.1f}")
    return rows


def write_secured_areas(vehicle: Vehicle, path: Path | None = None) -> list:
    rows = secured_areas(vehicle)
    path = path or OUT_DIR / "secured_areas.csv"
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    return rows


def main() -> None:
    vehicle = load_vehicle(sys.argv[1] if len(sys.argv) > 1 else "starship_block3")
    print("| Quantity | Drawn | Error | Config | Drawn / config | Source |\n|---|---|---|---|---|---|")
    for r in write_secured_areas(vehicle):
        cfg = "-" if r["config"] is None else f"{r['config']:g}"
        ratio = "-" if r["drawn_over_config"] is None else f"{r['drawn_over_config']:.3f}"
        print(f"| {r['quantity']} | {r['drawn']:.3f} | +-{r['error_abs']:.3f} | {cfg} | {ratio} | {r['source']} |")


if __name__ == "__main__":
    main()
