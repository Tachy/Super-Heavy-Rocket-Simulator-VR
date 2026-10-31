"""Vehicle model spec: the vehicle config (placement, sizes) + the visual detail file -> one JSON-able dict.

`load(vehicle_id) -> dict` reads `config/vehicles/<id>.toml`, `config/vehicles/detail/<id>.toml` and the engines'
`config/engines/*.toml`, validates the detail file and returns what the Blender generators need (they run on Blender's
Python 3.10 without `tomllib`, so the driver resolves everything here and passes it as JSON). Nothing of the physics
config is duplicated into the detail file or changed here.
"""
from __future__ import annotations

import json
import math
import re
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
VEHICLES = ROOT / "config" / "vehicles"
ENGINES = ROOT / "config" / "engines"


class VehicleSpecError(ValueError):
    """The detail file misses a key or has a value of the wrong type or unit."""


DETAIL_TABLES = {
    "hull": ("segments", "ring_pitch_m", "weld_height_m", "weld_half_width_m", "dent_slope_rms", "nose_tip_radius_m",
             "nose_tip_cap_m", "nose_drawn_length_m", "base_chamfer_m", "dome_start_m", "dome_height_m"),
    "booster.hot_stage": ("height_m", "flange_height_m", "flange_proud_m"),
    "booster.hot_stage_ring": ("height_m", "inner_radius_m"),
    "booster.top_truss": ("tooth_count", "height_m", "tube_radius_m", "post_height_m", "taper_top",
                          "taper_foot"),
    "booster.skirt": ("base_ring_radius_m", "ring_stations_m", "ring_proud_m", "ring_thickness_m", "tooth_count",
                      "tooth_height_m", "tooth_station_m", "tooth_proud_m", "port_count", "port_width_m",
                      "port_height_m", "port_proud_m", "port_station_m"),
    "booster.raceway": ("azimuth_deg", "pill_width_m", "pill_length_m", "pill_pitch_m", "height_m", "rail_radius_m"),
    "booster.strakes": ("ridge_azimuth_deg", "from_m", "to_m", "height_m", "base_width_m", "top_width_m", "chamfer_m"),
    "booster.grid_fin_fairing": ("length_frac", "width_m", "height_m"),
    "grid_fin": ("across_flats_m", "thickness_m", "cell_pitch_m", "wall_m", "frame_m", "shaft_radius_m",
                 "shaft_inset_m", "catch_hook_radius_m", "catch_hook_count", "catch_hook_pitch_m",
                 "edge_tooth_pitch_m", "edge_tooth_depth_m", "catch_fin_azimuths_deg"),
    "flap": ("root_thickness_m", "tip_thickness_m", "hinge_radius_m", "lug_length_m", "lug_height_m"),
    "flap.aft": ("hinge_drawn_length_m", "outline_u_m", "outline_v_m", "lug_u_m"),
    "flap.fwd": ("hinge_drawn_length_m", "outline_u_m", "outline_v_m", "lug_u_m"),
    "catch_pin": ("radius_m", "head_radius_m", "head_length_m"),
    "ship.chine": ("width_m", "height_m", "fade_m", "first_station_m", "last_station_m"),
    "ship.raceway": ("azimuth_from_belly_deg", "radius_m", "first_station_m", "last_station_m", "foot_width_m",
                     "foot_length_m", "foot_height_m"),
    "ship.ports": ("port_count", "placed_at_m", "azimuth_from_belly_deg", "spread_deg", "port_radius_m",
                   "port_height_m"),
    "ship.engine_bay": ("aft_chamfer_m", "wall_inner_radius_m", "plate_station_m", "plate_radius_m",
                        "plate_cone_station_m", "ring_stations_m", "ring_height_m", "ring_depth_m", "stringer_count",
                        "stringer_width_m", "stringer_depth_m"),
    "ship.tiles": ("hex_across_flats_m", "gap_m", "coverage_station_m", "coverage_half_angle_deg"),
}
ENGINE_KEYS = ("file", "bell_length_m", "throat_diameter_m", "contour_exponent", "wall_m", "mount_height_m",
               "mount_radius_frac", "segments")
SIGNED_KEYS = ("azimuth_from_belly_deg", "azimuth_deg", "ridge_azimuth_deg", "ridge_azimuth_deg", "outline_u_m", "outline_v_m", "catch_fin_azimuths_deg")  # may be zero / negative; others > 0


def _table(data: dict, dotted: str) -> dict:
    cur = data
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            raise VehicleSpecError(f"detail file: missing table [{dotted}]")
        cur = cur[part]
    return cur


def _number(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def validate_detail(detail: dict) -> None:
    for name, keys in DETAIL_TABLES.items():
        t = _table(detail, name)
        for k in keys:
            if k not in t:
                raise VehicleSpecError(f"[{name}]: missing key '{k}'")
            v = t[k]
            if isinstance(v, list):
                if not v or not all(_number(x) and (x >= 0.0 or k in SIGNED_KEYS) for x in v):
                    raise VehicleSpecError(f"[{name}].{k}: must be a non-empty list of numbers >= 0")
            elif not _number(v) or (v <= 0.0 and k not in SIGNED_KEYS):
                raise VehicleSpecError(f"[{name}].{k}: must be a positive number, got {v!r}")
    tiles = detail["ship"]["tiles"]
    if len(tiles["coverage_station_m"]) != len(tiles["coverage_half_angle_deg"]):
        raise VehicleSpecError("[ship.tiles]: coverage tables differ in length")
    if sorted(tiles["coverage_station_m"]) != list(tiles["coverage_station_m"]):
        raise VehicleSpecError("[ship.tiles].coverage_station_m: must be ascending")
    if any(a > 180.0 for a in tiles["coverage_half_angle_deg"]):
        raise VehicleSpecError("[ship.tiles].coverage_half_angle_deg: above 180")
    for kind in ("aft", "fwd"):
        f = detail["flap"][kind]
        if len(f["outline_u_m"]) != len(f["outline_v_m"]) or len(f["outline_u_m"]) < 3:
            raise VehicleSpecError(f"[flap.{kind}]: outline_u_m / outline_v_m: equal-length lists of >= 3 points")
    bay = detail["ship"]["engine_bay"]
    if len(bay["plate_radius_m"]) != len(bay["plate_cone_station_m"]):
        raise VehicleSpecError("[ship.engine_bay]: plate_radius_m / plate_cone_station_m: equal-length lists")
    recess = _table(detail, "ship").get("engine_recess", {})
    for eid, e in _table(detail, "engines").items():
        if not _number(recess.get(eid)) or recess[eid] < 0.0:
            raise VehicleSpecError(f"[ship.engine_recess].{eid}: must be a number >= 0")
        for k in ENGINE_KEYS:
            if k not in e:
                raise VehicleSpecError(f"[engines.{eid}]: missing key '{k}'")
            if k != "file" and (not _number(e[k]) or e[k] <= 0.0):
                raise VehicleSpecError(f"[engines.{eid}].{k}: must be a positive number")
        if not str(e["file"]).endswith(".gltf"):
            raise VehicleSpecError(f"[engines.{eid}].file: must be a .gltf name")


_VALUE_LINE = re.compile(r"^\s*[A-Za-z_][\w.]*\s*=")
_TAG = re.compile(r"#.*\[(E|S)\]")


def value_lines_without_tag(path) -> list[tuple[int, str]]:
    """(line number, text) of every `key = value` line without an `# [E]` / `# [S]` comment (single-line values)."""
    bad = []
    for n, line in enumerate(Path(path).read_text(encoding="utf-8").splitlines(), 1):
        if _VALUE_LINE.match(line) and not _TAG.search(line):
            bad.append((n, line))
    return bad


def flap_rotation_sign(hinge_azimuth_deg: float, belly_azimuth_deg: float) -> int:
    """Sign s such that the flap panel at deflection d is the stowed pose rotated by s * d about +X (station axis).

    Mirrors core/src/aero/flap.cpp: span(d) = stow cos d + e_r sin d with stow = X x e_r, negated when it points to the
    belly side. X x e_r rotated by +t about X is -e_r (so the unflipped stow needs -d), the flipped stow needs +d.
    """
    h, b = math.radians(hinge_azimuth_deg), math.radians(belly_azimuth_deg)
    stow = (-math.sin(h), math.cos(h))  # components (y, z) of X x e_r
    flipped = stow[0] * math.cos(b) + stow[1] * math.sin(b) > 0.0
    return 1 if flipped else -1


def load(vehicle_id: str, vehicles_dir: Path = VEHICLES, engines_dir: Path = ENGINES) -> dict:
    with open(vehicles_dir / f"{vehicle_id}.toml", "rb") as f:
        vehicle = tomllib.load(f)
    with open(vehicles_dir / "detail" / f"{vehicle_id}.toml", "rb") as f:
        detail = tomllib.load(f)
    validate_detail(detail)
    stages = {}
    engine_ids = set()
    for s in vehicle["stage"]:
        aero = s.get("aero", {})
        belly = float(aero.get("belly_azimuth_deg", 270.0))
        flaps = []
        for fl in s.get("flap", []):
            flaps.append({
                "id": fl["id"], "hinge_azimuth_deg": float(fl["hinge_azimuth_deg"]),
                "station_fwd_m": float(fl["station_fwd_m"]), "station_aft_m": float(fl["station_aft_m"]),
                "span_m": float(fl["span_m"]), "area_m2": float(fl["area_m2"]),
                "kind": "fwd" if fl["id"].startswith("forward") else "aft",
                "rotation_sign_about_plus_x": flap_rotation_sign(float(fl["hinge_azimuth_deg"]), belly),
            })
        engines = []
        for g in s.get("engine_group", []):
            engine_ids.add(g["engine"])
            count = int(g.get("count", 1))
            angles = g.get("angles_deg") or [360.0 * i / count for i in range(count)]
            if len(angles) != count:
                raise VehicleSpecError(f"{s['id']}.{g['id']}: angles_deg has {len(angles)} entries, count {count}")
            engines.append({"group": g["id"], "engine": g["engine"], "ring_radius_m": float(g["ring_radius_m"]),
                            "station_m": float(g.get("station_m", 0.0)), "angles_deg": [float(a) for a in angles]})
        stages[s["id"]] = {
            "length_m": float(s["length_m"]), "diameter_m": float(s["diameter_m"]),
            "nose_length_m": float(aero.get("nose_length_m", 0.0)), "belly_azimuth_deg": belly,
            "grid_fins": s.get("grid_fins"), "catch_pins": s.get("catch_pins"), "flaps": flaps, "engines": engines,
        }
    for k in ("booster", "ship"):
        if k not in stages:
            raise VehicleSpecError(f"{vehicle_id}: no stage '{k}'")
    engine_spec = {}
    for eid in sorted(engine_ids):
        with open(engines_dir / f"{eid}.toml", "rb") as f:
            eng = tomllib.load(f)
        if eid not in detail["engines"]:
            raise VehicleSpecError(f"detail file: no [engines.{eid}] for the engine used by {vehicle_id}")
        engine_spec[eid] = {"nozzle_exit_diameter_m": float(eng["performance"]["nozzle_exit_diameter_m"]),
                            **detail["engines"][eid]}
    for k in ("fwd", "aft"):  # one model per kind: every flap of a kind must share its size
        sizes = {(round(f["station_fwd_m"] - f["station_aft_m"], 6), f["span_m"])
                 for st in stages.values() for f in st["flaps"] if f["kind"] == k}
        if len(sizes) > 1:
            raise VehicleSpecError(f"flaps of kind '{k}' differ in size: {sorted(sizes)}")
    return {"vehicle_id": vehicle_id, "stages": stages, "engines": engine_spec, "detail": detail}


def dumps(spec: dict) -> str:
    return json.dumps(spec, indent=1, sort_keys=True) + "\n"
