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
import sys
import tomllib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "blender"))
import hull_profile  # noqa: E402  (pure Python, shared with the Blender generators)

ROOT = Path(__file__).resolve().parents[2]
VEHICLES = ROOT / "config" / "vehicles"
ENGINES = ROOT / "config" / "engines"




GENERATORS_TOML = Path(__file__).resolve().parent / "blender" / "generators.toml"


def generator_table(vehicle_id: str, path: Path = GENERATORS_TOML) -> dict:
    """The vehicle's row of blender/generators.toml: kind, scripts, bakers, generators."""
    with open(path, "rb") as f:
        table = tomllib.load(f)
    if vehicle_id not in table:
        raise VehicleSpecError(f"{vehicle_id}: no entry in {path.name} (known: {', '.join(sorted(table))})")
    return table[vehicle_id]


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
    "grid_fin": ("outline_z_m", "outline_y_m", "thickness_m", "cell_pitch_m", "wall_m", "frame_m", "band_m",
                 "edge_tooth_depth_m", "cusp_dip_fwd_m", "cusp_dip_aft_m", "cusp_soft", "cusp_steps", "rib_y_m", "rib_thickness_m",
                 "rib_wave_m", "bearing_length_m", "bearing_width_m", "bearing_thickness_m", "bearing_slant_deg",
                 "catch_fin_azimuths_deg", "hinge_ring_radius_m", "ring_length_m", "ring_bolts", "band_edges", "lattice_inset_m", "strut_beam_m", "lattice_axis_y_m",
                 "housing_half_root_m", "housing_half_rib_m", "root_gap_m",
                 "shaft_radius_m", "shaft_inset_m"),
    "flap": ("root_thickness_m", "tip_thickness_m", "hinge_radius_m", "lug_length_m", "lug_height_m"),
    "flap.hinge_cover": ("width_m", "height_m", "fade_m", "overhang_m"),
    "flap.aft": ("hinge_drawn_length_m", "outline_u_m", "outline_v_m", "lug_u_m"),
    "flap.fwd": ("hinge_drawn_length_m", "hinge_station_aft_m", "hinge_radius_aft_m", "hinge_lean_deg", "outline_u_m", "outline_v_m",
                 "lug_u_m"),
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
ENGINE_KEYS = ("file", "bell_length_m", "throat_from_exit_m", "rib_count", "throat_diameter_m", "contour_exponent", "wall_m", "mount_height_m",
               "mount_radius_frac", "segments")
SIGNED_KEYS = ("cusp_dip_fwd_m", "azimuth_from_belly_deg", "azimuth_deg", "ridge_azimuth_deg", "ridge_azimuth_deg", "outline_u_m", "outline_v_m", "outline_z_m", "outline_y_m",
               "catch_fin_azimuths_deg")  # may be zero / negative; others > 0


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
                if (not v and k != "lug_u_m") or not all(_number(x) and (x >= 0.0 or k in SIGNED_KEYS) for x in v):
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


def drawn_hinge(flap: dict, kind_detail: dict, profile: hull_profile.Profile) -> dict:
    """Where the model's hinge is drawn; the physics hinge in the vehicle config equals it (pinned by a test). Forward
    flap: the 1:100 model's own place, typed into the detail file (aft end station and radius, lean). Aft flap: on the
    config's hinge, on the barrel (hull radius, unleaned). Aft end station, radius, lean and forward end station."""
    drawn = kind_detail["hinge_drawn_length_m"]
    if flap["kind"] == "fwd":
        x_a, r_a, lean = kind_detail["hinge_station_aft_m"], kind_detail["hinge_radius_aft_m"], math.radians(kind_detail["hinge_lean_deg"])
    else:
        x_a = flap["station_aft_m"] + (flap["station_fwd_m"] - flap["station_aft_m"] - drawn) / 2.0
        r_a, lean = profile.base(x_a), 0.0
    return {"drawn_hinge_station_aft_m": x_a, "drawn_hinge_lean_deg": math.degrees(lean), "drawn_hinge_radius_aft_m": r_a,
            "drawn_hinge_station_fwd_m": x_a + drawn * math.cos(lean)}


def load(vehicle_id: str, vehicles_dir: Path = VEHICLES, engines_dir: Path = ENGINES) -> dict:
    """The spec of `vehicle_id`: the Starship's full one, or for a "generic" vehicle (generators.toml) its stages' size and
    the detail file as it is (its tables are validated by the generators that come with it)."""
    kind = generator_table(vehicle_id)["kind"]
    if kind == "generic":
        return load_generic(vehicle_id, vehicles_dir)
    if kind == "falcon9":
        return load_falcon9(vehicle_id, vehicles_dir, engines_dir)
    if kind == "dragon":
        return load_dragon(vehicle_id, vehicles_dir)
    return load_starship(vehicle_id, vehicles_dir, engines_dir)


FALCON9_DETAIL_KEYS = {
    "hull": ("segments", "base_station_m", "interstage_base_m", "raceway_azimuths_deg", "raceway_width_m", "raceway_height_m",
             "raceway_first_m", "pod_length_m", "pod_width_m", "pod_height_m", "interstage_wall_m"),
    "octaweb": ("cover_top_m", "chamfer_m", "fitting_length_m", "fitting_width_m", "fitting_proud_m"),
    "grid_fin": ("chord_m", "span_m", "root_chamfer_m", "depth_m", "cell_pitch_m", "wall_m", "rim_m", "root_gap_m",
                 "pivot_radius_m", "pivot_length_m", "stow_standoff_m"),
    "leg": ("hinge_station_m", "hinge_radius_m", "stowed_tip_radius_m", "stowed_outer_radius_m", "stowed_thickness_m",
            "root_width_m", "tip_width_m", "tip_depth_m", "pad_radius_m", "pad_thickness_m", "pusher_anchor_station_m",
            "pusher_anchor_radius_m", "pusher_leg_u_m", "pusher_radius_m"),
    "paint": ("hull_albedo", "interstage_albedo", "octaweb_albedo", "leg_albedo", "stage2_hull_albedo", "engine_plate_albedo",
              "exhaust_albedo", "mvac_albedo"),
    "second_stage": ("segments", "plate_thickness_m", "top_ring_height_m", "cap_rise_m", "exhaust_radius_m", "exhaust_azimuth_deg"),
    "mvac": ("file", "bell_length_m", "extension_from_exit_m", "top_radius_m", "contour_exponent", "chamber_radius_m",
             "flange_radius_m", "flange_height_m", "wall_m", "mount_height_m", "mount_radius_m", "duct_start_from_exit_m",
             "duct_end_from_exit_m", "segments"),
}


def validate_falcon9_detail(detail: dict) -> None:
    for table, keys in FALCON9_DETAIL_KEYS.items():
        t = _table(detail, table)
        for k in keys:
            if k not in t:
                raise VehicleSpecError(f"[{table}]: missing key '{k}'")
    for eid, e in _table(detail, "engines").items():
        for k in ENGINE_KEYS:
            if k != "rib_count" and k not in e:
                raise VehicleSpecError(f"[engines.{eid}]: missing key '{k}'")


def falcon9_leg(legs: dict, leg: dict) -> dict:
    """Rig of one leg from the detail hinge and the config's foot: a straight rigid leg turning about its hinge. Angles from
    the +X station axis towards the radial; the leg's length is what joins the hinge to the deployed foot."""
    dr = legs["footprint_radius_m"] - leg["hinge_radius_m"]
    ds = legs["foot_station_m"] - leg["hinge_station_m"]
    length = math.hypot(dr, ds)
    return {**leg, "length_m": length, "deployed_angle_deg": math.degrees(math.atan2(dr, ds)),
            "stowed_angle_deg": math.degrees(math.asin((leg["stowed_tip_radius_m"] - leg["hinge_radius_m"]) / length))}


def load_falcon9(vehicle_id: str, vehicles_dir: Path = VEHICLES, engines_dir: Path = ENGINES) -> dict:
    """The Falcon 9's stages: the physics numbers its generators draw on + the detail file. `stages.second_stage` and
    `mvac` (the Merlin Vacuum's exit diameter from its engine file, bell numbers from the detail file) serve the second stage."""
    with open(vehicles_dir / f"{vehicle_id}.toml", "rb") as f:
        vehicle = tomllib.load(f)
    with open(vehicles_dir / "detail" / f"{vehicle_id}.toml", "rb") as f:
        detail = tomllib.load(f)
    validate_falcon9_detail(detail)
    s = next(st for st in vehicle["stage"] if st["id"] == "first_stage")
    stage = {"length_m": float(s["length_m"]), "diameter_m": float(s["diameter_m"]),
             "interstage_top_m": float(s["length_m"]), "grid_fins": s["grid_fins"], "landing_legs": s["landing_legs"],
             "rcs_pods": s["rcs"]["pod"]}
    engine_spec = {}
    for eid in sorted({g["engine"] for g in s["engine_group"]}):
        with open(engines_dir / f"{eid}.toml", "rb") as f:
            eng = tomllib.load(f)
        engine_spec[eid] = {"nozzle_exit_diameter_m": float(eng["performance"]["nozzle_exit_diameter_m"]),
                            **detail["engines"][eid]}
    s2 = next(st for st in vehicle["stage"] if st["id"] == "second_stage")
    with open(engines_dir / "merlin_vacuum.toml", "rb") as f:
        mvac_engine = tomllib.load(f)
    mvac = {"nozzle_exit_diameter_m": float(mvac_engine["performance"]["nozzle_exit_diameter_m"]), **detail["mvac"]}
    second = {"length_m": float(s2["length_m"]), "diameter_m": float(s2["diameter_m"]), "base_station_m": stage["length_m"],
              "rcs_pods": s2["rcs"]["pod"]}
    return {"vehicle_id": vehicle_id, "stages": {"first_stage": stage, "second_stage": second}, "engines": engine_spec,
            "detail": detail, "leg": falcon9_leg(s["landing_legs"], detail["leg"]), "mvac": mvac}


DRAGON_DETAIL_KEYS = {
    "capsule": ("az_step_deg", "nose_columns", "shield_edge_x_m", "band_top_x_m", "profile_m", "hood_half_width_deg", "hood_outer_m", "bay_top_x_m", "bay_corner_drop_m", "bay_corner_power",
                "bay_half_width_deg_m", "bay_floor_m", "nozzle_x_m", "nozzle_half_x_m", "nozzle_half_az_deg", "nozzle_outline_scale", "seam_axis_x_m", "seam_slope", "seam_gap_m", "lip_thickness_m",
                "nose_hinge_x_m", "nose_hinge_r_m", "nose_hinge_azimuth_deg", "nose_open_angle_deg", "nose_open_time_s", "hinge_fairing_m", "hinge_arm_m",
                "nose_inner_shoulder_m", "nose_inner_cone_m", "nose_seal_m", "neck_m", "deck_holes", "deck_slot", "windows", "window_flange_scale", "hatch",
                "hatch_seam_m", "draco_clusters_az_deg", "draco_slots", "lift_panel_m", "lift_detail_m"),
    "trunk": ("segments", "shield_well_m", "top_wall_m", "floor_depth_m", "base_inner_r_m", "fin_count", "fin_first_azimuth_deg", "fin_outline_m", "fin_thickness_m", "array_azimuths_deg",
              "radiator_azimuths_deg", "array_top_x_m", "panel_base_m", "panel_proud_m", "umbilical_azimuth_deg", "umbilical_width_m", "umbilical_outline_m", "trunk_slots_m", "trunk_slots_azimuths_deg"),
    "idss": ("ring_extension_m", "ring_tilt_max_deg", "petal_height_m", "petal_base_diameter_m", "hook_count", "scs_recess_m", "tunnel_diameter_m", "petal_tip_diameter_m", "petal_theta_deg",
             "hook_radius_m", "hook_active_mm", "hook_passive_mm", "pin_theta_deg", "receptacle_theta_deg", "seal_diameters_m"),
    "paint": ("capsule_albedo", "shield_albedo", "band_albedo", "cowl_albedo", "pod_albedo", "window_albedo", "deck_albedo", "trunk_albedo", "radiator_albedo", "array_albedo"),
}

def validate_dragon_detail(detail: dict) -> None:
    for table, keys in DRAGON_DETAIL_KEYS.items():
        t = _table(detail, table)
        for k in keys:
            if k not in t:
                raise VehicleSpecError(f"[{table}]: missing key '{k}'")


def dragon_lobes(angles_deg: list) -> list:
    """Centres of the SuperDraco pods: the middle of each consecutive pair of the config's engine angles."""
    if len(angles_deg) % 2:
        raise VehicleSpecError("superdraco angles_deg: pairs expected (even count)")
    return [0.5 * (angles_deg[i] + angles_deg[i + 1]) for i in range(0, len(angles_deg), 2)]


def idss_block(detail: dict, capsule: dict) -> dict:
    """What the game's DockingMechanismView needs: the [idss] numbers, the port, and the two numbers derived from the IDD's hook figures: the active hook's pivot (closed pose = the
    outline as typed: leading edge at its closed station, tangent plane at its height) and the turn that brings the leading edge to its open station. The IDD gives no stroke as a number."""
    idss = dict(detail["idss"])
    outline = idss["hook_active_mm"]
    edge_mm = max(u for u, _ in outline)
    notch_v_mm = next(v for u, v in outline if u == edge_mm and v < 30.0)  # the hook's tangent plane in its own outline
    pivot_u = idss["hook_active_edge_closed_m"] - edge_mm / 1000.0
    pivot_x = idss["hook_tangent_above_hcs_m"] - notch_v_mm / 1000.0
    target = (idss["hook_active_edge_open_m"] - pivot_u) * 1000.0  # the largest u of the turned outline must reach this
    def largest_u(turn):
        return max(u * math.cos(turn) - v * math.sin(turn) for u, v in outline)
    lo, hi = 0.0, math.radians(90.0)
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if largest_u(mid) > target else (lo, mid)
    idss.update({"hook_pivot_m": [pivot_u, pivot_x], "hook_turn_deg": math.degrees(0.5 * (lo + hi)), "port_position_m": capsule["port_position_m"], "port_axis": capsule["port_axis"], "port_up": capsule["port_up"],
                 "strut_base_x_m": detail["capsule"]["neck_m"][2][0] - capsule["port_position_m"][0],
                 "nose_seal_m": detail["capsule"]["nose_seal_m"], "nose_inner_cone_m": detail["capsule"]["nose_inner_cone_m"]})  # the closed nose's inside, for the view's test
    return idss


def load_dragon(vehicle_id: str, vehicles_dir: Path = VEHICLES) -> dict:
    """Crew Dragon: the capsule's and trunk's physics numbers (sizes, hull top, shield radius, Draco pods, SuperDraco pairs,
    docking port) + the detail file. The nose cone's frame: hinge point, axis and open angle, derived in `nose`."""
    with open(vehicles_dir / f"{vehicle_id}.toml", "rb") as f:
        vehicle = tomllib.load(f)
    with open(vehicles_dir / "detail" / f"{vehicle_id}.toml", "rb") as f:
        detail = tomllib.load(f)
    validate_dragon_detail(detail)
    trunk = next(st for st in vehicle["stage"] if st["id"] == "trunk")
    cap = next(st for st in vehicle["stage"] if st["id"] == "capsule")
    c = detail["capsule"]
    group = next(g for g in cap["engine_group"] if g["id"] == "superdraco")
    hinge_az = math.radians(c["nose_hinge_azimuth_deg"])
    top_radius = float(cap["hull_top_diameter_m"]) / 2.0
    nose = {"stage_id": "capsule", "hinge_station_m": c["nose_hinge_x_m"], "hinge_azimuth_deg": c["nose_hinge_azimuth_deg"],
            "hinge_radius_m": c["nose_hinge_r_m"], "open_angle_deg": c["nose_open_angle_deg"], "open_time_s": c["nose_open_time_s"],
            "length_m": float(cap["length_m"]) - c["nose_hinge_x_m"],
            "axis": [0.0, -math.sin(hinge_az), math.cos(hinge_az)]}
    stages = {"trunk": {"length_m": float(trunk["length_m"]), "diameter_m": float(trunk["diameter_m"])},
              "capsule": {"length_m": float(cap["length_m"]), "diameter_m": float(cap["diameter_m"]),
                          "hull_top_radius_m": top_radius, "shield_nose_radius_m": float(cap["heat_shield"]["nose_radius_m"]),
                          "rcs_pods": cap["rcs"]["pod"], "superdraco_lobes_deg": dragon_lobes(group["angles_deg"]),
                          "superdraco_half_gap_deg": 0.5 * (group["angles_deg"][1] - group["angles_deg"][0]),
                          "port_position_m": cap["docking_port"][0]["position_m"], "port_axis": cap["docking_port"][0]["axis"], "port_up": cap["docking_port"][0]["up"]}}
    return {"vehicle_id": vehicle_id, "stages": stages, "engines": {}, "detail": detail, "nose": nose, "idss": idss_block(detail, stages["capsule"])}


def load_generic(vehicle_id: str, vehicles_dir: Path = VEHICLES) -> dict:
    with open(vehicles_dir / f"{vehicle_id}.toml", "rb") as f:
        vehicle = tomllib.load(f)
    with open(vehicles_dir / "detail" / f"{vehicle_id}.toml", "rb") as f:
        detail = tomllib.load(f)
    stages = {s["id"]: {"length_m": float(s["length_m"]), "diameter_m": float(s["diameter_m"])}
              for s in vehicle.get("stage", []) if "length_m" in s and "diameter_m" in s}
    return {"vehicle_id": vehicle_id, "stages": stages, "engines": {}, "detail": detail}


def load_starship(vehicle_id: str, vehicles_dir: Path = VEHICLES, engines_dir: Path = ENGINES) -> dict:
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
        profile = hull_profile.Profile({"length_m": float(s["length_m"]), "diameter_m": float(s["diameter_m"]),
                                       "nose_length_m": float(aero.get("nose_length_m", 0.0))}, detail["hull"],
                                       detail["ship"]["engine_bay"]["aft_chamfer_m"] if s["id"] == "ship" else None)
        flaps = []
        for fl in s.get("flap", []):
            flaps.append({
                "id": fl["id"], "hinge_azimuth_deg": float(fl["hinge_azimuth_deg"]),
                "station_fwd_m": float(fl["station_fwd_m"]), "station_aft_m": float(fl["station_aft_m"]),
                "span_m": float(fl["span_m"]), "area_m2": float(fl["area_m2"]),
                "kind": "fwd" if fl["id"].startswith("forward") else "aft",
                "rotation_sign_about_plus_x": flap_rotation_sign(float(fl["hinge_azimuth_deg"]), belly),
            })
            flaps[-1].update(drawn_hinge(flaps[-1], detail["flap"][flaps[-1]["kind"]], profile))
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
