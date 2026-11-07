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
COCKPITS = ROOT / "config" / "cockpits"
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
_TAG = re.compile(r"#.*\[(E|S|D)\]")  # D: measured / derived (config/README.md)


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
    if kind == "iss":
        return load_iss(vehicle_id, vehicles_dir)
    return load_starship(vehicle_id, vehicles_dir, engines_dir)


FALCON9_SURFACE_KEYS = ("soot_level", "soot_profile_m", "soot_windward_azimuth_deg", "soot_windward_share", "soot_albedo", "soot_roughness", "raceway_albedo", "hull_tiles_around", "hull_ring_pitch_m",
                        "hull_line_half_m", "hull_line_depth", "joint_height_m", "joint_rivet_pitch_m", "joint_rivet_radius_m", "joint_rivet_depth", "interstage_gain", "interstage_tiles_around",
                        "interstage_rows", "interstage_rivet_pitch_m", "interstage_rivet_radius_m", "octaweb_gain", "stage2_rows", "leg_gain")
FALCON9_DETAIL_KEYS = {
    "surface": FALCON9_SURFACE_KEYS,
    "hull": ("segments", "base_station_m", "interstage_base_m", "raceway_azimuths_deg", "raceway_width_m", "raceway_height_m",
             "raceway_first_m", "pod_length_m", "pod_width_m", "pod_height_m", "interstage_wall_m"),
    "octaweb": ("cover_top_m", "chamfer_m", "fitting_length_m", "fitting_width_m", "fitting_proud_m"),
    "grid_fin": ("chord_m", "span_m", "root_chamfer_m", "depth_m", "cell_pitch_m", "wall_m", "rim_m", "root_gap_m",
                 "pivot_radius_m", "pivot_length_m", "stow_standoff_m"),
    "leg": ("hinge_station_m", "hinge_radius_m", "stowed_tip_radius_m", "stowed_outer_radius_m", "stowed_thickness_m",
            "root_width_m", "tip_width_m", "tip_depth_m", "pad_radius_m", "pad_thickness_m", "pusher_anchor_station_m",
            "pusher_anchor_radius_m", "pusher_leg_u_m", "pusher_radius_m"),
    "paint": ("hull_albedo", "interstage_albedo", "interstage_roughness", "octaweb_albedo", "leg_albedo", "stage2_hull_albedo", "engine_plate_albedo",
              "exhaust_albedo", "mvac_albedo"),
    "second_stage": ("segments", "plate_thickness_m", "top_band_height_m", "skirt_radius_m", "exhaust_radius_m", "exhaust_azimuth_deg"),
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
              "engine_station_m": float(s2["engine_group"][0].get("station_m", 0.0)), "rcs_pods": s2["rcs"]["pod"]}
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

DRAGON_SURFACE_KEYS = ("flown_soot", "shield_new_albedo", "skin_rim_row_x_m", "skin_rivet_pitch_m", "skin_rivet_radius_m", "skin_line_half_m", "skin_seam_rivet_below_m", "skin_seam_paint_half_m", "skin_seam_paint",
                       "skin_panels", "shield_rings_r_m", "shield_segments", "array_period_m", "array_cells", "array_gain", "array_glass_factor", "radiator_panel_m", "radiator_line_half_m",
                       "radiator_rivet_pitch_m", "radiator_rivet_radius_m")
DRAGON_DETAIL_KEYS["surface"] = DRAGON_SURFACE_KEYS


def validate_dragon_detail(detail: dict) -> None:
    for table, keys in DRAGON_DETAIL_KEYS.items():
        t = _table(detail, table)
        for k in keys:
            if k not in t:
                raise VehicleSpecError(f"[{table}]: missing key '{k}'")


COCKPIT_KEYS = {
    "cockpit": ("capsule_stage",),
    "cabin": ("wall_m", "floor_x_m", "ceiling_x_m", "tunnel_radius_m", "tunnel_top_x_m", "profile_m", "windows", "hatch", "window_reveal_radius_m", "window_bezel_m", "hatch_trim_m", "ring_light_radii_m"),
    "eye": ("position_m", "forward", "up"),
    "console": ("centre_m", "normal", "up", "width_m", "height_m", "depth_m", "fold_deg", "strut_z_m", "screen_gap_m", "shelf_height_m", "shelf_top_gap_m", "module_split_m", "module_margin_m", "upper_top_x_m", "upper_depth_m", "locker_proud_m"),
    "hotas": ("props", "stick_grip_m", "throttle_grip_m"),
}
COCKPIT_SCREENS = ("left", "centre", "right")
COCKPIT_SEATS = ("s1", "s2", "s3", "s4")
COCKPIT_BUTTON_KINDS = ("push", "covered", "handle")
COCKPIT_BUTTON_SHAPES = ("round", "rect")


def validate_cockpit(cockpit: dict) -> None:
    """The cabin tables the generator reads: keys present, screens / seats / tablets complete, every button on a known screen, unit vectors normalised."""
    for table, keys in COCKPIT_KEYS.items():
        t = cockpit.get(table)
        if not isinstance(t, dict):
            raise VehicleSpecError(f"cockpit: missing table [{table}]")
        for k in keys:
            if k not in t:
                raise VehicleSpecError(f"cockpit [{table}]: missing key '{k}'")
    if tuple(s["id"] for s in cockpit.get("seat", [])) != COCKPIT_SEATS:
        raise VehicleSpecError(f"cockpit: [[seat]] ids must be {COCKPIT_SEATS}")
    screens = {s["id"]: s for s in cockpit.get("screen", [])}
    if tuple(sorted(screens)) != tuple(sorted(COCKPIT_SCREENS)):
        raise VehicleSpecError(f"cockpit: [[screen]] ids must be {COCKPIT_SCREENS}")
    if sorted(t["id"] for t in cockpit.get("tablet", [])) != ["l", "r"]:
        raise VehicleSpecError("cockpit: [[tablet]] ids must be l and r")
    if not cockpit.get("light"):
        raise VehicleSpecError("cockpit: no [[light]]")
    for seat in cockpit["seat"]:
        for k in ("foot_rest_m", "foot_rest_normal"):
            if k not in seat:
                raise VehicleSpecError(f"cockpit seat {seat['id']}: missing key '{k}'")
    if sorted(l.get("id") for l in cockpit.get("locker", [])) != ["5", "6"]:
        raise VehicleSpecError("cockpit: [[locker]] ids must be 5 and 6")

    def unit(name, v):
        if abs(math.sqrt(sum(c * c for c in v)) - 1.0) > 2e-3:
            raise VehicleSpecError(f"cockpit {name}: vector {v} is not a unit vector")

    unit("eye.forward", cockpit["eye"]["forward"])
    unit("eye.up", cockpit["eye"]["up"])
    unit("console.normal", cockpit["console"]["normal"])
    for seat in cockpit["seat"]:
        unit(f"seat {seat['id']} foot_rest_normal", seat["foot_rest_normal"])
    for sid, s in screens.items():
        unit(f"screen {sid} normal", s["normal"])
        unit(f"screen {sid} up", s["up"])
        for k in ("centre_m", "size_m"):
            if k not in s:
                raise VehicleSpecError(f"cockpit screen {sid}: missing key '{k}'")
    for t in cockpit["tablet"]:
        unit(f"tablet {t['id']} normal", t["normal"])
    ids = set()
    for b in cockpit.get("button", []):
        for k in ("id", "panel", "centre_m", "size_m", "shape", "kind", "function"):
            if k not in b:
                raise VehicleSpecError(f"cockpit button {b.get('id', '?')}: missing key '{k}'")
        if b["id"] in ids:
            raise VehicleSpecError(f"cockpit: duplicate button id {b['id']}")
        ids.add(b["id"])
        if b["panel"] not in screens:
            raise VehicleSpecError(f"cockpit button {b['id']}: unknown panel {b['panel']}")
        if b["kind"] not in COCKPIT_BUTTON_KINDS or b["shape"] not in COCKPIT_BUTTON_SHAPES:
            raise VehicleSpecError(f"cockpit button {b['id']}: unknown kind {b['kind']} / shape {b['shape']}")
    if "handle" not in ids:
        raise VehicleSpecError("cockpit: no [[button]] with id handle")


def load_cockpit(cockpit_id: str, cockpits_dir: Path = COCKPITS) -> dict:
    with open(cockpits_dir / f"{cockpit_id}.toml", "rb") as f:
        cockpit = tomllib.load(f)
    validate_cockpit(cockpit)
    return cockpit


FIGURE_KEYS = {
    "figure": ("stature_m", "hip_joint_up_m", "hip_joint_forward_m", "hip_joint_half_spacing_m", "knee_angle_deg", "thigh_raise_deg", "arm_reach_fraction"),
    "proportion": ("shoulder_height", "hip_height", "knee_height", "ankle_height", "upper_arm", "forearm", "hand", "shoulder_width", "head_height", "foot_length"),
    "pose": ("upper_arm_dir", "forearm_dir", "neck_lean_m", "head_centre_up_m", "head_centre_forward_m"),
    "hand": ("palm_length_m", "palm_half_width_m", "palm_half_thick_m", "index_m", "index_radius_m", "index_offset_m", "fingers_m", "fingers_half_width_m",
             "fingers_radius_m", "fingers_offset_m", "thumb_m", "thumb_radius_m", "thumb_root_offset_m", "thumb_root_forward_m", "thumb_spread_deg", "finger_curl_deg"),
    "suit": ("torso_rings_m", "thigh_radius_m", "shin_radius_m", "upper_arm_radius_m", "forearm_radius_m", "cuff_radius_m", "cuff_length_m", "boot_shaft_fraction",
             "boot_foot_half_width_m", "boot_foot_half_height_m", "boot_heel_m", "neck_radius_m"),
    "helmet": ("half_width_m", "half_depth_m", "half_below_m", "half_above_m", "visor_half_angle_deg", "visor_up_deg", "visor_down_deg", "visor_proud_m", "visor_edge_m", "visor_thickness_m", "rim_radius_m", "collar_radius_m", "collar_height_m",
               "liner_radius_m", "liner_inset_deg", "egg_taper", "crown_ridge_m", "crown_ridge_width_m", "visor_tint_split", "visor_low_albedo",
               "visor_low_roughness", "visor_low_metallic", "pivot_button_radius_m", "pivot_button_height_m", "pivot_el_deg", "pivot_az_offset_deg"),
}
FIGURE_MATERIALS = ("suit", "boots", "gloves", "helmet", "collar", "visor")


def validate_crew_figure(fig: dict) -> None:
    """The crew figure tables gen_crew_figure.py reads: keys present, the six materials."""
    for table, keys in FIGURE_KEYS.items():
        t = fig.get(table)
        if not isinstance(t, dict):
            raise VehicleSpecError(f"crew figure: missing table [{table}]")
        for k in keys:
            if k not in t:
                raise VehicleSpecError(f"crew figure [{table}]: missing key '{k}'")
    for m in FIGURE_MATERIALS:
        if set(fig.get("material", {}).get(m, {})) != {"albedo", "roughness", "metallic"}:
            raise VehicleSpecError(f"crew figure: [material.{m}] needs albedo, roughness, metallic")


def load_crew_figure(figure_id: str = "crew_figure", cockpits_dir: Path = COCKPITS) -> dict:
    with open(cockpits_dir / f"{figure_id}.toml", "rb") as f:
        fig = tomllib.load(f)
    validate_crew_figure(fig)
    return fig


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
    spec = {"vehicle_id": vehicle_id, "stages": stages, "engines": {}, "detail": detail, "nose": nose, "idss": idss_block(detail, stages["capsule"])}
    cockpit_id = vehicle.get("vehicle", {}).get("cockpit")
    if cockpit_id:  # the cabin tables (config/cockpits/<id>.toml, next to the vehicles folder): gen_dragon_cabin.py
        spec["cockpit"] = load_cockpit(cockpit_id, vehicles_dir.parent / "cockpits")
        if spec["cockpit"]["cockpit"]["capsule_stage"] != "capsule":
            raise VehicleSpecError("cockpit: capsule_stage must name the Dragon's capsule stage")
        spec["crew_figure"] = load_crew_figure("crew_figure", vehicles_dir.parent / "cockpits")  # the seated crew figure (gen_crew_figure.py)
    return spec


ISS_DETAIL_TABLES = ("paint", "pose", "ida", "lathe", "box", "bay", "plate", "tube", "wing", "gamma", "array", "irosa_plane", "joint", "array_shape", "truss_lattice")


def load_iss(vehicle_id: str, vehicles_dir: Path = VEHICLES) -> dict:
    """The ISS: the ports of the physics config (position, axis, up) + the drawing tables + the IDSS numbers of Dragon's detail file (one interface, one table: the system is
    androgynous). Port `number` 2 / 3 names the nodes; `pma_offset_dir` (drawing frame of idss_passive_ring.py) points from the ring axis to the Harmony axis."""
    with open(vehicles_dir / f"{vehicle_id}.toml", "rb") as f:
        vehicle = tomllib.load(f)
    with open(vehicles_dir / "detail" / f"{vehicle_id}.toml", "rb") as f:
        detail = tomllib.load(f)
    with open(vehicles_dir / "crew_dragon.toml", "rb") as f:
        dragon = tomllib.load(f)
    with open(vehicles_dir / "detail" / "crew_dragon.toml", "rb") as f:
        dragon_detail = tomllib.load(f)
    for table in ISS_DETAIL_TABLES:
        if table not in detail:
            raise VehicleSpecError(f"iss detail: missing table [{table}]")
    wings = len(detail["wing"])
    if len(detail["pose"]["beta_deg"]) != wings:
        raise VehicleSpecError("iss detail: [pose] beta_deg needs one angle per [[wing]]")
    joints = {row["id"]: row["origin_m"] for row in detail["joint"]}
    for wing in detail["wing"]:
        if wing["joint"] not in joints:
            raise VehicleSpecError(f"iss detail: wing {wing['array']}: unknown joint {wing['joint']}")
    station = next(st for st in vehicle["stage"] if st["id"] == "station")
    cap = next(st for st in dragon["stage"] if st["id"] == "capsule")
    # The Harmony axis against the ring axis: ring offset measured at each port (detail [[port]] ring_axis_offset_from_harmony_axis_m), direction from the configured position.
    harmony_axis = {"ida2": (None, 0.0, 0.0), "ida3": (46.72, 0.0, None)}
    ports = []
    for number, port in zip((2, 3), station["docking_port"]):
        axis, up, pos = port["axis"], port["up"], port["position_m"]
        reference = harmony_axis[port["id"]]
        to_harmony = [0.0 if reference[k] is None else reference[k] - pos[k] for k in range(3)]
        to_harmony = [0.0 if abs(a) > 0.5 else v for a, v in zip(axis, to_harmony)]
        # into the drawing frame: X = axis, Y = axis x up, Z = -up
        cross = (axis[1] * up[2] - axis[2] * up[1], axis[2] * up[0] - axis[0] * up[2], axis[0] * up[1] - axis[1] * up[0])
        dot = lambda a, b: sum(x * y for x, y in zip(a, b))
        length = math.sqrt(dot(to_harmony, to_harmony)) or 1.0
        offset_dir = [dot(to_harmony, axis) / length, dot(to_harmony, cross) / length, -dot(to_harmony, up) / length]
        ports.append({"id": port["id"], "number": number, "position_m": pos, "axis": axis, "up": up, "pma_offset_dir": offset_dir})
    return {"vehicle_id": vehicle_id, "stages": {"station": {"length_m": float(station["length_m"]), "diameter_m": float(station["diameter_m"])}}, "engines": {}, "detail": detail,
            "ports": ports, "idss": idss_block(dragon_detail, {"port_position_m": cap["docking_port"][0]["position_m"], "port_axis": cap["docking_port"][0]["axis"], "port_up": cap["docking_port"][0]["up"]})}


def load_generic(vehicle_id: str, vehicles_dir: Path = VEHICLES) -> dict:
    with open(vehicles_dir / f"{vehicle_id}.toml", "rb") as f:
        vehicle = tomllib.load(f)
    with open(vehicles_dir / "detail" / f"{vehicle_id}.toml", "rb") as f:
        detail = tomllib.load(f)
    stages = {s["id"]: {"length_m": float(s["length_m"]), "diameter_m": float(s["diameter_m"])}
              for s in vehicle.get("stage", []) if "length_m" in s and "diameter_m" in s}
    return {"vehicle_id": vehicle_id, "stages": stages, "engines": {}, "detail": detail}


VARIANT_BOOLS = ("tiles", "flaps", "chine", "catch_pins", "payload_door")
VARIANT_SKINS = ("steel", "insulated", "white")
DOCK_HARDWARE = "dock_hardware.toml"  # the shared probe-drogue drawing table (detail/), `[dorsal] shared = true` in a ship's detail file
VARIANT_BASE = "starship_block3"  # a variant's detail file holds only [variant] and its own tables; the rest is Block 3's


def variant_of(detail: dict) -> dict:
    """The ship variant switches of a detail file: [variant] over the defaults (all true, skin steel = Block 3)."""
    given = detail.get("variant", {})
    unknown = sorted(set(given) - set(VARIANT_BOOLS) - {"skin"})
    if unknown:
        raise VehicleSpecError(f"[variant]: unknown key {unknown}")
    variant = {k: True for k in VARIANT_BOOLS}
    variant["skin"] = "steel"
    for k, v in given.items():
        if k == "skin":
            if v not in VARIANT_SKINS:
                raise VehicleSpecError(f"[variant].skin: one of {VARIANT_SKINS}, got {v!r}")
        elif not isinstance(v, bool):
            raise VehicleSpecError(f"[variant].{k}: must be true or false, got {v!r}")
        variant[k] = v
    return variant


def resolve_stages(vehicle_id: str, vehicles_dir: Path = VEHICLES, _depth: int = 0) -> list:
    """The stage tables of a vehicle config, bottom to top. A taken-over stage (`from = "<vehicle>:<stage>"`) is the
    source stage's table (its physics, flaps, engines) with the entry's tank changes merged into `tank` by id."""
    if _depth > 4:
        raise VehicleSpecError(f"{vehicle_id}: stage.from chain too long (a cycle?)")
    with open(vehicles_dir / f"{vehicle_id}.toml", "rb") as f:
        vehicle = tomllib.load(f)
    stages = []
    for entry in vehicle["stage"]:
        if "from" not in entry:
            stages.append(entry)
            continue
        source_id, _, stage_id = entry["from"].partition(":")
        source = next((s for s in resolve_stages(source_id, vehicles_dir, _depth + 1) if s["id"] == stage_id), None)
        if source is None:
            raise VehicleSpecError(f"{vehicle_id}: stage.from = {entry['from']!r}: no such stage")
        # the entry's tank list only resizes tanks of the source (core/src/config/vehicle_config.cpp, load_stage_from)
        tanks = [dict(t) for t in source.get("tank", [])]
        for change in entry.get("tank", []):
            target = next((t for t in tanks if t["id"] == change["id"]), None)
            if target is None:
                raise VehicleSpecError(f"{vehicle_id}: tank {change['id']!r} is not on {entry['from']!r}")
            target.update(change)
        taken = {**source, "tank": tanks}
        for key in ("docking_port", "transfer_connector"):  # appended to the source stage's lists, as the core's append_docking_ports does
            if key in entry:
                taken[key] = list(source.get(key, [])) + list(entry[key])
        stages.append(taken)
    return stages


DORSAL_KEYS = ("probe_extension_m", "probe_stowed_clearance_m", "fairing_height_m", "fairing_radius_m", "fairing_chamfer_m", "fairing_bore_clearance_m", "fairing_embed_m",
               "probe_head_radius_m", "probe_head_length_m", "probe_head_collar_m", "probe_shaft_radius_m", "probe_shaft_length_m",
               "probe_tip_flat_radius_m", "drogue_mouth_radius_m", "drogue_depth_m", "drogue_throat_radius_m", "drogue_wall_m", "drogue_foot_m",
               "connector_pedestal_radius_m", "connector_nozzle_wall_m", "connector_nozzle_length_m", "connector_embed_m", "raceway_detour_m", "raceway_detour_ramp_m",
               "segments")


def dorsal_block(vehicle_id: str, stage_id: str, ship: dict, drawing: dict) -> dict:
    """The ship's `[[docking_port]]` (probe_drogue only) and `[[transfer_connector]]` tables with the `[dorsal]` drawing numbers."""
    for k in DORSAL_KEYS:
        v = drawing.get(k)
        if not _number(v) or v <= 0.0:
            raise VehicleSpecError(f"[dorsal].{k}: must be a positive number, got {v!r}")
    ports = []
    for p in ship.get("docking_port", []):
        if p.get("mechanism") != "probe_drogue":
            raise VehicleSpecError(f"{vehicle_id}: docking port {p.get('id')!r}: only probe_drogue ports are drawn by [dorsal]")
        if p.get("soft_capture") not in ("active", "passive"):
            raise VehicleSpecError(f"{vehicle_id}: docking port {p['id']!r}: soft_capture must be 'active' (probes) or 'passive' (drogues)")
        ports.append({k: p[k] for k in ("id", "mechanism", "position_m", "axis", "up", "soft_capture", "elements_m")})
    if not ports:
        raise VehicleSpecError(f"{vehicle_id}: [dorsal] needs a probe_drogue docking port on the ship stage")
    if len({tuple(p["axis"]) for p in ports}) > 1:
        raise VehicleSpecError(f"{vehicle_id}: the dorsal ports must share one axis")
    connectors = [{k: c[k] for k in ("id", "position_m", "line_diameter_m")} for c in ship.get("transfer_connector", [])]
    return {"stage_id": stage_id, "ports": ports, "connectors": connectors, "drawing": {k: drawing[k] for k in DORSAL_KEYS}}


INSULATION_KEYS = ("albedo", "roughness", "metallic", "pillow_width_m", "pillow_height_m", "pillow_bulge_m", "wrinkle_share", "seam_darkening", "seam_half_width_m")
RCS_POD_KEYS = ("length_m", "width_m", "height_m")


def insulation_block(vehicle_id: str, table: dict) -> dict:
    """`[insulation]` of a detail file with `skin = "insulated"`: the MLI blanket's quilting (bake) and shading (shader)."""
    missing = [k for k in INSULATION_KEYS if k not in table]
    if missing or sorted(set(table) - set(INSULATION_KEYS)):
        raise VehicleSpecError(f"{vehicle_id}: [insulation] needs exactly {INSULATION_KEYS}; missing {missing}, unknown {sorted(set(table) - set(INSULATION_KEYS))}")
    albedo = table["albedo"]
    if not (isinstance(albedo, list) and len(albedo) == 3 and all(_number(c) and 0.0 < c < 1.0 for c in albedo)):
        raise VehicleSpecError(f"[insulation].albedo: three numbers in (0, 1), got {albedo!r}")
    for k in INSULATION_KEYS[1:]:
        if not _number(table[k]) or table[k] < 0.0:
            raise VehicleSpecError(f"[insulation].{k}: must be a non-negative number, got {table[k]!r}")
    for k in ("roughness", "metallic", "wrinkle_share", "seam_darkening"):
        if table[k] > 1.0:
            raise VehicleSpecError(f"[insulation].{k}: at most 1, got {table[k]!r}")
    return {k: table[k] for k in INSULATION_KEYS}


def rcs_pods_block(vehicle_id: str, ship: dict, drawing: dict) -> dict:
    """The ship's RCS pods (station, azimuths from the config) with the drawn size of `[rcs_pods]`."""
    for k in RCS_POD_KEYS:
        if not _number(drawing.get(k)) or drawing[k] <= 0.0:
            raise VehicleSpecError(f"[rcs_pods].{k}: must be a positive number, got {drawing.get(k)!r}")
    pods = [{"id": p["id"], "station_m": float(p["station_m"]), "azimuths_deg": [float(a) for a in p["azimuths_deg"]]} for p in ship.get("rcs", {}).get("pod", [])]
    if not pods:
        raise VehicleSpecError(f"{vehicle_id}: [rcs_pods] needs [[stage.rcs.pod]] on the ship stage")
    return {"pods": pods, "drawing": {k: drawing[k] for k in RCS_POD_KEYS}}


HLS_TABLES = {
    "landing_thruster": ("throat_diameter_m", "bell_length_m", "wall_m", "contour_exponent", "pod_wall_m", "pod_length_m", "pod_taper_m", "bay_radius_m", "bay_albedo", "segments"),
    "nose_port": ("radius_m", "height_m", "embed_m", "recess_m", "face_radius_m", "hub_radius_m"),
    "solar_band": ("station_from_m", "station_to_m", "albedo", "line_albedo", "columns", "row_pitch_m", "line_half_width_m", "trim_height_m", "trim_albedo", "roughness", "metallic"),
    "landing_leg": ("pad_radius_m", "pad_height_m", "stem_radius_m", "stem_inside_m", "sleeve_radius_m", "sleeve_length_m", "leg_root_radius_m",
                    "leg_tip_radius_m", "clevis_height_m", "clevis_width_m", "pad_pivot_gap_m", "hub_radius_m", "pusher_anchor_station_m",
                    "pusher_anchor_radius_m", "pusher_leg_u_m", "pusher_radius_m", "anchor_size_m", "attach_embed_m", "sides"),
    "airlock": ("azimuth_deg", "sill_station_m", "height_m", "width_m", "frame_depth_m", "frame_width_m"),
    "elevator": ("azimuth_deg", "track_from_station_m", "track_to_station_m", "rail_gauge_m", "rail_width_m", "rail_depth_m", "rail_stand_off_m", "bracket_count",
                 "cable_radius_m", "basket_width_m", "basket_depth_m", "basket_height_m", "wall_height_m", "basket_bottom_station_m", "basket_clearance_m",
                 "frame_m", "mesh_bars", "mesh_bar_m"),
    "windows": ("station_m", "azimuth_deg", "count", "pitch_m", "width_m", "height_m", "corner_m", "frame_m", "frame_albedo", "glass_albedo"),
}
WHITE_KEYS = ("albedo", "roughness", "metallic", "panel_width_m", "panel_height_m", "groove_half_width_m", "groove_depth_m", "groove_darkening", "wave_share")


def _colour(vehicle_id: str, where: str, v) -> None:
    if not (isinstance(v, list) and len(v) == 3 and all(_number(c) and 0.0 <= c <= 1.0 for c in v)):
        raise VehicleSpecError(f"{where}: three numbers in [0, 1], got {v!r}")


def white_paint_block(vehicle_id: str, table: dict) -> dict:
    """`[white_paint]` of a detail file with `skin = "white"`: the paint's panel lines (bake) and shading (shader)."""
    missing = [k for k in WHITE_KEYS if k not in table]
    if missing or sorted(set(table) - set(WHITE_KEYS)):
        raise VehicleSpecError(f"{vehicle_id}: [white_paint] needs exactly {WHITE_KEYS}; missing {missing}, unknown {sorted(set(table) - set(WHITE_KEYS))}")
    _colour(vehicle_id, "[white_paint].albedo", table["albedo"])
    for k in WHITE_KEYS[1:]:
        if not _number(table[k]) or table[k] < 0.0:
            raise VehicleSpecError(f"[white_paint].{k}: must be a non-negative number, got {table[k]!r}")
    for k in ("roughness", "metallic", "groove_darkening", "wave_share"):
        if table[k] > 1.0:
            raise VehicleSpecError(f"[white_paint].{k}: at most 1, got {table[k]!r}")
    return {k: table[k] for k in WHITE_KEYS}


def hls_parts_block(vehicle_id: str, stage_id: str, ship: dict, detail: dict, engines_dir: Path = ENGINES) -> dict:
    """The HLS's own hull parts: the landing thrusters exactly as the core resolves the config's `landing_only` engine groups (position
    (station, r cos a, r sin a), thrust axis (cos c, -sin c cos a, -sin c sin a): core/src/vehicle/vehicle.cpp), with the drawing numbers of the
    detail file's [landing_thruster] / [nose_port] / [solar_band] / [windows]."""
    drawing = {}
    for name, keys in HLS_TABLES.items():
        t = detail.get(name)
        if not isinstance(t, dict):
            raise VehicleSpecError(f"{vehicle_id}: detail file has no [{name}]")
        missing = [k for k in keys if k not in t]
        if missing or sorted(set(t) - set(keys)):
            raise VehicleSpecError(f"{vehicle_id}: [{name}] needs exactly {keys}; missing {missing}, unknown {sorted(set(t) - set(keys))}")
        for k in keys:
            if k.endswith("albedo"):
                _colour(vehicle_id, f"[{name}].{k}", t[k])
            elif not _number(t[k]) or (t[k] <= 0.0 and k not in ("azimuth_deg", "metallic", "track_to_station_m")):
                raise VehicleSpecError(f"[{name}].{k}: must be a positive number, got {t[k]!r}")
        drawing[name] = {k: t[k] for k in keys}
    thrusters = []
    for g in ship.get("engine_group", []):
        if not g.get("landing_only", False):
            continue
        with open(engines_dir / f"{g['engine']}.toml", "rb") as f:
            exit_d = float(tomllib.load(f)["performance"]["nozzle_exit_diameter_m"])
        count = int(g.get("count", 1))
        angles = [math.radians(a) for a in (g.get("angles_deg") or [360.0 * i / count for i in range(count)])]
        if len(angles) != count:
            raise VehicleSpecError(f"{vehicle_id}.{g['id']}: angles_deg has {len(angles)} entries, count {count}")
        cant, ring, station = math.radians(float(g.get("cant_deg", 0.0))), float(g["ring_radius_m"]), float(g.get("station_m", 0.0))
        for a in angles:
            thrusters.append({"group": g["id"], "position_m": [station, ring * math.cos(a), ring * math.sin(a)],
                              "axis": [math.cos(cant), -math.sin(cant) * math.cos(a), -math.sin(cant) * math.sin(a)], "exit_diameter_m": exit_d})
    if not thrusters:
        raise VehicleSpecError(f"{vehicle_id}: no landing_only engine group on the ship stage")
    legs = ship.get("landing_legs")
    rig_keys = ("deploy_time_s", "hinge_station_m", "hinge_radius_m", "knuckle_height_m")
    if not isinstance(legs, dict) or legs.get("fixed", False) or not all(_number(legs.get(k)) for k in rig_keys):
        raise VehicleSpecError(f"{vehicle_id}: the HLS needs deployable [stage.landing_legs] with {rig_keys} (Task 6b)")
    leg_block = {"azimuths_deg": [float(a) for a in legs["azimuths_deg"]], "footprint_radius_m": float(legs["footprint_radius_m"]),
                 "foot_station_m": float(legs["foot_station_m"]), "stroke_m": float(legs["stroke_m"]), "deploy_time_s": float(legs["deploy_time_s"]),
                 **{k: float(legs[k]) for k in rig_keys[1:]}, "hull_radius_m": 0.5 * float(ship["diameter_m"])}
    gap = leg_block["knuckle_height_m"] - drawing["landing_leg"]["pad_height_m"] - 0.5 * drawing["landing_leg"]["sleeve_length_m"]
    if gap <= leg_block["stroke_m"]:
        raise VehicleSpecError(f"{vehicle_id}: the free stem ({gap:.3f} m: knuckle height less pad and half sleeve) must exceed the stroke")
    return {"stage_id": stage_id, "length_m": float(ship["length_m"]), "thrusters": thrusters, "legs": leg_block, "drawing": drawing}


def load_starship(vehicle_id: str, vehicles_dir: Path = VEHICLES, engines_dir: Path = ENGINES) -> dict:
    """Spec of a Block 3 vehicle or ship variant. Stages are keyed by ROLE ("booster", "ship"): the one stage that is not
    the booster is the "ship" role whatever its id (depot, hls); a variant spec also names it as `stage_id`."""
    with open(vehicles_dir / "detail" / f"{vehicle_id}.toml", "rb") as f:
        detail = tomllib.load(f)
    is_variant = "variant" in detail
    if is_variant and vehicle_id != VARIANT_BASE:
        with open(vehicles_dir / "detail" / f"{VARIANT_BASE}.toml", "rb") as f:
            base = tomllib.load(f)
        detail = {**base, **detail}  # table level: a variant may replace a whole table
    if detail.get("dorsal", {}).get("shared"):  # the probe-drogue drawing numbers are one table for every ship that has them (Task 4 fix, I2)
        with open(vehicles_dir / "detail" / DOCK_HARDWARE, "rb") as f:
            detail = {**detail, "dorsal": tomllib.load(f)["dorsal"]}
    variant = variant_of(detail)
    validate_detail(detail)
    stages = {}
    engine_ids = set()
    raw_ship = None
    for s in resolve_stages(vehicle_id, vehicles_dir):
        role = "booster" if s["id"] == "booster" else "ship"
        if role == "ship":
            raw_ship = s
        if role in stages:
            raise VehicleSpecError(f"{vehicle_id}: two stages of the role '{role}'")
        aero = s.get("aero", {})
        belly = float(aero.get("belly_azimuth_deg", 270.0))
        profile = hull_profile.Profile({"length_m": float(s["length_m"]), "diameter_m": float(s["diameter_m"]),
                                       "nose_length_m": float(aero.get("nose_length_m", 0.0))}, detail["hull"],
                                       detail["ship"]["engine_bay"]["aft_chamfer_m"] if role == "ship" else None)
        flaps = []
        for fl in (s.get("flap", []) if variant["flaps"] else []):
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
            if is_variant and g["engine"] not in detail["engines"]:
                continue  # an engine the variant has no model for (the HLS landing thrusters: its own task) keeps no mesh
            engine_ids.add(g["engine"])
            count = int(g.get("count", 1))
            angles = g.get("angles_deg") or [360.0 * i / count for i in range(count)]
            if len(angles) != count:
                raise VehicleSpecError(f"{s['id']}.{g['id']}: angles_deg has {len(angles)} entries, count {count}")
            engines.append({"group": g["id"], "engine": g["engine"], "ring_radius_m": float(g["ring_radius_m"]),
                            "station_m": float(g.get("station_m", 0.0)), "angles_deg": [float(a) for a in angles]})
        stages[role] = {
            "length_m": float(s["length_m"]), "diameter_m": float(s["diameter_m"]),
            "nose_length_m": float(aero.get("nose_length_m", 0.0)), "belly_azimuth_deg": belly,
            "grid_fins": s.get("grid_fins"), "catch_pins": s.get("catch_pins") if variant["catch_pins"] else None,
            "flaps": flaps, "engines": engines,
        }
        if is_variant:
            stages[role]["stage_id"] = s["id"]
            stages[role]["tanks"] = {t["id"]: {k: float(t[k]) for k in ("mass_kg", "base_station_m") if k in t}
                                     for t in s.get("tank", [])}
    if "ship" not in stages:
        raise VehicleSpecError(f"{vehicle_id}: no ship stage")
    if "booster" not in stages and not is_variant:
        raise VehicleSpecError(f"{vehicle_id}: no stage 'booster'")
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
    spec = {"vehicle_id": vehicle_id, "stages": stages, "engines": engine_spec, "detail": detail}
    if is_variant:
        spec["variant"] = variant
    if "dorsal" in detail:  # tanker probes / depot drogues and the transfer connectors, placed by the vehicle config (Task 3)
        spec["dorsal"] = dorsal_block(vehicle_id, stages["ship"]["stage_id"], raw_ship, detail["dorsal"])
    if is_variant and variant["skin"] == "insulated":  # the depot's MLI blanket look: baked map + shader numbers (Task 4)
        spec["insulation"] = insulation_block(vehicle_id, detail.get("insulation", {}))
    if is_variant and variant["skin"] == "white":  # the HLS: white paint, solar band, windows, thrusters, nose port (Task 5)
        spec["white_paint"] = white_paint_block(vehicle_id, detail.get("white_paint", {}))
        spec["hls_parts"] = hls_parts_block(vehicle_id, stages["ship"]["stage_id"], raw_ship, detail)
    if "rcs_pods" in detail:  # drawn RCS pods at the config's pod stations (Task 4)
        spec["rcs_pods"] = rcs_pods_block(vehicle_id, raw_ship, detail["rcs_pods"])
    return spec


def dumps(spec: dict) -> str:
    return json.dumps(spec, indent=1, sort_keys=True) + "\n"
