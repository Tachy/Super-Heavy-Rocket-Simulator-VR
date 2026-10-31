"""Loader and validator of a structure detail file (config/structures/*.toml, spec C).

`load(path) -> dict` returns the parsed TOML after checking the required keys, their types and units
(all lengths in metres, > 0). `value_lines_without_tag(path)` lists value lines lacking an `[E]` / `[S]` comment tag.
"""
from __future__ import annotations

import re
import tomllib
from pathlib import Path

PROFILE_KINDS = ("h_beam", "tube", "angle", "box")
SECTION_KEYS = ("height_m", "face_width_m", "pitch_m", "pattern", "leg_profile", "brace_profile")
CHOPSTICK_KEYS = ("length_m", "depth_m", "end_depth_m", "knee_depth_m", "foot_depth_m", "root_kink_x_m", "bend_out_m",
                  "leg_end_x_m", "connector_end_x_m", "leg_offset_m", "bearing_start_x_m", "bearing_tube_m",
                  "plate_thickness_m", "plate_height_m", "plate_end_x_m", "supply_line_m", "inner_face_y_m",
                  "chord_profile", "brace_profile", "thin_profile")
CARRIAGE_KEYS = ("width_m", "bottom_depth_m", "hinge_y_m", "drum_diameter_m", "drum_gap_m", "cable_count", "cable_size_m",
                 "actuator_anchor_x_m", "actuator_anchor_in_m", "actuator_anchor_depth_m", "actuator_lug_x_m",
                 "actuator_lug_out_m", "actuator_lug_depth_m", "actuator_barrel_m", "actuator_rod_m",
                 "actuator_barrel_length_m")
QD_ARM_KEYS = ("deck_m", "reach_m", "root_length_m", "root_width_m", "tip_width_m", "foot_depth_m")
TOWER_KEYS = ("height_m", "lattice_height_m", "sections", "floors", "deck", "core", "plinth", "cladding", "stairs",
              "roof")
PATTERNS = ("x_brace", "k_brace", "chevron")
## Per table: the keys that are lengths in metres (> 0); counts are integers >= 1 (COUNT_KEYS).
FLOORS_KEYS = ("pitch_m", "count")
DECK_KEYS = ("thickness_m", "inset_m")
CORE_KEYS = ("size_m",)
PLINTH_KEYS = ("height_m", "width_m")
CLADDING_KEYS = ("bottom_m", "top_m", "course_m", "columns", "seam_m", "thickness_m", "recess_m")
STAIRS_KEYS = ("width_m", "thickness_m", "run_m", "tray_width_m", "tray_depth_m")
ROOF_KEYS = ("deck_top_m", "plate_thickness_m", "overhang_m", "block_size_m", "block_base_m", "block_height_m",
             "louvre_count", "louvre_height_m", "louvre_depth_m", "railing_height_m", "railing_member_m",
             "railing_post_pitch_m")
MOUNT_KEYS = ("deck_height_m", "measured_height_m", "base_across_m", "base_toward_m", "top_across_m", "plinth_height_m",
              "deck_drop_m", "deck_plate_m", "pit_depth_m", "ring_outer_diameter_m", "ring_opening_diameter_m",
              "ring_thickness_m", "chamber_radius_m", "clamp_count", "opening_width_ground_m", "opening_width_deck_m",
              "opening_top_m", "lintel_depth_m", "lintel_height_m", "duct_radius_m", "ridge_top_m", "rib_pitch_m",
              "hood_inner_y_m", "hood_outer_y_m", "hood_height_m", "hood_drop_m", "door_width_m", "door_height_m",
              "qd_count", "qd_head_radius_m", "tile_m", "course_m", "panel_m", "seam_m", "map_size")
TRENCH_KEYS = ("centre_m", "width_m", "length_m", "wall_height_m", "wall_thickness_m", "hardstand_lift_m",
               "concrete_tile_m", "slab_m", "joint_m", "concrete_map_size", "foot_x_m", "foot_width_m", "foot_height_m",
               "mask_range_m")
COUNT_KEYS = ("count", "columns", "louvre_count")


class DetailFileError(ValueError):
    """The detail file misses a key or has a value of the wrong type or unit."""


def _need(table: dict, keys, where: str) -> None:
    for k in keys:
        if k not in table:
            raise DetailFileError(f"{where}: missing key '{k}'")


def _positive(table: dict, key: str, where: str) -> None:
    v = table[key]
    if isinstance(v, bool) or not isinstance(v, (int, float)) or v <= 0.0:
        raise DetailFileError(f"{where}.{key}: must be a positive number in metres, got {v!r}")


def load(path) -> dict:
    with open(Path(path), "rb") as f:
        data = tomllib.load(f)
    _need(data, ("tower", "profiles", "chopstick", "carriage", "qd_arm", "mount", "trench"), "file")
    tower, profiles, chop = data["tower"], data["profiles"], data["chopstick"]
    _need(tower, TOWER_KEYS, "tower")
    _positive(tower, "height_m", "tower")
    if not tower["sections"]:
        raise DetailFileError("tower.sections: empty")
    _positive(tower, "lattice_height_m", "tower")
    tables = (("floors", FLOORS_KEYS), ("deck", DECK_KEYS), ("core", CORE_KEYS), ("plinth", PLINTH_KEYS),
              ("cladding", CLADDING_KEYS), ("stairs", STAIRS_KEYS), ("roof", ROOF_KEYS))
    for table, keys in tables:
        if not isinstance(tower.get(table), dict):
            raise DetailFileError(f"tower.{table}: must be a table")
        _need(tower[table], keys, f"tower.{table}")
        for k in keys:
            if k in COUNT_KEYS:
                v = tower[table][k]
                if isinstance(v, bool) or not isinstance(v, int) or v < 1:
                    raise DetailFileError(f"tower.{table}.{k}: must be an integer >= 1, got {v!r}")
            else:
                _positive(tower[table], k, f"tower.{table}")
    if tower["cladding"]["top_m"] <= tower["cladding"]["bottom_m"]:
        raise DetailFileError("tower.cladding: top_m must be above bottom_m")
    for name, p in profiles.items():
        _need(p, ("kind", "size_m", "wall_m"), f"profiles.{name}")
        if p["kind"] not in PROFILE_KINDS:
            raise DetailFileError(f"profiles.{name}.kind: '{p['kind']}' not in {PROFILE_KINDS}")
        _positive(p, "size_m", f"profiles.{name}")
        _positive(p, "wall_m", f"profiles.{name}")
    for i, s in enumerate(tower["sections"]):
        where = f"tower.sections[{i}]"
        _need(s, SECTION_KEYS, where)
        for k in ("height_m", "face_width_m", "pitch_m"):
            _positive(s, k, where)
        if s["pattern"] not in PATTERNS:
            raise DetailFileError(f"{where}.pattern: '{s['pattern']}' not in {PATTERNS}")
        for k in ("leg_profile", "brace_profile"):
            if s[k] not in profiles:
                raise DetailFileError(f"{where}.{k}: unknown profile '{s[k]}'")
    _need(chop, CHOPSTICK_KEYS, "chopstick")
    for k in CHOPSTICK_KEYS:
        if not k.endswith("_profile"):
            _positive(chop, k, "chopstick")
    for table, keys in (("carriage", CARRIAGE_KEYS), ("qd_arm", QD_ARM_KEYS), ("mount", MOUNT_KEYS),
                        ("trench", TRENCH_KEYS)):
        _need(data[table], keys, table)
        for k in keys:
            _positive(data[table], k, table)
    for k in ("chord_profile", "brace_profile", "thin_profile"):
        if chop[k] not in profiles:
            raise DetailFileError(f"chopstick.{k}: unknown profile '{chop[k]}'")
    return data


LIGHTS_FILE_NAME = "starbase_olp2_lights.toml"
LIGHTS_KEYS = ("dusk_deg", "fade_deg", "max_shadowed", "shadow_range_m", "point_angle_rad", "flash_per_min", "flash_on_s",
               "red_cd", "white_day_cd")
GROUP_KINDS = ("flood", "omni", "obstruction")
GROUP_KEYS = ("name", "kind", "ref", "along_m", "across_m", "z_m", "range_m")


def load_lights(path) -> dict:
    """Lamps file (config/structures/starbase_olp2_lights.toml): `[lights]` settings and `[[lights.group]]` fixtures.

    Flood groups need lumens, cct_k, half_angle_deg and aim; omni groups lumens and cct_k; at most `max_shadowed`
    shadow-casting lights in all (shadow = per group and side, the first n of the group's `count`)."""
    with open(Path(path), "rb") as f:
        data = tomllib.load(f)
    settings = data.get("lights")
    if not isinstance(settings, dict):
        raise DetailFileError("lights: must be a table")
    _need(settings, LIGHTS_KEYS, "lights")
    if settings["dusk_deg"] >= 0.0:
        raise DetailFileError("lights.dusk_deg: the Sun's depression threshold must be negative")
    groups = settings.get("group", [])
    if not groups:
        raise DetailFileError("lights.group: empty")
    shadowed = 0
    for i, g in enumerate(groups):
        where = f"lights.group[{i}]"
        _need(g, GROUP_KEYS, where)
        if g["kind"] not in GROUP_KINDS:
            raise DetailFileError(f"{where}.kind: '{g['kind']}' not in {GROUP_KINDS}")
        if g["ref"] not in ("face", "axis"):
            raise DetailFileError(f"{where}.ref: must be 'face' or 'axis'")
        _positive(g, "range_m", where)
        extra = {"flood": ("lumens", "cct_k", "half_angle_deg", "aim"), "omni": ("lumens", "cct_k"), "obstruction": ()}
        _need(g, extra[g["kind"]], where)
        for k in ("lumens", "cct_k", "half_angle_deg"):
            if k in g:
                _positive(g, k, where)
        sides = 2 if g.get("mirror", False) else 1
        shadowed += min(int(g.get("shadow", 0)), int(g.get("count", 1))) * sides
    if shadowed > settings["max_shadowed"]:
        raise DetailFileError(f"lights: {shadowed} shadow-casting lights, max_shadowed is {settings['max_shadowed']}")
    return data


_VALUE_LINE = re.compile(r"^\s*[A-Za-z_][\w.]*\s*=")
_TAG = re.compile(r"#.*\[(E|S)\]")


def value_lines_without_tag(path) -> list[tuple[int, str]]:
    """(line number, text) of every `key = value` line that has no `# [E]` / `# [S]` comment."""
    bad = []
    for n, line in enumerate(Path(path).read_text(encoding="utf-8").splitlines(), 1):
        if _VALUE_LINE.match(line) and not _TAG.search(line):
            bad.append((n, line))
    return bad
