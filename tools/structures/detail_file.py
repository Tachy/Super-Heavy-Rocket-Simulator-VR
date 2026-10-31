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
CHOPSTICK_KEYS = ("length_m", "width_m", "depth_m", "end_depth_m", "knee_depth_m", "foot_depth_m", "root_kink_x_m",
                  "root_kink_deg", "tip_kink_x_m", "tip_kink_deg", "inner_face_y_m", "rail_profile", "chord_profile",
                  "brace_profile", "thin_profile")
CARRIAGE_KEYS = ("width_m", "bottom_depth_m", "hinge_y_m", "drum_diameter_m", "cable_count", "cable_size_m")
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
    _need(data, ("tower", "profiles", "chopstick", "carriage", "qd_arm"), "file")
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
    for table, keys in (("carriage", CARRIAGE_KEYS), ("qd_arm", QD_ARM_KEYS)):
        _need(data[table], keys, table)
        for k in keys:
            _positive(data[table], k, table)
    for k in ("rail_profile", "chord_profile", "brace_profile", "thin_profile"):
        if chop[k] not in profiles:
            raise DetailFileError(f"chopstick.{k}: unknown profile '{chop[k]}'")
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
