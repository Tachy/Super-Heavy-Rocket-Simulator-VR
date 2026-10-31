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
CHOPSTICK_KEYS = ("length_m", "width_m", "depth_m", "truss_pitch_m", "rail_profile", "chord_profile", "brace_profile")
TOWER_KEYS = ("height_m", "sections", "deck", "core", "roof", "foot")
PATTERNS = ("x_brace", "k_brace", "chevron")
## Per table: the keys that are lengths in metres (> 0); roof.pyramid_top_fraction is a fraction in (0, 1].
DECK_KEYS = ("thickness_m", "inset_m")
CORE_KEYS = ("size_m",)
ROOF_KEYS = ("plate_thickness_m", "overhang_m", "pyramid_height_m")
FOOT_KEYS = ("ring_width_m", "ring_thickness_m")


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
    _need(data, ("tower", "profiles", "chopstick"), "file")
    tower, profiles, chop = data["tower"], data["profiles"], data["chopstick"]
    _need(tower, TOWER_KEYS, "tower")
    _positive(tower, "height_m", "tower")
    if not tower["sections"]:
        raise DetailFileError("tower.sections: empty")
    for table, keys in (("deck", DECK_KEYS), ("core", CORE_KEYS), ("roof", ROOF_KEYS), ("foot", FOOT_KEYS)):
        if not isinstance(tower[table], dict):
            raise DetailFileError(f"tower.{table}: must be a table")
        _need(tower[table], keys, f"tower.{table}")
        for k in keys:
            _positive(tower[table], k, f"tower.{table}")
    _need(tower["roof"], ("pyramid_top_fraction",), "tower.roof")
    fraction = tower["roof"]["pyramid_top_fraction"]
    if isinstance(fraction, bool) or not isinstance(fraction, (int, float)) or not 0.0 < fraction <= 1.0:
        raise DetailFileError(f"tower.roof.pyramid_top_fraction: must be a fraction in (0, 1], got {fraction!r}")
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
    for k in ("length_m", "width_m", "depth_m", "truss_pitch_m"):
        _positive(chop, k, "chopstick")
    for k in ("rail_profile", "chord_profile", "brace_profile"):
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
