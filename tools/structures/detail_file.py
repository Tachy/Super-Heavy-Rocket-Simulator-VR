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
TOWER_KEYS = ("height_m", "sections")


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
