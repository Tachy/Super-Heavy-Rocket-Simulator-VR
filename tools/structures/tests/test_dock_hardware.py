"""One source for the probe-drogue drawing numbers (plan 2026-11-04 Task 4 fix, I1 / I2): the shared table, and its core numbers pinned to the C++ defaults."""
import re
import sys
import tomllib
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "blender"))
import dorsal_layout as dl  # noqa: E402
import vehicle_spec  # noqa: E402

HEADER = vehicle_spec.ROOT / "core" / "include" / "starsim" / "contact" / "probe_drogue_interface.hpp"
SHARED = vehicle_spec.VEHICLES / "detail" / vehicle_spec.DOCK_HARDWARE


def _core_default(name):
    m = re.search(r"double\s+" + name + r"\s*=\s*([0-9.eE+-]+)\s*;", HEADER.read_text(encoding="utf-8"))
    assert m, f"{name} not found in {HEADER.name}"
    return float(m.group(1))


def test_core_numbers_are_pinned_to_the_cpp_defaults():
    """The drawing's cone, seat, head and stroke ARE ProbeDrogueParameters' defaults (mirrored in dock_hardware.toml, nowhere else)."""
    with open(SHARED, "rb") as f:
        d = tomllib.load(f)["dorsal"]
    assert d["drogue_depth_m"] == _core_default("cone_depth_m")
    assert d["drogue_mouth_radius_m"] == _core_default("mouth_radius_m")
    assert d["drogue_throat_radius_m"] == _core_default("seat_radius_m")
    assert d["probe_head_radius_m"] == _core_default("head_radius_m")
    assert d["probe_extension_m"] == _core_default("extension_m")


@pytest.mark.parametrize("vehicle", ["starship_tanker", "starship_depot"])
def test_ship_detail_files_carry_no_copy_of_the_numbers(vehicle):
    with open(vehicle_spec.VEHICLES / "detail" / f"{vehicle}.toml", "rb") as f:
        assert tomllib.load(f)["dorsal"] == {"shared": True}


def test_tanker_and_depot_draw_with_the_same_numbers():
    tanker = vehicle_spec.load_starship("starship_tanker")["dorsal"]["drawing"]
    depot = vehicle_spec.load_starship("starship_depot")["dorsal"]["drawing"]
    assert tanker == depot
    with open(SHARED, "rb") as f:
        assert tanker == {k: v for k, v in tomllib.load(f)["dorsal"].items() if k in vehicle_spec.DORSAL_KEYS}


def test_a_seated_head_is_inside_the_cone_by_the_cone_depth():
    """Docked pose: root on the interface plane, head cone_depth beyond it = the seat (the cone's floor); extended it stands the core's extension beyond the seat."""
    spec = vehicle_spec.load_starship("starship_tanker")
    d = spec["dorsal"]["drawing"]
    hull_r = 0.5 * spec["stages"]["ship"]["diameter_m"]
    n = dl.element_nodes(spec["dorsal"]["ports"])[0]
    off = dl.probe_offsets_m(d, hull_r, n["element_m"], n["axis"])
    assert off["seated"] == d["drogue_depth_m"] == 0.5
    assert dl.drogue_cone_radius_m(d, off["seated"]) == pytest.approx(d["drogue_throat_radius_m"])
    assert off["extended"] - off["seated"] == pytest.approx(d["probe_extension_m"])
