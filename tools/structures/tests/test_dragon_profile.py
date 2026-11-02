"""The generated Crew Dragon follows the typed, measured outline (no third-party file needed). Skips without generated models
(python tools/structures/build_models.py --vehicle crew_dragon)."""
import sys
import tomllib
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "measure"))
import dragon_profile as dp  # noqa: E402

GENERATED = (dp.MODELS / "dragon_capsule.gltf").is_file()
# [E] tolerances of the drawn surface: chord of the rows (2-5 cm pitch on a 20-40 deg slope), the bay's arch bends rows near its lip
TOLERANCE_M = {"body_profile": 0.012, "cowl_crown": 0.02, "bay_floor": 0.03, "nose_seam": 0.01, "nose_tip": 0.001, "hinge_point": 0.01, "fin_outline": 0.02}


def _detail():
    with open(dp.DETAIL, "rb") as f:
        return tomllib.load(f)


def test_profile_table_is_a_single_valued_meridian_ending_at_the_config_length():
    c = _detail()["capsule"]
    xs = [p[0] for p in c["profile_m"]]
    assert xs == sorted(xs) and len(set(xs)) == len(xs)
    with open(dp.VEHICLE, "rb") as f:
        v = tomllib.load(f)
    cap = next(s for s in v["stage"] if s["id"] == "capsule")
    assert xs[-1] == pytest.approx(cap["length_m"], abs=0.001)
    assert max(p[1] for p in c["profile_m"]) == pytest.approx(cap["diameter_m"] / 2.0, abs=0.001)


def test_seam_plane_passes_the_hinge_side_at_the_hinge():
    c = _detail()["capsule"]
    low = c["seam_axis_x_m"] - c["seam_slope"] * dp.lerp(c["profile_m"], c["nose_hinge_x_m"])
    assert low == pytest.approx(c["nose_hinge_x_m"], abs=0.03)


@pytest.mark.skipif(not GENERATED, reason="models not generated")
@pytest.mark.parametrize("key", sorted(TOLERANCE_M))
def test_generated_outline_against_the_typed_numbers(key):
    value, where = dp.deviations()[key]
    assert value <= TOLERANCE_M[key], f"{key}: {value:.4f} m at {where}"
