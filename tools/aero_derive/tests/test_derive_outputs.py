"""The derivation script's rows: every Mach, every derived value with an error, a finite comparison."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import derive_ascent_descent as dad  # noqa: E402
import geometry  # noqa: E402

pytestmark = pytest.mark.skipif(not (geometry.MODEL_ROOT / "starship_block3" / "manifest.json").exists(),
                                reason="built vehicle models are git-ignored")


@pytest.fixture(scope="module")
def rows():
    return dad.run()


def test_every_mach_in_every_case(rows):
    assert sorted({r["mach"] for r in rows if r["case"] == "ascent_stack"}) == dad.ASCENT_MACHS
    for case in ("descent_body", "descent_total_drawn_12.63", "descent_total_config_13.5"):
        assert sorted({r["mach"] for r in rows if r["case"] == case}) == sorted(dad.MACHS)
    assert {r["column"] for r in rows if r["case"] == "ascent_stack"} == {"on", "off"}


def test_every_value_has_an_error_and_the_comparison_is_finite(rows):
    for r in rows:
        assert r["derived"] > 0.0 and math.isfinite(r["derived"])
        assert r["derived_err"] > 0.0 and math.isfinite(r["derived_err"])
        assert r["known"] > 0.0 and math.isfinite(r["ratio"]) and math.isfinite(r["diff"])
        assert r["verdict"].split(" (")[0] in ("within", "outside")


def test_fins_are_compared_separately_and_with_both_areas(rows):
    body = {r["mach"]: r["derived"] for r in rows if r["case"] == "descent_body"}
    drawn = {r["mach"]: r for r in rows if r["case"] == "descent_total_drawn_12.63"}
    config = {r["mach"]: r for r in rows if r["case"] == "descent_total_config_13.5"}
    for mach, value in body.items():
        assert drawn[mach]["derived"] == pytest.approx(value + drawn[mach]["fin_cd"])
        assert config[mach]["fin_cd"] / drawn[mach]["fin_cd"] == pytest.approx(13.5 / 12.626, rel=1e-3)


def test_flaps_stand_radial_in_the_ascent(rows):
    # d = 90 is the case; d = 0 only a sensitivity
    assert all("flaps_d0" in r for r in rows if r["case"] == "ascent_stack")
    assert all(r["flaps"] > 0.0 for r in rows if r["case"] == "ascent_stack")
