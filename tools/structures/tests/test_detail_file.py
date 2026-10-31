"""Tests of the structure detail file loader and the draft Pad B file (prototype plan task 1)."""
import sys
import tomllib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools" / "structures"))
sys.path.insert(0, str(ROOT / "tools" / "structures" / "measure"))

import detail_file  # noqa: E402
import ratio  # noqa: E402

DRAFT = ROOT / "config" / "structures" / "starbase_olp2.toml"
ROOF_DECK_M = 144.0  # [E] measurements: roof deck 144 +-5 m
LATTICE_TOP_M = 139.0  # [E] measurements: lattice panels end at 139 +-5 m
SILHOUETTE_MAX_M = 148.0  # plan task 1: silhouette top incl. railing


def test_required_keys_present():
    d = detail_file.load(DRAFT)
    assert set(detail_file.TOWER_KEYS) <= set(d["tower"])
    assert all(set(detail_file.SECTION_KEYS) <= set(s) for s in d["tower"]["sections"])
    assert set(detail_file.CHOPSTICK_KEYS) <= set(d["chopstick"])
    assert d["profiles"]


def test_section_heights_sum_to_lattice_height():
    d = detail_file.load(DRAFT)
    total = sum(s["height_m"] for s in d["tower"]["sections"])
    assert total == pytest.approx(LATTICE_TOP_M, abs=0.2)
    assert d["tower"]["lattice_height_m"] == pytest.approx(total, abs=1.0e-6)
    assert d["tower"]["height_m"] == pytest.approx(ROOF_DECK_M, abs=0.01)


def test_floors_are_26_of_5_3_m_and_fill_the_lattice():
    d = detail_file.load(DRAFT)
    floors = d["tower"]["floors"]
    assert floors["count"] == 26
    assert floors["pitch_m"] == pytest.approx(5.3, abs=0.1)
    assert floors["count"] * floors["pitch_m"] == pytest.approx(d["tower"]["lattice_height_m"], abs=0.2)
    for s in d["tower"]["sections"]:  # a section holds a whole number of floors
        n = s["height_m"] / floors["pitch_m"]
        assert n == pytest.approx(round(n), abs=0.01)


def test_member_sizes_plinth_and_cladding_follow_the_measurements():
    d = detail_file.load(DRAFT)
    assert d["profiles"]["leg"]["size_m"] == pytest.approx(2.2)
    assert d["profiles"]["brace"]["size_m"] == pytest.approx(0.5)
    assert d["tower"]["plinth"]["height_m"] == pytest.approx(12.2)
    clad = d["tower"]["cladding"]
    assert (clad["bottom_m"], clad["top_m"]) == (pytest.approx(12.2), pytest.approx(79.0))
    assert clad["bottom_m"] >= d["tower"]["plinth"]["height_m"] - 1.0e-6  # the plates start on the plinth


def test_x_brace_crossings_7_to_8_m_in_the_open_part():
    d = detail_file.load(DRAFT)
    open_sections = [s for s in d["tower"]["sections"] if s["pattern"] == "x_brace"]
    assert open_sections
    assert all(7.0 <= s["pitch_m"] <= 8.1 for s in open_sections)


def test_flat_roof_block_no_pyramid_and_silhouette_below_148_m():
    d = detail_file.load(DRAFT)
    roof = d["tower"]["roof"]
    assert not any("pyramid" in k for k in roof)
    block_top = roof["block_base_m"] + roof["block_height_m"]
    rail_top = roof["deck_top_m"] + roof["railing_height_m"]
    assert max(block_top, rail_top) <= SILHOUETTE_MAX_M
    assert roof["deck_top_m"] == pytest.approx(d["tower"]["height_m"])
    assert roof["block_base_m"] >= d["tower"]["lattice_height_m"] - 1.0e-6  # the block stands on the lattice top


def test_every_value_line_is_tagged():
    assert detail_file.value_lines_without_tag(DRAFT) == []


def test_untagged_line_is_detected(tmp_path):
    f = tmp_path / "x.toml"
    f.write_text('a = 1.0   # [E] ok\nb = 2.0\n', encoding="utf-8")
    assert detail_file.value_lines_without_tag(f) == [(2, "b = 2.0")]


@pytest.mark.parametrize("drop", ["tower", "profiles", "chopstick"])
def test_load_raises_on_missing_table(tmp_path, drop):
    data = tomllib.loads(DRAFT.read_text(encoding="utf-8"))
    text = DRAFT.read_text(encoding="utf-8")
    if drop == "tower":
        text = text.replace("[tower]\n", "[towr]\n", 1)
    elif drop == "profiles":
        text = text.replace("[profiles.", "[profs.")
    else:
        text = text.replace("[chopstick]", "[chop]")
    assert data  # draft itself parses
    f = tmp_path / "bad.toml"
    f.write_text(text, encoding="utf-8")
    with pytest.raises(detail_file.DetailFileError):
        detail_file.load(f)


def test_load_raises_on_missing_key_and_bad_unit(tmp_path):
    text = DRAFT.read_text(encoding="utf-8")
    f = tmp_path / "a.toml"
    f.write_text(text.replace("tip_kink_x_m", "pitch_typo_m"), encoding="utf-8")
    with pytest.raises(detail_file.DetailFileError, match="tip_kink_x_m"):
        detail_file.load(f)
    f.write_text(text.replace("length_m = 27.0", "length_m = -27.0"), encoding="utf-8")
    with pytest.raises(detail_file.DetailFileError, match="positive"):
        detail_file.load(f)


@pytest.mark.parametrize("old, new, match", [
    ("[tower.deck]", "[tower.dek]", "deck"),
    ("[tower.plinth]", "[tower.plnth]", "plinth"),
    ("top_m = 79.0", "top_m = -79.0", "positive"),
    ("columns = 2", "columns = 0", "columns"),
    ("size_m = 3.6 ", "size_m = -3.6 ", "positive"),
    ("louvre_count = 6", "louvre_count = 0", "louvre_count"),
    ('pattern = "x_brace"', 'pattern = "zigzag"', "pattern"),
])
def test_load_validates_tower_tables_and_pattern(tmp_path, old, new, match):
    text = DRAFT.read_text(encoding="utf-8")
    assert old in text
    f = tmp_path / "bad.toml"
    f.write_text(text.replace(old, new, 1), encoding="utf-8")
    with pytest.raises(detail_file.DetailFileError, match=match):
        detail_file.load(f)


def test_ratio_scale():
    pts = {"scale": {"a": [0, 0], "b": [90, 0], "length_m": 9.0},
           "measure": {"w": {"a": [0, 0], "b": [0, 120]}}}
    assert ratio.measure(pts)["w"] == pytest.approx(12.0)
