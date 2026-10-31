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
TOWER_HEIGHT_M = 144.5  # [E] spec A


def test_required_keys_present():
    d = detail_file.load(DRAFT)
    assert set(detail_file.TOWER_KEYS) <= set(d["tower"])
    assert all(set(detail_file.SECTION_KEYS) <= set(s) for s in d["tower"]["sections"])
    assert set(detail_file.CHOPSTICK_KEYS) <= set(d["chopstick"])
    assert d["profiles"]


def test_section_heights_sum_to_tower_height():
    d = detail_file.load(DRAFT)
    total = sum(s["height_m"] for s in d["tower"]["sections"])
    assert total == pytest.approx(TOWER_HEIGHT_M, rel=0.01)
    assert d["tower"]["height_m"] == pytest.approx(TOWER_HEIGHT_M, rel=0.01)


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
    f.write_text(text.replace("truss_pitch_m", "pitch_typo_m"), encoding="utf-8")
    with pytest.raises(detail_file.DetailFileError, match="truss_pitch_m"):
        detail_file.load(f)
    f.write_text(text.replace("length_m = 26.0", "length_m = -26.0"), encoding="utf-8")
    with pytest.raises(detail_file.DetailFileError, match="positive"):
        detail_file.load(f)


@pytest.mark.parametrize("old, new, match", [
    ("[tower.deck]", "[tower.dek]", "deck"),
    ("ring_width_m", "ring_wdth_m", "ring_width_m"),
    ("size_m = 3.6 ", "size_m = -3.6 ", "positive"),
    ("pyramid_top_fraction = 0.3", "pyramid_top_fraction = 1.5", "fraction"),
    ('pattern = "x_brace"', 'pattern = "zigzag"', "pattern"),
])
def test_load_validates_deck_core_roof_foot_and_pattern(tmp_path, old, new, match):
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
