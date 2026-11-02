"""Baked surfaces of the Falcon 9 and the Crew Dragon: sizes, determinism, the detail file's keys, seamless tiles, the soot parameter. No Blender needed."""
import copy
import io
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools" / "structures"))
import surface_textures as st  # noqa: E402
import vehicle_spec  # noqa: E402


@pytest.fixture(scope="module")
def falcon():
    return vehicle_spec.load_falcon9("falcon9_block5")


@pytest.fixture(scope="module")
def dragon():
    return vehicle_spec.load_dragon("crew_dragon")


def _texture_mib(path):
    from PIL import Image
    with Image.open(path) as im:
        w, h = im.size
        return w * h * len(im.getbands()) / 2**20, (w, h)


def test_falcon9_maps_exist_with_their_sizes(tmp_path, falcon):
    entries = st.write_falcon9(tmp_path, falcon)
    assert {"paint_hull", "paint_joint", "paint_interstage", "paint_octaweb", "paint_leg", "paint_hull2"} <= set(entries)
    for key, size in st.FALCON9_SIZES.items():
        assert _texture_mib(tmp_path / key)[1] == size
    assert entries["paint_hull"]["wear_file"] == st.SOOT_FILE
    assert entries["paint_interstage"]["wear_file"] == st.SOOT_FILE


def test_dragon_maps_exist_with_their_sizes(tmp_path, dragon):
    entries = st.write_dragon(tmp_path, dragon)
    assert {"paint_capsule", "paint_cowl", "paint_shield", "paint_band", "paint_array", "paint_radiator"} <= set(entries)
    for key, size in st.DRAGON_SIZES.items():
        assert _texture_mib(tmp_path / key)[1] == size


def test_texture_memory_is_stated_and_small(tmp_path, falcon, dragon):
    st.write_falcon9(tmp_path, falcon)
    st.write_dragon(tmp_path, dragon)
    total = sum(_texture_mib(tmp_path / f)[0] for f in (*st.FALCON9_SIZES, *st.DRAGON_SIZES))
    assert total * 4.0 / 3.0 < 32.0  # [E] with mips; the sum is reported in vehicle-models.md


def test_bake_is_deterministic(tmp_path, falcon, dragon):
    a, b = tmp_path / "a", tmp_path / "b"
    a.mkdir()
    b.mkdir()
    st.write_falcon9(a, falcon)
    st.write_falcon9(b, falcon)
    st.write_dragon(a, dragon)
    st.write_dragon(b, dragon)
    for f in (*st.FALCON9_SIZES, *st.DRAGON_SIZES):
        assert (a / f).read_bytes() == (b / f).read_bytes(), f


@pytest.mark.parametrize("key", st.FALCON9_SURFACE_KEYS)
def test_falcon9_detail_keys_are_required(falcon, key):
    detail = copy.deepcopy(falcon["detail"])
    del detail["surface"][key]
    with pytest.raises(vehicle_spec.VehicleSpecError):
        vehicle_spec.validate_falcon9_detail(detail)


@pytest.mark.parametrize("key", st.DRAGON_SURFACE_KEYS)
def test_dragon_detail_keys_are_required(dragon, key):
    detail = copy.deepcopy(dragon["detail"])
    del detail["surface"][key]
    with pytest.raises(vehicle_spec.VehicleSpecError):
        vehicle_spec.validate_dragon_detail(detail)


def test_soot_is_one_parameter(falcon):
    clean = copy.deepcopy(falcon)
    clean["detail"]["surface"]["soot_level"] = 0.0
    assert st.soot_map(clean).max() == 0
    full = st.soot_map(falcon)
    assert full.max() > 0
    half = copy.deepcopy(falcon)
    half["detail"]["surface"]["soot_level"] = 0.5 * falcon["detail"]["surface"]["soot_level"]
    assert abs(float(st.soot_map(half).mean()) - 0.5 * float(full.mean())) < 1.0


def test_soot_closes_around_the_hull_and_follows_the_profile(falcon):
    soot = st.soot_map(falcon).astype(float)
    assert np.abs(soot[:, 0] - soot[:, -1]).mean() < 2.0 * np.abs(soot[:, 0] - soot[:, 1]).mean() + 1.0
    rows = soot.mean(axis=1)
    length = st.soot_length_m(falcon)
    row = lambda x: int(x / length * soot.shape[0])  # noqa: E731
    assert rows[row(22.0)] > rows[row(40.0)]  # the photos: darkest in the hull's middle, light under the interstage
    assert rows[row(22.0)] > rows[row(3.0)]


def test_tiles_are_seamless(falcon, dragon):
    for image in (st.hull_tile(falcon), st.array_tile(dragon), st.interstage_tile(falcon)):
        step_inside = np.abs(image[:, 1:, :3].astype(float) - image[:, :-1, :3]).mean()
        step_wrap = np.abs(image[:, 0, :3].astype(float) - image[:, -1, :3]).mean()
        assert step_wrap < 4.0 * step_inside + 6.0


def test_new_dragon_has_no_soot_and_flown_has(dragon):
    new = st.capsule_skin(dragon)
    flown = copy.deepcopy(dragon)
    flown["detail"]["surface"]["flown_soot"] = 1.0
    sooted = st.capsule_skin(flown)
    assert dragon["detail"]["surface"]["flown_soot"] == 0.0
    assert sooted[..., :3].mean() < new[..., :3].mean()


def test_new_dragon_shield_is_the_new_albedo(dragon):
    s = dragon["detail"]["surface"]
    assert st.shield_albedo(dragon["detail"]) == pytest.approx(s["shield_new_albedo"])
    flown = copy.deepcopy(dragon["detail"])
    flown["surface"]["flown_soot"] = 1.0
    assert st.shield_albedo(flown) == pytest.approx(flown["paint"]["shield_albedo"])


def test_radiator_tile_closes_around_the_trunk(dragon):
    period = st.radiator_period_m(dragon)
    circumference = dragon["stages"]["trunk"]["diameter_m"] * np.pi
    assert abs(circumference / period[0] - round(circumference / period[0])) < 1e-9
    assert abs(period[0] - dragon["detail"]["surface"]["radiator_panel_m"]) < 0.02


def test_png_is_plain_8_bit():
    data = st.png_bytes(np.zeros((4, 8, 4), dtype=np.uint8))
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    from PIL import Image
    assert Image.open(io.BytesIO(data)).size == (8, 4)
