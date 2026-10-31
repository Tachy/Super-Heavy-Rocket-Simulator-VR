"""Hardstand colour (the terrain imagery's bright percentile) and the terrain mask under the apron."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools" / "structures"))
sys.path.insert(0, str(ROOT / "tools" / "structures" / "blender"))
import detail_file  # noqa: E402
import mount_layout as ml  # noqa: E402
import mount_textures as mt  # noqa: E402


@pytest.fixture(scope="module")
def detail():
    return detail_file.load(ROOT / "config" / "structures" / "starbase_olp2.toml")


def test_concrete_map_matches_the_bright_terrain_imagery(detail):
    """No bright patch against the terrain: the map's mean is the 95th luminance percentile of the NAIP imagery at the site."""
    mean = mt.bake_concrete(detail["trench"]).reshape(-1, 3).mean(axis=0) / 255.0
    assert mean == pytest.approx(mt.TERRAIN_BRIGHT, abs=0.012)


def test_apron_mask_covers_the_apron_a_texel_inside_and_not_the_pit(detail):
    t = detail["trench"]
    mask = mt.bake_apron_mask(detail)[:, :, 0] > 0
    half, px = t["hardstand_mask_half_m"], t["hardstand_mask_px_m"]

    def at(x, y):
        return bool(mask[int((y + half) / px), int((x + half) / px)])

    sh = ml.Shape(detail)
    ramp = ml.ramp_end_m(detail)
    assert not at(0.0, 0.0) and not at(0.0, ramp - 5.0) and not at(-sh.annex_x + 2.0, 0.0)  # mount, ramp, annex stay terrain
    assert at(30.0, 80.0) or at(80.0, 30.0) or at(60.0, 60.0)
    outline = ml.hardstand_outline(detail)
    assert max(max(abs(x), abs(y)) for x, y in outline) < half - 2.0
    area = mask.sum() * px * px
    cover = sum(ml.polygon_area(p) for p in ml.hardstand_pieces(detail))
    assert 0.9 * cover < area < cover  # eroded by one texel: a little less than the plate
    ys, xs = np.nonzero(mask)
    pieces = ml.hardstand_pieces(detail)
    for j, i in list(zip(ys, xs))[::997]:  # every masked texel lies on a piece of the plate
        x, y = -half + (i + 0.5) * px, -half + (j + 0.5) * px
        assert any(ml.clip_rect(p, x - 0.5, x + 0.5, y - 0.5, y + 0.5) for p in pieces), (x, y)
