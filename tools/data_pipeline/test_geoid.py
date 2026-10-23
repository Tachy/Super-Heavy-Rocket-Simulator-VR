"""Unit tests of the EGM2008 geoid pipeline (downsampling, CSV round trip, grid header). No network.

Run: python -m pytest tools/data_pipeline/test_geoid.py
"""

import sys
import tomllib
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import geoid  # noqa: E402


def synthetic_grid() -> np.ndarray:
    """A 2.5' node grid (4321 x 8640) whose value encodes the node index: cm = line * 10 + sample % 7 - 3."""
    lines = np.arange(geoid.LINES)[:, None]
    samples = np.arange(geoid.SAMPLES)[None, :]
    return (lines * 10 + samples % 7 - 3).astype(np.int32)


def test_downsample_1deg_takes_node_values():
    cm = synthetic_grid()
    coarse = geoid.downsample_1deg(cm)
    assert coarse.shape == (181, 361)
    step = geoid.NODES_PER_DEGREE
    for i, j in [(0, 0), (1, 1), (90, 180), (180, 359), (37, 211)]:
        assert coarse[i, j] == cm[i * step, j * step]  # the node itself, not a block average
    # lon +180 repeats lon -180 (the 2.5' grid has no +180 column)
    assert np.array_equal(coarse[:, 360], coarse[:, 0])


def test_csv_round_trip(tmp_path):
    coarse = geoid.downsample_1deg(synthetic_grid())
    path = tmp_path / "egm2008_1deg.csv"
    geoid.write_coarse_csv(path, coarse)
    text = path.read_text(encoding="utf-8")
    assert text.startswith("#")
    back = geoid.read_coarse_csv(path)
    assert back.shape == (181, 361)
    assert np.array_equal(back, coarse)


def test_grid_header_fields():
    text = geoid.grid_header_text([(10.0, 20.0, 1.25)])
    header = tomllib.loads(text)
    grid = header["grid"]
    assert grid["projection"] == "geographic"
    assert grid["lines"] == 4321 and grid["samples"] == 8640
    assert grid["north_latitude_deg"] == 90.0 and grid["west_longitude_deg"] == -180.0
    assert abs(grid["step_latitude_deg"] - 1.0 / 24.0) < 1e-15
    assert abs(grid["step_longitude_deg"] - 1.0 / 24.0) < 1e-15
    assert grid["global_longitude"] is True
    assert grid["height_reference"] == "egm2008_undulation"
    assert grid["dn_scale_m"] == 0.01
    assert header["check"][0] == {"latitude_deg": 10.0, "longitude_deg": 20.0, "height_m": 1.25}
