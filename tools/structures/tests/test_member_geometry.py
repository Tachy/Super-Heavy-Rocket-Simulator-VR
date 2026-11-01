"""Member faces: closed shells wound outward (back faces are culled in the game, an inward face reads as a hole)."""
import sys
from collections import Counter
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "blender"))
import member_geometry  # noqa: E402

KINDS = [("box", 2.2, 0.05, None), ("rect", 1.0, 0.04, 0.3), ("h_beam", 0.5, 0.04, None),
         ("angle", 0.5, 0.04, None), ("tube", 0.8, 0.03, None)]
LENGTH_M = 3.0
ROUND_DIGITS = 9


def _key(p):
    return tuple(round(c, ROUND_DIGITS) + 0.0 for c in p)


@pytest.mark.parametrize("kind,size,wall,size_y", KINDS)
def test_every_edge_is_met_once_in_each_direction(kind, size, wall, size_y):
    """A closed, consistently wound shell: each directed edge is matched by its reverse on the neighbouring face."""
    edges = Counter()
    for verts, _uvs, _smooth in member_geometry.member_faces(kind, size, wall, LENGTH_M, size_y):
        for a, b in zip(verts, verts[1:] + verts[:1]):
            edges[(_key(a), _key(b))] += 1
    assert all(count == 1 and edges[(b, a)] == 1 for (a, b), count in edges.items())


@pytest.mark.parametrize("kind,size,wall,size_y", KINDS)
def test_shell_winds_outward(kind, size, wall, size_y):
    """Positive signed volume, together with the edge test: all faces outward, none inward."""
    volume = 0.0
    for verts, _uvs, _smooth in member_geometry.member_faces(kind, size, wall, LENGTH_M, size_y):
        a = verts[0]
        for b, c in zip(verts[1:], verts[2:]):
            volume += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                       + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
    assert volume > 0.0
