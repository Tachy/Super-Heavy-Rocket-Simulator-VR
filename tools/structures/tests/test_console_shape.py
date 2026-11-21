"""console_shape (fidelity Task 4): the filleted housing body and the ceiling posts are closed, consistently wound surfaces; the
housing has no edge sharper than its fillet radius; the post runs from base to top with its joint head at the top."""
import math
import sys
from collections import Counter
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "blender"))
import console_shape as cs  # noqa: E402

RADIUS_TOL = 0.35  # [E] the radius a bent edge implies (chord / (2 sin(dihedral / 2))) from face normals: exact on the edge fillets, up to
# 32 % low on the cube-sphere corner patches at 4 segments (their face normals are not the vertex normals); a box edge reads ~0. The exact
# check is the vertex distance to the inner box below.


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _norm(a):
    return math.sqrt(sum(x * x for x in a))


def _normal(verts, f):
    n = [0.0, 0.0, 0.0]
    for i in range(len(f)):
        a, b = verts[f[i]], verts[f[(i + 1) % len(f)]]
        n[0] += (a[1] - b[1]) * (a[2] + b[2])
        n[1] += (a[2] - b[2]) * (a[0] + b[0])
        n[2] += (a[0] - b[0]) * (a[1] + b[1])
    k = _norm(n)
    return tuple(x / k for x in n)


def _directed_edges(faces):
    return Counter((f[i], f[(i + 1) % len(f)]) for f in faces for i in range(len(f)))


def assert_closed(verts, faces):
    d = _directed_edges(faces)
    assert max(d.values()) == 1, "an edge is used twice in the same direction (winding flipped)"
    for (a, b) in d:
        assert (b, a) in d, f"open edge {a}-{b}"


def signed_volume(verts, faces):
    vol = 0.0
    for f in faces:
        for i in range(1, len(f) - 1):
            a, b, c = verts[f[0]], verts[f[i]], verts[f[i + 1]]
            vol += sum(x * y for x, y in zip(a, _cross(b, c))) / 6.0
    return vol


def edge_dihedrals(verts, faces):
    """[(dihedral deg, edge length)] for every interior edge."""
    owner = {}
    for fi, f in enumerate(faces):
        for i in range(len(f)):
            owner[(f[i], f[(i + 1) % len(f)])] = fi
    out = []
    for (a, b), fi in owner.items():
        if a < b and (b, a) in owner:
            n0, n1 = _normal(verts, faces[fi]), _normal(verts, faces[owner[(b, a)]])
            c = max(-1.0, min(1.0, sum(x * y for x, y in zip(n0, n1))))
            out.append((math.degrees(math.acos(c)), _norm(_sub(verts[a], verts[b]))))
    return out


def test_rounded_box_is_closed_and_has_the_given_extents():
    w, h, d, r = 1.3, 0.5, 0.16, 0.04
    verts, faces = cs.rounded_box(w, h, d, r, 4)
    assert_closed(verts, faces)
    assert signed_volume(verts, faces) > 0.0, "faces wound outward"
    for axis, size in enumerate((w, h, d)):
        lo, hi = min(v[axis] for v in verts), max(v[axis] for v in verts)
        assert lo == pytest.approx(-size / 2, abs=1e-9) and hi == pytest.approx(size / 2, abs=1e-9)
    # volume of a rounded box: box minus the 12 edge and 8 corner remainders
    exact = w * h * d - (4 - math.pi) * r * r * (w + h + d - 6 * r) - (8 - 4 * math.pi / 3) * r ** 3
    assert signed_volume(verts, faces) == pytest.approx(exact, rel=0.01)


def test_rounded_box_has_no_edge_sharper_than_its_radius():
    r = 0.04
    verts, faces = cs.rounded_box(1.3, 0.5, 0.16, r, 4)
    dih = edge_dihedrals(verts, faces)
    assert max(a for a, _ in dih) < 90.0 - 45.0, "a fillet step of 90 / segments, never a box edge"
    for angle, length in dih:
        if angle > 0.5:  # bent edge: the radius the step implies (chord = 2 R sin(angle / 2))
            assert length / (2.0 * math.sin(math.radians(angle) / 2.0)) >= r * (1.0 - RADIUS_TOL)
    # every vertex lies exactly r from the inner box (the box shrunk by r): the surface is the inner box swept by a ball of radius r
    inner = (1.3 / 2 - r, 0.5 / 2 - r, 0.16 / 2 - r)
    for v in verts:
        gap = math.sqrt(sum(max(abs(c) - b, 0.0) ** 2 for c, b in zip(v, inner)))
        assert gap == pytest.approx(r, abs=1e-9)


def test_rounded_box_rejects_a_radius_over_half_a_side():
    with pytest.raises(ValueError):
        cs.rounded_box(1.0, 0.5, 0.1, 0.06, 4)


def test_shear_keeps_the_body_closed_and_drops_the_back_bottom():
    verts, faces = cs.rounded_box(1.0, 0.5, 0.2, 0.03, 3)
    out = cs.drop_underside(verts, [[0.0, 0.0], [0.2, 0.05]], h=0.5, d=0.2)
    assert_closed(out, faces)
    bottom = [(v, o) for v, o in zip(verts, out) if abs(v[1] + 0.25) < 1e-9]
    assert bottom
    for v, o in bottom:  # the bottom face falls linearly with the depth from the front face (z = +0.1)
        assert o[1] == pytest.approx(-0.25 - 0.05 * (0.1 - v[2]) / 0.2, abs=1e-9)
    assert max(v[1] for v in out) == pytest.approx(0.25, abs=1e-9), "the top stays"


def test_post_runs_from_base_to_top_with_the_joint_head_at_the_top():
    base, top = (2.84, -0.07, 0.55), (3.48, -0.6, 0.75)
    r, head_r, head_len = 0.017, 0.039, 0.04
    verts, faces = cs.post(base, top, r, head_r, head_len, 16, foot=(0.022, 0.10, 0.05, 0.30))
    assert_closed(verts, faces)
    assert signed_volume(verts, faces) > 0.0
    axis = _sub(top, base)
    L = _norm(axis)
    ax = tuple(x / L for x in axis)

    def along(v):
        return sum(a * b for a, b in zip(_sub(v, base), ax))

    def radial(v):
        s = along(v)
        return _norm(_sub(_sub(v, base), tuple(a * s for a in ax)))

    assert min(along(v) for v in verts) == pytest.approx(0.0, abs=1e-9)
    assert max(along(v) for v in verts) == pytest.approx(L, abs=1e-9)
    head = [radial(v) for v in verts if along(v) > L - head_len + 1e-6]
    assert max(head) == pytest.approx(head_r, rel=1e-6)
    assert {round(radial(v), 6) for v in verts} == {0.0, r, head_r, 0.022, 0.05}, "rod, head, sleeve, funnel foot and the cap centres only"
    assert max(radial(v) for v in verts if along(v) < 1e-6) == pytest.approx(0.05, rel=1e-6), "the funnel foot"


def test_post_head_sits_flush_on_a_tilted_ceiling():
    """The ceiling is the plane x = 3.48 and the post leans ~45 deg: the head's top lies in that plane, nothing pokes through it."""
    base, top = (2.84, -0.07, 0.55), (3.48, -0.6, 0.75)
    verts, faces = cs.post(base, top, 0.017, 0.039, 0.04, 20, foot=(0.022, 0.10, 0.05, 0.30), top_plane_normal=(1.0, 0.0, 0.0))
    assert_closed(verts, faces)
    assert signed_volume(verts, faces) > 0.0
    assert max(v[0] for v in verts) == pytest.approx(3.48, abs=1e-9)
    assert sum(1 for v in verts if abs(v[0] - 3.48) < 1e-9) == 21, "the head's top ring (20) and the cap centre lie on the ceiling"
