"""Measure the NASA ISS model D (IGOAL, Draco-compressed glb): per-node vertex extents in station metres.

Reference only: the file is untrusted and git-ignored; this reads the glTF JSON and decodes the Draco buffers with
DracoPy (a decoder, nothing from the file is executed), no mesh data is written to the repo.

    python -I tools/structures/measure/iss_measure.py <ISS_D_IGOAL.glb> table  [out.txt]
    python -I tools/structures/measure/iss_measure.py <ISS_D_IGOAL.glb> report [out.txt]

DracoPy is looked up on the path in the environment variable STARSIM_PYLIBS (python -I ignores PYTHONPATH).

Frames. The root node of the file carries a uniform negative scale (a point reflection); undoing the root, the file's
local axes are: +x forward (towards Harmony / the PMA-2 end), +y nadir, +z port (checked on the modules: Kibo on the
port side of Harmony, Columbus on the starboard side, PMA-3 above Harmony). The station frame of config/vehicles/iss.toml
is x forward, y starboard, z nadir, right-handed; from the file's local (x, y, z) it is (x, -z, y) (a proper rotation).
Model units are not metres; `metres_per_unit` scales from the known truss span.
"""
import json
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import glb_measure as gm  # noqa: E402

_DRACO = "KHR_draco_mesh_compression"
# Overall length of the integrated truss structure (ITS), NASA ISS fact sheet / Wikipedia "Integrated Truss Structure":
# 108.5 m (356 ft); the commonly cited span 109 m [S: iss.toml wiki_iss] is this number rounded.
TRUSS_SPAN_M = 109.0
TRUSS_SPAN_TOLERANCE_M = 1.0


def _draco_points(doc, binary, prim, faces=False):
    """Vertices of a primitive (plus triangles when `faces`, Draco only): plain accessor or Draco buffer."""
    ext = prim.get("extensions", {}).get(_DRACO)
    if ext is None:
        return gm._positions(doc, binary, prim["attributes"]["POSITION"])
    pylibs = os.environ.get("STARSIM_PYLIBS")
    if pylibs and pylibs not in sys.path:
        sys.path.append(pylibs)
    import DracoPy  # noqa: E402  (optional decoder, only for Draco files)
    view = doc["bufferViews"][ext["bufferView"]]
    start = view.get("byteOffset", 0)
    mesh = DracoPy.decode(bytes(binary[start: start + view["byteLength"]]))
    pts = np.asarray(mesh.points, dtype=float)
    return (pts, np.asarray(mesh.faces, dtype=np.int64)) if faces else pts


def _station_rotation():
    """Root-local (x fwd, y nadir, z port) -> station frame (x fwd, y starboard, z nadir)."""
    return np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])


class Station:
    """All mesh nodes of the model with vertices in the station frame, model units (not yet metres)."""

    def __init__(self, path, keep_faces=False):
        self.doc, binary = gm._load(path)
        self.faces = {}  # node index -> (m, 3) indices into vertices[node] (only with keep_faces, Draco primitives)
        world = gm._world_matrices(self.doc)
        nodes = self.doc["nodes"]
        parents = {c: i for i, n in enumerate(nodes) for c in n.get("children", [])}
        scene = self.doc["scenes"][self.doc.get("scene", 0)]
        roots = scene["nodes"]
        self.root_matrix = world[roots[0]]
        inv = np.linalg.inv(self.root_matrix)
        rot = _station_rotation()
        self.names = [n.get("name", f"node{i}") for i, n in enumerate(nodes)]
        self.parents = parents
        self.children = {i: list(n.get("children", [])) for i, n in enumerate(nodes)}
        self.vertices = {}  # node index -> (n, 3) station frame, model units
        for i, n in enumerate(nodes):
            if "mesh" not in n:
                continue
            chunks, fchunks, offset = [], [], 0
            for prim in self.doc["meshes"][n["mesh"]]["primitives"]:
                if "POSITION" not in prim["attributes"]:
                    continue
                if keep_faces and _DRACO in prim.get("extensions", {}):
                    p, f = _draco_points(self.doc, binary, prim, faces=True)
                    fchunks.append(f + offset)
                else:
                    p = _draco_points(self.doc, binary, prim)
                offset += len(p)
                w = (world[i][:3, :3] @ p.T).T + world[i][:3, 3]
                loc = (inv[:3, :3] @ w.T).T + inv[:3, 3]
                chunks.append(loc @ rot.T)
            if chunks:
                self.vertices[i] = np.vstack(chunks)
                if fchunks:
                    self.faces[i] = np.vstack(fchunks)
        self.node_origin = {i: (rot @ (inv @ world[i])[:3, 3]) for i in range(len(nodes))}
        self.metres_per_unit = None

    def index(self, name):
        hits = [i for i, n in enumerate(self.names) if n == name]
        if not hits:
            raise KeyError(name)
        return hits[0]

    def subtree(self, i):
        out, stack = [], [i]
        while stack:
            k = stack.pop()
            out.append(k)
            stack.extend(self.children[k])
        return out

    def path(self, i):
        p = []
        while i is not None:
            p.append(self.names[i])
            i = self.parents.get(i)
        return "/".join(reversed(p))

    def points(self, i, include_children=True, skip=()):
        """Vertices of node i (and its descendants) in model units; `skip` = name substrings to leave out."""
        idx = self.subtree(i) if include_children else [i]
        chunks = [self.vertices[k] for k in idx
                  if k in self.vertices and not any(s in self.names[k] for s in skip)]
        return np.vstack(chunks) if chunks else np.zeros((0, 3))

    def box(self, i, include_children=True, skip=()):
        p = self.points(i, include_children, skip)
        if len(p) == 0:
            return None
        return p.min(axis=0), p.max(axis=0)

    # ---- scale -------------------------------------------------------------------------------------------------
    def extent(self, i, axis, include_children=True):
        p = self.points(i, include_children)
        return float(p[:, axis].max() - p[:, axis].min())

    def truss_extent_units(self):
        """End-to-end extent along y of the truss segments and joints (arrays excluded): ~3450 units."""
        lo, hi = np.inf, -np.inf
        for i, name in enumerate(self.names):
            if name.startswith("Truss_") or name in ("PORT_ALPHA_ROT", "STBD_ALPHA_ROT"):
                for k in self.subtree(i):
                    if k in self.vertices and "IROSA" not in self.names[k] and "Array" not in self.names[k]:
                        lo = min(lo, self.vertices[k][:, 1].min())
                        hi = max(hi, self.vertices[k][:, 1].max())
        return hi - lo

    def array_span_units(self):
        """Span across the truss direction with the outermost arrays (P6 / S6) turned broadside.

        The commonly quoted 109 m is this span: outermost array centre to centre plus one blanket width. The arrays
        stand in the file with their width along z; the width is the z extent of the array mesh."""
        reach = []
        for i, name in enumerate(self.names):
            if name.endswith("_Array") and (name.startswith("P6") or name.startswith("S6")) and i in self.vertices:
                p = self.vertices[i]
                reach.append(abs((p[:, 1].max() + p[:, 1].min()) / 2) + (p[:, 2].max() - p[:, 2].min()) / 2)
        if not reach:
            raise KeyError("no P6 / S6 array meshes")
        return 2.0 * float(np.mean(reach))

    def set_scale_from_span(self, span_m=TRUSS_SPAN_M):
        """Metres per model unit from the 109 m span (see array_span_units); the file's unit turns out to be 1 inch."""
        self.metres_per_unit = scale_from_extent(self.array_span_units(), span_m)
        return self.metres_per_unit

    def set_scale_from_truss(self, span_m=TRUSS_SPAN_M):
        """The brief's first method (truss end to end = 109 m): kept to show it disagrees by 11 % (see the report)."""
        self.metres_per_unit = span_m / self.truss_extent_units()
        return self.metres_per_unit

    def metres(self, units):
        return np.asarray(units) * self.metres_per_unit

    # ---- docking adapters ---------------------------------------------------------------------------------------
    def ida_port(self, name, axis):
        """Ring plane, ring-axis centre, petal data of an IDA node (outward axis = a station axis, +-)."""
        a = np.asarray(axis, dtype=float)
        ns = [i for i in range(3) if abs(a[i]) < 0.5]
        root = self.index(name)
        pm = self.vertices[root] * self.metres_per_unit
        t = pm @ a
        base = pm[t < t.min() + 0.06 / 1.0]
        A = np.c_[2 * base[:, ns[0]], 2 * base[:, ns[1]], np.ones(len(base))]
        b = base[:, ns[0]] ** 2 + base[:, ns[1]] ** 2
        sol = np.linalg.lstsq(A, b, rcond=None)[0]
        centre = sol[:2]
        radius = float(np.sqrt(sol[2] + sol[0] ** 2 + sol[1] ** 2))
        pet = [k for k in self.subtree(root) if "Petals" in self.names[k]][0]
        pp = self.vertices[pet] * self.metres_per_unit
        tp = pp @ a
        # in-plane angles of the petals about the ring axis (atan2 over the two normal axes, in axis order)
        ang = np.degrees(np.arctan2(pp[:, ns[1]] - centre[1], pp[:, ns[0]] - centre[0]))
        sel = (tp < tp.min() + 0.15)
        h, _ = np.histogram(ang[sel], bins=36, range=(-180, 180))
        face = float(t.max())
        sign = 1.0 if a[np.argmax(np.abs(a))] > 0 else -1.0
        return {"axis": a.tolist(), "normal_axes": ns, "axis_centre_m": centre.tolist(), "base_radius_m": radius,
                "mesh_axial_m": [float(t.min()), float(t.max())], "petal_axial_m": [float(tp.min()), float(tp.max())],
                "petal_radius_max_m": float(np.hypot(pp[:, ns[0]] - centre[0], pp[:, ns[1]] - centre[1]).max()),
                "petal_angle_hist_10deg": h.tolist(),
                # plane along the (signed) station axis: the IDA mesh's outer face
                "plane_m": sign * face}


def scale_from_extent(extent_units, span_m=TRUSS_SPAN_M):
    """Metres per model unit from a measured extent of the known span."""
    return span_m / extent_units


def principal_axis(points):
    """Axis (unit vector, sign fixed to the largest component positive), centre and extents along (axis, 2 normals)."""
    c = points.mean(axis=0)
    u, s, vt = np.linalg.svd(points - c, full_matrices=False)
    a = vt[0]
    if a[np.argmax(np.abs(a))] < 0:
        a = -a
    proj = (points - c) @ vt.T
    return a, c, proj.max(axis=0) - proj.min(axis=0), vt


def fit_cylinder(points, axis_hint=None):
    """Axis, length, mean/max radius and centre of a (module-like) vertex cloud by PCA about its long direction."""
    if axis_hint is None:
        a, c, _, _ = principal_axis(points)
    else:
        a = np.asarray(axis_hint, dtype=float)
        a = a / np.linalg.norm(a)
        c = points.mean(axis=0)
    t = (points - c) @ a
    radial = np.linalg.norm((points - c) - np.outer(t, a), axis=1)
    return {"axis": a, "centre": c, "t_min": float(t.min()), "t_max": float(t.max()),
            "length": float(t.max() - t.min()), "r_max": float(radial.max()), "r_mean": float(radial.mean()),
            "r_p95": float(np.percentile(radial, 95))}


# ---- report ---------------------------------------------------------------------------------------------------
# Pressurised elements: node name -> axis of the barrel (station axis index, 0 x, 1 y, 2 z). Hints only; lengths and
# radii are measured from the mesh of the node itself (children such as handrails / connectors are separate nodes).
MODULES = {
    "Zarya_FGB": 0, "Zvezda_SM": 0, "Node1": 0, "USLab": 0, "Node2": 0, "PMA1": 0, "PMA2": 0, "PMA3": 2,
    "Node3": 1, "Cupola": 2, "Columbus": 1, "JEM_PM": 1, "JEM_PS": 1, "Airlock": 1, "Bishop_Airlock": 1, "BEAM": 1,
    "MLM": 2, "MRM1": 2, "MRM2": 2, "Russian_RSNode_DockingModule": 2, "PMM": 1,
}
SEGMENTS = ["Truss_S0", "Truss_P1", "Truss_S1", "Truss_P3", "Truss_S3", "Truss_P4", "Truss_S4", "Truss_P5", "Truss_S5",
            "Truss_P6", "Truss_S6", "Z1", "PORT_ALPHA_ROT", "STBD_ALPHA_ROT"]


def _barrel_radius(p, axis, centre):
    """Median over axial slices (0.5 m) of the largest distance from the axis: robust against single protrusions."""
    ns = [i for i in range(3) if i != axis]
    r = np.hypot(p[:, ns[0]] - centre[0], p[:, ns[1]] - centre[1])
    t = p[:, axis]
    out = []
    for a in np.arange(t.min(), t.max(), 0.5):
        m = (t >= a) & (t < a + 0.5)
        if m.sum() >= 30:
            out.append(r[m].max())
    return float(np.median(out)) if out else float(r.max())


def report(path, aft_end_m=None):
    st = Station(path)
    st.set_scale_from_span()
    s = st.metres_per_unit
    out = []
    w = out.append
    w(f"scale {s:.6f} m/unit ({s / 0.0254:.4f} inch/unit); span with arrays broadside "
      f"{st.metres(st.array_span_units()):.2f} m; truss end to end (arrays excluded) {st.metres(st.truss_extent_units()):.2f} m; "
      f"brief method (truss = 109 m) would give {TRUSS_SPAN_M / st.truss_extent_units():.6f}")
    zv = st.metres(st.vertices[st.index("Zvezda_SM")][:, 0].min())
    aft = zv if aft_end_m is None else aft_end_m
    w(f"station x origin: truss S0 centre; Russian segment aft end (Zvezda mesh) at x {zv:.2f} m; "
      f"config stations = x - ({aft:.2f}); module axis at z = {st.metres(st.node_origin[st.index('Node2')][2]):.2f} m below the truss axis")
    z0 = float(st.metres(st.node_origin[st.index("Node2")][2]))
    w("\n[pressurised elements, mesh of the node only; x,y relative to the truss S0 centre, z relative to the module axis]")
    w("name | axis | length | r_barrel | x min..max | y min..max | z min..max | station x min..max (config)")
    for name, ax in MODULES.items():
        i = st.index(name)
        p = st.vertices[i] * s
        p = p - np.array([0, 0, z0])
        lo, hi = p.min(0), p.max(0)
        ctr = (lo + hi) / 2
        rb = _barrel_radius(p, ax, [ctr[k] for k in range(3) if k != ax])
        w(f"{name} | {'xyz'[ax]} | {hi[ax] - lo[ax]:.2f} | {rb:.2f} | {lo[0]:.2f}..{hi[0]:.2f} | {lo[1]:.2f}..{hi[1]:.2f} | "
          f"{lo[2]:.2f}..{hi[2]:.2f} | {lo[0] - aft:.2f}..{hi[0] - aft:.2f}")
    w("\n[truss segments / joints, node meshes only, truss-axis frame (z from the truss axis)]")
    for name in SEGMENTS:
        i = st.index(name)
        p = st.vertices.get(i)
        if p is None:
            p = st.points(i, include_children=False)
        p = p * s if len(p) else p
        if not len(p):
            continue
        lo, hi = p.min(0), p.max(0)
        w(f"{name} | length(y) {hi[1] - lo[1]:.2f} | x {lo[0]:.2f}..{hi[0]:.2f} | y {lo[1]:.2f}..{hi[1]:.2f} | z {lo[2]:.2f}..{hi[2]:.2f}")
    w("\n[radiators and arrays]")
    for i, name in enumerate(st.names):
        if (("Radiator" in name and "Details" not in name) or name.endswith("_Array") or "Array_" in name
                or name.startswith("IROSA_Deployed_P") and "." not in name and "Details" not in name) and i in st.vertices:
            p = st.vertices[i] * s
            lo, hi = p.min(0), p.max(0)
            w(f"{name} | ext {hi[0]-lo[0]:.2f} x {hi[1]-lo[1]:.2f} x {hi[2]-lo[2]:.2f} | ctr {(lo[0]+hi[0])/2:.2f} {(lo[1]+hi[1])/2:.2f} {(lo[2]+hi[2])/2:.2f}")
    w("\n[joint node origins (rotation joints), truss-axis frame]")
    for i, name in enumerate(st.names):
        if "_ROT" in name and not name.startswith(("JEM_", "SPDM", "SSRMS", "SFA")):
            o = st.metres(st.node_origin[i])
            w(f"{name} | origin {o[0]:.2f} {o[1]:.2f} {o[2]:.2f}")
    w("\n[docking adapters]")
    for nm, ax in (("IDA2", (1.0, 0.0, 0.0)), ("IDA3", (0.0, 0.0, -1.0))):
        d = st.ida_port(nm, ax)
        w(f"{nm}: {json.dumps(d)}")
    return "\n".join(out)


def _fmt(v):
    return "[" + ", ".join(f"{x:.2f}" for x in v) + "]"


def detail_toml(path):
    """The measured table as TOML text in the station frame of config/vehicles/iss.toml (x from the Russian segment's
    aft end, y starboard from the module axis, z nadir from the module axis); every line tagged [D]."""
    st = Station(path)
    st.set_scale_from_span()
    s = st.metres_per_unit
    aft = float(st.metres(st.vertices[st.index("Zvezda_SM")][:, 0].min()))
    z0 = float(st.metres(st.node_origin[st.index("Node2")][2]))
    off = np.array([-aft, 0.0, -z0])

    def box(i, include_children=False):
        p = st.points(i, include_children) * s + off
        return p.min(0), p.max(0)

    tag = "# [D] NASA ISS model D, vertex extents by script (iss_measure.py)"
    L = []
    w = L.append
    w("# Measured layout of the ISS (NASA 3D Resources model D, IGOAL; vertices read by tools/structures/measure/iss_measure.py, nothing copied).")
    w("# Station frame of config/vehicles/iss.toml: x forward from the aft end of the Russian segment (Zvezda aft end), y starboard and z nadir from the module axis.")
    w("# Accuracy: scale +-0.5 % (cross-checks in docs/subsystems/iss.md), plane and axis fits +-0.1 m; extents include fittings of the node's own mesh.")
    w("# Gaps against config/vehicles/iss.toml: see docs/subsystems/iss.md (Model D measurement).")
    w("")
    w("[scale]")
    w(f"metres_per_unit = {s:.6f}                      {tag}: 109 m span with the P6/S6 arrays turned broadside; the file's unit is 1 inch")
    w(f"truss_end_to_end_m = {st.metres(st.truss_extent_units()):.2f}                   {tag}: truss without arrays")
    w(f"array_span_m = {st.metres(st.array_span_units()):.2f}                        {tag}: 109 m [S: NASA / Wikipedia 'width']")
    w(f"aft_end_model_x_m = {aft:.2f}                      {tag}: model x of the Russian segment aft end (config x = model x - this)")
    w(f"module_axis_below_truss_m = {z0:.2f}               {tag}")
    w("")
    for name, ax in MODULES.items():
        lo, hi = box(st.index(name))
        ctr = (lo + hi) / 2
        p = st.vertices[st.index(name)] * s + off
        rb = _barrel_radius(p, ax, [ctr[k] for k in range(3) if k != ax])
        w("[[module]]")
        w(f'id = "{name}"                              {tag}')
        w(f'axis = "{"xyz"[ax]}"')
        w(f"length_m = {hi[ax] - lo[ax]:.2f}")
        w(f"barrel_radius_m = {rb:.2f}")
        w(f"min_m = {_fmt(lo)}")
        w(f"max_m = {_fmt(hi)}")
        w("")
    for name in SEGMENTS:
        i = st.index(name)
        if i not in st.vertices:
            continue
        lo, hi = box(i)
        w("[[truss]]")
        w(f'id = "{name}"                              {tag}')
        w(f"min_m = {_fmt(lo)}")
        w(f"max_m = {_fmt(hi)}")
        w("")
    for i, name in enumerate(st.names):
        kind = None
        if name.endswith("_Radiator") and "Details" not in name and i in st.vertices:
            kind = "radiator"
        elif "Array" in name and "Details" not in name and not name.startswith("IROSA") and i in st.vertices:
            kind = "array"
        elif name.startswith("IROSA_Deployed_") and "." not in name and "Details" not in name and i in st.vertices:
            kind = "iROSA"
        if kind:
            lo, hi = box(i)
            w(f"[[{kind}]]" if kind != "iROSA" else "[[irosa]]")
            w(f'id = "{name}"                              {tag}')
            w(f"min_m = {_fmt(lo)}")
            w(f"max_m = {_fmt(hi)}")
            w("")
    for i, name in enumerate(st.names):
        if "_ROT" in name and not name.startswith(("JEM_", "SPDM", "SSRMS", "SFA")):
            o = st.metres(st.node_origin[i]) + off
            w("[[joint]]")
            w(f'id = "{name}"                              {tag}: node origin = rotation axis point')
            w(f"origin_m = {_fmt(o)}")
            w("")
    for nm, ax, up in (("IDA2", (1.0, 0.0, 0.0), (0.0, 0.0, -1.0)), ("IDA3", (0.0, 0.0, -1.0), (1.0, 0.0, 0.0))):
        d = st.ida_port(nm, ax)
        ns = d["normal_axes"]
        c = np.zeros(3)
        c[ns[0]], c[ns[1]] = d["axis_centre_m"]
        a = np.asarray(ax)
        sign = 1.0 if a[np.argmax(np.abs(a))] > 0 else -1.0
        plane_axis = np.argmax(np.abs(a))
        c[plane_axis] = d["plane_m"]
        c = c + off
        w("[[port]]")
        w(f'id = "{nm.lower()}"                              {tag}')
        w(f"axis = {_fmt(ax)}")
        w(f"mating_face_m = {_fmt(c)}")
        harmony = st.metres(st.node_origin[st.index("Node2")])
        offset = np.linalg.norm(np.asarray(d["axis_centre_m"]) - harmony[ns])
        w(f"ring_axis_offset_from_harmony_axis_m = {offset:.2f}")
        w(f"ring_radius_m = {d['base_radius_m']:.3f}")
        w(f"petal_tip_radius_m = {d['petal_radius_max_m']:.3f}")
        w(f"petal_length_m = {abs(d['petal_axial_m'][1] - d['petal_axial_m'][0]):.3f}")
        w("")
    # every value line carries a tag (config/README.md): lines without a comment get the short one
    L = [ln + "  # [D]" if "=" in ln and "#" not in ln and not ln.startswith(("#", "[")) else ln for ln in L]
    return "\n".join(L)


if __name__ == "__main__":
    mode = sys.argv[2] if len(sys.argv) > 2 else "report"
    text = detail_toml(sys.argv[1]) if mode == "detail" else report(sys.argv[1])
    if len(sys.argv) > 3:
        Path(sys.argv[3]).write_text(text, encoding="utf-8")
    else:
        print(text)
