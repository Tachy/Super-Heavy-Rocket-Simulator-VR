"""The ISS's fixed structure: iss_station.py -- --spec <json> --out <dir> writes iss_station.gltf (stage frame of config/vehicles/iss.toml: x forward from the Zvezda aft end,
y starboard, z nadir). Our own drawing from the typed tables of config/vehicles/detail/iss.toml (lathes, boxes, plates, tubes, truss bays); no mesh data of the reference.
Nodes: `pressurised` (US), `pressurised_ru`, `beam`, `pma_1`, `truss` (S0 to P3 / S3), `payloads`, `wings_ru`, `arm`, `radiator_fixed` (the PV radiators stand on the outboard truss: iss_arrays.py).
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402,F401
import vehicle_common as vc  # noqa: E402
from mathutils import Vector  # noqa: E402

LATHE_SEGMENTS = {"x": 48, "y": 48, "z": 48}  # [E] sagitta 4 mm at r 2.2
AXES = {"x": 0, "y": 1, "z": 2}
PAINT_NODES = {"us": "pressurised", "ru": "pressurised_ru", "beam": "beam", "pma": "pma_1", "truss": "truss", "payload": "payloads", "wing_ru": "wings_ru", "arm": "arm"}


def lathe(mesh, axis, centre, profile, group, segments):
    """Surface of revolution along a station axis; closed at both ends. centre: the two other coordinates in axis order."""
    a = AXES[axis]
    ns = [i for i in range(3) if i != a]

    def point(s, r, phi):
        p = [0.0, 0.0, 0.0]
        p[a] = s
        p[ns[0]] = centre[0] + r * math.cos(phi)
        p[ns[1]] = centre[1] + r * math.sin(phi)
        return Vector(p)

    def radial(phi):
        p = [0.0, 0.0, 0.0]
        p[ns[0]] = math.cos(phi)
        p[ns[1]] = math.sin(phi)
        return Vector(p)

    along = [0.0, 0.0, 0.0]
    along[a] = 1.0
    along = Vector(along)
    table = [(profile[0][0], 0.0)] + [tuple(row) for row in profile] + [(profile[-1][0], 0.0)]
    for k in range(len(table) - 1):
        (s0, r0), (s1, r1) = table[k], table[k + 1]
        for i in range(segments):
            p0, p1 = 2.0 * math.pi * i / segments, 2.0 * math.pi * (i + 1) / segments
            pts = [point(s0, r0, p0), point(s0, r0, p1), point(s1, r1, p1), point(s1, r1, p0)]
            uv0, uv1 = p0 * 2.2, p1 * 2.2  # metres at the barrel radius: the panel map's period (manifest)
            uvs = [(uv0, s0), (uv1, s0), (uv1, s1), (uv0, s1)]
            if k == 0:
                out, g = -along, None
            elif k == len(table) - 2:
                out, g = along, None
            else:
                mid = 0.5 * (p0 + p1)
                out, g = radial(mid) + along * (r0 - r1) / max(abs(s1 - s0), 1e-3), (group, k)
            mesh.face(pts, uvs, out, g)


def beam(mesh, a, b, half):
    """A square bar between two points: 8 triangles of sides (no caps: they meet at the nodes)."""
    a, b = Vector(a), Vector(b)
    d = (b - a).normalized()
    ref = Vector((0.0, 0.0, 1.0)) if abs(d.z) < 0.9 else Vector((1.0, 0.0, 0.0))
    s = d.cross(ref).normalized() * half
    t = d.cross(s).normalized() * half
    for u, v in ((s, t), (t, -s), (-s, -t), (-t, s)):
        mesh.face([a + u + v, a + u - v, b + u - v, b + u + v], [(0, 0), (1, 0), (1, 1), (0, 1)], u)


def lattice(mesh, bay, lat):
    """A truss bay: four longerons along y, a ring and two face diagonals per bay. Silhouette only: fine structure stays away (user: simple away from the ports)."""
    y0, y1 = bay["y_m"]
    x0, x1 = bay["x_m"]
    z0, z1 = bay["z_m"]
    n = int(bay["count"])
    long_half, brace_half = 0.5 * lat["member_m"], 0.5 * lat["brace_m"]
    corners = [(x0, z0), (x1, z0), (x1, z1), (x0, z1)]
    for x, z in corners:
        beam(mesh, (x, y0, z), (x, y1, z), long_half)
    for k in range(n + 1):
        y = y0 + (y1 - y0) * k / n
        for (xa, za), (xb, zb) in zip(corners, corners[1:] + corners[:1]):
            beam(mesh, (xa, y, za), (xb, y, zb), brace_half)
    for k in range(n):
        ya, yb = y0 + (y1 - y0) * k / n, y0 + (y1 - y0) * (k + 1) / n
        for i, ((xa, za), (xb, zb)) in enumerate(zip(corners, corners[1:] + corners[:1])):
            if k % 2:
                beam(mesh, (xa, ya, za), (xb, yb, zb), brace_half)
            else:
                beam(mesh, (xb, ya, zb), (xa, yb, za), brace_half)


def plate(mesh, corners, thickness=0.03):
    """A thin double-sided panel from four corners."""
    c = [Vector(p) for p in corners]
    n = (c[1] - c[0]).cross(c[3] - c[0]).normalized()
    u = (c[1] - c[0]).length
    v = (c[3] - c[0]).length
    uvs = [(0.0, 0.0), (u, 0.0), (u, v), (0.0, v)]
    t = n * (0.5 * thickness)
    mesh.face([p + t for p in c], uvs, n)
    mesh.face([p - t for p in c], uvs, -n)


def main():
    args = vc.common.script_args()
    spec = vc.load_spec(args["spec"])
    d = spec["detail"]
    vc.common.clear_scene()
    root = vc.empty("iss_station")
    meshes = {name: vc.VMesh() for name in set(PAINT_NODES.values())}
    for row in d["lathe"]:
        lathe(meshes[PAINT_NODES[row["paint"]]], row["axis"], row["centre_m"], row["profile_m"], row["id"], LATHE_SEGMENTS[row["axis"]])
    for row in d["box"]:
        lo, hi = Vector(row["min_m"]), Vector(row["max_m"])
        vc.box(meshes[PAINT_NODES[row["paint"]]], 0.5 * (lo + hi), (1, 0, 0), (0, 1, 0), (0, 0, 1), tuple(0.5 * (hi - lo)))
    for row in d["plate"]:
        if row["paint"] in PAINT_NODES:
            plate(meshes[PAINT_NODES[row["paint"]]], row["corners_m"])
    for row in d["tube"]:
        if row["id"].startswith("PVR_root"):
            continue  # on the outboard truss: iss_arrays.py
        vc.tube(meshes[PAINT_NODES[row["paint"]]], row["a_m"], row["b_m"], row["radius_m"], 12, row["id"])
    for bay in d["bay"]:
        if bay["outboard"] == 0:
            lattice(meshes["truss"], bay, d["truss_lattice"])
    tris = sum(m.to_object(name, root)[1] for name, m in meshes.items())
    vc.export_selected(Path(args["out"]) / "iss_station.gltf", [root])
    vc.report_triangles("iss_station", tris)


if __name__ == "__main__":
    main()
