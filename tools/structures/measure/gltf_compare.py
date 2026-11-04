"""Decoded compare of two folders of glTF files: per mesh node, the POSITION, NORMAL, TEXCOORD_0 and index accessors.

    python tools/structures/measure/gltf_compare.py <folder A> <folder B>

Blender's .bin files are not bit-stable between two runs of the same code, so file hashes cannot prove that a generator change
left a model alone. This decodes the accessors (float32 / uint16 / uint32 via numpy-free struct) and prints, per file and mesh node,
whether each attribute is equal exactly ("same"), equal as a sorted multiset of rows ("reordered") or "DIFFERENT". Exit 1 when
POSITION, TEXCOORD_0 or the indices of any node are not exactly the same (normals are reported only).
"""
import json
import struct
import sys
from pathlib import Path

COMPONENT = {5120: ("b", 1), 5121: ("B", 1), 5122: ("h", 2), 5123: ("H", 2), 5125: ("I", 4), 5126: ("f", 4)}
WIDTH = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}
ATTRIBUTES = ("POSITION", "NORMAL", "TEXCOORD_0", "indices")
STRICT = ("POSITION", "TEXCOORD_0", "indices")


def accessor_rows(gltf: dict, buffers: list, index: int) -> list:
    acc = gltf["accessors"][index]
    view = gltf["bufferViews"][acc["bufferView"]]
    code, size = COMPONENT[acc["componentType"]]
    width = WIDTH[acc["type"]]
    stride = view.get("byteStride") or size * width
    base = view.get("byteOffset", 0) + acc.get("byteOffset", 0)
    data = buffers[view["buffer"]]
    return [struct.unpack_from("<%d%s" % (width, code), data, base + i * stride) for i in range(acc["count"])]


def meshes(path: Path) -> dict:
    gltf = json.loads(path.read_text(encoding="utf-8"))
    buffers = [(path.parent / b["uri"]).read_bytes() for b in gltf.get("buffers", [])]
    out = {}
    for node in gltf["nodes"]:
        if "mesh" not in node:
            continue
        for k, prim in enumerate(gltf["meshes"][node["mesh"]]["primitives"]):
            attrs = {a: accessor_rows(gltf, buffers, prim["attributes"][a]) for a in ATTRIBUTES[:3] if a in prim["attributes"]}
            if "indices" in prim:
                attrs["indices"] = accessor_rows(gltf, buffers, prim["indices"])
            out[f"{node.get('name')}#{k}"] = attrs
    return out


def compare(a_dir: Path, b_dir: Path) -> bool:
    ok = True
    for fa in sorted(a_dir.glob("*.gltf")):
        fb = b_dir / fa.name
        if not fb.is_file():
            print(f"{fa.name}: missing in B")
            ok = False
            continue
        ma, mb = meshes(fa), meshes(fb)
        if set(ma) != set(mb):
            print(f"{fa.name}: node sets differ {sorted(set(ma) ^ set(mb))}")
            ok = False
            continue
        for name in sorted(ma):
            verdicts = []
            for attr in ATTRIBUTES:
                x, y = ma[name].get(attr), mb[name].get(attr)
                if x is None and y is None:
                    continue
                state = "same" if x == y else "reordered" if x is not None and y is not None and sorted(x) == sorted(y) else "DIFFERENT"
                if state != "same" and attr in STRICT:
                    ok = False
                verdicts.append(f"{attr}={state}")
            print(f"{fa.name} {name}: " + " ".join(verdicts))
    return ok


if __name__ == "__main__":
    sys.exit(0 if compare(Path(sys.argv[1]), Path(sys.argv[2])) else 1)
