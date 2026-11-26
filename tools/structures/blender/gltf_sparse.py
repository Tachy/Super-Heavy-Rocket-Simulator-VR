"""Morph targets as sparse accessors (glTF 2.0 accessor.sparse): a corrective shape only moves the vertices of its joint zone, so its
dense POSITION / NORMAL arrays are mostly zeros. Pure Python + numpy (runs in Blender after the export and in the tests).

`sparsify_targets(path)` rewrites a GLTF_SEPARATE file pair in place: every morph-target accessor becomes a sparse accessor without a
bufferView (an all-zero base) holding only its non-zero rows (indices uint32 + values), and the .bin is rebuilt from the views still in
use (4-byte aligned). min / max and every other accessor are kept."""
import json
from pathlib import Path

import numpy as np

COMPONENTS = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}
FLOAT = 5126
UINT32 = 5125


def _dense(doc, data, acc):
    view = doc["bufferViews"][acc["bufferView"]]
    comps = COMPONENTS[acc["type"]]
    stride = view.get("byteStride", 4 * comps)
    off = view.get("byteOffset", 0) + acc.get("byteOffset", 0)
    raw = np.frombuffer(data, dtype=np.uint8, count=stride * (acc["count"] - 1) + 4 * comps, offset=off)
    rows = np.lib.stride_tricks.as_strided(raw, shape=(acc["count"], 4 * comps), strides=(stride, 1))
    return np.ascontiguousarray(rows).view("<f4").reshape(acc["count"], comps)


def sparsify_targets(path):
    """Returns (bytes before, bytes after) of the .bin."""
    path = Path(path)
    doc = json.loads(path.read_text(encoding="utf-8"))
    assert len(doc["buffers"]) == 1, "one buffer expected"
    bin_path = path.parent / doc["buffers"][0]["uri"]
    data = bin_path.read_bytes()
    target_accessors = sorted({a for m in doc["meshes"] for p in m["primitives"] for t in p.get("targets", []) for a in t.values()})
    payload = {}  # accessor -> (indices uint32, values float32)
    for i in target_accessors:
        acc = doc["accessors"][i]
        if "bufferView" not in acc or acc["componentType"] != FLOAT:
            continue
        vals = _dense(doc, data, acc)
        rows = np.nonzero(np.any(vals != 0.0, axis=1))[0].astype("<u4")
        payload[i] = (rows, vals[rows].astype("<f4"))
    used = {a["bufferView"] for k, a in enumerate(doc["accessors"]) if "bufferView" in a and k not in payload}
    used |= {img["bufferView"] for img in doc.get("images", []) if "bufferView" in img}
    out = bytearray()
    views, remap = [], {}
    for k, v in enumerate(doc["bufferViews"]):
        if k not in used:
            continue
        while len(out) % 4:
            out.append(0)
        start = v.get("byteOffset", 0)
        nv = dict(v)
        nv["byteOffset"] = len(out)
        out += data[start:start + v["byteLength"]]
        remap[k] = len(views)
        views.append(nv)

    def add_view(blob):
        while len(out) % 4:
            out.append(0)
        views.append({"buffer": 0, "byteOffset": len(out), "byteLength": len(blob)})
        out.extend(blob)
        return len(views) - 1

    for k, a in enumerate(doc["accessors"]):
        if k in payload:
            rows, vals = payload[k]
            a.pop("bufferView", None)
            a.pop("byteOffset", None)
            if len(rows):
                a["sparse"] = {"count": int(len(rows)), "indices": {"bufferView": add_view(rows.tobytes()), "componentType": UINT32},
                               "values": {"bufferView": add_view(vals.tobytes())}}
            else:
                a.pop("sparse", None)
        elif "bufferView" in a:
            a["bufferView"] = remap[a["bufferView"]]
    for img in doc.get("images", []):
        if "bufferView" in img:
            img["bufferView"] = remap[img["bufferView"]]
    doc["bufferViews"] = views
    before = len(data)
    doc["buffers"][0]["byteLength"] = len(out)
    bin_path.write_bytes(bytes(out))
    path.write_text(json.dumps(doc, indent=1), encoding="utf-8")
    return before, len(out)


def target_rows(doc, data, acc):
    """The dense (count, comps) array of an accessor that may be sparse (all-zero base without a bufferView)."""
    comps = COMPONENTS[acc["type"]]
    vals = _dense(doc, data, acc) if "bufferView" in acc else np.zeros((acc["count"], comps), dtype="<f4")
    sp = acc.get("sparse")
    if sp:
        iv = doc["bufferViews"][sp["indices"]["bufferView"]]
        vv = doc["bufferViews"][sp["values"]["bufferView"]]
        idx = np.frombuffer(data, dtype="<u4", count=sp["count"], offset=iv.get("byteOffset", 0) + sp["indices"].get("byteOffset", 0))
        val = np.frombuffer(data, dtype="<f4", count=sp["count"] * comps, offset=vv.get("byteOffset", 0) + sp["values"].get("byteOffset", 0))
        vals = np.array(vals)
        vals[idx] = val.reshape(-1, comps)
    return vals
