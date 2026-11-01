"""Measure the built forward flap model (flap_fwd.gltf) and place it with the manifest at d = 90.

  gltf_flap_measure.py [model_dir]      (default data/models/starship_block3)

Prints the hinge length, the largest span, the panel polygon area (outline = the +Y face of the panel) and, with the first
forward flap's manifest placement at d = 90 (panel in the meridian plane), station and radius of each outline corner.
Frame: model X along the hinge from its aft end, model Z = span; the hinge leans by the manifest lean towards the axis
going forward, so station = x_aft + u cos l + v sin l and r = r_aft - u sin l + v cos l.
"""
from __future__ import annotations

import json
import math
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_DIR = ROOT / "data" / "models" / "starship_block3"


def positions(gltf: dict, blob: bytes, node_name: str) -> list[tuple[float, float, float]]:
    node = next(n for n in gltf["nodes"] if n["name"] == node_name)
    acc = gltf["accessors"][gltf["meshes"][node["mesh"]]["primitives"][0]["attributes"]["POSITION"]]
    view = gltf["bufferViews"][acc["bufferView"]]
    start = view.get("byteOffset", 0) + acc.get("byteOffset", 0)
    return [struct.unpack_from("<3f", blob, start + 12 * i) for i in range(acc["count"])]


def outline(points: list[tuple[float, float, float]]) -> list[tuple[float, float]]:
    """Unique (u, v) corners of the +Y face, in order round their centroid."""
    top = {(round(p[0], 4), round(p[2], 4)) for p in points if p[1] > 0.0}
    cx, cz = sum(u for u, _ in top) / len(top), sum(v for _, v in top) / len(top)
    return sorted(top, key=lambda p: math.atan2(p[1] - cz, p[0] - cx))


def shoelace(pts: list[tuple[float, float]]) -> float:
    return 0.5 * abs(sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1])))


def main(model_dir: Path) -> None:
    gltf = json.loads((model_dir / "flap_fwd.gltf").read_text(encoding="utf-8"))
    blob = (model_dir / gltf["buffers"][0]["uri"]).read_bytes()
    hinge, panel = positions(gltf, blob, "hinge"), positions(gltf, blob, "panel")
    corners = outline(panel)
    manifest = json.loads((model_dir / "manifest.json").read_text(encoding="utf-8"))
    entry = next(f for f in manifest["flaps"]["ship"] if f["kind"] == "fwd")
    lean = math.radians(entry["drawn_hinge_lean_deg"])
    print(f"hinge length (extent along the hinge): {max(p[0] for p in hinge) - min(p[0] for p in hinge):.3f} m")
    print(f"largest span (panel Z): {max(p[2] for p in panel):.3f} m")
    print(f"panel polygon area (+Y face, {len(corners)} corners): {shoelace(corners):.3f} m^2")
    print(f"placement d = 90: aft end station {entry['drawn_hinge_station_aft_m']} m, radius "
          f"{entry['drawn_hinge_radius_aft_m']} m, lean {entry['drawn_hinge_lean_deg']} deg")
    for u, v in corners:
        station = entry["drawn_hinge_station_aft_m"] + u * math.cos(lean) + v * math.sin(lean)
        radius = entry["drawn_hinge_radius_aft_m"] - u * math.sin(lean) + v * math.cos(lean)
        print(f"  corner u {u:7.3f} v {v:6.3f} -> station {station:7.3f} m, radius {radius:6.3f} m")


if __name__ == "__main__":
    main(Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_DIR)
