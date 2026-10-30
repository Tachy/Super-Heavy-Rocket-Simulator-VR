"""Chopstick arm: an open truss of round tubes (references: padb_chopsticks_around_booster_top_2026-03,
padb_chopstick_arm_closeup_worker_2025-12).

Run: blender -b --factory-startup -P gen_chopstick.py -- --config <toml> --out <dir> [--preview <png>]
Frame (Blender axes kept in the glTF): root node `chopstick` at the hinge (origin); the arm runs along +X from the hinge
to the tip at X = length_m; Y is across (+-width_m / 2); the top of the catch rails is the plane Z = 0, the truss hangs
down to Z = -depth_m. Nodes: `truss` (chords, posts, W diagonals, ties, deck, tip strut) and `rail` (catch rails).
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bpy  # noqa: E402
import common  # noqa: E402

TIP_DEPTH_FRAC = 0.30  # [E] truss depth at the tip / at the hinge: the lower chord rises toward the tip (closeups)
LOWER_CHORD_END_FRAC = 0.85  # [E] the lower chord ends here (x / length); a strut then climbs to the tip
RAIL_START_FRAC = 0.40  # [E] catch rails cover the outer 60 % of the arm (they meet the booster there)
POST_START_M = 0.35  # [E] first post this far from the hinge end plate
DECK_THICKNESS_M = 0.08  # [E] walkway plate on the top chords
TIP_INSET_M = 0.35  # [E] strut / tie ends stay inside the tip so cut ends do not overshoot X = length
HINGE_INSET_M = 0.12  # [E] same at the hinge end for the tilted lower chord


def build(detail):
    c = detail["chopstick"]
    prof = detail["profiles"]
    chord, brace, rail = prof[c["chord_profile"]], prof[c["brace_profile"]], prof[c["rail_profile"]]
    length, width, depth, pitch = c["length_m"], c["width_m"], c["depth_m"], c["truss_pitch_m"]
    r_rail, r_ch = rail["size_m"] / 2.0, chord["size_m"] / 2.0
    yc = width / 2.0 - r_ch                      # chord centre lines: the outer surfaces sit at +-width / 2
    z_top = -2.0 * r_rail - r_ch + 0.05          # top chord axis, rails sit on it (0.05 m overlap = weld)
    x_end = length * LOWER_CHORD_END_FRAC
    z_bot0 = -depth + r_ch                       # lower chord axis at the hinge, its underside is Z = -depth
    for _ in range(3):                           # the rising chord's square cut end dips r*cos(slope) below its axis
        z_bote = z_top + (z_bot0 - z_top) * TIP_DEPTH_FRAC
        slope = math.atan2(z_bote - z_bot0, x_end - HINGE_INSET_M)
        z_bot0 = -depth + r_ch * math.cos(slope)

    def z_bot(x):
        return z_bot0 + (z_bote - z_bot0) * x / x_end

    truss = common.MeshBuilder()
    rails = common.MeshBuilder()

    def tube(mb, prof_, p0, p1):
        mb.add_member("tube", prof_["size_m"], prof_["wall_m"], p0, p1)

    n = max(1, round((x_end - POST_START_M) / pitch))
    xs = [POST_START_M + i * (x_end - POST_START_M) / n for i in range(n + 1)]
    for side in (-1.0, 1.0):
        y = side * yc
        tube(truss, chord, (0.0, y, z_top), (length, y, z_top))                       # top chord
        tube(truss, chord, (HINGE_INSET_M, y, z_bot(HINGE_INSET_M)), (x_end, y, z_bote))  # lower chord
        tube(truss, chord, (x_end, y, z_bote), (length - TIP_INSET_M, y, z_top))       # strut up to the tip
        tube(rails, rail, (length * RAIL_START_FRAC, y, -r_rail), (length, y, -r_rail))  # catch rail
        for x in xs:                                                                   # posts
            tube(truss, brace, (x, y, z_bot(x)), (x, y, z_top))
        for i in range(n):                                                             # W diagonals
            a, b = xs[i], xs[i + 1]
            if i % 2 == 0:
                tube(truss, brace, (a, y, z_top), (b, y, z_bot(b)))
            else:
                tube(truss, brace, (a, y, z_bot(a)), (b, y, z_top))
    for x in xs + [length - TIP_INSET_M]:                                              # ties across the arm
        tube(truss, brace, (x, -yc, z_top), (x, yc, z_top))
        if x <= x_end:
            tube(truss, brace, (x, -yc, z_bot(x)), (x, yc, z_bot(x)))
    inner = 2.0 * (yc - r_ch) + 0.1
    truss.add_member("rect", inner, DECK_THICKNESS_M, (0.0, 0.0, z_top + r_ch - DECK_THICKNESS_M / 2.0),
                     (length, 0.0, z_top + r_ch - DECK_THICKNESS_M / 2.0), size_y=DECK_THICKNESS_M)
    return truss, rails


def main() -> None:
    args = common.script_args()
    detail = common.load_detail(args["config"])
    out = Path(args["out"])
    out.mkdir(parents=True, exist_ok=True)
    common.clear_scene()
    root = bpy.data.objects.new("chopstick", None)
    bpy.context.scene.collection.objects.link(root)
    truss, rails = build(detail)
    _, t_truss = truss.to_object("truss", root)
    _, t_rail = rails.to_object("rail", root)
    print(f"TRIANGLES chopstick {t_truss + t_rail}")
    common.export_gltf(out / "chopstick.gltf")
    if "preview" in args:
        c = detail["chopstick"]
        Path(args["preview"]).parent.mkdir(parents=True, exist_ok=True)
        common.render_preview(str(Path(args["preview"]).resolve()), (c["length_m"] * 0.5, 0.0, -c["depth_m"] * 0.45),
                              c["length_m"] * 2.1, -62.0, 14.0)


main()
