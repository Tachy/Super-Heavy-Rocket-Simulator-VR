"""Physics envelope check of the drawn Mechazilla: a drawn surface the vehicle touches sits on the physics number.

    python tools/structures/check_envelope.py

Tolerances 0.10 m (lengths) / 0.05 m (surfaces the vehicle touches). The only allowed exceptions are the rows of the
"Physics deviations" table in docs/subsystems/launch-structures.md (ALLOWED); they are listed by name. Exit 1 on a failure.
Every row is a comparison of real numbers: it is listed only when the drawn value differs from the physics / the reference.
Mount deck: the detail file's `deck_height_m` against the physics `mount_height_m`; azimuth / distance have no drawn
number in the detail file (TowerView reads them from the physics geometry); tower_view_test checks them in the scene.
"""
from __future__ import annotations

import sys
import tomllib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent / "blender"))
import detail_file  # noqa: E402
import mechazilla_layout  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
DETAIL = ROOT / "config" / "structures" / "starbase_olp2.toml"
SITE_ID = "starbase_olp1"
LENGTH_TOL_M = 0.10
SURFACE_TOL_M = 0.05
VEHICLE = ROOT / "config" / "vehicles" / "starship_block3.toml"
HULL_RADIUS_M = 4.5  # [S] 9 m booster diameter
MEASURED_LEG_CENTRES_M = 11.9  # [E] measured leg centre spacing, launch-structures-measurements.md (+-1 m)
LEG_CENTRES_TOL_M = 1.0
ALLOWED = ("Mount deck", "Arm length", "Tower face, height", "Leg spacing", "Hinge position", "QD arm", "Fins against the arm")


def tower(sites: dict) -> dict:
    """The [site.tower] table of the prototype's site."""
    table = sites.get("site", sites.get("sites"))
    for site in table:
        if site.get("id") == SITE_ID:
            return site["tower"]
    raise KeyError(SITE_ID)


def check(detail: dict, phys: dict) -> tuple[list, list]:
    """(failures, used exceptions): each failure and exception a text line / row name."""
    failures, exceptions = [], []

    def within(name, drawn, physics, tol, row=None):
        if abs(drawn - physics) <= tol:
            return
        if row is None:
            failures.append(f"{name}: drawn {drawn:.3f} m, physics {physics:.3f} m (tol {tol:.2f})")
        elif row not in exceptions:
            exceptions.append(row)

    # Mount: the ring top (what the stack stands on) is the physics height; the frustum's slope is stretched from the
    # measured 14.5 m, so the drawn shape differs from the measurement (deviations table, "Mount deck").
    within("Mount deck", detail["mount"]["deck_height_m"], phys["mount_height_m"], SURFACE_TOL_M)
    within("Mount deck", detail["mount"]["deck_height_m"], detail["mount"]["measured_height_m"], LENGTH_TOL_M, "Mount deck")
    c = detail["chopstick"]
    half_gap = 0.5 * phys["arm_closed_gap_m"]
    pivot = half_gap + 0.5 * phys["arm_width_m"]
    # What the hull touches: the arm's inner face at the closed pose; deck top on the pivot plane is Z = 0 by the layout.
    within("Arm inner face at the closed pose", pivot - c["inner_face_y_m"], half_gap, SURFACE_TOL_M)
    within("Arm length", c["length_m"], phys["arm_length_m"], LENGTH_TOL_M, "Arm length")
    within("Hinge position", detail["carriage"]["hinge_y_m"], pivot, LENGTH_TOL_M, "Hinge position")
    within("Tower width", detail["tower"]["sections"][0]["face_width_m"], phys["width_m"], LENGTH_TOL_M)
    within("Tower height", detail["tower"]["roof"]["block_base_m"] + detail["tower"]["roof"]["block_height_m"],
           phys["height_m"], LENGTH_TOL_M, "Tower face, height")
    # Arm tops: the deck strip's top is the physics arm's top plane (Z = 0 at the hinge height) the pins land on.
    strips = [m for m in mechazilla_layout.chopstick(detail) if m[0] == "strip"]
    within("Arm top plane", max(max(m[3][2], m[4][2]) + 0.5 * (m[5] or 0.0) for m in strips), 0.0, SURFACE_TOL_M)
    # Leg spacing: the outer width must stay inside the physics box; centres against the measurement.
    leg = detail["profiles"][detail["tower"]["sections"][0]["leg_profile"]]["size_m"]
    outer = detail["tower"]["sections"][0]["face_width_m"]
    if outer > phys["width_m"] + LENGTH_TOL_M:
        failures.append(f"Leg spacing: outer width {outer:.2f} m exceeds the physics box {phys['width_m']:.2f} m")
    within("Leg spacing", outer - leg, MEASURED_LEG_CENTRES_M, LEG_CENTRES_TOL_M, "Leg spacing")
    # QD arm: the head must not reach into the hull; a gap means it is static and not attached (no physics state).
    hinge_to_axis = phys["distance_m"] - 0.5 * phys["width_m"]
    gap = hinge_to_axis - detail["qd_arm"]["reach_m"] - HULL_RADIUS_M
    if gap < 0.0:
        failures.append(f"QD arm: head {-gap:.2f} m inside the hull")
    within("QD arm", gap, 0.0, SURFACE_TOL_M, "QD arm")
    # Fins against the arm: the arm's drawn structure below the deck at the axis station against the distance from the
    # pins down to the fin hinge (vehicle config); deeper structure runs through the fin plane of a catch.
    with open(VEHICLE, "rb") as f:
        stage = tomllib.load(f)["stage"][0]
    gap_fin_m = stage["catch_pins"]["station_m"] - stage["grid_fins"]["station_m"]
    depth_m = -mechazilla_layout.lower_chord_z(c, hinge_to_axis)
    if depth_m > gap_fin_m + SURFACE_TOL_M:
        exceptions.append("Fins against the arm")
    return failures, exceptions


def main() -> int:
    with open(ROOT / "config" / "launch_sites.toml", "rb") as f:
        sites = tomllib.load(f)
    failures, exceptions = check(detail_file.load(DETAIL), tower(sites))
    for row in exceptions:
        print(f"exception (deviations table): {row}")
    for line in failures:
        print(f"FAIL: {line}")
    print("ENVELOPE " + ("FAILED" if failures else "PASSED"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
