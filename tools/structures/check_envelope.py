"""Physics envelope check of the drawn Mechazilla: a drawn surface the vehicle touches sits on the physics number.

    python tools/structures/check_envelope.py

Tolerances 0.10 m (lengths) / 0.05 m (surfaces the vehicle touches). The only allowed exceptions are the rows of the
"Physics deviations" table in docs/subsystems/launch-structures.md (ALLOWED); they are listed by name. Exit 1 on a failure.
"""
from __future__ import annotations

import sys
import tomllib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import detail_file  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
DETAIL = ROOT / "config" / "structures" / "starbase_olp2.toml"
SITE_ID = "starbase_olp1"
LENGTH_TOL_M = 0.10
SURFACE_TOL_M = 0.05
ALLOWED = ("Arm length", "Tower face, height", "Leg spacing", "Hinge position", "QD arm", "Fins against the arm")


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
    # Arm deck against the fins and the static QD arm: no number in the physics, listed always.
    exceptions.extend(["Fins against the arm", "QD arm", "Leg spacing"])
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
