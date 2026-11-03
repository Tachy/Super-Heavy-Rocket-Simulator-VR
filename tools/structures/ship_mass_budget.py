"""Bottom-up dry CG of the Block 3 ship for a given dry mass (vehicle physics CG audit 2026-11-01, planner plan Task 3b).

The named items (engines, structure, shell, flaps, shield, lines, pins, RCS, avionics) are placed at the drawn / config
stations; their masses are estimates [E] unless a config value is named. The rest of the dry mass (the part no item
explains) is distributed like the shell (thicker walls and domes, stringers, welds): the most plausible of the audit's
scenarios; the others give the uncertainty.

  python tools/structures/ship_mass_budget.py --dry-t 177
"""
import argparse

import numpy as np

L, R, NOSE = 52.1, 4.5, 13.4      # ship length, radius, drawn ogive length [m] (starship_block3.toml)
BARREL_END = L - NOSE
RHO_STEEL = 7900.0
RHO_OGIVE = (R * R + NOSE * NOSE) / (2 * R)


def radius(x):
    s = np.clip(np.asarray(x, float) - BARREL_END, 0, NOSE)
    return np.sqrt(RHO_OGIVE ** 2 - s ** 2) + R - RHO_OGIVE


def geometry():
    xs = np.linspace(BARREL_END, L, 2001)
    r = radius(xs)
    ds = np.sqrt(np.gradient(xs) ** 2 + np.gradient(r) ** 2)
    ogive_area = np.sum(2 * np.pi * r * ds)
    ogive_cg = np.sum(2 * np.pi * r * ds * xs) / ogive_area
    xx = np.linspace(0, L, 5201)
    # heat shield coverage (detail [ship.tiles]): half angle over station
    half = np.interp(xx, [0.0, 13.0, 13.5, 21.5, 22.0, 38.0, 41.0, 52.1], [90, 90, 112, 112, 105, 105, 180, 180])
    dA = 2 * np.radians(half) * radius(xx) * np.gradient(xx)
    return ogive_area, ogive_cg, (dA * xx).sum() / dA.sum()


COMMON_DOME, FORWARD_DOME = 19.5, 32.2  # Block 3 ship's main tank domes [m] (tank columns of starship_block3.toml)
CH4_LINE_FOOT, CH4_LINE_KG = 3.3, 700.0  # CH4 feed line from the engines up through the LOX tank [E]

# Tanker with stretched main tanks (planner plan Task 4b, user 2026-11-03) [E]: the forward dome moves from 32.2 m up to
# the end of the cylindrical barrel (38.7 m, the ogive above holds the dome cap, the LOX header and its downcomer); the
# common dome moves up so that the main tanks keep O/F 3.6 (tank columns at the config densities). LOX at the bottom,
# CH4 above as in Block 3.
OXIDIZER_RHO, FUEL_RHO, MIXTURE = 1200.0, 440.0, 3.6  # config/propellants.toml methalox; Raptor O/F (main tanks)
LOX_BASE, BLOCK3_LOX_TOP, BLOCK3_CH4_BASE, BLOCK3_CH4_TOP = 3.5, 19.544, 20.0, 32.153  # tank columns [m]


def tanker_tanks(forward_dome=BARREL_END):
    """Stretched main tanks: LOX / CH4 mass [kg], column base stations [m] and the common dome station."""
    area = np.pi * R * R
    gap = BLOCK3_CH4_BASE - BLOCK3_LOX_TOP              # between the columns (common dome), kept
    ch4_top = BLOCK3_CH4_TOP + (forward_dome - FORWARD_DOME)  # the column top follows the dome as in Block 3
    volume = area * (ch4_top - LOX_BASE - gap)
    fuel = volume / (MIXTURE / OXIDIZER_RHO + 1.0 / FUEL_RHO)
    oxidizer = MIXTURE * fuel
    lox_top = LOX_BASE + oxidizer / (OXIDIZER_RHO * area)
    return oxidizer, fuel, lox_top + gap, ch4_top, COMMON_DOME + (lox_top - BLOCK3_LOX_TOP)


def items(tanker=False):
    ogive_area, ogive_cg, shield_cg = geometry()
    wall = 0.004 * RHO_STEEL * 1.25  # 4 mm [E config thermal], stringers +25 % [E]
    barrel_len = BARREL_END - 3.7
    barrel_area = 2 * np.pi * R * barrel_len
    dome = 1.3 * np.pi * R * R * 39.5  # 5 mm domes [E]
    common, forward = COMMON_DOME, FORWARD_DOME
    if tanker:
        forward = BARREL_END
        common = tanker_tanks(forward)[4]
    shift = common - COMMON_DOME  # the CH4 header and the CH4 line's top follow the common dome
    ch4_line_len = 2 * (11.9 - CH4_LINE_FOOT) + shift
    named = [  # name, kg, station m, error kg, error m
        ("Raptor 3 x 3", 3 * 1525.0, 2.2, 0.0, 0.4),
        ("RVac 3 x 3", 3 * 1950.0, 3.0, 450.0, 0.5),
        ("vehicle-side hardware x 6", 6 * 195.0, 3.3, 0.0, 0.3),
        ("thrust structure, puck, plate", 6000.0, 3.2, 3000.0, 0.5),
        ("skirt 0-3.7 m", 2 * np.pi * R * 3.7 * 60.0, 1.85, 2000.0, 0.2),
        ("barrel 3.7-38.7 m", barrel_area * wall, 3.7 + barrel_len / 2, 0.25 * barrel_area * wall, 0.5),
        ("ogive 38.7-52.1 m", ogive_area * wall, ogive_cg, 0.3 * ogive_area * wall, 0.3),
        ("LOX aft dome", dome, 3.7, 1500.0, 0.5),
        ("common dome", dome, common, 1500.0, 0.5),
        ("CH4 forward dome", dome, forward, 1500.0, 0.5),
        ("aft flaps", 2 * 36.48 * 110.0, 7.06, 0.4 * 2 * 36.48 * 110.0, 0.5),
        ("aft flap actuators, hinges", 2000.0, 8.0, 1000.0, 2.0),
        ("forward flaps", 2 * 12.374 * 110.0, 44.21, 0.4 * 2 * 12.374 * 110.0, 0.5),
        ("forward flap actuators, hinges", 1400.0, 44.5, 700.0, 1.0),
        ("heat shield", 10500.0, shield_cg, 3000.0, 1.0),
        ("payload bay door, dispenser", 0.0 if tanker else 6000.0, 37.0, 0.0 if tanker else 3000.0, 2.0),
        ("LOX header shell", 1500.0, 45.1, 750.0, 1.0),
        ("CH4 header shell", 1000.0, 21.2 + shift, 500.0, 1.0),
        ("LOX downcomer", 1500.0, 23.9, 750.0, 1.0),
        ("CH4 line", CH4_LINE_KG * ch4_line_len / (2 * (11.9 - CH4_LINE_FOOT)), CH4_LINE_FOOT + ch4_line_len / 2,
         350.0, 1.0),
        ("catch pins + frames", 2000.0, 36.5, 1000.0, 0.5),
        ("RCS pods", 1600.0, 26.0, 800.0, 2.0),
        ("avionics, batteries, pressurant", 5000.0, 24.0, 2500.0, 10.0),
    ]
    shell_m = barrel_area * wall + ogive_area * wall + 3 * dome
    shell_x = (barrel_area * wall * (3.7 + barrel_len / 2) + ogive_area * wall * ogive_cg
               + dome * (3.7 + common + forward)) / shell_m
    return named, shell_x


def totals(named):
    m = np.array([i[1] for i in named])
    x = np.array([i[2] for i in named])
    em = np.array([i[3] for i in named])
    ex = np.array([i[4] for i in named])
    M, cg = m.sum(), (m * x).sum() / m.sum()
    return M, cg, np.sqrt((((x - cg) * em / M) ** 2 + (m * ex / M) ** 2).sum())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-t", type=float, required=True, help="the Block 3 ship's dry mass")
    ap.add_argument("--tanker", action="store_true",
                    help="the tanker: no door / dispenser, main tanks stretched to the barrel end; the rest of the "
                         "Block 3 budget (same hull) kept, placed like the tanker's shell")
    a = ap.parse_args()
    named, shell_x = items()
    M, cg, ecg = totals(named)
    dry = a.dry_t * 1e3
    rest = dry - M
    print(f"named items {M / 1e3:.1f} t at {cg:.2f} +- {ecg:.2f} m; rest {rest / 1e3:.1f} t of {a.dry_t:.1f} t")
    if a.tanker:
        oxidizer, fuel, ch4_base, ch4_top, common = tanker_tanks()
        area = np.pi * R * R
        print(f"tanker main tanks: LOX {oxidizer:.1f} kg at {LOX_BASE:.2f}-{LOX_BASE + oxidizer / (OXIDIZER_RHO * area):.3f}"
              f" m ({oxidizer / OXIDIZER_RHO:.1f} m^3), CH4 {fuel:.1f} kg at {ch4_base:.3f}-{ch4_top:.3f} m"
              f" ({fuel / FUEL_RHO:.1f} m^3), common dome {common:.2f} m, total {(oxidizer + fuel) / 1e3:.1f} t")
        named, shell_x = items(tanker=True)
        M, cg, ecg = totals(named)
        dry = M + rest
        print(f"tanker named items {M / 1e3:.1f} t at {cg:.2f} +- {ecg:.2f} m; dry {dry / 1e3:.2f} t")
    for name, rx in (("like the shell (chosen)", shell_x), ("like the named items", cg),
                     ("uniform over the tank section 3.7-32 m", 17.85), ("uniform along the hull", L / 2)):
        print(f"  rest {name} at {rx:.2f} m: dry CG {(M * cg + rest * rx) / dry:.2f} m")


if __name__ == "__main__":
    main()
