"""Sanity report for a vehicle configuration: masses, tank geometry, thrust, burn time and
the Tsiolkovsky delta-v budget.

Usage: python tools/vehicle_report.py <vehicle-id> [--payload-kg N]

This is an engineering aid for config authors. The simulation core implements the same
equations in C++ (core/propulsion); keep both in sync.
"""

from __future__ import annotations

import argparse
import math
import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
G0_MPS2 = 9.80665              # standard gravity (exact, CGPM 1901)
P0_PA = 101325.0               # standard sea-level pressure (exact)


def load_toml(path: Path) -> dict:
    with path.open("rb") as f:
        return tomllib.load(f)


@dataclass(frozen=True)
class Engine:
    id: str
    thrust_ref_n: float
    isp_ref_s: float
    reference_pressure_pa: float
    exit_area_m2: float
    max_ambient_pressure_pa: float
    mixture_ratio: float
    dry_kg: float

    @staticmethod
    def load(engine_id: str) -> "Engine":
        data = load_toml(CONFIG_DIR / "engines" / f"{engine_id}.toml")
        perf = data["performance"]
        return Engine(
            id=engine_id,
            thrust_ref_n=perf["thrust_ref_n"],
            isp_ref_s=perf["isp_ref_s"],
            reference_pressure_pa=perf["reference_pressure_pa"],
            exit_area_m2=math.pi * (perf["nozzle_exit_diameter_m"] / 2.0) ** 2,
            max_ambient_pressure_pa=perf["max_ambient_pressure_pa"],
            mixture_ratio=data["engine"]["mixture_ratio"],
            dry_kg=data["mass"]["dry_kg"],
        )

    @property
    def mass_flow_kgps(self) -> float:
        """Full-throttle mass flow, independent of ambient pressure."""
        return self.thrust_ref_n / (self.isp_ref_s * G0_MPS2)

    def thrust_n(self, ambient_pressure_pa: float) -> float:
        """F(p) = F_ref - (p - p_ref) * A_exit."""
        return self.thrust_ref_n - (ambient_pressure_pa - self.reference_pressure_pa) * self.exit_area_m2

    def isp_s(self, ambient_pressure_pa: float) -> float:
        return self.thrust_n(ambient_pressure_pa) / (self.mass_flow_kgps * G0_MPS2)


def tsiolkovsky_delta_v(isp_s: float, initial_mass_kg: float, final_mass_kg: float) -> float:
    """Ideal rocket equation: dv = Isp * g0 * ln(m0 / mf)."""
    return isp_s * G0_MPS2 * math.log(initial_mass_kg / final_mass_kg)


def stage_mass_kg(stage: dict) -> tuple[float, float]:
    """Returns (dry incl. payload/fairing and RCS supplies, main propellant)."""
    rcs = sum(t["mass_kg"] for t in stage.get("tank", []) if t["contents"] == "rcs")
    dry = stage["dry_mass_kg"] + stage.get("payload_mass_kg", 0.0) + stage.get("fairing_mass_kg", 0.0) + rcs
    propellant = sum(t["mass_kg"] for t in stage.get("tank", []) if t["contents"] != "rcs")
    return dry, propellant


def report(vehicle_id: str, payload_kg: float) -> int:
    vehicle = load_toml(CONFIG_DIR / "vehicles" / f"{vehicle_id}.toml")
    propellants = {p["id"]: p for p in load_toml(CONFIG_DIR / "propellants.toml")["propellant"]}
    prop = propellants[vehicle["vehicle"]["propellant"]]
    stages = {s["id"]: s for s in vehicle["stage"]}
    order = vehicle["vehicle"]["stages_bottom_to_top"]
    stages[order[-1]]["payload_mass_kg"] = payload_kg
    warnings: list[str] = []

    print(f"=== {vehicle['vehicle']['name']}  (payload {payload_kg / 1000:.1f} t)\n")
    total_dv = 0.0
    for index, stage_id in enumerate(order):
        stage = stages[stage_id]
        dry, propellant = stage_mass_kg(stage)
        upper_mass = sum(sum(stage_mass_kg(stages[s])) for s in order[index + 1:])
        reserve = stage.get("recovery", {}).get("reserve_propellant_kg", 0.0)
        usable = propellant - reserve

        all_groups = [(g, Engine.load(g["engine"])) for g in stage.get("engine_group", [])]
        groups = [(g, e) for g, e in all_groups
                  if not g.get("abort_only", False) and not g.get("landing_only", False)]
        landing = [(g, e) for g, e in all_groups if g.get("landing_only", False)]
        if propellant <= 0.0 or not groups:
            # No main propulsion (e.g. Crew Dragon's capsule and trunk): masses, supplies and abort-only engines only.
            print(f"--- {stage['name']}")
            print(f"  dry {dry / 1e3:9.1f} t (incl. RCS supplies)   no main propellant")
            for tank in stage.get("tank", []):
                print(f"  tank {tank['id']:<10} {tank['mass_kg'] / 1e3:8.3f} t  ({tank['contents']})")
            for group, engine in all_groups:
                print(f"  {group['count']:2d} x {engine.id:<15} SL {engine.thrust_n(P0_PA) / 1e3:7.0f} kN   abort only")
            print()
            continue
        mdot = sum(g["count"] * e.mass_flow_kgps for g, e in groups)
        thrust_sl = sum(g["count"] * e.thrust_n(P0_PA) for g, e in groups
                        if e.max_ambient_pressure_pa >= P0_PA)
        thrust_vac = sum(g["count"] * e.thrust_n(0.0) for g, e in groups)
        isp_vac_eff = thrust_vac / (mdot * G0_MPS2)

        m0 = dry + propellant + upper_mass
        mf = m0 - usable
        dv = tsiolkovsky_delta_v(isp_vac_eff, m0, mf)
        total_dv += dv

        area = math.pi * (stage["diameter_m"] / 2.0) ** 2
        print(f"--- {stage['name']}")
        print(f"  dry {dry / 1e3:9.1f} t   propellant {propellant / 1e3:9.1f} t   "
              f"wet {(dry + propellant) / 1e3:9.1f} t   mass fraction {propellant / (dry + propellant):.3f}")
        for tank in stage.get("tank", []):
            density = (tank["density_kg_m3"] if tank["contents"] == "rcs"
                       else prop["oxidizer_density_kg_m3" if tank["contents"] == "oxidizer" else "fuel_density_kg_m3"])
            tank_area = math.pi * (tank.get("diameter_m", stage["diameter_m"]) / 2.0) ** 2
            length = tank["mass_kg"] / density / tank_area
            top = tank["base_station_m"] + length
            role = " (header)" if tank.get("role") == "header" else ""
            print(f"  tank {tank['id']:<10} {tank['mass_kg'] / 1e3:8.1f} t  {tank['mass_kg'] / density:8.1f} m3  "
                  f"station {tank['base_station_m']:5.1f} .. {top:5.1f} m{role}")
            if top > stage["length_m"]:
                warnings.append(f"{stage_id}/{tank['id']}: tank top {top:.1f} m exceeds stage length")
        ox = sum(t["mass_kg"] for t in stage["tank"] if t["contents"] == "oxidizer")
        fuel = sum(t["mass_kg"] for t in stage["tank"] if t["contents"] == "fuel")
        print(f"  loaded O/F {ox / fuel:.2f} (engine {groups[0][1].mixture_ratio:.2f})")
        if abs(ox / fuel - groups[0][1].mixture_ratio) > 0.15:
            warnings.append(f"{stage_id}: loaded O/F {ox / fuel:.2f} differs from engine mixture ratio")
        for group, engine in groups:
            print(f"  {group['count']:2d} x {engine.id:<15} SL {engine.thrust_n(P0_PA) / 1e3:7.0f} kN "
                  f"({engine.isp_s(P0_PA):5.1f} s)   vac {engine.thrust_n(0.0) / 1e3:7.0f} kN "
                  f"({engine.isp_s(0.0):5.1f} s)   mdot {engine.mass_flow_kgps:6.1f} kg/s")
        for group, engine in landing:  # landing thrusters: not in the delta-v budget
            axial = group["count"] * engine.thrust_n(0.0) * math.cos(math.radians(group.get("cant_deg", 0.0)))
            print(f"  landing thrusters: {group['count']} x {engine.thrust_n(0.0) / 1e3:.0f} kN vac "
                  f"(axial {axial / 1e3:.0f} kN, station {group.get('station_m', 0.0):.1f} m)")
        print(f"  total thrust SL {thrust_sl / 1e6:6.2f} MN   vac {thrust_vac / 1e6:6.2f} MN   "
              f"mdot {mdot:8.1f} kg/s   Isp_vac,eff {isp_vac_eff:5.1f} s")
        print(f"  ignition mass {m0 / 1e3:9.1f} t   T/W at ignition: SL {thrust_sl / (m0 * G0_MPS2):4.2f}  "
              f"vac {thrust_vac / (m0 * G0_MPS2):4.2f}")
        print(f"  usable propellant {usable / 1e3:8.1f} t (reserve {reserve / 1e3:.1f} t)   "
              f"full-thrust burn time {usable / mdot:6.1f} s")
        print(f"  ideal delta-v (vacuum, Tsiolkovsky) {dv:7.0f} m/s\n")
        if index == 0 and thrust_sl / (m0 * G0_MPS2) < 1.15:
            warnings.append(f"{stage_id}: liftoff T/W below 1.15")

    print(f"ideal total delta-v (vacuum, no gravity/drag losses): {total_dv:7.0f} m/s")
    print("typical LEO requirement incl. losses: ~9300-9500 m/s")
    for w in warnings:
        print(f"WARNING: {w}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("vehicle_id")
    parser.add_argument("--payload-kg", type=float, default=0.0)
    args = parser.parse_args()
    return report(args.vehicle_id, args.payload_kg)


if __name__ == "__main__":
    sys.exit(main())
