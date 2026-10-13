"""Plot simulated telemetry (core TelemetryRecorder CSV), optionally against a webcast reference.

Usage:
  python tools/plot_telemetry.py <simulated.csv> [--reference <webcast table.csv>] --out <plot.png>
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def read_csv(path: Path) -> dict[str, list[float]]:
    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return {key: [float(r[key]) for r in rows] for key in rows[0]}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("simulation", type=Path)
    parser.add_argument("--reference", type=Path, help="meithan-format webcast telemetry CSV")
    parser.add_argument("--out", type=Path, default=Path("telemetry.png"))
    args = parser.parse_args()

    sim = read_csv(args.simulation)
    ref = read_csv(args.reference) if args.reference else None

    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    panels = [
        (axes[0][0], "Geschwindigkeit relativ zur Erde [m/s]", "speed_relative_mps", 1.0, "Smoothed speed [m/s]"),
        (axes[0][1], "Höhe [km]", "altitude_m", 1e-3, "Smoothed altitude [km]"),
        (axes[1][0], "Staudruck [kPa]", "dynamic_pressure_pa", 1e-3, "Dynamic pressure [kPa]"),
        (axes[1][1], "Mach-Zahl", "mach", 1.0, "Mach number"),
    ]
    for ax, title, key, scale, ref_key in panels:
        ax.plot(sim["time_s"], [v * scale for v in sim[key]], label="Simulation", color="#3b6fb6", linewidth=2)
        if ref and ref_key in ref:
            ax.plot(ref["Time [s]"], ref[ref_key], label="Webcast-Telemetrie", color="#d0632b", linestyle="--")
        ax.set_title(title)
        ax.grid(alpha=0.3)
        ax.legend()
    for ax in axes[1]:
        ax.set_xlabel("Zeit nach Abheben [s]")
    max_t = max(sim["time_s"])
    for row in axes:
        for ax in row:
            ax.set_xlim(0, max_t)
    fig.suptitle(args.simulation.stem)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
