"""Unit tests of vehicle_report's `from` stage resolution (same rules as core/src/config/vehicle_config.cpp).

Run: python tools/test_vehicle_report.py
"""

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import vehicle_report  # noqa: E402

SOURCE = """[vehicle]
id = "source"
[[stage]]
id = "ship"
dry_mass_kg = 100.0
[[stage.docking_port]]
id = "nose"
"""


def config_with(d: Path, vehicles: dict) -> Path:
    (d / "vehicles").mkdir()
    for name, text in vehicles.items():
        (d / "vehicles" / f"{name}.toml").write_text(text, encoding="utf-8")
    return d


def resolve(vehicles: dict, vehicle_id: str) -> dict:
    with tempfile.TemporaryDirectory() as d:
        config_dir = config_with(Path(d), vehicles)
        vehicle = vehicle_report.load_toml(config_dir / "vehicles" / f"{vehicle_id}.toml")
        return vehicle_report.resolve_stages(vehicle, config_dir)


def reused(body: str) -> str:
    return '[vehicle]\nid = "test"\n[[stage]]\n' + body


class FromResolution(unittest.TestCase):
    def test_cycle_is_refused(self):
        with self.assertRaisesRegex(SystemExit, "chain too long"):
            resolve({"test": reused('from = "test:ship"\n')}, "test")

    def test_three_part_reference_is_refused(self):
        with self.assertRaisesRegex(SystemExit, "<vehicle>:<stage>"):
            resolve({"source": SOURCE, "test": reused('from = "source:ship:x"\n')}, "test")

    def test_other_keys_cannot_be_set(self):
        with self.assertRaisesRegex(SystemExit, "cannot be set on a reused stage"):
            resolve({"source": SOURCE, "test": reused('from = "source:ship"\nlength_m = 1.0\n')}, "test")

    def test_added_ports_are_appended_and_dry_mass_replaced(self):
        stages = resolve({"source": SOURCE, "test": reused('from = "source:ship"\ndry_mass_kg = 50.0\n'
                                                           '[[stage.docking_port]]\nid = "dorsal"\n')}, "test")
        self.assertEqual([p["id"] for p in stages["ship"]["docking_port"]], ["nose", "dorsal"])
        self.assertEqual(stages["ship"]["dry_mass_kg"], 50.0)

    def test_tanker_from_the_real_config(self):
        vehicle = vehicle_report.load_toml(vehicle_report.CONFIG_DIR / "vehicles" / "starship_tanker.toml")
        ship = vehicle_report.resolve_stages(vehicle)["ship"]
        self.assertEqual(ship["dry_mass_kg"], 209000.0)
        self.assertEqual([p["id"] for p in ship["docking_port"]], ["transfer_dock"])


if __name__ == "__main__":
    unittest.main()
