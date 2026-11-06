"""Offline consistency checks of config/lunar_terrain_products.toml (the labels are checked by
probe_lunar_products.py --verify-config, which needs the network)."""

import tomllib
import unittest
from pathlib import Path

import probe_lunar_products

ROOT = Path(__file__).resolve().parents[2]
BYTES = {"int16": 2, "uint16": 2, "float32": 4, "uint8": 1}
PROJECTIONS = {"simple_cylindrical", "polar_stereographic_south", "polar_stereographic_north"}


def load() -> dict:
    with (ROOT / "config" / "lunar_terrain_products.toml").open("rb") as f:
        return tomllib.load(f)


class ProductConfig(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load()
        self.products = {p["id"]: p for p in self.config["product"]}
        with (ROOT / "tools" / "data_pipeline" / "sources.toml").open("rb") as f:
            self.sources = {s["id"]: s for s in tomllib.load(f)["source"]}

    def test_selection_names_existing_products_of_the_right_kind(self):
        for key, kind in (("height_order", "height"), ("albedo_order", "albedo")):
            order = self.config["selection"][key]
            self.assertEqual(len(order), len(set(order)))
            for pid in order:
                self.assertEqual(self.products[pid]["kind"], kind, pid)

    def test_every_product_has_a_source_with_a_licence(self):
        for pid, product in self.products.items():
            source = self.sources[product["source"]]
            self.assertIn("public domain", source["license"], pid)

    def test_file_layouts_are_consistent(self):
        urls = set()
        for pid, product in self.products.items():
            self.assertIn(product["projection"], PROJECTIONS, pid)
            self.assertIn(product["sample_type"], BYTES, pid)
            self.assertTrue(product["file"], pid)
            for f in product["file"]:
                self.assertNotIn(f["url"], urls)
                urls.add(f["url"])
                size = f["header_bytes"] + f["lines"] * f["samples"] * BYTES[product["sample_type"]]
                self.assertGreater(size, 0)
                self.assertEqual(f["record_bytes"] % BYTES[product["sample_type"]], 0, f["url"])
                if product["projection"] == "simple_cylindrical":
                    # pixel-edge convention: the file spans exactly lines / ppd degrees of latitude
                    self.assertAlmostEqual(f["lines"] / f["pixels_per_degree"] % 15.0, 0.0, places=9, msg=f["url"])
                    top = f["first_latitude_deg"]
                    self.assertLessEqual(top, product["latitude_max_deg"])
                    self.assertGreaterEqual(top - f["lines"] / f["pixels_per_degree"], product["latitude_min_deg"])
                else:
                    self.assertEqual(f["lines"], f["samples"], f["url"])
                    self.assertAlmostEqual(f["pole_line"], (f["lines"] - 1) / 2.0, delta=0.5, msg=f["url"])

    def test_cylindrical_files_tile_the_coverage_without_gaps(self):
        for pid, product in self.products.items():
            if product["projection"] != "simple_cylindrical":
                continue
            cells = set()
            for f in product["file"]:
                lat_span = f["lines"] / f["pixels_per_degree"]
                lon_span = f["samples"] / f["pixels_per_degree"]
                cell = (round(f["first_latitude_deg"] - lat_span), round(f["first_longitude_deg"]), round(lat_span), round(lon_span))
                self.assertNotIn(cell, cells, pid)
                cells.add(cell)
            area = sum(c[2] * c[3] for c in cells)
            self.assertEqual(area, (product["latitude_max_deg"] - product["latitude_min_deg"]) * 360.0, pid)

    def test_polar_products_carry_the_20_m_override_only_where_decided(self):
        for pid, product in self.products.items():
            if "tile_half_size_m" in product:
                self.assertEqual(product["resolution_m"], 20.0, pid)
                self.assertEqual(product["tile_half_size_m"], 10000.0, pid)


class Pds4Label(unittest.TestCase):
    def test_wac_label_fields(self):
        text = """<Header><offset unit="byte">0</offset><object_length unit="byte">109440</object_length></Header>
        <Array_2D_Image><offset unit="byte">109440</offset><Element_Array><data_type>IEEE754LSBSingle</data_type></Element_Array>
        <Axis_Array><axis_name>Line</axis_name><elements>18240</elements></Axis_Array>
        <Axis_Array><axis_name>Sample</axis_name><elements>27360</elements></Axis_Array></Array_2D_Image>
        <comment> WAC 643 nm band median mosaic </comment>"""
        parsed = probe_lunar_products.parse_pds4(text)
        self.assertEqual((parsed["lines"], parsed["samples"], parsed["header_bytes"], parsed["image_offset"]),
                         (18240, 27360, 109440, 109440))
        self.assertEqual(parsed["data_type"], "IEEE754LSBSingle")


if __name__ == "__main__":
    unittest.main()
