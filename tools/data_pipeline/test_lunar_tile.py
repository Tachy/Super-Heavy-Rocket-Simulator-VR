"""Unit tests of the LOLA tile pipeline (projection, label parsing, crop). No network.

Run: python tools/data_pipeline/test_lunar_tile.py
"""

import json
import math
import struct
import subprocess
import sys
import tomllib
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).parent))
import lunar_tile as lt  # noqa: E402

R = 1737400.0


class Projection(unittest.TestCase):
    def test_pole_is_at_the_pole_pixel(self):
        line, sample = lt.polar_south_line_sample(-math.pi / 2, 0.0, 15199.5, 15199.5, 20.0)
        self.assertAlmostEqual(line, 15199.5)
        self.assertAlmostEqual(sample, 15199.5)

    def test_longitude_zero_points_up_and_east_is_to_the_right(self):
        lat = math.radians(-85.0)
        l0, s0 = lt.polar_south_line_sample(lat, 0.0, 15199.5, 15199.5, 20.0)
        l90, s90 = lt.polar_south_line_sample(lat, math.radians(90.0), 15199.5, 15199.5, 20.0)
        self.assertLess(l0, 15199.5)  # up
        self.assertAlmostEqual(s0, 15199.5)
        self.assertGreater(s90, 15199.5)  # right
        self.assertAlmostEqual(l90, 15199.5)

    def test_round_trip(self):
        for lat_deg, lon_deg in [(-84.79, 29.19), (-89.0, 200.0), (-80.5, 359.0)]:
            line, sample = lt.polar_south_line_sample(math.radians(lat_deg), math.radians(lon_deg),
                                                      15199.5, 15199.5, 20.0)
            lat, lon = lt.polar_south_lat_lon(line, sample, 15199.5, 15199.5, 20.0)
            self.assertAlmostEqual(math.degrees(lat), lat_deg, places=9)
            self.assertAlmostEqual(math.degrees(lon) % 360.0, lon_deg % 360.0, places=9)

    def test_pixel_distance_from_the_pole_matches_the_label_note(self):
        # LOLA GDR help: colatitude of a pixel r metres from the pole = 2 atan(0.5 r / R)
        line, _ = lt.polar_south_line_sample(math.radians(-85.0), 0.0, 15199.5, 15199.5, 20.0)
        r = (15199.5 - line) * 20.0
        self.assertAlmostEqual(math.degrees(2.0 * math.atan(0.5 * r / R)), 5.0, places=9)

    def test_cylindrical_ldem16(self):
        # LDEM_16: line 1439.5 at the equator, sample 2879.5 at 180 E, 16 px/deg
        line, sample = lt.cylindrical_line_sample(0.0, math.pi, 16.0, 1439.5, -0.5)
        self.assertAlmostEqual(line, 1439.5)
        self.assertAlmostEqual(sample, 2879.5)
        line, sample = lt.cylindrical_line_sample(math.radians(-90.0), math.radians(-90.0), 16.0, 1439.5, -0.5)
        self.assertAlmostEqual(line, 1439.5 + 90 * 16)
        self.assertAlmostEqual(sample, 270 * 16 - 0.5)  # longitudes wrap to 0..360


class Label(unittest.TestCase):
    def test_parses_keywords_with_units(self):
        text = ('LINES = 30400\nLINE_SAMPLES = 30400\nSAMPLE_BITS = 16\nSCALING_FACTOR = 0.5\n'
                'MAP_SCALE = 20 <METERS/PIXEL>\nLINE_PROJECTION_OFFSET = 15199.5 <PIXEL>\n'
                'SAMPLE_TYPE = LSB_INTEGER\nMAP_PROJECTION_TYPE = "POLAR STEREOGRAPHIC"\n'
                'A_AXIS_RADIUS = 1737.4 <KM>\n')
        label = lt.parse_label(text)
        self.assertEqual(label["LINES"], 30400.0)
        self.assertEqual(label["MAP_SCALE"], 20.0)
        self.assertEqual(label["LINE_PROJECTION_OFFSET"], 15199.5)
        self.assertEqual(label["SAMPLE_TYPE"], "LSB_INTEGER")
        self.assertEqual(label["MAP_PROJECTION_TYPE"], "POLAR STEREOGRAPHIC")
        self.assertEqual(label["A_AXIS_RADIUS"], 1737.4)

    def test_first_occurrence_wins(self):
        # labels repeat keywords in nested objects; the image object's value comes first
        label = lt.parse_label("LINES = 10\nOBJECT = X\nLINES = 3\nEND_OBJECT = X\n")
        self.assertEqual(label["LINES"], 10.0)


class Crop(unittest.TestCase):
    def test_crop_rows_and_bilinear(self):
        samples, lines = 8, 6
        rows = b"".join(struct.pack("<8h", *[10 * l + s for s in range(samples)]) for l in range(lines))
        tile = lt.crop_rows(rows, samples, first_sample=2, count=3)  # 6 x 3
        self.assertEqual(len(tile), 6)
        self.assertEqual(tile[1], [12, 13, 14])
        self.assertAlmostEqual(lt.bilinear(tile, 0.5, 0.5), 0.25 * (2 + 3 + 12 + 13))
        self.assertEqual(lt.bilinear(tile, 2.0, 1.0), 23)


class RejectedTile(unittest.TestCase):
    def test_a_tile_failing_the_orientation_check_is_removed(self):
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            (out / "site.toml").write_text("[grid]\n", encoding="utf-8")
            (out / "site.i16").write_bytes(b"\0\0")

            def failing(tile_name, tile_dir, global_dir):
                raise RuntimeError("mis-oriented")

            with mock.patch.object(lt, "check_orientation", failing):
                with self.assertRaises(RuntimeError):
                    lt.finish_tile("site", out, out)
            self.assertFalse((out / "site.toml").exists())  # fetch.py must not take it as present next time
            self.assertFalse((out / "site.i16").exists())


def _fake_root(tmp: str) -> Path:
    root = Path(tmp)
    (root / "build" / "core").mkdir(parents=True)
    (root / lt.CLI_EXE).write_bytes(b"")
    return root


def _plan(args):
    kind, lat, lon = args[3], args[4], args[5]
    ranges = [{"index": i, "file_index": 0, "url": f"https://example.invalid/{kind}.img", "first": 100 * i,
               "last": 100 * i + 99} for i in range(2)]
    return subprocess.CompletedProcess(args, 0, json.dumps({"cache_key": f"{kind}_p_{lat}_{lon}_75", "ranges": ranges}), "")


def _assemble(args):
    out = Path(args[5])
    out.write_text('[grid]\nprojection = "simple_cylindrical"\nheights = "x.i16"\n', encoding="utf-8")
    out.with_suffix(".i16").write_bytes(b"\0\0")
    return subprocess.CompletedProcess(args, 0, "", "")


def _range_body(url, byte_range=None):
    return bytes(byte_range[1] - byte_range[0] + 1)


class SiteTilesThroughTheCli(unittest.TestCase):
    SITES = [{"id": f"site_{k}", "latitude_deg": 10.0 * k, "longitude_deg": 20.0 * k} for k in range(4)]

    def test_fetch_site_tiles_builds_requests_via_the_cli_for_every_site(self):
        import tempfile
        calls, gets = [], []

        def cli(args):
            calls.append(args)
            return _plan(args) if args[1] == "plan" else _assemble(args)

        def get(url, byte_range=None):
            gets.append((url, byte_range))
            return _range_body(url, byte_range)

        with tempfile.TemporaryDirectory() as tmp:
            root = _fake_root(tmp)
            with mock.patch.object(lt, "_run_cli", cli), mock.patch.object(lt, "_get", get):
                lt.fetch_site_tiles(self.SITES, root)
            plans = [c for c in calls if c[1] == "plan"]
            self.assertEqual(len(plans), 8)  # four sites x (height, albedo)
            self.assertEqual(sorted({c[3] for c in plans}), ["albedo", "height"])
            self.assertEqual(len([c for c in calls if c[1] == "assemble"]), 8)
            self.assertEqual(len(gets), 16)  # two ranges per tile
            sites_dir = root / "data" / "terrain" / "sites"
            self.assertEqual(len(list(sites_dir.glob("*.toml"))), 8)
            self.assertEqual(list(sites_dir.glob(".*work")), [])  # the work folders are gone

    def test_an_albedo_refusal_is_not_fatal_a_height_refusal_is(self):
        import tempfile
        refuse_height = [False]

        def cli(args):
            if args[1] == "plan" and (args[3] == "albedo" or refuse_height[0]):
                return subprocess.CompletedProcess(args, 2, "", "no product")
            return _plan(args) if args[1] == "plan" else _assemble(args)

        with tempfile.TemporaryDirectory() as tmp:
            root = _fake_root(tmp)
            with mock.patch.object(lt, "_run_cli", cli), mock.patch.object(lt, "_get", _range_body):
                lt.fetch_site_tiles(self.SITES[:1], root)
                refuse_height[0] = True
                with self.assertRaises(SystemExit):
                    lt.fetch_site_tiles(self.SITES[:1], root, force=True)

    def test_http_429_stops_and_reports_the_retry_after(self):
        import tempfile
        calls = []

        def get(url, byte_range=None):
            calls.append(url)
            raise lt.RateLimited(url, 3600)

        with tempfile.TemporaryDirectory() as tmp:
            root = _fake_root(tmp)
            with mock.patch.object(lt, "_run_cli", _plan), mock.patch.object(lt, "_get", get):
                with self.assertRaises(SystemExit) as stop:
                    lt.fetch_site_tiles(self.SITES, root)
            self.assertIn("3600", str(stop.exception))
            self.assertLessEqual(len(calls), 2)  # no retry loop, no further tiles

    def test_get_does_not_retry_a_429(self):
        attempts = []

        def refuse(url, byte_range):
            attempts.append(1)
            raise lt.RateLimited(url, 5)

        with mock.patch.object(lt, "_open", refuse):
            with self.assertRaises(lt.RateLimited):
                lt._get("https://example.invalid/x", (0, 9))
        self.assertEqual(len(attempts), 1)


class Sources(unittest.TestCase):
    def test_old_ldem_16_config_entry_is_gone(self):
        path = Path(__file__).parent / "sources.toml"
        sources = tomllib.loads(path.read_text(encoding="utf-8"))["source"]
        by_id = {s["id"]: s for s in sources}
        self.assertNotIn("lola_ldem_16", by_id)
        self.assertIn("lola_ldem_64", by_id)
        self.assertNotIn("ldem_16", path.read_text(encoding="utf-8"))
        self.assertTrue(by_id["lunar_test_sites"]["opt_in"])
        self.assertEqual(len(by_id["lunar_test_sites"]["sites"]), 2)


if __name__ == "__main__":
    unittest.main()
