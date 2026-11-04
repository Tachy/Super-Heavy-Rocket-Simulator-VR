"""Unit tests of the Diviner rock abundance pipeline (label -> header). No network.

Run: python tools/data_pipeline/test_rock_abundance.py
"""

import sys
import tomllib
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import lunar_tile as lt  # noqa: E402
import rock_abundance as ra  # noqa: E402

# The image and projection keywords of the real DGDR_RA_AVG_CYL_032_IMG.LBL (PDS Geosciences Node, read 2026-11-04).
LABEL = """
 OBJECT                   = IMAGE
  LINES                   = 3840
  LINE_SAMPLES            = 11520
  SAMPLE_TYPE             = LSB_INTEGER
  SAMPLE_BITS             = 16
  SCALING_FACTOR          = 0.001
  OFFSET                  = 0.00
  MISSING_CONSTANT        = -32768
 END_OBJECT               = IMAGE
 MAP_PROJECTION_TYPE          = "SIMPLE CYLINDRICAL"
 MAP_RESOLUTION               = 32 <pix/degree>
 POSITIVE_LONGITUDE_DIRECTION = "EAST"
 CENTER_LONGITUDE             = 180.0 <deg>
 MAXIMUM_LATITUDE             = 60 <deg>
 MINIMUM_LATITUDE             = -60 <deg>
 LINE_PROJECTION_OFFSET       = 1919.5 <pixel>
 SAMPLE_PROJECTION_OFFSET     = 5759.5 <pixel>
"""


class Header(unittest.TestCase):
    def test_real_label_gives_the_layout_the_core_loader_expects(self):
        grid = tomllib.loads(ra.header_text(lt.parse_label(LABEL), "DGDR_RA_AVG_CYL_032_IMG"))["grid"]
        self.assertEqual(grid["lines"], 3840)
        self.assertEqual(grid["samples"], 11520)
        self.assertEqual(grid["pixels_per_degree"], 32.0)
        self.assertEqual(grid["zero_line"], 0.5 * 3840 - 0.5)  # equator between lines 1919 and 1920
        self.assertEqual(grid["zero_sample"], -0.5)  # sample 0 centred half a pixel east of longitude 0
        self.assertEqual(grid["max_latitude_deg"], 60.0)
        self.assertEqual(grid["dn_scale"], 0.001)
        self.assertEqual(grid["missing_dn"], -32768)
        self.assertEqual(grid["values"], ra.VALUES_NAME)

    def test_unexpected_layout_is_refused(self):
        for old, new in (("MINIMUM_LATITUDE             = -60", "MINIMUM_LATITUDE             = -50"),
                         ("SAMPLE_BITS             = 16", "SAMPLE_BITS             = 32"),
                         ("OFFSET                  = 0.00", "OFFSET                  = 0.10")):
            with self.assertRaises(RuntimeError):
                ra.header_text(lt.parse_label(LABEL.replace(old, new)), "X")


if __name__ == "__main__":
    unittest.main()
