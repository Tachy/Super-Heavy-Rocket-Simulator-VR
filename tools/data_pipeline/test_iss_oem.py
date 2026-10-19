"""Unit tests of the ISS OEM archive (naming, de-duplication). No network.

Run: python tools/data_pipeline/test_iss_oem.py
"""

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import iss_oem  # noqa: E402

HEADER = b"CCSDS_OEM_VERS = 2.0\r\nCREATION_DATE  = 2026-10-14T18:30:01.420\r\nORIGINATOR     = NASA/JSC/FOD/TOPO\r\n"


class ArchiveName(unittest.TestCase):
    def test_creation_date_without_colons(self):
        self.assertEqual(iss_oem.archive_name(HEADER.decode()), "ISS.OEM_J2K_EPH_2026-10-14T18-30-01.txt")

    def test_missing_creation_date_is_an_error(self):
        with self.assertRaises(ValueError):
            iss_oem.archive_name("CCSDS_OEM_VERS = 2.0\n")


class Store(unittest.TestCase):
    def test_new_version_is_stored_once_byte_exact(self):
        with tempfile.TemporaryDirectory() as d:
            first = iss_oem.store(HEADER, Path(d))
            self.assertIsNotNone(first)
            self.assertEqual(first.read_bytes(), HEADER)
            self.assertIsNone(iss_oem.store(HEADER, Path(d)))
            newer = HEADER.replace(b"18:30:01", b"19:00:00")
            self.assertIsNotNone(iss_oem.store(newer, Path(d)))
            self.assertEqual(len(list(Path(d).glob("ISS.OEM_J2K_EPH_*.txt"))), 2)


if __name__ == "__main__":
    unittest.main()
