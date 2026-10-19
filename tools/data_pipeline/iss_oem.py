"""NASA ISS trajectory data (CCSDS OEM): fetch the current file and keep every version.

The file at the source changes with every NASA update (~3x a week, 15 days ahead). Each fetched version is stored as
data/iss_oem/ISS.OEM_J2K_EPH_<CREATION_DATE>.txt (colons replaced), so past dates stay covered by the newest
prediction made before them (core: orbit::OemArchive).
"""

from __future__ import annotations

import re
from pathlib import Path

_CREATION = re.compile(r"^CREATION_DATE\s*=\s*(\S+)", re.M)


def archive_name(text: str) -> str:
    match = _CREATION.search(text)
    if not match:
        raise ValueError("OEM without CREATION_DATE")
    stamp = match.group(1).split(".")[0].replace(":", "-")
    return f"ISS.OEM_J2K_EPH_{stamp}.txt"


def store(data: bytes, archive_dir: Path) -> Path | None:
    """Stores a version byte-exact; returns its path, or None if that version is already archived."""
    archive_dir.mkdir(parents=True, exist_ok=True)
    target = archive_dir / archive_name(data.decode("ascii", errors="replace"))
    if target.exists():
        return None
    target.write_bytes(data)
    return target


def fetch(source: dict, repo_root: Path, download) -> None:
    """Always downloads (the current file is a moving target) and archives a new version."""
    archive_dir = repo_root / source["target"]
    partial = archive_dir / "current.download"
    download(source["url"], partial)
    try:
        stored = store(partial.read_bytes(), archive_dir)
    finally:
        partial.unlink(missing_ok=True)
    print(f"  archived {stored.name}" if stored else "  this version is already archived")
