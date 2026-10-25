"""Generate CREDITS.md from tools/data_pipeline/sources.toml.

Usage: python tools/data_pipeline/credits.py
"""

from __future__ import annotations

import sys
import tomllib
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


def main() -> int:
    with (Path(__file__).with_name("sources.toml")).open("rb") as f:
        sources = tomllib.load(f)["source"]
    lines = [
        "# Credits",
        "",
        "Generated from `tools/data_pipeline/sources.toml` by `tools/data_pipeline/credits.py` - do not edit.",
        "",
        "| Data / software | Credit | Licence |",
        "|---|---|---|",
    ]
    for source in sources:
        lines.append(f"| [{source['name']}]({source['url']}) | {source['credit']} | {source['license']} |")
    lines += [
        "",
        "Code dependencies: Godot Engine (MIT), godot-cpp (MIT), Catch2 (BSL-1.0),",
        "DOP853 coefficients from SciPy (BSD-3-Clause).",
        "",
    ]
    # Licences whose terms require their full text with the binaries (e.g. BSD-3 clause 2), verbatim.
    with_text = [s for s in sources if s.get("license_text")]
    if with_text:
        lines += ["## Third-party licences", ""]
        for source in with_text:
            lines += [f"### {source['name']}", "", "```", source["license_text"].strip("\n"), "```", ""]
    (REPO_ROOT / "CREDITS.md").write_text("\n".join(lines), encoding="utf-8")
    print(f"wrote {REPO_ROOT / 'CREDITS.md'} ({len(sources)} sources)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
