"""Diviner rock abundance map (Moon procedural surface, rocks).

Product: LRO Diviner GDR Level 3 rock abundance, average of the night-time local times, simple cylindrical,
32 px/deg (DGDR_RA_AVG_CYL_032_IMG, PDS4 bundle urn:nasa:pds:lro_diviner_derived1, PDS3 label beside the image;
Bandfield et al. 2011). Layout read from its label [S]: 3840 lines x 11520 samples of int16 LSB, line 0 = 60 N
(LINE_PROJECTION_OFFSET 1919.5 = the 0-based line of the equator), longitude 0..360 east with CENTER_LONGITUDE 180 at
SAMPLE_PROJECTION_OFFSET 5759.5 (sample of longitude 0 = -0.5: sample 0 is centred half a pixel east of 0), area fraction
= DN x SCALING_FACTOR (0.001) + OFFSET (0), MISSING_CONSTANT -32768, covers 60 S - 60 N. The 128 px/deg version is
1.9 GB; 32 px/deg (948 m pixels, 88 MB) is ample for a population scale.

Output in data/terrain/: the DN unchanged (diviner_rock_abundance.i16) and a TOML header (diviner_rock_abundance.toml)
read by core/src/config/rock_abundance_file.cpp.
"""

from __future__ import annotations

from pathlib import Path

from lunar_tile import parse_label, _get, _label_url, _require

VALUES_NAME = "diviner_rock_abundance.i16"
HEADER_NAME = "diviner_rock_abundance.toml"


def header_text(label: dict, product: str) -> str:
    """TOML header for the core loader from the product's PDS3 label (checks the layout the loader assumes)."""
    _require(label, "SAMPLE_TYPE", "LSB_INTEGER")
    _require(label, "SAMPLE_BITS", 16.0)
    _require(label, "MAP_PROJECTION_TYPE", "SIMPLE CYLINDRICAL")
    _require(label, "POSITIVE_LONGITUDE_DIRECTION", "EAST")
    lines, samples = int(label["LINES"]), int(label["LINE_SAMPLES"])
    ppd = label["MAP_RESOLUTION"]
    max_lat = label["MAXIMUM_LATITUDE"]
    if label["MINIMUM_LATITUDE"] != -max_lat or abs(2 * max_lat * ppd - lines) > 1e-9 or abs(360 * ppd - samples) > 1e-9:
        raise RuntimeError(f"{product}: unexpected grid {lines} x {samples} at {ppd} px/deg, "
                           f"latitude {label['MINIMUM_LATITUDE']} .. {max_lat}")
    if label.get("OFFSET", 0.0) != 0.0:
        raise RuntimeError(f"{product}: OFFSET {label['OFFSET']} (the loader assumes 0)")
    # SAMPLE_PROJECTION_OFFSET is the sample of CENTER_LONGITUDE: sample of longitude 0 = offset - ppd x centre
    zero_sample = label["SAMPLE_PROJECTION_OFFSET"] - ppd * label["CENTER_LONGITUDE"]
    return ("# LRO Diviner GDR L3 rock abundance (average) - written by tools/data_pipeline/rock_abundance.py\n"
            "[grid]\n"
            f'product = "{product}"\n'
            'projection = "simple_cylindrical"\n'
            f"lines = {lines}\n"
            f"samples = {samples}\n"
            f"pixels_per_degree = {ppd}\n"
            f"zero_line = {label['LINE_PROJECTION_OFFSET']}\n"
            f"zero_sample = {zero_sample}\n"
            f"max_latitude_deg = {max_lat}\n"
            f"dn_scale = {label['SCALING_FACTOR']}\n"
            f"missing_dn = {int(label['MISSING_CONSTANT'])}\n"
            f'values = "{VALUES_NAME}"\n')


def fetch(source: dict, repo_root: Path, download, force: bool = False) -> None:
    out_dir = (repo_root / source["target"]).parent
    header, values = out_dir / HEADER_NAME, out_dir / VALUES_NAME
    if header.exists() and values.exists() and not force:
        print("  already present")
        return
    product = source["url"].rsplit("/", 1)[-1][: -len(".img")].upper()
    label = parse_label(_get(_label_url(source["url"])).decode("latin-1"))
    text = header_text(label, product)  # check the layout before the 88 MB download
    download(source["url"], values)
    expected = 2 * int(label["LINES"]) * int(label["LINE_SAMPLES"])
    if values.stat().st_size != expected:
        values.unlink()
        raise RuntimeError(f"{source['url']}: wrong size, expected {expected} bytes")
    header.write_text(text, encoding="utf-8")
    print(f"  {product} {int(label['LINES'])} x {int(label['LINE_SAMPLES'])} -> {values}")
