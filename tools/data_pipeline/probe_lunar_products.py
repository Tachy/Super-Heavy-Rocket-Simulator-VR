"""Research probe for the lunar terrain / albedo products of config/lunar_terrain_products.toml.

For every candidate product: fetch the label (PDS3 .lbl, PDS4 .xml or the PDS3 header attached to the image), print
lines / samples / sample type / scale / projection, and time ranged reads of 64 rows x 2,500 samples (sequential and
8 in parallel) plus a bulk read of full rows. From the measured per-row rate it estimates the download time of a
150 km x 150 km tile (rows as separate ranges). Also prints the licence evidence (NASA media usage guidelines, the PDS
Geosciences copyright page link) and the photometric-normalisation text of the albedo labels.

Network use is small (labels, ranged reads of a few MB); no product is downloaded whole.
Usage: python tools/data_pipeline/probe_lunar_products.py [--only id,id] [--licence] [--orientation]
"""

from __future__ import annotations

import argparse
import math
import re
import statistics
import struct
import sys
import time
import urllib.error
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lunar_tile import SPHERE_RADIUS_M, _get, _label_url, parse_label  # noqa: E402

PDS_GEO = "https://pds-geosciences.wustl.edu/lro/lro-l-lola-3-rdr-v1/lrolol_1xxx/data"
LROC = ("https://pds.mcp.nasa.gov/data/store/img/lunar_reconnaissance_orbiter/pds4/lroc/lro-l-lroc-5-rdr/"
        "LROLRC_2001/DATA/MDR/WAC_EMP")

TILE_ROWS = 64
TILE_SAMPLES = 2500
PARALLEL = 8
TILE_HALF_M = 75000.0

# id -> (img url, label kind). "pds3" = <name>.lbl next to the image, "pds4" = <name>.xml (+ attached PDS3 header).
CANDIDATES = {
    "sldem2015_512_30n_60n_000_045": (f"{PDS_GEO}/sldem2015/tiles/float_img/sldem2015_512_30n_60n_000_045_float.img", "pds3"),
    "sldem2015_512_00n_30n_000_045": (f"{PDS_GEO}/sldem2015/tiles/float_img/sldem2015_512_00n_30n_000_045_float.img", "pds3"),
    "ldem_64": (f"{PDS_GEO}/lola_gdr/cylindrical/img/ldem_64.img", "pds3"),
    "ldem_80s_20m": (f"{PDS_GEO}/lola_gdr/polar/img/ldem_80s_20m.img", "pds3"),
    "ldem_80n_20m": (f"{PDS_GEO}/lola_gdr/polar/img/ldem_80n_20m.img", "pds3"),
    "ldem_60s_60m": (f"{PDS_GEO}/lola_gdr/polar/img/ldem_60s_60m.img", "pds3"),
    "ldem_60n_60m": (f"{PDS_GEO}/lola_gdr/polar/img/ldem_60n_60m.img", "pds3"),
    "wac_emp_643nm_e300n0450_304p": (f"{LROC}/WAC_EMP_643NM_E300N0450_304P.IMG", "pds4"),
    "wac_emp_643nm_p900s0000_304p": (f"{LROC}/WAC_EMP_643NM_P900S0000_304P.IMG", "pds4"),
    "wac_emp_643nm_p900n0000_304p": (f"{LROC}/WAC_EMP_643NM_P900N0000_304P.IMG", "pds4"),
}

BYTES = {"LSB_INTEGER16": 2, "PC_REAL32": 4, "IEEE754LSBSingle": 4}

LICENCE_PAGES = [
    ("https://www.nasa.gov/nasa-brand-center/images-and-media/", r"NASA content[^.]{0,300}generally are not subject to copyright[^.]*\."),
    ("https://pds-geosciences.wustl.edu/missions/lro/lola.htm", r"Copyright/Image Use Policy"),
    ("https://pds-geosciences.wustl.edu/about/copyright.htm", r"(?i)(not copyrighted|public domain|copyright)[^.]{0,200}\."),
]


RETRIES = {"count": 0}


def _get_retry(url: str, byte_range: tuple[int, int] | None = None) -> bytes:
    """_get with exponential back-off on HTTP 429 / 503 (the NASA PDS front end rate-limits bursts); counts retries."""
    for attempt in range(7):
        try:
            return _get(url, byte_range)
        except urllib.error.HTTPError as error:
            if error.code not in (429, 503) or attempt == 6:
                raise
            RETRIES["count"] += 1
            time.sleep(2.0 ** attempt)
    raise AssertionError("unreachable")


def parse_pds4(text: str) -> dict:
    """The few PDS4 fields the products need: lines, samples, data type, header / data offset, comment text."""
    def first(pattern: str):
        m = re.search(pattern, text, re.S)
        return m.group(1) if m else None
    axes = re.findall(r"<axis_name>(\w+)</axis_name>\s*<elements>(\d+)</elements>", text)
    return {
        "lines": int(dict(axes)["Line"]), "samples": int(dict(axes)["Sample"]),
        "data_type": first(r"<data_type>(\w+)</data_type>"),
        "header_bytes": int(first(r"<Header>\s*<offset[^>]*>0</offset>\s*<object_length[^>]*>(\d+)<")),
        "image_offset": int(first(r"<Array_2D_Image>.*?<offset[^>]*>(\d+)</offset>")),
        "comment": re.sub(r"\s+", " ", first(r"<comment>\s*(WAC [^<]*)</comment>") or ""),
    }


def describe(pid: str, url: str, kind: str) -> dict:
    """Layout of one product as a dict (the keys the config file stores)."""
    info: dict = {"id": pid, "url": url}
    if kind == "pds3":
        label = parse_label(_get_retry(_label_url(url)).decode("latin-1"))
        info.update(lines=int(label["LINES"]), samples=int(label["LINE_SAMPLES"]),
                    record_bytes=int(label["RECORD_BYTES"]), data_offset=0)
        bits = int(label["SAMPLE_BITS"])
        info["sample_type"] = {("LSB_INTEGER", 16): "int16", ("PC_REAL", 32): "float32", ("LSB_INTEGER", 8): "uint8"}[
            (label["SAMPLE_TYPE"], bits)]
        info["unit"] = label.get("UNIT")
        info["scale"], info["offset"] = label["SCALING_FACTOR"], label["OFFSET"]
        info["projection"] = label["MAP_PROJECTION_TYPE"]
        info["label"] = label
        return info
    x = parse_pds4(_get_retry(url[: -len(".IMG")] + ".xml").decode("utf-8"))
    header = parse_label(_get_retry(url, (0, x["header_bytes"] - 1)).decode("latin-1"))
    info.update(lines=x["lines"], samples=x["samples"], record_bytes=x["header_bytes"],
                data_offset=x["image_offset"], sample_type="float32", label=header, comment=x["comment"])
    info["projection"] = header.get("MAP_PROJECTION_TYPE", "(header has no projection keywords)")
    return info


def row_range(info: dict, line: int, first_sample: int, count: int) -> tuple[int, int]:
    width = BYTES_FOR[info["sample_type"]]
    start = info["data_offset"] + (line * info["samples"] + first_sample) * width
    return start, start + count * width - 1


BYTES_FOR = {"int16": 2, "float32": 4, "uint8": 1}


def time_reads(info: dict) -> dict:
    """Ranged reads of TILE_ROWS rows x TILE_SAMPLES samples from the middle of the file."""
    count = min(TILE_SAMPLES, info["samples"])
    line0 = info["lines"] // 2 - TILE_ROWS // 2
    first = info["samples"] // 2 - count // 2
    ranges = [row_range(info, line0 + k, first, count) for k in range(TILE_ROWS)]
    t = time.perf_counter()
    sequential = [_get_retry(info["url"], r) for r in ranges]
    seq_s = time.perf_counter() - t
    t = time.perf_counter()
    with ThreadPoolExecutor(PARALLEL) as pool:
        parallel = list(pool.map(lambda r: _get_retry(info["url"], r), ranges))
    par_s = time.perf_counter() - t
    if sequential != parallel:
        raise RuntimeError(f"{info['id']}: sequential and parallel reads differ")
    width = BYTES_FOR[info["sample_type"]]
    bulk_rows = 16
    start = info["data_offset"] + line0 * info["samples"] * width
    t = time.perf_counter()
    bulk = _get_retry(info["url"], (start, start + bulk_rows * info["samples"] * width - 1))
    bulk_s = time.perf_counter() - t
    return {"seq_s": seq_s, "par_s": par_s, "bulk_s": bulk_s, "bulk_mb": len(bulk) / 1e6,
            "row_bytes": count * width, "sample_row": sequential[0]}


def pixel_size_m(info: dict) -> float:
    label = info["label"]
    if "MAP_SCALE" in label:
        scale = label["MAP_SCALE"]
        return scale * 1000.0 if "<km" in str(label.get("MAP_SCALE_UNIT", "")) or scale < 1.0 else scale
    return SPHERE_RADIUS_M * math.radians(1.0) / 304.0  # WAC_EMP 304 ppd (PDS4 label: 99.75 m)


def report(pid: str, url: str, kind: str, measure: bool) -> None:
    info = describe(pid, url, kind)
    print(f"\n== {pid}\n  url: {url}")
    print(f"  layout: {info['lines']} lines x {info['samples']} samples, {info['sample_type']}, "
          f"data offset {info['data_offset']} B, record {info['record_bytes']} B, projection {info['projection']}")
    label = info["label"]
    if "scale" in info:
        print(f"  value = DN x {info['scale']} + (offset {info['offset']}), unit {info['unit']}")
    for key in ("MAP_RESOLUTION", "MAP_SCALE", "LINE_PROJECTION_OFFSET", "SAMPLE_PROJECTION_OFFSET",
                "CENTER_LATITUDE", "CENTER_LONGITUDE", "MINIMUM_LATITUDE", "MAXIMUM_LATITUDE"):
        if key in label:
            print(f"  {key} = {label[key]}")
    if "comment" in info:
        print(f"  label text: {info['comment']}")
    if not measure:
        return
    m = time_reads(info)
    size = pixel_size_m(info)
    fmt = {"int16": "h", "float32": "f", "uint8": "B"}[info["sample_type"]]
    row = struct.unpack(f"<{len(m['sample_row']) // BYTES_FOR[info['sample_type']]}{fmt}", m["sample_row"])
    valid = [v for v in row if v > -1.0e30]
    print(f"  sample row (middle of the file): {len(valid)} / {len(row)} valid, raw min {min(valid):.4g}, "
          f"max {max(valid):.4g}, mean {statistics.fmean(valid):.4g}")
    rows = math.ceil(2 * TILE_HALF_M / size)
    seq_row, par_row = m["seq_s"] / TILE_ROWS, m["par_s"] / TILE_ROWS
    print(f"  {TILE_ROWS} rows x {m['row_bytes'] // BYTES_FOR[info['sample_type']]} samples "
          f"({TILE_ROWS * m['row_bytes'] / 1e6:.2f} MB): sequential {m['seq_s']:.2f} s ({seq_row * 1000:.0f} ms/row), "
          f"{PARALLEL} parallel {m['par_s']:.2f} s ({par_row * 1000:.0f} ms/row)")
    print(f"  bulk 16 full rows: {m['bulk_mb']:.1f} MB in {m['bulk_s']:.2f} s ({m['bulk_mb'] / m['bulk_s']:.1f} MB/s)")
    print(f"  150 km tile ({rows} rows at {size:.1f} m): sequential {rows * seq_row:.0f} s, "
          f"{PARALLEL} parallel {rows * par_row:.0f} s; full-row bulk would move "
          f"{rows * info['samples'] * BYTES_FOR[info['sample_type']] / 1e6:.0f} MB")


def scaling(pid: str) -> None:
    """Row requests per second at 8 / 16 / 32 parallel (64 was answered with HTTP 403 by PDS Geosciences) connections, and the rate of one long contiguous range."""
    url, kind = CANDIDATES[pid]
    info = describe(pid, url, kind)
    count = min(TILE_SAMPLES, info["samples"])
    width = BYTES_FOR[info["sample_type"]]
    rows = 256
    line0 = info["lines"] // 2 - rows // 2
    ranges = [row_range(info, line0 + k, info["samples"] // 2 - count // 2, count) for k in range(rows)]
    print(f"\n== scaling on {pid} ({rows} row ranges of {count * width / 1e3:.1f} kB)")
    for workers in (8, 16, 32):
        t = time.perf_counter()
        with ThreadPoolExecutor(workers) as pool:
            list(pool.map(lambda r: _get_retry(url, r), ranges))
        dt = time.perf_counter() - t
        print(f"  {workers:2d} parallel: {dt:6.2f} s, {rows / dt:6.1f} requests/s, {rows * count * width / dt / 1e6:5.2f} MB/s")
    start = info["data_offset"] + line0 * info["samples"] * width
    for bulk_rows in (64, 256):
        length = bulk_rows * info["samples"] * width
        t = time.perf_counter()
        data = _get_retry(url, (start, start + length - 1))
        dt = time.perf_counter() - t
        print(f"  one range of {bulk_rows} full rows: {len(data) / 1e6:6.1f} MB in {dt:5.2f} s = {len(data) / dt / 1e6:5.2f} MB/s")


def licence_check() -> None:
    print("\n== licence evidence")
    for url, pattern in LICENCE_PAGES:
        try:
            text = _get(url).decode("utf-8", "replace")
        except Exception as error:  # noqa: BLE001 - report and go on
            print(f"  {url}: {error}")
            continue
        text = re.sub(r"\s+", " ", re.sub(r"<[^>]*>", " ", text))
        m = re.search(pattern, text, re.S)
        print(f"  {url}: {m.group(0)[:400] if m else 'no match'}")


# ----------------------------------------------------------------------------------------------- orientation check

def north_polar_orientation() -> None:
    """Which of four sample / line sign conventions of the north polar stereographic product matches LDEM_16?

    rho = 2 R tan(pi/4 - lat/2); candidates: sample = pole + s1 rho sin(lon) / scale, line = pole + s2 rho cos(lon) / scale.
    Reference: LDEM_16 (1.9 km pixels, whole file 33 MB); each point is a 32 x 32 block mean of the product.
    """
    info = describe("ldem_60n_60m", CANDIDATES["ldem_60n_60m"][0], "pds3")
    label = info["label"]
    pole_line, pole_sample, scale = label["LINE_PROJECTION_OFFSET"], label["SAMPLE_PROJECTION_OFFSET"], label["MAP_SCALE"]
    glob = describe("ldem_16", f"{PDS_GEO}/lola_gdr/cylindrical/img/ldem_16.img", "pds3")
    data = _get(glob["url"])
    gl = parse_label(_get(_label_url(glob["url"])).decode("latin-1"))
    ppd, lines, samples = gl["MAP_RESOLUTION"], glob["lines"], glob["samples"]
    zero_line = gl["LINE_PROJECTION_OFFSET"]
    zero_sample = gl["SAMPLE_PROJECTION_OFFSET"] - ppd * gl["CENTER_LONGITUDE"]
    values = struct.unpack(f"<{lines * samples}h", data)

    def ref(lat_deg: float, lon_deg: float) -> float:
        line = round(zero_line - ppd * lat_deg)
        sample = round(zero_sample + ppd * (lon_deg % 360.0)) % samples
        return gl["SCALING_FACTOR"] * values[line * samples + sample]

    block = 24  # 24 x 60 m = 1.4 km, about one LDEM_16 pixel (1.9 km)
    points = [(lat, lon) for lat in (68.0, 76.0) for lon in range(10, 360, 60)]
    errors = {(s1, s2): [] for s1 in (1, -1) for s2 in (1, -1)}

    def block_mean(lat_deg: float, lon_deg: float, s1: int, s2: int) -> float | None:
        rho = 2.0 * SPHERE_RADIUS_M * math.tan(math.pi / 4.0 - math.radians(lat_deg) / 2.0)
        sample = pole_sample + s1 * rho * math.sin(math.radians(lon_deg)) / scale
        line = pole_line + s2 * rho * math.cos(math.radians(lon_deg)) / scale
        l0, s0 = int(line) - block // 2, int(sample) - block // 2
        if l0 < 0 or s0 < 0 or l0 + block > info["lines"] or s0 + block > info["samples"]:
            return None
        rows = [_get_retry(info["url"], row_range(info, l0 + k, s0, block)) for k in range(block)]
        flat = [v for r in rows for v in struct.unpack(f"<{block}h", r)]
        return label["SCALING_FACTOR"] * statistics.fmean(flat)

    jobs = [(lat, lon, key) for (lat, lon) in points for key in errors]
    with ThreadPoolExecutor(16) as pool:
        means = list(pool.map(lambda j: block_mean(j[0], j[1], *j[2]), jobs))
    for (lat, lon, key), v in zip(jobs, means):
        errors[key].append(None if v is None else v - ref(lat, lon))
    for (s1, s2), diffs in errors.items():
        d = [x for x in diffs if x is not None]
        print(f"  sample sign {s1:+d}, line sign {s2:+d}: rms {math.sqrt(sum(x * x for x in d) / len(d)):8.1f} m over {len(d)} points")


def verify_config(config_path: Path, host: str = "") -> int:
    """Compare every file entry of the product config with its label; returns the number of mismatches."""
    import tomllib
    with config_path.open("rb") as f:
        config = tomllib.load(f)
    bad = 0
    for product in config["product"]:
        for entry in product["file"]:
            url = entry["url"]
            if host not in url:
                continue
            kind = "pds4" if url.endswith(".IMG") else "pds3"
            info = describe(product["id"], url, kind)
            label = info["label"]
            expected = {"lines": info["lines"], "samples": info["samples"], "record_bytes": info["record_bytes"],
                        "header_bytes": info["data_offset"]}
            if kind == "pds3":
                expected["record_bytes"] = int(label["RECORD_BYTES"])
            if "pixels_per_degree" in entry:
                expected["pixels_per_degree"] = label["MAP_RESOLUTION"]
                expected["first_latitude_deg"] = label["MAXIMUM_LATITUDE"]
                if "WESTERNMOST_LONGITUDE" in label:
                    expected["first_longitude_deg"] = label["WESTERNMOST_LONGITUDE"]
            else:
                expected["pole_line"] = label["LINE_PROJECTION_OFFSET"]
                expected["pole_sample"] = label["SAMPLE_PROJECTION_OFFSET"]
                expected["scale_m"] = label["MAP_SCALE"]
            problems = [f"{k}: config {entry[k]} label {v}" for k, v in expected.items() if abs(entry[k] - v) > 1e-9]
            if info["sample_type"] != product["sample_type"]:
                problems.append(f"sample_type: config {product['sample_type']} label {info['sample_type']}")
            if product["kind"] == "height":
                scale = info["scale"] * (1000.0 if info["unit"] == "KILOMETER" else 1.0)
                if abs(scale - product["scale"]) > 1e-12 or info["offset"] != SPHERE_RADIUS_M / (1000.0 if info["unit"] == "KILOMETER" else 1.0):
                    problems.append(f"scale / offset: config {product['scale']} label {scale} / {info['offset']}")
            print(f"  {'ok ' if not problems else 'BAD'} {product['id']}: {url.rsplit('/', 1)[-1]} {'; '.join(problems)}")
            bad += len(problems)
    return bad


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--only", help="comma separated candidate ids")
    parser.add_argument("--no-measure", action="store_true", help="labels only")
    parser.add_argument("--scaling", help="comma separated candidate ids: parallelism / bulk-range scaling test")
    parser.add_argument("--verify-config", action="store_true", help="compare config/lunar_terrain_products.toml with the labels")
    parser.add_argument("--verify-host", default="", help="with --verify-config: only files whose URL contains this text")
    parser.add_argument("--licence", action="store_true", help="print the licence evidence")
    parser.add_argument("--orientation", action="store_true", help="check the north polar axis signs against LDEM_16")
    args = parser.parse_args()
    wanted = set(args.only.split(",")) if args.only else set(CANDIDATES)
    for pid, (url, kind) in CANDIDATES.items():
        if pid in wanted:
            report(pid, url, kind, measure=not args.no_measure)
    for pid in (args.scaling.split(",") if args.scaling else []):
        scaling(pid)
    print(f"\n(HTTP 429/503 retries during timing: {RETRIES['count']})")
    if args.verify_config:
        print("\n== config vs labels")
        if verify_config(Path(__file__).resolve().parents[2] / "config" / "lunar_terrain_products.toml", args.verify_host):
            sys.exit("config does not match the labels")
    if args.licence:
        licence_check()
    if args.orientation:
        print("\n== north polar orientation (ldem_60n_60m vs LDEM_16)")
        north_polar_orientation()


if __name__ == "__main__":
    main()
