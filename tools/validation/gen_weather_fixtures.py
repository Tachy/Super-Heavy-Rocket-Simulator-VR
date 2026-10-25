"""Generates the decoder fixtures of the weather_decode module (core/src/weather_decode, M8 weather and clouds).

Writes tests/validation/weather/:
  era5_cc_cutout.bin            a 4-level x 8 x 16 cut-out of a real ERA5 fraction_of_cloud_cover hour, re-encoded
                                with the store's codec: numcodecs Blosc(cname="lz4", clevel=5, shuffle=SHUFFLE);
                                one block, split into byte streams
  era5_cc_cutout_noshuffle.bin  the same values without the shuffle filter
  era5_cc_cutout_values.bin     the raw float32 little-endian values (levels x rows x columns), the bit-exact reference
  era5_cc_blocks.bin            a 4 x 64 x 96 cut-out with 64 kB blocks: a split block + an unsplit leftover block
  era5_cc_blocks_values.bin     its raw values
  era5_cc_reference.toml        where the cut-outs come from, their shapes
  gfs_<var>_<level>.grb2        single real GFS 0.25 deg GRIB2 messages (byte range from the .idx)
  gfs_<var>_<level>_reference.toml  grid metadata, the packing (template, E, D) and the values at 20 grid points,
                                decoded by cfgrib (ecCodes)

Offline tool only, not part of the build (references are TOML: the tests read them with toml++):
    python -m pip install numpy numcodecs xarray cfgrib eccodes
    python -I tools/validation/gen_weather_fixtures.py
Downloads (~28 MB ERA5 chunk, ~0.9 MB GFS) are cached in data/raw/weather_fixtures/.
"""
import datetime
import pathlib
import struct
import urllib.request

import numcodecs
import numpy as np
import xarray as xr

REPO = pathlib.Path(__file__).resolve().parents[2]
OUT = REPO / "tests" / "validation" / "weather"
CACHE = REPO / "data" / "raw" / "weather_fixtures"

ERA5_STORE = "https://storage.googleapis.com/gcp-public-data-arco-era5/ar/full_37-1h-0p25deg-chunk-1.zarr-v3"
ERA5_TIME_INDEX = 1100000          # hours since 1900-01-01: 2025-06-27 08 UTC
ERA5_LEVEL_INDICES = [17, 21, 25, 30]  # 300, 500, 700, 850 hPa
ERA5_ROW0, ERA5_COLUMN0 = 256, 1044    # 26.0 N, 261.0 E (Gulf coast near Starbase)
ERA5_ROWS, ERA5_COLUMNS = 8, 16
ERA5_BLOCKS_ROWS, ERA5_BLOCKS_COLUMNS = 64, 96  # 96 kB: one 64 kB block + a 32 kB leftover

GFS_FILE = "https://noaa-gfs-bdp-pds.s3.amazonaws.com/gfs.20260920/00/atmos/gfs.t00z.pgrb2.0p25.f006"
# (variable, level as in the .idx, fixture name): CLMR 300 mb is the brief's record (nearly all zero, 1.6 kB),
# CLMR 850 mb the "few hundred kB" one with real cloud water, CLMR 50 mb an all-zero field GFS packs with 5.0.
GFS_RECORDS = [
    ("CLMR", "300 mb", "gfs_clmr_300mb"),
    ("CLMR", "850 mb", "gfs_clmr_850mb"),
    ("CLMR", "50 mb", "gfs_clmr_50mb"),
    ("TCDC", "300 mb", "gfs_tcdc_300mb"),
]
POINTS_PER_RECORD = 20


def fetch(url, name, byte_range=None):
    path = CACHE / name
    if path.exists():
        return path.read_bytes()
    CACHE.mkdir(parents=True, exist_ok=True)
    headers = {"User-Agent": "starsim-validation"}
    if byte_range is not None:
        headers["Range"] = f"bytes={byte_range[0]}-{byte_range[1]}"
    print(f"  downloading {url} {byte_range or ''}")
    with urllib.request.urlopen(urllib.request.Request(url, headers=headers)) as response:
        data = response.read()
    path.write_bytes(data)
    return data


def toml_float(value):
    v = float(value)
    if np.isnan(v):
        return "nan"
    return repr(v)  # the float64 of a float32 value: repr round-trips it exactly


def encode(name, codec, values):
    """Encodes the float32 array (typesize 4, as zarr hands it to the codec) and checks the round trip."""
    frame = codec.encode(values)
    assert codec.decode(frame) == values.tobytes()
    (OUT / name).write_bytes(frame)
    _, _, flags, typesize, nbytes, blocksize, _ = struct.unpack("<BBBBIII", frame[:16])
    print(f"  {name}: {len(frame)} B, flags {flags:#04x}, typesize {typesize}, {nbytes} B in blocks of {blocksize}")
    return blocksize


def write_era5():
    chunk = fetch(f"{ERA5_STORE}/fraction_of_cloud_cover/{ERA5_TIME_INDEX}.0.0.0", "era5_cc_chunk.bin")
    levels_chunk = fetch(f"{ERA5_STORE}/level/0", "era5_level_chunk.bin")
    full = np.frombuffer(numcodecs.Blosc().decode(chunk), "<f4").reshape(37, 721, 1440)
    levels_hpa = np.frombuffer(numcodecs.Blosc().decode(levels_chunk), "<i8")

    def cut_out(rows, columns):
        return np.ascontiguousarray(full[ERA5_LEVEL_INDICES, ERA5_ROW0:ERA5_ROW0 + rows,
                                         ERA5_COLUMN0:ERA5_COLUMN0 + columns]).astype("<f4")

    cut = cut_out(ERA5_ROWS, ERA5_COLUMNS)
    store_codec = numcodecs.Blosc(cname="lz4", clevel=5, shuffle=numcodecs.Blosc.SHUFFLE)
    encode("era5_cc_cutout.bin", store_codec, cut)
    encode("era5_cc_cutout_noshuffle.bin",
           numcodecs.Blosc(cname="lz4", clevel=5, shuffle=numcodecs.Blosc.NOSHUFFLE), cut)
    (OUT / "era5_cc_cutout_values.bin").write_bytes(cut.tobytes())
    # c-blosc keeps small buffers in one block (forced block sizes below 64 kB are raised): a larger cut-out gives the
    # block table, two split blocks and an unsplit leftover block as in the store's 512 kB-block chunks.
    big = cut_out(ERA5_BLOCKS_ROWS, ERA5_BLOCKS_COLUMNS)
    blocksize = encode("era5_cc_blocks.bin", numcodecs.Blosc(cname="lz4", clevel=5, shuffle=numcodecs.Blosc.SHUFFLE,
                                                             blocksize=16384), big)
    assert big.nbytes > blocksize and big.nbytes % blocksize != 0
    (OUT / "era5_cc_blocks_values.bin").write_bytes(big.tobytes())

    hour = datetime.datetime(1900, 1, 1) + datetime.timedelta(hours=ERA5_TIME_INDEX)
    lines = [
        "# ERA5 fraction_of_cloud_cover cut-outs (gen_weather_fixtures.py); raw values in era5_cc_*_values.bin",
        f'store = "{ERA5_STORE}"',
        f'chunk = "fraction_of_cloud_cover/{ERA5_TIME_INDEX}.0.0.0"',
        f'time_utc = "{hour.isoformat()}Z"',
        f"levels_hpa = [{', '.join(str(int(levels_hpa[i])) for i in ERA5_LEVEL_INDICES)}]",
        f"first_row = {ERA5_ROW0}",
        f"first_column = {ERA5_COLUMN0}",
        f"levels = {len(ERA5_LEVEL_INDICES)}",
        "",
        "[cutout]  # era5_cc_cutout.bin, era5_cc_cutout_noshuffle.bin",
        f"rows = {ERA5_ROWS}",
        f"columns = {ERA5_COLUMNS}",
        f"value_min = {toml_float(cut.min())}",
        f"value_max = {toml_float(cut.max())}",
        "",
        "[blocks]  # era5_cc_blocks.bin",
        f"rows = {ERA5_BLOCKS_ROWS}",
        f"columns = {ERA5_BLOCKS_COLUMNS}",
        f"blocksize_bytes = {blocksize}",
    ]
    (OUT / "era5_cc_reference.toml").write_text("\n".join(lines) + "\n", encoding="utf-8")


def sign_magnitude16(raw):
    value = struct.unpack(">H", raw)[0]
    return -(value & 0x7FFF) if value & 0x8000 else value


def packing_of(message):
    """Template number, binary scale E, decimal scale D and bit count of section 5."""
    pos = 16
    while pos < len(message) - 4:
        length, number = struct.unpack(">IB", message[pos:pos + 5])
        if number == 5:
            template = struct.unpack(">H", message[pos + 9:pos + 11])[0]
            e = sign_magnitude16(message[pos + 15:pos + 17])
            d = sign_magnitude16(message[pos + 17:pos + 19])
            return template, e, d, message[pos + 19]
        pos += length
    raise ValueError("no section 5")


def choose_points(values, rng):
    """Ten random grid points plus the ten largest values (cloud fields are mostly zero)."""
    flat = values.ravel()
    random_points = rng.choice(flat.size, POINTS_PER_RECORD // 2, replace=False)
    largest = np.argsort(flat, kind="stable")[::-1]
    largest = [i for i in largest if i not in set(random_points)][:POINTS_PER_RECORD // 2]
    return [divmod(int(i), values.shape[1]) for i in list(random_points) + list(largest)]


def write_gfs():
    index = fetch(GFS_FILE + ".idx", "gfs_f006.idx").decode("ascii").splitlines()
    entries = [line.split(":") for line in index]
    rng = np.random.default_rng(20261025)
    for variable, level, name in GFS_RECORDS:
        i = next(k for k, e in enumerate(entries) if e[3] == variable and e[4] == level)
        start, end = int(entries[i][1]), int(entries[i + 1][1]) - 1
        message = fetch(GFS_FILE, f"{name}.grb2", (start, end))
        assert message[:4] == b"GRIB" and message[-4:] == b"7777"
        path = OUT / f"{name}.grb2"
        path.write_bytes(message)
        dataset = xr.open_dataset(path, engine="cfgrib", backend_kwargs={"indexpath": ""})
        field = next(iter(dataset.data_vars.values()))
        values = field.values
        latitudes, longitudes = dataset.latitude.values, dataset.longitude.values
        template, e, d, bits = packing_of(message)
        step = float(longitudes[1] - longitudes[0])
        lines = [
            f"# GFS {variable} {level} (gen_weather_fixtures.py), values decoded by cfgrib {field.dtype}",
            f'source = "{GFS_FILE}"',
            f"byte_range = [{start}, {end}]",
            f'variable = "{variable}"',
            f'level = "{level}"',
            f"rows = {values.shape[0]}",
            f"columns = {values.shape[1]}",
            f"first_latitude_deg = {toml_float(latitudes[0])}",
            f"first_longitude_deg = {toml_float(longitudes[0])}",
            f"step_deg = {toml_float(step)}",
            f"packing_template = {template}",
            f"binary_scale = {e}",
            f"decimal_scale = {d}",
            f"bits_per_value = {bits}",
            "",
        ]
        for row, column in choose_points(values, rng):
            lines += ["[[point]]", f"row = {row}", f"column = {column}",
                      f"value = {toml_float(values[row, column])}", ""]
        (OUT / f"{name}_reference.toml").write_text("\n".join(lines), encoding="utf-8")
        print(f"  {name}: {len(message)} B, template 5.{template}, E {e}, D {d}, {bits} bits, max {values.max()}")
        dataset.close()


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    write_era5()
    write_gfs()


if __name__ == "__main__":
    main()
