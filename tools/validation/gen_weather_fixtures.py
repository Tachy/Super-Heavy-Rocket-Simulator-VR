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
  era5_columns.toml             six ERA5 columns at one hour (the weather snapshot tests): all 37 levels of
                                temperature, the three cloud fields and the store's geopotential, plus surface
                                pressure, surface geopotential and total column water vapour
  gfs_column.toml               one GFS column (HGT / TMP on the 22 levels 1000..50 mb + HGT surface), cfgrib-decoded
  era5_plausibility.toml        ERA5 columns of two hours (2024-04-08 18 UTC, 2025-08-15 18 UTC) with the store's own
                                single-level total_cloud_cover and cloud_base_height beside them: the real-date
                                plausibility test of the snapshot's column cover and cloud base

Offline tool only, not part of the build (references are TOML: the tests read them with toml++):
    python -m pip install numpy numcodecs xarray cfgrib eccodes
    python -I tools/validation/gen_weather_fixtures.py
Downloads (six whole ERA5 chunks, ~0.5 GB; ~25 MB GFS records) are cached in data/raw/weather_fixtures/.
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

# Weather snapshot columns (name, row, column on the 0.25 deg grid: lat = 90 - row / 4, lon = column / 4).
ERA5_PROFILE_COLUMNS = [
    ("tropics", 340, 600),            # 5.0 N, 150.0 E (western Pacific warm pool)
    ("midlatitude_ocean", 180, 1320),  # 45.0 N, 330.0 E (North Atlantic)
    ("starbase", 256, 1051),          # 26.0 N, 262.75 E
    ("cape", 246, 1118),              # 28.5 N, 279.5 E
    ("himalaya", 240, 340),           # 30.0 N, 85.0 E
    ("antarctic_plateau", 660, 493),  # 75.0 S, 123.25 E (Dome C)
]
ERA5_COLUMN_VARIABLES = [  # (store variable, TOML key)
    ("temperature", "temperature_k"),
    ("fraction_of_cloud_cover", "cloud_fraction"),
    ("specific_cloud_liquid_water_content", "liquid_kg_kg"),
    ("specific_cloud_ice_water_content", "ice_kg_kg"),
    ("geopotential", "geopotential_m2_s2"),
]
GFS_COLUMN_ROW, GFS_COLUMN_COLUMN = 201, 1020  # 39.75 N, 255.0 E (Denver: the 850 mb level is below the ground)
GFS_LEVELS_MB = [1000, 975, 950, 925, 900, 850, 800, 750, 700, 650, 600,
                 550, 500, 450, 400, 350, 300, 250, 200, 150, 100, 50]


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
    # block table, a split block and an unsplit leftover block as in the store's 512 kB-block chunks.
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


def toml_list(values):
    return "[" + ", ".join(toml_float(v) for v in values) + "]"


def era5_chunk(variable, four_dimensional, time_index=ERA5_TIME_INDEX):
    key = f"{time_index}.0.0.0" if four_dimensional else f"{time_index}.0.0"
    name = "era5_cc_chunk.bin" if variable == "fraction_of_cloud_cover" else f"era5_{variable}_chunk.bin"
    if time_index != ERA5_TIME_INDEX:
        name = f"{time_index}_{name}"
    shape = (37, 721, 1440) if four_dimensional else (721, 1440)
    data = fetch(f"{ERA5_STORE}/{variable}/{key}", name)
    return np.frombuffer(numcodecs.Blosc().decode(data), "<f4").reshape(shape)


def write_era5_columns():
    levels_hpa = np.frombuffer(numcodecs.Blosc().decode(fetch(f"{ERA5_STORE}/level/0", "era5_level_chunk.bin")), "<i8")
    hour = datetime.datetime(1900, 1, 1) + datetime.timedelta(hours=ERA5_TIME_INDEX)
    lines = [
        "# Six ERA5 columns at one hour (gen_weather_fixtures.py): the weather snapshot tests. Level arrays follow",
        "# levels_hpa (the store's order, 1 .. 1000 hPa); values are the store's float32 as float64.",
        f'store = "{ERA5_STORE}"',
        f"time_index = {ERA5_TIME_INDEX}",
        f'time_utc = "{hour.isoformat()}Z"',
        f"levels_hpa = [{', '.join(str(int(v)) for v in levels_hpa)}]",
        "",
    ]
    columns = [{"name": name, "row": row, "column": column} for name, row, column in ERA5_PROFILE_COLUMNS]
    for variable, key in ERA5_COLUMN_VARIABLES:
        field = era5_chunk(variable, True)
        for c in columns:
            c[key] = field[:, c["row"], c["column"]]
        del field
    for variable, key in (("surface_pressure", "surface_pressure_pa"),
                          ("geopotential_at_surface", "surface_geopotential_m2_s2"),
                          ("total_column_water_vapour", "total_column_water_vapour_kg_m2")):
        field = era5_chunk(variable, False)
        for c in columns:
            c[key] = field[c["row"], c["column"]]
    for c in columns:
        lines += ["[[column]]", f'name = "{c["name"]}"', f"row = {c['row']}", f"column = {c['column']}",
                  f"latitude_deg = {90.0 - 0.25 * c['row']}", f"longitude_deg = {0.25 * c['column']}",
                  f"surface_pressure_pa = {toml_float(c['surface_pressure_pa'])}",
                  f"surface_geopotential_m2_s2 = {toml_float(c['surface_geopotential_m2_s2'])}"]
        lines.append(f"total_column_water_vapour_kg_m2 = {toml_float(c['total_column_water_vapour_kg_m2'])}")
        lines += [f"{key} = {toml_list(c[key])}" for _, key in ERA5_COLUMN_VARIABLES]
        lines.append("")
        print(f"  column {c['name']}: p_s {float(c['surface_pressure_pa']):.0f} Pa, "
              f"z_s {float(c['surface_geopotential_m2_s2']) / 9.80665:.0f} gpm")
    (OUT / "era5_columns.toml").write_text("\n".join(lines), encoding="utf-8")


# Real-date plausibility: (hour since 1900-01-01, label). 2024-04-08 18 UTC is the eclipse hour of the visual set, 2025-08-15
# 18 UTC the Starbase overcast hour (docs/subsystems/weather-clouds.md "Visual set").
PLAUSIBILITY_HOURS = [(1089330, "2024-04-08 18 UTC"), (1101186, "2025-08-15 18 UTC")]
PLAUSIBILITY_CELLS = ["starbase", "cape", "midlatitude_ocean", "tropics"]  # of ERA5_PROFILE_COLUMNS
OVERCAST_WINDOW = 20  # cells (5 deg) around Starbase: the cell with the largest total_cloud_cover is added


def write_era5_plausibility():
    levels_hpa = np.frombuffer(numcodecs.Blosc().decode(fetch(f"{ERA5_STORE}/level/0", "era5_level_chunk.bin")), "<i8")
    lines = [
        "# ERA5 columns of two hours with the store's single-level total_cloud_cover (tcc) and cloud_base_height (cbh, m",
        "# above the ground, nan without cloud) and the 2 m temperature / dew point (the lifting condensation level) beside them",
        "# (gen_weather_fixtures.py): the real-date plausibility test.",
        f'store = "{ERA5_STORE}"',
        f"levels_hpa = [{', '.join(str(int(v)) for v in levels_hpa)}]",
        "",
    ]
    ground = era5_chunk("geopotential_at_surface", False)  # constant in time: any hour
    named = {name: (row, column) for name, row, column in ERA5_PROFILE_COLUMNS}
    for time_index, label in PLAUSIBILITY_HOURS:
        hour = datetime.datetime(1900, 1, 1) + datetime.timedelta(hours=time_index)
        tcc = era5_chunk("total_cloud_cover", False, time_index)
        cbh = era5_chunk("cloud_base_height", False, time_index)
        sp = era5_chunk("surface_pressure", False, time_index)
        tcwv = era5_chunk("total_column_water_vapour", False, time_index)
        t2m = era5_chunk("2m_temperature", False, time_index)
        d2m = era5_chunk("2m_dewpoint_temperature", False, time_index)
        row0, column0 = named["starbase"]
        window = tcc[row0 - OVERCAST_WINDOW:row0 + OVERCAST_WINDOW, column0 - OVERCAST_WINDOW:column0 + OVERCAST_WINDOW]
        wr, wc = np.unravel_index(int(np.argmax(window)), window.shape)
        cells = [(name, *named[name]) for name in PLAUSIBILITY_CELLS]
        cells.append(("starbase_window_overcast", row0 - OVERCAST_WINDOW + int(wr), column0 - OVERCAST_WINDOW + int(wc)))
        fields = {key: era5_chunk(variable, True, time_index) for variable, key in ERA5_COLUMN_VARIABLES
                  if variable != "geopotential"}
        for name, row, column in cells:
            lines += ["[[column]]", f'name = "{name}"', f'time_utc = "{hour.isoformat()}Z"', f"time_index = {time_index}",
                      f"row = {row}", f"column = {column}", f"latitude_deg = {90.0 - 0.25 * row}",
                      f"longitude_deg = {0.25 * column}",
                      f"surface_pressure_pa = {toml_float(sp[row, column])}",
                      f"surface_geopotential_m2_s2 = {toml_float(ground[row, column])}",
                      f"total_column_water_vapour_kg_m2 = {toml_float(tcwv[row, column])}",
                      f"total_cloud_cover = {toml_float(tcc[row, column])}",
                      f"cloud_base_height_m = {toml_float(cbh[row, column])}",
                      f"temperature_2m_k = {toml_float(t2m[row, column])}",
                      f"dewpoint_2m_k = {toml_float(d2m[row, column])}"]
            lines += [f"{key} = {toml_list(fields[key][:, row, column])}" for _, key in ERA5_COLUMN_VARIABLES
                      if key != "geopotential_m2_s2"]
            lines.append("")
            print(f"  {label} {name}: tcc {float(tcc[row, column]):.2f}, cbh {float(cbh[row, column]):.0f} m")
        del fields
    (OUT / "era5_plausibility.toml").write_text("\n".join(lines), encoding="utf-8")


def write_gfs_column():
    index = fetch(GFS_FILE + ".idx", "gfs_f006.idx").decode("ascii").splitlines()
    entries = [line.split(":") for line in index]

    def value(variable, level):
        i = next(k for k, e in enumerate(entries) if e[3] == variable and e[4] == level)
        start, end = int(entries[i][1]), int(entries[i + 1][1]) - 1
        name = f"gfs_{variable.lower()}_{level.replace(' ', '')}.grb2"
        fetch(GFS_FILE, name, (start, end))
        dataset = xr.open_dataset(CACHE / name, engine="cfgrib", backend_kwargs={"indexpath": ""})
        v = float(next(iter(dataset.data_vars.values())).values[GFS_COLUMN_ROW, GFS_COLUMN_COLUMN])
        dataset.close()
        return v

    lines = [
        f"# One GFS column (gen_weather_fixtures.py), values decoded by cfgrib at grid row {GFS_COLUMN_ROW}, column "
        f"{GFS_COLUMN_COLUMN}",
        f'source = "{GFS_FILE}"',
        f"row = {GFS_COLUMN_ROW}",
        f"column = {GFS_COLUMN_COLUMN}",
        f"latitude_deg = {90.0 - 0.25 * GFS_COLUMN_ROW}",
        f"longitude_deg = {0.25 * GFS_COLUMN_COLUMN}",
        f"levels_mb = [{', '.join(str(v) for v in GFS_LEVELS_MB)}]",
        f"surface_height_m = {toml_float(value('HGT', 'surface'))}",
        f"geopotential_height_m = {toml_list(value('HGT', f'{lvl} mb') for lvl in GFS_LEVELS_MB)}",
        f"temperature_k = {toml_list(value('TMP', f'{lvl} mb') for lvl in GFS_LEVELS_MB)}",
    ]
    (OUT / "gfs_column.toml").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    write_era5()
    write_gfs()
    write_era5_columns()
    write_gfs_column()
    write_era5_plausibility()


if __name__ == "__main__":
    main()
