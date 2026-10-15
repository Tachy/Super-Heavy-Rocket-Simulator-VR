# Runtime data (git-ignored)

Everything in this folder is downloaded by `python tools/data_pipeline/fetch.py`
from the sources (with licences) listed in `tools/data_pipeline/sources.toml`.

- `spice/` - NAIF SPICE kernels: leap seconds, DE440 ephemeris, Earth (ITRF93) and Moon orientation, GM values.
- `terrain/` - LOLA elevation: the global grid LDEM_16 and 20 m tiles (LDEM_80S_20M crops) around the sites of
  `config/lunar_sites.toml` (lunar l2).
