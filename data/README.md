# Runtime data (git-ignored)

Everything in this folder is downloaded by `python tools/data_pipeline/fetch.py`
from the sources (with licences) listed in `tools/data_pipeline/sources.toml`, then partly converted by
`python tools/data_pipeline/prepare.py`.

- `spice/` - NAIF SPICE kernels: leap seconds, DE440 ephemeris, Earth (ITRF93) and Moon orientation, GM values.
- `terrain/` - LOLA elevation: the global grid LDEM_16 and 20 m tiles (LDEM_80S_20M crops) around the sites of
  `config/lunar_sites.toml` (lunar l2).
- `raw/` - downloaded originals: Blue Marble / Black Marble Earth, LROC Moon colour map, HYG star catalogue.
- `textures/`, `stars/` - written by `prepare.py` from `raw/` (the game's Earth/Moon textures and star list).
- `webcasts/` - webcast videos for the telemetry transcription (`tools/telemetry/webcast_telemetry.py`); fetched by
  hand (see that tool's header), only the transcribed tables in `tests/validation/telemetry/` are committed.
