# Weather service (M8 weather and clouds, Task 7)

`game/world/weather/weather_service.gd` (`WeatherService`, owned by `World.weather`) chooses, downloads, builds, caches
and loads the weather snapshots the clouds are drawn from. Physics, decoding, the bracket and the source choice are the
core's (`StarsimCore.weather_*`); the service is orchestration only. `weather_fetcher.gd` wraps `HTTPRequest` (Range,
`.part` + rename, injectable for tests). Requests run on HTTP threads with no cap on the whole request (ERA5 chunks are
tens of MB); a transfer is given up only after `STALL_TIMEOUT_S` (30 s) without a new byte. A ranged request is accepted
only with HTTP 206 (a 200 full-file answer is rejected), an unranged one only with 200.

- **Never blocks**: `update(epoch_tdb_s, warp)` runs per frame; downloads are HTTPRequest nodes, builds and loads run on
  the `WorkerThreadPool`. `weather_set_active` is cache-only: it never reads a file, a file outside the core's 3-entry
  load cache (least recently used out, a hit becomes the newest) answers `ok = false` ("Wetterdatei nicht geladen") and
  leaves the active pair; the owner loads the file again on a worker (`reload_into_core_cache`, `_push_blend`).
- **Sources**: on the first update the ERA5 `.zattrs` and the newest GFS cycle (`.idx` of up to 4 candidates) are probed; an
  unanswered probe is repeated every `retry_after_s` (60 s), so a session started offline gets ERA5 / GFS once online.
  `weather_bracket` returns the two snapshots; each missing one is a job (requests -> build -> `data/weather/cache/<key>.wthr`).
  ERA5 hour = 6 timed files + the static `era5_geopotential_at_surface.blosc` (downloaded once, kept in `cache/raw`).
  Raw files are deleted after a build; a failed or damaged build deletes its output.
- **Analog**: the shipped `data/weather/analog` 1-degree file of the epoch's month and nearest 6-hour slot is the
  baseline at once (nothing active yet, or the wanted snapshots failed); online the full-resolution ERA5 hour of the same
  calendar day in 2025 (the bracket's analog id) replaces it. Label date from the id fields, not the file name.
- **Warp**: step from `weather_snapshot_step(warp, seconds_per_download)` (measured, smoothed); step 0 = hold: no new
  requests, the last pair stays, line "Wetter: Stand HH:00 UTC". Dropping the warp requeues the current bracket.
- **Failures**: a failed snapshot is retried after 60 s (a snapshot file that cannot be loaded is deleted and rebuilt then); meanwhile the last pair or the analog flies ("Wetter: typisch
  (Analogtag ...)"). A time jump drops queued downloads of the old time (the running job finishes).
- **No data**: while nothing is active and the shipped analog is missing / unreadable and no wanted snapshot is still
  downloading, the line reads "Wetter: aus (keine Daten)" instead of "Wetter: wird geladen ...".
- **Cache**: 4 GB cap (about 30 full-resolution snapshots of 131 MB); the oldest built files go first (modification
  time = build time; files in use are not re-touched), active and wanted files are protected.
- **UI**: `top_bar.gd` shows the line under the local time, `time_panel.gd` has the "Wetter" check box (persisted in
  `user://settings.cfg`, section `weather`, key `enabled`, default on). `STARSIM_DEBUG_WEATHER=1` prints size and time of
  every built snapshot. `render_check.gd` keeps weather off unless `--weather[=max seconds]`.
- **Measured** (2026-10-25, 24-core desktop, fast line): one ERA5 hour 148-150 MB downloaded, 130.7 MB built, 5.6-5.9 s.
- The clouds cross-fade every pair or source change themselves (`CloudLayer`, `weather-clouds.md`). The CPU light follows
  that visible state: with `light_driven_by_layer` (set by the world) `CloudLayer` is the only writer of the core's
  active pair (slot files and weight, pushed on a slot change or a weight move of 0.01; an empty slot is "" = the empty
  snapshot, blended toward zero depths) and the service sets nothing. Without a layer (headless tests) the service sets
  the pair itself at the switch and on blend moves. Test: `game/tests/weather_service_test.gd`.
- **RAM**: a loaded snapshot dictionary holds the three texel arrays (field 133 + ice 33 + column 66 = 232 MB at
  0.25 deg); the core also caches the `WeatherSnapshot` of the last three files (~130 MB each). Only the target pair's dictionaries are
  kept (`_loaded`); `CloudLayer` erases the `cloud_*_texels` keys once a slot's upload is complete (a `CloudFieldTexture`
  drops its copy when done), the service erases them after announcing when no layer listens, and a result that arrives
  with the weather off or after the target moved on is dropped. A dictionary without texels counts as not loaded: a pair
  that needs it again reloads it on a worker (the core cache makes that a conversion, no disk read). Steady state:
  the core cache (3 x ~130 MB) + the pair's metadata (before: two more dictionaries, ~464 MB); during an upload the
  uploading snapshot's 232 MB on top, freed when its slot is complete.
