# Webcast telemetry tables

Speed and altitude of booster and ship as shown in SpaceX's Starship webcasts, transcribed by
`tools/telemetry/webcast_telemetry.py` (template OCR of the telemetry strip, 2 samples/s; Flight 5 after
T+8:40 at 1 sample/s). Sources and licence notes: `tools/data_pipeline/sources.toml`.

| File | Flight | Vehicles | Coverage |
|---|---|---|---|
| `flight12_2026-05-22.csv` | Flight 12 | Block 3: B19 / S39 | T-15 s to T+66 min: ship entry to 57 km, last 1.5 km to splashdown |
| `flight13_2026-07-24.csv` | Flight 13 | Block 3: B20 / S40 | T-15 s to T+65 min: ship coast, entry and splashdown |
| `flight14_2026-09-28.csv` | Flight 14 | Block 3: B21 / S41 | T-15 s to T+3 h 9 min: ship orbit (gaps), entry and splashdown |

Sampling: 2 frames/s to T+10 min; the ship's coast and entry after that at 1 frame/s (flight 14's entry at
0.5 frame/s). Raw scans of several runs are merged by video time (sorted, duplicates dropped) before `clean`.

Only the current configuration (Block 3) is kept (project decision 2026-10-13); earlier flights were evaluated
once and removed with their vehicle configurations.

Layout: round dials (flight 10 on): altitude to 0.1 km, propellant gauge arcs and engine icons; the booster's
speed dial gives way to its engine icon before the landing burn. (The tool still reads the older text-strip
layout, `--layout strip`.)

`flights.toml`: which vehicles (serials, block, Raptor generation) flew on each flight test, outcomes and what
to watch for when evaluating a flight, with sources. Only flights with a documented configuration are evaluated.

Columns: `t_s` (time after liftoff from the mission clock, +-0.25 s), `booster_speed_kmh`,
`booster_altitude_km`, `ship_speed_kmh`, `ship_altitude_km`, and (dials layout only) `booster_propellant_pct`,
`booster_engines_lit`, `ship_propellant_pct`, `ship_engines_lit` (gauge fill in % of the arc, lit engine dots).
Empty cells: nothing valid was read (field hidden, or a reading rejected by the outlier test).

Validation: the reader matched hand-taken screenshots exactly on an earlier flight; two independent scans agree
reading for reading; flight 14's gauge readings were checked against unwrapped gauge images.

Caveats: the displayed speed is presumably relative to the ground; altitude is rounded to whole km (derive
vertical speed over several seconds). Rebuild a table:

```
python tools/telemetry/webcast_telemetry.py locate <video>
python tools/telemetry/webcast_telemetry.py scan <video> <liftoff_video_s - 15> 615 raw.csv --fps 2 --layout dials
python tools/telemetry/webcast_telemetry.py clean raw.csv <table.csv> --flight "<name>" --url <webcast>
```

Drag of the falling booster (ballistic coefficient, C_D for an assumed mass):
`python tools/telemetry/drag_estimate.py <table.csv> --from 340 --to 392`.
Masses and thrust from the propellant gauge (dials layout; absolute scale from a sourced tank capacity):
`python tools/telemetry/mass_estimate.py <table.csv> --capacity-t 3650 --isp-sl 330 --isp-vac 350 --from 20 --to 130`
(stack/booster) or `--vehicle ship --capacity-t 1600 --isp-sl 365 --isp-vac 365 --cd 0 --from 200 --to 500`.
New layout scans: `scan ... --layout dials` (1920x1080).
