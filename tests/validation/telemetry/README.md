# Webcast telemetry tables

Speed and altitude of booster and ship as shown in SpaceX's Starship webcasts, transcribed by
`tools/telemetry/webcast_telemetry.py` (template OCR of the telemetry strip, 2 samples/s; Flight 5 after
T+8:40 at 1 sample/s). Sources and licence notes: `tools/data_pipeline/sources.toml`.

| File | Flight | Vehicles | Layout | Coverage |
|---|---|---|---|---|
| `flight4_2024-06-06.csv` | Flight 4 | Block 1: B11 / S29 | strip | T-10 s to T+10 min |
| `flight5_2024-10-13.csv` | Flight 5 | Block 1: B12 (caught) / S30 | strip | T-10 s to ship splashdown (~T+65 min) |
| `flight7_2025-01-16.csv` | Flight 7 | Block 2: B14-1 (caught) / S33 | strip | T-10 s to T+10 min (ship lost ~T+8:30) |
| `flight8_2025-03-06.csv` | Flight 8 | Block 2: B15-1 (caught) / S34 | strip | T-10 s to T+10 min (ship lost ~T+9:30) |
| `flight10_2025-08-26.csv` | Flight 10 | Block 2: B16 / S37 | dials | T-15 s to T+10 min |
| `flight12_2026-05-22.csv` | Flight 12 | Block 3: B19 / S39 | dials | T-15 s to T+10 min |
| `flight13_2026-07-24.csv` | Flight 13 | Block 3: B20 / S40 | dials | T-15 s to T+10 min |
| `flight14_2026-09-28.csv` | Flight 14 | Block 3: B21 / S41 | dials | T-15 s to T+10 min |

Layouts: "strip" (flights up to 9: text strip, altitude in whole km), "dials" (flight 10 on: round dials,
altitude to 0.1 km, propellant gauge arc and engine icons; the booster's speed dial gives way to its engine icon
before the landing burn).

`flights.toml`: which vehicles (serials, block, Raptor generation) flew on each flight test, outcomes and what
to watch for when evaluating a flight, with sources. Only flights with a documented configuration are evaluated.

Columns: `t_s` (time after liftoff from the mission clock, +-0.25 s), `booster_speed_kmh`,
`booster_altitude_km`, `ship_speed_kmh`, `ship_altitude_km`, and (dials layout only) `booster_propellant_pct`,
`booster_engines_lit`, `ship_propellant_pct`, `ship_engines_lit` (gauge fill in % of the arc, lit engine dots).
Empty cells: nothing valid was read (field hidden, or a reading rejected by the outlier test).

Validation: Flight 5 readings match screenshots taken by hand (T+6:08 to T+6:30) exactly; two independent
scans agree reading for reading.

Caveats: the displayed speed is presumably relative to the ground; altitude is rounded to whole km (derive
vertical speed over several seconds). Rebuild a table:

```
python tools/telemetry/webcast_telemetry.py locate <video>
python tools/telemetry/webcast_telemetry.py scan <video> <liftoff_video_s - 10> 610 raw.csv --fps 2
python tools/telemetry/webcast_telemetry.py clean raw.csv <table.csv> --flight "<name>" --url <webcast>
```

Drag of the falling booster (ballistic coefficient, C_D for an assumed mass):
`python tools/telemetry/drag_estimate.py <table.csv> --from 340 --to 392`.
Masses and thrust from the propellant gauge (dials layout; absolute scale from a sourced tank capacity):
`python tools/telemetry/mass_estimate.py <table.csv> --capacity-t 3650 --isp-sl 330 --isp-vac 350 --from 20 --to 130`
(stack/booster) or `--vehicle ship --capacity-t 1600 --isp-sl 365 --isp-vac 365 --cd 0 --from 200 --to 500`.
New layout scans: `scan ... --layout dials` (1920x1080).
