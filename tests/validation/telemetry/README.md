# Webcast telemetry tables

Speed and altitude of booster and ship as shown in SpaceX's Starship webcasts, transcribed by
`tools/telemetry/webcast_telemetry.py` (template OCR of the telemetry strip, 2 samples/s; Flight 5 after
T+8:40 at 1 sample/s). Sources and licence notes: `tools/data_pipeline/sources.toml`.

| File | Flight | Vehicles | Coverage |
|---|---|---|---|
| `flight5_2024-10-13.csv` | Flight 5 | Block 1: B12 (caught) / S30 | T-10 s to ship splashdown (~T+65 min) |
| `flight7_2025-01-16.csv` | Flight 7 | Block 2: B14-1 (caught) / S33 | T-10 s to T+10 min (ship lost ~T+8:30) |
| `flight8_2025-03-06.csv` | Flight 8 | Block 2: B15-1 (caught) / S34 | T-10 s to T+10 min (ship lost ~T+9:30) |

`flights.toml`: which vehicles (serials, block, Raptor generation) flew on each flight test, outcomes and what
to watch for when evaluating a flight, with sources. Only flights with a documented configuration are evaluated.

Columns: `t_s` (time after liftoff from the mission clock, +-0.25 s), `booster_speed_kmh`,
`booster_altitude_km`, `ship_speed_kmh`, `ship_altitude_km`. Empty cells: nothing valid was read (field hidden,
or a reading rejected by the outlier test). Resolution as displayed: 1 km/h and 1 km.

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
