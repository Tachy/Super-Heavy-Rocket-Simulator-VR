# Subsystems - index

Split on 2026-10-25 (the per-task context is kept small). Each `docs/subsystems/<file>.md` holds a subsystem's CURRENT
state: model, code, standing decisions, validation, diagnostics, open points. The verbatim notes as they stood before
the split - reasons behind decisions, superseded numbers, earlier approaches - are in `docs/history/<slug>.md`.
Older code comments refer to sections of this file by their former titles; the table maps them.

| Former section | Current state | History |
|---|---|---|
| Vehicle data, ascent and booster recovery | `subsystems/vehicle-data.md` | `history/vehicle-data.md` |
| Guidance, vehicles and missions (M5 onwards) | `subsystems/guidance-missions.md` (index) -> `gnc-core.md`, `ship-entry-return.md`, `falcon9.md`, `crew-dragon.md`, `lunar.md` | `history/guidance-missions.md` |
| ISS rendezvous and docking (milestone i) | `subsystems/iss.md` | `history/iss.md` |
| Orbital refuelling (milestone r) | `subsystems/refuelling.md` | `history/refuelling.md` |
| Crew escape (ejection seats on the crewed ship return) | `subsystems/crew-escape.md` | `history/crew-escape.md` |
| Mission timeline | `subsystems/mission-timeline.md` | `history/mission-timeline.md` |
| Ascent trajectory and staging | `subsystems/ascent-staging.md` | `history/ascent-staging.md` |
| Earth terrain and geoid (M8 Plan A) | `subsystems/earth-terrain.md` | `history/earth-terrain.md` |
| Terrain rendering (M8 Plan B) | `subsystems/terrain-rendering.md` | `history/terrain-rendering.md` |
| Atmosphere rendering (M8) | `subsystems/atmosphere-rendering.md` (index) -> `atmosphere-scattering.md`, `eye-adaptation.md`, `atmosphere-eclipse-refraction.md`, `atmosphere-performance.md` | `history/atmosphere-rendering.md` |
| Sea state and deck motion | `subsystems/sea-state.md` | `history/sea-state.md` |
| Weather clouds (M8 weather and clouds) | `subsystems/weather-clouds.md` (field, optics, light, far field, performance, known limits), `subsystems/weather-service.md` (sources, downloads, cache, UI line) | `history/weather-clouds.md` |
| VR frame budget, perf log, main-thread sections | `subsystems/vr-performance.md` | - |
| Startup loading screen (desktop layer, VR quad layer, boot steps) | `subsystems/loading-screen.md` | `history/loading-screen.md` |

New subsystems get their own `docs/subsystems/<slug>.md`.
