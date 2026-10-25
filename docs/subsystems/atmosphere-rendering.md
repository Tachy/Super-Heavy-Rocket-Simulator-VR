# Atmosphere rendering (M8) - index

History and the reasons behind superseded decisions: `docs/history/atmosphere-rendering.md`.

- `docs/subsystems/atmosphere-scattering.md`: Bruneton tables and cache, CPU / shader evaluation, WGS84 mapping, sky,
  surfaces, vehicles, Sun light, exposure; code map and core validation.
- `docs/subsystems/eye-adaptation.md`: cone gain and pacing, eye adaptation away from the Earth, earthshine, night
  vision (rods, brightness / colourfulness, city lights, glow), Moon regolith and moonlight.
- `docs/subsystems/atmosphere-eclipse-refraction.md`: star visibility, solar-eclipse sky, astronomical refraction.
- `docs/subsystems/atmosphere-performance.md`: GPU check (`atmosphere_check.gd`), render values, performance runs,
  diagnostics, draw-stall and eclipse-cost open points.
