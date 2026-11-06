# Task 16 fix round 1

- I1: terrain-rendering.md Limits line corrected (Earth terrain receives shadows, casts none); ansichten.md new German entry "Schatten Erde" after "Mondschatten nah". NOT done: docs/subsystems/moon-surface.md:616 (Task 17 owns it; the one-word fix "(the Earth: unchanged)" -> Earth now has its own near shadows is left to it).
- I2: shadow atlas VRAM (8192 x 8192 x 4 B = 256 MB, shared with the Moon's, never both active) recorded in vr-performance.md as COMPUTED (no measurement: render_check has no VRAM print and a GPU code change was out of scope); code comment on ATLAS_SIZE. 4096 / 16-bit atlas not tried, stated.
- Minors: M1 (soft row = plausibility check, 734->716 m explanation) in terrain-rendering.md Validation and test header; M2 rerun command documented; M3 eye-over-tower-centre documented. Left: M4 (not measured, note only), M5 (already in known-issues.md), M1's missing 31.1 deg soft row in the report (the doc table already has it).
- Tests: earth_near_shadow_test PASS; smoke_test PASSED (only comments changed in code).
