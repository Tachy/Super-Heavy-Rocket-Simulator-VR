# Moon Terrain Shadows Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The lunar terrain occludes the Sun and the earthshine (horizon maps per terrain node), and vehicles / the eye
on the Moon take the same terrain horizon for their direct Sun.

**Architecture:** The core computes terrain horizon elevations (great-circle march over the lunar heights, curvature
exact, early-out by a relief bound). The node builder bakes 16 azimuths per grid vertex into a per-node texture;
`terrain_moon.gdshader` turns the horizon in the light's azimuth into the visible fraction of the solar disc. A
single-point query replaces the mean-sphere Sun fraction near the lunar ground.

**Tech Stack:** C++20 core (`environment`), GDExtension bindings, GDScript (TerrainNodeBuilder / TerrainQuadtree on
WorkerThreadPool), Godot spatial shader.

**Spec:** `docs/superpowers/specs/2026-10-30-moon-terrain-shadows-design.md`

## Global Constraints
- CLAUDE.md rules: core pure float64, no Godot; bindings thin; SI units, angles rad, frame in names; constants with
  source tags [S]/[E]/[D]; no magic numbers; small files.
- Never block the main / render thread on worker tasks; node builds stay on WorkerThreadPool.
- Earth terrain and every Earth / cloud path unchanged.
- Process: one GPU run at a time, 300 s timeout; before a GPU run check Get-Process for a Godot you did not start
  (the user may be in VR) and wait (poll 60 s, foreground), never kill it; never `--xr-mode off` for stereo; leave no
  Godot / tail / background job running. Images to `screenshots/moon_shadows/`.
- Tests: only affected tags / scripts (see each task). Commit on main with own paths only
  (`git commit -m ... -- <paths>`), push after each task. Attribution lines at the end of each commit message:
  `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` and
  `Claude-Session: https://claude.ai/code/session_01WYV5w1ScwAoA2gKrXU7S88`.
- Docs: `docs/subsystems/terrain-rendering.md` (and lunar.md where the vehicle Sun changes) current state only;
  superseded text verbatim to `docs/history/<slug>.md`.

## Review Focus
1. Sun just below / just above the local horizon at the terminator: lit peaks stay, sun-facing slopes behind higher
   terrain or the curvature go dark; no flicker or sector banding when the Sun's azimuth crosses a sector boundary.
2. Node seams: neighbouring nodes (and nodes one level apart, skirts, morphing) show no shadow step at their edge.
3. Poles (mons_mouton_im2 site tiles, azimuth frame at lat ~ -84 deg; the pole itself): no NaN, no wrong east/north.
4. Antimeridian and the global grid / site tile blend: the march crosses lon +-180 and the tile edge without jumps.
5. Vehicle on the ground in a crater with the Sun behind the rim: its direct Sun goes to 0 and comes back smoothly as
   the Sun rises; in orbit / far from the ground the old mean-sphere / eclipse fraction is unchanged.

---

### Task 1: Core terrain horizon

**Files:**
- Create: `core/include/starsim/environment/terrain_horizon.hpp`, `core/src/environment/terrain_horizon.cpp`
- Modify: `core/include/starsim/environment/terrain.hpp` (relief bound), `lunar_terrain.hpp/.cpp` (its value),
  core CMake lists
- Test: `core/tests/test_terrain_horizon.cpp` (tag `[terrain_horizon]`)

**Interfaces (produces):**
```cpp
namespace starsim::environment {
// Terrain horizon elevation (rad, + above the local horizontal of the point's sphere direction) seen from a point at
// (latitude_rad, longitude_rad) at height_m above the terrain's reference sphere, along the great circle in
// azimuth_rad (clockwise from north). Samples terrain between min_distance_m and the relief bound's cut-off.
struct HorizonMarch { double min_distance_m; double first_step_m; };
double horizon_elevation_rad(const Terrain& terrain, double sphere_radius_m, double latitude_rad, double longitude_rad,
                             double height_m, double azimuth_rad, const HorizonMarch& march);
// Many points, azimuth_count azimuths k * 2 pi / azimuth_count each; result point-major, azimuth-minor.
std::vector<float> horizon_elevations(const Terrain& terrain, double sphere_radius_m,
                                      const std::vector<double>& latitude_rad, const std::vector<double>& longitude_rad,
                                      const std::vector<double>& height_m, int azimuth_count, const HorizonMarch& march);
}
// Terrain gains: virtual double max_height_m() const  (upper bound of height_m over the body; LunarTerrain: the max
// over its global grid and tiles, computed at construction).
```
- [ ] Step 1: tests first (they fail):
  - smooth sphere (a test Terrain with height 0): horizon from height h is `-acos(R / (R + h))` within 1e-4 rad for
    h = 10 m, 2 km, 20 km;
  - a single Gaussian mountain on the sphere (test Terrain): elevation from a point at known distance equals the
    analytic maximum of `atan2(r1 cos t - r0, r1 sin t)` over the profile within 0.1 deg (1.745e-3 rad);
  - real LunarTerrain (skip if `data/terrain` missing, as other lunar tests do): 64 random points x 16 azimuths
    (fixed seed) against a brute-force march (step = terrain resolution / 2 to 400 km, no early-out): |diff| <= 0.1
    deg where the reference is below 5 deg; plus points across lon +-180 and at lat -89.9 / +89.9 finite;
  - early-out is conservative: never higher than the brute force by more than the tolerance;
  - timing printed (not asserted): 65 x 65 points x 16 azimuths at leaf spacing near the IM-2 site and at level 7.
- [ ] Step 2: implement. Geometry exact on the sphere: target at angular distance t, radius r1 = R + H(t), observer
  r0 = R + h: elevation `atan2(r1 cos t - r0, r1 sin t)`. Steps grow geometrically from `first_step_m` (factor a named
  [E] constant chosen so the brute-force test holds). Stop when the bound with r1 = R + max_height_m() cannot beat the
  running maximum, or at the antipodal limit. Great-circle destination formulas for (lat, lon); longitudes wrap.
- [ ] Step 3: `tools/build.ps1 -NoBindings -TestFilter "[terrain_horizon],[lunar]"` green.
- [ ] Step 4: commit `feat(core): terrain horizon elevations for terrain shadows`, push.

### Task 2: Bindings and horizon textures per Moon node

**Files:**
- Modify: `bindings/src/starsim_core.hpp/.cpp` (two methods, thin), `game/world/terrain/height_source.gd`,
  `terrain_node_builder.gd`, `terrain_quadtree.gd`, `game/shaders/terrain_moon.gdshader`
- Test: `game/tests/terrain_quadtree_test.gd` (headless), `game/tests/render_check.gd` runs (GPU)

**Interfaces:**
- Consumes: Task 1.
- Produces: `StarsimCore.horizon_elevations(body: String, lats_deg, lons_deg, heights_m: PackedFloat64Array,
  azimuth_count: int, min_distance_m: float, first_step_m: float) -> PackedFloat32Array` (rad; empty when the body has
  no terrain); `StarsimCore.horizon_elevation_rad(body, lat_deg, lon_deg, height_m, azimuth_rad) -> float` (NAN
  without terrain; full march from the terrain resolution). `HeightSource.has_horizon() -> bool` (Moon only).
  Builder result key `"horizon"`: an Image (or null), the texture the quadtree hands to the node material as
  `horizon_map`; the Moon's CUSTOM2 (unused there - no water mask) carries the vertex's uv in it.
- [ ] Step 1: headless test in terrain_quadtree_test: a Moon node at the IM-2 site builds a horizon image of the
  stated size; decoding a texel returns the core's value within the encoding precision; Earth nodes build none.
- [ ] Step 2: bake. N_AZ = 16. Encoding: precision <= 0.1 deg for |elevation| <= 5 deg (e.g. RGBA8 with a
  sign * sqrt mapping over [-E_MAX, E_MAX], or half floats - implementer's call, state it). Bake budget: one node's
  horizons <= 30 ms [E] on one worker; if the full 65 x 65 march exceeds it, split: a near march per vertex
  (min_distance 0 up to 16 node spacings) and a far march (from 16 spacings) on every 4th vertex, bilinearly
  interpolated, max of both. Skirt vertices take their edge vertex's horizon. State bake ms per node (leaf, level 7).
- [ ] Step 3: shader. Local east / north at the fragment from the Moon-fixed position (pole: any consistent frame,
  no NaN); the light's elevation and azimuth from `to_sun` / `to_earth` against the sphere direction; horizon by
  linear interpolation between the two neighbouring sectors (texture filtering between vertices); visible fraction of
  a disc of angular radius `asin(sun_radius_m / |to_sun|)` above a straight horizon (circular-segment area, exact).
  Multiplies the Sun term and the umbra glow; the earthshine uses the Earth's direction with the Earth's angular
  radius. Comment in the shader header replaces the "not modelled" note.
- [ ] Step 4: GPU render checks (before = current main, after), `screenshots/moon_shadows/`:
  terminator view `--utc=2024-04-15T19:00:00 --body=moon --lat=0 --lon=-2 --eye_height=15000 --azimuth=90
  --elevation=-8 --fov=70`; the same at --eye_height=2 near the terminator on the dayside (long shadows); IM-2 site
  ground view with the Sun low; one orbit view (100 km) across the terminator; one stereo run. Frame time on the Moon
  before / after (`--gpu_time`), within +-2 %. Read every PNG; send the pairs to the controller in the report.
- [ ] Step 5: `tools/build.ps1 -TestFilter "[terrain_horizon]"`, smoke_test, terrain_quadtree_test green; commit
  `feat(game): lunar terrain shadows from baked horizon maps`, push.

### Task 3: Direct Sun of vehicles and the eye on the Moon

**Files:**
- Modify: `game/world/atmosphere.gd` (`moon_visible_fraction` or its caller), `game/world/world.gd`
  (`_update_sun_light`)
- Test: `game/tests/atmosphere_check.gd` (the existing low-point case around line 807) or a headless smoke_test case

**Interfaces:** Consumes `StarsimCore.horizon_elevation_rad` (Task 2).
- [ ] Step 1: test: a point 2 m above the lunar ground at the IM-2 site, Sun direction chosen 1 deg below the
  terrain horizon in its azimuth -> fraction 0; 1 deg above -> 1; at the horizon -> ~0.5. In lunar orbit (100 km) the
  fraction equals the current behaviour.
- [ ] Step 2: near the lunar ground (where `_moon_ground_radius` applies today) the Sun's visible fraction = the
  disc fraction above the terrain horizon from `horizon_elevation_rad` in the Sun's azimuth, times the eclipse
  fraction; elsewhere unchanged. Query at most once per frame (cache by point / Sun direction change thresholds as
  [E] constants if the full march costs > 0.5 ms on the main thread - measure and state).
- [ ] Step 3: tests green; commit `feat(game): vehicles' direct Sun on the Moon from the terrain horizon`, push.

### Task 4: Docs
- terrain-rendering.md (Moon: horizon maps, encoding, budgets, numbers measured; remove the known limit, add the
  remaining limits: grid-resolution edges, no vehicle cast shadows), lunar.md / atmosphere doc where the vehicle Sun
  is described, architecture.md (core `environment::terrain_horizon`), ansichten.md only if something visible to the
  player is described there. Commit `docs: lunar terrain shadows`, push.

### Final
Final review (Opus) over the whole range, one fix round, user VR test.
