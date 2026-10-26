# Static clouds - design

Date: 2026-10-26. Status: draft for the user's approval. Follows the VR performance plan
(`docs/superpowers/plans/2026-10-26-vr-performance.md`, parts 1-2) and changes the M8 clouds
(`docs/subsystems/weather-clouds.md`).

## Why

In the headset the clouds flicker, look blurred and have unnatural, soft edges, and they cost too much: the VR GPU frame
is 15-19.5 ms (budget 11.1 ms at 90 Hz) where the scene without clouds needs 5-9 ms. Every frame every cloud pixel is
marched anew - 48-96 steps through the density, and at every step a second march of 5 + 2 samples toward the Sun - with
a random start offset per pixel and frame whose noise the temporal accumulation has to hide (the flicker). Measured at
the VR pixel count (terrain_perf `--render_scale=1.65 --gpu_passes`, 2026-10-26): the light march is ~60-65 % of the
cloud pass (6.9 -> 2.5 ms in the descent with 5 + 2 -> 2 + 0 light samples).

The user does not want clouds that change by themselves (user 2026-10-26): "realistic, physically lit STATIC clouds
that look more like clouds" are enough. The clouds' look may change only when its causes change - the weather hour and
the Sun's position.

## What the user wants (their words, condensed)

- The moments that matter: clouds seen **from below** (pad, ascent) and **from orbit**. Flying through counts only as
  "a real cloud", not how it changes.
- **From outside** a cumulus is **clearly outlined**; the fractal belongs in the **shape** (cauliflower lobes of the
  silhouette), not in frayed or blurred edges. **Inside** it is transparent by its density: the deeper one flies in, the
  less of the outside terrain is seen - like fog.
- **Cumulus never touch the ground**: their base is flat at the height where the rising air condenses; on the ground
  there is only fog or rain shafts.
- **From orbit**: the cloud pattern, the shadows on the ground and the cloud height must be right, and the near
  volumetric clouds must pass into the far view (the "satellite image" look) without a visible seam. At that distance
  the clouds need not be truly volumetric.
- **Stable 90 Hz have priority** over cloud quality.
- Both eyes see the same clouds (part 2 of the VR plan, kept).

## The user's definition of a cloud (user 2026-10-26, verbatim in substance)

Clouds are bodies of fog of different density (by cloud type) with an irregular shape computed once (randomised) per
cloud type. They are lit by the Sun (with its colours); the sides facing away go grey (darker). Terrain or the Sun seen
through a cloud fades the more material (fog) lies between, down to zero visibility; the Sun reaches further than the
terrain (it is far brighter). Every part of this design serves that definition; where it adds something (flat bases,
crisp outlines, the far field) it is a physical consequence or an implementation of it.

## Goals

1. No flicker: a static scene shows a still cloud image; a moving camera shows the same clouds from another viewpoint.
2. Crisp cumulus silhouettes from outside; fog-like extinction inside.
3. Flat cumulus bases at the lifting condensation level; no cumulus at the ground (fog / low stratus stay possible where
   the data has them).
4. A seamless near -> far hand-over well below orbit altitude; the far clouds from the column model with their pattern,
   height and ground shadows.
5. **Shape by cloud type** (user's definition): the irregular, once-computed shape depends on the type, derived from the
   data per column and layer since ERA5 / GFS give no type: cumulus (low liquid cloud whose fraction thickens upward /
   broken cover: lobed, cauliflower silhouettes, flat base), stratus / stratocumulus (a deck with fraction ~1: flat
   sheet, top undulating, little vertical structure), cirrus (ice, high: thin fibrous streaks). Cover and water
   invariants hold per type (decision 3 below).
6. **The Sun through clouds** (user's definition): the Sun disk is drawn unclipped in HDR before the clouds are composited
   and is attenuated by exp(-tau) like everything behind a cloud, so it shows through optical depths at which the
   terrain has long vanished (with the forward-scattering aureole from the phase function); zero visibility only where
   tau exceeds what the Sun's ~1e5 x the terrain's radiance can cross. A render check at the Sun through a deck of
   increasing tau validates it.
7. Cloud cost in VR (centre view, both eyes) <= 2.5 ms GPU in every phase (pad, climb, decks, 13-60 km, orbit), so the
   frame fits 90 Hz once the rest of the scene does.

## Non-goals

- Clouds that evolve (growth, drift by wind, turbulence). Wind is M8 part 4.
- GOES / satellite cloud imagery (M8 part 3): open, the user decides after this rework.
- Lightning, rain shafts, fog as its own effect (M8 part 2).
- The main-thread frame budget (plumes, terrain traversal): separate work; 90 Hz overall needs it too.

## Anti-goals (failure even if it "works")

- Any visible change of a cloud's shape or light while nothing changes (the flicker under another name).
- Blurred / frayed silhouettes, or a "cotton wool" glow instead of an outline.
- A visible seam, pop or brightness jump at the near/far hand-over or when a light cache refreshes.
- Different clouds in the two eyes.
- Cumulus standing on the ground.

## Constraints

- Physics first (CLAUDE.md): lighting stays physically based - the Monte Carlo transfer table, the atmosphere's
  transmittance and in-scatter, eclipse, Moon light at night. A cached quantity must equal what the per-frame march would
  compute up to its documented resolution.
- The cloud shapes stay Earth-fixed and come from the existing noise / field; the shape change asked here (crisp
  outline, lobed silhouette) supersedes the M8 "soft fractal edges, frozen shapes" ruling (user 2026-10-26).
- Desktop: may take the same path; its look must not get worse (render_check A/B with the user).
- Stereo exactness and the centre view (`CloudCentreView`, `cloud_vr_stereo_test`) stay.
- Data: only free sources; 2 m temperature and 2 m dew point from ERA5 / GFS are free single-level fields
  (`tools/data_pipeline/sources.toml` gets them if not covered by the existing entries).
- Tests: every model change has a unit test or a validation case with a documented tolerance; VR runs with
  `--perf-log` are the performance gate (`docs/subsystems/vr-performance.md`).

## The design

Three parts; each can be built and measured on its own, in this order.

### A. Light cache instead of the light march

The light a cloud sample receives depends on the Sun direction, the weather snapshot (and its blend) and the point - not
on the camera. It is computed once into a 3D cache and only looked up while marching.

- **What is cached per voxel:** what the march's light source needs today: the optical depth toward the Sun
  `tau_light` (shapes near, grid mean far, as `cloud_light_optical_depth`) - and the Moon's when the Moon is the light.
  The column terms (`cloud_column_depths`) and the transfer table stay as they are (cheap lookups).
- **Where:** a camera-anchored clipmap of Earth-fixed 3D textures over the cloud shell (-0.5..20 km), several cascades
  (fine near the camera, coarse far); each cascade re-anchors when the camera leaves its central part and refills only
  the uncovered slab (toroidal addressing).
- **When it is computed:** a compute pass with a fixed per-frame budget fills voxels progressively. Full refresh when
  the snapshot blend changes by a visible step, when the Sun direction has turned by more than an angle threshold, or on
  a clock jump; between, the old cache stays valid. A refresh is cross-faded (two cache generations) so it never pops.
- **Builder decides:** cascade count, extents and voxel sizes (budget: VRAM and fill time), the angle threshold, the
  per-frame budget, the lookup filter. Rule: the cached `tau_light` at the voxel size must reproduce the per-frame march
  within a documented tolerance on the cloud_check validation views.
- **Shadow maps:** stay (ground light); the builder may derive them from the cache if that is cheaper and equal.

### B. Deterministic march with crisp silhouettes

With the light march gone, the view march gets the budget for a deterministic sampling that needs no per-pixel random
offset:

- **No per-frame jitter.** The start offset is fixed (a function of the ray, not the frame) or the steps are aligned to
  Earth-fixed planes, so an unchanged view gives an unchanged image. Banding is removed by more steps where it matters,
  not by noise.
- **The silhouette is found, not sampled:** when a step crosses into cloud, the entry point is refined (bisection or a
  secant on the density threshold) so the outline is sharp at the pixel level.
- **The edge profile:** the outside edge of a cumulus gets steep - a short ramp in metres (tens of m, the turbulent rim)
  instead of the soft width table of M8 - while the lobes (the shape's low octaves and detail levels) stay; thin clouds
  (low extinction, cirrus, wisps) stay translucent by their density.
- **Inside:** unchanged physics - the transmittance of the density decides how much of the outside is seen.
- **Temporal accumulation and the soft-edge tent of part 2** become what they then are: a light smoothing of what is
  left, never a hider of noise. If they are not needed any more they go.

### C. Near / far hand-over and cumulus bases

- **Hand-over distance:** the volumetric march covers distances up to a hand-over range (the builder picks it from the
  budget - a few tens of km); beyond it the pixel takes the far field's column model (`cloud_far_layer`: pattern, height
  by bisection, MC-table lighting, ground shadows through the far shadow map) - in the cloud pass itself, per pixel, not
  only beyond 1.5e7 m. Over a band of distances both are blended so the change is invisible; the far field gets the
  same light cache inputs where it can. In orbit (420 km) every pixel is far: the column model alone.
- **Cumulus bases (lifting condensation level):** the snapshot gets the 2 m temperature T and 2 m dew point Td
  (ERA5 single-level / GFS 2 m above ground) per column; the base of convective cloud is z_LCL = 125 m/K x (T - Td)
  above the ground (Espy; the builder may use the exact LCL formula, e.g. Romps 2017, if its cost is fine). Cloud
  density of the lowest cloud layer below z_LCL is cut with a flat base (a ramp of tens of m), except where the data
  shows saturated air at the ground (T - Td below a small threshold: fog / stratus may reach down). The rule applies to
  the field's convective (liquid, low) cloud only; ice and layered cloud keep their ERA5 heights.
- **Validation:** the existing ERA5 cloud-base fixture (`cbh` comparison, `weather-clouds.md` Validation) checks the
  new bases on its real dates.

## Measures of success

- VR run (`Start-VR.cmd --perf-log`, FSR 1.5 as in the VR plan): cloud cost <= 2.5 ms GPU in every phase (with
  minus without clouds), and no 45 Hz lock caused by the clouds.
- The user's look check in the headset: no flicker, crisp outlines from below, fog inside, no seam when climbing to
  orbit, the orbit pattern with its shadows, no cumulus on the ground.
- Image checks (render_check, fixed exposure, the VR plan's view set plus an orbit and a sunset view): two renders of an
  unchanged scene are identical (no temporal noise), before / after PNGs to the user.

## Decisions taken for the builder (Claude's calls, 2026-10-26 - the user may overrule)

1. **Which cloud the LCL applies to:** the lowest liquid cloud whose base lies in the boundary layer (below 3 km above
   the ground [E]) - cumulus and also stratocumulus, whose base in a well-mixed boundary layer sits at the LCL too. No
   new convective / stratiform field is needed; only T and Td at 2 m join the snapshot (format version bump, analog files
   regenerated).
2. **LCL against the data's own base:** the LCL is a LOWER bound - condensate below it is moved up into the layers
   above it within the same column, the column's water path is conserved (the far field, both shadow maps and the CPU
   light keep agreeing). A data base above the LCL stays where the data has it. The cbh fixture is re-baselined with a
   documented tolerance (LCL vs ERA5 cbh may differ by more than one layer).
3. **Cover and water invariants stay:** the steep edge keeps covered area = cloud fraction and mean water = grid mean
   at every detail state (tables t[L], m[L] rebuilt and re-validated), so the near / far / shadow cross-checks remain.
4. **Stable while the camera moves:** deterministic means the same image for the same view; while moving, aliasing of
   far broken cumulus is avoided by the hand-over - the volume ends before footprints need the shape prefilter's
   per-frame offsets; where a footprint still exceeds it, a deterministic footprint average (mip-based cover) replaces
   the jittered prefilter.
5. **Near / far seam:** the 10 % near-far difference accepted in M8 Task 10 stays acceptable inside a blend band wide
   enough that no edge is seen; the user's headset check is the gate. No side-lighting model for the far field unless
   that check fails.
6. **Budget metric:** the 2.5 ms are the p99 of the cloud cost per frame in the VR run, light-cache fill and refresh
   cross-fade included (stable 90 Hz is the user's priority, so bursts are not allowed).

## Left to the builder

Cache layout and update scheduling, the deterministic step scheme, the bisection depth, the hand-over range and blend
width, the edge ramp in metres, shader / effect structure, and the order of the work inside each part - each with its
[E] rationale and a test, as CLAUDE.md asks.

## Open (the user decides later)

- GOES / satellite imagery (M8 part 3): after this rework.
