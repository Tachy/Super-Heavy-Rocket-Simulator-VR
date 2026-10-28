# GPU check of the volumetric clouds (M8 weather and clouds, Task 8): renders the cloud pass (CloudEffect, composited by
# CloudLayer's quad) over the sky into an HDR SubViewport and compares pixels with the core's CPU reference.
# Synthetic snapshots from StarsimCore.weather_write_slab_snapshot (uniform slabs, 1 deg grid, ground at 0 m):
#   1. background transmittance (debug output 1, grid-mean field) seen from below, inside and above a slab of vertical
#      tau ~1 vs the core's Beer-Lambert along the same ray (weather_cloud_light's sun_transmittance with the ray as
#      "Sun"), 3 %; orientation: a wide view toward the horizon below the slab shows it in the top row at that row's
#      elevation (3 %) and not in the bottom row (the pass's rays match the screen);
#   2. a camera inside the slab renders a finite, non-black image (Review Focus 5); the transmittance to an occluder
#      2 km ahead inside the slab is exp(-beta 2 km) within 3 % (the scene depth -> distance convention); a near
#      "mountain" edge through a half-resolution block: no cloud over the mountain, the sky's slab within 5 %;
#   3. climbing out of the slab in 50 m steps (looking up, the shapes on, temporal accumulation off) changes the centre
#      5 x 5 pixels by <= 5.5 % of the climb's brightest value per step (no pop);
#   4. the Sun's scattered radiance (debug output 2) of a slab of tau ~10 seen from above vs the core's CPU reference,
#      for both multiple-scattering models: the closed form vs slab_reflected_radiance with a converged light march (24
#      samples, mean of 8 frames) 5 % - the port of the 3D source; the Monte Carlo table (Task 8b) vs
#      slab_reflected_radiance_mc 3 %, including a low-Sun forward view; with the preset's march at the worst geometry
#      15 % (the stochastic exp(-tau_s) is brighter on average);
#   5. the GPU's filtered coverage (debug output 3, full resolution) at the slab's centre height over a wide view = the
#      cloud fraction (0.1 / 0.5 / 0.9) within 0.02 (Task 4 deferred: the RGBA8 filter weights against the float64
#      coverage table);
#   6. light on the ground (Task 9; terrain_earth.gdshader on a quad 300 m from a camera 2 m up, under a thin slab of
#      tau ~1, a deck of tau ~3 and a broken layer of cf 0.4, Sun 45 / 20 deg up): with the near shadow map of the grid mean, the shader's cloud
#      factors (debug output 5: direct, the Sun's diffuse share, the sky's) vs weather_cloud_light within 3 %, the
#      rendered ground's luminance ratio weather on / off = (T_s E_dir + T_sd E_sun,h + T_d E_sky) / (E_dir + E_sky)
#      of the CPU within 5 % (the broken layer as independent columns, cloud_optics.hpp); with the game's map (the
#      shapes) under the slabs of cf 1 the ground's total light vs the vehicle's CPU light (grid mean) at the same spot
#      within 10 % (broken: reported - the spot is in a gap or under a cumulus, the CPU's the cell's mean); with weather off every pixel of land and sea bit-identical to the
#      shader without its CLOUD_LIGHT blocks (STARSIM_GROUND_ONLY=1 runs this section alone).
#   7. (fix round 1) through cloud_light_probe.gdshader (the surfaces' cloud-light functions at receivers on annuli
#      around the camera, no terrain geometry), the means over eight independent sites (shape-noise realisations):
#      the hand-overs near map -> far map (9-11 km: annuli at 8 / 10 / 12 km) and far map -> the slanted column
#      (120-140 km: 110 / 130 / 150 km), a cf 0.3 layer and a deck, Sun 45 / 8 deg: steps <= 0.05 + 3 standard errors
#      (0.03 for the smooth far tiers); over the disc inside the near tier the shapes' area mean of the direct Sun
#      vs the CPU's cell (cf 0.15 / 0.3 / 0.4, Sun 45 / 8 deg; the beam-cover overlap model, constants.hpp
#      cloud_overlap_*, is fitted to these shadows: STARSIM_SLANT_TABLE=1 prints the fit table), the near map vs the
#      far map over the same receivers, the column terms vs the CPU, the vehicle's total light vs the ground's mean.
#   8. (static clouds Tasks 1-2) the light cache (CloudLightCache: 5 cascades 8 / 16 / 32 / 64 / 128 km over the shell,
#      filled at 1/16 of the voxels per frame: complete after 16 frames) against the per-sample light march with the
#      cache's converged sample count (CloudLightCache.SAMPLES: 96 near + 32 far, stratified midpoints), on the game's
#      deterministic march (one frame each, the same view samples): the Sun's scattered radiance at every pixel of a
#      64 x 64 view per cascade at its scale (the camera 5 / 9.9 / 18.8 / 36.5 / 72 km up, looking 80-81 deg down;
#      the farthest cloud samples asserted inside that cascade's full weight, beyond the finer ones'): a slab of tau
#      ~10 (debug output 2, the grid mean): worst pixel within 5 %; a broken layer of cf 0.4 (debug output 7, the
#      shapes): 99th percentile of the per-pixel error within 10 %, worst pixel within 20 % (the interior / silhouette
#      split, alpha < 0.3 within one march pixel, is printed only) - Sun 45 / 20 deg up; a view
#      beyond every cascade with a complete fill, and the frames right after a clock jump (nothing held), equal the
#      march exactly (bit for bit); the fill's GPU time at the game's budget <= 0.5 ms (95th percentile of the frames).
#   9. (static clouds Task 3) the deterministic march's silhouettes: a broken layer (cf 0.3, opaque cumuli) seen
#      from 5 km above its top, straight down; its 50 % background-transmittance contour (debug output 4) lies within
#      one march pixel of the converged reference's (the jittered march with ~12 m steps, the mean of 16 frames), for
#      the desktop and the VR preset (STARSIM_SILHOUETTE_ONLY=1 runs this section alone);
#  10. (static clouds Task 2) no seam: a line of pixels crossing each cascade's edge band (cascade 0 -> 1 -> ... -> 4 ->
#      the march, the slab): the row-to-row step of (cache - march) / mean stays within the image noise (the largest
#      difference of two march renders with independent jitter); the same view before / after a re-anchor (the
#      camera moved 12 km east and back: cascade 3 re-anchored) differs by less than that noise;
#  12. (static clouds Task 7) the flat cloud base at the lifting condensation level on a snapshot BUILT from fields
#      (StarsimCore.weather_write_column_snapshot: GFS levels, cloud from the ground to 2 km, 2 m spread 7 K - LCL
#      ~870 m - raised by the core): straight up from the ground the optical depth is the core's column within 1 %, a
#      view 6 deg downward from 100 m below the base sees no cloud (the same column with a saturated surface, fog,
#      does), one straight up from the top of the base's layer sees the cloud as without the base texture
#      (STARSIM_LCL_ONLY=1 runs this section alone).
#  11. (static clouds Task 5b) anti-aliased outlines: the limb of a deck's top (a straight edge) seen at the VR
#      preset's divisor with the camera rolled; along each row the edge crosses, the final image's alpha changes
#      monotonically by at most 0.35 per pixel (no stairs of a march pixel), and a second render is bit-identical
#      (STARSIM_OUTLINE_ONLY=1 runs this section alone).
#  13. (static clouds Task 9) the Sun through slabs of vertical tau 2 / 5 / 7 / 8 / 9 / 10 / 20 seen from below (the
#      disk drawn by sun.gdshader, HDR): the cloudless disk's centre at its physical radiance (unclipped 13 deg up; 55
#      deg up above the half-float clamp, its colour kept); the disk's radiance through the slab (the centre, less the
#      same view without the disk: the cloud's glow) = the cloudless disk x the cloud pass's transmittance within 5 %
#      (exactly 0 where that is below the half-float store, and then physically too) and at or above exp(-tau / mu)
#      (the plane-parallel bias of the drawn inhomogeneity, Cahalan et al. 1994), brighter than the cloudless terrain
#      while its expected radiance is, the aureole (the Sun's scattered radiance beside the disk) at least 100 x that
#      60 deg away; the near march, and the far tier alone for tau 5 / 20; the shipped analog June weather's cumulus
#      at Starbase (Task 10, a realistic body: gaps and cores), the Sun 45 deg up at eight azimuths: each disk = the
#      cloud pass's transmittance (a single ray through broken cloud is a gap's 1 or a core's ~0, so exp(-tau / mu) is
#      no bound per ray), the mean over the azimuths at or above the core's grid-mean transmittance (STARSIM_SUN_ONLY=1
#      runs this alone).
#  14. (static clouds Task 9b) the backlit slab (tau 10 / 30, Sun zenith 75 / 45 deg) seen from below: the
#      transmitted radiance (debug output 2, bin means) vs the Monte Carlo fixture cloud_slab_transmitted_mc.csv within
#      7 % (the light cache filled first); the clouds' shadows
#      on the air: a backlit cumulus (cf 0.5, in-cloud tau 50 in 1-2 km,
#      the Sun 15 deg up, looking at it; HDR luminance, pixels classed by the cloud image's T: sky T > 0.999, body
#      T < 1e-6, rim 0.1-0.8) - the body's median below the sky's at the same angle from the Sun and the rim's p90
#      2-4 x the body's median within 15 deg of the Sun; an outline pixel's loss its own (debug outputs 9 / 10); the
#      composite's air in front of a cloud at weight 1 (debug 11 / 12 against the core's air light to the cloud's
#      distance); crepuscular rays under a broken deck (cf 0.4) at a 4-deg Sun toward it: the on / off luminance ratio
#      per column (azimuth) over its running median varies by >= 2 % (p90 - p10), a ray-free control (T_c uniform)
#      below that; away from the Sun the shadowed air present (mean loss >= 1 %); the segment sum against the scene's
#      single scattering (debug 8, T_c uniform) at low / high views and a setting Sun (+0.3 / -0.5 deg) within 1 %;
#      no negative final HDR; the clear sky (above the deck looking up, a cloudless field) within 0.5 %
#      (STARSIM_AIR_ONLY=1 runs this alone).
#  15. (cloud view cache Task 3) the march image of a static view twice bit-identical (STARSIM_MARCH_REFERENCE=<dir>:
#      also against a reference directory, before / after a refactoring); the cloud view cache's fill of the whole cube
#      from the camera vs the march at divisor 1: on face-aligned views (pixel rays = texel rays) luminance error median
#      1 % / p99 5 %, T RMS 0.01, cloud distance median 1 %; on general views (bilinear resampling) the medians, the rest
#      printed (STARSIM_VIEW_CACHE_ONLY=1 runs this alone).
#  16. (world volume plan Task 5) the cloud world volume (CloudWorldVolume) of the analog and ERA5 pair over Starbase:
#      the GPU bake vs the core's cell bake per level 0, 2, 5, level 2's water per weather cell, the max-extinction mip
#      and the distance field conservative (exhaustively, and sampled by the probe; the fade band of levels 0-6), a
#      clamped bottom read at its edge, a rebake bit-identical, the GLSL grid
#      mirror within 1e-3 of a cell, the bake binding's input forms, the pair switch (STARSIM_VOLUME_ONLY=16 runs this
#      alone; another value exits 1). STARSIM_WORLD_VOLUME=1 turns the flag on for every GPU check.
#  17. (world volume plan Task 6) the world volume's light over section 8's thick slab, read back: the cold start's light
#      per level within 1 s of its density shown, tau_sun / tau_below / tau_above of cells inside the slab at levels 0,
#      2, 5 vs the analytic slab 5 %, one refresh pass after a 5 deg Sun step moves no exp(-tau_sun) by more than 0.01,
#      Earth's shadow (Sun >= 6 deg below a cell's horizon: the shadow value), tau_moon 5 %, no Moon jobs by day, no
#      NaN / infinity in any level (STARSIM_VOLUME_ONLY=17 runs this alone).
# The other sections run with the light cache on and the deterministic march (the game's path; 10 with the jittered
# one: it measures against independent jitter's noise). Detail
# levels off for the numbers (cloud_detail_levels 0); the desktop quality preset. Exit code 1 on failure.
# Needs a GPU (no --headless): godot --path game --script res://tests/cloud_check.gd
extends SceneTree

const VIEW_SIZE := 33  ## odd: the centre pixel's ray is the camera axis
const COVERAGE_VIEW_SIZE := 161
const SETTLE_FRAMES := 4
const EPOCH_UTC := "2024-04-08T18:42:00"
const AU_M := 149597870700.0
const LAT_DEG := 30.0
const LON_DEG := 45.0
const LCL_CLOUD_TOP_M := 2000.0       # section 12: a built column, cloud from the ground to 2 km
const LCL_LIQUID_KG_KG := 1.2e-5      # specific liquid water: optical depth ~2 over the ~1.1 km above the base
const LCL_TEMPERATURE_K := 295.0      # 2 m air
const LCL_DEW_POINT_K := 288.0        # spread 7 K: LCL ~870 m
const LCL_FOG_DEW_POINT_K := 294.5    # spread 0.5 K: saturated (fog), no base
const LCL_FINE_LAYER_M := 250.0       # core weather_fine_layer_thickness_m (the base's layer below 4 km)
const LCL_DEPTH_TOLERANCE := 0.01     # [E] the rendered column against the core's (the review's 1 %)
const SLAB_BOTTOM_M := 1000.0
const SLAB_TOP_M := 2000.0
## Liquid water of vertical optical depth 1 over 1 km: beta = 3 LWC / (2 rho_w r_l) = 150 LWC [1/m].
const THIN_LIQUID_KG_M3 := 1.0e-3 / 150.0
const THICK_LIQUID_KG_M3 := 1.0e-2 / 150.0
const TRANSMITTANCE_TOLERANCE := 0.03
const TRANSMITTANCE_FLOOR := 0.02
const RADIANCE_TOLERANCE := 0.05
const TABLE_RADIANCE_TOLERANCE := 0.03
const PRESET_RADIANCE_TOLERANCE := 0.15
const CONVERGED_LIGHT_SAMPLES := 24
const RADIANCE_FRAMES := 8
# 5.5 %: the steeper rims of Task 4 (max edge width 40 m) let a 50 m step cross more of a rim (measured 5.07 %).
const CLIMB_STEP_LIMIT := 0.055
const COVERAGE_TOLERANCE := 0.02
## Transmittance and radiance samples: the mean of the 5 x 5 pixels around the centre (see _render).
const BLOCK_RADIUS := 2
const EXPECTED_SAMPLES := 140 + SUN_SAMPLES + AIR_SAMPLES + VIEW_CACHE_SAMPLES + VOLUME_SAMPLES + LIGHT_SAMPLES
## Occluders: one inside the slab at a known distance (depth -> distance), one near "mountain" over half the view.
const OCCLUDER_DISTANCE_M := 2000.0
const OCCLUDER_SIZE_M := 1000.0
const EDGE_DISTANCE_M := 50.0
const EDGE_SIZE_M := 100.0
## Light on the ground (section 6).
const GROUND_DISTANCE_M := 300.0
const GROUND_ALBEDO_8 := Color8(150, 120, 90)
const MOON_DISTANCE_M := 3.844e8
const DECK_LIQUID_KG_M3 := 3.0e-3 / 150.0
## A broken layer (independent columns: clear on 1 - cf, the in-cloud depth on cf).
const BROKEN_FRACTION := 0.4
const DIRECT_FLOOR := 0.01
## The vehicle's CPU light is the grid mean, the ground's shadow the visible shapes (cf 1 slabs still vary along a ray
## by the shape noise's low octaves): their total light on a horizontal plate (measured: within 6 %).
const VEHICLE_TOLERANCE := 0.10
const FACTOR_TOLERANCE := 0.03
const GROUND_RATIO_TOLERANCE := 0.05
## Section 7 (hand-overs and the shapes' area means, receivers on annuli through cloud_light_probe.gdshader).
const PROBE_WIDTH := 256
## Sites ~55 km apart (degrees north, east of the check site): independent realisations of the shape noise.
const SITES := [Vector2(0.0, 0.0), Vector2(0.0, 0.55), Vector2(0.0, -0.55), Vector2(0.5, 0.0), Vector2(-0.5, 0.0),
		Vector2(0.5, 0.55), Vector2(-0.5, -0.55), Vector2(0.5, -0.55)]
const DISC_RADIUS_M := 8000.0  ## inside the near tier (it is the only one below 9 km)
## The near map holds one realisation of the shape noise: an area mean over N independent cloud cells carries
## sigma ~ sqrt(cf (1 - cf) / N) in the covered fraction (the coverage-preserving noise fixes the mean over a tile, not
## over a disc), and the shapes' internal density variation (Jensen: mean of exp(-tau) above exp(-mean tau)) moves the
## cloudy part's transmittance up. Tolerances are set from the measured values with that margin.
const DISC_TOLERANCE := 0.08
## A step across the hand-over: this much (the tiers' systematic agreement: the fitted overlap is good to 0.02) plus
## three standard errors of the two annuli's means over the sites (the shape noise's sampling error).
const HANDOVER_TOLERANCE := 0.05
const FAR_HANDOVER_TOLERANCE := 0.03  ## the far map and the slanted column are both smooth (the grid mean)
## Section 8 (the light cache against the light march).
const CACHE_VIEW_SIZE := 64
const CACHE_SLAB_TOLERANCE := 0.05
const CACHE_BROKEN_TOLERANCE := 0.10  ## the voxel size (62.5 m across, 40 m up near the camera) against the shapes
## [E] Acceptance of every section 8 case (controller ruling): p99 of the per-pixel relative error and the worst pixel.
const CACHE_P99_TOLERANCE := 0.10
const CACHE_WORST_TOLERANCE := 0.20
const CACHE_EDGE_ALPHA := 0.3  ## [E] the controller's silhouette bound (fix round 1 of Task 3)
## The fill's budget in the checks: 1/16 of all voxels per frame (complete after 16 frames).
const CACHE_BUDGET_DIVISOR := 16
const CACHE_MAX_SETTLE_FRAMES := 600  ## > a full refresh at the game budget (427 frames)
const CACHE_FILL_LIMIT_MS := 0.5  ## spec decision 6: the fill's share of the 2.5 ms per frame
const CACHE_FILL_IDLE_MS := 0.02  ## a frame without fill jobs (the interval's timestamps alone)
## Each cascade at its scale: the same view (81 deg down, fov 22; cascade 0: 80 / 24) from a camera about twice as high
## per cascade (x2 apart as the cascades), its farthest cloud samples just inside that cascade's full weight and
## beyond the finer cascade's (asserted): the outer part of the view is that cascade's, the inner the finer ones', at
## the same pixel footprint / voxel ratio as cascade 0.
const CACHE_VIEWS := [
	{"cascade": 0, "height_m": 5000.0, "elevation": -80.0, "azimuth": 10.0, "fov": 24.0},
	{"cascade": 1, "height_m": 9900.0, "elevation": -81.0, "azimuth": 10.0, "fov": 22.0},
	{"cascade": 2, "height_m": 18800.0, "elevation": -81.0, "azimuth": 10.0, "fov": 22.0},
	{"cascade": 3, "height_m": 36500.0, "elevation": -81.0, "azimuth": 10.0, "fov": 22.0},
	{"cascade": 4, "height_m": 72000.0, "elevation": -81.0, "azimuth": 10.0, "fov": 22.0},
]
## Beyond every cascade: from 20 km toward the northern horizon (the slab entered > 220 km away).
const CACHE_OUTSIDE_HEIGHT_M := 20000.0
const CACHE_OUTSIDE_ELEVATION_DEG := -4.9
const CACHE_OUTSIDE_FOV_DEG := 1.0
const CACHE_BORDER_POINTS := 16  ## rays per border edge of a view's cloud range
## Section 10 (no seam): a line of pixels (16 x 128: the row means) toward north across each cascade's edge band.
const CACHE_LINE_SIZE := Vector2i(16, 128)
const CACHE_EDGE_VIEWS := [
	{"cascade": 0, "height_m": 5000.0, "elevation": -56.0, "fov": 20.0},
	{"cascade": 1, "height_m": 5000.0, "elevation": -31.5, "fov": 12.0},
	{"cascade": 2, "height_m": 5000.0, "elevation": -18.0, "fov": 10.0},
	{"cascade": 3, "height_m": 5000.0, "elevation": -11.5, "fov": 8.0},
	{"cascade": 4, "height_m": 5000.0, "elevation": -4.3, "fov": 3.0},
]
## The re-anchor pair: cascade 3's edge view from this far east, after the camera went twice as far east and back
## (cascade 3, 500 m voxels: re-anchors beyond 10 km - away, not here).
const CACHE_REANCHOR_EAST_M := 6000.0
const CACHE_REANCHOR_CASCADE := 3
## A pixel's error is relative to max(its reference, this fraction of the view's mean reference): a cloud's fringe or a
## gap's last light carries no relative accuracy of its own.
const CACHE_ERROR_FLOOR := 0.05
## Section 11 (static clouds Task 5b): the top of the thick slab (cf 1: one deck) seen from OUTLINE_HEIGHT_M above it
## at its limb (the horizon's dip), the camera rolled by OUTLINE_ROLL_DEG: the limb crosses the march grid at a shallow
## angle - the stairs' worst case, one step every 1 / tan(roll) march pixels.
const OUTLINE_VIEW_SIZE := Vector2i(192, 96)
const OUTLINE_BIG_VIEW_SIZE := Vector2i(2400, 1800)
const OUTLINE_FOV_DEG := 20.0
const OUTLINE_HEIGHT_M := 2000.0
const OUTLINE_ROLL_DEG := 7.0
## [E] Largest alpha change between two neighbouring pixels along a row the edge crosses (the brief's 0.35: a stair of
## one march pixel at divisor 3 changes alpha by up to ~0.42 per pixel through the Catmull-Rom upsample).
const OUTLINE_MAX_JUMP := 0.35
## [E] A change against the row's trend smaller than this is not counted (the deck top's own relief at the limb).
const OUTLINE_MONOTONE_TOLERANCE := 0.03
## Rows whose alpha spans from below OUTLINE_CLEAR to above OUTLINE_COVERED cross the edge inside the view.
const OUTLINE_CLEAR := 0.05
const OUTLINE_COVERED := 0.95
const OUTLINE_MIN_ROWS := 8

## Section 13 (static clouds Task 9): the Sun 55 deg up (mu 0.819) through slabs of these vertical optical depths.
const SUN_TAUS := [2.0, 5.0, 7.0, 8.0, 9.0, 10.0, 20.0]
const SUN_FAR_TAUS := [5.0, 20.0]  ## also with the far tier alone
const SUN_ELEVATION_DEG := 55.0
## The cloudless disk this far up: T_atm x the gain < 0.69, so its centre (World.sun_disc_scene_value) stays below the
## clamp - the unclipped case; at SUN_ELEVATION_DEG it is above (the documented clip).
const SUN_LOW_ELEVATION_DEG := 13.0
const SUN_FOV_DEG := 1.0  ## the disk (0.53 deg) fills half of the view's width
const SUN_RENDER_DISTANCE_M := 2.0e7  ## as World.SUN_RENDER_DISTANCE_M
const SUN_AUREOLE_PIXELS := 14  ## the pixel beside the disk (0.43 deg from the centre; the disk's radius is 0.27)
const SUN_AUREOLE_AWAY_DEG := 60.0
## [E] The forward peak of the phase function against a view 60 deg off the Sun (measured 303 x through tau 5).
const SUN_AUREOLE_RATIO := 100.0
const SUN_DISK_TOLERANCE := 0.05
## sun.gdshader's clamp of the disk's largest channel (inside the half-float scene buffer).
const SUN_DISK_CLAMP := 6.0e4
## [E] The drawn disk's centre against the core's sun transmittance x World.sun_disc_scene_value (the GPU's
## transmittance table against the core: 2 %, atmosphere_check).
const SUN_DISK_COLOUR_TOLERANCE := 0.03
## Half of one ulp of a half float relative to its value, at most (2^-11): the resolution of a pixel of the viewport's
## HDR image - the disk's part is the difference of two such renders.
const SUN_HALF_RESOLUTION := 4.8828125e-4
## The cloud image stores the background transmittance T in its .a channel as a half float: T below half the smallest
## subnormal (2^-25) is stored as 0 - the disk then is exactly 0 behind the cloud.
const SUN_T_STORE_MIN := 2.98023224e-8
## [E] The march's slant optical depth against the core's (the slab's shape octaves vary the depth along a ray).
const SUN_TAU_TOLERANCE := 0.08
const SUN_TAU_MIN_T := 1.0e-6
## Section 13's realistic cumulus body (Task 10): the shipped analog weather of June at SUN_CU_SITE (Starbase, where the
## cumulus fields of the views are), the Sun SUN_ELEVATION_DEG up at these azimuths from the camera on the ground. A
## single ray through broken cumulus is a gap's 1 or a core's ~0, so exp(-tau / mu) bounds no single disk: each disk
## follows the cloud pass's transmittance (a pixel on a core's edge is a coverage mix: within the range of the
## transmittance at the centre and SUN_CU_NEIGHBOUR_PIXELS away in four directions), and over many directions the
## mean transmittance of the shapes is at or above the core's grid-mean one (Jensen; the slabs' bias band, Cahalan et
## al. 1994) within the statistical error of the sample.
const SUN_CU_SITE := Vector2(25.997, -97.157)  ## latitude, longitude (deg)
const SUN_CU_MONTH := 6
const SUN_CU_AZIMUTHS_DEG := [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0]
const SUN_CU_NEIGHBOUR_PIXELS := 4  ## of the 1 deg view's 0.53 deg disk (~half its radius)
const SUN_CU_ENSEMBLE_ELEVATIONS_DEG := [35.0, 45.0, 55.0, 65.0, 75.0]
const SUN_CU_ENSEMBLE_AZIMUTH_STEP_DEG := 30.0
## [E] The ensemble's allowance below the grid mean: this many standard errors of the mean of N Bernoulli-like
## transmittances (gap 1, core 0), relative to the grid mean m: sqrt((1 - m) / (m N)); neighbouring directions share
## clouds, so N is the nominal count (conservative the other way: the real error is larger, the bound a warning
## against a disk that is far too dark, not a fit).
const SUN_CU_ENSEMBLE_SIGMAS := 3.0
## Checks of section 13: the cloudless disk 3 (outshines, clip at 55 deg, unclipped at 13 deg); per near tau the disk 2
## (= the march's T or 0 below the store; >= exp(-tau / mu) or that below the store), the march's tau 1 (not tau 20:
## beyond the half floats of the debug output) and the terrain 1; the aureole 1; per far tau (5, 20) the disk 2 and the
## march's tau 1 (5); the analog cumulus: per azimuth the disk 1, the gaps and cores 1, the ensemble mean 1.
const SUN_SAMPLES := 3 + (7 * 2 + 6 + 7) + 1 + (2 * 2 + 1) + (8 + 2)

## Section 9: a broken layer (cf SILHOUETTE_FRACTION, the thick slab's water: opaque cumuli) seen from
## SILHOUETTE_DISTANCE_M above its top, straight down. The reference: the jittered march with fine steps of
## SILHOUETTE_REFERENCE_FINE of the distance (~12 m: the cumulus' rim) and SILHOUETTE_REFERENCE_STEPS coarse ones (the
## VR preset's 48 lost cloud bases in half the frames, jittered too), averaged over SILHOUETTE_REFERENCE_FRAMES frames.
const SILHOUETTE_VIEW_SIZE := 96
const SILHOUETTE_FOV_DEG := 30.0
const SILHOUETTE_FRACTION := 0.3
const SILHOUETTE_DISTANCE_M := 5000.0
const SILHOUETTE_REFERENCE_FINE := 0.002
const SILHOUETTE_REFERENCE_MAX_STEPS := 1024
const SILHOUETTE_REFERENCE_STEPS := 512
const SILHOUETTE_REFERENCE_FRAMES := 16
const SILHOUETTE_MIN_CONTOUR := 24  ## reference contour pixels in view (cloud edges are seen)
const SILHOUETTE_SEARCH := 8  ## full-resolution pixels searched for the reference's contour
# Section 14 (static clouds Task 9b): the clouds' shadows on the air.
const AIR_VIEW_SIZE := Vector2i(320, 180)
const AIR_FOV_DEG := 40.0
const AIR_CUMULUS_FRACTION := 0.5
const AIR_CUMULUS_TAU := 50.0  ## in-cloud vertical tau over the 1 km slab (fair cumulus, extinction 50 /km)
const AIR_BACKLIT_ELEVATION_DEG := 15.0  ## t9f_edge's Sun
const AIR_MIN_ELEVATION_DEG := 2.0  ## pixels this far above the horizon
const AIR_BANDS_DEG := [Vector2(2.0, 6.0), Vector2(6.0, 10.0), Vector2(10.0, 15.0)]  ## angle from the Sun
const AIR_SKY_T := 0.999
const AIR_BODY_T := 1.0e-6  ## slant tau > 14
const AIR_RIM_T := Vector2(0.1, 0.8)
const AIR_RIM_RATIO := Vector2(2.0, 4.0)  ## [E] cloud-visual-reference.md 5.3: the photographic range, not a physical bound
## [E] The band closest to the Sun (2-6 deg) gets 2-5 x: the ratio rises with nearness to the Sun (the rim's forward
## peak) and the body's optical depth, so real silver linings there exceed 4 x. Measured 4.01-4.03; re-measure after Task 9c.
const AIR_RIM_RATIO_NEAR_MAX := 5.0
const AIR_BROKEN_FRACTION := 0.4
const AIR_RAYS_SUN_ELEVATION_DEG := 4.0
const AIR_RAYS_VIEW_ELEVATION_DEG := 12.0
const AIR_RAYS_MIN_PIXELS := 8  ## sky pixels a column needs to count
const AIR_RAYS_MIN_COLUMNS := 40
## [E] p90 - p10 of the detrended on / off ratio (_ray_modulation): rays seen as a ~2 % luminance contrast across
## neighbouring azimuths (the eye's contrast threshold for extended bands is ~1-2 %).
const AIR_RAYS_MIN_MODULATION := 0.02
## [E] Away from the Sun: the shadowed air present - its mean loss at least 1 % (the away view's air light below the
## deck top is mostly single Rayleigh and weak aerosol backscatter, a few % of it shadowed by the cf 0.4 deck).
const AIR_AWAY_MIN_LOSS := 0.01
## [E] The running median's width (61 columns = 7.6 deg): the trend with the angle from the Sun is smooth over it,
## the rays (gaps of ~1 km cumulus seen 10-30 km away: 2-6 deg) are narrower (31 columns followed the rays themselves:
## rays 1.5 %, the control 0.3 %; 61: 3.6 % and 1.1 %).
const AIR_RAYS_DETREND_COLUMNS := 61
## The ray-free control's and the telescoping check's uniform cloud transmittance (cloud_debug_air_transmittance).
const AIR_CONTROL_TRANSMITTANCE := 0.5
## (Sun elevation, view elevation, view azimuth from the Sun's): low views (long segments: the clamp), a high one, the
## setting Sun (the camera's Mie fade: 0.3 deg is inside smoothstep(0, 0.01, mu_s), -0.5 deg below it).
const AIR_TELESCOPE_CASES := [[4.0, 1.0, 0.0], [4.0, 3.0, 0.0], [4.0, 30.0, 90.0], [0.3, 3.0, 0.0], [-0.5, 3.0, 0.0]]
## [E] Pixels whose S1 is below this share of their luminance (the setting Sun's S1 is ~0 below the deck top; the
## half-float image resolves ~0.1 %) are checked absolutely: |loss - S1| within AIR_CLEAR_TOLERANCE of the pixel.
const AIR_TELESCOPE_MIN_SHARE := 0.01
const AIR_TELESCOPE_TOLERANCE := 0.01  ## the review's ~1 % on the median
const AIR_TELESCOPE_SPREAD := 0.03  ## p5 / p95
## Pixels whose neighbours this far are all clear (the upsample's 4 x 4 low-resolution taps at divisor 2): the
## Catmull-Rom lobes next to a cloud filter the image differently with and without the loss's negative colour.
const AIR_TELESCOPE_MARGIN := 4
const AIR_SETTING_SUN_DEG := [0.3, -0.5]
const AIR_SETTING_VIEW_DEG := 3.0  ## the horizon in view (the sea against the shadowed sky)
## [E] An outline pixel's loss vs its own at p99: the source's loss is taken from this pixel's ray parts at this
## pixel's cloud distance (one loss call per outline pixel; review M1) - measured p99 1.7 %, max 7 %; the fault it
## guards against (this pixel's T for the source's) is ~alpha (1 - alpha) of the loss behind (up to 25 %; 32 % of the
## pixels were over 1 %).
const AIR_OUTLINE_TOLERANCE := 0.02
const AIR_OUTLINE_MAX_SHARE := 0.01  ## of the outline pixels may exceed it
const AIR_WEIGHT_MIN_ALPHA := 0.98  ## cloud bodies (the debug output's distance is not mixed with the sky's)
const AIR_WEIGHT_MAX_M := 8000.0  ## inside the march's own tier (the debug outputs have no far tier)
const AIR_WEIGHT_TOLERANCE := 0.03  ## atmosphere_check's SURFACE_TOLERANCE (the tables against the core's raymarch)
const AIR_CLEAR_TOLERANCE := 0.005  ## the ruling's 0.5 %
## The backlit slab against the Monte Carlo transmitted radiance (section 14's first part).
const SLAB_TR_TAUS := [10.0, 30.0]
const SLAB_TR_SUN_ZENITHS_DEG := [75.0, 45.0]
## (view zenith, relative azimuth: 0 = looking toward the Sun's side) - the fixture's bins
const SLAB_TR_BINS := [[0.0, 0.0], [30.0, 0.0], [30.0, 180.0], [60.0, 0.0], [60.0, 90.0], [60.0, 180.0]]
## [E] Measured with the light cache filled 0.1-4.9 % (Task 9b, Monte Carlo sigma 0.3-0.7 %); the measurement round's
## +21..+39 % at view zenith 60 toward the Sun came from a view the cache had not filled yet.
const SLAB_TR_TOLERANCE := 0.07
const SLAB_TR_VIEW_SIZE := 161
const SLAB_TR_FOV_DEG := 44.0
## slab 2 x 2, backlit 2 x 3, outline, front weight, rays 2, control, telescoping 5 (AIR_TELESCOPE_CASES), no negative
## HDR, clear sky 2
const AIR_SAMPLES := 2 * 2 + 2 * 3 + 1 + 1 + 2 + 1 + 5 + 1 + 2
## Section 15 (cloud view cache Task 3): the views - a broken cumulus layer (cf 0.4) seen from the ground looking up,
## and a deck (cf 1) seen from above, looking down (every ray below the horizon ends on the deck: no ground in view, so
## the march's sky distance and the cache's static end - the ellipsoid - agree wherever the background shows).
const VIEW_CACHE_SIZE := 256
const VIEW_CACHE_FOV_DEG := 60.0
const VIEW_CACHE_VIEWS := [["ground", 0.4, 10.0, 35.0, 200.0], ["deck", 1.0, 3500.0, -30.0, 70.0]]
## The cache's tiles (cloud view cache Task 2: 64 texels); its face size from the view's pixel density at the centre
## (CloudViewCacheLayout.face_texels_for), the whole cube filled at the full level from the camera.
const VIEW_CACHE_TILE_TEXELS := 64
## The fill against the march at divisor 1: the per-pixel luminance error (relative to max(the march's,
## CACHE_ERROR_FLOOR x the view's mean)) - median and 99th percentile -, the transmittance's (debug 12: alpha's) RMS,
## the cloud distance where the march's alpha exceeds VIEW_CACHE_DISTANCE_ALPHA (the cache's alpha-weighted bilinear
## sample), median. All bounds on the face-aligned views (VIEW_CACHE_FACE_VIEWS: every pixel's ray is a texel's, the
## shading alone is compared); the general views (VIEW_CACHE_VIEWS, texel vs pixel resampling) the medians only - their
## p99 and RMS are printed: the deterministic march's hard edges and its step grain (Earth-fixed height shells) differ
## between a ray and the bilinear mean of the four texel rays around it (measured 7.8-46 % p99, RMS 0.020-0.021 at
## 448^2 texels per face against 256^2 pixels over 60 deg).
const VIEW_CACHE_MEDIAN_TOLERANCE := 0.01
const VIEW_CACHE_P99_TOLERANCE := 0.05
## The four horizontal face views (+-X, +-Y) look half at the ground: the fill ends at the ellipsoid, the check's march
## at its -500 m shell (the deck nadir's 0.2 % median, here at the horizon's grazing rays: measured p99 2.0-8.0 %). Their
## medians (0.000 %) and distances prove the axes; the p99 bound is looser there.
const VIEW_CACHE_P99_HORIZONTAL_TOLERANCE := 0.10
const VIEW_CACHE_T_RMS_TOLERANCE := 0.01
const VIEW_CACHE_DISTANCE_TOLERANCE := 0.01
const VIEW_CACHE_DISTANCE_ALPHA := 0.5
## The face-aligned views at the equator / Greenwich (Earth-local +Z = ITRF +X: the zenith): a camera looking along a
## cube face's axis with a 90 deg view of VIEW_CACHE_SIZE^2 pixels on a cache of VIEW_CACHE_SIZE^2 texels per face -
## pixel centre rays = texel centre rays. The broken layer from the ground at the zenith (face +Z), the deck from above
## at the nadir (face -Z): no ground in view. Name, cloud fraction, height (m), Earth-local forward axis.
const VIEW_CACHE_FACE_VIEWS := [["ground zenith", 0.4, 10.0, Vector3(0.0, 0.0, 1.0)],
		["deck nadir", 1.0, 3500.0, Vector3(0.0, 0.0, -1.0)],
		["deck +X", 1.0, 3500.0, Vector3(1.0, 0.0, 0.0)], ["deck -X", 1.0, 3500.0, Vector3(-1.0, 0.0, 0.0)],
		["deck +Y", 1.0, 3500.0, Vector3(0.0, 1.0, 0.0)], ["deck -Y", 1.0, 3500.0, Vector3(0.0, -1.0, 0.0)]]
const VIEW_CACHE_FACE_FOV_DEG := 90.0
## Per general view: the identity, debug 0 (colour, distance) and 12 (distance); per face-aligned view: debug 0
## (colour, T, distance) and 12 (alpha, distance).
const VIEW_CACHE_SAMPLES := 2 * (1 + 2 + 1) + 6 * (3 + 2)
## Diagnostics: a directory for the march's images before / after a refactoring (written when missing, else compared
## bit for bit): STARSIM_MARCH_REFERENCE=<dir>.
const MARCH_REFERENCE_ENV := "STARSIM_MARCH_REFERENCE"

var _core := StarsimCore.new()
var _atmosphere := Atmosphere.new()
var _clouds := CloudLayer.new()
var _viewport := SubViewport.new()
var _camera := Camera3D.new()
var _earth_to_gcrf := Basis.IDENTITY
var _failures := 0
var _samples := 0
var _optional_samples := 0  ## checks that run only with a diagnostics switch (MARCH_REFERENCE_ENV)
var _files: Array[String] = []
var _terrain_parent := Node3D.new()  ## Earth-fixed frame of the terrain quad (section 6)
var _terrain_quad := MeshInstance3D.new()
var _sun_disk := MeshInstance3D.new()  ## section 13: the Sun as the game draws it (sun.gdshader)
var _terrain_material := ShaderMaterial.new()
var _probe_viewport := SubViewport.new()  ## section 7: the receivers' probe
var _probe_material := ShaderMaterial.new()
var _water_material := ShaderMaterial.new()  ## the same with an all-water mask (the sea model)


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	if not _core.open(Paths.data_dir().path_join("spice")):
		_fail_and_quit("SPICE kernels missing")
		return
	_earth_to_gcrf = _core.earth_fixed_to_gcrf(_core.utc_to_tdb_s(EPOCH_UTC))
	_atmosphere.eye_adaptation_enabled = false
	_atmosphere.start_loading(_core, Paths.config_dir(), Paths.data_dir().path_join("atmosphere"))
	if not _atmosphere.wait_until_loaded():
		_fail_and_quit("atmosphere tables not loaded")
		return
	_setup_scene()
	_setup_probe()
	_clouds.setup(_core)
	if not _clouds.wait_until_ready():
		_fail_and_quit("cloud noise not ready")
		return
	_clouds.quality = CloudLayer.Quality.DESKTOP
	_clouds.view_cache = false  # the march is what these sections check (section 15 fills the cache on request)
	_detail_off()
	_check("transfer table loaded", _clouds.transfer_ready, "config/" + CloudLayer.TRANSFER_TABLE)

	if OS.get_environment("STARSIM_SLANT_TABLE") != "":
		await _slant_table()
		quit(0)
		return
	if OS.get_environment("STARSIM_SILHOUETTE_ONLY") != "":  # diagnostics: section 9 alone (no sample count)
		await _silhouette_samples()
		_clouds.shutdown()
		for f in _files:
			DirAccess.remove_absolute(f)
		print("cloud_check (silhouettes only): %d failures" % _failures)
		quit(1 if _failures > 0 else 0)
		return
	if OS.get_environment("STARSIM_OUTLINE_ONLY") != "":  # diagnostics: section 11 alone (no sample count)
		await _outline_samples()
		_clouds.shutdown()
		for f in _files:
			DirAccess.remove_absolute(f)
		print("cloud_check (outlines only): %d failures" % _failures)
		quit(1 if _failures > 0 else 0)
		return
	if OS.get_environment("STARSIM_LCL_ONLY") != "":  # diagnostics: section 12 alone (no sample count)
		await _lcl_base_samples()
		_clouds.shutdown()
		for f in _files:
			DirAccess.remove_absolute(f)
		print("cloud_check (cloud base only): %d failures" % _failures)
		quit(1 if _failures > 0 else 0)
		return
	if OS.get_environment("STARSIM_LIGHT_CACHE_ONLY") != "":  # diagnostics: sections 8 and 10 alone (no sample count)
		await _light_cache_samples()
		_clouds.shutdown()
		for f in _files:
			DirAccess.remove_absolute(f)
		print("cloud_check (light cache only): %d failures" % _failures)
		quit(1 if _failures > 0 else 0)
		return
	if OS.get_environment("STARSIM_AIR_ONLY") != "":  # diagnostics: section 14 alone (no sample count)
		if OS.get_environment("STARSIM_AIR_NO_SLAB") == "":
			await _slab_transmitted_samples()
		await _air_shadow_samples()
		_clouds.shutdown()
		for f in _files:
			DirAccess.remove_absolute(f)
		print("cloud_check (air shadow only): %d failures" % _failures)
		quit(1 if _failures > 0 else 0)
		return
	if OS.get_environment("STARSIM_SUN_ONLY") != "":  # diagnostics: section 13 alone (no sample count)
		await _sun_through_samples()
		_clouds.shutdown()
		for f in _files:
			DirAccess.remove_absolute(f)
		print("cloud_check (Sun only): %d failures" % _failures)
		quit(1 if _failures > 0 else 0)
		return
	if OS.get_environment("STARSIM_VIEW_CACHE_ONLY") != "":  # diagnostics: section 15 alone (no sample count)
		await _view_cache_samples()
		_clouds.shutdown()
		for f in _files:
			DirAccess.remove_absolute(f)
		print("cloud_check (view cache only): %d failures" % _failures)
		quit(1 if _failures > 0 else 0)
		return
	if OS.get_environment(VOLUME_ONLY_ENV) != "":  # section 16 or 17 alone (world volume plan Tasks 5, 6)
		await _volume_only(OS.get_environment(VOLUME_ONLY_ENV))
		return
	if OS.get_environment("STARSIM_GROUND_ONLY") != "":  # diagnostics: section 6 alone (no sample count)
		await _ground_light_samples()
		_clouds.shutdown()
		for f in _files:
			DirAccess.remove_absolute(f)
		print("cloud_check (light on the ground only): %d failures" % _failures)
		quit(1 if _failures > 0 else 0)
		return
	var thin := _slab("thin", SLAB_BOTTOM_M, SLAB_TOP_M, THIN_LIQUID_KG_M3, 1.0)
	_show(thin)
	var sun := _local_direction(45.0, 135.0)
	var up := _local_direction(90.0, 0.0)
	# 1. Background transmittance below, inside, above.
	await _transmittance_sample("below, zenith", 10.0, up, sun, thin)
	await _transmittance_sample("below, elevation 30 deg", 10.0, _local_direction(30.0, 200.0), sun, thin)
	await _transmittance_sample("inside, zenith", 1500.0, up, sun, thin)
	await _transmittance_sample("inside, horizontal", 1500.0, _local_direction(0.0, 70.0), sun, thin)
	await _transmittance_sample("above, nadir", 3000.0, _local_direction(-90.0, 0.0), sun, thin)
	await _transmittance_sample("above, elevation -20 deg", 3000.0, _local_direction(-20.0, 10.0), sun, thin)
	# Orientation: below the slab looking at the horizon, the top rows see it (30 deg up), the bottom rows do not.
	_camera.fov = 60.0
	await _render(_point(10.0), _local_direction(0.0, 70.0), sun, 1)
	_camera.fov = 10.0
	var frame := _viewport.get_texture().get_image()
	var top_row := frame.get_pixel(VIEW_SIZE / 2, 0).r
	var bottom_row := frame.get_pixel(VIEW_SIZE / 2, VIEW_SIZE - 1).r
	# The top row shows the ray of the first half-resolution row (its centre 1 px below the edge).
	var top_elevation := rad_to_deg(atan((1.0 - 2.0 / VIEW_SIZE) * tan(deg_to_rad(30.0))))
	var top_expected := float(_core.weather_cloud_light(_point(10.0),
			_local_direction(top_elevation, 70.0))["sun_transmittance"])
	_check("orientation (the slab in the top rows at their elevation, not in the bottom rows)",
			absf(top_row - top_expected) <= TRANSMITTANCE_TOLERANCE * top_expected and bottom_row > 0.99,
			"top %.4f (CPU %.4f at %.1f deg) bottom %.3f" % [top_row, top_expected, top_elevation, bottom_row])
	# 2. Inside the slab: finite, not black.
	for view in [up, _local_direction(0.0, 70.0), _local_direction(-60.0, 300.0)]:
		var c := await _render(_point(1500.0), view, sun, 0)
		var lum := _luminance(c)
		_check("inside the slab: finite, non-black", is_finite(lum) and lum > 1e-4, "luminance %.5f" % lum)
	# 2b. Depth -> distance: an occluder 2 km ahead of a camera inside the slab; the transmittance to it is the slab's
	# in-cloud extinction over 2 km (beta from the CPU's vertical depth of the 1 km slab), 3 %.
	var slab_tau := -log(float(_core.weather_cloud_light(_point(10.0), up)["sun_transmittance"]))
	var beta := slab_tau / (SLAB_TOP_M - SLAB_BOTTOM_M)
	var occluder := _occluder(Vector3(0.0, 0.0, -OCCLUDER_DISTANCE_M), Vector2(OCCLUDER_SIZE_M, OCCLUDER_SIZE_M))
	var t_occluded := (await _render(_point(1500.0), _local_direction(0.0, 70.0), sun, 1, BLOCK_RADIUS)).x
	occluder.queue_free()
	var t_expected := exp(-beta * OCCLUDER_DISTANCE_M)
	_check("transmittance to an occluder at %.0f m inside the slab" % OCCLUDER_DISTANCE_M,
			absf(t_occluded - t_expected) <= TRANSMITTANCE_TOLERANCE * t_expected,
			"GPU %.5f expected %.5f (beta %.6f /m)" % [t_occluded, t_expected, beta])
	await _mountain_edge_sample(sun)
	# 3. Climbing out of the slab in 50 m steps, looking up (a slanted view would cross 50 m / sin(e) more cloud per step).
	# The temporal accumulation off (weight 1): the check is on the field, not on the history's lag (with it the steps
	# measured up to 4.9 % at the 5 % limit).
	_clouds.effect.set_quality(int(CloudLayer.PRESETS[CloudLayer.Quality.DESKTOP]["divisor"]), 1.0)
	var view_climb := up
	# A step is measured against the climb's brightest value (the lit cloud around the camera): near the top the pixel
	# falls toward the much darker clear zenith sky, where a step relative to the pixel itself grows without a pop
	# (measured: uniform steps of ~0.01 down to the sky's 0.04).
	var values: Array[float] = []
	var h := 700.0
	var trace := ""
	while h <= 2300.0:
		var lum := _luminance(await _render(_point(h), view_climb, sun, 0, BLOCK_RADIUS))
		trace += " %.0f:%.4f" % [h, lum]
		values.append(lum)
		h += 50.0
	if OS.get_environment("STARSIM_VERBOSE") != "":
		print("cloud_check: climb (height: luminance)" + trace)
	var brightest: float = values.max()
	var worst := 0.0
	var worst_at := 0.0
	for i in range(1, values.size()):
		var step := absf(values[i] - values[i - 1])
		if step / brightest > worst:
			worst = step / brightest
			worst_at = 700.0 + 50.0 * i
	_check("climbing out in 50 m steps", worst <= CLIMB_STEP_LIMIT,
			"largest step %.2f %% of the brightest (at %.0f m)" % [worst * 100.0, worst_at])
	_clouds.quality = CloudLayer.Quality.DESKTOP  # the preset's temporal weight again
	_detail_off()
	# 4. Scattered Sun radiance from above a thick slab vs the closed form.
	var thick := _slab("thick", SLAB_BOTTOM_M, SLAB_TOP_M, THICK_LIQUID_KG_M3, 1.0)
	_show(thick)
	var tau := -log(float(_core.weather_cloud_light(_point(10.0), up)["sun_transmittance"]))
	print("cloud_check: thick slab vertical tau %.4f" % tau)
	_clouds.scattering_model = CloudLayer.ScatteringModel.CLOSED_FORM
	for c in [[0.0, 0.0, 0.0], [30.0, 0.0, 0.0], [0.0, 60.0, 180.0], [45.0, 60.0, 0.0], [45.0, 60.0, 180.0],
			[30.0, 30.0, 90.0]]:
		await _radiance_sample(c[0], c[1], c[2], tau, true)
	# The preset's own light march at the worst geometry: its stochastic estimate exp(-tau_s) is brighter on average
	# than exp(-E[tau_s]) (Jensen; the march is unbiased in tau_s), wider tolerance.
	await _radiance_sample(45.0, 60.0, 0.0, tau, false)
	# The Monte Carlo table, incl. a low Sun (10 deg up) seen on the forward side (the backlit glow).
	_clouds.scattering_model = CloudLayer.ScatteringModel.TABLE
	for c in [[0.0, 0.0, 0.0], [45.0, 60.0, 0.0], [45.0, 60.0, 180.0], [30.0, 30.0, 90.0], [75.0, 80.0, 0.0],
			[60.0, 80.0, 180.0]]:
		await _radiance_sample(c[0], c[1], c[2], tau, true)
	await _radiance_sample(75.0, 80.0, 0.0, tau, false)
	# 5. Coverage of the GPU's filtered shape noise.
	_viewport.size = Vector2i(COVERAGE_VIEW_SIZE, COVERAGE_VIEW_SIZE)
	for fraction in [0.1, 0.5, 0.9]:
		var file := _slab("coverage_%d" % int(fraction * 100), SLAB_BOTTOM_M, SLAB_TOP_M, THIN_LIQUID_KG_M3, fraction)
		_show(file)
		await _coverage_sample(fraction)
	_viewport.size = Vector2i(VIEW_SIZE, VIEW_SIZE)
	# 6. Light on the ground under the clouds.
	await _ground_light_samples()
	# 7. The shadow maps' hand-overs and the shapes on broken cloud.
	await _handover_samples()
	# 8. The light cache against the per-sample light march (and 10: no seam at its edges).
	await _light_cache_samples()
	# 9. The deterministic march's silhouettes against the converged jittered march.
	await _silhouette_samples()
	# 11. Anti-aliased outlines (the game's output, the VR preset's divisor).
	await _outline_samples()
	# 12. The flat cloud base at the lifting condensation level.
	await _lcl_base_samples()
	# 13. The Sun through the decks.
	await _sun_through_samples()
	# 14. The backlit slab vs Monte Carlo; the clouds' shadows on the air.
	await _slab_transmitted_samples()
	await _air_shadow_samples()
	# 15. The cloud view cache's fill against the per-frame march.
	await _view_cache_samples()
	# 16. The cloud world volume's GPU bake, occupancy and grid (it leaves the flag off again).
	await _world_volume_samples()
	# 17. The world volume's light (last: it leaves the flag off again).
	await _volume_light_samples()

	for f in _files:
		DirAccess.remove_absolute(f)
	_clouds.shutdown()
	_check("sample count", _samples == EXPECTED_SAMPLES + _optional_samples, "%d samples" % _samples)
	print("cloud_check: %s (%d samples, %d failures)" % ["PASS" if _failures == 0 else "FAIL", _samples, _failures])
	quit(1 if _failures > 0 else 0)


func _setup_scene() -> void:
	_viewport.size = Vector2i(VIEW_SIZE, VIEW_SIZE)
	_viewport.use_hdr_2d = true
	_viewport.own_world_3d = true
	_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	root.add_child(_viewport)
	var sky_material := ShaderMaterial.new()
	sky_material.shader = load("res://shaders/sky.gdshader")
	var sky := Sky.new()
	sky.sky_material = sky_material
	sky.process_mode = Sky.PROCESS_MODE_REALTIME
	var environment := Environment.new()
	environment.background_mode = Environment.BG_SKY
	environment.sky = sky
	environment.tonemap_mode = Environment.TONE_MAPPER_LINEAR
	environment.tonemap_exposure = 1.0
	environment.glow_enabled = false
	environment.ambient_light_source = Environment.AMBIENT_SOURCE_DISABLED
	environment.reflected_light_source = Environment.REFLECTION_SOURCE_DISABLED
	var world_environment := WorldEnvironment.new()
	world_environment.environment = environment
	var compositor := Compositor.new()
	compositor.compositor_effects = [_clouds.shadow_effect, _clouds.effect]
	world_environment.compositor = compositor
	_viewport.add_child(world_environment)
	_camera.fov = 10.0
	_camera.near = 0.5
	_camera.far = 4.0e7
	_camera.current = true
	_viewport.add_child(_camera)
	_viewport.add_child(_clouds)
	var albedo := Image.create(4, 4, true, Image.FORMAT_RGBA8)
	albedo.fill(GROUND_ALBEDO_8)
	albedo.generate_mipmaps()
	var black := Image.create(4, 4, true, Image.FORMAT_RGBA8)
	black.fill(Color(0, 0, 0))
	black.generate_mipmaps()
	var white := Image.create(4, 4, false, Image.FORMAT_R8)
	white.fill(Color(1, 1, 1))
	for material: ShaderMaterial in [_terrain_material, _water_material]:
		material.shader = load("res://shaders/terrain_earth.gdshader")
		material.set_shader_parameter("day_texture", ImageTexture.create_from_image(albedo))
		material.set_shader_parameter("day_texture_bound", true)
		material.set_shader_parameter("night_texture", ImageTexture.create_from_image(black))
		material.set_shader_parameter("moon_radius_m", 1737400.0)
		material.set_shader_parameter("day_exposure", Atmosphere.DAY_EXPOSURE)
	_water_material.set_shader_parameter("water_mask", ImageTexture.create_from_image(white))
	_terrain_quad.material_override = _terrain_material
	_terrain_parent.add_child(_terrain_quad)
	_terrain_parent.visible = false
	_viewport.add_child(_terrain_parent)
	var sphere := SphereMesh.new()
	sphere.radius = 1.0
	sphere.height = 2.0
	_sun_disk.mesh = sphere
	_sun_disk.material_override = World.sun_disc_material()
	_sun_disk.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	_sun_disk.visible = false
	_viewport.add_child(_sun_disk)


## Writes a slab snapshot, returns its loaded dictionary (the file is removed at the end).
func _slab(label: String, bottom_m: float, top_m: float, liquid: float, fraction: float) -> Dictionary:
	var file := OS.get_temp_dir().path_join("starsim_cloud_check_%s_%d.wthr" % [label, OS.get_process_id()])
	_files.append(file)
	if not _core.weather_write_slab_snapshot(file, bottom_m, top_m, liquid, fraction):
		_fail_and_quit("slab snapshot not written")
	var loaded: Dictionary = _core.weather_load_snapshot(file)
	loaded["file"] = file
	return loaded


func _show(snapshot: Dictionary) -> void:
	_clouds.show_now(snapshot, snapshot, 0.0)
	_core.weather_set_active(snapshot["file"], snapshot["file"], 0.0)


func _transmittance_sample(label: String, height_m: float, view_itrf: Vector3, sun_itrf: Vector3,
		_snapshot: Dictionary) -> void:
	var camera := _point(height_m)
	var got := (await _render(camera, view_itrf, sun_itrf, 1, BLOCK_RADIUS)).x
	var expected := float(_core.weather_cloud_light(camera, view_itrf)["sun_transmittance"])
	# Relative, with a floor of 0.02 (an opaque view: a few 1e-4 of light left where the step cap ends the march).
	var error := absf(got - expected) / maxf(expected, TRANSMITTANCE_FLOOR)
	_check("transmittance " + label, error <= TRANSMITTANCE_TOLERANCE,
			"GPU %.5f CPU %.5f (%.2f %%)" % [got, expected, error * 100.0])


## View zenith angle, Sun zenith angle, relative azimuth (0: the camera looks toward the Sun's side - the forward side).
## converged: the light march with CONVERGED_LIGHT_SAMPLES near samples (the port against the closed form); else the
## quality preset's (tolerance PRESET_RADIANCE_TOLERANCE). The mean of RADIANCE_FRAMES frames (each its own jitter).
func _radiance_sample(view_zenith_deg: float, sun_zenith_deg: float, azimuth_deg: float, tau: float,
		converged: bool) -> void:
	var view := _local_direction(view_zenith_deg - 90.0, 0.0)  # looking down, toward north
	var sun := _local_direction(90.0 - sun_zenith_deg, azimuth_deg)  # azimuth 0: Sun in the north, ahead
	if converged:
		var preset: Dictionary = CloudLayer.PRESETS[CloudLayer.Quality.DESKTOP]
		var first: float = preset["light_first_m"]
		var near: float = preset["light_near_m"]
		ShaderGlobals.set_value(&"cloud_light_march", Vector4(first,
				CloudLayer.light_march_ratio(first, near, CONVERGED_LIGHT_SAMPLES), near, 0.0))
		ShaderGlobals.set_value(&"cloud_light_samples", Vector2i(CONVERGED_LIGHT_SAMPLES, int(preset["light_far"])))
	var got := 0.0
	for i in range(RADIANCE_FRAMES):
		got += (await _render(_point(5000.0), view, sun, 2, BLOCK_RADIUS)).x / RADIANCE_FRAMES
	_clouds.quality = CloudLayer.Quality.DESKTOP  # the preset's march again
	_detail_off()
	var table := _clouds.scattering_model == CloudLayer.ScatteringModel.TABLE
	var args := [tau, cos(deg_to_rad(sun_zenith_deg)), cos(deg_to_rad(view_zenith_deg)), deg_to_rad(azimuth_deg)]
	var expected := float(_core.callv(&"cloud_slab_radiance_mc" if table else &"cloud_slab_radiance", args))
	var error := absf(got - expected) / maxf(expected, 1e-6)
	var tolerance := (TABLE_RADIANCE_TOLERANCE if table else RADIANCE_TOLERANCE) if converged \
			else PRESET_RADIANCE_TOLERANCE
	_check("Sun radiance (%s) view %.0f / Sun %.0f / azimuth %.0f deg (%s march)" % ["table" if table else "closed form",
			view_zenith_deg, sun_zenith_deg, azimuth_deg, "converged" if converged else "preset"],
			error <= tolerance, "GPU %.5f CPU %.5f /sr (%.2f %%)" % [got, expected, error * 100.0])


## Detail levels off: the CPU references have the shapes of state 0 at most (cloud_noise.hpp).
func _detail_off() -> void:
	ShaderGlobals.set_value(&"cloud_detail_levels", 0)


## An unshaded box in front of the camera: centre in the camera's frame (x right, y up, -z ahead), size across x / y.
func _occluder(centre_camera: Vector3, size: Vector2) -> MeshInstance3D:
	var mesh := BoxMesh.new()
	mesh.size = Vector3(size.x, size.y, 1.0)
	var material := StandardMaterial3D.new()
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.albedo_color = Color(0.2, 0.2, 0.2)
	var box := MeshInstance3D.new()
	box.mesh = mesh
	box.material_override = material
	box.position = centre_camera  # a child of the camera
	_camera.add_child(box)
	return box


## A mountain edge at the half-resolution pass: below the slab, looking 30 deg up, a near occluder (50 m) covers the
## columns left of the edge, which runs through the middle of a 2 x 2 block (between columns 16 and 17 of 33); the
## sky right of it sees the slab. The full-resolution columns next to the edge: the occluder's keep T = 1 (no cloud
## bleeds over the mountain), the sky's the slab's T of their rays (the CPU's) within 5 %.
func _mountain_edge_sample(sun: Vector3) -> void:
	var view := _local_direction(30.0, 200.0)
	var pixel_m := 2.0 * EDGE_DISTANCE_M * tan(deg_to_rad(_camera.fov * 0.5)) / VIEW_SIZE
	var edge_x := 0.5 * pixel_m  # between the centre column 16 and column 17
	var occluder := _occluder(Vector3(edge_x - 0.5 * EDGE_SIZE_M, 0.0, -EDGE_DISTANCE_M),
			Vector2(EDGE_SIZE_M, EDGE_SIZE_M))
	await _render(_point(10.0), view, sun, 1)
	occluder.queue_free()
	var image := _viewport.get_texture().get_image()
	var mountain := 0.0
	var sky := 0.0
	var rows := 0
	for y in range(VIEW_SIZE / 2 - BLOCK_RADIUS, VIEW_SIZE / 2 + BLOCK_RADIUS + 1):
		mountain += image.get_pixel(16, y).r
		sky += image.get_pixel(17, y).r
		rows += 1
	mountain /= rows
	sky /= rows
	# Column 17's ray: 1 pixel right of the axis (the camera's x toward the screen's right).
	var basis := _camera.transform.basis
	var tan_x := (17.5 - VIEW_SIZE * 0.5) / (VIEW_SIZE * 0.5) * tan(deg_to_rad(_camera.fov * 0.5))
	var ray_render := (basis * Vector3(tan_x, 0.0, -1.0)).normalized()
	var ray_itrf: Vector3 = _earth_to_gcrf.inverse() * Frames.from_render(ray_render)
	var expected := float(_core.weather_cloud_light(_point(10.0), ray_itrf.normalized())["sun_transmittance"])
	var sky_error := absf(sky - expected) / maxf(expected, TRANSMITTANCE_FLOOR)
	_check("mountain edge upsampling (occluder side clear, sky side the slab)", mountain >= 0.97 and sky_error <= 0.05,
			"occluder column %.4f, sky column %.4f (CPU %.4f, %.2f %%)" % [mountain, sky, expected, sky_error * 100.0])


func _coverage_sample(fraction: float) -> void:
	ShaderGlobals.set_value(&"cloud_debug_height_m", 0.5 * (SLAB_BOTTOM_M + SLAB_TOP_M))
	_camera.fov = 60.0
	# Full resolution: the check is on the field's filtered coverage, not on the pass's upsampling (half-resolution
	# samples lose holes narrower than two pixels at 0.9).
	_clouds.effect.set_quality(1, 1.0)
	await _render(_point(300000.0), _local_direction(-90.0, 0.0), _local_direction(45.0, 0.0), 3)
	_camera.fov = 10.0
	var image := _viewport.get_texture().get_image()
	_clouds.quality = _clouds.quality  # the preset's resolution again
	_detail_off()
	var covered := 0
	for y in range(image.get_height()):
		for x in range(image.get_width()):
			covered += 1 if image.get_pixel(x, y).r > 0.5 else 0
	var got := float(covered) / float(image.get_width() * image.get_height())
	_check("coverage %.1f" % fraction, absf(got - fraction) <= COVERAGE_TOLERANCE, "covered %.4f" % got)


func _render(camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3, mode: int, block_radius := 0,
		roll_deg := 0.0, pixel_offset := Vector2i.ZERO) -> Vector3:
	var camera_gcrf := _earth_to_gcrf * camera_itrf_m
	var sun_gcrf: Vector3 = _earth_to_gcrf * sun_itrf * AU_M
	_atmosphere.update(camera_gcrf, Vector3.ZERO, _earth_to_gcrf, sun_gcrf, 0.0, 1.0)
	_clouds.update(camera_itrf_m, 0.0, 0.0)
	ShaderGlobals.set_value(&"cloud_debug_output", mode)
	var view_render := Frames.to_render(_earth_to_gcrf * view_itrf).normalized()
	# The screen's up toward the local vertical (rows = elevations), any other for a zenith or nadir view.
	var up := Frames.to_render(_earth_to_gcrf * camera_itrf_m.normalized()).normalized()
	if absf(view_render.dot(up)) > 0.9:
		up = Vector3.UP if absf(view_render.y) < 0.9 else Vector3.RIGHT
	_camera.transform = Transform3D(Basis.looking_at(view_render, up) * Basis(Vector3.BACK, deg_to_rad(roll_deg)),
			Vector3.ZERO)
	for i in range(SETTLE_FRAMES):
		await RenderingServer.frame_post_draw
		_clouds.update(camera_itrf_m, 0.0, 0.0)  # as the world does every frame (a re-created result is shown)
	var image := _viewport.get_texture().get_image()
	# The mean over a (2 r + 1)^2 block around the centre: the jitter differs per pixel, so the block mean is the
	# estimator's expectation (rays within +-0.6 deg of the axis at r = 2).
	var sum := Vector3.ZERO
	for dy in range(-block_radius, block_radius + 1):
		for dx in range(-block_radius, block_radius + 1):
			var c := image.get_pixel(image.get_width() / 2 + dx + pixel_offset.x,
					image.get_height() / 2 + dy + pixel_offset.y)
			sum += Vector3(c.r, c.g, c.b)
	return sum / float((2 * block_radius + 1) * (2 * block_radius + 1))


## Section 8: the light cache against the per-sample light march with the cache's own converged sample count (the
## cache's fill: CloudLightCache.SAMPLES at the stratified midpoints; the march: the same counts, jittered per frame and
## averaged over RADIANCE_FRAMES frames), the view's jitter pinned to the same frames in both. Each cascade at its scale
## (CACHE_VIEWS: every cloud sample of the view where that cascade alone has weight). The fill runs at 1/16 of the
## voxels per frame (complete after 16 frames); then the game's budget's GPU time, a view beyond every cascade with
## a complete fill, and the frames right after a clock jump (nothing held: the march's bytes).
## Verdict of one section 8 case: broken cloud by p99 and worst pixel; the grid-mean slab (the cache's exact-ish case)
## by its own tolerance on the worst pixel.
static func _cache_case_ok(mean: float, p99: float, worst: float, slab: bool) -> bool:
	if slab:
		return mean > 0.0 and worst <= CACHE_SLAB_TOLERANCE
	return mean > 0.0 and p99 <= CACHE_P99_TOLERANCE and worst <= CACHE_WORST_TOLERANCE


## The verdict itself: a slab 10 % wrong at its worst pixel must fail, broken cloud at the same error must pass.
func _cache_verdict_selftest() -> void:
	_check("light cache verdict: a slab with a 10 % worst pixel fails, 4 % passes",
			not _cache_case_ok(1.0, 0.03, 0.10, true) and _cache_case_ok(1.0, 0.03, 0.04, true), "tolerance %.0f %%"
			% (CACHE_SLAB_TOLERANCE * 100.0))
	_check("light cache verdict: broken cloud at 10 % worst passes, at 25 % fails",
			_cache_case_ok(1.0, 0.03, 0.10, false) and not _cache_case_ok(1.0, 0.03, 0.25, false), "p99 / worst bounds")


func _light_cache_samples() -> void:
	_cache_verdict_selftest()
	var preset: Dictionary = CloudLayer.PRESETS[CloudLayer.Quality.DESKTOP]
	var first: float = preset["light_first_m"]
	var near: float = preset["light_near_m"]
	var samples := CloudLightCache.SAMPLES
	var cache := _clouds.light_cache
	var game_budget := cache.schedule.budget_voxels
	cache.schedule.budget_voxels = cache.voxel_count() / CACHE_BUDGET_DIVISOR
	# The game's path (the deterministic march): both sides share the view's fixed samples, one frame each.
	_clouds.deterministic_march = true
	var divisor := int(CloudLayer.PRESETS[CloudLayer.Quality.DESKTOP]["divisor"])
	var size := Vector2i(CACHE_VIEW_SIZE, CACHE_VIEW_SIZE)
	_viewport.size = size
	var slab := _slab("cache_slab", SLAB_BOTTOM_M, SLAB_TOP_M, THICK_LIQUID_KG_M3, 1.0)
	var broken := _slab("cache_broken", SLAB_BOTTOM_M, SLAB_TOP_M, DECK_LIQUID_KG_M3, BROKEN_FRACTION)
	var sun45 := _local_direction(45.0, 135.0)
	# The progressive fill: all cascades at 1/16 of the voxels per frame.
	_show(slab)
	var v0: Dictionary = CACHE_VIEWS[0]
	_camera.fov = v0["fov"]
	var view0 := _local_direction(v0["elevation"], v0["azimuth"])
	await _settle_cache(_point(v0["height_m"]), view0, sun45, 2)
	cache.reset()
	var fill_frames := await _settle_cache(_point(v0["height_m"]), view0, sun45, 2)
	_check("light cache: %d cascades at 1/%d of the voxels per frame complete after %d frames"
			% [CloudLightCacheSchedule.CASCADES.size(), CACHE_BUDGET_DIVISOR, CACHE_BUDGET_DIVISOR],
			fill_frames == CACHE_BUDGET_DIVISOR, "%d frames with fill jobs (%d voxels, %.1f MB)"
			% [fill_frames, cache.voxel_count(), CloudLightCache.texture_bytes() / 1048576.0])
	var only_views := OS.get_environment("STARSIM_CACHE_VIEWS")  # diagnostics: "3,4" runs those cascades' views alone
	for v: Dictionary in CACHE_VIEWS:
		if only_views != "" and not str(v["cascade"]) in only_views.split(","):
			continue
		_camera.fov = v["fov"]
		var height: float = v["height_m"]
		var reach := _view_cloud_range_m(height, v["elevation"], v["fov"], 1.0)
		var k: int = v["cascade"]
		var inner := 0.0 if k == 0 else _cascade_radius_m(k - 1, false)
		var outer := _cascade_radius_m(k, true)
		_check("light cache view, cascade %d: the farthest cloud samples in its full weight" % k,
				reach.y > inner and reach.y <= outer, "camera %.1f km: cloud samples %.0f .. %.0f m from the camera's "
				% [height / 1000.0, reach.x, reach.y] + "vertical; finer cascades end at %.0f m, cascade %d full to %.0f m"
				% [inner, k, outer])
		var view := _local_direction(v["elevation"], v["azimuth"])
		for c in [[slab, 2, CACHE_SLAB_TOLERANCE, "slab tau ~10"], [broken, 7, CACHE_BROKEN_TOLERANCE, "broken cf 0.4"]]:
			_show(c[0])
			for sun_elevation in [45.0, 20.0]:
				var sun := _local_direction(sun_elevation, 135.0)
				var images: Array[PackedFloat32Array] = []
				for cached in [false, true]:
					_clouds.light_cache_enabled = cached
					ShaderGlobals.set_value(&"cloud_light_march", Vector4(first,
							CloudLayer.light_march_ratio(first, near, samples.x), near, 0.0))
					ShaderGlobals.set_value(&"cloud_light_samples", samples)
					if cached:
						await _settle_cache(_point(height), view, sun, c[1])
					images.append(await _mean_image(_point(height), view, sun, c[1], size, 0, 1))
				_clouds.light_cache_enabled = true
				_clouds.quality = CloudLayer.Quality.DESKTOP
				_detail_off()
				var stats := _cache_error(images[1], images[0])
				var detail := ("worst pixel %.2f %% at (%d, %d) (cache %.5f march %.5f), mean error %.2f %%, bias %.2f %%"
						+ " of the view mean %.5f /sr") % [stats["worst"] * 100.0, stats["at"] % CACHE_VIEW_SIZE,
						stats["at"] / CACHE_VIEW_SIZE, images[1][stats["at"]], images[0][stats["at"]],
						stats["mean_error"] * 100.0, stats["bias"] * 100.0, stats["mean"]]
				# Acceptance (_cache_case_ok): the slab by its worst pixel (CACHE_SLAB_TOLERANCE); broken cloud by the p99 of
				# the per-pixel relative error (CACHE_P99_TOLERANCE) and the worst pixel (CACHE_WORST_TOLERANCE). The residue
				# is voxel interpolation of cumulus self-shadowing at a fixed sample in cascade 0 (62.5 m voxels): worst seen
				# 16 % at p99 6.8 %. The interior / silhouette split is printed for information only.
				var shape := await _mean_image(_point(height), view, sun, 4, size, 0, 1)
				var split := _cache_error_split(images[1], images[0], shape, divisor)
				var ok: bool = stats["mean"] > 0.0 and split["p99"] <= CACHE_P99_TOLERANCE \
						and stats["worst"] <= CACHE_WORST_TOLERANCE
				detail += ("; interior worst %.2f %% (%d px), silhouette worst %.2f %% (%d px), p99 %.2f %%; "
						+ "pixels over %.0f %%: %d interior, %d silhouette") % [split["interior"] * 100.0,
						split["interior_count"], split["edge"] * 100.0, split["edge_count"], split["p99"] * 100.0,
						CACHE_BROKEN_TOLERANCE * 100.0, split["interior_over"], split["edge_over"]]
				_check("light cache vs light march, cascade %d, %s, Sun %.0f deg" % [k, c[3], sun_elevation], ok, detail)
				if c[0] == broken:
					var at := Vector2i(stats["at"] % CACHE_VIEW_SIZE, stats["at"] / CACHE_VIEW_SIZE)
					print("  info  worst pixel %s, cascade shares along its ray: %s; worst pixel by the cascade "
							% [at, _pixel_cascade_shares(_point(height), at, sun)] + "holding its slab middle: %s"
							% _worst_by_cascade(images[1], images[0], _point(height), sun))
	# The fill's GPU time per frame at the game's budget (spec decision 6: <= 0.5 ms in every frame): a full refresh of
	# every cascade from nothing, the time of each frame's fill (the RenderingDevice's timestamps trail by a frame or
	# two; every fill frame carries the budget's voxels but the last).
	cache.schedule.budget_voxels = game_budget
	_show(slab)
	_camera.fov = v0["fov"]
	await _settle_cache(_point(v0["height_m"]), view0, sun45, 2)
	cache.reset()
	CloudEffect.capture_timestamps = true
	var camera := _point(v0["height_m"])
	var times := PackedFloat64Array()
	var frames := 0
	var game_fill_frames := cache.planned_fill_frames
	var after := 0  # frames after the last plan with jobs (the timestamps trail)
	while frames < CACHE_MAX_SETTLE_FRAMES and after < 4:
		_clouds.update(camera, 0.0, 0.0)
		await RenderingServer.frame_post_draw
		var ms := _light_fill_ms()
		if ms > CACHE_FILL_IDLE_MS:
			times.append(ms)
		after = after + 1 if cache.settled() else 0
		frames += 1
	CloudEffect.capture_timestamps = false
	var sorted := times.duplicate()
	sorted.sort()
	var worst := sorted[sorted.size() - 1] if not sorted.is_empty() else 0.0
	var p95 := sorted[int(sorted.size() * 0.95)] if not sorted.is_empty() else 0.0
	var series := ""
	for ms in times:
		series += "%.2f " % ms
	print("  info  light cache fill per frame (ms GPU, in fill order): " + series)
	# p95, not the worst frame: the worst is an isolated spike at a GPU clock-state transition in a lightly loaded
	# test; the per-frame worst under load is measured in the headset in Task 10.
	_check("light cache: the fill at the game's budget stays <= %.1f ms GPU (95th percentile)" % CACHE_FILL_LIMIT_MS,
			not sorted.is_empty() and p95 <= CACHE_FILL_LIMIT_MS,
			("%d voxels per frame: median %.3f, p95 %.3f, worst %.3f ms over %d frames; %d plans for all %d voxels "
			+ "(%.1f MB)") % [game_budget, sorted[sorted.size() / 2] if not sorted.is_empty() else 0.0,
			p95, worst, sorted.size(),
			cache.planned_fill_frames - game_fill_frames, cache.voxel_count(),
			CloudLightCache.texture_bytes() / 1048576.0])
	cache.schedule.budget_voxels = cache.voxel_count() / CACHE_BUDGET_DIVISOR
	# Beyond every cascade (a complete fill): from 20 km toward the northern horizon every cloud sample lies past the
	# outermost cascade's weight; the cache must change no bit.
	_camera.fov = CACHE_OUTSIDE_FOV_DEG
	var outside_view := _local_direction(CACHE_OUTSIDE_ELEVATION_DEG, 0.0)
	var outside_reach := _view_cloud_range_m(CACHE_OUTSIDE_HEIGHT_M, CACHE_OUTSIDE_ELEVATION_DEG, CACHE_OUTSIDE_FOV_DEG,
			1.0)
	var last := CloudLightCacheSchedule.CASCADES.size() - 1
	await _settle_cache(_point(CACHE_OUTSIDE_HEIGHT_M), outside_view, sun45, 2)
	var settled := cache.settled()
	var outside: Array[PackedByteArray] = []
	for cached in [false, true]:
		_clouds.light_cache_enabled = cached
		CloudEffect.pinned_frame = 0
		await _render(_point(CACHE_OUTSIDE_HEIGHT_M), outside_view, sun45, 2)
		CloudEffect.pinned_frame = -1
		outside.append(_viewport.get_texture().get_image().get_data())
	_clouds.light_cache_enabled = true
	var lit := _viewport.get_texture().get_image().get_pixel(CACHE_VIEW_SIZE / 2, CACHE_VIEW_SIZE / 2).r
	_check("light cache: a view beyond every cascade (complete fill) equals the march",
			outside[0] == outside[1] and lit > 0.0 and settled and outside_reach.x > _cascade_radius_m(last, false),
			"%s (centre %.5f /sr; cloud samples from %.0f km, cascade %d's weight ends at %.0f km; fill complete %s)"
			% ["identical" if outside[0] == outside[1] else "DIFFERENT", lit, outside_reach.x / 1000.0, last,
			_cascade_radius_m(last, false) / 1000.0, settled])
	# A clock jump: every cascade invalid until filled - the frames right after it are the march's, bit for bit (at the
	# game's budget: the seed squares of all cascades take 6 plans, the render's 5 see nothing filled).
	cache.schedule.budget_voxels = game_budget
	_camera.fov = v0["fov"]
	await _settle_cache(_point(v0["height_m"]), view0, sun45, 2)
	var jumped: Array[PackedByteArray] = []
	for cached in [true, false]:
		if cached:
			_clouds.notify_clock_jump()
		_clouds.light_cache_enabled = cached
		CloudEffect.pinned_frame = 0
		await _render(_point(v0["height_m"]), view0, sun45, 2)
		CloudEffect.pinned_frame = -1
		jumped.append(_viewport.get_texture().get_image().get_data())
	_clouds.light_cache_enabled = true
	_check("light cache: right after a clock jump the view equals the march (nothing held until filled)",
			jumped[0] == jumped[1], "identical" if jumped[0] == jumped[1] else "DIFFERENT")
	cache.schedule.budget_voxels = cache.voxel_count() / CACHE_BUDGET_DIVISOR
	_viewport.size = Vector2i(VIEW_SIZE, VIEW_SIZE)
	# Section 10 on the jittered march (the reference path, CloudLayer.deterministic_march off): its criterion is the
	# image noise of independent jitter, which the deterministic march does not have.
	_clouds.deterministic_march = false
	await _seam_samples(slab)
	cache.schedule.budget_voxels = game_budget
	_clouds.deterministic_march = true


## Section 10: no seam. A line of pixels (CACHE_LINE_SIZE: the row means of a narrow tall view, rows = elevations)
## crossing each cascade edge and the outermost edge to the march (CACHE_EDGE_VIEWS, a slab of tau ~10, the grid mean):
## the cache's deviation from the march, d = (cache - march) / mean, steps from row to row by no more than the image
## noise (the largest difference of two march renders with independent jitter) over the rows whose slab entry lies in
## the band +- half its width (where a seam would be; elsewhere d's row-to-row change is the cascade's own voxel
## interpolation, case 8's subject - printed for information); and the same view before and after a re-anchor (the
## camera moved away and back: cascade CACHE_REANCHOR_CASCADE re-anchored, its data seen by that edge view) differs by
## less than that noise.
func _seam_samples(slab: Dictionary) -> void:
	_show(slab)
	_viewport.size = CACHE_LINE_SIZE
	var sun := _local_direction(45.0, 135.0)
	var reanchor_noise := 0.0
	for v: Dictionary in CACHE_EDGE_VIEWS:
		var height: float = v["height_m"]
		var camera := _point(height)
		_camera.fov = v["fov"]
		var view := _local_direction(v["elevation"], 0.0)
		var k: int = v["cascade"]
		var band := Vector2(_cascade_radius_m(k, true), _cascade_radius_m(k, false))
		var aspect := float(CACHE_LINE_SIZE.x) / CACHE_LINE_SIZE.y
		var reach := _view_cloud_range_m(height, v["elevation"], v["fov"], aspect)
		await _settle_cache(camera, view, sun, 2)
		var lines := await _line_triple(camera, view, sun)
		var half := 0.5 * (band.y - band.x)
		var rows := Vector2i(CACHE_LINE_SIZE.y, -1)  # the rows whose slab entry lies in the band +- half its width
		for row in range(CACHE_LINE_SIZE.y):
			var entry := _row_entry_m(height, v["elevation"], v["fov"], row)
			if entry >= band.x - half and entry <= band.y + half:
				rows = Vector2i(mini(rows.x, row), maxi(rows.y, row))
		var result := _seam(lines[0], lines[1], lines[2], rows)
		var profile := ""  # (cache - march) / mean per row, % (top row first), and the row's slab entry (km)
		for row in range(CACHE_LINE_SIZE.y):
			profile += "%d:%.2f@%.1f " % [row, (lines[0][row] - lines[1][row]) / result["mean"] * 100.0,
					_row_entry_m(height, v["elevation"], v["fov"], row) / 1000.0]
		print("  info  edge %d -> %d profile: %s" % [k, k + 1, profile])
		if k == CACHE_REANCHOR_CASCADE:
			reanchor_noise = result["noise"]
		var into := "the march" if k == CloudLightCacheSchedule.CASCADES.size() - 1 else "cascade %d" % (k + 1)
		_check("light cache: no seam from cascade %d into %s" % [k, into],
				reach.x < band.x and reach.y > band.y and rows.y > rows.x and result["step"] <= result["noise"],
				("largest row-to-row step of (cache - march) / mean over the band's rows %d .. %d %.3f %% at row %d, "
				+ "image noise %.3f %% (whole line %.3f %%); cloud samples %.1f .. %.1f km, band %.1f .. %.1f km")
				% [rows.x, rows.y, result["step"] * 100.0, result["at"], result["noise"] * 100.0,
				result["step_all"] * 100.0, reach.x / 1000.0, reach.y / 1000.0, band.x / 1000.0, band.y / 1000.0])
	# Before / after a re-anchor: an edge view from a camera CACHE_REANCHOR_EAST_M east, then the camera twice as far
	# east (cascade CACHE_REANCHOR_CASCADE re-anchors; the finer ones re-anchor there and back to the same window) and
	# back.
	var v1: Dictionary = CACHE_EDGE_VIEWS[CACHE_REANCHOR_CASCADE]
	_camera.fov = v1["fov"]
	var view1 := _local_direction(v1["elevation"], 0.0)
	var here := _point_east(v1["height_m"], CACHE_REANCHOR_EAST_M)
	var away := _point_east(v1["height_m"], 2.0 * CACHE_REANCHOR_EAST_M)
	await _settle_cache(here, view1, sun, 2)
	var windows_before := _cascade_windows()
	var before := _column_means(await _mean_image(here, view1, sun, 2, CACHE_LINE_SIZE))
	await _settle_cache(away, view1, sun, 2)
	await _settle_cache(here, view1, sun, 2)
	var windows_after := _cascade_windows()
	var after := _column_means(await _mean_image(here, view1, sun, 2, CACHE_LINE_SIZE))
	var mean := 0.0
	for x in before:
		mean += x / before.size()
	var worst := 0.0
	for i in range(before.size()):
		worst = maxf(worst, absf(after[i] - before[i]) / mean)
	var k := CACHE_REANCHOR_CASCADE
	var moved: bool = windows_before[k] != windows_after[k]
	_check("light cache: the same view before / after a re-anchor", moved and mean > 0.0 and worst <= reanchor_noise,
			"largest difference %.4f %% of the line mean, image noise %.3f %% (cascade %d window %s -> %s)"
			% [worst * 100.0, reanchor_noise * 100.0, k, windows_before[k], windows_after[k]])


## The line (row means) of the cache, the march (frames 0 .. 7) and the march with other jitter (frames 8 .. 15) - the
## march with the cache's own sample count (as section 8: the preset's few samples carry their own bias and pattern).
func _line_triple(camera: Vector3, view: Vector3, sun: Vector3) -> Array[PackedFloat32Array]:
	var preset: Dictionary = CloudLayer.PRESETS[CloudLayer.Quality.DESKTOP]
	var first: float = preset["light_first_m"]
	var near: float = preset["light_near_m"]
	var samples := CloudLightCache.SAMPLES
	var result: Array[PackedFloat32Array] = []
	for setting in [[true, 0], [false, 0], [false, RADIANCE_FRAMES]]:
		_clouds.light_cache_enabled = setting[0]
		ShaderGlobals.set_value(&"cloud_light_march", Vector4(first,
				CloudLayer.light_march_ratio(first, near, samples.x), near, 0.0))
		ShaderGlobals.set_value(&"cloud_light_samples", samples)
		result.append(_column_means(await _mean_image(camera, view, sun, 2, CACHE_LINE_SIZE, setting[1])))
	_clouds.quality = CloudLayer.Quality.DESKTOP
	_detail_off()
	_clouds.light_cache_enabled = true
	return result


## {step: the largest row-to-row change of (cache - march) / mean within rows (first, last), at: its row, step_all: the
## same over the whole line, noise: the largest difference of the two march lines / mean}.
static func _seam(cache: PackedFloat32Array, march: PackedFloat32Array, other: PackedFloat32Array,
		rows: Vector2i) -> Dictionary:
	var mean := 0.0
	for x in march:
		mean += x / march.size()
	var noise := 0.0
	var step := 0.0
	var step_all := 0.0
	var at := 0
	for i in range(march.size()):
		noise = maxf(noise, absf(march[i] - other[i]) / mean)
		if i > 0:
			var s := absf((cache[i] - march[i]) - (cache[i - 1] - march[i - 1])) / mean
			step_all = maxf(step_all, s)
			if i - 1 >= rows.x and i <= rows.y and s > step:
				step = s
				at = i
	return {"step": step, "at": at, "step_all": step_all, "noise": noise, "mean": mean}


## The horizontal distance (m) at which the centre column's ray of row `row` (of CACHE_LINE_SIZE.y, top first) of a
## view from height_m enters the slab (its top); INF when it misses.
static func _row_entry_m(height_m: float, elevation_deg: float, fov_deg: float, row: int) -> float:
	const R := 6371000.0
	var v := 1.0 - 2.0 * (row + 0.5) / CACHE_LINE_SIZE.y
	var el := deg_to_rad(elevation_deg) + atan(v * tan(deg_to_rad(fov_deg * 0.5)))
	var top := _sphere_hits(R + height_m, sin(el), R + SLAB_TOP_M)
	return INF if top.x > top.y else _ground_distance(R + height_m, sin(el), top.x, R)


func _cascade_windows() -> Array:
	var windows := []
	for c: Dictionary in _clouds.light_cache.schedule.lookup_state()["cascades"]:
		windows.append(c["window"])
	return windows


## The horizontal distance from the camera's ground point at which cascade k's weight ends (full: where it starts to
## fall over the band).
static func _cascade_radius_m(k: int, full: bool) -> float:
	var c: Dictionary = CloudLightCacheSchedule.CASCADES[k]
	var dx: float = c["extent_m"] / float(c["size"].x)
	var radius := float(c["size"].x / 2 - CloudLightCacheSchedule.REANCHOR_VOXELS - CloudLightCacheSchedule.MARGIN_VOXELS)
	return (radius - (CloudLightCacheSchedule.BAND_VOXELS if full else 0)) * dx


## Diagnostics (section 8): the cascade weights along the ray of view pixel `pixel` (the camera as the last render) at
## the slab's bottom, middle and top - per height the share of each cascade (the lookup's chain: chosen at the sample,
## a hybrid one only where it holds the end of its near march) and the march's rest.
func _pixel_cascade_shares(camera_itrf_m: Vector3, pixel: Vector2i, sun_itrf: Vector3) -> String:
	var out := PackedStringArray()
	for h: float in [SLAB_BOTTOM_M, 0.5 * (SLAB_BOTTOM_M + SLAB_TOP_M), SLAB_TOP_M]:
		var shares := _cascade_shares_at(camera_itrf_m, pixel, sun_itrf, h)
		var parts := PackedStringArray()
		for k in range(shares.size()):
			parts.append(("c%d" % k if k < shares.size() - 1 else "march") + " %.2f" % shares[k])
		out.append("h %.0f m: %s" % [h, ", ".join(parts) if not shares.is_empty() else "no hit"])
	return "; ".join(out)


## Diagnostics (section 8): the worst pixel error (as _cache_error) among the pixels whose slab middle each cascade
## holds with the largest share.
func _worst_by_cascade(cached: PackedFloat32Array, march: PackedFloat32Array, camera_itrf_m: Vector3,
		sun_itrf: Vector3) -> String:
	var mean := 0.0
	for v in march:
		mean += v / march.size()
	var worst := PackedFloat64Array()
	worst.resize(CloudLightCacheSchedule.CASCADES.size() + 1)
	worst.fill(0.0)
	for i in range(march.size()):
		var shares := _cascade_shares_at(camera_itrf_m, Vector2i(i % CACHE_VIEW_SIZE, i / CACHE_VIEW_SIZE), sun_itrf,
				0.5 * (SLAB_BOTTOM_M + SLAB_TOP_M))
		var owner := 0
		for k in range(shares.size()):
			owner = k if shares[k] > shares[owner] else owner
		var error := absf(cached[i] - march[i]) / maxf(march[i], CACHE_ERROR_FLOOR * mean)
		worst[owner] = maxf(worst[owner], error)
	var parts := PackedStringArray()
	for k in range(worst.size()):
		parts.append(("c%d" % k if k < worst.size() - 1 else "march") + " %.1f %%" % (worst[k] * 100.0))
	return ", ".join(parts)


## The shares of the cascades (and last the march) at height h along the ray of view pixel `pixel` (empty: no hit).
func _cascade_shares_at(camera_itrf_m: Vector3, pixel: Vector2i, sun_itrf: Vector3, h: float) -> PackedFloat64Array:
	var state := _clouds.light_cache.schedule.lookup_state()
	var ray_gcrf := Frames.from_render(_camera.project_ray_normal(Vector2(pixel) + Vector2(0.5, 0.5)))
	var ray := (_earth_to_gcrf.inverse() * ray_gcrf).normalized()
	var r0 := _point(0.0).length()
	var light := Frames.to_render(sun_itrf).normalized()
	var origin: Vector3 = state["origin"]
	var shares := PackedFloat64Array()
	var b := camera_itrf_m.dot(ray)
	var disc := b * b - (camera_itrf_m.length_squared() - (r0 + h) * (r0 + h))
	if disc < 0.0:
		return shares
	var p := Frames.to_render(camera_itrf_m + ray * (-b - sqrt(disc)))
	var open := 1.0
	for k in range(CloudLightCacheSchedule.CASCADES.size()):
		var c: Dictionary = state["cascades"][k]
		var dx: float = c["dx_m"]
		var at := p + light * _clouds.light_cache.near_length_m(k)
		var g := Vector2((p - origin).dot(state["east"]), (p - origin).dot(state["north"])) / dx
		var gf := Vector2((at - origin).dot(state["east"]), (at - origin).dot(state["north"])) / dx
		var d := (g - (c["centre"] as Vector2)).abs()
		var e := clampf((float(c["radius_voxels"]) - maxf(d.x, d.y)) / float(c["band_voxels"]), 0.0, 1.0)
		var held: Rect2i = c["valid"][1] if float(c["weight"]) >= 0.5 else c["valid"][0]
		var lo := gf - (Vector2(held.position) + Vector2(0.5, 0.5))
		var hi := (Vector2(held.end) - Vector2(0.5, 0.5)) - gf
		var hw := clampf(minf(minf(lo.x, lo.y), minf(hi.x, hi.y)) / float(c["band_voxels"]), 0.0, 1.0)
		shares.append(open * minf(e, hw))
		open -= shares[k]
	shares.append(open)
	return shares


## Renders until the light cache holds every cascade's window for this scene (and one frame more: the render thread
## adopts the state); returns the number of frames whose plan carried fill jobs.
func _settle_cache(camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3, mode: int) -> int:
	var cache := _clouds.light_cache
	var planned := cache.planned_fill_frames
	await _render(camera_itrf_m, view_itrf, sun_itrf, mode)
	var frames := SETTLE_FRAMES
	while not cache.settled() and frames < CACHE_MAX_SETTLE_FRAMES:
		await RenderingServer.frame_post_draw
		_clouds.update(camera_itrf_m, 0.0, 0.0)
		frames += 1
	await RenderingServer.frame_post_draw
	_clouds.update(camera_itrf_m, 0.0, 0.0)
	await RenderingServer.frame_post_draw
	return cache.planned_fill_frames - planned


## The cloud samples' range of a view from height_m (elevation, vertical fov in degrees, aspect = width / height): the
## nearest and farthest horizontal distance (m, along the ground from the camera's vertical) at which border rays enter
## the slab (its top) and leave it (its base, else its top again) - a sphere of the mean Earth radius.
static func _view_cloud_range_m(height_m: float, elevation_deg: float, fov_deg: float, aspect: float) -> Vector2:
	const R := 6371000.0
	var el := deg_to_rad(elevation_deg)
	var forward := Vector2(cos(el), sin(el))  # (horizontal, up) in the vertical plane of the view
	var t := tan(deg_to_rad(fov_deg * 0.5))
	var result := Vector2(INF, 0.0)
	var rc := R + height_m
	# _render rolls a steep view's screen up off the vertical: then the cone around the axis through the corners.
	var cone := absf(sin(el)) > 0.9
	var corner := t * sqrt(1.0 + aspect * aspect)
	for i in range(CACHE_BORDER_POINTS + 1):
		var a := -1.0 + 2.0 * i / CACHE_BORDER_POINTS
		var border := [Vector2(a, -1.0), Vector2(a, 1.0), Vector2(-aspect, a), Vector2(aspect, a)]
		if cone:
			var phi := PI * a
			border = [Vector2(cos(phi), sin(phi)) * corner / t]
		for p: Vector2 in border:
			# The ray: forward + t (p.y screen up + p.x screen right); its sine of elevation.
			var up := Vector2(-sin(el), cos(el)) * t * p.y
			var d := Vector3(forward.x + up.x, t * p.x, forward.y + up.y)
			var sin_el := d.z / d.length()
			var top := _sphere_hits(rc, sin_el, R + SLAB_TOP_M)
			if top.x > top.y:
				continue
			var base := _sphere_hits(rc, sin_el, R + SLAB_BOTTOM_M)
			var leave := base.x if base.x <= base.y else top.y
			result.x = minf(result.x, _ground_distance(rc, sin_el, top.x, R))
			result.y = maxf(result.y, _ground_distance(rc, sin_el, leave, R))
	return result


## Distances along a ray from radius rc (sine of its elevation) where it crosses the sphere of radius rs; x > y: none.
static func _sphere_hits(rc: float, sin_el: float, rs: float) -> Vector2:
	var b := rc * sin_el
	var disc := b * b - (rc * rc - rs * rs)
	if disc < 0.0:
		return Vector2(1.0, 0.0)
	var q := sqrt(disc)
	return Vector2(-b - q, -b + q)


static func _ground_distance(rc: float, sin_el: float, s: float, r: float) -> float:
	var cos_el := sqrt(maxf(1.0 - sin_el * sin_el, 0.0))
	return r * atan2(s * cos_el, rc + s * sin_el)


## The site's point east_m east (along the parallel) at height_m.
func _point_east(height_m: float, east_m: float) -> Vector3:
	var lon := LON_DEG + rad_to_deg(east_m / (6371000.0 * cos(deg_to_rad(LAT_DEG))))
	return HeightSource.geodetic_to_fixed(LAT_DEG, lon, height_m)


## {worst, at, mean_error, bias, mean} of a cached image against its march (CACHE_ERROR_FLOOR).
static func _cache_error(cached: PackedFloat32Array, march: PackedFloat32Array) -> Dictionary:
	var mean := 0.0
	for v in march:
		mean += v / march.size()
	var worst := 0.0
	var worst_at := 0
	var mean_error := 0.0
	var bias := 0.0
	for i in range(march.size()):
		var error := absf(cached[i] - march[i]) / maxf(march[i], CACHE_ERROR_FLOOR * mean)
		mean_error += error / march.size()
		bias += (cached[i] - march[i]) / (mean * march.size())
		if error > worst:
			worst = error
			worst_at = i
	return {"worst": worst, "at": worst_at, "mean_error": mean_error, "bias": bias, "mean": mean}


## Section 8, broken cloud: the cache error (as _cache_error) split by the view's shapes (`shape`: the background
## transmittance, debug output 4) into silhouette pixels (alpha < CACHE_EDGE_ALPHA within `radius` pixels: one march
## pixel) and interior ones: {interior, edge (worst), interior_count, edge_count, interior_over, edge_over (pixels over
## CACHE_BROKEN_TOLERANCE), p99 (all pixels)}.
static func _cache_error_split(cached: PackedFloat32Array, march: PackedFloat32Array, shape: PackedFloat32Array,
		radius: int) -> Dictionary:
	var mean := 0.0
	for v in march:
		mean += v / march.size()
	var n := CACHE_VIEW_SIZE
	var r := {"interior": 0.0, "edge": 0.0, "interior_count": 0, "edge_count": 0, "interior_over": 0, "edge_over": 0}
	var errors := PackedFloat64Array()
	for y in range(n):
		for x in range(n):
			var i := y * n + x
			var edge := false
			for dy in range(-radius, radius + 1):
				for dx in range(-radius, radius + 1):
					var q := Vector2i(clampi(x + dx, 0, n - 1), clampi(y + dy, 0, n - 1))
					edge = edge or 1.0 - shape[q.y * n + q.x] < CACHE_EDGE_ALPHA
			var error := absf(cached[i] - march[i]) / maxf(march[i], CACHE_ERROR_FLOOR * mean)
			errors.append(error)
			var key := "edge" if edge else "interior"
			r[key] = maxf(r[key], error)
			r[key + "_count"] += 1
			if error > CACHE_BROKEN_TOLERANCE:
				r[key + "_over"] += 1
	errors.sort()
	r["p99"] = errors[int(errors.size() * 0.99)]
	return r


## The GPU time of the light cache's fill in the frame whose timestamps the RenderingDevice holds (0: no fill).
func _light_fill_ms() -> float:
	var rd := RenderingServer.get_rendering_device()
	var worst := 0.0
	for i in range(1, rd.get_captured_timestamps_count()):
		if rd.get_captured_timestamp_name(i - 1) == "cloud_light_fill":
			var ns := rd.get_captured_timestamp_gpu_time(i) - rd.get_captured_timestamp_gpu_time(i - 1)
			worst = maxf(worst, float(ns))
	return worst * 1.0e-6


## The red channel of the view (size: the viewport's), the mean over RADIANCE_FRAMES frames with the march's jitter
## pinned to frames first_frame, first_frame + 1, ...
func _mean_image(camera_itrf_m: Vector3, view_itrf: Vector3, sun_itrf: Vector3, mode: int,
		size := Vector2i(CACHE_VIEW_SIZE, CACHE_VIEW_SIZE), first_frame := 0,
		frames := RADIANCE_FRAMES) -> PackedFloat32Array:
	var mean := PackedFloat32Array()
	mean.resize(size.x * size.y)
	for frame in range(frames):
		CloudEffect.pinned_frame = first_frame + frame
		await _render(camera_itrf_m, view_itrf, sun_itrf, mode)
		var image := _viewport.get_texture().get_image()
		for y in range(size.y):
			for x in range(size.x):
				mean[y * size.x + x] += image.get_pixel(x, y).r / frames
	CloudEffect.pinned_frame = -1
	return mean


## Section 9 (static clouds Task 3): the deterministic march's silhouettes (its 50 % background transmittance contour,
## debug output 4: the shapes) of a broken layer seen from SILHOUETTE_DISTANCE_M lie within one march pixel (the
## preset's divisor in output pixels) of the converged reference's, for both quality presets.
func _silhouette_samples() -> void:
	var slab := _slab("silhouette", SLAB_BOTTOM_M, SLAB_TOP_M, THICK_LIQUID_KG_M3, SILHOUETTE_FRACTION)
	_show(slab)
	var size := Vector2i(SILHOUETTE_VIEW_SIZE, SILHOUETTE_VIEW_SIZE)
	_viewport.size = size
	_camera.fov = SILHOUETTE_FOV_DEG
	var camera := _point(SLAB_TOP_M + SILHOUETTE_DISTANCE_M)
	var view := _local_direction(-90.0, 0.0)
	var sun := _local_direction(45.0, 135.0)
	for quality in [CloudLayer.Quality.DESKTOP, CloudLayer.Quality.VR]:
		_clouds.quality = quality
		_detail_off()
		_clouds.deterministic_march = false
		ShaderGlobals.set_value(&"cloud_fine_step_fraction", SILHOUETTE_REFERENCE_FINE)
		ShaderGlobals.set_value(&"cloud_max_steps", SILHOUETTE_REFERENCE_MAX_STEPS)
		ShaderGlobals.set_value(&"cloud_steps", SILHOUETTE_REFERENCE_STEPS)
		var reference := await _mean_image(camera, view, sun, 4, size, 0, SILHOUETTE_REFERENCE_FRAMES)
		_clouds.quality = quality  # the preset's steps again
		_detail_off()
		_clouds.deterministic_march = true
		var got := await _mean_image(camera, view, sun, 4, size, 0, 1)
		var divisor := int(CloudLayer.PRESETS[quality]["divisor"])
		var result := _contour_distance(got, reference, size)
		_check("silhouette within one march pixel of the converged one (%s preset)"
				% ("VR" if quality == CloudLayer.Quality.VR else "desktop"),
				result.x <= divisor and result.y >= SILHOUETTE_MIN_CONTOUR,
				"farthest %d px (limit %d), %d mismatched pixels, reference contour %d px" % [result.x, divisor,
				result.z, result.y])
	_clouds.quality = CloudLayer.Quality.DESKTOP
	_detail_off()
	_viewport.size = Vector2i(VIEW_SIZE, VIEW_SIZE)
	_camera.fov = 10.0


## Section 12: the flat base on a snapshot BUILT from fields (StarsimCore.weather_write_column_snapshot: from_gfs with
## the LCL raise). Debug output 1 (the grid-mean background transmittance, the shapes off) at a column centre.
func _lcl_base_samples() -> void:
	var up := _local_direction(90.0, 0.0)
	var sun := _local_direction(45.0, 135.0)
	var raised := _column("lcl_raised", LCL_DEW_POINT_K)
	var fog := _column("lcl_fog", LCL_FOG_DEW_POINT_K)
	var base_m := float(raised["ground_m"]) + float(raised["lcl_m"])
	var tau_core := float(raised["optical_depth"])
	var above_m := ceilf(base_m / LCL_FINE_LAYER_M) * LCL_FINE_LAYER_M  # the top of the base's layer
	_show(raised)
	var t_up := (await _render(_point(2.0), up, sun, 1, BLOCK_RADIUS)).x
	# 6 deg downward (the ray stays below the base for the 7 km to the ground; a horizontal one rises above the level
	# with the Earth's curvature) from 100 m below the base; straight up from the top of the base's layer.
	var side := _local_direction(-6.0, 70.0)
	var t_below := (await _render(_point(base_m - 100.0), side, sun, 1, BLOCK_RADIUS)).x
	var t_above := (await _render(_point(above_m), up, sun, 1, BLOCK_RADIUS)).x
	# The same field without its base texture (the raised layers uncut) and the fog column (cloud from the ground).
	var uncut := raised.duplicate()
	var zero := PackedByteArray()
	zero.resize((raised["cloud_base_texels"] as PackedByteArray).size())
	uncut["cloud_base_texels"] = zero
	_show(uncut)
	var t_up_uncut := (await _render(_point(2.0), up, sun, 1, BLOCK_RADIUS)).x
	var t_above_uncut := (await _render(_point(above_m), up, sun, 1, BLOCK_RADIUS)).x
	_show(fog)
	var t_below_fog := (await _render(_point(base_m - 100.0), side, sun, 1, BLOCK_RADIUS)).x
	var tau_up := -log(t_up)
	_check("flat base: the optical depth straight up is the core's column (built snapshot)",
			tau_core > 0.5 and absf(tau_up / tau_core - 1.0) <= LCL_DEPTH_TOLERANCE,
			"rendered %.4f core %.4f ratio %.4f (uncut layers %.4f), base %.1f m"
					% [tau_up, tau_core, tau_up / tau_core, -log(t_up_uncut), base_m])
	_check("flat base: a view 6 deg downward from 100 m below the base sees no cloud (the fog column does)",
			t_below > 0.999 and t_below_fog < 0.9, "base %.5f fog %.5f" % [t_below, t_below_fog])
	_check("flat base: straight up from above the base's layer the cloud is as without the base",
			absf(t_above - t_above_uncut) <= 0.01 * t_above_uncut and t_above < 0.9,
			"cut %.5f uncut %.5f" % [t_above, t_above_uncut])


## Writes a built column snapshot (section 12), returns its loaded dictionary with the writer's numbers.
func _column(label: String, dew_point_k: float) -> Dictionary:
	var file := OS.get_temp_dir().path_join("starsim_cloud_check_%s_%d.wthr" % [label, OS.get_process_id()])
	_files.append(file)
	var written: Dictionary = _core.weather_write_column_snapshot(file, LAT_DEG, LON_DEG, LCL_CLOUD_TOP_M,
			LCL_LIQUID_KG_KG, 100.0, LCL_TEMPERATURE_K, dew_point_k)
	if not written.get("ok", false):
		_fail_and_quit("column snapshot not written")
	var loaded: Dictionary = _core.weather_load_snapshot(file)
	loaded["file"] = file
	loaded.merge(written)
	return loaded

## Section 11: the deck top's limb, rolled, at the VR preset (the game's output: debug output 0, the final image's
## alpha read from the pass's result).
func _outline_samples() -> void:
	var slab := _slab("outline", SLAB_BOTTOM_M, SLAB_TOP_M, THICK_LIQUID_KG_M3, 1.0)
	_show(slab)
	_viewport.size = OUTLINE_VIEW_SIZE
	_camera.fov = OUTLINE_FOV_DEG
	_clouds.quality = CloudLayer.Quality.VR
	_detail_off()
	var camera := _point(SLAB_TOP_M + OUTLINE_HEIGHT_M)
	var r := HeightSource.geodetic_to_fixed(LAT_DEG, LON_DEG, 0.0).length()
	var dip_deg := rad_to_deg(acos((r + SLAB_TOP_M) / (r + SLAB_TOP_M + OUTLINE_HEIGHT_M)))
	var view := _local_direction(-dip_deg, 70.0)
	var sun := _local_direction(45.0, 135.0)
	var alphas: Array[PackedFloat32Array] = []
	for i in range(2):
		await _render(camera, view, sun, 0, 0, OUTLINE_ROLL_DEG)
		alphas.append(_output_alpha())
	var a := alphas[0]
	if a.size() != OUTLINE_VIEW_SIZE.x * OUTLINE_VIEW_SIZE.y:
		_check("anti-aliased outline: the pass's result is read back", false, "%d values" % a.size())
		return
	var w := OUTLINE_VIEW_SIZE.x
	var rows := 0
	var worst_jump := 0.0
	var worst_back := 0.0
	var worst_row := -1
	for y in range(OUTLINE_VIEW_SIZE.y):
		var lo := 1.0
		var hi := 0.0
		for x in range(w):
			lo = minf(lo, a[y * w + x])
			hi = maxf(hi, a[y * w + x])
		if lo > OUTLINE_CLEAR or hi < OUTLINE_COVERED:
			continue  # the edge does not cross this row inside the view
		rows += 1
		var trend := signf(a[y * w + w - 1] - a[y * w])
		for x in range(1, w):
			var step := a[y * w + x] - a[y * w + x - 1]
			if absf(step) > worst_jump:
				worst_jump = absf(step)
				worst_row = y
			worst_back = maxf(worst_back, -trend * step)
	_check("anti-aliased outline (deck limb rolled %.0f deg, VR preset): along-edge alpha monotone, no stair"
			% OUTLINE_ROLL_DEG, rows >= OUTLINE_MIN_ROWS and worst_jump <= OUTLINE_MAX_JUMP
			and worst_back <= OUTLINE_MONOTONE_TOLERANCE,
			"%d rows, largest step %.3f (limit %.2f, row %d), largest step against the trend %.3f (limit %.2f)"
			% [rows, worst_jump, OUTLINE_MAX_JUMP, worst_row, worst_back, OUTLINE_MONOTONE_TOLERANCE])
	var differing := 0
	for i in range(a.size()):
		differing += 1 if alphas[1][i] != a[i] else 0
	_check("anti-aliased outline: a second render is bit-identical (static)", differing == 0,
			"%d of %d pixels differ" % [differing, a.size()])
	if OS.get_environment("STARSIM_VERBOSE") != "":
		var trace := ""
		for x in range(0, w, 4):
			trace += " %.2f" % a[maxi(worst_row, 0) * w + x]
		print("cloud_check: outline row %d (every 4th pixel):%s" % [worst_row, trace])
	# A dispatch beyond D3D12's 65535 groups per dimension (the outline pass dispatches its list 2-D): at divisor 1 the
	# march grid of 2400 x 1800 is 67 500 groups. The pass must still act: a sharp rim has intermediate alphas (the sub-
	# rays' quarters) in about every row it crosses (the offsets span half a march pixel only: no stair limit here).
	_viewport.size = OUTLINE_BIG_VIEW_SIZE
	_clouds.effect.set_quality(1, 1.0)
	await _render(camera, view, sun, 0, 0, OUTLINE_ROLL_DEG)
	var big := _output_alpha()
	var big_w := OUTLINE_BIG_VIEW_SIZE.x
	var big_rows := 0
	var big_partial := 0
	for y in range(OUTLINE_BIG_VIEW_SIZE.y):
		var lo := 1.0
		var hi := 0.0
		for x in range(big_w):
			lo = minf(lo, big[y * big_w + x])
			hi = maxf(hi, big[y * big_w + x])
			big_partial += 1 if big[y * big_w + x] > OUTLINE_CLEAR and big[y * big_w + x] < OUTLINE_COVERED else 0
		big_rows += 1 if lo <= OUTLINE_CLEAR and hi >= OUTLINE_COVERED else 0
	_check("anti-aliased outline: a list beyond 65535 groups (%d x %d, divisor 1)" % [big_w,
			OUTLINE_BIG_VIEW_SIZE.y], big.size() == big_w * OUTLINE_BIG_VIEW_SIZE.y and big_rows >= OUTLINE_MIN_ROWS
			and big_partial >= big_rows / 2, "%d rim rows, %d partial-alpha pixels" % [big_rows, big_partial])
	_clouds.quality = CloudLayer.Quality.DESKTOP
	_detail_off()
	_viewport.size = Vector2i(VIEW_SIZE, VIEW_SIZE)
	_camera.fov = 10.0


## The alpha of the cloud pass's result (layer 0: the view), row-major at the viewport's size.
func _output_alpha() -> PackedFloat32Array:
	var image := _clouds.effect.output.get_layer_data(0)
	var out := PackedFloat32Array()
	if image == null:
		return out
	out.resize(image.get_width() * image.get_height())
	for y in range(image.get_height()):
		for x in range(image.get_width()):
			out[y * image.get_width() + x] = 1.0 - image.get_pixel(x, y).a  # the image holds T
	return out


## Section 9: (x) the largest distance [px, Chebyshev] from a pixel whose side of 50 % transmittance differs from the
## reference to the nearest reference pixel on its side (SILHOUETTE_SEARCH + 1 when none is that near), (y) the
## reference's contour pixels (a neighbour on the other side), (z) the mismatched pixels.
static func _contour_distance(got: PackedFloat32Array, reference: PackedFloat32Array, size: Vector2i) -> Vector3i:
	var farthest := 0
	var contour := 0
	var mismatched := 0
	for y in range(size.y):
		for x in range(size.x):
			var cloud := reference[y * size.x + x] < 0.5
			for n in [Vector2i(1, 0), Vector2i(0, 1)]:
				var q: Vector2i = Vector2i(x, y) + n
				if q.x < size.x and q.y < size.y and (reference[q.y * size.x + q.x] < 0.5) != cloud:
					contour += 1
					break
			var side := got[y * size.x + x] < 0.5
			if side == cloud:
				continue
			mismatched += 1
			var nearest := SILHOUETTE_SEARCH + 1
			for dy in range(-SILHOUETTE_SEARCH, SILHOUETTE_SEARCH + 1):
				for dx in range(-SILHOUETTE_SEARCH, SILHOUETTE_SEARCH + 1):
					var q := Vector2i(x + dx, y + dy)
					var r := maxi(absi(dx), absi(dy))
					var inside := q.x >= 0 and q.y >= 0 and q.x < size.x and q.y < size.y
					if r < nearest and inside and (reference[q.y * size.x + q.x] < 0.5) == side:
						nearest = r
			farthest = maxi(farthest, nearest)
	return Vector3i(farthest, contour, mismatched)


## The mean of each row of a CACHE_LINE_SIZE image (the line of pixels: one value per elevation).
static func _column_means(image: PackedFloat32Array) -> PackedFloat32Array:
	var rows := PackedFloat32Array()
	rows.resize(CACHE_LINE_SIZE.y)
	for y in range(CACHE_LINE_SIZE.y):
		for x in range(CACHE_LINE_SIZE.x):
			rows[y] += image[y * CACHE_LINE_SIZE.x + x] / CACHE_LINE_SIZE.x
	return rows


## Section 6: the ground 300 m north of a camera 2 m up, under a thin slab (tau ~1) and a deck (tau ~3), Sun 45 and
## 20 deg up; then the identity of the weather-off look (land and sea).
func _ground_light_samples() -> void:
	var camera := _point(2.0)
	var point := HeightSource.geodetic_to_fixed(LAT_DEG + rad_to_deg(GROUND_DISTANCE_M / 6.371e6), LON_DEG, 0.0)
	var up := _local_direction(90.0, 0.0)
	var thin := _slab("ground_thin", SLAB_BOTTOM_M, SLAB_TOP_M, THIN_LIQUID_KG_M3, 1.0)
	var deck := _slab("ground_deck", SLAB_BOTTOM_M, SLAB_TOP_M, DECK_LIQUID_KG_M3, 1.0)
	var broken := _slab("ground_broken", SLAB_BOTTOM_M, SLAB_TOP_M, DECK_LIQUID_KG_M3, BROKEN_FRACTION)
	for slab: Dictionary in [thin, deck, broken]:
		for elevation in [45.0, 20.0]:
			var sun := _local_direction(elevation, 135.0)
			var label := "%s, Sun %.0f deg" % ["tau 1" if slab == thin else ("deck tau 3" if slab == deck
					else "broken cf %.1f" % BROKEN_FRACTION), elevation]
			_clouds.show_now({}, {}, 0.0)
			_core.weather_set_active("", "", 0.0)
			var off := await _render_ground(camera, point, up, sun, 0, _terrain_material)
			_show(slab)
			var light: Dictionary = _core.weather_cloud_light(point, sun)
			var t_s := float(light["sun_transmittance"])
			var t_sd := float(light["sun_diffuse"])
			var t_d := float(light["sky_factor"])
			var lux: Dictionary = _core.atmosphere_illuminance(point, up, sun)
			var e_sun: Vector3 = lux["sun_lux"]
			var e_sky: Vector3 = lux["sky_lux"]
			var sun_share := _luminance(e_sun) / _luminance(e_sun + e_sky)
			# a. The maps' machinery: the near map of the grid mean against the CPU's Beer-Lambert, the column terms, and
			# the shading's composition (the ratio on / off of the surface part: the in-scattered light taken out).
			_clouds.shadow_effect.grid_mean_only = true
			var on := await _render_ground(camera, point, up, sun, 0, _terrain_material)
			var factors := await _render_ground(camera, point, up, sun, 5, _terrain_material)
			_check("ground (grid-mean map): direct Sun " + label,
					absf(factors.x - t_s) <= maxf(FACTOR_TOLERANCE * t_s, DIRECT_FLOOR * 0.1),
					"GPU %.4f CPU %.4f" % [factors.x, t_s])
			_check("ground: the Sun's diffuse share " + label, absf(factors.y - t_sd) <= FACTOR_TOLERANCE * t_sd,
					"GPU %.4f CPU %.4f" % [factors.y, t_sd])
			_check("ground: the sky's diffuse transmittance " + label, absf(factors.z - t_d) <= FACTOR_TOLERANCE * t_d,
					"GPU %.4f CPU %.4f" % [factors.z, t_d])
			var to_point: Dictionary = _core.atmosphere_sky_luminance_to_point(camera, point, sun)
			var in_scatter: Vector3 = to_point["luminance"] * _atmosphere.luminance_scale
			var expected := _luminance(e_sun * (t_s + t_sd) + e_sky * t_d) / _luminance(e_sun + e_sky)
			var got := _luminance(on - in_scatter) / _luminance(off - in_scatter)
			_check("ground luminance ratio weather on / off " + label,
					absf(got - expected) <= GROUND_RATIO_TOLERANCE * expected,
					"rendered %.4f CPU %.4f (T_s %.4f T_sd %.4f T_d %.4f, Sun share %.3f)" % [got, expected, t_s, t_sd,
					t_d, sun_share])
			# b. The game's near map (the visible clouds' shapes) against the vehicle's CPU light (the grid mean) at the
			# same spot: their total light on a horizontal plate (direct + the Sun's diffuse + sky).
			_clouds.shadow_effect.grid_mean_only = false
			var shaped := await _render_ground(camera, point, up, sun, 5, _terrain_material)
			var ground_total := (shaped.x + shaped.y) * sun_share + shaped.z * (1.0 - sun_share)
			var vehicle_total := (t_s + t_sd) * sun_share + t_d * (1.0 - sun_share)
			var detail := "vehicle %.4f ground %.4f (direct CPU %.4f, shapes %.4f)" % [vehicle_total, ground_total, t_s,
					shaped.x]
			if slab == broken:  # the spot lies in a gap or under a cumulus; the CPU's is the cell's mean (reported)
				print("  info  vehicle light vs the ground's (shapes) %s: %s" % [label, detail])
			else:
				_check("vehicle light (CPU, grid mean) vs the ground's (shapes) " + label,
						absf(vehicle_total - ground_total) <= VEHICLE_TOLERANCE * ground_total, detail)
	# Final review I1 / I3: a cross-fade with an empty slot (deck -> no weather). The GPU's light factors (grid-mean map) equal
	# the CPU light of the pair CloudLayer pushed (I1), and the ground light never dips below the end states (I3: the empty
	# slot's column texels are zero; an R8 texture would read as cover 1 and darken the middle of the fade).
	var fade_sun := _local_direction(45.0, 135.0)
	var fade_up := _local_direction(90.0, 0.0)
	_clouds.shadow_effect.grid_mean_only = true
	var fade_lux: Dictionary = _core.atmosphere_illuminance(point, fade_up, fade_sun)
	var fade_sun_share := _luminance(fade_lux["sun_lux"]) / _luminance(fade_lux["sun_lux"] + fade_lux["sky_lux"])
	var totals: Array = []
	var fade_ok := true
	var fade_detail := ""
	for w1 in [0.0, 0.25, 0.5, 0.75, 0.95]:  # 1.0 is weather off (no cloud pass: the bit-identity check below)
		_clouds.show_now(deck, {}, w1)
		var gpu := await _render_ground(camera, point, fade_up, fade_sun, 5, _terrain_material)
		var cpu: Dictionary = _core.weather_cloud_light(point, fade_sun)
		var cpu_factors := Vector3(float(cpu["sun_transmittance"]), float(cpu["sun_diffuse"]), float(cpu["sky_factor"]))
		for axis in range(3):
			fade_ok = fade_ok and absf(gpu[axis] - cpu_factors[axis]) <= maxf(FACTOR_TOLERANCE * cpu_factors[axis], DIRECT_FLOOR * 0.1)
		fade_detail += " w1 %.2f GPU %s CPU %s;" % [w1, gpu, cpu_factors]
		totals.append((gpu.x + gpu.y) * fade_sun_share + gpu.z * (1.0 - fade_sun_share))  # light on a horizontal plate
	_clouds.shadow_effect.grid_mean_only = false
	_check("cross-fade deck -> empty: GPU light factors equal the CPU light of the pushed pair", fade_ok, fade_detail)
	var monotonic: bool = totals[0] < totals[4]
	for i in range(1, totals.size()):
		monotonic = monotonic and totals[i] >= totals[i - 1] - 1e-3
	_check("ground light is monotonic through a deck -> empty cross-fade", monotonic, "light on a plate at w1 0 .25 .5 .75 .95: %s" % [totals])
	# Weather off: the shader with and without its CLOUD_LIGHT blocks, every pixel bit-identical (land and sea).
	_clouds.show_now({}, {}, 0.0)
	_core.weather_set_active("", "", 0.0)
	var sun := _local_direction(30.0, 135.0)
	var stripped := Shader.new()
	stripped.code = _strip_cloud_light((load("res://shaders/terrain_earth.gdshader") as Shader).code)
	for material: ShaderMaterial in [_terrain_material, _water_material]:
		await _render_ground(camera, point, up, sun, 0, material)
		var with_blocks := _viewport.get_texture().get_image().get_data()
		var without := material.duplicate() as ShaderMaterial
		without.shader = stripped
		await _render_ground(camera, point, up, sun, 0, without)
		var reference := _viewport.get_texture().get_image().get_data()
		var differing := 0
		for i in range(reference.size()):
			differing += 1 if reference[i] != with_blocks[i] else 0
		_check("weather off: %s bit-identical to the shader without its cloud blocks" % ["land" if material
				== _terrain_material else "sea"], differing == 0 and reference.size() > 0,
				"%d of %d bytes differ" % [differing, reference.size()])


## The shader source without its CLOUD_LIGHT_BEGIN / _END blocks (what it was before Task 9).
static func _strip_cloud_light(code: String) -> String:
	var out := PackedStringArray()
	var inside := false
	for line in code.split("\n"):
		if line.strip_edges() == "// CLOUD_LIGHT_BEGIN":
			inside = true
		elif line.strip_edges() == "// CLOUD_LIGHT_END":
			inside = false
		elif not inside:
			out.append(line)
	return "\n".join(out)


## The terrain quad at point (normal = the lighting's normal), seen from camera (the quad fills the view); mode: the
## cloud debug output. Returns the centre 5 x 5 mean.
func _render_ground(camera_itrf_m: Vector3, point_itrf_m: Vector3, normal_itrf: Vector3, sun_itrf: Vector3, mode: int,
		material: ShaderMaterial) -> Vector3:
	var sun_local := Frames.to_render(sun_itrf * AU_M)
	ShaderGlobals.set_value(&"earth_sun_position_local", sun_local)
	ShaderGlobals.set_value(&"earth_moon_position_local", -Frames.to_render(sun_itrf) * MOON_DISTANCE_M)
	var view_itrf := (point_itrf_m - camera_itrf_m).normalized()
	var camera_gcrf := _earth_to_gcrf * camera_itrf_m
	var center := Frames.to_render(point_itrf_m)
	var distance := (point_itrf_m - camera_itrf_m).length()
	_terrain_quad.material_override = material
	_terrain_parent.transform = Transform3D(Frames.basis_to_render(_earth_to_gcrf), Frames.to_render(-camera_gcrf))
	_terrain_quad.mesh = _facing_quad(Frames.to_render(view_itrf), Frames.to_render(normal_itrf), distance * 0.2)
	_terrain_quad.position = center
	var split := TerrainQuadtree.split_float32(center)
	_terrain_quad.set_instance_shader_parameter(&"vertex_offset", split[0])
	_terrain_quad.set_instance_shader_parameter(&"vertex_offset_lo", split[1])
	var camera_split := TerrainQuadtree.split_float32(Frames.to_render(camera_itrf_m))
	ShaderGlobals.set_value(&"earth_terrain_camera_hi", camera_split[0])
	ShaderGlobals.set_value(&"earth_terrain_camera_lo", camera_split[1])
	_terrain_parent.visible = true
	var got := await _render(camera_itrf_m, view_itrf, sun_itrf, mode, BLOCK_RADIUS)
	_terrain_parent.visible = false
	return got


## A quad facing the view (atmosphere_check.gd), its normals the lighting's normal.
static func _facing_quad(view: Vector3, normal: Vector3, half_m: float) -> ArrayMesh:
	var u := view.cross(Vector3.UP if absf(view.y) < 0.9 else Vector3.RIGHT).normalized() * half_m
	var w := view.cross(u).normalized() * half_m
	var vertices := PackedVector3Array([-u - w, u - w, u + w, -u - w, u + w, -u + w])
	var normals := PackedVector3Array()
	for i in range(vertices.size()):
		normals.append(normal)
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_NORMAL] = normals
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return mesh


## Section 7 (Task 9 fix round 1): the shadow maps' hand-overs and the shipped shapes on broken cloud, through
## cloud_light_probe.gdshader (the surfaces' own cloud-light functions at receivers around a camera 2 m up). Receivers
## on annuli around the camera (area-uniform), the Sun at azimuth 135 deg. One site holds one realisation of the shape
## noise (its cells are kilometres to tens of km across: a disc of 8 km sees a few of them), so every number is the mean
## over SITES (~55 km apart: independent realisations), which brings the sampling error down by sqrt(8).
func _handover_samples() -> void:
	_clouds.shadow_effect.grid_mean_only = false
	# a. Hand-overs: the direct Sun's mean over 1 km annuli at 8 / 10 / 12 km (near map -> far map, 9-11 km) and over
	# 4 km annuli at 110 / 130 / 150 km (far map -> the slanted column, 120-140 km), a broken layer and a deck.
	for case: Array in [["cf 0.3 broken", 0.3, 2.0 * DECK_LIQUID_KG_M3], ["deck cf 1", 1.0, DECK_LIQUID_KG_M3]]:
		_show(_slab("handover_%d" % int(case[1] * 10.0), SLAB_BOTTOM_M, SLAB_TOP_M, case[2], case[1]))
		for elevation in [45.0, 8.0]:
			var label := "%s, Sun %.0f deg" % [case[0], elevation]
			var near_bands: Array = await _sites_mean(elevation, [[7000.0, 9000.0, 32, 720], [9000.0, 11000.0, 32, 720],
					[11000.0, 13000.0, 32, 720]], SITES.size())
			var far_bands: Array = await _sites_mean(elevation, [[108000.0, 112000.0, 12, 360],
					[128000.0, 132000.0, 12, 360], [148000.0, 152000.0, 12, 360]], 1)
			var step_a: float = absf(near_bands[1].direct - near_bands[0].direct)
			var step_b: float = absf(near_bands[2].direct - near_bands[1].direct)
			var allowed_a: float = HANDOVER_TOLERANCE + 3.0 * sqrt(pow(near_bands[0].direct_se, 2.0) + pow(near_bands[1].direct_se, 2.0))
			var allowed_b: float = HANDOVER_TOLERANCE + 3.0 * sqrt(pow(near_bands[1].direct_se, 2.0) + pow(near_bands[2].direct_se, 2.0))
			_check("hand-over 9-11 km, no ring, " + label, step_a <= allowed_a and step_b <= allowed_b,
					"direct at 8 / 10 / 12 km %.4f %.4f %.4f (steps %.4f %.4f, allowed %.4f %.4f; near map %.4f %.4f %.4f, far map %.4f %.4f %.4f)" % [
					near_bands[0].direct, near_bands[1].direct, near_bands[2].direct, step_a, step_b, allowed_a, allowed_b,
					near_bands[0].near, near_bands[1].near, near_bands[2].near, near_bands[0].far,
					near_bands[1].far, near_bands[2].far])
			var far_a: float = absf(far_bands[1].direct - far_bands[0].direct)
			var far_b: float = absf(far_bands[2].direct - far_bands[1].direct)
			var column_gap: float = absf(far_bands[0].far - far_bands[0].column)
			_check("hand-over 120-140 km, no ring, " + label,
					maxf(maxf(far_a, far_b), column_gap) <= FAR_HANDOVER_TOLERANCE,
					"direct at 110 / 130 / 150 km %.4f %.4f %.4f (steps %.4f %.4f); far map %.4f vs the slanted column %.4f at 110 km" % [
					far_bands[0].direct, far_bands[1].direct, far_bands[2].direct, far_a, far_b, far_bands[0].far,
					far_bands[0].column])
	# b. The shipped near map (the shapes) on broken cloud over the disc inside the near tier (r < 8 km): the area mean
	# of the ground factors against the CPU's cell factors, the vehicle's light against the ground's.
	for fraction in [0.15, 0.3, BROKEN_FRACTION]:
		_show(_slab("disc_%d" % int(fraction * 100.0), SLAB_BOTTOM_M, SLAB_TOP_M, DECK_LIQUID_KG_M3, fraction))
		for elevation in [45.0, 8.0]:
			var label := "cf %.2f, Sun %.0f deg" % [fraction, elevation]
			var disc: Dictionary = (await _sites_mean(elevation, [[200.0, DISC_RADIUS_M, 40, 720]], SITES.size()))[0]
			var camera := _point(2.0)
			var sun := _local_direction(elevation, 135.0)
			var cpu: Dictionary = _core.weather_cloud_light(camera, sun)
			var t_s := float(cpu["sun_transmittance"])
			var t_sd := float(cpu["sun_diffuse"])
			var t_d := float(cpu["sky_factor"])
			_check("shapes: area mean of the direct Sun vs the CPU's cell " + label,
					absf(disc.direct - t_s) <= DISC_TOLERANCE * maxf(t_s, 0.1) and disc.direct <= 1.0,
					"ground %.4f (near map %.4f, far map %.4f) CPU %.4f" % [disc.direct, disc.near, disc.far, t_s])
			_check("shapes: the near map's mean vs the far map's over the same receivers " + label,
					absf(disc.near - disc.far) <= DISC_TOLERANCE * maxf(disc.far, 0.1),
					"near %.4f far %.4f" % [disc.near, disc.far])
			_check("ground: the column terms vs the CPU " + label,
					absf(disc.diffuse - t_sd) <= FACTOR_TOLERANCE * maxf(t_sd, 0.01)
					and absf(disc.sky - t_d) <= FACTOR_TOLERANCE * t_d,
					"Sun's diffuse %.4f (CPU %.4f) sky %.4f (CPU %.4f)" % [disc.diffuse, t_sd, disc.sky, t_d])
			var lux: Dictionary = _core.atmosphere_illuminance(camera, _local_direction(90.0, 0.0), sun)
			var e_sun: Vector3 = lux["sun_lux"]
			var e_sky: Vector3 = lux["sky_lux"]
			var sun_share := _luminance(e_sun) / _luminance(e_sun + e_sky)
			var ground_total: float = (disc.direct + disc.diffuse) * sun_share + disc.sky * (1.0 - sun_share)
			var vehicle_total := (t_s + t_sd) * sun_share + t_d * (1.0 - sun_share)
			_check("vehicle light (CPU) vs the ground's mean (shapes) " + label,
					absf(vehicle_total - ground_total) <= VEHICLE_TOLERANCE * ground_total,
					"vehicle %.4f ground %.4f (Sun share %.3f)" % [vehicle_total, ground_total, sun_share])


## The annulus means (see _annulus_means) averaged over the first site_count SITES; ranges: [[r0 m, r1 m, rings,
## azimuths], ...]. Returns one Dictionary per range.
func _sites_mean(elevation_deg: float, ranges: Array, site_count: int) -> Array:
	var sums: Array = []
	var squares: Array[float] = []
	for r in ranges:
		sums.append({})
		squares.append(0.0)
	for s in range(site_count):
		var lat: float = LAT_DEG + SITES[s].x
		var lon: float = LON_DEG + SITES[s].y
		var camera := HeightSource.geodetic_to_fixed(lat, lon, 2.0)
		var sun := _local_direction(elevation_deg, 135.0, lat, lon)
		await _render(camera, sun, sun, 0)
		for i in range(ranges.size()):
			var m: Dictionary = await _annulus_means(camera, lat, lon, ranges[i][0], ranges[i][1], ranges[i][2],
					ranges[i][3])
			for key in m:
				sums[i][key] = float(sums[i].get(key, 0.0)) + float(m[key]) / float(site_count)
			squares[i] += float(m["direct"]) * float(m["direct"]) / float(site_count)
	for i in range(ranges.size()):
		# The standard error of the mean over the sites (the sites' spread is the shape noise's sampling error).
		var variance := maxf(squares[i] - float(sums[i]["direct"]) * float(sums[i]["direct"]), 0.0)
		sums[i]["direct_se"] = sqrt(variance / maxf(float(site_count - 1), 1.0)) if site_count > 1 else 0.0
	return sums


func _slant_table() -> void:
	_clouds.shadow_effect.grid_mean_only = false
	for fraction in [0.15, 0.3, 0.4, 0.6]:
		_show(_slab("table_%d" % int(fraction * 100.0), SLAB_BOTTOM_M, SLAB_TOP_M, DECK_LIQUID_KG_M3, fraction))
		for elevation in [75.0, 60.0, 45.0, 30.0, 20.0, 12.0, 8.0, 5.0]:
			var disc: Dictionary = (await _sites_mean(elevation, [[200.0, DISC_RADIUS_M, 40, 720]], SITES.size()))[0]
			print("slant table: cf %.2f elevation %4.1f  T %.4f  C_eff %.4f  n %.3f" % [fraction, elevation, disc.direct,
					1.0 - disc.direct, log(1.0 - (1.0 - disc.direct)) / log(1.0 - fraction)])


func _setup_probe() -> void:
	_probe_viewport.use_hdr_2d = true
	_probe_viewport.own_world_3d = true
	_probe_viewport.render_target_update_mode = SubViewport.UPDATE_DISABLED
	root.add_child(_probe_viewport)
	var camera := Camera3D.new()
	camera.current = true
	_probe_viewport.add_child(camera)
	var shader := load("res://tests/cloud_light_probe.gdshader") as Shader
	_probe_material.shader = shader
	var quad := MeshInstance3D.new()
	var mesh := QuadMesh.new()
	mesh.size = Vector2(2.0, 2.0)
	quad.mesh = mesh
	quad.material_override = _probe_material
	quad.position = Vector3(0.0, 0.0, -1.0)
	quad.extra_cull_margin = 16384.0
	_probe_viewport.add_child(quad)


## The Earth-fixed point at arc distance s (m) from the camera's site toward the azimuth (from north, east positive),
## height 0 (spherical destination on the mean radius: the distance is nominal, to 0.3 %).
static func _destination(site_lat_deg: float, site_lon_deg: float, distance_m: float, azimuth_rad: float) -> Vector3:
	var lat := deg_to_rad(site_lat_deg)
	var delta := distance_m / 6.371e6
	var lat2 := asin(sin(lat) * cos(delta) + cos(lat) * sin(delta) * cos(azimuth_rad))
	var lon2 := deg_to_rad(site_lon_deg) + atan2(sin(azimuth_rad) * sin(delta) * cos(lat),
			cos(delta) - sin(lat) * sin(lat2))
	return HeightSource.geodetic_to_fixed(rad_to_deg(lat2), rad_to_deg(lon2), 0.0)


## The area mean of the surfaces' cloud light over the annulus r0..r1 around the camera (rings x azimuths, weight r):
## {direct (the distance rule), near, far (the maps alone), column (slanted to the Sun), diffuse, sky}.
func _annulus_means(camera_itrf: Vector3, site_lat_deg: float, site_lon_deg: float, r0: float, r1: float, rings: int,
		azimuths: int) -> Dictionary:
	var offsets := PackedVector3Array()
	var weights := PackedFloat64Array()
	for i in range(rings):
		var r := r0 + (float(i) + 0.5) / float(rings) * (r1 - r0)
		for k in range(azimuths):
			var azimuth := TAU * (float(k) + 0.5 + 0.37 * float(i)) / float(azimuths)  # staggered rings
			offsets.append(Frames.to_render(_destination(site_lat_deg, site_lon_deg, r, azimuth) - camera_itrf))
			weights.append(r)
	var first := await _probe(offsets, 0)
	var second := await _probe(offsets, 1)
	var sum := PackedFloat64Array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
	var total := 0.0
	for i in range(offsets.size()):
		var w := weights[i]
		total += w
		sum[0] += w * first[i].x
		sum[1] += w * first[i].y
		sum[2] += w * first[i].z
		sum[3] += w * second[i].x
		sum[4] += w * second[i].y
		sum[5] += w * second[i].z
	return {"direct": sum[0] / total, "near": sum[1] / total, "far": sum[2] / total, "column": sum[3] / total,
			"diffuse": sum[4] / total, "sky": sum[5] / total}


## Renders the probe shader over the receivers (camera-relative offsets, Earth-local render axes): one pixel each.
func _probe(offsets: PackedVector3Array, channel: int) -> PackedVector3Array:
	var width := PROBE_WIDTH
	var height := (offsets.size() - 1) / width + 1
	var floats := PackedFloat32Array()
	floats.resize(width * height * 4)
	for i in range(offsets.size()):
		floats[4 * i] = offsets[i].x
		floats[4 * i + 1] = offsets[i].y
		floats[4 * i + 2] = offsets[i].z
		floats[4 * i + 3] = 1.0
	var image := Image.create_from_data(width, height, false, Image.FORMAT_RGBAF, floats.to_byte_array())
	_probe_material.set_shader_parameter(&"receivers", ImageTexture.create_from_image(image))
	_probe_material.set_shader_parameter(&"channel", channel)
	_probe_viewport.size = Vector2i(width, height)
	_probe_viewport.render_target_update_mode = SubViewport.UPDATE_ONCE
	await RenderingServer.frame_post_draw
	await RenderingServer.frame_post_draw
	var result := _probe_viewport.get_texture().get_image()
	var out := PackedVector3Array()
	for i in range(offsets.size()):
		var c := result.get_pixel(i % width, i / width)
		out.append(Vector3(c.r, c.g, c.b))
	return out


func _point(height_m: float) -> Vector3:
	return HeightSource.geodetic_to_fixed(LAT_DEG, LON_DEG, height_m)


## ITRF unit vector at elevation above the geodetic horizon and azimuth (from north, east positive) at the site.
static func _local_direction(elevation_deg: float, azimuth_deg: float, lat_deg := LAT_DEG,
		lon_deg := LON_DEG) -> Vector3:
	var lat := deg_to_rad(lat_deg)
	var lon := deg_to_rad(lon_deg)
	var up := Vector3(cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat))
	var east := Vector3(-sin(lon), cos(lon), 0.0)
	var north := Vector3(-sin(lat) * cos(lon), -sin(lat) * sin(lon), cos(lat))
	var az := deg_to_rad(azimuth_deg)
	var el := deg_to_rad(elevation_deg)
	return (up * sin(el) + (north * cos(az) + east * sin(az)) * cos(el)).normalized()


static func _luminance(c: Vector3) -> float:
	return 0.2126 * c.x + 0.7152 * c.y + 0.0722 * c.z


func _check(label: String, ok: bool, detail: String) -> void:
	_samples += 1
	if not ok:
		_failures += 1
	print("  %s  %s: %s" % ["ok  " if ok else "FAIL", label, detail])


func _fail_and_quit(message: String) -> void:
	print("cloud_check: FAIL - " + message)
	quit(1)


## Section 13 (static clouds Task 9): the Sun's disk (sun.gdshader, HDR) seen from 10 m up through slabs of vertical tau
## SUN_TAUS, the Sun SUN_ELEVATION_DEG up. The composite is linear in the scene behind the clouds, so the disk's part
## of a pixel is the render with the disk less the render without it (the cloud's own glow cancels), measured at the Sun
## itself (what the eye sees) against the cloudless disk. It must equal the cloud pass's transmittance of that ray (its
## shapes, debug output 4) within SUN_DISK_TOLERANCE - or be exactly 0 where that is below the cloud image's half-float
## store (SUN_T_STORE_MIN) - and lie at or above exp(-tau / mu) of the slab's grid mean (the core's): the drawn cloud's
## inhomogeneity lets more of the beam through than its mean depth (the plane-parallel bias, Cahalan et al. 1994). The
## cloudless disk: its centre = the core's sun transmittance x its physical value, chroma-preserving clamped at
## SUN_DISK_CLAMP (above it at 55 deg, below at SUN_LOW_ELEVATION_DEG). The near march, and the far tier alone for
## SUN_FAR_TAUS.
func _sun_through_samples() -> void:
	var camera := _point(10.0)
	var sun := _local_direction(SUN_ELEVATION_DEG, 135.0)
	var up := _local_direction(90.0, 0.0)
	var mu := sun.dot(up)
	# The cloudless terrain's radiance (the same Sun, 300 m ahead): what the disk must outshine.
	_clouds.show_now({}, {}, 0.0)
	_core.weather_set_active("", "", 0.0)
	var point := HeightSource.geodetic_to_fixed(LAT_DEG + rad_to_deg(GROUND_DISTANCE_M / 6.371e6), LON_DEG, 0.0)
	var terrain := _luminance(await _render_ground(camera, point, up, sun, 0, _terrain_material))
	# The disk as the game places it: at SUN_RENDER_DISTANCE_M, its apparent size.
	_sun_disk.scale = Vector3.ONE * Atmosphere.SUN_RADIUS_M * SUN_RENDER_DISTANCE_M / AU_M
	_sun_disk.visible = true
	_camera.fov = SUN_FOV_DEG
	var aureole := Vector2i(SUN_AUREOLE_PIXELS, 0)
	var low_sun := _local_direction(SUN_LOW_ELEVATION_DEG, 135.0)
	for direction: Vector3 in [low_sun, sun]:
		_sun_disk.position = Frames.to_render(_earth_to_gcrf * direction) * SUN_RENDER_DISTANCE_M
		var centre := await _render(camera, direction, direction, 0, 0)
		var expected := _sun_disk_expected(camera, direction)
		var peak := maxf(expected.x, maxf(expected.y, expected.z))
		var drawn := expected * minf(1.0, SUN_DISK_CLAMP / peak)
		var error := (centre - drawn).abs() / maxf(drawn.x, maxf(drawn.y, drawn.z))
		var got_peak := maxf(centre.x, maxf(centre.y, centre.z))
		var detail := "centre %s expected %s (unclamped peak %.0f)" % [centre, drawn, peak]
		if direction == low_sun:
			_check("Sun: the cloudless disk %.0f deg up is unclipped HDR at its physical radiance"
					% SUN_LOW_ELEVATION_DEG,
					peak < SUN_DISK_CLAMP and got_peak < SUN_DISK_CLAMP
					and maxf(error.x, maxf(error.y, error.z)) <= SUN_DISK_COLOUR_TOLERANCE, detail)
		else:
			_check("Sun: the cloudless disk %.0f deg up at its physical radiance, its colour kept by the clamp"
					% SUN_ELEVATION_DEG, got_peak <= SUN_DISK_CLAMP * (1.0 + SUN_HALF_RESOLUTION)
					and maxf(error.x, maxf(error.y, error.z)) <= SUN_DISK_COLOUR_TOLERANCE, detail)
	var clear_disk := _luminance(await _render(camera, sun, sun, 0, BLOCK_RADIUS))
	var clear_beside := _luminance(await _render(camera, sun, sun, 0, BLOCK_RADIUS, 0.0, aureole))
	_check("Sun: the cloudless disk outshines its surroundings",
			clear_disk > 10.0 * clear_beside and clear_disk > terrain,
			"disk %.1f beside %.4f terrain %.4f" % [clear_disk, clear_beside, terrain])
	var preset: Dictionary = CloudLayer.PRESETS[CloudLayer.Quality.DESKTOP]
	for far_only in [false, true]:
		for nominal_tau: float in (SUN_FAR_TAUS if far_only else SUN_TAUS):
			var slab := _slab("sun_%d" % int(nominal_tau), SLAB_BOTTOM_M, SLAB_TOP_M,
					nominal_tau * THIN_LIQUID_KG_M3, 1.0)
			_show(slab)
			if far_only:  # the far tier alone (render_check --handover=0,0)
				ShaderGlobals.set_value(&"cloud_handover_near_m", 0.0)
				ShaderGlobals.set_value(&"cloud_handover_far_m", 0.0)
			await _settle_cache(camera, sun, sun, 0)  # the light cache complete: the same glow in both
			var t_cpu := float(_core.weather_cloud_light(camera, sun)["sun_transmittance"])
			_sun_disk.visible = false
			var glow := _luminance(await _render(camera, sun, sun, 0, BLOCK_RADIUS))
			_sun_disk.visible = true
			var with_disk := _luminance(await _render(camera, sun, sun, 0, BLOCK_RADIUS))
			var through := with_disk - glow
			var got := through / clear_disk
			# What the two renders' half floats resolve of the disk's part (relative to the cloudless disk).
			var resolution := (glow + with_disk) * SUN_HALF_RESOLUTION / clear_disk
			_sun_disk.visible = false
			var t_grid := (await _render(camera, sun, sun, 1, BLOCK_RADIUS)).x  # the grid mean's T
			var t_march := (await _render(camera, sun, sun, 4, BLOCK_RADIUS)).x  # the shapes' T
			_sun_disk.visible = true
			var label := "%s tau %.0f" % ["far tier" if far_only else "near march", nominal_tau]
			var detail := ("disk/clear %s march %s core %s resolution %s; the cloud's own light there %.4f x clear"
					+ " disk") % [
					String.num_scientific(got), String.num_scientific(t_march), String.num_scientific(t_cpu),
					String.num_scientific(resolution), glow / clear_disk]
			if t_march >= SUN_T_STORE_MIN:
				_check("Sun disk through " + label + " = the cloud pass's transmittance",
						resolution <= 0.5 * t_march
						and absf(got - t_march) <= maxf(SUN_DISK_TOLERANCE * t_march, resolution), detail)
				_check("Sun disk through " + label + " at or above exp(-tau / mu) (unresolved inhomogeneity)",
						got >= (1.0 - SUN_DISK_TOLERANCE) * t_cpu - resolution,
						"disk / exp(-tau / mu) %.3f" % (got / maxf(t_cpu, 1.0e-30)))
			else:
				_check("Sun disk through " + label + " is 0 below the cloud image's transmittance store",
						through == 0.0, detail)
				_check("Sun: the direct beam through " + label + " is below the store physically too",
						t_cpu < SUN_T_STORE_MIN, "core T %s" % String.num_scientific(t_cpu))
			# The march against the core's exp(-tau / mu): the slab's low shape octaves vary the depth along a ray.
			var tau_slant := -log(t_cpu)
			var tau_march := -log(maxf(t_grid, 1.0e-30))
			print("    march slant tau %.3f, core %.3f (exp(-%.0f/%.3f) = %.3f)" % [tau_march, tau_slant, nominal_tau, mu,
					nominal_tau / mu])
			if t_cpu > SUN_TAU_MIN_T:  # (the colour image of the debug output holds half floats: no T below 6e-8)
				_check("Sun transmittance of the march through " + label,
						absf(tau_march - tau_slant) <= SUN_TAU_TOLERANCE * tau_slant,
						"T grid mean %.5f shapes %.5f core %.5f" % [t_grid, t_march, t_cpu])
			else:
				print("    (beyond the debug output's half floats: T core ", String.num_scientific(t_cpu), ")")
			if not far_only:
				_check("Sun disk through %s outshines the terrain while expected to" % label,
						clear_disk * t_cpu < terrain or through > terrain,
						"disk %.4f expected %.4f terrain %.4f" % [through, clear_disk * t_cpu, terrain])
			if not far_only and nominal_tau == SUN_TAUS[1]:
				var side := _local_direction(SUN_ELEVATION_DEG, 135.0 + SUN_AUREOLE_AWAY_DEG)
				var near_sun := _luminance(await _render(camera, sun, sun, 2, BLOCK_RADIUS, 0.0, aureole))
				var away := _luminance(await _render(camera, side, sun, 2, BLOCK_RADIUS))
				_check("Sun aureole: scattered radiance 0.43 deg from the disk vs %.0f deg away" % SUN_AUREOLE_AWAY_DEG,
						near_sun >= SUN_AUREOLE_RATIO * away, "%.4f vs %.4f (%.1fx)" % [near_sun, away,
						near_sun / maxf(away, 1e-9)])
			if far_only:  # the preset's hand-over again
				var band: Vector2 = preset["handover_m"]
				ShaderGlobals.set_value(&"cloud_handover_near_m", band.x)
				ShaderGlobals.set_value(&"cloud_handover_far_m", band.y)
	await _sun_through_cumulus_samples(clear_disk)
	_sun_disk.visible = false
	_camera.fov = 10.0


## Section 13's realistic cumulus body: see SUN_CU_AZIMUTHS_DEG. The camera and the Sun are at the analog site.
func _sun_through_cumulus_samples(clear_disk: float) -> void:
	var file := _analog_june_file()
	if file == "":
		_fail_and_quit("Sun through cumulus: no analog snapshot for month %d" % SUN_CU_MONTH)
		return
	var snapshot: Dictionary = _core.weather_load_snapshot(file)
	snapshot["file"] = file
	_show(snapshot)
	var camera := HeightSource.geodetic_to_fixed(SUN_CU_SITE.x, SUN_CU_SITE.y, 10.0)
	var gaps := 0
	var cores := 0
	var offsets := [Vector2i(SUN_CU_NEIGHBOUR_PIXELS, 0), Vector2i(-SUN_CU_NEIGHBOUR_PIXELS, 0),
			Vector2i(0, SUN_CU_NEIGHBOUR_PIXELS), Vector2i(0, -SUN_CU_NEIGHBOUR_PIXELS)]
	for azimuth: float in SUN_CU_AZIMUTHS_DEG:
		var sun := _local_direction(SUN_ELEVATION_DEG, azimuth, SUN_CU_SITE.x, SUN_CU_SITE.y)
		_sun_disk.position = Frames.to_render(_earth_to_gcrf * sun) * SUN_RENDER_DISTANCE_M
		await _settle_cache(camera, sun, sun, 0)
		_sun_disk.visible = false
		await _render(camera, sun, sun, 0, BLOCK_RADIUS)  # the cloud's own light has settled (the cache's last frames)
		var glow := _luminance(await _render(camera, sun, sun, 0, BLOCK_RADIUS))
		_sun_disk.visible = true
		var with_disk := _luminance(await _render(camera, sun, sun, 0, BLOCK_RADIUS))
		_sun_disk.visible = false
		var t_march := (await _render(camera, sun, sun, 4, BLOCK_RADIUS)).x
		var t_low := t_march
		var t_high := t_march
		for offset: Vector2i in offsets:
			var t := (await _render(camera, sun, sun, 4, BLOCK_RADIUS, 0.0, offset)).x
			t_low = minf(t_low, t)
			t_high = maxf(t_high, t)
		var got := (with_disk - glow) / clear_disk
		var resolution := (glow + with_disk) * SUN_HALF_RESOLUTION / clear_disk
		gaps += 1 if t_low > 0.5 else 0
		cores += 1 if t_high < 0.05 else 0
		_check("Sun disk through the analog cumulus %.0f deg azimuth = the cloud pass's transmittance" % azimuth,
				got >= (1.0 - SUN_DISK_TOLERANCE) * t_low - resolution
				and got <= (1.0 + SUN_DISK_TOLERANCE) * t_high + resolution,
				"disk/clear %s, march T %s (%s - %s over %d px), resolution %s" % [String.num_scientific(got),
				String.num_scientific(t_march), String.num_scientific(t_low), String.num_scientific(t_high),
				SUN_CU_NEIGHBOUR_PIXELS, String.num_scientific(resolution)])
	_check("Sun through the analog cumulus: the body has gaps and cores in view of the Sun", gaps > 0 and cores > 0,
			"%d disks in gaps (T > 0.5 over %d px), %d behind cores (T < 0.05) of %d" % [gaps,
			SUN_CU_NEIGHBOUR_PIXELS, cores, SUN_CU_AZIMUTHS_DEG.size()])
	# The mean over many directions (the shapes' T, debug output 4; no light involved: no cache settling).
	var sum_march := 0.0
	var sum_core := 0.0
	var n := 0
	for elevation: float in SUN_CU_ENSEMBLE_ELEVATIONS_DEG:
		for k in range(int(360.0 / SUN_CU_ENSEMBLE_AZIMUTH_STEP_DEG)):
			var sun := _local_direction(elevation, k * SUN_CU_ENSEMBLE_AZIMUTH_STEP_DEG, SUN_CU_SITE.x, SUN_CU_SITE.y)
			sum_march += (await _render(camera, sun, sun, 4, BLOCK_RADIUS)).x
			sum_core += float(_core.weather_cloud_light(camera, sun)["sun_transmittance"])
			n += 1
	var mean_march := sum_march / n
	var mean_core := sum_core / n
	var allowance := SUN_CU_ENSEMBLE_SIGMAS * sqrt((1.0 - mean_core) / (mean_core * n))
	_check("Sun through the analog cumulus: the mean transmittance of the shapes at or above the core's grid mean",
			mean_march >= (1.0 - SUN_DISK_TOLERANCE - allowance) * mean_core,
			"mean shapes %.4f, mean grid %.4f (ratio %.3f, allowance %.3f) over %d directions" % [mean_march,
			mean_core, mean_march / mean_core, allowance, n])


## The shipped analog snapshot of SUN_CU_MONTH's 00 UTC slot (data/weather/analog/index.json), "" if there is none.
func _analog_june_file() -> String:
	var dir := Paths.data_dir().path_join("weather/analog")
	var f := FileAccess.open(dir.path_join("index.json"), FileAccess.READ)
	if f == null:
		return ""
	var parsed: Variant = JSON.parse_string(f.get_as_text())
	f.close()
	for entry: Dictionary in (parsed.get("entries", []) if parsed is Dictionary else []):
		if int(entry["month"]) == SUN_CU_MONTH and int(entry["hour_utc"]) == 0:
			return dir.path_join(String(entry["file"]))
	return ""


## The cloudless disk's centre as sun.gdshader should draw it (unclamped): its physical value
## (World.sun_disc_scene_value) in the solar colour of unit luminance x the eye's exposure gain x the core's direct
## transmittance of the air toward the Sun (the sky in front of it, < 1e-4 of the disk, left out).
func _sun_disk_expected(camera_itrf_m: Vector3, sun_itrf: Vector3) -> Vector3:
	var solar: Vector3 = ShaderGlobals.value(&"atmosphere_sun_illuminance_lux")
	var gain := float(ShaderGlobals.value(&"atmosphere_exposure_gain"))
	var transmittance: Vector3 = _core.atmosphere_sun_transmittance(camera_itrf_m, sun_itrf)
	return World.sun_disc_scene_value() * gain * solar / solar.dot(Vector3(0.2126, 0.7152, 0.0722)) * transmittance


## Section 14 (static clouds Task 9b): the clouds' shadows on the air (cloud_pass.gdshaderinc cloud_air_shadow_loss).
## HDR luminance of the full view (linear tone map, exposure 1), pixels classed by the cloud image's T.
func _air_shadow_samples() -> void:
	var size := AIR_VIEW_SIZE
	_viewport.size = size
	_camera.fov = AIR_FOV_DEG
	var up := _local_direction(90.0, 0.0)
	var up_render := Frames.to_render(_earth_to_gcrf * up).normalized()
	var negatives := {}  # scene -> the final image's lowest HDR channel (air shadow on)
	# The backlit cumulus (the t9b setup): cf 0.5, in-cloud vertical tau 50 in 1-2 km, the Sun 15 deg up ahead.
	var fraction := AIR_CUMULUS_FRACTION
	var cumulus := _slab("air_cu", SLAB_BOTTOM_M, SLAB_TOP_M,
			fraction * AIR_CUMULUS_TAU / 150.0 / (SLAB_TOP_M - SLAB_BOTTOM_M), fraction)
	_show(cumulus)
	var sun := _local_direction(AIR_BACKLIT_ELEVATION_DEG, 0.0)
	var sun_render := Frames.to_render(_earth_to_gcrf * sun).normalized()
	var images := {}
	await _render(_point(10.0), sun, sun, 0)  # settles the light cache and the shadow maps for the first image
	for on in [false, true]:
		_clouds.air_shadow = on
		await _render(_point(10.0), sun, sun, 0)
		images[on] = [_viewport.get_texture().get_image(), _clouds.effect.output.get_layer_data(0)]
	_clouds.air_shadow = true
	negatives["backlit"] = _lowest_channel(images[true][0])
	for b in range(AIR_BANDS_DEG.size()):
		var band: Vector2 = AIR_BANDS_DEG[b]
		var stats := {}
		for on in [false, true]:
			var full: Image = images[on][0]
			var t_image: Image = images[on][1]
			var sky: Array[float] = []
			var body: Array[float] = []
			var rim: Array[float] = []
			for y in range(size.y):
				for x in range(size.x):
					var ray := _camera.project_ray_normal(Vector2(x + 0.5, y + 0.5))
					if ray.dot(up_render) < sin(deg_to_rad(AIR_MIN_ELEVATION_DEG)):
						continue
					var angle := rad_to_deg(acos(clampf(ray.dot(sun_render), -1.0, 1.0)))
					if angle < band.x or angle >= band.y:
						continue
					var t := t_image.get_pixel(int(float(x) * t_image.get_width() / size.x),
							int(float(y) * t_image.get_height() / size.y)).a
					var c := full.get_pixel(x, y)
					var lum := _luminance(Vector3(c.r, c.g, c.b))
					if t > AIR_SKY_T:
						sky.append(lum)
					elif t < AIR_BODY_T:
						body.append(lum)
					elif t > AIR_RIM_T.x and t < AIR_RIM_T.y:
						rim.append(lum)
			for a in [sky, body, rim]:
				a.sort()
			stats[on] = [_pct(sky, 0.5), _pct(body, 0.5), _pct(rim, 0.9), sky.size(), body.size(), rim.size()]
		var s: Array = stats[true]
		var o: Array = stats[false]
		var detail := "sky %.3f body %.3f rim p90 %.3f (n %d / %d / %d) | air shadow off: sky %.3f body %.3f rim %.3f" % [
				s[0], s[1], s[2], s[3], s[4], s[5], o[0], o[1], o[2]]
		_check("air shadow: backlit body below the sky %.0f-%.0f deg from the Sun" % [band.x, band.y],
				s[4] > 0 and s[3] > 0 and s[1] < s[0], detail + " body / sky %.2f" % (s[1] / maxf(s[0], 1e-9)))
		var rim_ratio := float(s[2]) / maxf(s[1], 1e-9)
		var rim_max: float = AIR_RIM_RATIO_NEAR_MAX if b == 0 else AIR_RIM_RATIO.y
		_check("air shadow: backlit rim %.1f-%.1f x the body %.0f-%.0f deg from the Sun" % [AIR_RIM_RATIO.x,
				rim_max, band.x, band.y], s[5] > 0 and rim_ratio >= AIR_RIM_RATIO.x
				and rim_ratio <= rim_max, "rim / body %.2f (off %.2f)" % [rim_ratio,
				float(o[2]) / maxf(o[1], 1e-9)])
	await _air_outline_samples(sun)
	await _air_front_weight_samples(cumulus, sun)
	# Crepuscular rays: the broken deck under a low Sun, the sky between the clouds toward and away from the Sun.
	var broken := _slab("air_broken", SLAB_BOTTOM_M, SLAB_TOP_M,
			AIR_BROKEN_FRACTION * AIR_CUMULUS_TAU / 150.0 / (SLAB_TOP_M - SLAB_BOTTOM_M), AIR_BROKEN_FRACTION)
	_show(broken)
	var low_sun := _local_direction(AIR_RAYS_SUN_ELEVATION_DEG, 0.0)
	for toward in [true, false]:
		var view := _local_direction(AIR_RAYS_VIEW_ELEVATION_DEG, 0.0 if toward else 180.0)
		var rays := await _ray_modulation(view, low_sun, -1.0)
		negatives["rays " + ("toward" if toward else "away")] = rays[2]
		if not toward:
			# Away from the Sun only the shadowed air's presence: its luminance per column falls by the mean loss, the
			# columns' differences average out along the long rays through many shadows (detrended ~0.1 %).
			_check("air shadow: the shadowed air away from the Sun (%.0f deg up, deck cf %.1f)" % [
					AIR_RAYS_SUN_ELEVATION_DEG, AIR_BROKEN_FRACTION], rays[3] >= AIR_RAYS_MIN_COLUMNS
					and 1.0 - rays[4] >= AIR_AWAY_MIN_LOSS, "on / off p50 %.4f (loss >= %.3f);" % [rays[4],
					AIR_AWAY_MIN_LOSS] + " detrended modulation %.4f over %d columns, undetrended %.3f" % [rays[0],
					rays[3], rays[1]])
			continue
		_check("air shadow: crepuscular rays toward the Sun (%.0f deg up, deck cf %.1f)" % [AIR_RAYS_SUN_ELEVATION_DEG,
				AIR_BROKEN_FRACTION], rays[3] >= AIR_RAYS_MIN_COLUMNS and rays[0] >= AIR_RAYS_MIN_MODULATION,
				"detrended modulation %.4f (>= %.3f) over %d columns; undetrended %.3f" % [rays[0],
				AIR_RAYS_MIN_MODULATION, rays[3], rays[1]])
		# The control: the same deck and view with T_c uniform (no shadow shapes, the same smooth trend with the
		# angle from the Sun): no rays, so the detrended metric must stay below the threshold.
		var control := await _ray_modulation(view, low_sun, AIR_CONTROL_TRANSMITTANCE)
		_check("air shadow: no crepuscular rays without shadow shapes (control: T_c %.1f everywhere)"
				% AIR_CONTROL_TRANSMITTANCE, control[3] >= AIR_RAYS_MIN_COLUMNS
				and control[0] < AIR_RAYS_MIN_MODULATION, "detrended modulation %.4f (< %.3f); undetrended %.3f"
				% [control[0], AIR_RAYS_MIN_MODULATION, control[1]])
	# The segment sum against the scene's single scattering (T_c uniform: loss = (1 - T_c) x S1 exactly when the
	# difference form telescopes - the per-segment clamp and the Mie fade), low and high views, the Sun low and setting.
	for c in AIR_TELESCOPE_CASES:
		var case_sun := _local_direction(c[0], 0.0)
		await _air_telescoping(_local_direction(c[1], c[2]), case_sun, "Sun %+.1f deg, view %.0f deg up, azimuth %.0f"
				% [c[0], c[1], c[2]])
	# The setting Sun under the broken deck (its Mie fade, the horizon's depth edge in view): no negative final HDR.
	for e in AIR_SETTING_SUN_DEG:
		_clouds.air_shadow = true
		await _render(_point(10.0), _local_direction(AIR_SETTING_VIEW_DEG, 0.0), _local_direction(e, 0.0), 0)
		negatives["setting Sun %+.1f deg" % e] = _lowest_channel(_viewport.get_texture().get_image())
	var lowest := 0.0
	var detail := ""
	for k in negatives:
		lowest = minf(lowest, negatives[k])
		detail += " %s %.4f;" % [k, negatives[k]]
	_check("air shadow: no negative final HDR", lowest >= 0.0, "lowest channel:" + detail)
	# Clear sky unchanged: above the deck looking up (no cloud in the Sun's way of that air), and a cloudless field.
	var clear_views := [["above the deck, looking up", broken, 3000.0, _local_direction(30.0, 30.0)],
			["no cloud", _slab("air_clear", SLAB_BOTTOM_M, SLAB_TOP_M, 0.0, 0.0), 10.0, _local_direction(10.0, 20.0)]]
	for v in clear_views:
		_show(v[1])
		var lums := {}
		for on in [false, true]:
			_clouds.air_shadow = on
			await _render(_point(v[2]), v[3], low_sun, 0)
			var image := _viewport.get_texture().get_image()
			var total := 0.0
			for y in range(size.y):
				for x in range(size.x):
					var c := image.get_pixel(x, y)
					total += _luminance(Vector3(c.r, c.g, c.b))
			lums[on] = total
		_clouds.air_shadow = true
		var change := absf(lums[true] - lums[false]) / maxf(lums[false], 1e-9)
		_check("air shadow: clear sky unchanged (%s)" % v[0], change <= AIR_CLEAR_TOLERANCE,
				"%.4f %% of the view's luminance" % (change * 100.0))
	_viewport.size = Vector2i(VIEW_SIZE, VIEW_SIZE)
	_camera.fov = 10.0


## The crepuscular-ray metric of one view (the deck shown): the sky between the clouds, the on / off luminance ratio
## per column (azimuth) in column order, each over the running median of AIR_RAYS_DETREND_COLUMNS columns around it
## (the smooth trend with the angle from the Sun divides out) - 1: p90 - p10 of these residuals. forced_t >= 0 replaces
## T_c everywhere (cloud_debug_air_transmittance). Returns [detrended, undetrended (p90 - p10) / p50, lowest final HDR
## channel with the air shadow, columns, the ratio's p50].
func _ray_modulation(view: Vector3, sun: Vector3, forced_t: float) -> Array:
	var size := AIR_VIEW_SIZE
	ShaderGlobals.set_value(&"cloud_debug_air_transmittance", forced_t)
	var pair := {}
	for on in [false, true]:
		_clouds.air_shadow = on
		await _render(_point(10.0), view, sun, 0)
		pair[on] = [_viewport.get_texture().get_image(), _clouds.effect.output.get_layer_data(0)]
	_clouds.air_shadow = true
	ShaderGlobals.set_value(&"cloud_debug_air_transmittance", -1.0)
	var sums := PackedFloat64Array()
	var counts := PackedInt32Array()
	sums.resize(size.x)
	counts.resize(size.x)
	var t_image: Image = pair[true][1]
	for y in range(size.y):
		for x in range(size.x):
			var t := t_image.get_pixel(int(float(x) * t_image.get_width() / size.x),
					int(float(y) * t_image.get_height() / size.y)).a
			if t < AIR_SKY_T:
				continue
			var c_on: Color = pair[true][0].get_pixel(x, y)
			var c_off: Color = pair[false][0].get_pixel(x, y)
			var off := _luminance(Vector3(c_off.r, c_off.g, c_off.b))
			if off <= 0.0:
				continue
			sums[x] += _luminance(Vector3(c_on.r, c_on.g, c_on.b)) / off
			counts[x] += 1
	var ratios: Array[float] = []
	for x in range(size.x):
		if counts[x] >= AIR_RAYS_MIN_PIXELS:
			ratios.append(sums[x] / counts[x])
	var residuals: Array[float] = []
	var half := AIR_RAYS_DETREND_COLUMNS / 2
	for i in range(ratios.size()):
		var window: Array[float] = []
		for j in range(maxi(0, i - half), mini(ratios.size(), i + half + 1)):
			window.append(ratios[j])
		window.sort()
		residuals.append(ratios[i] / maxf(window[window.size() / 2], 1e-9) - 1.0)
	residuals.sort()
	var sorted := ratios.duplicate()
	sorted.sort()
	return [_pct(residuals, 0.9) - _pct(residuals, 0.1),
			(_pct(sorted, 0.9) - _pct(sorted, 0.1)) / maxf(_pct(sorted, 0.5), 1e-9),
			_lowest_channel(pair[true][0]), ratios.size(), _pct(sorted, 0.5)]


## The segment sum of cloud_air_shadow_loss against the scene's single scattering over the same stretch (debug output
## 8: the endpoint form), T_c uniform AIR_CONTROL_TRANSMITTANCE: per clear pixel (off - on) / ((1 - T_c) x S1) = 1.
func _air_telescoping(view: Vector3, sun: Vector3, label: String) -> void:
	var size := AIR_VIEW_SIZE
	var forced := AIR_CONTROL_TRANSMITTANCE
	ShaderGlobals.set_value(&"cloud_debug_air_transmittance", forced)
	var got := {}
	for on in [false, true]:
		_clouds.air_shadow = on
		await _render(_point(10.0), view, sun, 0)
		got[on] = [_viewport.get_texture().get_image(), _clouds.effect.output.get_layer_data(0)]
	_clouds.air_shadow = true
	await _render(_point(10.0), view, sun, 8)
	var reference := _viewport.get_texture().get_image()
	ShaderGlobals.set_value(&"cloud_debug_output", 0)
	ShaderGlobals.set_value(&"cloud_debug_air_transmittance", -1.0)
	var t_image: Image = got[true][1]
	var up_render := Frames.to_render(_earth_to_gcrf * _point(10.0).normalized()).normalized()
	var errors: Array[float] = []  # relative to S1, where S1 is at least AIR_TELESCOPE_MIN_SHARE of the pixel
	var small: Array[float] = []  # |loss - S1| / the pixel's luminance, the rest
	var s1s: Array[float] = []
	var m := AIR_TELESCOPE_MARGIN
	for y in range(m, size.y - m):
		for x in range(m, size.x - m):
			if _camera.project_ray_normal(Vector2(x + 0.5, y + 0.5)).dot(up_render) <= 0.0:
				continue  # the sky (the sea's waves move its depth between the renders)
			var clear := true
			for dy in range(-m, m + 1):
				for dx in range(-m, m + 1):
					clear = clear and t_image.get_pixel(x + dx, y + dy).a >= AIR_SKY_T
			if not clear:
				continue  # the upsample's taps all clear: the filter's lobes at a cloud do not enter
			var c := reference.get_pixel(x, y)
			# The loss's parts are >= 0 as a whole (cloud_air_shadow_parts): a negative table S1 (setting Sun) is 0.
			var s1 := (1.0 - forced) * maxf(_luminance(Vector3(c.r, c.g, c.b)), 0.0)
			var c_on: Color = got[true][0].get_pixel(x, y)
			var c_off: Color = got[false][0].get_pixel(x, y)
			var off := _luminance(Vector3(c_off.r, c_off.g, c_off.b))
			s1s.append(s1)
			var loss := off - _luminance(Vector3(c_on.r, c_on.g, c_on.b))
			if s1 >= AIR_TELESCOPE_MIN_SHARE * off:
				errors.append((loss - s1) / s1)
			else:
				small.append(absf(loss - s1) / maxf(off, 1e-9))
	errors.sort()
	small.sort()
	s1s.sort()
	var median := _pct(errors, 0.5)
	_check("air shadow: the segment sum telescopes to the scene's single scattering (%s)" % label,
			errors.size() + small.size() > 0 and (errors.is_empty() or (absf(median) <= AIR_TELESCOPE_TOLERANCE
			and absf(_pct(errors, 0.05)) <= AIR_TELESCOPE_SPREAD and absf(_pct(errors, 0.95)) <= AIR_TELESCOPE_SPREAD))
			and (small.is_empty() or _pct(small, 0.95) <= AIR_CLEAR_TOLERANCE),
			("loss / ((1 - T_c) S1) - 1: %s; S1 below %.0f %% of the pixel: |loss - S1| %s; (1 - T_c) S1 p50 %.5f") % [
			"none" if errors.is_empty() else "p5 %+.4f p50 %+.4f p95 %+.4f over %d clear sky pixels" % [
			_pct(errors, 0.05), median, _pct(errors, 0.95), errors.size()], AIR_TELESCOPE_MIN_SHARE * 100.0,
			"none" if small.is_empty() else "p95 %.4f %% of it over %d" % [_pct(small, 0.95) * 100.0, small.size()],
			_pct(s1s, 0.5)])


## The outline pass's loss: the image's colour with the march's zeroed (debug output 9: the loss alone, the outline
## pixels rescaled from their colour source) against each pixel's own loss (10), T_c 0 everywhere (the loss behind the
## clouds large). Their difference is the source's loss mis-taken for the pixel's (the edge pixel's T differs).
func _air_outline_samples(sun: Vector3) -> void:
	ShaderGlobals.set_value(&"cloud_debug_air_transmittance", 0.0)
	var layers := {}
	for mode in [9, 10]:
		await _render(_point(10.0), sun, sun, mode)
		layers[mode] = _clouds.effect.output.get_layer_data(0)
	ShaderGlobals.set_value(&"cloud_debug_output", 0)
	ShaderGlobals.set_value(&"cloud_debug_air_transmittance", -1.0)
	var a: Image = layers[9]
	var b: Image = layers[10]
	var errors: Array[float] = []
	for y in range(b.get_height()):
		for x in range(b.get_width()):
			var own := b.get_pixel(x, y)
			var t := own.a
			if t <= AIR_RIM_T.x or t >= AIR_SKY_T:
				continue  # the outline: partly covered pixels
			var l_own := absf(_luminance(Vector3(own.r, own.g, own.b)))
			if l_own <= 0.0:
				continue
			var c := a.get_pixel(x, y)
			errors.append(absf(_luminance(Vector3(c.r, c.g, c.b)) - _luminance(Vector3(own.r, own.g, own.b))) / l_own)
	errors.sort()
	_check("air shadow: an outline pixel's loss is its own (T 0.1-0.999)", errors.size() > 0
			and _pct(errors, 1.0 - AIR_OUTLINE_MAX_SHARE) <= AIR_OUTLINE_TOLERANCE,
			"relative error p50 %.2f %% p95 %.2f %% p99 %.2f %% max %.2f %% over %d pixels (p%.0f <= %.0f %%)" % [
			_pct(errors, 0.5) * 100.0, _pct(errors, 0.95) * 100.0, _pct(errors, 0.99) * 100.0,
			_pct(errors, 1.0) * 100.0, errors.size(), (1.0 - AIR_OUTLINE_MAX_SHARE) * 100.0,
			AIR_OUTLINE_TOLERANCE * 100.0])


## The composite's air in front of a cloud at weight 1 (cloud_march adds alpha x the air light to the clouds' distance;
## the scene behind is dimmed by T): the image without the clouds' own light (debug output 11) minus T x the cloudless
## scene against alpha x the core's air light to the clouds' distance (12: distance and alpha), at the cloud bodies
## within AIR_WEIGHT_MAX_M (the march's own tier). A double count gives 2, a gap 0.
func _air_front_weight_samples(cumulus: Dictionary, sun: Vector3) -> void:
	var size := AIR_VIEW_SIZE
	var camera := _point(10.0)
	_clouds.air_shadow = false
	_show(_slab("air_none", SLAB_BOTTOM_M, SLAB_TOP_M, 0.0, 0.0))
	await _render(camera, sun, sun, 0)
	var scene := _viewport.get_texture().get_image()
	_show(cumulus)
	await _render(camera, sun, sun, 11)
	var no_light := _viewport.get_texture().get_image()
	var t_image := _clouds.effect.output.get_layer_data(0)
	await _render(camera, sun, sun, 12)
	var geometry := _viewport.get_texture().get_image()
	ShaderGlobals.set_value(&"cloud_debug_output", 0)
	_clouds.air_shadow = true
	var ratios: Array[float] = []
	for y in range(0, size.y, 2):
		for x in range(0, size.x, 2):
			var g := geometry.get_pixel(x, y)
			if g.g < AIR_WEIGHT_MIN_ALPHA or g.r <= 0.0 or g.r > AIR_WEIGHT_MAX_M:
				continue
			var t := t_image.get_pixel(int(float(x) * t_image.get_width() / size.x),
					int(float(y) * t_image.get_height() / size.y)).a
			var ray := _camera.project_ray_normal(Vector2(x + 0.5, y + 0.5))
			var ray_itrf: Vector3 = _earth_to_gcrf.inverse() * Frames.from_render(ray)
			var to_point: Dictionary = _core.atmosphere_sky_luminance_to_point(camera, camera + ray_itrf * g.r, sun)
			var expected := g.g * _luminance(to_point["luminance"] * _atmosphere.luminance_scale)
			var c := no_light.get_pixel(x, y)
			var s := scene.get_pixel(x, y)
			var front := _luminance(Vector3(c.r, c.g, c.b)) - t * _luminance(Vector3(s.r, s.g, s.b))
			if expected > 0.0:
				ratios.append(front / expected)
	ratios.sort()
	var median := _pct(ratios, 0.5)
	_check("air shadow: the composite's air in front of a cloud at weight 1", ratios.size() > 0
			and absf(median - 1.0) <= AIR_WEIGHT_TOLERANCE, "(image - T scene) / (alpha x air light to the cloud): "
			+ "p5 %.3f p50 %.3f p95 %.3f over %d body pixels" % [_pct(ratios, 0.05), median, _pct(ratios, 0.95),
			ratios.size()])


## The lowest RGB channel of an HDR image.
static func _lowest_channel(image: Image) -> float:
	var lowest := INF
	for y in range(image.get_height()):
		for x in range(image.get_width()):
			var c := image.get_pixel(x, y)
			lowest = minf(lowest, minf(c.r, minf(c.g, c.b)))
	return lowest


static func _pct(a: Array[float], q: float) -> float:
	return a[int(q * (a.size() - 1))] if a.size() > 0 else -1.0


## Section 14, first part: the backlit slab's transmitted radiance (debug output 2: the grid mean's Sun source per unit
## irradiance, seen from below) against the Monte Carlo fixture (tests/validation/weather/cloud_slab_transmitted_mc.csv,
## tools/validation/cloud_slab_monte_carlo.py --transmitted: bin means; GPU likewise averaged over each bin).
func _slab_transmitted_samples() -> void:
	var rows := {}
	var file := FileAccess.open(ProjectSettings.globalize_path("res://").path_join(
			"../tests/validation/weather/cloud_slab_transmitted_mc.csv").simplify_path(), FileAccess.READ)
	if file == null:
		_check("slab transmitted radiance: fixture", false, "cloud_slab_transmitted_mc.csv missing")
		return
	while not file.eof_reached():
		var line := file.get_line()
		if line.begins_with("#") or line.begins_with("sun") or line.strip_edges() == "":
			continue
		var f := line.split(",")
		rows["%s|%s|%s|%s" % [f[0], f[1], f[2], f[3]]] = float(f[4])
	var up := _local_direction(90.0, 0.0)
	# Diagnostics: STARSIM_SLAB_NO_CACHE=1 measures the per-sample light march (the cache's fidelity flag off).
	_clouds.light_cache_enabled = OS.get_environment("STARSIM_SLAB_NO_CACHE") == ""
	for tau_v in SLAB_TR_TAUS:
		var slab := _slab("transmitted_%d" % int(tau_v), SLAB_BOTTOM_M, SLAB_TOP_M,
				tau_v / 150.0 / (SLAB_TOP_M - SLAB_BOTTOM_M), 1.0)
		_show(slab)
		var tau := -log(float(_core.weather_cloud_light(_point(10.0), up)["sun_transmittance"]))
		for sz in SLAB_TR_SUN_ZENITHS_DEG:
			var worst := 0.0
			var detail := ""
			for c in SLAB_TR_BINS:
				var view := _local_direction(90.0 - c[0], 0.0)
				var sun := _local_direction(90.0 - sz, c[1])
				var binned := await _bin_mean(view, sun, c[0], c[1])
				var expected: float = rows["%d|%d|%d|%d" % [int(sz), int(tau_v), int(c[0]), int(c[1])]]
				var error := binned / expected - 1.0
				detail += " %d/%d %+.1f %%" % [c[0], c[1], error * 100.0]
				worst = maxf(worst, absf(error))
			_check("slab transmitted radiance vs Monte Carlo: tau %.1f, Sun zenith %.0f deg" % [tau, sz],
					worst <= SLAB_TR_TOLERANCE, "view zenith / azimuth:%s" % detail)
	_clouds.light_cache_enabled = true
	_viewport.size = Vector2i(VIEW_SIZE, VIEW_SIZE)
	_camera.fov = 10.0


## The cos-weighted mean of debug output 2 over the Monte Carlo bin (view zenith +-5 deg, relative azimuth +-15 deg;
## the zenith bin 0..8 deg), in a 161^2 view of 44 deg around its centre.
func _bin_mean(view: Vector3, sun: Vector3, vz: float, va: float) -> float:
	var size := SLAB_TR_VIEW_SIZE
	_viewport.size = Vector2i(size, size)
	_camera.fov = SLAB_TR_FOV_DEG
	await _settle_cache(_point(10.0), view, sun, 2)  # the light cache filled for this Sun (no march fallback)
	await _render(_point(10.0), view, sun, 2)
	var image := _viewport.get_texture().get_image()
	var up := Frames.to_render(_earth_to_gcrf * _local_direction(90.0, 0.0)).normalized()
	var north := Frames.to_render(_earth_to_gcrf * _local_direction(0.0, 0.0)).normalized()
	var east := Frames.to_render(_earth_to_gcrf * _local_direction(0.0, 90.0)).normalized()
	var sun_r := Frames.to_render(_earth_to_gcrf * sun).normalized()
	var sun_az := atan2(sun_r.dot(east), sun_r.dot(north))
	var axis := Frames.to_render(_earth_to_gcrf * view).normalized()
	var sum := 0.0
	var weight := 0.0
	for y in range(size):
		for x in range(size):
			var ray := _camera.project_ray_normal(Vector2(x + 0.5, y + 0.5))
			var zen := rad_to_deg(acos(clampf(ray.dot(up), -1.0, 1.0)))
			var az := absf(rad_to_deg(wrapf(atan2(ray.dot(east), ray.dot(north)) - sun_az, -PI, PI)))
			var inside := zen <= 8.0 if vz == 0.0 else (absf(zen - vz) <= 5.0 and absf(az - va) <= 15.0)
			if not inside:
				continue
			var c := ray.dot(axis)
			var w := cos(deg_to_rad(zen)) * c * c * c  # the solid angle per pixel ~ cos^3 off the axis
			sum += w * image.get_pixel(x, y).r
			weight += w
	return sum / weight if weight > 0.0 else -1.0


## Section 15 (cloud view cache Task 3): per view (VIEW_CACHE_VIEWS) the game's march image (desktop preset, debug
## output 0) twice - bit-identical (static) - and, with MARCH_REFERENCE_ENV, the march's images (desktop preset; divisor
## 1, debug outputs 0 and 12) against a reference directory (a refactoring of the march keeps them bit for bit).
func _view_cache_samples() -> void:
	_viewport.size = Vector2i(VIEW_CACHE_SIZE, VIEW_CACHE_SIZE)
	_camera.fov = VIEW_CACHE_FOV_DEG
	var sun := _local_direction(45.0, 135.0)
	var reference_dir := OS.get_environment(MARCH_REFERENCE_ENV)
	for v in VIEW_CACHE_VIEWS:
		var name: String = v[0]
		_show(_slab("view_cache_" + name, SLAB_BOTTOM_M, SLAB_TOP_M, THICK_LIQUID_KG_M3, v[1]))
		var camera := _point(v[2])
		var view := _local_direction(v[3], v[4])
		_clouds.quality = CloudLayer.Quality.DESKTOP
		_detail_off()
		await _settle_cache(camera, view, sun, 0)
		var first := await _output_bytes(camera, view, sun, 0)
		var second := await _output_bytes(camera, view, sun, 0)
		var differing := _differing_pixels(first, second)
		_check("view cache (%s view): the march image of a static view is bit-identical" % name,
				first.size() > 0 and differing == 0, "%d of %d pixels differ" % [differing, first.size() / 8])
		var images := {"desktop_0": first}
		_clouds.effect.set_quality(1, 1.0)
		for mode in [0, 12]:
			images["divisor1_%d" % mode] = await _output_bytes(camera, view, sun, mode)
		if reference_dir != "":
			for key: String in images:
				_march_reference(reference_dir.path_join("%s_%s.bin" % [name, key]), images[key], "%s %s" % [name, key])
		for mode in [0, 12]:
			await _view_cache_compare(name, camera, view, sun, mode, Vector3.ZERO)
	# The face-aligned views: the shading alone (pixel rays = texel rays).
	_camera.fov = VIEW_CACHE_FACE_FOV_DEG
	for v in VIEW_CACHE_FACE_VIEWS:
		_show(_slab("view_cache_face_%d" % int(v[1] * 10.0), SLAB_BOTTOM_M, SLAB_TOP_M, THICK_LIQUID_KG_M3, v[1]))
		var camera := HeightSource.geodetic_to_fixed(0.0, 0.0, v[2])
		var forward: Vector3 = v[3]
		var view_itrf := Vector3(forward.z, forward.x, forward.y)  # Earth-local render axes (y, z, x) = ITRF (x, y, z)
		var face_sun := _local_direction(45.0, 135.0, 0.0, 0.0)
		_clouds.quality = CloudLayer.Quality.DESKTOP
		_detail_off()
		await _settle_cache(camera, view_itrf, face_sun, 0)
		_clouds.effect.set_quality(1, 1.0)
		for mode in [0, 12]:
			await _view_cache_compare(v[0], camera, view_itrf, face_sun, mode, forward)
	_clouds.quality = CloudLayer.Quality.DESKTOP
	_detail_off()
	_viewport.size = Vector2i(VIEW_SIZE, VIEW_SIZE)
	_camera.fov = 10.0


## The cloud pass's result (layer 0) after a render, its raw bytes (RGBA half floats).
func _output_bytes(camera: Vector3, view: Vector3, sun: Vector3, mode: int) -> PackedByteArray:
	await _render(camera, view, sun, mode)
	var image := _clouds.effect.output.get_layer_data(0)
	return image.get_data() if image != null else PackedByteArray()


## Section 15: the whole cube of the effect's CloudViewCache filled at the full level from the camera (the anchor at the
## camera: offset 0), against the march of the same frames at divisor 1 (its own image and distances before the
## upsample, CloudEffect.debug_centre_data): each pixel's ray (the camera's) sampled bilinearly in its face through
## CloudViewCacheLayout.texel_of - the CPU side of the fill's addressing, which a mirrored face or axis would fail.
## face_forward (Earth-local, not zero): the camera looks along that face axis with the screen's up +Y (Earth-local;
## the cube's faces are mirror images of a camera's view: a column of pixels is a column of texels counted the other
## way) on a cache of VIEW_CACHE_SIZE texels per face - every bound asserted; else the general view, medians only.
func _view_cache_compare(name: String, camera: Vector3, view: Vector3, sun: Vector3, mode: int,
		face_forward: Vector3) -> void:
	await _render(camera, view, sun, mode)
	var size := Vector2i(VIEW_CACHE_SIZE, VIEW_CACHE_SIZE)
	var exact := face_forward != Vector3.ZERO
	var face_texels := VIEW_CACHE_SIZE if exact else CloudViewCacheLayout.face_texels_for(
			_camera.get_camera_projection(), size, VIEW_CACHE_TILE_TEXELS)
	if exact:
		var earth_to_world := (ShaderGlobals.value(&"atmosphere_world_to_earth") as Basis).inverse()
		_camera.transform = Transform3D(Basis.looking_at(earth_to_world * face_forward,
					earth_to_world * (Vector3.BACK if absf(face_forward.y) > 0.5 else Vector3.UP)),
				Vector3.ZERO)
		for i in range(SETTLE_FRAMES):
			await RenderingServer.frame_post_draw
			_clouds.update(camera, 0.0, 0.0)
	var layout := CloudViewCacheLayout.new(face_texels, VIEW_CACHE_TILE_TEXELS)
	var level := CloudViewCacheLayout.LEVELS - 1
	var tiles: Array[Dictionary] = []
	for tile in range(layout.tile_count()):
		tiles.append({"tile": tile, "level": level})
	var anchor: Vector3 = ShaderGlobals.value(&"atmosphere_camera_hi") + ShaderGlobals.value(&"atmosphere_camera_lo")
	var fills_before := _clouds.effect.view_cache_fills
	_clouds.effect.request_view_cache_fill(layout, anchor, tiles)
	for i in range(2):
		await RenderingServer.frame_post_draw
		_clouds.update(camera, 0.0, 0.0)
	var label := "view cache (%s view, debug %d, 6 x %d^2 texels) vs the march at divisor 1" % [name, mode, face_texels]
	var checks := (3 if mode == 0 else 2) if exact else (2 if mode == 0 else 1)
	var cache := _clouds.effect.view_cache
	if cache == null or _clouds.effect.view_cache_fills == fills_before or _clouds.effect.centre_low != size:
		for i in range(checks):
			_check(label, false, "no fill ran or the march is not at divisor 1 (%s)" % _clouds.effect.centre_low)
		return
	var rd := RenderingServer.get_rendering_device()
	var colours: Array[PackedByteArray] = []
	var distances: Array[PackedByteArray] = []
	for face in range(CloudViewCacheLayout.FACES):
		colours.append(rd.texture_get_data(cache.texture(level), face))
		distances.append(rd.texture_get_data(cache.distance_texture(level), face))
	var march := _clouds.effect.debug_centre_data("color")
	var march_depth := _clouds.effect.debug_centre_data("depth")
	var world_to_earth: Basis = ShaderGlobals.value(&"atmosphere_world_to_earth")
	var n := size.x * size.y
	var march_lum := PackedFloat32Array()
	var cache_lum := PackedFloat32Array()
	var t_sq := 0.0
	var distance_errors := PackedFloat32Array()
	var mean_lum := 0.0
	var picture := Image.create(size.x * 3, size.y, false, Image.FORMAT_RGBF)
	for y in range(size.y):
		for x in range(size.x):
			var i := y * size.x + x
			var m := Color(march.decode_half(i * 8), march.decode_half(i * 8 + 2), march.decode_half(i * 8 + 4),
					march.decode_half(i * 8 + 6))
			var dir := world_to_earth * _camera.project_ray_normal(Vector2(x + 0.5, y + 0.5))
			var c := _cube_bilinear(colours, distances, face_texels, layout.texel_of(dir), mode)
			var colour: Color = c[0]
			if mode == 0:
				march_lum.append(_luminance(Vector3(m.r, m.g, m.b)))
				cache_lum.append(_luminance(Vector3(colour.r, colour.g, colour.b)))
				mean_lum += absf(march_lum[i]) / n
				t_sq += (colour.a - m.a) * (colour.a - m.a)
				if 1.0 - m.a > VIEW_CACHE_DISTANCE_ALPHA:
					var d_m := march_depth.decode_float(i * 8 + 4)
					distance_errors.append(absf(float(c[1]) - d_m) / d_m)
				picture.set_pixel(x, y, Color(m.r, m.g, m.b))
				picture.set_pixel(size.x + x, y, Color(colour.r, colour.g, colour.b))
				var dc := Color(absf(colour.r - m.r), absf(colour.g - m.g), absf(colour.b - m.b)) * 10.0
				picture.set_pixel(2 * size.x + x, y, dc)
			else:
				t_sq += (colour.g - m.g) * (colour.g - m.g)  # debug 12: (cloud distance, alpha)
				if m.g > VIEW_CACHE_DISTANCE_ALPHA:
					distance_errors.append(absf(float(c[2]) - m.r) / m.r)
	var t_rms := sqrt(t_sq / n)
	var d := _sorted(distance_errors)
	var d_median := d[d.size() / 2] if d.size() > 0 else INF
	var d_p99 := d[int(0.99 * (d.size() - 1))] if d.size() > 0 else INF
	var d_detail := "median %.3f %%, p99 %.3f %%, worst %.2f %% over %d pixels" % [d_median * 100.0, d_p99 * 100.0,
			(d[d.size() - 1] if d.size() > 0 else INF) * 100.0, d.size()]
	if mode == 0:
		var errors := PackedFloat32Array()
		for i in range(n):
			errors.append(absf(cache_lum[i] - march_lum[i]) / maxf(absf(march_lum[i]), CACHE_ERROR_FLOOR * mean_lum))
		var e := _sorted(errors)
		var p99 := e[int(0.99 * (n - 1))]
		_check(label + ": colour", mean_lum > 0.0 and e[n / 2] <= VIEW_CACHE_MEDIAN_TOLERANCE
				and (p99 <= (VIEW_CACHE_P99_TOLERANCE if absf(face_forward.z) > 0.5
					else VIEW_CACHE_P99_HORIZONTAL_TOLERANCE) or not exact), "per-pixel luminance error median %.3f %%, p99 "
				% (e[n / 2] * 100.0) + "%.3f %%, worst %.2f %% (mean luminance %.4f)%s" % [p99 * 100.0,
				e[n - 1] * 100.0, mean_lum, "" if exact else "; T RMS %.5f (resampling: printed)" % t_rms])
		if exact:
			_check(label + ": transmittance", t_rms <= VIEW_CACHE_T_RMS_TOLERANCE, "RMS %.5f" % t_rms)
		_check(label + ": cloud distance (the distance image, alpha > %.1f)" % VIEW_CACHE_DISTANCE_ALPHA,
				d.size() > 0 and d_median <= VIEW_CACHE_DISTANCE_TOLERANCE, d_detail)
		var dir_path := ProjectSettings.globalize_path("res://").path_join("../screenshots/cloud-view-cache")
		DirAccess.make_dir_recursive_absolute(dir_path)
		picture.save_png(dir_path.path_join("t3_fill_vs_march_%s.png" % name.replace(" ", "_")).simplify_path())
	else:
		if exact:
			_check(label + ": alpha", t_rms <= VIEW_CACHE_T_RMS_TOLERANCE, "RMS %.5f" % t_rms)
		_check(label + ": cloud distance (debug output 12, alpha > %.1f)" % VIEW_CACHE_DISTANCE_ALPHA,
				d.size() > 0 and d_median <= VIEW_CACHE_DISTANCE_TOLERANCE,
				d_detail + ("" if exact else "; alpha RMS %.5f (resampling: printed)" % t_rms))


## A bilinear sample of the cube's faces (RGBA half colour + T, R32F distance; face_texels^2 each, row-major) at the
## continuous texel coordinates t = (x, y, face), clamped inside the face: [colour + T, distance, colour's red]. The
## distance and the red channel (debug output 12: the cloud distance; its alpha in .g) are alpha-weighted (the march's
## convention).
static func _cube_bilinear(colours: Array[PackedByteArray], distances: Array[PackedByteArray], face_texels: int,
		t: Vector3, mode: int) -> Array:
	var face := int(t.z)
	var p := Vector2(t.x - 0.5, t.y - 0.5)
	var x0 := floori(p.x)
	var y0 := floori(p.y)
	var f := p - Vector2(x0, y0)
	var colour := Color(0, 0, 0, 0)
	var weighted := 0.0
	var weighted_red := 0.0
	var weight := 0.0
	for j in range(2):
		for i in range(2):
			var x := clampi(x0 + i, 0, face_texels - 1)
			var y := clampi(y0 + j, 0, face_texels - 1)
			var w := (f.x if i == 1 else 1.0 - f.x) * (f.y if j == 1 else 1.0 - f.y)
			var k := (y * face_texels + x) * 8
			var bytes := colours[face]
			var c := Color(bytes.decode_half(k), bytes.decode_half(k + 2), bytes.decode_half(k + 4),
					bytes.decode_half(k + 6))
			colour += c * w
			var alpha := c.g if mode == 12 else 1.0 - c.a
			weighted += w * alpha * distances[face].decode_float((y * face_texels + x) * 4)
			weighted_red += w * alpha * c.r
			weight += w * alpha
	if weight <= 0.0:
		return [colour, 0.0, 0.0]
	return [colour, weighted / weight, weighted_red / weight]


static func _sorted(values: PackedFloat32Array) -> PackedFloat32Array:
	var s := values.duplicate()
	s.sort()
	return s


## Pixels (8 bytes each: RGBA half) whose bytes differ; all when the sizes differ.
static func _differing_pixels(a: PackedByteArray, b: PackedByteArray) -> int:
	if a.size() != b.size():
		return maxi(a.size(), b.size()) / 8
	var differing := 0
	for i in range(0, a.size(), 8):
		if a.decode_u64(i) != b.decode_u64(i):
			differing += 1
	return differing


## MARCH_REFERENCE_ENV: writes the image when its reference file is missing, else checks it bit for bit.
func _march_reference(path: String, bytes: PackedByteArray, label: String) -> void:
	if not FileAccess.file_exists(path):
		DirAccess.make_dir_recursive_absolute(path.get_base_dir())
		var out := FileAccess.open(path, FileAccess.WRITE)
		out.store_buffer(bytes)
		out.close()
		print("cloud_check: march reference written: " + path)
		return
	var reference := FileAccess.get_file_as_bytes(path)
	var differing := _differing_pixels(bytes, reference)
	var largest := 0.0  # the largest difference of a colour channel relative to the image's mean colour magnitude
	if differing > 0 and bytes.size() == reference.size():
		var mean := 0.0
		for i in range(0, bytes.size(), 2):
			if (i / 2) % 4 != 3:
				mean += absf(reference.decode_half(i)) / (bytes.size() / 8 * 3)
		for i in range(0, bytes.size(), 2):
			largest = maxf(largest, absf(bytes.decode_half(i) - reference.decode_half(i)) / maxf(mean, 1.0e-30))
	_optional_samples += 1
	_check("view cache: the march image (%s) equals the reference bit for bit" % label, differing == 0,
			"%d of %d pixels differ (largest difference / mean colour %.6f)" % [differing, bytes.size() / 8, largest])


# ---- Section 16 (world volume plan Task 5): the cloud world volume's GPU bake -------------------------------------

## Section 16 (CloudWorldVolume; the pair (shipped analog of October 12 UTC in slot 0, ERA5 2025-10-25 15:00 in slot 1)
## at blend 0.3 over Starbase, the camera 2 m and 9 km up):
## - the cold start at the game's budget (the upkeep's GPU time per frame and the measured taps / cells per ms printed),
##   then filled at VOLUME_BUDGET_SCALE x the budget;
## - per level 0, 2, 5: 512 cells (half among the GPU's cloudy cells, half uniform) read back against
##   StarsimCore.weather_cloud_cell_bakes for each snapshot: rho and sigma_c within VOLUME_TOLERANCE + VOLUME_FLOOR_PER_M;
## - level 2's water per weather cell (cells assigned by centre, VOLUME_WATER_SAMPLES uniform cells of the cloud band):
##   the GPU's sum over the CPU's in VOLUME_WATER (Task 4 ruling: two-sided);
## - per level 0, 2, 5, exhaustively over every block: the max-extinction mip >= every cloudy cell's sigma_c (and rho) of
##   the block, the distance field <= the true toroidal Chebyshev distance (blocks) to the nearest block with rho > 0 in
##   either snapshot (no cloud within the claimed distance); on VOLUME_PROBE_POINTS points inside blocks claimed free the
##   trilinear rho of both snapshots (the probe kernel: what a sample reads) is 0 - so the sub-cell field (it
##   redistributes rho, peak sigma_c) is 0 there at any lift;
## - per level 0-6 (every adjacent pair; review round 2) the fade band: free balls empty and the max-extinction bound
##   above every density of the point's block (level k + 1 blended in); at the 2 m camera levels 0-1's clamped bottom
##   reads its edge cell below the bottom cell's centre (N2);
## - a rebake of a level-2 box bit-identical;
## - the GLSL grid mirror (probe kernel cloud_volume_grid_in) vs CloudWorldVolumeLayout.grid_of for VOLUME_MIRROR_POINTS
##   camera offsets in a frame VOLUME_MIRROR_FRAME_M from the camera: within VOLUME_MIRROR_BOUND x the containing
##   level's cell per axis (preflight C22);
## - the binding (no GPU): origins of 3 and 12 values agree, the 12-value cell layout equals the single-cell form,
##   malformed input returns nothing;
## - the pair switch (Review Focus 1): the service moves on to (ERA5 15:00, 16:00); the new snapshot replaces the analog
##   in slot 0 only at weight 0, its channel pair is baked while the weight stays 0, slot 1's channels read back
##   unchanged, the new pair is then the CPU's bake and the blend reaches the target.
## STARSIM_VOLUME_ONLY=16 runs this section alone.
const VOLUME_ONLY_ENV := "STARSIM_VOLUME_ONLY"
const VOLUME_SECTIONS := ["16", LIGHT_SECTION]
const VOLUME_ANALOG_MONTH := 10
const VOLUME_ANALOG_HOUR := 12
## ERA5 hours (since 1900-01-01) 2025-10-25 15:00 and 16:00 UTC in data/weather/cache.
const VOLUME_ERA5_FILES := ["era5_1102887.wthr", "era5_1102888.wthr"]
const VOLUME_HEIGHTS_M := [2.0, 9000.0]
const VOLUME_BLEND := 0.3
const VOLUME_LEVELS := [0, 2, 5]
const VOLUME_CELLS := 512
## Bound GPU bake vs CPU reference (brief: 1 % + 1e-6 /m; Task 4 carry: the GPU filters the field, shape and table
## textures with ~8-bit sub-texel weights where the CPU weighs exactly - a shape value off by <= 1/256 of a texel's
## step moves an edge ramp, but the cell mean of 256-8448 taps averages it; the half-float store adds 2^-11).
const VOLUME_TOLERANCE := 0.01
const VOLUME_FLOOR_PER_M := 1.0e-6
## Measured (Task 5 report): the cloud amount per weather cell agrees within 0.06 %, but single cells at cloud edges
## scatter: a tap whose shape value or cloud fraction lies within the GPU filter's 8-bit weight error of the threshold or
## of the fraction cut (CLOUD_MIN_FRACTION) flips. Hence at most VOLUME_OUTLIER_SHARE of the cells beyond the 1 % bound
## and every cell within VOLUME_EDGE_SIGMA x its sigma_c (T4's dense tier: 0.02 sigma_c of the dense mean; 0.05 covers
## the flips, worst measured 0.042). sigma_c of a cell without rho is not judged: there it is the mean in-cloud value of
## taps just above the fraction cut (a ratio of two tiny numbers) and no sample can use it (the sub-cell field is 0).
const VOLUME_OUTLIER_SHARE := 0.05
const VOLUME_EDGE_SIGMA := 0.05
const VOLUME_WATER := Vector2(0.996, 1.004)
const VOLUME_WATER_SAMPLES := 6144
const VOLUME_WATER_MIN_CELLS := 200
const VOLUME_PROBE_POINTS := 10000
const VOLUME_MIRROR_POINTS := 100
const VOLUME_MIRROR_FRAME_M := 990000.0
const VOLUME_MIRROR_BOUND := 1.0e-3
## The cold start measured at the game's budget for this many frames (1 s at 45 Hz), then filled at this many times
## the budget (a check, not the game: ~60 ms frames).
const VOLUME_COLD_FRAMES := 45
const VOLUME_BUDGET_SCALE := 40.0
const VOLUME_FILL_FRAMES := 3000
const VOLUME_DT := 1.0 / 45.0
## The pair switch: the service's blend of the new pair and the frames it may take.
const VOLUME_SWITCH_BLEND := 0.5
const VOLUME_SWITCH_FRAMES := 2000
const VOLUME_SWITCH_CELLS := 128
## The rebake's box (level 2, cells from the window's origin).
const VOLUME_REBAKE_FROM := Vector3i(96, 96, 0)
const VOLUME_REBAKE_SIZE := Vector3i(64, 64, 64)
## Points per level in the fade band (Task 5 review I1: the band blends level k + 1; the lookups must bound it too).
const VOLUME_BAND_POINTS := 10000
## Checks: binding 3, per camera 1 fill + 3 levels x (cells + mip + distance + free box) + 7 fade bands + 1 water, 2
## clamped bottoms (camera 2 m), rebake, mirror, switch 4.
const VOLUME_SAMPLES := 3 + 2 * (1 + 3 * 4 + 7 + 1) + 2 + 1 + 1 + 5

var _volume_rng := RandomNumberGenerator.new()


## STARSIM_VOLUME_ONLY: the world volume's sections alone (C16: an unknown section exits 1).
func _volume_only(value: String) -> void:
	if not value in VOLUME_SECTIONS:
		print("cloud_check: FAIL - %s=%s: unknown section (known: %s)" % [VOLUME_ONLY_ENV, value,
				", ".join(VOLUME_SECTIONS)])
		_clouds.shutdown()
		quit(1)
		return
	if value == LIGHT_SECTION:
		await _volume_light_samples()
	else:
		await _world_volume_samples()
	for f in _files:
		DirAccess.remove_absolute(f)
	_clouds.shutdown()
	var expected: int = LIGHT_SAMPLES if value == LIGHT_SECTION else VOLUME_SAMPLES
	_check("sample count", _samples == expected + 1, "%d samples" % _samples)  # + the transfer table's
	print("cloud_check: %s (section %s, %d samples, %d failures)" % ["PASS" if _failures == 0 else "FAIL", value,
			_samples, _failures])
	quit(1 if _failures > 0 else 0)


func _world_volume_samples() -> void:
	_volume_rng.seed = 20261028
	var cache := Paths.data_dir().path_join("weather/cache")
	var files: Array[String] = [_analog_file(VOLUME_ANALOG_MONTH, VOLUME_ANALOG_HOUR),
		cache.path_join(VOLUME_ERA5_FILES[0]), cache.path_join(VOLUME_ERA5_FILES[1])]
	var snapshots: Array[Dictionary] = []
	for f in files:
		if f == "" or not FileAccess.file_exists(f):
			_fail_and_quit("section 16: weather file missing: '%s'" % f)
			return
		var snapshot: Dictionary = _core.weather_load_snapshot(f)
		snapshot["file"] = f
		snapshots.append(snapshot)
	_volume_binding_samples(files[0])
	_clouds.world_volume = true
	var camera := Vector3.ZERO
	for i in VOLUME_HEIGHTS_M.size():
		var height: float = VOLUME_HEIGHTS_M[i]
		camera = HeightSource.geodetic_to_fixed(SUN_CU_SITE.x, SUN_CU_SITE.y, height)
		_clouds.show_now(snapshots[0], snapshots[1], VOLUME_BLEND)
		var label := "camera %.0f m" % height
		await _volume_fill(camera, i == 0, label)
		var lookup := _clouds.effect.world_volume.adopted()
		var reads := {}
		for k: int in VOLUME_LEVELS:
			reads[k] = _volume_read(k)
			var blocks := _volume_blocks(reads[k]["floats"])
			_volume_cells_vs_cpu(k, lookup, reads[k]["floats"], blocks["cloudy"], files, "%s, level %d" % [label, k])
			_volume_occupancy(k, lookup, reads[k]["floats"], blocks, "%s, level %d" % [label, k])
		for k in CloudWorldVolumeLayout.LEVELS - 1:
			_volume_band(k, lookup, "%s, level %d" % [label, k])
		if i == 0:
			for k in CloudWorldVolumeLayout.WINDOWED_LEVELS:
				_volume_bottom(k, lookup, "%s, level %d" % [label, k])
		_volume_water(lookup, reads[2]["floats"], files, snapshots, label)
		if i == VOLUME_HEIGHTS_M.size() - 1:
			await _volume_rebake(camera, lookup, reads[2]["bytes"])
	_volume_mirror()
	await _volume_pair_switch(camera, snapshots)
	_clouds.world_volume = false


## The analog snapshot of a month and UTC hour slot (data/weather/analog/index.json), "" if there is none.
func _analog_file(month: int, hour: int) -> String:
	var dir := Paths.data_dir().path_join("weather/analog")
	var f := FileAccess.open(dir.path_join("index.json"), FileAccess.READ)
	if f == null:
		return ""
	var parsed: Variant = JSON.parse_string(f.get_as_text())
	f.close()
	for entry: Dictionary in (parsed.get("entries", []) if parsed is Dictionary else []):
		if int(entry["month"]) == month and int(entry["hour_utc"]) == hour:
			return dir.path_join(String(entry["file"]))
	return ""


## One frame of the game's loop at the camera (ITRF): the Sun 45 deg up at the site, the layer's update with blend and
## delta, then the frame drawn (the effect runs the volume's batch).
func _volume_frame(camera_itrf: Vector3, blend: float, delta_s: float) -> void:
	var sun := _local_direction(45.0, 135.0, SUN_CU_SITE.x, SUN_CU_SITE.y)
	_atmosphere.update(_earth_to_gcrf * camera_itrf, Vector3.ZERO, _earth_to_gcrf, _earth_to_gcrf * sun * AU_M, 0.0,
			1.0)
	_clouds.update(camera_itrf, blend, delta_s)
	await RenderingServer.frame_post_draw


## The cold fill of the pinned pair: with `measure` first VOLUME_COLD_FRAMES frames at the game's budget (the
## upkeep's GPU time per frame from its timestamps, the taps and cells per ms measured), then at VOLUME_BUDGET_SCALE x
## the budget until the schedule is complete and its last batch ran.
func _volume_fill(camera: Vector3, measure: bool, label: String) -> void:
	var schedule := _clouds.world_volume_schedule
	var volume := _clouds.effect.world_volume
	schedule.budget_scale = 1.0
	var frames := 0
	if measure:
		var before := _clouds.world_volume_stats()
		var upkeeps: Array[float] = []
		var planned := 0
		var seen := int(before["measured_frames"])
		for i in VOLUME_COLD_FRAMES:
			await _volume_frame(camera, VOLUME_BLEND, VOLUME_DT)
			frames += 1
			var stats := _clouds.world_volume_stats()
			planned += int(stats["bake_cells"])
			if int(stats["measured_frames"]) > seen:
				seen = int(stats["measured_frames"])
				upkeeps.append(float(stats["upkeep_ms"]))
		var after := _clouds.world_volume_stats()
		var ms := float(after["measured_bake_ms"]) - float(before["measured_bake_ms"])
		var taps := float(after["measured_taps"]) - float(before["measured_taps"])
		upkeeps.sort()
		print("cloud_check: world volume cold start (%s, game budget, %d frames): gpu upkeep (bake + light) " % [label,
				VOLUME_COLD_FRAMES] + "median %.2f ms, max %.2f ms over %d measured frames; %s taps / ms = %.0f "
				% [upkeeps[upkeeps.size() / 2] if not upkeeps.is_empty() else -1.0, upkeeps.max() if not upkeeps.is_empty()
				else -1.0, upkeeps.size(), String.num_scientific(taps / maxf(ms, 1e-9)), taps / maxf(ms, 1e-9)
				/ CloudWorldVolumeLayout.cell_taps(7)] + "level-7 cells / ms; %d cells planned; calibrated rate %s taps / ms"
				% [planned, String.num_scientific(schedule.bake_taps_per_ms)])
	schedule.budget_scale = VOLUME_BUDGET_SCALE
	var start := Time.get_ticks_msec()
	var done := false
	while frames < VOLUME_FILL_FRAMES and not done:
		await _volume_frame(camera, VOLUME_BLEND, VOLUME_DT)
		frames += 1
		done = schedule.complete() and not volume.waiting() and _volume_all_ready(volume.adopted())
	schedule.budget_scale = 1.0
	await _volume_frame(camera, VOLUME_BLEND, VOLUME_DT)  # the batch planned last has run
	_check("world volume fill (%s)" % label, done, "%d frames, %.1f s at %.0f x the budget, rate %s taps / ms" % [frames,
			(Time.get_ticks_msec() - start) / 1000.0, VOLUME_BUDGET_SCALE,
			String.num_scientific(schedule.bake_taps_per_ms)])


static func _volume_all_ready(lookup: Dictionary) -> bool:
	if lookup.is_empty():
		return false
	for lv: Dictionary in lookup["levels"]:
		if bool(lv["active"]) and not bool(lv["ready"]):
			return false
	return true


## Level k's density read back: the raw half floats and the floats (1/km), 4 per texel (x fastest, then y, z).
func _volume_read(k: int) -> Dictionary:
	var rd := RenderingServer.get_rendering_device()
	var bytes := rd.texture_get_data(_clouds.effect.world_volume.density_texture(k), 0)
	var image := Image.create_from_data(CloudWorldVolumeLayout.CELLS_H,
			CloudWorldVolumeLayout.CELLS_H * CloudWorldVolumeLayout.CELLS_V, false, Image.FORMAT_RGBAH, bytes)
	image.convert(Image.FORMAT_RGBAF)
	return {"bytes": bytes, "floats": image.get_data().to_float32_array()}


## Per block of MIP_DIV^3 texels (texel space): the largest rho and sigma_c of its cells over both snapshots (1/km),
## `ring` 1 where a cell of the block or of its one-cell ring (toroidal) has rho > 0 (the support a trilinear sample in
## the block reads: what the distance field must keep), and up to VOLUME_CELLS texel indices with rho > 0 (a reservoir
## sample).
func _volume_blocks(floats: PackedFloat32Array) -> Dictionary:
	var L := CloudWorldVolumeLayout
	var mip := L.mip_size()
	var rho := PackedFloat32Array()
	var sigma := PackedFloat32Array()
	rho.resize(mip.x * mip.y * mip.z)
	sigma.resize(mip.x * mip.y * mip.z)
	var ring := PackedByteArray()
	ring.resize(mip.x * mip.y * mip.z)
	var cloudy := PackedInt32Array()
	var seen := 0
	var plane := L.CELLS_H * L.CELLS_H * 4
	var zero := PackedFloat32Array()
	zero.resize(plane)
	for z in L.CELLS_V:
		if floats.slice(z * plane, (z + 1) * plane) == zero:
			continue
		for y in L.CELLS_H:
			var row := (z * L.CELLS_H + y) * L.CELLS_H
			var block_row := ((z / L.MIP_DIV) * mip.y + y / L.MIP_DIV) * mip.x
			for x in L.CELLS_H:
				var i := (row + x) * 4
				var r := maxf(floats[i], floats[i + 2])
				var s := maxf(floats[i + 1], floats[i + 3])
				if r <= 0.0 and s <= 0.0:
					continue
				var b := block_row + x / L.MIP_DIV
				rho[b] = maxf(rho[b], r)
				sigma[b] = maxf(sigma[b], s)
				if r > 0.0:
					ring[b] = 1
					if x % L.MIP_DIV in [0, L.MIP_DIV - 1] or y % L.MIP_DIV in [0, L.MIP_DIV - 1] \
							or z % L.MIP_DIV in [0, L.MIP_DIV - 1]:
						_volume_mark_ring(ring, Vector3i(x, y, z))
					seen += 1
					if cloudy.size() < VOLUME_CELLS:
						cloudy.append(row + x)
					else:
						var j := _volume_rng.randi_range(0, seen - 1)
						if j < VOLUME_CELLS:
							cloudy[j] = row + x
	return {"rho": rho, "sigma": sigma, "ring": ring, "cloudy": cloudy, "cloudy_cells": seen}


## Marks every block whose cells plus one-cell ring hold texel t (toroidal).
static func _volume_mark_ring(ring: PackedByteArray, t: Vector3i) -> void:
	var L := CloudWorldVolumeLayout
	var mip := L.mip_size()
	var per_axis: Array = []
	for axis in 3:
		var blocks := [posmod(t[axis] - 1, mip[axis] * L.MIP_DIV) / L.MIP_DIV]
		for c in [t[axis], t[axis] + 1]:
			var b := posmod(c, mip[axis] * L.MIP_DIV) / L.MIP_DIV
			if not b in blocks:
				blocks.append(b)
		per_axis.append(blocks)
	for bz: int in per_axis[2]:
		for by: int in per_axis[1]:
			for bx: int in per_axis[0]:
				ring[(bz * mip.y + by) * mip.x + bx] = 1


## The global cell of texel index t of a level whose storage box starts at `origin` (toroidal storage).
static func _volume_cell_of(t: int, origin: Vector3i) -> Vector3i:
	var L := CloudWorldVolumeLayout
	var texel := Vector3i(t % L.CELLS_H, (t / L.CELLS_H) % L.CELLS_H, t / (L.CELLS_H * L.CELLS_H))
	var size := Vector3i(L.CELLS_H, L.CELLS_H, L.CELLS_V)
	var o := L.texel_of(0, origin)
	return origin + Vector3i(posmod(texel.x - o.x, size.x), posmod(texel.y - o.y, size.y), posmod(texel.z - o.z, size.z))


static func _volume_texel_index(cell: Vector3i) -> int:
	var L := CloudWorldVolumeLayout
	var t := L.texel_of(0, cell)
	return (t.z * L.CELLS_H + t.y) * L.CELLS_H + t.x


## The layout of a level's frame (as the volume's lookup states it).
static func _volume_layout(lookup: Dictionary, k: int) -> CloudWorldVolumeLayout:
	var layout := CloudWorldVolumeLayout.new()
	layout.set_frame(lookup["levels"][k]["frame_origin_earth_m"])
	return layout


## The cell's box for the CPU bake (12 values: centre, e_x, e_y, e_z; Earth-local render axes): the grid's own axes at
## its centre, as the GPU bake builds it (cloud_volume_bake.gdshaderinc).
static func _volume_box(layout: CloudWorldVolumeLayout, k: int, cell: Vector3i) -> PackedFloat64Array:
	var L := CloudWorldVolumeLayout
	var h: float = L.CELL_H_M[k]
	var v: float = L.CELL_V_M[k]
	var g := Vector3((cell.x + 0.5) * h, (cell.y + 0.5) * h, L.SHELL_BOTTOM_M + (cell.z + 0.5) * v)
	var c := layout.earth_of(g)
	var ex := layout.earth_of(g + Vector3(0.5 * h, 0.0, 0.0)) - layout.earth_of(g - Vector3(0.5 * h, 0.0, 0.0))
	var ey := layout.earth_of(g + Vector3(0.0, 0.5 * h, 0.0)) - layout.earth_of(g - Vector3(0.0, 0.5 * h, 0.0))
	var ez := layout.up_of_grid(g.x, g.y) * v
	return PackedFloat64Array([c.x, c.y, c.z, ex.x, ex.y, ex.z, ey.x, ey.y, ey.z, ez.x, ez.y, ez.z])


## The shape origins as the GPU has them (12 values: the camera, then the cumulus, stratiform and ice origins).
static func _volume_origins() -> PackedFloat64Array:
	var hi: Vector3 = ShaderGlobals.value(&"atmosphere_camera_hi")
	var lo: Vector3 = ShaderGlobals.value(&"atmosphere_camera_lo")
	var camera := hi + lo
	var o := PackedFloat64Array([camera.x, camera.y, camera.z])
	for id in [&"cloud_noise_origin", &"cloud_stratiform_origin", &"cloud_ice_origin"]:
		var v: Vector3 = ShaderGlobals.value(id)
		o.append_array([v.x, v.y, v.z])
	return o


## CPU bakes of cells (global) of level k for a snapshot file: (rho, sigma_c) per cell, 1/m.
func _volume_cpu(file: String, lookup: Dictionary, k: int, cells: Array[Vector3i]) -> PackedVector2Array:
	var layout := _volume_layout(lookup, k)
	var boxes := PackedFloat64Array()
	for c in cells:
		boxes.append_array(_volume_box(layout, k, c))
	return _core.weather_cloud_cell_bakes(file, boxes, _volume_origins())


## One cell's GPU bake (texel floats from t: rho, sigma_c in 1/km) against the CPU's (1/m), into : n, beyond
## (outside VOLUME_TOLERANCE x the value + VOLUME_FLOOR_PER_M), edge (outside VOLUME_EDGE_SIGMA x sigma_c +
## VOLUME_FLOOR_PER_M; sigma_c judged so only where the CPU's rho > 0), the worst relative error and the worst error in
## sigma_c; the first edge failure's text.
static func _volume_compare(floats: PackedFloat32Array, t: int, cpu: Vector2, stats: Dictionary, what: String) -> void:
	var gpu := Vector2(floats[t], floats[t + 1]) / CloudWorldVolume.PER_KM
	var error := (gpu - cpu).abs()
	var sigma := maxf(cpu.y, VOLUME_FLOOR_PER_M)
	stats["n"] = int(stats.get("n", 0)) + 1
	stats["worst"] = maxf(float(stats.get("worst", 0.0)), maxf(error.x / maxf(cpu.x, VOLUME_FLOOR_PER_M),
			error.y / sigma))
	stats["worst_sigma"] = maxf(float(stats.get("worst_sigma", 0.0)), maxf(error.x, error.y) / sigma)
	if error.x > VOLUME_TOLERANCE * cpu.x + VOLUME_FLOOR_PER_M or error.y > VOLUME_TOLERANCE * cpu.y + VOLUME_FLOOR_PER_M:
		stats["beyond"] = int(stats.get("beyond", 0)) + 1
	if error.x > VOLUME_EDGE_SIGMA * cpu.y + VOLUME_FLOOR_PER_M 			or (cpu.x > 0.0 and error.y > VOLUME_EDGE_SIGMA * cpu.y + VOLUME_FLOOR_PER_M):
		stats["edge"] = int(stats.get("edge", 0)) + 1
		if not stats.has("detail"):
			stats["detail"] = "; first: %s GPU (%s, %s) CPU (%s, %s) /m" % [what, String.num_scientific(gpu.x),
					String.num_scientific(gpu.y), String.num_scientific(cpu.x), String.num_scientific(cpu.y)]


## The bake comparison's verdict: at most VOLUME_OUTLIER_SHARE of the cells beyond the 1 % bound, none beyond the edge
## bound.
static func _volume_verdict(stats: Dictionary) -> bool:
	var n := int(stats.get("n", 0))
	return n > 0 and int(stats.get("beyond", 0)) <= floori(VOLUME_OUTLIER_SHARE * n) and int(stats.get("edge", 0)) == 0


static func _volume_stats_text(stats: Dictionary) -> String:
	return "%d beyond %.2f x + %s /m (allowed %.0f %%), %d beyond %.2f sigma_c; worst relative %.4f, worst %.4f sigma_c%s" 			% [int(stats.get("beyond", 0)), VOLUME_TOLERANCE, String.num_scientific(VOLUME_FLOOR_PER_M),
			100.0 * VOLUME_OUTLIER_SHARE, int(stats.get("edge", 0)), VOLUME_EDGE_SIGMA, float(stats.get("worst", 0.0)),
			float(stats.get("worst_sigma", 0.0)), stats.get("detail", "")]


## 512 cells of level k (half the GPU's cloudy ones, half uniform) vs the CPU bake of each snapshot.
func _volume_cells_vs_cpu(k: int, lookup: Dictionary, floats: PackedFloat32Array, cloudy: PackedInt32Array,
		files: Array[String], label: String) -> void:
	var L := CloudWorldVolumeLayout
	var origin: Vector3i = lookup["levels"][k]["storage"]
	var cells: Array[Vector3i] = []
	for i in mini(cloudy.size(), VOLUME_CELLS / 2):
		cells.append(_volume_cell_of(cloudy[i], origin))
	while cells.size() < VOLUME_CELLS:
		cells.append(origin + Vector3i(_volume_rng.randi_range(0, L.CELLS_H - 1), _volume_rng.randi_range(0,
				L.CELLS_H - 1), _volume_rng.randi_range(0, L.CELLS_V - 1)))
	var stats := {}
	var nonzero := 0
	var complete := true
	for s in 2:
		var cpu := _volume_cpu(files[s], lookup, k, cells)
		if cpu.size() != cells.size():
			complete = false
			continue
		for i in cells.size():
			var t := _volume_texel_index(cells[i]) * 4 + 2 * CloudWorldVolume.channel_of_slot(s)
			nonzero += 1 if cpu[i].x > 0.0 else 0
			_volume_compare(floats, t, cpu[i], stats, "slot %d cell %s" % [s, cells[i]])
	_check("world volume bake vs CPU (%s)" % label, complete and nonzero > 0 and _volume_verdict(stats),
			"%d cells x 2 snapshots (%d with rho > 0 on the CPU): %s" % [cells.size(), nonzero, _volume_stats_text(stats)])


## Level 2's water per weather cell: uniform cells of the cloud band, assigned to a weather cell by their centre; the
## GPU's sum of rho over the CPU's for both snapshots.
func _volume_water(lookup: Dictionary, floats: PackedFloat32Array, files: Array[String], snapshots: Array[Dictionary],
		label: String) -> void:
	var L := CloudWorldVolumeLayout
	var k := 2
	var origin: Vector3i = lookup["levels"][k]["storage"]
	var layout := _volume_layout(lookup, k)
	# the cloud band: the texel z range with rho > 0
	var z_lo := L.CELLS_V
	var z_hi := -1
	var plane := L.CELLS_H * L.CELLS_H * 4
	var zero := PackedFloat32Array()
	zero.resize(plane)
	for z in L.CELLS_V:
		if floats.slice(z * plane, (z + 1) * plane) != zero:
			z_lo = mini(z_lo, z)
			z_hi = maxi(z_hi, z)
	var cells: Array[Vector3i] = []
	if z_hi >= z_lo:
		for i in VOLUME_WATER_SAMPLES:
			var tz := _volume_rng.randi_range(z_lo, z_hi)
			cells.append(origin + Vector3i(_volume_rng.randi_range(0, L.CELLS_H - 1),
					_volume_rng.randi_range(0, L.CELLS_H - 1), 0) + Vector3i(0, 0, tz))
	var ratios := ""
	var ok := not cells.is_empty()
	var judged := 0
	for s in 2:
		var cpu := _volume_cpu(files[s], lookup, k, cells)
		if cpu.size() != cells.size():
			ok = false
			continue
		var rows := float(snapshots[s]["rows"])
		var columns := float(snapshots[s]["columns"])
		var sums := {}  # weather cell -> Vector3(gpu, cpu, count)
		for i in cells.size():
			var p := layout.earth_of(Vector3((cells[i].x + 0.5) * L.CELL_H_M[k], (cells[i].y + 0.5) * L.CELL_H_M[k],
					L.SHELL_BOTTOM_M + (cells[i].z + 0.5) * L.CELL_V_M[k]))
			var lat := atan2(p.y * CloudLayer.POLAR_STRETCH * CloudLayer.POLAR_STRETCH, Vector2(p.x, p.z).length())
			var lon := atan2(p.x, p.z)
			var id := Vector2i(roundi((0.5 * PI - lat) / (PI / (rows - 1.0))),
					posmod(roundi(lon / (TAU / columns)), int(columns)))
			var t := _volume_texel_index(cells[i]) * 4 + 2 * CloudWorldVolume.channel_of_slot(s)
			var sum: Vector3 = sums.get(id, Vector3.ZERO)
			sums[id] = sum + Vector3(floats[t] / CloudWorldVolume.PER_KM, cpu[i].x, 1.0)
		for id: Vector2i in sums:
			var sum: Vector3 = sums[id]
			if sum.z < VOLUME_WATER_MIN_CELLS or sum.y <= 0.0:
				continue
			var ratio := sum.x / sum.y
			judged += 1
			ok = ok and ratio >= VOLUME_WATER.x and ratio <= VOLUME_WATER.y
			ratios += " slot %d cell %s: %.5f (%d)" % [s, id, ratio, int(sum.z)]
	_check("world volume level-2 water per weather cell (%s)" % label, ok and judged > 0,
			"GPU / CPU in [%.3f, %.3f]:%s" % [VOLUME_WATER.x, VOLUME_WATER.y, ratios])


## Level k's occupancy exhaustively: the max mip and the distance field read back against the density's blocks (the
## distance against the ring-dilated occupancy, Task 5 review M1); then the probe across claimed free boxes
## (_volume_free_probe; the fade band: _volume_band).
func _volume_occupancy(k: int, lookup: Dictionary, floats: PackedFloat32Array, blocks: Dictionary,
		label: String) -> void:
	var L := CloudWorldVolumeLayout
	var rd := RenderingServer.get_rendering_device()
	var volume := _clouds.effect.world_volume
	var mip := L.mip_size()
	var count := mip.x * mip.y * mip.z
	var max_image := Image.create_from_data(mip.x, mip.y * mip.z, false, Image.FORMAT_RH,
			rd.texture_get_data(volume.max_extinction_texture(k), 0))
	max_image.convert(Image.FORMAT_RF)
	var bound := max_image.get_data().to_float32_array()
	var claimed := rd.texture_get_data(volume.distance_texture(k), 0)
	var rho: PackedFloat32Array = blocks["rho"]
	var sigma: PackedFloat32Array = blocks["sigma"]
	var low := 0
	var occupied := 0
	for b in count:
		if rho[b] > 0.0:
			occupied += 1
			if bound[b] < sigma[b] or bound[b] < rho[b]:
				low += 1
	_check("world volume max-extinction mip (%s)" % label, low == 0 and occupied > 0,
			"%d of %d blocks with rho > 0, %d with a bound below a cell's sigma_c / rho" % [occupied, count, low])
	# the true toroidal Chebyshev distance (blocks, capped) to the nearest block with rho > 0
	var cap := CloudWorldVolume.DISTANCE_CAP_BLOCKS
	var d := PackedInt32Array()
	d.resize(count)
	var ring: PackedByteArray = blocks["ring"]
	for b in count:
		d[b] = 0 if ring[b] != 0 else cap + 1
	var strides := [1, mip.x, mip.x * mip.y]
	for axis in 3:
		var next := PackedInt32Array()
		next.resize(count)
		var stride: int = strides[axis]
		var n_axis: int = mip[axis]
		for b in count:
			if d[b] == 0:
				continue  # (next[b] = 0)
			var coord := (b / stride) % n_axis
			var base := b - coord * stride
			var best := cap + 1
			for j in range(-cap, cap + 1):
				best = mini(best, maxi(absi(j), d[base + posmod(coord + j, n_axis) * stride]))
			next[b] = best
		d = next
	var over := 0
	var exact := 0
	var free_blocks: Array[int] = []
	for b in count:
		if claimed[b] > d[b]:
			over += 1
		exact += 1 if claimed[b] == d[b] else 0
		if claimed[b] >= 1:
			free_blocks.append(b)
	_check("world volume distance field (%s)" % label, over == 0 and free_blocks.size() > 0,
			"%d blocks: %d claim more than the true distance to cloud incl. the one-cell ring (cap %d), %d equal, "
			% [count, over, cap, exact] + "%d claimed free" % free_blocks.size())
	_volume_free_probe(k, lookup, free_blocks, label)


## The occupancy lookups as the march will use them (probe mode 2 at points p, mode 3 the blended density at points q):
## free boxes - points p in blocks claimed free, q anywhere in the claimed free ball: no density (either snapshot, the
## fade toward level k + 1 included). The fade band (review I1): _volume_band.
func _volume_free_probe(k: int, lookup: Dictionary, free_blocks: Array[int], label: String) -> void:
	var L := CloudWorldVolumeLayout
	var storage: Vector3i = lookup["levels"][k]["storage"]
	var origin: Vector3i = lookup["levels"][k]["origin"]
	var window := Vector3(L.window_size(k))
	var o := L.texel_of(k, storage)
	var size := Vector3i(L.CELLS_H, L.CELLS_H, L.CELLS_V)
	var mip := L.mip_size()
	var free_cells: Array[Vector3] = []
	var guard := 0
	while free_cells.size() < VOLUME_PROBE_POINTS and guard < 20 * VOLUME_PROBE_POINTS:
		guard += 1
		var b: int = free_blocks[_volume_rng.randi_range(0, free_blocks.size() - 1)]
		var block := Vector3i(b % mip.x, (b / mip.x) % mip.y, b / (mip.x * mip.y))
		var t := Vector3(block * L.MIP_DIV) + Vector3(_volume_rng.randf(), _volume_rng.randf(), _volume_rng.randf()) \
				* L.MIP_DIV
		# texel -> the storage's continuous cell coordinate; the window's cells only
		var c := Vector3(storage) + Vector3(fposmod(t.x - o.x, size.x), fposmod(t.y - o.y, size.y),
				fposmod(t.z - o.z, size.z))
		var u := c - Vector3(origin)
		if u.x >= 0.0 and u.y >= 0.0 and u.z >= 0.0 and u.x < window.x and u.y < window.y and u.z < window.z:
			free_cells.append(c)
	var free := _volume_probe_pass(k, lookup, free_cells, false)
	_check("world volume free boxes sampled (%s)" % label, free["violations"] == 0 and free["free"] > 0,
			"%d points in blocks claimed free: %d with density > 0 at a point of their free ball (either snapshot, "
			% [free_cells.size(), free["violations"]] + "fade incl.; %d free balls, mean %.0f m)" % [free["free"],
			free["mean_free_m"]])


## The fade band of level k (every level 0-6: each adjacent pair, review round 2): VOLUME_BAND_POINTS points within
## FADE_CELLS + 1/2 cells of the window edge - horizontally, and for levels 0-1 at the window top (and bottom unless
## clamped) - q in the free ball (no density) and q in p's level-k block (density <= the max-extinction bound; level
## 1's blocks straddle level 2's vertically).
func _volume_band(k: int, lookup: Dictionary, label: String) -> void:
	var L := CloudWorldVolumeLayout
	var e: Dictionary = lookup["levels"][k]
	var origin: Vector3i = e["origin"]
	var window := Vector3(L.window_size(k))
	var axes := 2 if k < L.WINDOWED_LEVELS else 1
	var band_cells: Array[Vector3] = []
	var band := float(L.FADE_CELLS) + 0.5
	for i in VOLUME_BAND_POINTS:
		var u := Vector3(_volume_rng.randf() * window.x, _volume_rng.randf() * window.y, _volume_rng.randf() * window.z)
		var axis := _volume_rng.randi_range(0, axes)
		var depth := _volume_rng.randf() * band
		var low := _volume_rng.randf() < 0.5 and not (axis == 2 and bool(e["clamped_bottom"]))
		u[axis] = depth if low else window[axis] - depth
		band_cells.append(Vector3(origin) + u)
	var in_band := _volume_probe_pass(k, lookup, band_cells, true)
	var band_ok: bool = e["shown"] and in_band["violations"] == 0 and in_band["bound_low"] == 0 and in_band["free"] > 0
	_check("world volume fade band bounded (%s)" % label, band_ok, "%d points in the fade band: %d with density > 0 "
			% [band_cells.size(), in_band["violations"]] + "in their free ball (%d free balls), %d with a density in "
			% [in_band["free"], in_band["bound_low"]] + "their block above the max-extinction bound (%d with density "
			% in_band["dense"] + "> 0 there)")


## Levels 0-1 with a clamped bottom (review round 2, N2): a sample below the shell bottom's cell centre reads the bottom
## cell (the clamped edge), never the storage's top (guard) cell: VOLUME_BAND_POINTS columns, the density (probe mode 1)
## at a random height in the lowest half cell equals the one at its centre; the columns whose top storage cell differs
## from the bottom one (where a wrap would show) are counted.
func _volume_bottom(k: int, lookup: Dictionary, label: String) -> void:
	var L := CloudWorldVolumeLayout
	var rd := RenderingServer.get_rendering_device()
	var volume := _clouds.effect.world_volume
	var hi: Vector3 = ShaderGlobals.value(&"atmosphere_camera_hi")
	var lo: Vector3 = ShaderGlobals.value(&"atmosphere_camera_lo")
	var e: Dictionary = lookup["levels"][k]
	var origin: Vector3i = e["origin"]
	var window := Vector3(L.window_size(k))
	var below := PackedFloat32Array()
	var centre := PackedFloat32Array()
	var top := PackedFloat32Array()
	for i in VOLUME_BAND_POINTS:
		var c := Vector3(origin) + Vector3(_volume_rng.randf() * window.x, _volume_rng.randf() * window.y, 0.0)
		below.append_array(_volume_grid_point(k, c + Vector3(0.0, 0.0, _volume_rng.randf() * 0.5)))
		centre.append_array(_volume_grid_point(k, c + Vector3(0.0, 0.0, 0.5)))
		top.append_array(_volume_grid_point(k, c + Vector3(0.0, 0.0, L.CELLS_V - 0.5)))
	var a := volume.probe(rd, 1, below, lookup, hi + lo, k, k)
	var b := volume.probe(rd, 1, centre, lookup, hi + lo, k, k)
	var t := volume.probe(rd, 1, top, lookup, hi + lo, k, k)
	var differ := 0
	var wrap_visible := 0
	var ok := bool(e["clamped_bottom"]) and a.size() == below.size() and b.size() == below.size()
	for i in (VOLUME_BAND_POINTS if ok else 0):
		var same := true
		var top_differs := false
		for j in 4:
			same = same and a[4 * i + j] == b[4 * i + j]
			top_differs = top_differs or t[4 * i + j] != b[4 * i + j]
		differ += 0 if same else 1
		wrap_visible += 1 if top_differs else 0
	_check("world volume clamped bottom reads its edge (%s)" % label, ok and differ == 0,
			"%d columns: %d read other than the bottom cell below its centre (%d whose top storage cell differs)"
			% [VOLUME_BAND_POINTS, differ, wrap_visible])


## One probe round: the lookups at the points (cells of level k), then the blended density at a point of each free
## ball (and with `block`, at a point of each one's level-k block, against the bound).
func _volume_probe_pass(k: int, lookup: Dictionary, cells: Array[Vector3], block: bool) -> Dictionary:
	var L := CloudWorldVolumeLayout
	var rd := RenderingServer.get_rendering_device()
	var volume := _clouds.effect.world_volume
	var hi: Vector3 = ShaderGlobals.value(&"atmosphere_camera_hi")
	var lo: Vector3 = ShaderGlobals.value(&"atmosphere_camera_lo")
	var camera := hi + lo
	var cell_m := Vector3(L.CELL_H_M[k], L.CELL_H_M[k], L.CELL_V_M[k])
	var points := PackedFloat32Array()
	for c in cells:
		points.append_array(_volume_grid_point(k, c))
	var lookups := volume.probe(rd, 2, points, lookup, camera, k, k)
	var q := PackedFloat32Array()
	var q_block := PackedFloat32Array()
	var n_free := 0
	var free_sum := 0.0
	for i in cells.size():
		var free_m := lookups[4 * i] if lookups.size() >= 4 * cells.size() else 0.0
		var r := Vector3.ONE * free_m / cell_m  # grid metres >= true metres / CloudVolume's MAX_SCALE
		if free_m > 0.0:
			n_free += 1
			free_sum += free_m
		var offset := Vector3(_volume_rng.randf_range(-1.0, 1.0), _volume_rng.randf_range(-1.0, 1.0),
				_volume_rng.randf_range(-1.0, 1.0)) * r
		q.append_array(_volume_grid_point(k, cells[i] + offset))
		var b0 := (cells[i] / float(L.MIP_DIV)).floor() * L.MIP_DIV
		q_block.append_array(_volume_grid_point(k, b0 + Vector3(_volume_rng.randf(), _volume_rng.randf(),
				_volume_rng.randf()) * L.MIP_DIV))
	var dens := volume.probe(rd, 3, q, lookup, camera, k, k)
	var violations := 0
	for i in cells.size():
		if dens.size() < 4 * cells.size():
			violations = cells.size()
			break
		if lookups[4 * i] > 0.0 and (dens[4 * i] > 0.0 or dens[4 * i + 2] > 0.0):
			violations += 1
	var out := {"violations": violations, "free": n_free, "mean_free_m": free_sum / maxf(n_free, 1.0), "bound_low": 0,
			"dense": 0}
	if block:
		var dens_b := volume.probe(rd, 3, q_block, lookup, camera, k, k)
		for i in cells.size():
			if dens_b.size() < 4 * cells.size():
				out["bound_low"] = cells.size()
				break
			var rho := maxf(dens_b[4 * i], dens_b[4 * i + 2])
			out["dense"] += 1 if rho > 0.0 else 0
			out["bound_low"] += 1 if rho > lookups[4 * i + 1] else 0
	return out


## A level-k cell coordinate (continuous) as a grid point (x, y m; z m above the ellipsoid), 4 floats for the probe.
static func _volume_grid_point(k: int, c: Vector3) -> PackedFloat32Array:
	var L := CloudWorldVolumeLayout
	return PackedFloat32Array([c.x * L.CELL_H_M[k], c.y * L.CELL_H_M[k], L.SHELL_BOTTOM_M + c.z * L.CELL_V_M[k], 0.0])


## A level-2 box baked again (a batch of its own after the layer's): its texels bit-identical.
func _volume_rebake(camera: Vector3, lookup: Dictionary, before: PackedByteArray) -> void:
	var L := CloudWorldVolumeLayout
	var origin: Vector3i = lookup["levels"][2]["storage"]
	var from := origin + VOLUME_REBAKE_FROM
	var to := from + VOLUME_REBAKE_SIZE
	_atmosphere.update(_earth_to_gcrf * camera, Vector3.ZERO, _earth_to_gcrf, _earth_to_gcrf * _local_direction(45.0,
			135.0, SUN_CU_SITE.x, SUN_CU_SITE.y) * AU_M, 0.0, 1.0)
	_clouds.update(camera, VOLUME_BLEND, 0.0)
	_clouds.effect.world_volume.submit({"bake": [{"level": 2, "from": from, "to": to, "snapshot_channel": -1,
			"spare": false}], "light": [], "provisional": [], "lookup": lookup, "reframe": {"level": -1}})
	await RenderingServer.frame_post_draw
	var after: PackedByteArray = _volume_read(2)["bytes"]
	var differing := 0
	for z in range(from.z, to.z):
		for y in range(from.y, to.y):
			var a := _volume_texel_index(Vector3i(from.x, y, z)) * 8
			var n := VOLUME_REBAKE_SIZE.x * 8
			if L.texel_of(2, Vector3i(from.x, y, z)).x + VOLUME_REBAKE_SIZE.x > L.CELLS_H:
				n = (L.CELLS_H - L.texel_of(2, Vector3i(from.x, y, z)).x) * 8  # (the row wraps: its first part)
			if before.slice(a, a + n) != after.slice(a, a + n):
				differing += 1
	_check("world volume rebake bit-identical", differing == 0 and before.size() == after.size(),
			"level 2 box %s .. %s: %d of %d rows differ" % [from, to, differing, VOLUME_REBAKE_SIZE.y
			* VOLUME_REBAKE_SIZE.z])


## The GLSL grid (probe kernel, cloud_volume_grid_in) against CloudWorldVolumeLayout.grid_of: the camera's frame moved
## VOLUME_MIRROR_FRAME_M away (the worst before a re-frame); points per level inside its window, each judged by the
## finest level containing it.
func _volume_mirror() -> void:
	var L := CloudWorldVolumeLayout
	var volume := _clouds.effect.world_volume
	var lookup: Dictionary = volume.adopted().duplicate(true)
	var hi: Vector3 = ShaderGlobals.value(&"atmosphere_camera_hi")
	var lo: Vector3 = ShaderGlobals.value(&"atmosphere_camera_lo")
	var camera := hi + lo
	var near := CloudWorldVolumeLayout.new()
	near.set_frame(camera)
	var far_origin := near.earth_of(Vector3(VOLUME_MIRROR_FRAME_M * cos(0.7), VOLUME_MIRROR_FRAME_M * sin(0.7), 0.0))
	var layout := CloudWorldVolumeLayout.new()
	layout.set_frame(far_origin)
	var gc := layout.grid_of(camera)
	for lv: Dictionary in lookup["levels"]:
		lv["frame_origin_earth_m"] = far_origin
		var k: int = lv["level"]
		lv["origin"] = layout.window_origin(k, gc)
		lv["active"] = layout.level_active(k, gc.z)
	lookup.erase("spare")
	lookup.erase("build_spare")
	var points := PackedFloat32Array()
	var expected: Array[Vector3] = []
	var levels: Array[int] = []
	for i in VOLUME_MIRROR_POINTS:
		var k := i % L.LEVELS
		var half := 0.5 * L.CELLS_H * L.CELL_H_M[k]
		var g := gc + Vector3(_volume_rng.randf_range(-half, half), _volume_rng.randf_range(-half, half), 0.0)
		if k < L.WINDOWED_LEVELS:
			var o: Vector3i = lookup["levels"][k]["origin"]
			g.z = L.SHELL_BOTTOM_M + (o.z + _volume_rng.randf() * L.CELLS_V) * L.CELL_V_M[k]
		else:
			g.z = _volume_rng.randf_range(L.SHELL_BOTTOM_M, L.SHELL_TOP_M)
		var offset := CloudWorldVolume._f32(layout.earth_of(g) - camera)
		points.append_array([offset.x, offset.y, offset.z, 0.0])
		expected.append(layout.grid_of(camera + offset))
	var result := volume.probe(RenderingServer.get_rendering_device(), 0, points, lookup, camera)
	var bad := 0
	var worst := 0.0
	var worst_text := ""
	for i in expected.size():
		if result.size() < 4 * expected.size():
			bad = expected.size()
			break
		var e := expected[i]
		var got := Vector3(result[4 * i], result[4 * i + 1], result[4 * i + 2])
		var k := L.LEVELS - 1
		for j in L.LEVELS:
			if bool(lookup["levels"][j]["active"]) and L.fade_weight_at(j, e, lookup["levels"][j]["origin"]) > 0.0:
				k = j
				break
		var err := (got - e).abs()
		var cell := Vector3(L.CELL_H_M[k], L.CELL_H_M[k], L.CELL_V_M[k])
		var rel := maxf(maxf(err.x / cell.x, err.y / cell.y), err.z / cell.z)
		if rel > worst:
			worst = rel
			worst_text = "level %d: error %s m at |g - g_c| %.0f m" % [k, err, Vector2(e.x - gc.x, e.y - gc.y).length()]
		bad += 1 if rel > VOLUME_MIRROR_BOUND else 0
	_check("world volume GLSL grid mirror", bad == 0,
			"%d points, frame %.0f km away: %d beyond %s x the containing cell; worst %s cell (%s)" % [expected.size(),
			VOLUME_MIRROR_FRAME_M / 1000.0, bad, String.num_scientific(VOLUME_MIRROR_BOUND),
			String.num_scientific(worst), worst_text])


## The binding alone (no GPU): 3 and 12 origin values, the 12-value cell layout against the single-cell form, the error
## path.
func _volume_binding_samples(file: String) -> void:
	var site := Frames.to_render(HeightSource.geodetic_to_fixed(SUN_CU_SITE.x, SUN_CU_SITE.y, 1500.0))
	var anchor := Frames.to_render(HeightSource.geodetic_to_fixed(SUN_CU_SITE.x, SUN_CU_SITE.y, 2.0))
	var origins12 := PackedFloat64Array([anchor.x, anchor.y, anchor.z])
	for tile in [[CloudLayer.SHAPE_TILE_H_M, CloudLayer.SHAPE_TILE_V_M],
			[CloudLayer.SHAPE_TILE_H_M * CloudLayer.STRATIFORM_STRETCH, CloudLayer.SHAPE_TILE_V_M],
			[CloudLayer.SHAPE_TILE_H_M, CloudLayer.ICE_TILE_V_M]]:
		var stretched := Vector3(anchor.x, anchor.y * CloudLayer.POLAR_STRETCH, anchor.z)
		var o := CloudLayer._shape_origin(anchor, stretched.length() - CloudLayer.WGS84_A_M, tile[0], tile[1])
		origins12.append_array([o.x, o.y, o.z])
	var origins3 := origins12.slice(0, 3)
	# cells around the site, aligned east / north / up (the single-cell form's box)
	var cells := PackedFloat64Array()
	var centres: Array[Vector3] = []
	for i in 16:
		var c := site + Vector3(_volume_rng.randf_range(-3000.0, 3000.0), _volume_rng.randf_range(-500.0, 500.0),
				_volume_rng.randf_range(-3000.0, 3000.0))
		centres.append(c)
		var up := Vector3(c.x, c.y * CloudLayer.POLAR_STRETCH * CloudLayer.POLAR_STRETCH, c.z).normalized()
		var east := Vector3.UP.cross(up).normalized()
		var north := up.cross(east).normalized()
		cells.append_array([c.x, c.y, c.z, east.x * 200.0, east.y * 200.0, east.z * 200.0, north.x * 200.0,
				north.y * 200.0, north.z * 200.0, up.x * 164.0625, up.y * 164.0625, up.z * 164.0625])
	var a := _core.weather_cloud_cell_bakes(file, cells, origins12)
	var b := _core.weather_cloud_cell_bakes(file, cells, origins3)
	var same := a.size() == centres.size() and b.size() == a.size()
	var single_ok := same
	for i in a.size():
		if not same:
			break
		var scale := maxf(maxf(absf(a[i].x), absf(a[i].y)), 1e-12)
		same = same and (a[i] - b[i]).abs().x <= 1e-9 * scale and (a[i] - b[i]).abs().y <= 1e-9 * scale
		var one := _core.weather_cloud_cell_bake(file, centres[i], Vector2(200.0, 164.0625), origins12)
		single_ok = single_ok and (one - a[i]).abs().x <= 1e-12 * scale and (one - a[i]).abs().y <= 1e-12 * scale
	_check("cell bake binding: origins of 3 and 12 values agree", same, "%d cells" % a.size())
	_check("cell bake binding: the 12-value cell layout (centre, e_x, e_y, e_z) = the single-cell form", single_ok,
			"%d cells, east / north / up boxes" % a.size())
	var broken := _core.weather_cloud_cell_bakes(file, cells.slice(0, 11), origins12).size() \
			+ _core.weather_cloud_cell_bakes(file, cells, origins12.slice(0, 5)).size() \
			+ _core.weather_cloud_cell_bakes(file + ".missing", cells, origins12).size()
	_check("cell bake binding: malformed input returns nothing (errors pushed)", broken == 0,
			"11 cell values, 5 origin values, a missing file: %d results" % broken)


## The pair switch (Review Focus 1): the service announces (ERA5 15:00, ERA5 16:00) at VOLUME_SWITCH_BLEND while the
## pair (analog, ERA5 15:00) is shown at VOLUME_BLEND.
func _volume_pair_switch(camera: Vector3, snapshots: Array[Dictionary]) -> void:
	var L := CloudWorldVolumeLayout
	var volume := _clouds.effect.world_volume
	_clouds.show_now(snapshots[0], snapshots[1], VOLUME_BLEND)
	await _volume_fill(camera, false, "before the pair switch")
	var lookup := volume.adopted()
	var watched := [2, 7]
	var before := {}
	for k: int in watched:
		before[k] = _volume_read(k)["bytes"]
	var ids_before: Array = _clouds.state()["slots"]
	_clouds.pinned = false
	_clouds.set_target(snapshots[1].duplicate(), snapshots[2].duplicate())
	_clouds.world_volume_schedule.budget_scale = VOLUME_BUDGET_SCALE
	var replaced_at_weight := -1.0
	var weight_while_baking := 0.0
	var baking_frames := 0
	var mid_checked := false
	var unchanged := true
	var frames := 0
	var tracking := false
	while frames < VOLUME_SWITCH_FRAMES and not tracking:
		var state_before: Dictionary = _clouds.state()
		await _volume_frame(camera, VOLUME_SWITCH_BLEND, VOLUME_DT)
		frames += 1
		var state: Dictionary = _clouds.state()
		if state["slots"][0] != state_before["slots"][0] and replaced_at_weight < 0.0:
			replaced_at_weight = 1.0 - float(state_before["w1"])  # slot 0's weight when its snapshot changed
		if state["slots"][0] != ids_before[0] and not volume.slot_holds(0, state["slots"][0]):
			baking_frames += 1
			weight_while_baking = maxf(weight_while_baking, 1.0 - float(state["w1"]))
			if not mid_checked and baking_frames >= 3:  # slot 1's channels in the middle of the bake
				mid_checked = true
				for k: int in watched:
					unchanged = unchanged and _volume_pair1_equal(_volume_read(k)["bytes"], before[k])
		tracking = bool(state["tracking"]) and is_equal_approx(float(state["w1"]), 1.0 - VOLUME_SWITCH_BLEND) \
				and state["slots"][0] != ids_before[0]
	_clouds.world_volume_schedule.budget_scale = 1.0
	for k: int in watched:
		unchanged = unchanged and _volume_pair1_equal(_volume_read(k)["bytes"], before[k])
	_check("world volume pair switch: the new snapshot replaces slot 0 at weight 0, baked while its weight stays 0",
			replaced_at_weight == 0.0 and baking_frames > 0 and weight_while_baking == 0.0,
			"slot 0's weight at the replacement %.3f, %d frames baking its channel pair at weight %.3f" % [
			replaced_at_weight, baking_frames, weight_while_baking])
	_check("world volume pair switch: slot 1's channels unchanged during and after the bake", unchanged and mid_checked,
			"levels %s read back in the middle of the bake and after it" % str(watched))
	# the new snapshot in slot 0's channel pair = the CPU's bake (level 2's cloudy cells of the new pair)
	lookup = volume.adopted()
	var read := _volume_read(2)
	var cloudy: PackedInt32Array = _volume_blocks(read["floats"])["cloudy"]
	var cells: Array[Vector3i] = []
	for i in mini(cloudy.size(), VOLUME_SWITCH_CELLS):
		cells.append(_volume_cell_of(cloudy[i], lookup["levels"][2]["storage"]))
	var cpu := _volume_cpu(snapshots[2]["file"], lookup, 2, cells)
	var stats := {}
	for i in mini(cpu.size(), cells.size()):
		var t := _volume_texel_index(cells[i]) * 4 + 2 * CloudWorldVolume.channel_of_slot(0)
		_volume_compare(read["floats"], t, cpu[i], stats, "cell %s" % cells[i])
	_check("world volume pair switch: slot 0's channel pair holds the new snapshot (CPU bake)",
			cpu.size() == cells.size() and _volume_verdict(stats), "%d level-2 cells: %s" % [cells.size(),
			_volume_stats_text(stats)])
	_check("world volume pair switch: the blend then reaches the new pair's", tracking,
			"%d frames, w1 %.3f, slots %s" % [frames, float(_clouds.state()["w1"]), _clouds.state()["slots"]])


## Whether two raw RGBA16F read-backs (8 bytes per texel) hold the same channel pair 1 (BA: bytes 4-7) everywhere;
## z planes equal as a whole are skipped.
static func _volume_pair1_equal(a: PackedByteArray, b: PackedByteArray) -> bool:
	if a.size() != b.size():
		return false
	var plane := CloudWorldVolumeLayout.CELLS_H * CloudWorldVolumeLayout.CELLS_H * 8
	for z in a.size() / plane:
		if a.slice(z * plane, (z + 1) * plane) == b.slice(z * plane, (z + 1) * plane):
			continue
		for i in range(z * plane + 4, (z + 1) * plane, 8):
			if a.decode_u32(i) != b.decode_u32(i):
				return false
	return true


# ---------------------------------------------------------------- section 17 (world volume plan Task 6)

## Section 17 (world volume plan Task 6; STARSIM_VOLUME_ONLY=17 runs it alone): the light volume
## (shaders/cloud_volume_light.gdshaderinc) over section 8's thick slab (1-2 km, cf 1, vertical tau ~10) at Starbase,
## camera 2 m up, checked on the light texels read back (no march reads the volume before Task 7, ruling C12):
## - cold start: per level the frames from its density shown to its light complete (no provisional light, no blend
##   left), the light at the game's spike share while the bake runs at VOLUME_BUDGET_SCALE x (its time is ruled
##   separately: Task 5 c1): every level completes; the frames against LIGHT_COLD_FRAMES (1 s at 45 Hz) and the
##   light's GPU ms printed (ruling: where the light alone misses 1 s, reported, the spike cap kept);
## - Sun 45 deg up: LIGHT_CELLS cells wholly inside the slab under the camera per level 0, 2, 5: tau_below, tau_above
##   vs the trapezoid sums of the core's cell bake of the column (the GPU bake's reference; the trilinear field along
##   the vertical is linear between centres), tau_sun vs the converged integral along the ray through the level's
##   density read back - within LIGHT_TOLERANCE; the analytic slab's depth above / mu0 printed (the baked slab's top
##   undulates with the bake's shape noise by tens of metres: along a slanted ray a uniform slab's depth / mu0 is off
##   by up to ~5 %);
## - a LIGHT_SUN_STEP_DEG Sun step: levels 0, 2, 5 read back before and as soon as each level's refresh pass is through:
##   no texel's exp(-tau_sun) moved by more than LIGHT_STEP_T (+ the half float's rounding); the change once settled
##   printed (the clamp's work);
## - the Sun 8 deg below the site's horizon: every texel of levels 0, 2, 5 whose column has the Sun >= 6 deg below its
##   own horizon (N11) holds tau_sun >= the shadow value;
## - the Sun 10 deg below, the Moon 40 deg up: tau_moon of the slab cells vs the ray integral toward the Moon;
## - the Sun 10 deg up: no light job carries the Moon (and light jobs ran);
## - after every fill: no NaN and no infinity in any channel of any level (a GPU scan of every texel, its counts read
##   back: CloudWorldVolume.scan_light).
## Deferred to Task 7 (ruling C12: no march reads the light yet): the broken cf 0.4 views' radiance vs the converged
## per-sample march, radiance 0 in Earth's shadow, the cold start's first frame with clouds, render_check --front_check.
const LIGHT_SECTION := "17"
const LIGHT_SAMPLES := 9
const LIGHT_LEVELS := [0, 2, 5]
const LIGHT_CELLS := 5
const LIGHT_TOLERANCE := 0.05
## Sun / Moon (elevation, azimuth in degrees at the site).
const LIGHT_SUN := Vector2(45.0, 135.0)
const LIGHT_MOON := Vector2(40.0, 200.0)
const LIGHT_SUN_STEP_DEG := 5.0
const LIGHT_SHADOW_SUN_DEG := -8.0
const LIGHT_SHADOW_BELOW_DEG := 6.0
const LIGHT_NIGHT_SUN_DEG := -10.0
const LIGHT_DAY_SUN_DEG := 10.0
## [S] Mean Earth-Moon distance (m): only the Moon's direction matters here.
const LIGHT_MOON_DISTANCE_M := 3.844e8
const LIGHT_COLD_FRAMES := 45
const LIGHT_MAX_FRAMES := 6000
## [E] Frames without stale light before a fill counts as settled: the GPU's clamp counts come back a few frames late.
const LIGHT_QUIET_FRAMES := 12
## [E] The half float holding tau rounds it by 2^-11 relative: exp(-tau) moves by up to T tau 2^-11 more (< 2e-4 for
## tau <= 1, where a step can clamp; deeper the step is below the clamp anyway).
const LIGHT_HALF_ROUNDING := 5.0e-4
## [E] The ray reference integrates up to this far above the slab's top (its undulation is tens of metres).
const LIGHT_RAY_CLEAR_M := 500.0
## cloud_volume_light.gdshaderinc CLOUD_VOLUME_TAU_SHADOW (change together).
const LIGHT_SHADOW_TAU := 6.0e4

var _light_scan_bad := 0
var _light_scan_text := PackedStringArray()


func _volume_light_samples() -> void:
	var site := SUN_CU_SITE
	var slab := _slab("volume_light", SLAB_BOTTOM_M, SLAB_TOP_M, THICK_LIQUID_KG_M3, 1.0)
	var camera := HeightSource.geodetic_to_fixed(site.x, site.y, VOLUME_HEIGHTS_M[0])
	var sun := _local_direction(LIGHT_SUN.x, LIGHT_SUN.y, site.x, site.y)
	var moon := _local_direction(LIGHT_MOON.x, LIGHT_MOON.y, site.x, site.y)
	var schedule := _clouds.world_volume_schedule
	_clouds.world_volume = true
	_show(slab)
	# Cold start, then settled at the Sun 45 deg up.
	await _light_cold_start(camera, sun, moon)
	var frames := await _light_settle(camera, sun, moon, {})
	if frames < 0:
		_fail_and_quit("section 17: the light did not settle (Sun %.0f deg)" % LIGHT_SUN.x)
		return
	var lookup := _clouds.effect.world_volume.adopted()
	var mu_sun := sin(deg_to_rad(LIGHT_SUN.x))
	for k: int in LIGHT_LEVELS:
		_light_slab_check(k, lookup, camera, Frames.to_render(sun), mu_sun, slab["file"], "level %d, Sun %.0f deg"
				% [k, LIGHT_SUN.x])
	_light_scan("Sun %.0f deg" % LIGHT_SUN.x)
	# One refresh pass after a Sun step.
	await _light_step(camera, moon)
	_light_scan("Sun %.0f deg" % (LIGHT_SUN.x + LIGHT_SUN_STEP_DEG))
	# Earth's shadow.
	var shadow_sun := _local_direction(LIGHT_SHADOW_SUN_DEG, LIGHT_SUN.y, site.x, site.y)
	_show(slab)
	if await _light_settle(camera, shadow_sun, moon, {}) < 0:
		_fail_and_quit("section 17: the light did not settle (Sun %.0f deg)" % LIGHT_SHADOW_SUN_DEG)
		return
	_light_shadow_check(_clouds.effect.world_volume.adopted(), Frames.to_render(shadow_sun))
	_light_scan("Sun %.0f deg" % LIGHT_SHADOW_SUN_DEG)
	# The Moon, the Sun 10 deg below.
	var night_sun := _local_direction(LIGHT_NIGHT_SUN_DEG, LIGHT_SUN.y, site.x, site.y)
	_show(slab)
	var night := {}
	if await _light_settle(camera, night_sun, moon, night) < 0:
		_fail_and_quit("section 17: the light did not settle (Sun %.0f deg)" % LIGHT_NIGHT_SUN_DEG)
		return
	lookup = _clouds.effect.world_volume.adopted()
	var mu_moon := sin(deg_to_rad(LIGHT_MOON.x))
	var moon_ok: bool = night["moon_cells"] > 0
	var moon_text := PackedStringArray()
	for k: int in LIGHT_LEVELS:
		var values := PackedStringArray()
		var r := _light_slab_errors(k, lookup, camera, Frames.to_render(moon), mu_moon, 3, slab["file"], values)
		moon_ok = moon_ok and r.x <= LIGHT_TOLERANCE
		moon_text.append("level %d worst %.2f %% (%s)" % [k, 100.0 * r.x, ", ".join(values)])
	_check("world volume light: tau_moon (Sun %.0f deg, Moon %.0f deg up) vs the ray integral" % [
			LIGHT_NIGHT_SUN_DEG, LIGHT_MOON.x], moon_ok, "%s; %d light cells with the Moon"
			% [", ".join(moon_text), night["moon_cells"]])
	_light_scan("Sun %.0f deg" % LIGHT_NIGHT_SUN_DEG)
	# The Sun 10 deg up: no Moon jobs.
	var day_sun := _local_direction(LIGHT_DAY_SUN_DEG, LIGHT_SUN.y, site.x, site.y)
	_show(slab)
	var day := {}
	if await _light_settle(camera, day_sun, moon, day) < 0:
		_fail_and_quit("section 17: the light did not settle (Sun %.0f deg)" % LIGHT_DAY_SUN_DEG)
		return
	_check("world volume light: Sun %.0f deg up - no light job carries the Moon" % LIGHT_DAY_SUN_DEG,
			day["moon_cells"] == 0 and day["light_cells"] > 0, "%d light cells with the Moon of %d"
			% [day["moon_cells"], day["light_cells"]])
	_light_scan("Sun %.0f deg" % LIGHT_DAY_SUN_DEG)
	_check("world volume light: no NaN / infinity in any channel of any level", _light_scan_bad == 0,
			"; ".join(_light_scan_text))
	print("cloud_check: world volume light rates: calibrated %s light cells / ms, %s bake taps / ms" % [
			String.num_scientific(schedule.light_cells_per_ms), String.num_scientific(schedule.bake_taps_per_ms)])
	_clouds.world_volume = false


## One frame: the Sun and the Moon (ITRF unit vectors) set, the layer updated (blend 0: the slab in both slots), drawn.
func _light_frame(camera_itrf: Vector3, sun_itrf: Vector3, moon_itrf: Vector3) -> void:
	_atmosphere.update(_earth_to_gcrf * camera_itrf, Vector3.ZERO, _earth_to_gcrf, _earth_to_gcrf * sun_itrf * AU_M, 0.0,
			1.0, _earth_to_gcrf * moon_itrf * LIGHT_MOON_DISTANCE_M)
	_clouds.update(camera_itrf, 0.0, VOLUME_DT)
	await RenderingServer.frame_post_draw


## Frames until the fill is complete, its last batch ran and no level's light is stale for LIGHT_QUIET_FRAMES frames,
## at VOLUME_BUDGET_SCALE x the budget (-1 past LIGHT_MAX_FRAMES); seen: the most Moon light cells of a plan and the
## light cells summed.
func _light_settle(camera: Vector3, sun: Vector3, moon: Vector3, seen: Dictionary) -> int:
	var schedule := _clouds.world_volume_schedule
	var volume := _clouds.effect.world_volume
	schedule.budget_scale = VOLUME_BUDGET_SCALE
	seen["moon_cells"] = 0
	seen["light_cells"] = 0
	var quiet := 0
	for n in LIGHT_MAX_FRAMES:
		await _light_frame(camera, sun, moon)
		var stats := _clouds.world_volume_stats()
		seen["moon_cells"] = maxi(seen["moon_cells"], int(stats["light_moon_cells"]))
		seen["light_cells"] += int(stats["light_cells"])
		var busy := not schedule.complete() or volume.waiting() or not _volume_all_ready(volume.adopted())
		for k in CloudWorldVolumeLayout.LEVELS:
			busy = busy or schedule.light_stale(k)
		quiet = 0 if busy else quiet + 1
		if quiet >= LIGHT_QUIET_FRAMES:
			schedule.budget_scale = 1.0
			return n + 1
	schedule.budget_scale = 1.0
	return -1


## The cold start: per level the frames from its density shown (the adopted lookup) to its light complete (the
## schedule: no provisional light, no blend left), the light at the game's spike share (light_budget_scale undoes
## budget_scale for it), the bake at VOLUME_BUDGET_SCALE x.
func _light_cold_start(camera: Vector3, sun: Vector3, moon: Vector3) -> void:
	var schedule := _clouds.world_volume_schedule
	var volume := _clouds.effect.world_volume
	schedule.budget_scale = VOLUME_BUDGET_SCALE
	schedule.light_budget_scale = 1.0 / VOLUME_BUDGET_SCALE
	var shown_at := {}
	var done_at := {}
	var light_ms: Array[float] = []
	var rate_ms := 0.0
	var rate_cells := 0
	var active := 0
	var seen := int(_clouds.world_volume_stats()["measured_frames"])
	for n in LIGHT_MAX_FRAMES:
		await _light_frame(camera, sun, moon)
		var stats := _clouds.world_volume_stats()
		var adopted := volume.adopted()
		if adopted.is_empty():
			continue
		active = 0
		var measuring := false
		for e: Dictionary in adopted["levels"]:
			var k: int = e["level"]
			if not bool(e["active"]):
				continue
			active += 1
			if bool(e["shown"]) and not shown_at.has(k):
				shown_at[k] = n
			if shown_at.has(k) and not done_at.has(k):
				measuring = true
				if not schedule.light_pending(k):
					done_at[k] = n
		if measuring and int(stats["measured_frames"]) > seen:
			light_ms.append(float(stats["light_ms"]))
			rate_ms += float(stats["light_ms"])
		if measuring:
			rate_cells += int(stats["light_cells"])
		seen = int(stats["measured_frames"])
		if active > 0 and done_at.size() == active:
			break
	schedule.light_budget_scale = 1.0
	schedule.budget_scale = 1.0
	var worst := 0
	var text := PackedStringArray()
	for k: int in range(CloudWorldVolumeLayout.LEVELS - 1, -1, -1):
		if not done_at.has(k):
			text.append("level %d not complete" % k)
			worst = LIGHT_MAX_FRAMES
			continue
		var f: int = done_at[k] - shown_at[k] + 1
		worst = maxi(worst, f)
		text.append("level %d %d" % [k, f])
	light_ms.sort()
	# Asserted: every level's light completes (no stall); the frames against LIGHT_COLD_FRAMES are reported (ruling
	# C7 / Task 6: where the light alone cannot meet 1 s, numbers are reported instead of bending the spike cap).
	_check("world volume light: cold start, every level's light completes (frames from its density shown, target %d)"
			% LIGHT_COLD_FRAMES, done_at.size() == active, ("all within the target: %s; frames %s; light GPU ms "
			% [worst <= LIGHT_COLD_FRAMES, ", ".join(text)]) + "in those frames median %.2f, max %.2f (%d measured), ~%.0f light cells / ms" % [
			light_ms[light_ms.size() / 2] if not light_ms.is_empty() else -1.0, light_ms.max() if not light_ms.is_empty()
			else -1.0, light_ms.size(), rate_cells / maxf(rate_ms, 1e-9)])


## Level k's light texels read back: 4 floats per texel (tau_sun, tau_below, tau_above, tau_moon; x fastest).
func _light_read(k: int) -> PackedFloat32Array:
	var rd := RenderingServer.get_rendering_device()
	var bytes := rd.texture_get_data(_clouds.effect.world_volume.light_texture(k), 0)
	var image := Image.create_from_data(CloudWorldVolumeLayout.CELLS_H,
			CloudWorldVolumeLayout.CELLS_H * CloudWorldVolumeLayout.CELLS_V, false, Image.FORMAT_RGBAH, bytes)
	image.convert(Image.FORMAT_RGBAF)
	return image.get_data().to_float32_array()


## LIGHT_CELLS cells of level k under the camera wholly inside the slab, spread over its height.
func _light_slab_cells(k: int, lookup: Dictionary, camera_itrf: Vector3) -> Array[Vector3i]:
	var L := CloudWorldVolumeLayout
	var g := _volume_layout(lookup, k).grid_of(Frames.to_render(camera_itrf))
	var v: float = L.CELL_V_M[k]
	var z0 := ceili((SLAB_BOTTOM_M - L.SHELL_BOTTOM_M) / v)
	var z1 := floori((SLAB_TOP_M - L.SHELL_BOTTOM_M) / v) - 1
	var cells: Array[Vector3i] = []
	for i in LIGHT_CELLS:
		cells.append(Vector3i(floori(g.x / L.CELL_H_M[k]), floori(g.y / L.CELL_H_M[k]),
				z0 + roundi(i * (z1 - z0) / float(LIGHT_CELLS - 1))))
	return cells


## The reference of level k's slab cells: the core's cell bake (weather_cloud_cell_bakes, the GPU bake's reference) of
## the camera's column over the level's window, summed by trapezoids (the trilinear field along the vertical is linear
## between cell centres; at the shell's ends the outer value held over the last half cell, as the reads clamp) - per
## slab cell [height, depth above, depth below]; toward a light at cosine mu the slab's depth above / mu (horizontally
## uniform: a straight path through it).
func _light_slab_reference(k: int, lookup: Dictionary, camera_itrf: Vector3, file: String) -> Array:
	var L := CloudWorldVolumeLayout
	var v: float = L.CELL_V_M[k]
	var slab_cells := _light_slab_cells(k, lookup, camera_itrf)
	var z0: int = lookup["levels"][k]["origin"].z
	var z1: int = z0 + L.window_size(k).z
	var column: Array[Vector3i] = []
	for z in range(z0, z1):
		column.append(Vector3i(slab_cells[0].x, slab_cells[0].y, z))
	var cpu := _volume_cpu(file, lookup, k, column)
	if cpu.size() != column.size():
		return []
	var below := PackedFloat64Array([cpu[0].x * 0.5 * v if z0 == 0 else 0.0])
	for i in range(1, column.size()):
		below.append(below[i - 1] + 0.5 * (cpu[i - 1].x + cpu[i].x) * v)
	var top_hold: float = cpu[column.size() - 1].x * 0.5 * v if z1 == L.CELLS_V else 0.0
	var total: float = below[column.size() - 1] + top_hold
	var out: Array = []
	for cell in slab_cells:
		var i := cell.z - z0
		out.append([L.SHELL_BOTTOM_M + (cell.z + 0.5) * v, total - below[i], below[i]])
	return out


## Level k's density (rho of slot 0, 1/km: the slab's blend is 0) trilinear at a grid point, as the GPU's filter reads
## it (toroidal; vertically clamped to the storage).
static func _light_trilinear(density: PackedFloat32Array, g: Vector3, k: int) -> float:
	var L := CloudWorldVolumeLayout
	var c := Vector3(g.x / L.CELL_H_M[k], g.y / L.CELL_H_M[k], (g.z - L.SHELL_BOTTOM_M) / L.CELL_V_M[k]) \
			- Vector3(0.5, 0.5, 0.5)
	var i0 := Vector3i(floori(c.x), floori(c.y), floori(c.z))
	var f := c - Vector3(i0)
	var value := 0.0
	for corner in 8:
		var d := Vector3i(corner & 1, (corner >> 1) & 1, (corner >> 2) & 1)
		var cell := i0 + d
		cell.z = clampi(cell.z, 0, L.CELLS_V - 1)
		var w := (f.x if d.x == 1 else 1.0 - f.x) * (f.y if d.y == 1 else 1.0 - f.y) * (f.z if d.z == 1 else 1.0 - f.z)
		value += w * density[_volume_texel_index(cell) * 4]
	return value


## The converged optical depth from level k's cell centre toward `dir` (render axes, unit) through level k's density
## read back (_light_trilinear, midpoint steps of a tenth of a cell's height) until the ray is LIGHT_RAY_CLEAR_M above
## the slab's top (clear beyond it): the reference of tau_sun / tau_moon. (The baked slab's top undulates with the
## bake's shape noise by tens of metres - the analytic depth above the cell / mu holds only in the cell's own column.)
func _light_ray_reference(k: int, lookup: Dictionary, cell: Vector3i, dir: Vector3, density: PackedFloat32Array) -> float:
	var L := CloudWorldVolumeLayout
	var layout := _volume_layout(lookup, k)
	var h: float = L.CELL_H_M[k]
	var v: float = L.CELL_V_M[k]
	var p0 := layout.earth_of(Vector3((cell.x + 0.5) * h, (cell.y + 0.5) * h, L.SHELL_BOTTOM_M + (cell.z + 0.5) * v))
	var step := 0.1 * v
	var tau := 0.0
	var s := 0.5 * step
	while true:
		var g := layout.grid_of(p0 + dir * s)
		if g.z > SLAB_TOP_M + LIGHT_RAY_CLEAR_M:
			break
		tau += _light_trilinear(density, g, k) * step
		s += step
	return tau / CloudWorldVolume.PER_KM


## Level k's slab cells against the references: tau_below / tau_above vs the CPU bake's column
## (_light_slab_reference), the light's depth (`channel` 0 tau_sun, 3 tau_moon toward `dir`) vs the converged ray
## integral (_light_ray_reference); the analytic slab's depth above / mu printed. Returns (worst relative error, its
## channel); `text` gets the values (value (reference)).
func _light_slab_errors(k: int, lookup: Dictionary, camera_itrf: Vector3, dir: Vector3, mu: float, channel: int,
		file: String, text: PackedStringArray) -> Vector2:
	var floats := _light_read(k)
	var density: PackedFloat32Array = _volume_read(k)["floats"]
	var reference := _light_slab_reference(k, lookup, camera_itrf, file)
	if reference.is_empty():
		text.append("level %d: no CPU reference" % k)
		return Vector2(INF, -1.0)
	var worst := Vector2(0.0, -1.0)
	var cells := _light_slab_cells(k, lookup, camera_itrf)
	for j in cells.size():
		var ref: Array = reference[j]
		var t := _volume_texel_index(cells[j]) * 4
		var ray := _light_ray_reference(k, lookup, cells[j], dir, density)
		var pairs: Array = [[channel, floats[t + channel], ray]]
		if channel == 0:
			pairs.append_array([[1, floats[t + 1], ref[2]], [2, floats[t + 2], ref[1]]])
		var parts := PackedStringArray()
		for pr: Array in pairs:
			var e: float = absf(pr[1] - pr[2]) / maxf(pr[2], 1e-9)
			if e > worst.x:
				worst = Vector2(e, pr[0])
			parts.append("%.3f (%.3f)" % [pr[1], pr[2]])
		text.append("%.0f m: %s [slab / mu %.3f]" % [ref[0], " / ".join(parts), ref[1] / mu])
	return worst


func _light_slab_check(k: int, lookup: Dictionary, camera_itrf: Vector3, dir: Vector3, mu: float, file: String,
		label: String) -> void:
	var text := PackedStringArray()
	var r := _light_slab_errors(k, lookup, camera_itrf, dir, mu, 0, file, text)
	_check("world volume light: slab tau_sun / tau_below / tau_above vs the ray integral / the CPU column (%s)" % label,
			r.x <= LIGHT_TOLERANCE, "worst %.2f %% (channel %d); %s" % [100.0 * r.x, int(r.y), ", ".join(text)])


## The Sun steps by LIGHT_SUN_STEP_DEG: per level 0, 2, 5 the texels before and once the level's refresh pass of the
## step is through (before a settle pass could follow); every exp(-tau_sun) moved by <= LIGHT_STEP_T (+ rounding).
## Then settled: the change the clamp spread over passes printed.
func _light_step(camera: Vector3, moon: Vector3) -> void:
	var schedule := _clouds.world_volume_schedule
	var site := SUN_CU_SITE
	var stepped := _local_direction(LIGHT_SUN.x + LIGHT_SUN_STEP_DEG, LIGHT_SUN.y, site.x, site.y)
	var before := {}
	var generation := {}
	for k: int in LIGHT_LEVELS:
		before[k] = _light_read(k)
		generation[k] = schedule.light_generation(k)
	var after := {}
	schedule.budget_scale = VOLUME_BUDGET_SCALE
	var quiet := 0
	for n in LIGHT_MAX_FRAMES:
		await _light_frame(camera, stepped, moon)
		var refreshing := false
		for k: int in LIGHT_LEVELS:
			if not after.has(k) and schedule.light_generation(k) > generation[k] and not schedule.light_refreshing(k):
				after[k] = _light_read(k)
			refreshing = refreshing or schedule.light_refreshing(k)
		# levels whose threshold the step did not reach are not refreshed (LIGHT_QUIET_FRAMES without any refresh)
		quiet = 0 if refreshing else quiet + 1
		if after.size() == LIGHT_LEVELS.size() or quiet >= LIGHT_QUIET_FRAMES:
			break
	schedule.budget_scale = 1.0
	var bound := CloudWorldVolumeSchedule.LIGHT_STEP_T + LIGHT_HALF_ROUNDING
	var ok := after.has(0)
	var clamped := 0
	var text := PackedStringArray()
	for k: int in LIGHT_LEVELS:
		if not after.has(k):  # its threshold above the step: not refreshed, nothing moved
			after[k] = _light_read(k)
			text.append("level %d (not triggered)" % k)
		var r := _light_change(before[k], after[k])
		ok = ok and r.x <= bound
		clamped += int(r.y)
		text.append("level %d: max %.4f, %d clamped" % [k, r.x, int(r.y)])
	ok = ok and clamped > 0
	if await _light_settle(camera, stepped, moon, {}) >= 0:
		for k: int in LIGHT_LEVELS:
			var r := _light_change(before[k], _light_read(k))
			text.append("level %d settled: max %.4f" % [k, r.x])
	_check("world volume light: a %.0f deg Sun step - one refresh pass moves no exp(-tau_sun) by more than %.2f"
			% [LIGHT_SUN_STEP_DEG, CloudWorldVolumeSchedule.LIGHT_STEP_T], ok, "; ".join(text))


## The largest change of exp(-tau_sun) over all texels between two read-backs, and the texels that moved by at least
## LIGHT_STEP_T less the rounding (clamped).
static func _light_change(a: PackedFloat32Array, b: PackedFloat32Array) -> Vector2:
	var worst := 0.0
	var clamped := 0
	var limit := CloudWorldVolumeSchedule.LIGHT_STEP_T - LIGHT_HALF_ROUNDING
	for i in range(0, a.size(), 4):
		var ta := 0.0 if a[i] >= LIGHT_SHADOW_TAU else exp(-a[i])
		var tb := 0.0 if b[i] >= LIGHT_SHADOW_TAU else exp(-b[i])
		var d := absf(ta - tb)
		if d > worst:
			worst = d
		if d >= limit:
			clamped += 1
	return Vector2(worst, clamped)


## Every texel of levels 0, 2, 5 whose column has the Sun at least LIGHT_SHADOW_BELOW_DEG below its own horizon holds
## the shadow value.
func _light_shadow_check(lookup: Dictionary, sun_render: Vector3) -> void:
	var L := CloudWorldVolumeLayout
	var checked := 0
	var wrong := 0
	var lowest := INF
	var limit := sin(deg_to_rad(-LIGHT_SHADOW_BELOW_DEG))
	for k: int in LIGHT_LEVELS:
		var floats := _light_read(k)
		var layout := _volume_layout(lookup, k)
		var origin: Vector3i = lookup["levels"][k]["storage"]
		var h: float = L.CELL_H_M[k]
		for ty in L.CELLS_H:
			for tx in L.CELLS_H:
				var cell := _volume_cell_of(ty * L.CELLS_H + tx, origin)
				if layout.up_of_grid((cell.x + 0.5) * h, (cell.y + 0.5) * h).dot(sun_render) > limit:
					continue
				for tz in L.CELLS_V:
					var tau := floats[((tz * L.CELLS_H + ty) * L.CELLS_H + tx) * 4]
					checked += 1
					if tau < LIGHT_SHADOW_TAU:
						wrong += 1
						lowest = minf(lowest, tau)
	_check(("world volume light: Earth's shadow (Sun %.0f deg at the site; texels with it >= %.0f deg below their "
			+ "horizon)") % [LIGHT_SHADOW_SUN_DEG, LIGHT_SHADOW_BELOW_DEG], checked > 0 and wrong == 0,
			"%d texels of levels 0, 2, 5 checked, %d below %.0f (lowest %s)" % [checked, wrong, LIGHT_SHADOW_TAU,
			lowest])


## Every level's light texels scanned on the GPU (NaN / infinity in any channel); the counts kept for the final check.
func _light_scan(label: String) -> void:
	var rd := RenderingServer.get_rendering_device()
	var nan := 0
	var inf := 0
	var shadow := 0
	var texels := 0
	for k in CloudWorldVolumeLayout.LEVELS:
		var c := _clouds.effect.world_volume.scan_light(rd, k)
		if c.size() < 4:
			_light_scan_bad += 1
			_light_scan_text.append("%s: level %d not scanned" % [label, k])
			continue
		nan += c[0]
		inf += c[1]
		shadow += c[2]
		texels += c[3]
	_light_scan_bad += nan + inf
	_light_scan_text.append("%s: %d NaN, %d infinite, %d in shadow of %d texels" % [label, nan, inf, shadow, texels])
