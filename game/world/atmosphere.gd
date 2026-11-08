class_name Atmosphere
extends RefCounted
## The Earth's precomputed atmosphere on the GPU (M8 atmosphere): loads the core's tables (StarsimCore.load_atmosphere:
## the cache in data/atmosphere, or a first-run precomputation) on a WorkerThreadPool task, uploads them as float
## textures, sets the global shader uniforms of shaders/atmosphere.gdshaderinc and updates the per-frame ones.
## No physics here: the tables and every evaluation come from the core; this only converts frames and units.
##
## Scene units (one exposure, spec "One exposure for everything"): scene value = luminance (cd/m^2) x DAY_EXPOSURE x
## pi / E_v,TOA x 1 / d_AU^2 - a white Lambertian surface under the top-of-atmosphere Sun at 1 AU renders as
## DAY_EXPOSURE, as the Earth shaders' day lighting does. The eye adaptation (EyeAdaptation, fidelity flag
## eye_adaptation_enabled) multiplies all of it by the cone gain (1 by day, up to ~1.2e4 at night) and adds the rods'
## image in a post step (VisionEffect, night vision).

## Scene value of a white Lambertian surface under the full TOA Sun, the one exposure of the game [call: M8 atmosphere
## spec, "stays 1.5"]. The `day_exposure` uniforms of shaders/earth.gdshader and terrain_earth.gdshader are set from
## it (world.gd); they have no default of their own.
const DAY_EXPOSURE := 1.5
## Astronomical unit (m) [S: IAU 2012 Resolution B2]; the tables are for the Sun at 1 AU.
const ASTRONOMICAL_UNIT_M := 149597870700.0
## a / b of WGS84 (1 / (1 - f), f = 1 / 298.257223563) [S: NIMA TR8350.2]: the model space's polar stretch, as the core.
const POLAR_STRETCH := 1.0 / (1.0 - 1.0 / 298.257223563)
const CACHE_FILE := "earth_atmosphere.bin"
## [S: ITU-R BT.709] luminance weights of the linear RGB channels (as star_visibility.gdshaderinc).
const LUMINANCE_WEIGHTS := Vector3(0.2126, 0.7152, 0.0722)
## [call] In-scattering segments of the eclipse sky while the Moon's penumbra touches the atmosphere (spec goal 8).
const ECLIPSE_SEGMENTS := 8
## [S: IAU 2015 Resolution B3] nominal solar radius; [S: core constants.hpp moon_mean_radius_m] the Moon's mean radius.
const SUN_RADIUS_M := 695700000.0
const MOON_RADIUS_M := 1737400.0
## [S: NIMA TR8350.2] WGS84 semi-major axis.
const WGS84_A_M := 6378137.0
## [call] Heights (m, above the ellipsoid) of the umbra's light (_update_umbra_light) and its update interval (s).
const UMBRA_HEIGHTS_M := [0.0, 4000.0, 12000.0]
const UMBRA_UPDATE_S := 1.0
## [call] _eclipsed_sky_fluence's elevation samples (deg: the zenith, then rings of four azimuths) and the edges of the
## bands they stand for; dense near the horizon, where the totality sky brightens steeply (a 0-15 deg band sampled at
## 6 deg alone gave 3x less fluence than at 4 deg). Solid angle of a band: 2 pi (sin e2 - sin e1), split over its ring.
const UMBRA_ELEVATIONS_DEG := [90.0, 30.0, 11.0, 6.0, 3.0, 1.0]
const UMBRA_BAND_EDGES_DEG := [50.0, 15.0, 8.0, 4.5, 2.0, 0.0]
## [call] Segments per direction (converged: 8 vs 16 within 9 %).
const UMBRA_SEGMENTS := 8
## [call] The eye adaptation's limb samples graze the Earth this high (m) above the model ground.
const LIMB_SAMPLE_HEIGHT_M := 10000.0
## The eye adaptation away from the Earth (adaptation_luminance, _moon_in_view_luminance):
## [S: World.EARTH_MEAN_RADIUS_M, the Moon shaders' earth_radius_m] the Earth's mean radius of the Earth's shadow.
const EARTH_MEAN_RADIUS_M := 6371000.0
## [D] Mean albedo-texture value of the lunar surface: data/textures/moon_color.png (CGI Moon Kit, LROC WAC colour
## mosaic, tools/data_pipeline/sources.toml "moon_color") sRGB-decoded, BT.709 luminance, area-weighted over the sphere
## (0.2997, kept by the WAC w-map tie of tools/data_pipeline/moon_albedo.py) - the albedo the Moon shaders multiply.
const MOON_MEAN_TEXTURE_ALBEDO := 0.30
## [D] The albedo texture's mean colour (linear Rec.709, area-weighted over the sphere, same decode as above): B/R 0.894,
## G/R 0.974 - red-sloped, but only about a third of the real regolith's slope in log ratio (look fixes T5b).
const MOON_TEXTURE_MEAN_RGB := Vector3(0.3077, 0.2998, 0.2753)
## [S: Allen's Astrophysical Quantities, full Moon B-V 0.92 vs the Sun's 0.65 -> R(440)/R(550) 0.78; a linear spectral
## slope through the CIE 1931 observer (Wyman et al. 2013 fit) into linear Rec.709 under a neutral light] The mean
## regolith's reflectance colour as ratios to red: G/R 0.85, B/R 0.66 (0.62-0.73 for dB-V 0.20-0.32).
const MOON_REGOLITH_COLOUR_RATIO := Vector3(1.0, 0.85, 0.66)
## Fidelity flag (user 2026-11-07): the regolith colour factor on. Off: the texture's own colour (the former look;
## render_check --moon_colour=texture).
static var moon_colour_correction := true
## Fidelity flag (bug hunt 2026-11-08, user: "the Moon is cream, not grey"): near the Moon the field's illuminant takes
## the regolith's colour by the share of the field the Moon fills (moon_field_tint). Off: the Sun's colour above the air
## (the T5b look; render_check --moon_field=off).
static var moon_field_adaptation := true
## The regolith's reflectance (shaders/moon_regolith.gdshaderinc, regolith_reflectance here; night-vision fix round I3,
## I4): albedo x regolith_scale x E/pi x Lommel-Seeliger x the K&S phase factor (or Hapke x hapke_normalisation, the
## default since the moon procedural surface plan Task 9), for the Sun and the earthshine alike.
## [D: tools/validation/moon_disc_calibration.py] the albedo texture's near-side mean weighted by projected area
## (cos^2 lat cos lon, sRGB-decoded, BT.709): the full-phase disc seen from the Earth (the maria pull it below the
## sphere's 0.30). The texture's large-scale albedo follows the WAC Hapke w map since Task 11 fix C
## (tools/data_pipeline/moon_albedo.py; the bare colour mosaic gave 0.2245).
const MOON_NEAR_SIDE_DISC_ALBEDO := 0.2611
## [S: K&S 1991 sec. 2, as Schaefer's skycalc applies it] the opposition surge: the Moon's illuminance x (1.35 - 0.05 a)
## for phase angles a < 7 deg (1 at 7 deg). The shaders' REGOLITH_* constants: keep in sync.
const MOON_OPPOSITION_PEAK := 1.35
const MOON_OPPOSITION_PER_DEG := 0.05
const MOON_OPPOSITION_END_DEG := 7.0
## [call] The regolith's phase law is held beyond this angle (K&S's quartic turns up past ~150 deg).
const REGOLITH_PHASE_MAX_DEG := 150.0
## [call] The regolith's least cosine to the eye (moon_regolith.gdshaderinc): a normal turned further away (the faceted
## silhouette) is taken back onto the limb before the light's cosine.
const REGOLITH_MU_MIN := 1.0e-3
## [D: pi x the tables' 4.24e4 cd/m^2 white plate] TOA illuminance (lux) for the regolith scale before the tables load.
const NOMINAL_TOA_LUX := 1.3327e5
## [S: NASA Earth Fact Sheet, Bond albedo 0.306] the Earth's light on the Moon (earthshine, earthshine_relative).
const EARTH_BOND_ALBEDO := 0.306
## [S: Lambert-sphere phase integral q = 3/2, A_Bond = p q; e.g. Seager, Exoplanet Atmospheres (2010) ch. 3] the
## geometric albedo of a Lambertian sphere over its Bond albedo.
const LAMBERT_GEOMETRIC_PER_BOND := 2.0 / 3.0
## Moonlight (night vision, 2026-10-25): the Moon's illuminance outside the atmosphere at the mean distance by
## Krisciunas & Schaefer 1991, "A model of the brightness of moonlight", PASP 103, 1033 [S]: Eq. 9 the Moon's V
## magnitude m = -12.73 + 0.026 |a| + 4e-9 a^4 (phase angle a in degrees), Eq. 8 the illuminance
## I* = 10^(-0.4 (m + 16.57)) foot-candles (full Moon 0.313 lux) [S: 1 fc = 10.7639 lux]; scaled by
## (MOON_MEAN_DISTANCE_M / d)^2 [D]. Their accuracy: 8-23 %.
const MOON_MAGNITUDE_FULL := -12.73
const MOON_MAGNITUDE_PER_DEG := 0.026
const MOON_MAGNITUDE_PER_DEG4 := 4.0e-9
const MOON_FOOT_CANDLE_OFFSET := 16.57
const LUX_PER_FOOT_CANDLE := 10.763910
## [S: Krisciunas & Schaefer 1991, the mean Earth-Moon distance of their model] (m).
const MOON_MEAN_DISTANCE_M := 384400000.0
## [call] Moonlight is evaluated (shaders, the vehicles' second light, the adaptation's Moon terms) only while the
## Moon's white plate (relative illuminance x the reference luminance) is at least this share of the luminance the eye
## is adapted to: below it the moonlight is invisible. By day (full Moon ~0.1 cd/m^2 against >= 1e3) and at new Moon
## (an eclipse) it is off - day frames stay exactly as before, at no cost.
const MOONLIGHT_MIN_SHARE := 1.0e-3
## [call] The twilight glow (_twilight_glow_luminance): elevations sampled along the Sun's azimuth (deg; the glow is
## brightest low over the horizon) and the soft exclusion around the Sun: a sample's luminance is weighted by a smoothstep
## of its angle from the Sun, 0 up to TWILIGHT_GLOW_MIN_SUN_DEG (inside the aureole) and 1 from TWILIGHT_GLOW_FULL_SUN_DEG
## [E: the aureole's glare has faded to the sky's own level by ~12-15 deg]. A hard cutoff made the target jump whenever a
## sample crossed it (eye-adaptation.md).
const TWILIGHT_GLOW_ELEVATIONS_DEG := [1.0, 3.0, 6.0, 10.0, 20.0, 40.0]
const TWILIGHT_GLOW_MIN_SUN_DEG := 8.0
const TWILIGHT_GLOW_FULL_SUN_DEG := 14.0
## [E] By day the brightest sky is the Sun's aureole (38 000 cd/m^2 at Sun 15 deg, above the white plate): the glow is a
## twilight term, so it fades out with the Sun's elevation, full up to the first and gone from the second (deg). The plate
## is the larger term from Sun +3 deg on (4800 vs 6800 cd/m^2 at +5), so the fade changes no day value and no target steps.
const TWILIGHT_GLOW_FADE_START_DEG := 3.0
const TWILIGHT_GLOW_FADE_END_DEG := 8.0
## [call] In a solar eclipse at the eye (night-vision fix round I2) the brightest sky is the ring around the umbra: the
## glow then samples these azimuths (evenly from the Sun's) at these elevations, the eclipsed sky (_eclipsed_in_scatter).
const ECLIPSE_GLOW_AZIMUTHS := 8
const ECLIPSE_GLOW_ELEVATIONS_DEG := [1.0, 3.0, 6.0, 12.0]
## [S: Ferwerda et al. 1996 sec. 4.4] the world adaptation luminance is half the highest visible luminance: the glow
## terms (twilight, totality, the orbital limb) enter at this share of the brightest sky (review M5).
const GLOW_ADAPTATION_SHARE := 0.5
## [call] The orbital-night limb glow (_limb_glow_luminance): tangent heights sampled toward the Sun (m).
const LIMB_GLOW_HEIGHTS_M := [2000.0, 5000.0, 10000.0, 20000.0, 40000.0]
## [call] An eye on the Moon stands at least this high above its ground sphere (render ground views: 2 m).
const MOON_EYE_MIN_HEIGHT_M := 2.0
## [E] The terrain's horizon decides the Sun within this distance of the Moon's centre (its sphere of influence, 66 000 km):
## no height switch (relief shadows at every height, MoonSunHorizon's exact pre-check skips the march where nothing can
## reach the Sun); beyond it - Earth's surroundings, where the atmosphere / the Earth's eclipse own the Sun - the mean
## sphere stays (its step there: relief 10.8 km / 66 000 km = 0.009 deg, under 2 % of the solar disc).
const MOON_TERRAIN_SUN_MAX_DISTANCE_M := 66.0e6
## [call] The lunar cap's quadrature: rings uniform in sin^2 of the nadir angle (cosine weights), directions per ring.
const MOON_VIEW_RINGS := 4
const MOON_VIEW_AZIMUTHS := 8

## True once the textures are uploaded and atmosphere_enabled is set.
var ready := false
## A table load is running on a worker thread.
var loading := false
## Table key of the status line for the UI while the tables load or compute ("" when idle); after a failure the key of
## the report text (report.<code>). Resolved at display time (status_text), so a language switch re-renders it.
var status_key := ""
## AtmosphereParameters of the loaded tables (StarsimCore.load_atmosphere "parameters").
var parameters := {}
## GPU format of the tables, chosen before start_loading. Half float [call, Task 5 fix round 1]: half the VRAM (42 vs 85
## MB) and fetch bandwidth; relative rounding ~5e-4, the GPU check stays within its 2 % (numbers in the Task 5
## report); RGBAF for comparisons (atmosphere_check.gd --f32). The transmittance table is always RGBAF [call, Task 6]:
## its grazing values from the ground (T to the top ~1e-5 and below) fall into the half-float subnormal range, and the
## aerial perspective divides two of them (a terrain pixel 20 km from the Cape camera was 4 % off in blue with RGBAH,
## 0.05 % with RGBAF); +8 MB.
var table_format := Image.FORMAT_RGBAH
## Per-frame values (readable by tests): table luminance (cd/m^2 for the Sun at 1 AU) -> scene value (includes the
## 1 / d_AU^2 of the true Sun distance: do not apply it to a true luminance), camera height above the model ground (stretched
## space), the Sun's Earth-local direction and the camera's Earth-local position.
var luminance_scale := 0.0
var camera_altitude_m := 0.0
var sun_direction_local := Vector3.ZERO
var camera_local_m := Vector3.ZERO
## Fidelity flag: eye adaptation (EyeAdaptation) - on in the game; off: the fixed day exposure (atmosphere_check.gd).
var eye_adaptation_enabled := true
## The background share the measured field is taken as (EyeAdaptation.FIELD_MID_GREY, Y_b 0.2, by default; one value for
## every scene since T13 - the white cabin's walls are held by the local adaptation; cockpit_shots.gd --field_key and
## --cabin_keys set it for comparisons).
var field_key := _field_key_from_arguments(OS.get_cmdline_user_args())


## User argument --field_key=<v> (comparisons; T13: --protect_adaptation --field_key=0.12 --display_curve=agx is the
## rule before T13), else Y_b.
static func _field_key_from_arguments(arguments: PackedStringArray) -> float:
	for argument in arguments:
		if argument.begins_with("--field_key="):
			var value := float(argument.get_slice("=", 1))
			if value > 0.0:
				return value
			push_warning("Atmosphere: ignoring %s (not a positive number)" % argument)
	return EyeAdaptation.FIELD_MID_GREY
## The cone gain of the eye adaptation this frame (1 without it). Times `brightness` it is in luminance_scale and
## scene_per_cd_m2, and the shaders read that product as atmosphere_exposure_gain for what is not lit through the tables
## (Moon, Sun disc).
var exposure_gain := 1.0
## Artificial light the eye sees (cd/m^2): the luminance of the lamp-lit surfaces of a site in view (World sets it each frame
## from TowerView.lamp_adaptation_cd_m2, 0 = none); enters L_a as one more candidate of the maximum, view-weighted by the caller.
var lamp_adaptation_cd_m2 := 0.0
## The colour of those lamps' light (linear Rec.709 at luminance 1; TowerView.lamp_light_colour): the field illuminant's
## lamp part (look fixes T5b fix round 1).
var lamp_light_colour := Vector3.ONE
## An enclosure the eye is in and its own light (linear Rec.709 at luminance 1, World: the cabin lamps of a cockpit the
## camera is inside, CockpitView.light_colour); ZERO: none. The field illuminant blends to it as the surround's
## visibility V closes (V 0: the enclosed view is lit by its lamps).
var interior_light_colour := Vector3.ZERO
## The adopted white this frame (look fixes T5b fix round 1, user 2026-11-08): the field's illuminant (field_illuminant,
## linear Rec.709 at luminance 1) and CIECAM02's degree of adaptation D toward it (EyeAdaptation.white_degree).
var field_illuminant_rgb := Vector3.ONE
var white_degree := 1.0
## Disability glare (look fixes T5b fix round 1): the direct Sun's illuminance at the eye on a plane normal to it (lux;
## transmittance, the clouds' direct beam and the visible fraction applied; adaptation_luminance), the veil of the last
## field measurement (cd/m^2, photopic and scotopic; glare_veil), the Sun's angle from the view axis there (deg) and
## the visibility applied to it (the meter's "seen" share with the Sun in the frame, else the surround's V).
var last_sun_normal_lux := 0.0
var last_veil_cd_m2 := 0.0
var last_veil_scotopic_cd_m2 := 0.0
var last_veil_angle_deg := -1.0
var last_veil_visibility := 0.0
## The glare spread's mean veil over the field at the last measurement (cd/m^2, photopic): part of the veil added to the field.
var last_veil_image_cd_m2 := 0.0
## The meter's Sun mask at the last measurement: [masked weight, its weight that showed the disc].
var last_sun_mask_weights := [0, 0]
## Fidelity flag (look fixes T5b fix round 1): the Sun's disability glare veil on the field (render_check --glare=off:
## none; the disc stays masked).
static var glare_enabled := true
var _sun_light_unit_rgb := Vector3.ONE  ## the direct Sun's colour at the eye (luminance 1): the scotopic veil
var _sun_distance_au := 1.0  ## update: the Sun's distance (the veil's disc size)
var _illuminant_down_rgb := Vector3.ZERO  ## adaptation_luminance: the light on the horizontal plate at the eye (RGB cd/m^2)
var _illuminant_far_rgb := Vector3.ZERO  ## away from the air: the earthshine / Earth-in-view plate (RGB cd/m^2)
## A true luminance (cd/m^2, at the Sun's actual distance) -> scene value, incl. the exposure gain.
var scene_per_cd_m2 := 0.0
## The luminance the eye is adapted to this frame (cd/m^2, EyeAdaptation.adaptation_cd_m2; 0 with the adaptation off):
## the global atmosphere_adaptation_cd_m2, the star threshold's background floor.
var adaptation_cd_m2 := 0.0
## Eclipse sky segments this frame (0: no penumbra on the Earth).
var eclipse_segments := 0
var _eye := EyeAdaptation.new()
## The eye adaptation's two parts at the last update (cd/m^2; adaptation_luminance), for tests and the report.
var last_adaptation_down_cd_m2 := 0.0
var last_adaptation_up_cd_m2 := 0.0
## Away from the Earth: the Moon in view (cd/m^2), the earthshine on a plate at the Moon facing the Earth (lux) and the
## Sun's visible fraction the down part used (0 .. 1).
var last_adaptation_moon_cd_m2 := 0.0
var last_earthshine_lux := 0.0
## The earthshine this frame as the Moon shaders' scene value (global moon_earthshine): a white Lambertian at the Moon
## facing the Earth, DAY_EXPOSURE x earthshine_relative, without the exposure gain (the shaders apply
## atmosphere_exposure_gain to the whole Moon); 0 without a Moon.
var earthshine_scene := 0.0
var last_adaptation_visible_fraction := 1.0
## The Sun's visible fraction at the camera from the lunar terrain's horizon (MoonSunHorizon; world.gd sets it per
## frame, NAN: no terrain or beyond MOON_TERRAIN_SUN_MAX_DISTANCE_M): it replaces the Moon sphere's occultation.
var eye_terrain_sun_fraction := NAN
## The twilight glow at the last update (cd/m^2; 0 with the Sun up or outside the atmosphere).
var last_adaptation_glow_cd_m2 := 0.0
## Night vision at the last update: the scotopic adaptation luminance (scotopic cd/m^2, paced), the rods' scale of the
## post step (EyeAdaptation.rod_scale), the moonlight relative to the TOA Sun at its distance (moonlight_relative; 0
## without a Moon) and whether the moonlight terms were active (MOONLIGHT_MIN_SHARE).
var scotopic_cd_m2 := 0.0
var rod_scale := 0.0
## The suprathreshold factors at the last update (EyeAdaptation.brightness / chroma; 1 by day or with the adaptation
## off): the brightness is in luminance_scale, scene_per_cd_m2 and atmosphere_exposure_gain (not in exposure_gain), the
## chroma in the post step.
var brightness := 1.0
var chroma := 1.0
## The post step's white balance at the last update (EyeAdaptation.white_balance of the TOA Sun's colour; identity with
## the adaptation off or EyeAdaptation.white_balance_enabled false).
var white_balance := Basis.IDENTITY
var moonlight_relative_value := 0.0
var moonlight_active := false
## The unpaced targets of the last adaptation_luminance (photopic and scotopic cd/m^2).
var last_target_cd_m2 := 0.0
var last_target_scotopic_cd_m2 := 0.0
## Night vision's rod step on the HDR scene (attached to the WorldEnvironment's Compositor: world.gd, the GPU check).
var vision_effect := VisionEffect.new()
## The field luminance meter (scene-referred adaptation, Task 14; attached before vision_effect in world.gd's
## Compositor; not in the GPU check's, which keeps the analytic rule).
var adaptation_meter := AdaptationMeter.new()
## The physical glare spread (look fixes T13; GlareSpread): after the transparent pass, before vision_effect (world.gd's
## Compositor). Its image part feeds the meter one frame late; the Sun's veil is drawn per pixel from last_sun_normal_lux.
var glare_effect := GlareSpreadEffect.new()
var glare_add := GlareSpreadEffect.new(true, GlareSpreadEffect.Stage.ADD, glare_effect)
## The field beyond the frame (look fixes T4; AdaptationSurround, fidelity flag AdaptationSurround.enabled): the
## out-of-frame directions within 70 deg of the view axis, from the analytic model (_surround_luminance).
var adaptation_surround := AdaptationSurround.new()
## Set by World every frame before update: the frame's view for the surround ({} = none: the frame alone) and the near
## imagery's albedo profile around the eye (GroundAlbedoNear; empty: the tables' ground albedo).
var adaptation_view := {}
var surround_albedo_profile := PackedVector3Array()
## Fidelity flag (Task 14): the eye adapts to the field it sees (AdaptationMeter, EyeAdaptation.FIELD_MID_GREY); off,
## or until the meter has measured the view: the white-plate rule (adaptation_luminance). The user argument
## --analytic_adaptation switches it off at start (before / after comparisons).
var scene_referred_adaptation := not OS.get_cmdline_user_args().has("--analytic_adaptation")
## Which statistic of the measured field sets the white (with scene_referred_adaptation; eye-adaptation.md):
## MEAN_GREY - the game's rule (unified appearance model, look fixes T13): the centre-weighted log average taken as the
## scene's background, L_a = L_f / field_key (Y_b, EyeAdaptation.FIELD_MID_GREY); bright regions are protected by the
## local adaptation (LocalAdaptation) and the display curve's shoulder, not by the global rule;
## ANCHOR - the field's AdaptationMeter.ANCHOR_PERCENTILE-th percentile taken as a white, L_a = L_anchor /
## EyeAdaptation.ANCHOR_WHITE (field_key not used; user argument --anchored_adaptation);
## PROTECT - rule D, the game's rule 2026-11-07 .. T13: the darker of the two, L_a = max(L_f / field_key, L_anchor /
## EyeAdaptation.ANCHOR_WHITE) (user argument --protect_adaptation).
## Everything downstream (gains, pacing, clamps, fallbacks) is the same.
enum FieldRule { MEAN_GREY, ANCHOR, PROTECT }
var field_rule := _field_rule_from_arguments(OS.get_cmdline_user_args())


func field_rule_name() -> String:
	return ["field", "anchor", "protect"][field_rule]


static func _field_rule_from_arguments(arguments: PackedStringArray) -> FieldRule:
	if arguments.has("--protect_adaptation"):
		return FieldRule.PROTECT
	return FieldRule.ANCHOR if arguments.has("--anchored_adaptation") else FieldRule.MEAN_GREY


## The adaptation luminance a field rule takes from the measured field (L_f) and anchor (L_anchor), cd/m^2 (photopic or
## scotopic alike): MEAN_GREY L_f / key; ANCHOR L_anchor / ANCHOR_WHITE; PROTECT the larger (darker exposure) of the two.
static func field_adaptation_luminance(rule: FieldRule, field_cd_m2: float, anchor_cd_m2: float, key: float) -> float:
	var grey := field_cd_m2 / key
	var white := anchor_cd_m2 / EyeAdaptation.ANCHOR_WHITE
	match rule:
		FieldRule.ANCHOR:
			return white
		FieldRule.PROTECT:
			return maxf(grey, white)
	return grey


## The view is still streaming in (World: terrain nodes loading): a snapped view stays snapped to the field until the
## meter has seen it settled.
var view_loading := false
## The field luminance at the last measurement (cd/m^2, photopic and scotopic; AdaptationMeter.field_log_average) and
## whether this frame's adaptation came from it.
var last_field_cd_m2 := 0.0
var last_field_scotopic_cd_m2 := 0.0
## The anchor at the last measurement (cd/m^2, photopic and scotopic; AdaptationMeter.field_anchor; measured under both rules).
var last_anchor_cd_m2 := 0.0
var last_anchor_scotopic_cd_m2 := 0.0
var adaptation_from_field := false
## [call] Frames without a new field measurement before the analytic rule takes over again (the meter answers every
## ~2-4 frames).
const FIELD_STALE_FRAMES := 30
## --adaptation_log: every this many measurements one log line.
const ADAPTATION_LOG_EVERY := 20
## [call] After a view jump the eye stays snapped to the measured field at least this long (wall time, s): the new
## view's streamed terrain may start loading only some frames after the jump.
const FIELD_SNAP_MIN_S := 1.0
## Checks (render_check --gain_probe): > 0 holds the cone gain at this value (the eye is not advanced; the meter keeps
## measuring), so the field measured at two gains shows whether the measurement feeds back on the gain.
var field_probe_gain := 0.0
var _field_epoch := 0
var _field_serial := 0
var _field_valid := false
var _field_snap := true
var _field_snap_start_ms := 0
var _field_loading := false
var _field_age := 0
var _log_adaptation := OS.get_cmdline_user_args().has("--adaptation_log")
var _moon_direction_fixed := Vector3.ZERO
## This update's inputs of the surround model (_surround_begin): [eye ITRF m, Sun unit ITRF, Sun from the Earth ITRF m,
## d_AU, GCRF -> ITRF, Moon ITRF m (ZERO: none), eye from the Earth GCRF m, Moon from the Earth GCRF m].
var _surround_inputs := []
var _cloud_cache := {}  ## cloud_light_at per frame: [position, direction, probe, frame] -> Vector3
## The direct Sun through the clouds as drawn (World: CloudLayer.direct_sun_transmittance(key, position_itrf_m) -> T or
## NAN): replaces the grid mean's direct beam of the Sun light and the eye's plate where it has a value. Unset (checks):
## the grid mean.
var direct_sun: Callable
## The umbra's light (_update_umbra_light) at the last evaluation: the eclipsed sky's fluence (scalar irradiance, cd/m^2 x
## sr per channel at 1 AU) at UMBRA_HEIGHTS_M (tests).
var umbra_fluence: Array[Vector3] = [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]
var _umbra_tdb_s := -INF
## Tests: compute the umbra's light on the calling thread (one render later it is in place).
var umbra_light_synchronous := false
var _umbra_task_id := -1
var _umbra_result: Array[Vector3] = []
var _tdb_s := NAN
var _fixed_to_gcrf := Basis.IDENTITY
var _earth_center_gcrf_m := Vector3.ZERO

var _core: StarsimCore
## The regolith's BRDF (configure_moon_brdf; static like regolith_reflectance): Hapke through the core, else the
## former Lommel-Seeliger law.
static var moon_brdf_hapke := false
static var _hapke_core: StarsimCore = null
static var _hapke_theta_bar_rad := 0.0
static var _hapke_norm := 0.0
var _task_id := -1
var _result := {}  ## written by the worker task, read after it completed
var _textures: Array[Texture] = []  ## keeps the uploaded textures alive


## Starts loading the tables in the background (config_dir/atmosphere/earth_optics.toml, cache in cache_dir).
func start_loading(core: StarsimCore, config_dir: String, cache_dir: String) -> void:
	if loading or ready:
		return
	_core = core
	ShaderGlobals.set_value(&"atmosphere_enabled", false)
	var cached := FileAccess.file_exists(cache_dir.path_join(CACHE_FILE))
	status_key = "loading.atmosphere_load" if cached else "loading.atmosphere_compute"
	loading = true
	_task_id = WorkerThreadPool.add_task(_load_task.bind(config_dir, cache_dir), false, "atmosphere tables")


## The status line in the active language ("" when idle).
func status_text() -> String:
	return I18n.text(status_key) if status_key != "" else ""


## Main thread: uploads the tables once the worker finished. Returns true when the atmosphere is ready.
func poll() -> bool:
	if loading and WorkerThreadPool.is_task_completed(_task_id):
		WorkerThreadPool.wait_for_task_completion(_task_id)
		loading = false
		_upload(_result)
		_result = {}
	return ready


## Blocks until a running load finished and uploads it (tests).
func wait_until_loaded() -> bool:
	if loading:
		WorkerThreadPool.wait_for_task_completion(_task_id)
		loading = false
		_upload(_result)
		_result = {}
	return ready


## Waits for the worker tasks still running (the table load, the umbra light) before the owner goes away (review M2:
## they write into this object and use the core). Quitting during a first-run precomputation blocks until it ends.
func shutdown() -> void:
	if loading:
		WorkerThreadPool.wait_for_task_completion(_task_id)
		loading = false
		_result = {}
	if _umbra_task_id >= 0:
		WorkerThreadPool.wait_for_task_completion(_umbra_task_id)
		_umbra_task_id = -1
	_set_vision(0.0)  # review M4


## Per frame: the camera, the Earth's centre and orientation (fixed -> GCRF) and the apparent Sun position (GCRF, as
## seen from the Earth's centre), all float64. The camera is differenced against the Earth's centre here, in float64.
## Eye adaptation (eye_adaptation_enabled): adaptation_dt_s is the time the viewer had to adapt since the last frame
## (world.gd: the longer of the frame time and the simulated time step), sun_visible_fraction the part of the solar disc
## the Moon leaves at the camera.
## moon_apparent_gcrf_m: the Moon's apparent position (GCRF, as seen from the Earth's centre) for the eclipse sky;
## Vector3.ZERO: no eclipse. tdb_s: the instant (the Moon's shadow on points other than the camera: the eye adaptation's
## view of the Earth, the umbra's light); NAN: no shadow there. moon_gcrf_m: the Moon's geometric centre (GCRF) for the eye
## adaptation away from the Earth (the lunar ground in view); Vector3.ZERO: none.
func update(camera_gcrf_m: Vector3, earth_center_gcrf_m: Vector3, earth_fixed_to_gcrf: Basis,
		sun_apparent_gcrf_m: Vector3, adaptation_dt_s := 0.0, sun_visible_fraction := 1.0,
		moon_apparent_gcrf_m := Vector3.ZERO, tdb_s := NAN, moon_gcrf_m := Vector3.ZERO) -> void:
	# The Moon shaders' lights need no tables (before them: the vacuum scene units, gain 1).
	var earthshine_relative_value := 0.0
	if moon_gcrf_m != Vector3.ZERO:
		earthshine_relative_value = earthshine_relative(moon_gcrf_m - earth_center_gcrf_m,
				sun_apparent_gcrf_m - earth_center_gcrf_m)
	earthshine_scene = DAY_EXPOSURE * earthshine_relative_value
	var sun_au := (sun_apparent_gcrf_m - earth_center_gcrf_m).length() / ASTRONOMICAL_UNIT_M
	ShaderGlobals.set_value(&"moon_earthshine", earthshine_scene)
	ShaderGlobals.set_value(&"moon_sun_scene", DAY_EXPOSURE / (sun_au * sun_au))
	ShaderGlobals.set_value(&"moon_sun_color", moon_sun_color())
	ShaderGlobals.set_value(&"moon_albedo_tint", moon_albedo_tint())
	ShaderGlobals.set_value(&"moon_grey_tint", moon_grey_tint())
	ShaderGlobals.set_value(&"atmosphere_moon_colour", moon_light_colour())
	ShaderGlobals.set_value(&"moon_regolith_scale",
			regolith_scale(float(parameters.get("toa_solar_illuminance_lux", NOMINAL_TOA_LUX))))
	if not poll():
		_set_vision(0.0)  # no rods left from an earlier state (review M4)
		return
	last_earthshine_lux = earthshine_relative_value * float(parameters["toa_solar_illuminance_lux"])
	_fixed_to_gcrf = earth_fixed_to_gcrf
	_earth_center_gcrf_m = earth_center_gcrf_m
	var gcrf_to_fixed := earth_fixed_to_gcrf.transposed()
	var camera_fixed := gcrf_to_fixed * (camera_gcrf_m - earth_center_gcrf_m)
	camera_local_m = Frames.to_render(camera_fixed)
	var split := TerrainQuadtree.split_float32(camera_local_m)
	var stretched := Vector3(camera_fixed.x, camera_fixed.y, camera_fixed.z * POLAR_STRETCH)
	camera_altitude_m = stretched.length() - float(parameters["bottom_radius_m"])
	var sun_from_earth := sun_apparent_gcrf_m - earth_center_gcrf_m
	var d_au := sun_from_earth.length() / ASTRONOMICAL_UNIT_M
	_sun_distance_au = d_au
	sun_direction_local = Frames.to_render(gcrf_to_fixed * sun_from_earth).normalized()
	var sun_fixed := (gcrf_to_fixed * sun_from_earth).normalized()
	# The Moon's shadow on other points than the camera only while its penumbra touches the atmosphere (review M1:
	# otherwise every _visible_at is 1 and its CSPICE calls are skipped).
	var penumbra := false
	if moon_apparent_gcrf_m != Vector3.ZERO:
		penumbra = penumbra_touches(sun_from_earth, moon_apparent_gcrf_m - earth_center_gcrf_m,
				float(parameters["top_radius_m"]))
	_tdb_s = tdb_s if penumbra else NAN
	# The Moon as a light (night vision): its direction from the Earth's centre (apparent if given, else geometric) and
	# its illuminance; active only while the eye is adapted to a dim scene.
	var moon_light_from_earth := moon_apparent_gcrf_m if moon_apparent_gcrf_m != Vector3.ZERO else moon_gcrf_m
	moonlight_relative_value = 0.0
	_moon_direction_fixed = Vector3.ZERO
	if moon_light_from_earth != Vector3.ZERO:
		moon_light_from_earth -= earth_center_gcrf_m
		moonlight_relative_value = moonlight_relative(moon_light_from_earth, sun_from_earth,
				float(parameters["toa_solar_illuminance_lux"]))
		_moon_direction_fixed = (gcrf_to_fixed * moon_light_from_earth).normalized()
	exposure_gain = 1.0
	adaptation_cd_m2 = 0.0
	scotopic_cd_m2 = 0.0
	rod_scale = 0.0
	brightness = 1.0
	chroma = 1.0
	white_balance = Basis.IDENTITY
	moonlight_active = false
	adaptation_from_field = false
	if eye_adaptation_enabled:
		var moon_fixed := gcrf_to_fixed * (moon_gcrf_m - earth_center_gcrf_m) if moon_gcrf_m != Vector3.ZERO else Vector3.ZERO
		var luminance := adaptation_luminance(camera_fixed, sun_fixed, sun_visible_fraction, d_au, moon_fixed)
		luminance = maxf(luminance, lamp_adaptation_cd_m2)  # lamp-lit surfaces in view (white)
		var scotopic := maxf(last_target_scotopic_cd_m2,
				lamp_adaptation_cd_m2 * Vector3.ONE.dot(EyeAdaptation.SCOTOPIC_WEIGHTS))  # white lamps light the rods too
		if scene_referred_adaptation and AdaptationSurround.enabled:
			_surround_inputs = [camera_fixed, sun_fixed, gcrf_to_fixed * sun_from_earth, d_au, gcrf_to_fixed,
					moon_fixed, camera_gcrf_m - earth_center_gcrf_m, moon_gcrf_m - earth_center_gcrf_m]
			var eye_gcrf: Vector3 = _surround_inputs[6]
			var from_moon_gcrf: Vector3 = eye_gcrf - (_surround_inputs[7] as Vector3)
			var up_gcrf: Vector3 = earth_fixed_to_gcrf * _geodetic_up(camera_fixed)
			if moon_gcrf_m != Vector3.ZERO and near_moon(eye_gcrf.length(), from_moon_gcrf.length()):
				up_gcrf = from_moon_gcrf.normalized()
			adaptation_surround.refresh(Frames.to_render(up_gcrf), _surround_begin, _surround_luminance)
		_take_field_measurement()
		if scene_referred_adaptation and _field_valid:
			# The scene-referred rule (EyeAdaptation.FIELD_MID_GREY): the white the measured field implies takes the
			# place of the white plate. The analytic terms above stay for the report and as the fallback.
			adaptation_from_field = true
			if _field_snap:
				_eye.snap()
			var rule_cd_m2 := field_adaptation_luminance(field_rule, last_field_cd_m2, last_anchor_cd_m2,
					field_key)
			var rule_scotopic_cd_m2 := field_adaptation_luminance(field_rule, last_field_scotopic_cd_m2,
					last_anchor_scotopic_cd_m2, field_key)
			_eye.advance(rule_cd_m2, rule_scotopic_cd_m2, reference_luminance(d_au), adaptation_dt_s)
			# A new view stays snapped while it is still streaming in, and at least FIELD_SNAP_MIN_S after the jump
			# (the streaming may not have started on its first frames).
			_field_snap = _field_snap and (_field_loading
					or Time.get_ticks_msec() - _field_snap_start_ms < int(FIELD_SNAP_MIN_S * 1000.0))
		else:
			_eye.advance(luminance, scotopic, reference_luminance(d_au), adaptation_dt_s)
		exposure_gain = _eye.gain
		adaptation_cd_m2 = _eye.adaptation_cd_m2
		scotopic_cd_m2 = _eye.scotopic_cd_m2
		rod_scale = _eye.rod_scale
		brightness = _eye.brightness
		chroma = _eye.chroma
		# The adopted white (look fixes T5b fix round 1): the field's illuminant with CIECAM02's D, paced.
		field_illuminant_rgb = field_illuminant()
		_eye.advance_white(field_illuminant_rgb, adaptation_dt_s)
		white_degree = _eye.white_degree
		if EyeAdaptation.white_balance_enabled:
			white_balance = _eye.white
		moonlight_active = moonlight_relative_value * reference_luminance(d_au) >= MOONLIGHT_MIN_SHARE * adaptation_cd_m2
		if field_probe_gain > 0.0:
			exposure_gain = field_probe_gain
			brightness = 1.0
	ShaderGlobals.set_value(&"atmosphere_adaptation_cd_m2", adaptation_cd_m2)
	ShaderGlobals.set_value(&"atmosphere_moon_light",
			moonlight_relative_value if moonlight_active else 0.0)
	if _moon_direction_fixed != Vector3.ZERO:
		ShaderGlobals.set_value(&"atmosphere_moon_direction", Frames.to_render(_moon_direction_fixed))
	_set_vision(rod_scale, chroma, white_balance)
	# The display's pre-exposure: the cone gain x the brightness factor (Stevens effect; the rods, relative to the
	# pixel, follow it). The star threshold reads the sky's true luminance back through scene_per_cd_m2: unchanged.
	var display_gain := exposure_gain * brightness
	var base := DAY_EXPOSURE * PI / float(parameters["toa_solar_illuminance_lux"])
	luminance_scale = base * display_gain / (d_au * d_au)
	scene_per_cd_m2 = base * display_gain
	ShaderGlobals.set_value(&"atmosphere_scene_per_cd_m2", scene_per_cd_m2)
	ShaderGlobals.set_value(&"atmosphere_exposure_gain", display_gain)
	# The field meter reads this frame back in true luminance (nothing measured with the adaptation off).
	adaptation_meter.set_frame(scene_per_cd_m2 if eye_adaptation_enabled else 0.0, _field_epoch, view_loading,
			adaptation_view, sun_seen_cd_m2(scene_per_cd_m2))
	# The glare spread: the Sun's direction (render world), its veil's illuminance in scene units (the visibility the meter
	# measured: hidden or enclosed Sun: none), its colour and the mask radius of the disc.
	adaptation_meter.glare_effect = glare_effect
	glare_effect.meter = adaptation_meter
	glare_effect.vision = vision_effect
	for effect in [glare_effect, glare_add]:
		effect.set_frame(scene_per_cd_m2 > 0.0, Frames.to_render(sun_from_earth.normalized()),
				last_sun_normal_lux * last_veil_visibility * scene_per_cd_m2 if glare_enabled else 0.0, _sun_light_unit_rgb,
				cos(AdaptationMeter.SUN_MASK_RADIUS_FACTOR * asin(minf(SUN_RADIUS_M / (ASTRONOMICAL_UNIT_M * d_au), 1.0))),
				eye_adaptation_enabled, display_gain)
	eclipse_segments = 0
	if moon_apparent_gcrf_m != Vector3.ZERO:
		var moon_from_earth := moon_apparent_gcrf_m - earth_center_gcrf_m
		if penumbra:
			eclipse_segments = ECLIPSE_SEGMENTS
			_update_umbra_light(gcrf_to_fixed * sun_from_earth, gcrf_to_fixed * moon_from_earth, sun_fixed)
		ShaderGlobals.set_value(&"atmosphere_moon_position",
				Frames.to_render(gcrf_to_fixed * moon_from_earth))
	ShaderGlobals.set_value(&"atmosphere_sun_distance_m", sun_from_earth.length())
	ShaderGlobals.set_value(&"atmosphere_eclipse_segments", eclipse_segments)
	if eclipse_segments == 0 and _umbra_task_id >= 0:
		# The eclipse ended with a task in flight: its fluence belongs to another epoch, never uploaded (review M5).
		WorkerThreadPool.wait_for_task_completion(_umbra_task_id)
		_umbra_task_id = -1
	if eclipse_segments == 0 and _umbra_tdb_s != -INF:
		_umbra_tdb_s = -INF
		for k in range(3):
			umbra_fluence[k] = Vector3.ZERO
			ShaderGlobals.set_value(StringName("atmosphere_umbra_fluence_%d" % k), Vector3.ZERO)
	# Render world axes are GCRF in render axes; Earth-local = P R^T P^T (the transpose of the body's render basis).
	var world_to_earth := Frames.basis_to_render(earth_fixed_to_gcrf).transposed()
	ShaderGlobals.set_value(&"atmosphere_camera_hi", split[0])
	ShaderGlobals.set_value(&"atmosphere_camera_lo", split[1])
	ShaderGlobals.set_value(&"atmosphere_camera_altitude_m", camera_altitude_m)
	ShaderGlobals.set_value(&"atmosphere_world_to_earth", world_to_earth)
	ShaderGlobals.set_value(&"atmosphere_sun_direction", sun_direction_local)
	ShaderGlobals.set_value(&"atmosphere_luminance_scale", luminance_scale)


## The colour of the Sun's light on the Moon (global moon_sun_color, moon_regolith.gdshaderinc; T5 fix round 1, review
## I3): the Sun above the atmosphere in linear Rec.709 at luminance 1 - the same core spectrum the post step's white
## balance adopts (solar_illuminance_rgb x sun_radiance_to_luminance), so a grey regolith comes out grey. White before
## the tables have loaded.
func moon_sun_color() -> Vector3:
	if not (parameters.has("solar_illuminance_rgb") and parameters.has("sun_radiance_to_luminance")):
		return Vector3.ONE
	var sun := Vector3(parameters["solar_illuminance_rgb"]) * Vector3(parameters["sun_radiance_to_luminance"])
	var luminance := EyeAdaptation.LUMINANCE_WEIGHTS.dot(sun)
	return sun / luminance if luminance > 0.0 else Vector3.ONE


## The field's illuminant (linear Rec.709 at luminance 1; look fixes T5b fix round 1, user 2026-11-08): the light that
## reaches the scene the eye adapts to, estimated physically, not as a grey world of the frame. Of the plate at the eye
## (in the air the horizontal plate, the global illuminance - fix round 2: Sun + sky through the clouds, the Moon + the
## moonlit sky at night, the tower lamps' lit surfaces in their colour; above the air the Sun at normal incidence) and,
## away from the Earth's air, the earthshine at the Moon (the Earth in view elsewhere), the brighter: the Sun above the
## air in space, Sun + sky on the ground, moonlight at night. An enclosure the eye is in (interior_light_colour, the
## cabin) takes over as the surround's visibility V closes: colour = V x outside + (1 - V) x interior. Below
## FIELD_ILLUMINANT_MIN_CD_M2 (a moonless night: nothing to adapt to, the rods see) ZERO: the adopted white is held.
func field_illuminant() -> Vector3:
	if not EyeAdaptation.adopted_white_follows_scene:
		return moon_sun_color()
	var down := _illuminant_down_rgb + lamp_light_colour * lamp_adaptation_cd_m2
	var outside := down if down.dot(LUMINANCE_WEIGHTS) >= _illuminant_far_rgb.dot(LUMINANCE_WEIGHTS) else _illuminant_far_rgb
	var luminance := outside.dot(LUMINANCE_WEIGHTS)
	var colour := outside / luminance if luminance >= FIELD_ILLUMINANT_MIN_CD_M2 else Vector3.ZERO
	if interior_light_colour != Vector3.ZERO:
		if colour == Vector3.ZERO:
			return interior_light_colour
		var v := adaptation_surround.last_visibility if scene_referred_adaptation and AdaptationSurround.enabled else 0.0
		colour = colour * v + interior_light_colour * (1.0 - v)
	return colour


## [S: Stiles 1929; Holladay 1926; CIE 146:2002 "CIE equations for disability glare", the Stiles-Holladay disability
## glare formula L_veil = 10 sum_i E_gl,i / theta_i^2 (L_veil cd/m^2, E_gl lux at the eye in the plane normal to the
## line of sight, theta deg from the line of sight; stated for 1 deg < theta < 30 deg)] The constant 10 of that form.
const GLARE_VEIL_COEFFICIENT := 10.0
## [call] Below this the angle is held (deg): the Sun's own apparent radius at 1 AU (16 arcmin); the form is extended
## outside its 1-30 deg range in both directions, beyond 30 deg it falls with the plane-normal E's cos theta to 0 at 90
## deg (the eye's field: nothing from behind).
const GLARE_MIN_ANGLE_DEG := 0.2666
## [call] A masked sample this fraction of the Sun disc's own (de-gained) luminance or brighter shows the disc (the
## limb darkening 0.4 at the edge; the aureole is >= 1e3 x fainter): AdaptationMeter's "seen" counter.
const SUN_SEEN_SHARE := 0.05
## [call] The plate luminance (cd/m^2) below which the field has no illuminant to adapt to (field_illuminant: the white
## is held): a crescent Moon's plate is ~3e-3, a moonless clear night's ~1e-4 (starlight, not modelled: 0 here).
const FIELD_ILLUMINANT_MIN_CD_M2 := 1.0e-3
## [call] The Sun counters are exact on the pixels (AdaptationMeter, bug fix 2026-11-08: on the sparse grid a sample more or
## less switched the veil's visibility as the Sun moved by a fraction of a pixel). Measured with them: the open Sun's seen
## pixels are 0.25 of the mask at 15 px/deg (0.36 of the disc's share; edge pixels below the threshold), fewer for a smaller
## Sun; a seen share of this fraction of the disc's area counts as the whole Sun visible (saturates: pixel noise at the limb
## does not show), a partly hidden Sun scales down linearly below it.
const SUN_SEEN_FULL_SHARE := 0.15
## [S: sun.gdshader] the Sun disc's scene value is clamped to this largest channel (half-float range).
const SUN_DISC_SCENE_CLAMP := 6.0e4


## The Sun disc's "seen" threshold for the meter (cd/m^2 de-gained): SUN_SEEN_SHARE of the disc's mean luminance at the
## eye (the direct beam's normal illuminance over the disc's solid angle; through clouds and air), or of the clamped
## value the shader draws; 1e30 without a direct Sun (no masked sample shows it).
func sun_seen_cd_m2(scene_per_cd: float) -> float:
	var visible := maxf(last_adaptation_visible_fraction, 1.0e-6)
	var lux := last_sun_normal_lux / visible  # the visible part of the disc keeps its luminance
	if lux <= 0.0 or scene_per_cd <= 0.0:
		return 1.0e30
	var d_m := ASTRONOMICAL_UNIT_M * _sun_distance_au
	var solid_angle := TAU * (1.0 - sqrt(1.0 - pow(SUN_RADIUS_M / d_m, 2.0)))
	var peak := _sun_light_unit_rgb.x if _sun_light_unit_rgb.x > _sun_light_unit_rgb.y else _sun_light_unit_rgb.y
	peak = maxf(peak, _sun_light_unit_rgb.z)
	var clamped := SUN_DISC_SCENE_CLAMP / maxf(peak, 1.0e-6) / scene_per_cd  # the clamp on the largest channel, in luminance
	return SUN_SEEN_SHARE * minf(lux / solid_angle, clamped)


## The Sun's disability glare veil over the field (look fixes T5b fix round 1, user 2026-11-08): [photopic, scotopic
## cd/m^2]. L_veil = GLARE_VEIL_COEFFICIENT x E_gl / theta^2, theta (deg, at least GLARE_MIN_ANGLE_DEG) the Sun's angle
## from the view axis of the view the histograms were measured in (view: AdaptationMeter's, "head" -Z), E_gl =
## last_sun_normal_lux x cos theta (the plane normal to the line of sight; 0 beyond 90 deg). Whether the Sun reaches the
## eye: with its direction in a rendered view, the meter's masked samples that show the disc (sun_weights [mask weight,
## seen weight] / AdaptationMeter.SUN_DISC_SHARE, at most 1); outside the frame (or no masked sample) the surround's
## visibility V (an open view: 1; an enclosure, whose walls the model does not know: 0). The scotopic veil takes the
## direct Sun's colour through SCOTOPIC_WEIGHTS. The veil lies over the whole retinal image, so it is added to the
## field's statistics, L_f + L_veil and L_anchor + L_veil (_take_field_measurement; a black field becomes the veil): looking
## toward the Sun light-adapts the eye whatever the Sun's share of the pixels, inside the frame or just outside it, and
## a veil never darkens the eye. (Added per sample instead, a veil below a lit field would bring the black sky back into
## the field's mean and lower it: 400 km, Sun 70 deg off axis, L_a 2.0e4 -> 6.5e3 in a trial render.)
func glare_veil(view: Dictionary, sun_weights: Array) -> Vector2:
	last_veil_angle_deg = -1.0
	last_veil_visibility = 0.0
	if not view.has("sun") or not view.has("head") or last_sun_normal_lux <= 0.0:
		return Vector2.ZERO
	var head: Basis = view["head"]
	var sun: Vector3 = (view["sun"] as Vector3).normalized()
	var cos_theta := clampf((-head.z.normalized()).dot(sun), -1.0, 1.0)
	last_veil_angle_deg = rad_to_deg(acos(cos_theta))
	if cos_theta <= 0.0:
		return Vector2.ZERO
	var in_frame := false
	var sun_head := head.transposed() * sun
	for eye: Array in view.get("eyes", []):
		if AdaptationSurround.in_view(sun_head, eye[0], eye[1]):
			in_frame = true
			break
	var masked := float(sun_weights[0]) if sun_weights.size() >= 2 else 0.0
	var visibility := adaptation_surround.last_visibility if scene_referred_adaptation and AdaptationSurround.enabled else 0.0
	if in_frame and masked > 0.0:
		visibility = clampf(float(sun_weights[1]) / (masked * AdaptationMeter.SUN_DISC_SHARE * SUN_SEEN_FULL_SHARE), 0.0, 1.0)
	last_veil_visibility = visibility
	var theta := maxf(last_veil_angle_deg, GLARE_MIN_ANGLE_DEG)
	var veil := GLARE_VEIL_COEFFICIENT * last_sun_normal_lux * cos_theta / (theta * theta) * visibility
	return Vector2(veil, veil * maxf(_sun_light_unit_rgb.dot(EyeAdaptation.SCOTOPIC_WEIGHTS), 0.0))


## The regolith colour factor for the albedo texture (global moon_albedo_tint): per channel, the measured mean
## reflectance colour (MOON_REGOLITH_COLOUR_RATIO) at the texture's mean luminance over the texture's mean colour
## (MOON_TEXTURE_MEAN_RGB). The mean of tint x texture is the measured colour at the unchanged mean luminance, so the
## calibrated albedo (MOON_MEAN_TEXTURE_ALBEDO, regolith_scale) and the disc calibration stay; local colour variation
## of the mosaic is kept. ONE with the flag off.
static func moon_albedo_tint() -> Vector3:
	if not moon_colour_correction:
		return Vector3.ONE
	var luminance := EyeAdaptation.LUMINANCE_WEIGHTS.dot(MOON_TEXTURE_MEAN_RGB)
	var target := MOON_REGOLITH_COLOUR_RATIO * (luminance / EyeAdaptation.LUMINANCE_WEIGHTS.dot(MOON_REGOLITH_COLOUR_RATIO))
	return target / MOON_TEXTURE_MEAN_RGB


## The moonlight's colour over the Sun's (global atmosphere_moon_colour; look fixes T5b fix round 1): the regolith's
## mean colour (moon_grey_tint) scaled so that the Sun's light times it keeps the Sun's luminance - the measured
## moonlight (moonlight_relative, Krisciunas & Schaefer) stays the photometric value, it only takes the regolith's red
## slope (full Moon B-V 0.92 against the Sun's 0.65). Multiplies every moonlight term (atm_moon_* in
## atmosphere.gdshaderinc, moon_light_at, adaptation_luminance). ONE with moon_colour_correction off.
func moon_light_colour() -> Vector3:
	var tint := moon_grey_tint()
	if not (parameters.has("solar_illuminance_rgb") and parameters.has("sun_radiance_to_luminance")):
		return tint
	var sun := Vector3(parameters["solar_illuminance_rgb"]) * Vector3(parameters["sun_radiance_to_luminance"])
	var tinted := (sun * tint).dot(LUMINANCE_WEIGHTS)
	return tint * (sun.dot(LUMINANCE_WEIGHTS) / tinted) if tinted > 0.0 else tint


## The adapting field near the Moon (bug hunt 2026-11-08, user: "the Moon looks cream instead of grey"): the field the
## eye adapts to is the light it receives from its surroundings [S: CIE 159:2004 / CIECAM02, the adopted white is that
## of the adapting field; von Kries cone gains set by the field's mean light]. On the Earth that light is daylight off
## a near-neutral world, so the illuminant stands for it (field_illuminant); on and near the Moon the field is regolith
## against a black sky - every bit of light the eye gets (the Sun's disc masked) carries the regolith's red slope, and a
## uniformly coloured field is seen as near grey [S: Helson 1938 / Judd 1940, the Helson-Judd effect: a field of one
## surface colour is perceived toward neutral; Apollo crews reported the regolith grey at high Sun]. The plates'
## colour therefore takes the regolith's (moon_grey_tint, luminance 1) by the share of the eye's surroundings the Moon
## fills: 1 - cos(theta), theta the lunar ground sphere's angular radius (1 on the ground, 0.48 at 300 km, 0.23 at
## 1000 km, ~0 from the Earth) [call: the hemisphere-normalised solid-angle share - view-independent (both VR eyes,
## head turns) and continuous; the full Moon from the Earth needs no term, its night illuminant is moonlight already].
## The display keeps the degree of adaptation D (white_degree), so the regolith stays slightly warm, not white.
## ONE with moon_field_adaptation or moon_colour_correction off.
static func moon_field_tint(eye_from_moon_m: float, ground_radius_m: float) -> Vector3:
	if not (moon_field_adaptation and moon_colour_correction) or eye_from_moon_m <= 0.0:
		return Vector3.ONE
	var s := clampf(ground_radius_m / eye_from_moon_m, 0.0, 1.0)
	var share := 1.0 - sqrt(1.0 - s * s)
	return Vector3.ONE.lerp(moon_grey_tint(), share)


## The same colour factor for the grey Moon users (the sky radiance map's Moon, the eye adaptation's Moon in view),
## normalised to luminance 1 (global moon_grey_tint): the mean regolith's colour without a texture.
static func moon_grey_tint() -> Vector3:
	if not moon_colour_correction:
		return Vector3.ONE
	return MOON_REGOLITH_COLOUR_RATIO / EyeAdaptation.LUMINANCE_WEIGHTS.dot(MOON_REGOLITH_COLOUR_RATIO)


## Eye adaptation's luminance at an Earth-fixed eye (cd/m^2), the larger of two white-plate equivalents (both view-
## direction independent, so identical for both VR eyes and steady under head motion):
## - down: a white Lambertian at the eye lit by the direct Sun at normal incidence (TOA x the transmittance to the Sun)
##   and the sky's horizontal irradiance, (E_sun,n + E_sky) / pi x visible_fraction (the sky's part as well: the CPU
##   irradiance knows no eclipse, inside the Moon's shadow the sky overhead is shadowed with the eye);
## - up [Task 8+9 review I1]: the Earth in view. The visible ground along 13 directions below the eye (the nadir,
##   weight 0.25; six at 0.6 x the Earth disc's angular radius, 0.5; six grazing the limb LIMB_SAMPLE_HEIGHT_M up, or
##   2 deg above the horizontal on the ground, 0.25 - a cosine-weighted cap), each the plate equivalent of what it
##   shows: the ground's illuminance / pi x the view's transmittance plus the in-scattered light on the way
##   (StarsimCore.atmosphere_sky_luminance_to_point), both x the visible fraction at the ground point, or the limb's sky
##   luminance (x the visible fraction at the ray's closest approach); times the view factor of the Earth's cap
##   (R / r)^2. The ground's albedo is not applied: the plate equivalent of a lit ground is the light on it, as for the
##   down part. On the ground this stays below the down part (the light on the ground and the horizon sky do not exceed
##   the Sun + sky at the eye); in orbit an eye in the Moon's umbra or just past orbital sunset adapts to the sunlit
##   Earth or limb it looks at.
## Away from the Earth (the eye above the top of the atmosphere; inside it nothing changes) [Moon-eye fix 2026-10-25]:
## - down: the transmittance is 1 there (or the grazing ray's), the sky irradiance the top boundary's (~0); the visible
##   fraction also takes the Earth's disc in front of the Sun (an eye in the Earth's shadow far out: the transmittance
##   alone is the Sun's centre ray, a sharp edge) [call: where both act, within the Sun's radius of the limb, the
##   penumbra is counted twice - a little too dark there]. For an eye within MOON_EYE_MIN_HEIGHT_M of the lunar mean
##   sphere or below it, the Moon's part is recomputed on its ground sphere (moon_ground_radius: an eye below the mean
##   sphere stands on the terrain; the passed fraction uses the mean sphere, which would hide the Sun from it) unless
##   eye_terrain_sun_fraction is finite (the lunar terrain's horizon, world.gd), which replaces the Moon's part.
## - up: the Earth in view x the part of its disc the Moon leaves (a lunar eye with the Earth below its horizon);
## - Moon in view (_moon_in_view_luminance, with moon_itrf_m): the sunlit and earthshine-lit regolith x its view factor.
## The result is the largest of the three (as down / up: plates facing the Sun, the Earth, the Moon).
## BT.709 luminance of the RGB luminance channels; the tables' 1 AU values / d_AU^2. moon_itrf_m: the Moon's geometric
## centre, Earth-fixed (Vector3.ZERO: none).
func adaptation_luminance(eye_itrf_m: Vector3, sun_direction_itrf: Vector3, visible_fraction: float, d_au: float,
		moon_itrf_m := Vector3.ZERO) -> float:
	var solar: Vector3 = parameters["solar_illuminance_rgb"]
	var sun_factor: Vector3 = parameters["sun_radiance_to_luminance"]
	var transmittance := _core.atmosphere_sun_transmittance(eye_itrf_m, sun_direction_itrf)
	var up := _geodetic_up(eye_itrf_m)
	var sky: Vector3 = _core.atmosphere_illuminance(eye_itrf_m, up, sun_direction_itrf)["sky_lux"]
	var outside := _stretch(eye_itrf_m).length() > float(parameters["top_radius_m"])
	# The Moon's parts above the atmosphere (its occultations, the Moon in view): physical anywhere, negligible far out.
	var lunar := outside and moon_itrf_m != Vector3.ZERO
	var near_moon_eye := lunar and near_moon(eye_itrf_m.length(), (moon_itrf_m - eye_itrf_m).length())
	var sun_itrf_m := sun_direction_itrf * (d_au * ASTRONOMICAL_UNIT_M)
	var visible := visible_fraction
	var moon_ground_m := 0.0
	if lunar:
		moon_ground_m = moon_ground_radius((moon_itrf_m - eye_itrf_m).length())
		visible = sun_visible_near_moon(eye_itrf_m, sun_itrf_m, moon_itrf_m, visible_fraction, eye_terrain_sun_fraction)
	# (the terrain's fraction comes with the Earth's occultation already)
	if outside and not (lunar and is_finite(eye_terrain_sun_fraction)):
		visible *= 1.0 - occulted_fraction(sun_itrf_m - eye_itrf_m, SUN_RADIUS_M, -eye_itrf_m, EARTH_MEAN_RADIUS_M)
	last_adaptation_visible_fraction = visible
	# Clouds (Task 9): the plate's direct Sun through them, the Sun's light diffused through the column (isotropic from
	# above: mu0 x its normal-incidence irradiance on the horizontal) and the sky's diffuse transmittance
	# (StarsimCore.weather_cloud_light: 1, 0, 1 with weather off - the same numbers as before).
	var clouds := cloud_light_at(eye_itrf_m, sun_direction_itrf, "eye")
	var sun_plate: Vector3 = solar * sun_factor * transmittance
	var mu0 := maxf(sun_direction_itrf.dot(up), 0.0)
	var down_rgb := (sun_plate * (clouds.x + clouds.y * mu0) + sky * clouds.z) * visible / (PI * d_au * d_au)
	# The illuminant's colour (field_illuminant; T5b fix round 2, review I1): inside the air the light on the
	# HORIZONTAL plate, the global illuminance E_sun,n mu0 + E_sky (the light on the ground and the deck the scene is
	# made of, as daylight is measured: CIE 015 / Judd et al. 1964, Hernandez-Andres et al. 2001); the luminance's
	# plate above takes the Sun at normal incidence (1 / mu0 more Sun: 4940 K-coloured direct beam over the 10 500 K
	# sky). Above the air (no ground, no sky) the Sun's own colour, the normal plate.
	# Under broken clouds the eye adapts to the scene it sees, lit on average by the cloud field's mean direct beam, not to the
	# light at its own point (look fixes T13: in a cumulus' shadow the eye's plate had the sky alone, B/R 1.44, and the
	# sunlit clouds in view went orange): the colour takes the weather grid's mean cloud light (no shaped probe).
	var clouds_mean := cloud_light_at(eye_itrf_m, sun_direction_itrf)
	var illuminant_rgb := (sun_plate * (clouds_mean.x + clouds_mean.y * mu0) + sky * clouds_mean.z) * visible 			/ (PI * d_au * d_au)
	if not outside:
		illuminant_rgb = (sun_plate * ((clouds_mean.x + clouds_mean.y) * mu0) + sky * clouds_mean.z) * visible 				/ (PI * d_au * d_au)
	# The direct Sun at the eye for its disability glare (glare_veil): normal illuminance, lux.
	var direct := sun_plate * clouds.x
	last_sun_normal_lux = direct.dot(LUMINANCE_WEIGHTS) * visible / (d_au * d_au)
	if direct.dot(LUMINANCE_WEIGHTS) > 0.0:
		_sun_light_unit_rgb = direct / direct.dot(LUMINANCE_WEIGHTS)
	var earth_rgb := _earth_in_view_luminance(eye_itrf_m, sun_direction_itrf) / (d_au * d_au)
	var moon_rgb := Vector3.ZERO
	if lunar:
		earth_rgb *= 1.0 - occulted_fraction(-eye_itrf_m, EARTH_MEAN_RADIUS_M, moon_itrf_m - eye_itrf_m, moon_ground_m)
		# Earthshine on a plate at the Moon facing the Earth: the Lambert-sphere Earth of earthshine_relative, the same
		# value the Moon shaders get (moon_earthshine).
		last_earthshine_lux = earthshine_relative(moon_itrf_m, sun_itrf_m) * float(parameters["toa_solar_illuminance_lux"])
		# The regolith's mean colour (moon_grey_tint, luminance 1) under the solar colour (sunlight and earthshine), at
		# the term's luminance; as the sky shader's Moon draws it (its product is not renormalised).
		var solar_rgb := solar * sun_factor
		moon_rgb = solar_rgb / solar_rgb.dot(LUMINANCE_WEIGHTS) * moon_grey_tint() * _moon_in_view_luminance(eye_itrf_m, moon_itrf_m,
				sun_itrf_m, moon_ground_m, last_earthshine_lux, d_au) * (1.0 - occulted_fraction(moon_itrf_m - eye_itrf_m,
				moon_ground_m, -eye_itrf_m, EARTH_MEAN_RADIUS_M))
	# Moonlight (night vision), only while it can matter (the Moon's white plate at least MOONLIGHT_MIN_SHARE of the
	# Sun's terms): inside the atmosphere the Moon's plate at the eye (direct + the moonlit sky),
	# above it (orbit night side; an eye nearer the Earth than the Moon) the moonlit Earth in view.
	var dim := moonlight_relative_value * reference_luminance(d_au) >= MOONLIGHT_MIN_SHARE * maxf(
			down_rgb.dot(LUMINANCE_WEIGHTS), earth_rgb.dot(LUMINANCE_WEIGHTS))
	var nearer_earth := moon_itrf_m == Vector3.ZERO or (moon_itrf_m - eye_itrf_m).length() > eye_itrf_m.length()
	if dim and _moon_direction_fixed != Vector3.ZERO and moonlight_relative_value > 0.0 and nearer_earth:
		# The moonlight's colour: the Sun's x the regolith's (moon_light_colour; as the shaders' atm_moon_*).
		var moon_scale := moonlight_relative_value / (d_au * d_au) * moon_light_colour()
		if outside:
			earth_rgb += _earth_in_view_luminance(eye_itrf_m, _moon_direction_fixed) * moon_scale
		else:
			var moon_t := _core.atmosphere_sun_transmittance(eye_itrf_m, _moon_direction_fixed)
			var moon_sky: Vector3 = _core.atmosphere_illuminance(eye_itrf_m, up, _moon_direction_fixed)["sky_lux"]
			var moon_clouds := cloud_light_at(eye_itrf_m, _moon_direction_fixed)
			var moon_plate: Vector3 = solar * sun_factor * moon_t
			var moon_mu := maxf(_moon_direction_fixed.dot(up), 0.0)
			down_rgb += (moon_plate * (moon_clouds.x + moon_clouds.y * moon_mu) + moon_sky * moon_clouds.z) \
					* (moon_scale / PI)
			illuminant_rgb += (moon_plate * ((moon_clouds.x + moon_clouds.y) * moon_mu) + moon_sky * moon_clouds.z) \
					* (moon_scale / PI)
	# The field's illuminant (field_illuminant): the light on the horizontal plate at the eye (Sun, sky, Moon), and away from the
	# Earth's air the light of the Earth's side - near the Moon the earthshine (coloured as the Sun, as the shaders draw
	# it), elsewhere the Earth in view (the orbit's night side). The Moon / Earth in view and the glow are what is seen,
	# not what lights the scene, and stay out.
	_illuminant_down_rgb = illuminant_rgb
	_illuminant_far_rgb = Vector3.ZERO
	if outside:
		_illuminant_far_rgb = earth_rgb
		if lunar:
			var solar_unit := solar * sun_factor / (solar * sun_factor).dot(LUMINANCE_WEIGHTS)
			_illuminant_far_rgb = solar_unit * (last_earthshine_lux / PI) * (1.0 - occulted_fraction(-eye_itrf_m,
					EARTH_MEAN_RADIUS_M, moon_itrf_m - eye_itrf_m, moon_ground_m))
			# Near the Moon the field the eye adapts to is the regolith's light (moon_field_tint): both plates take its
			# colour by the share of the field the Moon fills (luminance kept).
			var tint := moon_field_tint((moon_itrf_m - eye_itrf_m).length(), moon_ground_m)
			_illuminant_down_rgb *= tint
			_illuminant_far_rgb *= tint
	# The glow (night vision), GLOW_ADAPTATION_SHARE of the brightest sky: inside the atmosphere with the Sun below the
	# eye's horizon (twilight) or the eye in a solar eclipse (the ring around the umbra; review I2: the eye's own
	# eclipse, not the global penumbra flag - a twilight eye far from the shadow keeps its glow); above it with the Sun
	# behind the Earth (orbital night) and the eye not near the Moon, the brightest limb toward the Sun.
	var glow_rgb := Vector3.ZERO
	var eclipsed := visible < 1.0 and not is_nan(_tdb_s)
	if not outside:
		# (the eye's own eclipse keeps the glow at any Sun elevation)
		var fade := 1.0
		if not eclipsed:
			fade = 1.0 - smoothstep(TWILIGHT_GLOW_FADE_START_DEG, TWILIGHT_GLOW_FADE_END_DEG,
					rad_to_deg(asin(clampf(sun_direction_itrf.dot(up), -1.0, 1.0))))
		if fade > 0.0:
			glow_rgb = _twilight_glow_luminance(eye_itrf_m, up, sun_direction_itrf, eclipsed) * (fade
					* GLOW_ADAPTATION_SHARE / (d_au * d_au))
	elif outside and transmittance == Vector3.ZERO and not near_moon_eye:
		glow_rgb = _limb_glow_luminance(eye_itrf_m, sun_direction_itrf) * (GLOW_ADAPTATION_SHARE / (d_au * d_au))
	last_adaptation_down_cd_m2 = down_rgb.dot(LUMINANCE_WEIGHTS)
	last_adaptation_up_cd_m2 = earth_rgb.dot(LUMINANCE_WEIGHTS)
	last_adaptation_moon_cd_m2 = moon_rgb.dot(LUMINANCE_WEIGHTS)
	last_adaptation_glow_cd_m2 = glow_rgb.dot(LUMINANCE_WEIGHTS)
	# The largest term (as before) sets the photopic adaptation luminance; its colour the scotopic one.
	var chosen := down_rgb
	if last_adaptation_up_cd_m2 > chosen.dot(LUMINANCE_WEIGHTS):
		chosen = earth_rgb
	if last_adaptation_moon_cd_m2 > chosen.dot(LUMINANCE_WEIGHTS):
		chosen = moon_rgb
	if last_adaptation_glow_cd_m2 > chosen.dot(LUMINANCE_WEIGHTS):
		chosen = glow_rgb
	last_target_cd_m2 = chosen.dot(LUMINANCE_WEIGHTS)
	last_target_scotopic_cd_m2 = maxf(chosen.dot(EyeAdaptation.SCOTOPIC_WEIGHTS), 0.0)
	return last_target_cd_m2


## The glow of the eye adaptation (RGB luminance, cd/m^2 at 1 AU): the brightest sky luminance, leaving out directions
## within TWILIGHT_GLOW_MIN_SUN_DEG of the Sun, wherever the head points (view-independent, VR-stable); the caller takes
## GLOW_ADAPTATION_SHARE of it (Ferwerda et al. 1996 sec. 4.4: half the highest visible luminance). By day the white
## plate is the larger anyway (the day look unchanged).
## - Twilight: along the Sun's azimuth at TWILIGHT_GLOW_ELEVATIONS_DEG (the glow over the Sun's horizon is the
##   brightest sky). Accepted limit (review M6): azimuth-only, a brighter sky elsewhere (the Moon's aureole) is not
##   looked for.
## - eclipsed (the eye in a solar eclipse, review I2): the ring around the umbra surrounds the observer, so
##   ECLIPSE_GLOW_AZIMUTHS azimuths at ECLIPSE_GLOW_ELEVATIONS_DEG, each the eclipsed sky (_eclipsed_in_scatter, as the
##   Earth in view) - the same "brightest sky" rule as in twilight.
func _twilight_glow_luminance(eye_itrf_m: Vector3, up: Vector3, sun_direction_itrf: Vector3,
		eclipsed := false) -> Vector3:
	var horizontal := sun_direction_itrf - up * sun_direction_itrf.dot(up)
	if horizontal.length_squared() < 1.0e-12:
		if not eclipsed:
			return Vector3.ZERO
		horizontal = up.cross(Vector3(0, 0, 1) if absf(up.z) < 0.9 else Vector3(1, 0, 0))
	horizontal = horizontal.normalized()
	var side := up.cross(horizontal)
	var directions: Array[Vector3] = []
	if eclipsed:
		for k in range(ECLIPSE_GLOW_AZIMUTHS):
			var azimuth := TAU * k / ECLIPSE_GLOW_AZIMUTHS
			var h := horizontal * cos(azimuth) + side * sin(azimuth)
			for elevation_deg: float in ECLIPSE_GLOW_ELEVATIONS_DEG:
				var e := deg_to_rad(elevation_deg)
				directions.append((h * cos(e) + up * sin(e)).normalized())
	else:
		for elevation_deg: float in TWILIGHT_GLOW_ELEVATIONS_DEG:
			var e := deg_to_rad(elevation_deg)
			directions.append((horizontal * cos(e) + up * sin(e)).normalized())
	var brightest := Vector3.ZERO
	for v in directions:
		var weight := smoothstep(TWILIGHT_GLOW_MIN_SUN_DEG, TWILIGHT_GLOW_FULL_SUN_DEG,
				rad_to_deg(v.angle_to(sun_direction_itrf)))
		if weight == 0.0:
			continue
		var sky: Vector3
		if eclipsed:
			sky = _eclipsed_in_scatter(eye_itrf_m, v, _top_exit_distance(eye_itrf_m, v), false, sun_direction_itrf, [])
		else:
			sky = _core.atmosphere_sky_luminance(eye_itrf_m, v, sun_direction_itrf)["luminance"]
		sky *= weight
		if sky.dot(LUMINANCE_WEIGHTS) > brightest.dot(LUMINANCE_WEIGHTS):
			brightest = sky
	return brightest


## The orbital-night analogue of _twilight_glow_luminance (RGB, cd/m^2 at 1 AU): an eye above the atmosphere with the
## Sun behind the Earth sees the twilight limb toward the Sun; the brightest sky along the limb in the Sun's direction
## at the tangent heights LIMB_GLOW_HEIGHTS_M (above the WGS84 semi-major axis) [call: as the ground's glow, the
## brightest part of the view wherever the head points].
func _limb_glow_luminance(eye_itrf_m: Vector3, sun_direction_itrf: Vector3) -> Vector3:
	var r := eye_itrf_m.length()
	var c := eye_itrf_m / r
	var toward := sun_direction_itrf - c * sun_direction_itrf.dot(c)
	if toward.length_squared() < 1.0e-12:
		return Vector3.ZERO
	toward = toward.normalized()
	var brightest := Vector3.ZERO
	for height: float in LIMB_GLOW_HEIGHTS_M:
		var radius := WGS84_A_M + height
		if radius >= r:
			continue
		var tangent := c * (radius * radius / r) + toward * (radius * sqrt(1.0 - (radius / r) * (radius / r)))
		var v := (tangent - eye_itrf_m).normalized()
		var sky: Vector3 = _core.atmosphere_sky_luminance(eye_itrf_m, v, sun_direction_itrf)["luminance"]
		if sky.dot(LUMINANCE_WEIGHTS) > brightest.dot(LUMINANCE_WEIGHTS):
			brightest = sky
	return brightest


## The Moon's illuminance outside the atmosphere over the TOA Sun's at 1 AU [D]: Krisciunas & Schaefer 1991 (Eq. 8, 9)
## at the phase angle a (Sun - Moon - Earth's centre) with the opposition surge (moon_illuminance_lux), x
## (MOON_MEAN_DISTANCE_M / d)^2, / the tables' TOA illuminance at 1 AU, x the solar disc the Earth leaves at the Moon's
## centre (a lunar eclipse; the umbra's red glow left out). Its users (shaders, the adaptation) multiply it by the 1 AU
## tables / d_AU^2, so the Moon follows the Sun's distance like the real one (review M1). Full Moon at the mean
## distance: 0.313 lux (x 1.35 at a = 0), ~2.4e-6 of the Sun. Vectors from the Earth's centre, m, one frame.
static func moonlight_relative(moon_from_earth_m: Vector3, sun_from_earth_m: Vector3, toa_lux_1au: float) -> float:
	var to_sun := sun_from_earth_m - moon_from_earth_m
	var phase_deg := rad_to_deg(to_sun.angle_to(-moon_from_earth_m))
	var distance := MOON_MEAN_DISTANCE_M / moon_from_earth_m.length()
	var lit := 1.0 - occulted_fraction(to_sun, SUN_RADIUS_M, -moon_from_earth_m, EARTH_MEAN_RADIUS_M)
	return moon_illuminance_lux(phase_deg) * distance * distance / toa_lux_1au * lit


## K&S 1991 Eq. 8, 9 with the opposition surge: the Moon's illuminance (lux) at the mean distance, phase angle in deg.
static func moon_illuminance_lux(phase_deg: float) -> float:
	var magnitude := MOON_MAGNITUDE_FULL + MOON_MAGNITUDE_PER_DEG * phase_deg + MOON_MAGNITUDE_PER_DEG4 * pow(phase_deg, 4.0)
	return pow(10.0, -0.4 * (magnitude + MOON_FOOT_CANDLE_OFFSET)) * LUX_PER_FOOT_CANDLE * moon_opposition(phase_deg)


## The opposition surge factor at the phase angle (deg): MOON_OPPOSITION_PEAK - MOON_OPPOSITION_PER_DEG a below
## MOON_OPPOSITION_END_DEG, else 1.
static func moon_opposition(phase_deg: float) -> float:
	if phase_deg < MOON_OPPOSITION_END_DEG:
		return MOON_OPPOSITION_PEAK - MOON_OPPOSITION_PER_DEG * phase_deg
	return 1.0


## The regolith scale R (the shaders' moon_regolith_scale) [D]: the full-phase disc of the albedo texture's near side
## (MOON_NEAR_SIDE_DISC_ALBEDO; Lommel-Seeliger 1 at mu0 = mu, the phase factor without the surge 1) gives K&S's
## illuminance at the mean distance: R = E_KS(0) / (E_toa / pi x the disc's solid angle x the albedo) - ~0.513, the
## former shaders' 0.8 / 1.5 = 0.533 within 4 %: their excess was the missing phase law (quarter Moon x 4), not the scale.
static func regolith_scale(toa_lux_1au: float) -> float:
	var disc_sr := PI * pow(MOON_RADIUS_M / MOON_MEAN_DISTANCE_M, 2.0)
	var full_lux := moon_illuminance_lux(0.0) / moon_opposition(0.0)
	return full_lux / (toa_lux_1au / PI * disc_sr * MOON_NEAR_SIDE_DISC_ALBEDO)


## The regolith's phase factor f(g) (shaders/moon_regolith.gdshaderinc: keep in sync) from the local phase angle's
## cosine: the opposition surge x K&S's relative phase law / the Lommel-Seeliger sphere's disc-integrated phase function.
static func regolith_phase_factor(cos_g: float) -> float:
	var g_deg := minf(rad_to_deg(acos(clampf(cos_g, -1.0, 1.0))), REGOLITH_PHASE_MAX_DEG)
	var g := deg_to_rad(g_deg)
	var phi := 1.0 if g_deg < 0.01 else 1.0 - sin(0.5 * g) * tan(0.5 * g) * log(1.0 / tan(0.25 * g))
	var relative := pow(10.0, -0.4 * (MOON_MAGNITUDE_PER_DEG * g_deg + MOON_MAGNITUDE_PER_DEG4 * pow(g_deg, 4.0)))
	return moon_opposition(g_deg) * relative / maxf(phi, 1.0e-6)


## The regolith's BRDF for unit vectors (the shaders' regolith_reflectance, moon_regolith.gdshaderinc): Hapke x the
## normalisation (moon_brdf_hapke, the core's I/F) or Lommel-Seeliger x f(g); 0 with the light below the horizon.
## A normal facing away from the eye is taken onto the limb first (the shader's silhouette fix: REGOLITH_MU_MIN).
static func regolith_reflectance(n: Vector3, to_light: Vector3, to_eye: Vector3) -> float:
	var mu := n.dot(to_eye)
	if mu < REGOLITH_MU_MIN:
		n = (n + to_eye * (REGOLITH_MU_MIN - mu)).normalized()
		mu = REGOLITH_MU_MIN
	var mu0 := n.dot(to_light)
	if mu0 <= 0.0:
		return 0.0
	var cos_g := to_light.dot(to_eye)
	if moon_brdf_hapke:
		return _hapke_norm * _hapke_core.moon_hapke_reflectance(mu0, mu, acos(clampf(cos_g, -1.0, 1.0)),
				_hapke_theta_bar_rad)
	return 2.0 * mu0 / (mu0 + mu) * regolith_phase_factor(cos_g)


## Hapke's normalisation [D] (the shaders' moon_hapke_norm): the uniform Hapke sphere's disc-integrated I/F at phase 0
## (disc_full, StarsimCore.load_moon_hapke) scaled to the Lommel-Seeliger law's full-phase disc, MOON_OPPOSITION_PEAK
## (Lommel-Seeliger x f integrates to 1 x the surge at phase 0): the full Moon - the regolith scale's calibration point,
## K&S's illuminance - keeps its brightness with either law, and so does the albedo map's mean.
static func hapke_normalisation(disc_full: float) -> float:
	return MOON_OPPOSITION_PEAK / disc_full


## Chooses the regolith's BRDF (World.moon_brdf: "hapke" or "lommel_seeliger") for the shaders (globals moon_brdf,
## moon_hapke_*) and regolith_reflectance: loads the Hapke parameters through the core (config/moon_surface.toml).
## Without them the former law stays and a warning says so.
static func configure_moon_brdf(core: StarsimCore, config_dir: String, brdf: String) -> void:
	var hapke: Dictionary = core.load_moon_hapke(config_dir)
	var present := bool(hapke.get("present", false))
	if brdf == "hapke" and not present:
		push_warning("Atmosphere: %s - Moon with Lommel-Seeliger" % String(hapke.get("error", "no Hapke parameters")))
	moon_brdf_hapke = brdf == "hapke" and present
	_hapke_core = core if present else null
	if present:
		_hapke_theta_bar_rad = float(hapke["theta_bar_rad"])
		_hapke_norm = hapke_normalisation(float(hapke["disc_reflectance_full"]))
		ShaderGlobals.set_value(&"moon_hapke_albedo_phase", Vector4(float(hapke["w"]), float(hapke["b"]),
				float(hapke["c"]), float(hapke["porosity_k"])))
		ShaderGlobals.set_value(&"moon_hapke_opposition", Vector4(float(hapke["bs0"]), float(hapke["hs"]),
				float(hapke["bc0"]), float(hapke["hc"])))
		ShaderGlobals.set_value(&"moon_hapke_theta_bar_rad", _hapke_theta_bar_rad)
		ShaderGlobals.set_value(&"moon_hapke_norm", _hapke_norm)
	ShaderGlobals.set_value(&"moon_brdf", 1 if moon_brdf_hapke else 0)


## The Moon's light for Godot's lit materials (world.gd's second DirectionalLight3D) at an Earth-fixed point:
## [colour, energy] as sun_light_at, with the Moon's direction and its relative illuminance; [WHITE, 0] while the
## moonlight is inactive (daylight adaptation, no Moon, the tables not loaded).
func moon_light_at(position_itrf_m: Vector3) -> Array:
	if not ready or not moonlight_active or _moon_direction_fixed == Vector3.ZERO:
		return [Color.WHITE, 0.0]
	# No probe key: S is baked toward the Sun only, the Moon takes the grid mean along its own direction.
	return sun_light_at(position_itrf_m, _moon_direction_fixed, moonlight_relative_value, "", moon_light_colour())


## Night vision's state into the shaders and the post step: the rods' scale and the cones' map (white balance and chroma
## factor; VisionEffect: c' = C c + s t (w . c), C = EyeAdaptation.cone_map) and the inverse for the display-referred
## emitters (vision.gdshaderinc, Sherman-Morrison: c = q - u (w . q), q = C^-1 c', u = s C^-1 t / (1 + s w . C^-1 t)).
func _set_vision(scale: float, chroma_factor := 1.0, white := Basis.IDENTITY) -> void:
	var tint := EyeAdaptation.ROD_TINT
	var weights := EyeAdaptation.SCOTOPIC_WEIGHTS
	var cone := EyeAdaptation.cone_map(chroma_factor, white)
	var cone_inverse := cone.inverse()
	var inverse := Vector3.ZERO
	if scale > 0.0:
		var tint_q := cone_inverse * tint
		inverse = tint_q * (scale / (1.0 + scale * weights.dot(tint_q)))
	ShaderGlobals.set_value(&"vision_rod_weights", weights)
	ShaderGlobals.set_value(&"vision_inverse_tint", inverse)
	ShaderGlobals.set_value(&"vision_cone_inverse", cone_inverse)
	vision_effect.set_rods(scale, cone)


## Near the Moon: its disc larger than the Earth's (review I1: an eye in LEO is not near it and keeps the limb glow).
static func near_moon(eye_from_earth_m: float, eye_from_moon_m: float) -> bool:
	return MOON_RADIUS_M * eye_from_earth_m > EARTH_MEAN_RADIUS_M * eye_from_moon_m


## [call] The lunar ground sphere of the eye adaptation (m about the Moon's centre) for an eye this far from it: the
## mean sphere, or MOON_EYE_MIN_HEIGHT_M below the eye when the eye stands below it (terrain under the mean radius).
static func moon_ground_radius(eye_distance_m: float) -> float:
	return minf(MOON_RADIUS_M, eye_distance_m - MOON_EYE_MIN_HEIGHT_M)


## Whether the lunar terrain's horizon (MoonSunHorizon) decides the Sun at a point: within
## MOON_TERRAIN_SUN_MAX_DISTANCE_M of the Moon's centre (any height; terrain above the mean sphere hides the Sun from
## its own ground, which the mean-sphere fraction cannot know).
static func moon_terrain_sun_applies(point_m: Vector3, moon_m: Vector3) -> bool:
	return (moon_m - point_m).length() < MOON_TERRAIN_SUN_MAX_DISTANCE_M


## The Sun's visible fraction at a point near the Moon, the one combination of the vehicles' light (world.gd) and the eye
## adaptation: terrain_fraction (MoonSunHorizon.sun_visible_fraction; NAN: none) x the Earth's disc in front of the Sun
## (the terrain knows only the Moon), else moon_visible_fraction. Vectors Earth-centred, any one frame, m.
static func sun_visible_near_moon(point_m: Vector3, sun_m: Vector3, moon_m: Vector3, mean_sphere_visible: float,
		terrain_fraction: float) -> float:
	if not is_finite(terrain_fraction):
		return moon_visible_fraction(point_m, sun_m, moon_m, mean_sphere_visible)
	return terrain_fraction * (1.0 - occulted_fraction(sun_m - point_m, SUN_RADIUS_M, -point_m, EARTH_MEAN_RADIUS_M))


## The Sun's visible fraction past the Moon at a point (vectors in any one frame and origin, m) for the eye adaptation
## and the vehicles' Sun light (world.gd): mean_sphere_visible is the core's (StarsimCore.solar_eclipse_fraction_gcrf,
## the Moon's mean sphere); a point within MOON_EYE_MIN_HEIGHT_M of that sphere or below it - on lunar terrain under the
## mean radius, where the mean sphere would hide the Sun - gets it on its own ground sphere (moon_ground_radius).
## Near the Moon the callers use sun_visible_near_moon (the terrain's horizon replaces this).
static func moon_visible_fraction(point_m: Vector3, sun_m: Vector3, moon_m: Vector3, mean_sphere_visible: float) -> float:
	var ground_m := moon_ground_radius((moon_m - point_m).length())
	if ground_m >= MOON_RADIUS_M:
		return mean_sphere_visible
	return 1.0 - occulted_fraction(sun_m - point_m, SUN_RADIUS_M, moon_m - point_m, ground_m)


## Earthshine at the Moon (its centre; the near side's ground is 0.5 % closer, left out): the illuminance on a plate
## facing the Earth over the top-of-atmosphere solar illuminance at 1 AU [D]. The Earth a Lambertian sphere of
## EARTH_BOND_ALBEDO (geometric albedo LAMBERT_GEOMETRIC_PER_BOND x A) at the phase angle a (Sun - Earth - Moon):
## p (R / d)^2 (sin a + (pi - a) cos a) / pi / d_AU^2 - pi x A x the Lambert disc's plate luminance, the quantity the eye
## adaptation's Earth in view approximates (its 13-direction cap reads 0.8 x at full and ~0.4 x at half Earth from the
## Moon, hence the closed form here). Full Earth at 384 400 km and 1 AU: 5.6e-5 (7.2 lux; measured estimates of the full
## earthshine run ~7-15 lux with the Earth's real, non-Lambertian phase curve). Clouds, oceans' glint, city lights: not
## modelled. Vectors from the Earth's centre, m, one frame.
static func earthshine_relative(moon_from_earth_m: Vector3, sun_from_earth_m: Vector3) -> float:
	var d_au := sun_from_earth_m.length() / ASTRONOMICAL_UNIT_M
	var a := moon_from_earth_m.angle_to(sun_from_earth_m)
	var phase := (sin(a) + (PI - a) * cos(a)) / PI
	var view := EARTH_MEAN_RADIUS_M / moon_from_earth_m.length()
	return LAMBERT_GEOMETRIC_PER_BOND * EARTH_BOND_ALBEDO * view * view * phase / (d_au * d_au)


## The Moon in view (cd/m^2 at the true Sun distance): the regolith's luminance as the Moon shaders draw it, averaged
## cosine-weighted over the lunar cap below the eye (MOON_VIEW_RINGS rings uniform in sin^2 of the nadir angle, each
## MOON_VIEW_AZIMUTHS directions), times the cap's view factor (R / r)^2 - the analogue of _earth_in_view_luminance.
## Both lights through the one regolith reflectance (regolith_reflectance, moon_regolith.gdshaderinc; review I3):
## albedo x regolith_scale x E / pi x LS x f(g), E the Sun's (x the solar disc the Earth leaves at the ground point: a
## lunar eclipse; the reddened umbra glow of the shaders is left out) and the earthshine's (earthshine_relative).
## albedo: MOON_MEAN_TEXTURE_ALBEDO [call: the map's mean, not the texel under the eye]. Cast shadows: none (as the
## shaders). Vectors Earth-fixed, m.
func _moon_in_view_luminance(eye_itrf_m: Vector3, moon_itrf_m: Vector3, sun_itrf_m: Vector3, radius: float,
		earthshine_lux: float, d_au: float) -> float:
	var eye := eye_itrf_m - moon_itrf_m  # from the Moon's centre
	var r := eye.length()
	var nadir := -eye / r
	var east := nadir.cross(Vector3(0, 0, 1) if absf(nadir.z) < 0.9 else Vector3(1, 0, 0)).normalized()
	var north := nadir.cross(east)
	var sin_cap := radius / r
	var regolith := MOON_MEAN_TEXTURE_ALBEDO * regolith_scale(float(parameters["toa_solar_illuminance_lux"]))
	var sun_scale := regolith * reference_luminance(d_au)
	var earth_scale := regolith * earthshine_lux / PI
	var total := 0.0
	for ring in range(MOON_VIEW_RINGS):
		var sin_theta := sin_cap * sqrt((ring + 0.5) / MOON_VIEW_RINGS)
		var cos_theta := sqrt(maxf(1.0 - sin_theta * sin_theta, 0.0))
		for k in range(MOON_VIEW_AZIMUTHS):
			var azimuth := TAU * (k + 0.5 * (ring % 2)) / MOON_VIEW_AZIMUTHS
			var v := nadir * cos_theta + (east * cos(azimuth) + north * sin(azimuth)) * sin_theta
			var along := eye.dot(v)
			var hit := eye + v * (-along - sqrt(maxf(along * along - (r * r - radius * radius), 0.0)))
			var n := hit.normalized()
			var ground_itrf := moon_itrf_m + hit
			var sunlit := regolith_reflectance(n, (sun_itrf_m - ground_itrf).normalized(), -v)
			var luminance := 0.0
			if sunlit > 0.0:
				luminance += sun_scale * sunlit * (1.0 - occulted_fraction(sun_itrf_m - ground_itrf, SUN_RADIUS_M,
						-ground_itrf, EARTH_MEAN_RADIUS_M))
			luminance += earth_scale * regolith_reflectance(n, -ground_itrf.normalized(), -v)
			total += luminance
	return total / (MOON_VIEW_RINGS * MOON_VIEW_AZIMUTHS) * sin_cap * sin_cap


## Fraction of a light source's disc hidden by an occluding sphere, seen from the observer (vectors from it, m). Mirror
## of core/src/environment/eclipse.cpp (Montenbruck & Gill 3.85-3.90) and shaders/eclipse.gdshaderinc: keep in sync.
static func occulted_fraction(to_source: Vector3, source_radius: float, to_occluder: Vector3,
		occluder_radius: float) -> float:
	var source_distance := to_source.length()
	var occluder_distance := to_occluder.length()
	if occluder_distance <= occluder_radius:
		return 1.0
	if occluder_distance > source_distance:
		return 0.0
	var a := asin(minf(1.0, source_radius / source_distance))
	var b := asin(occluder_radius / occluder_distance)
	var c := atan2(to_source.cross(to_occluder).length(), to_source.dot(to_occluder))
	if c >= a + b:
		return 0.0
	if c <= b - a:
		return 1.0
	if c <= a - b:
		return (b * b) / (a * a)
	var x := (c * c + a * a - b * b) / (2.0 * c)
	var y := sqrt(maxf(0.0, a * a - x * x))
	var area := a * a * acos(clampf(x / a, -1.0, 1.0)) + b * b * acos(clampf((c - x) / b, -1.0, 1.0)) - c * y
	return clampf(area / (PI * a * a), 0.0, 1.0)


## The "up" part of adaptation_luminance (RGB luminance channels, cd/m^2 at 1 AU); sun_direction_itrf: the light
## (the Sun, or the Moon for the moonlit Earth: then x its relative illuminance by the caller).
func _earth_in_view_luminance(eye_itrf_m: Vector3, sun_direction_itrf: Vector3) -> Vector3:
	var radius := float(parameters["bottom_radius_m"])
	var eye := _stretch(eye_itrf_m)  # the model space: the ground a sphere of radius `radius`
	var r := eye.length()
	var nadir := -eye / r
	var east := nadir.cross(Vector3(0, 0, 1) if absf(nadir.z) < 0.9 else Vector3(1, 0, 0)).normalized()
	var north := nadir.cross(east)
	var disc := asin(clampf(radius / r, 0.0, 1.0))  # the Earth disc's angular radius about the nadir
	var limb_radius := radius + LIMB_SAMPLE_HEIGHT_M
	var limb := asin(limb_radius / r) if r > limb_radius else PI * 0.5 + deg_to_rad(2.0)
	var total := _view_luminance(eye_itrf_m, eye, nadir, radius, sun_direction_itrf) * 0.25
	for k in range(6):
		var azimuth := TAU * k / 6.0
		var side := east * cos(azimuth) + north * sin(azimuth)
		for ring: Array in [[0.6 * disc, 0.5 / 6.0], [limb, 0.25 / 6.0]]:
			var angle: float = ring[0]
			var v := (nadir * cos(angle) + side * sin(angle)).normalized()
			total += _view_luminance(eye_itrf_m, eye, v, radius, sun_direction_itrf) * float(ring[1])
	return total * (radius / r) * (radius / r)


## [call] The surround model's eye stands at least this far above its model ground (the ellipsoid; the eye on terrain
## below it, Starbase ~-20 m, would otherwise sit inside the model sphere): a person's eye height.
const SURROUND_EYE_MIN_HEIGHT_M := 2.0
## [E] The near imagery's albedo profile fades to the tables' ground albedo between these horizontal distances (m), as the
## sky cube's (sky.gdshader radiance_local_albedo_near_m / _far_m defaults, GroundAlbedoNear.face_albedo).
const SURROUND_ALBEDO_FADE_NEAR_M := 1500.0
const SURROUND_ALBEDO_FADE_FAR_M := 6000.0


## The surround model's state for one store (AdaptationSurround.refresh): this update's geometry, the ground's albedo
## and the sky cube's cloud factors at the eye (cloud_radiance_factors: the light under the clouds over the clear sky's,
## sky and ground halves; 1 without weather).
func _surround_begin() -> Dictionary:
	var eye_fixed: Vector3 = _surround_inputs[0]
	var sun_fixed: Vector3 = _surround_inputs[1]
	var moon_fixed: Vector3 = _surround_inputs[5]
	var eye_gcrf: Vector3 = _surround_inputs[6]
	var moon_gcrf: Vector3 = _surround_inputs[7]
	var state := {"eye": eye_fixed, "sun": sun_fixed, "sun_m": _surround_inputs[2], "d_au": _surround_inputs[3],
			"to_fixed": _surround_inputs[4], "profile": surround_albedo_profile,
			"albedo": Vector3.ONE * float(parameters["ground_albedo"]), "near_moon": false}
	if moon_fixed != Vector3.ZERO and near_moon(eye_gcrf.length(), (eye_gcrf - moon_gcrf).length()):
		state["near_moon"] = true
		state["moon"] = moon_fixed
		var regolith := MOON_MEAN_TEXTURE_ALBEDO * regolith_scale(float(parameters["toa_solar_illuminance_lux"]))
		state["sun_scale"] = regolith * reference_luminance(float(_surround_inputs[3]))
		state["earth_scale"] = regolith * last_earthshine_lux / PI
		return state
	var radius := float(parameters["bottom_radius_m"])
	var model := _stretch(eye_fixed)
	model = model.normalized() * maxf(model.length(), radius + SURROUND_EYE_MIN_HEIGHT_M)
	state["model_eye"] = model
	state["eye"] = _unstretch(model)
	var factors := cloud_radiance_factors(_unstretch(model), sun_fixed)
	state["cloud_sky"] = factors[0]
	state["cloud_ground"] = factors[1]
	var up := _geodetic_up(_unstretch(model))
	state["east"] = Vector3(0, 0, 1).cross(up).normalized()
	state["north"] = up.cross(state["east"])
	return state


## What an open view shows along a render-world direction (RGB luminance channels, cd/m^2 at the true Sun distance), the
## analytic content of the sky cube (sky.gdshader) for AdaptationSurround: from the Earth's air or space the sky along the
## ray (core sky luminance, the Moon's penumbra segmented) or the model ground (lit by the Sun and the sky, through the
## air; _view_luminance with the ground's albedo: the near imagery's profile by horizontal distance and azimuth, faded to
## the tables' ground albedo, as the cube), times the cube's cloud factor of that half; near the Moon the regolith on the
## eye's ground sphere as _moon_in_view_luminance draws it (both lights, grey), the sky black (the Earth's disc and the
## stars left out). Not modelled [call]: structures, vehicles, terrain relief, the drawn clouds' shapes, city lights.
func _surround_luminance(direction_render: Vector3, state: Dictionary) -> Vector3:
	var v_fixed: Vector3 = ((state["to_fixed"] as Basis) * Frames.from_render(direction_render)).normalized()
	if state["near_moon"]:
		return Vector3.ONE * _surround_regolith(v_fixed, state)
	var radius := float(parameters["bottom_radius_m"])
	var eye: Vector3 = state["model_eye"]
	var eye_fixed: Vector3 = state["eye"]
	var v := _stretch(v_fixed).normalized()
	var along := eye.dot(v)
	var discriminant := along * along - (eye.length_squared() - radius * radius)
	var ground := along < 0.0 and discriminant >= 0.0
	var albedo: Vector3 = state["albedo"]
	var profile: PackedVector3Array = state["profile"]
	if ground and profile.size() == GroundAlbedoNear.profile_size():
		var offset := _unstretch(eye + v * (-along - sqrt(discriminant))) - eye_fixed
		var east: Vector3 = state["east"]
		var north: Vector3 = state["north"]
		var horizontal := Vector2(offset.dot(east), offset.dot(north))
		var r_h := horizontal.length()
		albedo = GroundAlbedoNear.profile_at(profile, r_h, atan2(horizontal.x, horizontal.y)).lerp(albedo,
				smoothstep(SURROUND_ALBEDO_FADE_NEAR_M, SURROUND_ALBEDO_FADE_FAR_M, r_h))
	var seen := _view_luminance(eye_fixed, eye, v, radius, state["sun"], albedo)
	seen *= state["cloud_ground"] if ground else state["cloud_sky"]
	var d_au := float(state["d_au"])
	return seen / (d_au * d_au)


## The regolith luminance (cd/m^2) along an Earth-fixed unit direction from an eye near the Moon (0: the ray misses the
## eye's lunar ground sphere, moon_ground_radius).
func _surround_regolith(v: Vector3, state: Dictionary) -> float:
	var moon: Vector3 = state["moon"]
	var eye: Vector3 = (state["eye"] as Vector3) - moon
	var r := eye.length()
	var radius := moon_ground_radius(r)
	var along := eye.dot(v)
	var discriminant := along * along - (r * r - radius * radius)
	if along >= 0.0 or discriminant < 0.0:
		return 0.0
	var hit := eye + v * (-along - sqrt(discriminant))
	var n := hit.normalized()
	var ground := moon + hit
	var sun_m: Vector3 = state["sun_m"]
	var luminance := 0.0
	var sunlit := regolith_reflectance(n, (sun_m - ground).normalized(), -v)
	if sunlit > 0.0:
		luminance += float(state["sun_scale"]) * sunlit * (1.0 - occulted_fraction(sun_m - ground, SUN_RADIUS_M,
				-ground, EARTH_MEAN_RADIUS_M))
	luminance += float(state["earth_scale"]) * regolith_reflectance(n, -ground.normalized(), -v)
	return luminance


## One view direction of _earth_in_view_luminance (model-space unit vector v from the model-space eye), RGB.
## albedo: the ground's (RGB; ONE: the white plate's equivalent of the adaptation's up part).
func _view_luminance(eye_itrf_m: Vector3, eye: Vector3, v: Vector3, radius: float, sun_itrf: Vector3,
		albedo := Vector3.ONE) -> Vector3:
	var along := eye.dot(v)
	var discriminant := along * along - (eye.length_squared() - radius * radius)
	if along < 0.0 and discriminant >= 0.0:
		var hit := _unstretch(eye + v * (-along - sqrt(discriminant)))
		var lux: Dictionary = _core.atmosphere_illuminance(hit, _geodetic_up(hit), sun_itrf)
		var to_point: Dictionary = _core.atmosphere_sky_luminance_to_point(eye_itrf_m, hit, sun_itrf)
		var t: Vector3 = to_point["transmittance"]
		var lit: Vector3 = lux["sun_lux"] + lux["sky_lux"]
		var seen: Vector3 = lit * albedo / PI * t + to_point["luminance"]
		return seen * _visible_at(hit)
	var view := _unstretch(v).normalized()
	if not is_nan(_tdb_s):
		# The Moon's penumbra on the atmosphere: the sky along the ray segmented and weighted by the Sun's visible
		# fraction (as _eclipsed_sky_fluence). One factor at the ray's closest approach - behind a ground eye looking
		# up - let the uneclipsed horizon sky beyond the umbra set the eye in totality at ~56 cd/m^2 (the real
		# totality sky: ~1-16), which hid the magnitude-1 stars once the stars read the adaptation (final review I1).
		var exit := _top_exit_distance(eye_itrf_m, view)
		if exit > 0.0:
			return _eclipsed_in_scatter(eye_itrf_m, view, exit, false, sun_itrf, [])
	var sky: Vector3 = _core.atmosphere_sky_luminance(eye_itrf_m, view, sun_itrf)["luminance"]
	return sky * _visible_at(_unstretch(eye - v * along))  # the ray's closest approach


## Distance (m) along the Earth-fixed unit ray from the point to where it leaves the top of the atmosphere (the
## sphere of _eclipsed_sky_fluence: the model's top height above the WGS84 semi-major axis); 0 if it never crosses it.
func _top_exit_distance(point_itrf_m: Vector3, v: Vector3) -> float:
	var top := float(parameters["top_radius_m"]) - float(parameters["bottom_radius_m"]) + WGS84_A_M
	var along := point_itrf_m.dot(v)
	var discriminant := along * along - (point_itrf_m.length_squared() - top * top)
	return maxf(-along + sqrt(discriminant), 0.0) if discriminant >= 0.0 else 0.0


## The in-scattered sky along the ray from the point (Earth-fixed unit v) up to d_end (cd/m^2 per channel at 1 AU) with
## the Moon's shadow: UMBRA_SEGMENTS segments at d_end (i/n)^2, each segment's in-scatter (differences of
## StarsimCore.atmosphere_sky_luminance_to_point; the last one to the sky's end unless the ray ends on the ground)
## weighted by the Sun's visible fraction at its midpoint. instant: see _visible_at.
func _eclipsed_in_scatter(point_itrf_m: Vector3, v: Vector3, d_end: float, ends_on_ground: bool, sun_itrf: Vector3,
		instant: Array) -> Vector3:
	var previous := Vector3.ZERO
	var eclipsed := Vector3.ZERO
	for i in range(1, UMBRA_SEGMENTS + 1):
		var f := float(i) / UMBRA_SEGMENTS
		var cumulative: Vector3
		if i < UMBRA_SEGMENTS or ends_on_ground:
			cumulative = _core.atmosphere_sky_luminance_to_point(point_itrf_m, point_itrf_m + v * (d_end * f * f),
					sun_itrf)["luminance"]
		else:
			cumulative = _core.atmosphere_sky_luminance(point_itrf_m, v, sun_itrf)["luminance"]
		var f_mid := (float(i) - 0.5) / UMBRA_SEGMENTS
		eclipsed += (cumulative - previous) * _visible_at(point_itrf_m + v * (d_end * f_mid * f_mid), instant)
		previous = cumulative
	return eclipsed


## The Sun's unocculted fraction (Moon) at an Earth-fixed point at this update's instant (1 without one); instant:
## [tdb_s, fixed_to_gcrf, earth_center_gcrf_m] captured for a worker, [] = this update's.
func _visible_at(point_itrf_m: Vector3, instant := []) -> float:
	var t: float = _tdb_s if instant.is_empty() else instant[0]
	if is_nan(t):
		return 1.0
	var rotation: Basis = _fixed_to_gcrf if instant.is_empty() else instant[1]
	var center: Vector3 = _earth_center_gcrf_m if instant.is_empty() else instant[2]
	return 1.0 - _core.solar_eclipse_fraction_gcrf(rotation * point_itrf_m + center, t)


## The light in the umbra air (atmosphere_umbra_fluence_*, the second-order source of the eclipse sky,
## atmosphere.gdshaderinc): at the ground point of the umbra axis (or the surface point nearest to it) and
## UMBRA_HEIGHTS_M above, the fluence (scalar irradiance) of the eclipsed sky - its luminance along the zenith and four
## azimuths on each ring of UMBRA_ELEVATIONS_DEG (UMBRA_SEGMENTS segments at d (i/n)^2, each weighted by the visible fraction at its midpoint; the
## segments' in-scatter as differences of StarsimCore.atmosphere_sky_luminance_to_point), times the solid angles of the
## elevation bands (UMBRA_BAND_EDGES_DEG); above the ground also the lower hemisphere (the air below and the lit ground
## beyond the umbra). The low ring dominates: in totality the light reaches the umbra
## nearly horizontally from the sunlit air ~100 km away. Once per simulated UMBRA_UPDATE_S (the umbra moves ~1 km/s).
func _update_umbra_light(sun_fixed_m: Vector3, moon_fixed_m: Vector3, sun_direction_itrf: Vector3) -> void:
	if _umbra_task_id >= 0:
		if not WorkerThreadPool.is_task_completed(_umbra_task_id):
			return
		WorkerThreadPool.wait_for_task_completion(_umbra_task_id)
		_umbra_task_id = -1
		_upload_umbra_light()
	if is_nan(_tdb_s) or absf(_tdb_s - _umbra_tdb_s) < UMBRA_UPDATE_S:
		return
	_umbra_tdb_s = _tdb_s
	var axis := (moon_fixed_m - sun_fixed_m).normalized()
	var b := moon_fixed_m.dot(axis)
	var c := moon_fixed_m.length_squared() - WGS84_A_M * WGS84_A_M
	var ground: Vector3
	if b * b - c >= 0.0:
		ground = moon_fixed_m + axis * (-b - sqrt(b * b - c))
	else:  # the umbra misses the Earth: the surface point nearest to the axis
		ground = (moon_fixed_m - axis * b).normalized() * WGS84_A_M
	# ~10 ms of core calls: on a worker (the core's lookups are const, CSPICE is serialised by the core), uploaded on a
	# later frame; synchronous for tests that render one frame later (umbra_light_synchronous).
	var instant := [_tdb_s, _fixed_to_gcrf, _earth_center_gcrf_m]
	var job := _compute_umbra_light.bind(ground, sun_direction_itrf, instant)
	if umbra_light_synchronous:
		job.call()
		_upload_umbra_light()
	else:
		_umbra_task_id = WorkerThreadPool.add_task(job, false, "umbra light")


## Worker part of _update_umbra_light: the fluences at UMBRA_HEIGHTS_M into _umbra_result.
func _compute_umbra_light(ground: Vector3, sun_direction_itrf: Vector3, instant: Array) -> void:
	var up := _geodetic_up(ground)
	var result: Array[Vector3] = []
	for k in range(3):
		var height: float = UMBRA_HEIGHTS_M[k]
		result.append(_eclipsed_sky_fluence(ground + up * height, up, sun_direction_itrf, ground.length(), instant))
	_umbra_result = result


## Main thread: the computed fluences into the shader globals.
func _upload_umbra_light() -> void:
	var heights := Vector3.ZERO
	var model_offset := WGS84_A_M - float(parameters["bottom_radius_m"])  # ellipsoid above the model ground
	for k in range(3):
		umbra_fluence[k] = _umbra_result[k]
		heights[k] = float(UMBRA_HEIGHTS_M[k]) + model_offset
		ShaderGlobals.set_value(StringName("atmosphere_umbra_fluence_%d" % k), _umbra_result[k])
	ShaderGlobals.set_value(&"atmosphere_umbra_heights_m", heights)


## The eclipsed sky's fluence at an Earth-fixed point (cd/m^2 x sr per channel at 1 AU; see _update_umbra_light);
## ground_radius_m: the Earth's radius under the point. Points above the ground also take the lower hemisphere (the
## mirrored rings): the air down to the ground and the ground itself (the tables' ground_albedo, lit by the Sun and the
## sky, x its visible fraction).
func _eclipsed_sky_fluence(point_itrf_m: Vector3, up: Vector3, sun_itrf: Vector3, ground_radius_m: float,
		instant: Array) -> Vector3:
	var east := up.cross(Vector3(0, 0, 1) if absf(up.z) < 0.9 else Vector3(1, 0, 0)).normalized()
	var north := up.cross(east)
	var top := float(parameters["top_radius_m"]) - float(parameters["bottom_radius_m"]) + WGS84_A_M
	var fluence := Vector3.ZERO
	var signs := [1.0, -1.0] if point_itrf_m.length() - ground_radius_m > 10.0 else [1.0]
	for hemisphere: float in signs:
		for k in range(1 + 4 * (UMBRA_ELEVATIONS_DEG.size() - 1)):
			var ring := 0 if k == 0 else 1 + (k - 1) / 4
			var elevation: float = UMBRA_ELEVATIONS_DEG[ring] * hemisphere
			var upper: float = 90.0 if ring == 0 else UMBRA_BAND_EDGES_DEG[ring - 1]
			var solid_angle := TAU * (sin(deg_to_rad(upper)) - sin(deg_to_rad(UMBRA_BAND_EDGES_DEG[ring])))
			if ring > 0:
				solid_angle /= 4.0
			var azimuth := (k % 4) * PI * 0.5
			var v := up * hemisphere
			if k > 0:
				v = (up * sin(deg_to_rad(elevation)) + (north * cos(azimuth) + east * sin(azimuth))
						* cos(deg_to_rad(elevation))).normalized()
			var along := point_itrf_m.dot(v)
			var d_end := -along + sqrt(along * along - (point_itrf_m.length_squared() - top * top))
			var ground_discriminant := along * along - (point_itrf_m.length_squared() - ground_radius_m * ground_radius_m)
			var hits_ground := along < 0.0 and ground_discriminant >= 0.0
			if hits_ground:
				d_end = -along - sqrt(ground_discriminant)
			var eclipsed := _eclipsed_in_scatter(point_itrf_m, v, d_end, hits_ground, sun_itrf, instant)
			if hits_ground:
				var hit := point_itrf_m + v * d_end
				var lux: Dictionary = _core.atmosphere_illuminance(hit, _geodetic_up(hit), sun_itrf)
				var t: Vector3 = _core.atmosphere_sky_luminance_to_point(point_itrf_m, hit, sun_itrf)["transmittance"]
				var lit: Vector3 = lux["sun_lux"] + lux["sky_lux"]
				eclipsed += lit * (float(parameters["ground_albedo"]) / PI) * t * _visible_at(hit, instant)
			fluence += eclipsed * solid_angle
	return fluence


static func _geodetic_up(point_itrf_m: Vector3) -> Vector3:
	return Vector3(point_itrf_m.x, point_itrf_m.y, point_itrf_m.z * POLAR_STRETCH * POLAR_STRETCH).normalized()


static func _stretch(v: Vector3) -> Vector3:
	return Vector3(v.x, v.y, v.z * POLAR_STRETCH)


static func _unstretch(v: Vector3) -> Vector3:
	return Vector3(v.x, v.y, v.z / POLAR_STRETCH)


## Eye adaptation's reference luminance (cd/m^2): the white Lambertian under the top-of-atmosphere Sun at d_AU, the state
## DAY_EXPOSURE is defined for (same channels and weights as adaptation_luminance).
## The white plate under the TOA Sun at this frame's Sun distance (cd/m^2); 0 before the tables have loaded.
func reference_luminance_now() -> float:
	if not (parameters.has("solar_illuminance_rgb") and parameters.has("sun_radiance_to_luminance")):
		return 0.0
	return reference_luminance(_sun_distance_au)


func reference_luminance(d_au: float) -> float:
	var solar: Vector3 = parameters["solar_illuminance_rgb"]
	var sun_factor: Vector3 = parameters["sun_radiance_to_luminance"]
	return (solar * sun_factor).dot(LUMINANCE_WEIGHTS) / (PI * d_au * d_au)


## True when the Moon's penumbral cone reaches the sphere of radius_m about the Earth's centre (vectors from it, m): the
## cone's half angle asin((R_sun + R_moon) / |Sun - Moon|), its apex on the Sun's side of the Moon.
static func penumbra_touches(sun_m: Vector3, moon_m: Vector3, radius_m: float) -> bool:
	var axis := moon_m - sun_m
	var sun_moon_m := axis.length()
	axis /= sun_moon_m
	var along := -moon_m.dot(axis)  # the Earth's centre beyond the Moon along the shadow axis
	if along <= 0.0:
		return false
	var off_axis := (-moon_m - axis * along).length()
	var sin_half := (SUN_RADIUS_M + MOON_RADIUS_M) / sun_moon_m
	var apex_m := MOON_RADIUS_M / sin_half  # apex distance on the Sun's side of the Moon
	var cone_radius := (apex_m + along) * tan(asin(sin_half))
	# The Earth's sphere touches the cone where its distance from the axis, measured across the cone wall, is below
	# its radius: off_axis - cone_radius < radius / cos(half angle).
	return off_axis - cone_radius < radius_m / cos(asin(sin_half))


## The next update takes the eye as adapted to its view (a new view, a test's fixed instant).
func snap_adaptation() -> void:
	_eye.snap()
	_field_epoch += 1  # field results of the old view are dropped; the first ones of the new view snap the eye
	_field_valid = false
	_field_snap = true
	_field_snap_start_ms = Time.get_ticks_msec()
	adaptation_surround.reset()  # the surround of the old place


## The newest field measurement of this view (AdaptationMeter): last_field_cd_m2 / last_field_scotopic_cd_m2 and
## _field_valid. A measurement older than FIELD_STALE_FRAMES frames (the meter detached or not running) falls back to
## the analytic rule; the next one takes over paced (review M-6: no visible step after a hitch; only a new view snaps).
func _take_field_measurement() -> void:
	var result := adaptation_meter.take_result(_field_epoch, _field_serial)
	if result.is_empty():
		_field_age += 1
		if _field_age > FIELD_STALE_FRAMES and _field_valid:
			_field_valid = false
		return
	_field_age = 0
	_field_serial = result[2]
	var counts: PackedInt32Array = result[0]
	if scene_referred_adaptation:
		counts = adaptation_surround.combine(counts, result[3])  # + the field beyond the frame (look fixes T4)
	# The Sun's disability glare over the field (its disc is masked out of the histograms; T5b fix round 1).
	var veil := glare_veil(result[3], result[4]) if glare_enabled else Vector2.ZERO
	last_veil_cd_m2 = veil.x
	last_veil_scotopic_cd_m2 = veil.y
	# + the glare spread's veil of the sources the display cannot show (the Earth, the Moon; its image part, one frame old):
	# the mean over the field, added as the Sun's is (not per sample: see AdaptationMeter.veil_mean).
	last_veil_image_cd_m2 = 0.0
	if glare_enabled and result.size() > 5:
		var image_veil := Vector2(AdaptationMeter.veil_mean(result[5], 0),
				AdaptationMeter.veil_mean(result[5], AdaptationMeter.HISTOGRAM_BINS))
		last_veil_image_cd_m2 = image_veil.x
		veil += image_veil
	var photopic := AdaptationMeter.field_log_average(counts, 0)
	if photopic < 0.0:
		return
	# The veil lies over the whole field: it raises the field and its anchor by itself (a black field becomes the veil).
	last_field_cd_m2 = photopic + veil.x
	last_field_scotopic_cd_m2 = maxf(AdaptationMeter.field_log_average(counts, AdaptationMeter.HISTOGRAM_BINS), 0.0) + veil.y
	last_anchor_cd_m2 = maxf(AdaptationMeter.field_anchor(counts, 0), 0.0) + veil.x
	last_anchor_scotopic_cd_m2 = maxf(AdaptationMeter.field_anchor(counts, AdaptationMeter.HISTOGRAM_BINS), 0.0) + veil.y
	last_sun_mask_weights = result[4]
	_field_loading = result[1]
	_field_valid = true
	if _log_adaptation and _field_serial % ADAPTATION_LOG_EVERY == 0:
		print(("adaptation: field %s cd/m^2 (scotopic %s), anchor %s (scotopic %s), %s, white-plate rule %s cd/m^2; frame"
				+ " (visibility statistic, whole-weight cut) %s, surround model in frame %s, visibility %.3f, out-of-frame share"
				+ " %.3f; glare veil %s cd/m^2 (Sun %.2f deg off axis, visibility %.2f, mask %s)") % [
				last_field_cd_m2, last_field_scotopic_cd_m2, last_anchor_cd_m2, last_anchor_scotopic_cd_m2,
				"loading" if _field_loading else "settled", last_target_cd_m2, adaptation_surround.last_frame_field_cd_m2,
				adaptation_surround.last_model_in_cd_m2, adaptation_surround.last_visibility, adaptation_surround.last_out_share,
				last_veil_cd_m2, last_veil_angle_deg, last_veil_visibility, last_sun_mask_weights])


## The Sun light of Godot's lit materials (vehicles, structures: world.gd's DirectionalLight3D) at an Earth-fixed point:
## [colour, energy]. Per channel it is the scene value atm_sun_at gives the Earth's surfaces (one exposure): the TOA
## solar illuminance of the tables (its own colour in the RGB luminance space) x the transmittance to the Sun
## (StarsimCore.atmosphere_sun_transmittance: red and dim at sunset, zero below the horizon or in the Earth's shadow) x
## luminance_scale / pi x visible_fraction (the unocculted part of the solar disc, Moon) - a white Lambertian at normal
## incidence renders it; in luminance that is DAY_EXPOSURE / d_AU^2 x T under the TOA Sun. Godot multiplies a
## directional light's energy by pi and its Lambert BRDF divides by pi, so energy = the brightest channel's value and
## colour = the channels over it, sRGB-encoded (Godot linearises light colours). Before the tables are loaded: the
## vacuum Sun (white, DAY_EXPOSURE). probe_key: the drawn shapes' S probe (Sun direction only; "" for other lights).
func sun_light_at(position_itrf_m: Vector3, sun_direction_itrf: Vector3, visible_fraction: float,
		probe_key := "sun_light", tint := Vector3.ONE) -> Array:
	var scene := Vector3.ONE * DAY_EXPOSURE
	if ready:
		var transmittance := _core.atmosphere_sun_transmittance(position_itrf_m, sun_direction_itrf)
		var solar: Vector3 = parameters["solar_illuminance_rgb"]
		var sun_factor: Vector3 = parameters["sun_radiance_to_luminance"]
		scene = solar * sun_factor * transmittance * (luminance_scale / PI)
		# The direct beam through the clouds (the drawn shapes' S, else the grid mean along the ray; 1 with weather off).
		scene *= cloud_light_at(position_itrf_m, sun_direction_itrf, probe_key).x
	scene *= tint * visible_fraction
	var peak := maxf(scene.x, maxf(scene.y, scene.z))
	var color := Color.WHITE
	if peak > 0.0:
		color = Color(scene.x / peak, scene.y / peak, scene.z / peak).linear_to_srgb()
	return [color, peak]


## The clouds' light at an Earth-fixed point for the light from light_direction_itrf (StarsimCore.weather_cloud_light,
## the active weather pair's grid mean, Task 9): x the direct beam's transmittance, y the light's diffuse part through
## the column per unit of its horizontal irradiance, z the sky's diffuse transmittance; (1, 0, 1) without weather.
## sun_probe (the Sun's direct beam only): x from the drawn clouds' shapes instead (direct_sun under that key, 2026-10-30)
## where they have a value. Cached per frame (the same point and direction several times: Sun light, ambient,
## adaptation).
func cloud_light_at(position_itrf_m: Vector3, light_direction_itrf: Vector3, sun_probe := "") -> Vector3:
	var key := [position_itrf_m, light_direction_itrf, sun_probe, Engine.get_process_frames()]
	if _cloud_cache.has(key):
		return _cloud_cache[key]
	if _cloud_cache.size() > 8:
		_cloud_cache.clear()
	# Only while the clouds are drawn (cloud_enabled: noise ready, weather shown, within the scaled-space limit): the
	# vehicles and the eye are dimmed exactly when the GPU's clouds are there (Task 9 fix 1; one frame behind at most).
	var shown: bool = ShaderGlobals.value(&"cloud_enabled")
	var light: Dictionary = _core.weather_cloud_light(position_itrf_m, light_direction_itrf) if _core != null and shown else {}
	var result := Vector3(float(light.get("sun_transmittance", 1.0)), float(light.get("sun_diffuse", 0.0)),
			float(light.get("sky_factor", 1.0)))
	if shown and sun_probe != "" and direct_sun.is_valid():
		var shaped: float = direct_sun.call(sun_probe, position_itrf_m)
		if not is_nan(shaped):
			result.x = shaped
	_cloud_cache[key] = result
	return result


## The factors of the sky's radiance cube at an Earth-fixed point under the clouds, the light there over the clear
## sky's, by
## channel (the cube's clear sky is blue, the light under a deck mostly the Sun's, scattered; the droplets are grey):
## [0] for the sky's half, the diffuse light, (T_d E_sky + T_sd E_h) / E_sky (above 1 under a bright deck: the Sun's
## light,
## scattered, now comes from the sky); [1] for the ground's half, which already holds the direct light,
## (E_h (T_s + T_sd) + E_sky T_d) / (E_h + E_sky). E_sky and E_h at the point, so the air under the deck is in the
## colour.
## Each sum over the Sun and, while the cube shows it, the Moon (their light relative to the Sun's); all 1 with weather
## off
## (T_s = T_d = 1, T_sd = 0).
func cloud_radiance_factors(position_itrf_m: Vector3, sun_direction_itrf: Vector3) -> Array[Vector3]:
	if not ready or not ShaderGlobals.value(&"cloud_enabled"):
		return [Vector3.ONE, Vector3.ONE]
	var up := _geodetic_up(position_itrf_m)
	var terms := _cloud_light_terms(position_itrf_m, up, sun_direction_itrf, 1.0)
	if moonlight_active and _moon_direction_fixed != Vector3.ZERO:
		var moon_terms := _cloud_light_terms(position_itrf_m, up, _moon_direction_fixed, moonlight_relative_value)
		for k in range(terms.size()):
			terms[k] += moon_terms[k]
	var column_sky := cloud_light_at(position_itrf_m, sun_direction_itrf).z  # no light at all: what the column passes
	return [_channel_ratio(terms[2], terms[0], column_sky), _channel_ratio(terms[3], terms[0] + terms[1], column_sky)]


## One source's light for cloud_radiance_factors, by channel: [E_sky, E_h, the diffuse light under the deck, the
## ground's
## light under it].
func _cloud_light_terms(position_itrf_m: Vector3, up: Vector3, direction_itrf: Vector3,
		relative: float) -> Array[Vector3]:
	var clouds := cloud_light_at(position_itrf_m, direction_itrf)
	var lux: Dictionary = _core.atmosphere_illuminance(position_itrf_m, up, direction_itrf)
	var sky: Vector3 = (lux["sky_lux"] as Vector3) * relative
	var beam: Vector3 = (lux["sun_lux"] as Vector3) * relative if direction_itrf.dot(up) > 0.0 else Vector3.ZERO
	return [sky, beam, clouds.z * sky + clouds.y * beam, (clouds.x + clouds.y) * beam + clouds.z * sky]


## a / b by channel; where b holds no light, the fallback.
static func _channel_ratio(a: Vector3, b: Vector3, fallback: float) -> Vector3:
	return Vector3(a.x / b.x if b.x > 0.0 else fallback, a.y / b.y if b.y > 0.0 else fallback,
			a.z / b.z if b.z > 0.0 else fallback)


func _load_task(config_dir: String, cache_dir: String) -> void:
	var result: Dictionary = _core.load_atmosphere(config_dir, cache_dir)
	if result.get("ok", false):
		# Images are built here (the large copies), textures on the main thread (_upload).
		var t_size: Vector2i = result["transmittance_size"]
		var s_size: Vector3i = result["scattering_size"]
		var i_size: Vector2i = result["irradiance_size"]
		result["transmittance_image"] = _image_2d(result["transmittance"], t_size, Image.FORMAT_RGBAF)
		result["irradiance_image"] = _image_2d(result["irradiance"], i_size)
		result["scattering_images"] = _images_3d(result["scattering"], s_size)
		result["single_mie_images"] = _images_3d(result["single_mie"], s_size)
		result["single_rayleigh_images"] = _images_3d(result["single_rayleigh"], s_size)
		for key in ["transmittance", "irradiance", "scattering", "single_mie", "single_rayleigh"]:
			result.erase(key)
	_result = result


func _image_2d(data: PackedFloat32Array, size: Vector2i, format := table_format) -> Image:
	var image := Image.create_from_data(size.x, size.y, false, Image.FORMAT_RGBAF, data.to_byte_array())
	if format != Image.FORMAT_RGBAF:
		image.convert(format)
	return image


## One image per depth slice (z = r), x fastest within a slice (the core's layout).
func _images_3d(data: PackedFloat32Array, size: Vector3i) -> Array[Image]:
	var bytes := data.to_byte_array()
	var slice_bytes := size.x * size.y * 4 * 4
	var images: Array[Image] = []
	for z in range(size.z):
		var image := Image.create_from_data(size.x, size.y, false, Image.FORMAT_RGBAF,
				bytes.slice(z * slice_bytes, (z + 1) * slice_bytes))
		if table_format != Image.FORMAT_RGBAF:
			image.convert(table_format)
		images.append(image)
	return images


func _upload(result: Dictionary) -> void:
	if not result.get("ok", false):
		# The player sees the report text (data.atmosphere_unavailable); the cause stays in the English log.
		status_key = "report." + str(result.get("report", {}).get("code", "data.atmosphere_unavailable"))
		push_error("Atmosphere: " + str(result.get("error", "load failed")))
		return
	parameters = result["parameters"]
	var s_size: Vector3i = result["scattering_size"]
	var transmittance := ImageTexture.create_from_image(result["transmittance_image"])
	var irradiance := ImageTexture.create_from_image(result["irradiance_image"])
	var scattering := ImageTexture3D.new()
	scattering.create(table_format, s_size.x, s_size.y, s_size.z, false, result["scattering_images"])
	var single_mie := ImageTexture3D.new()
	single_mie.create(table_format, s_size.x, s_size.y, s_size.z, false, result["single_mie_images"])
	# The single Rayleigh part (static clouds Task 9b): the cloud pass's cloud shadows on the air.
	var single_rayleigh := ImageTexture3D.new()
	single_rayleigh.create(table_format, s_size.x, s_size.y, s_size.z, false, result["single_rayleigh_images"])
	_textures = [transmittance, irradiance, scattering, single_mie, single_rayleigh]
	var t_size: Vector2i = result["transmittance_size"]
	var i_size: Vector2i = result["irradiance_size"]
	var cells: Vector4i = result["scattering_cells"]  # (nu, mu_s, mu, r)
	ShaderGlobals.set_value(&"atmosphere_transmittance", transmittance)
	ShaderGlobals.set_value(&"atmosphere_irradiance", irradiance)
	ShaderGlobals.set_value(&"atmosphere_scattering", scattering)
	ShaderGlobals.set_value(&"atmosphere_single_mie", single_mie)
	ShaderGlobals.set_value(&"atmosphere_single_rayleigh", single_rayleigh)
	ShaderGlobals.set_value(&"atmosphere_bottom_radius_m", float(parameters["bottom_radius_m"]))
	ShaderGlobals.set_value(&"atmosphere_top_radius_m", float(parameters["top_radius_m"]))
	ShaderGlobals.set_value(&"atmosphere_mu_s_min", float(parameters["mu_s_min"]))
	ShaderGlobals.set_value(&"atmosphere_mie_phase_g", float(parameters["mie_phase_g"]))
	ShaderGlobals.set_value(&"atmosphere_sun_angular_radius_rad", float(parameters["sun_angular_radius_rad"]))
	ShaderGlobals.set_value(&"atmosphere_sky_radiance_to_luminance", parameters["sky_radiance_to_luminance"])
	var solar: Vector3 = parameters["solar_illuminance_rgb"]
	var sun_factor: Vector3 = parameters["sun_radiance_to_luminance"]
	ShaderGlobals.set_value(&"atmosphere_sun_illuminance_lux", solar * sun_factor)
	ShaderGlobals.set_value(&"atmosphere_ground_albedo", float(parameters["ground_albedo"]))
	# A true luminance (cd/m^2, at the Sun's actual distance) -> scene value: luminance_scale without the 1 / d_AU^2
	# (that factor converts the tables' 1 AU values); the star threshold reads the sky's true luminance back with it.
	# Without the exposure gain until the first update sets both.
	scene_per_cd_m2 = DAY_EXPOSURE * PI / float(parameters["toa_solar_illuminance_lux"])
	ShaderGlobals.set_value(&"atmosphere_scene_per_cd_m2", scene_per_cd_m2)
	ShaderGlobals.set_value(&"atmosphere_scattering_size", Vector4(cells.x, cells.y, cells.z, cells.w))
	ShaderGlobals.set_value(&"atmosphere_table_sizes", Vector4(t_size.x, t_size.y, i_size.x, i_size.y))
	ShaderGlobals.set_value(&"atmosphere_enabled", true)
	ready = true
	status_key = ""
