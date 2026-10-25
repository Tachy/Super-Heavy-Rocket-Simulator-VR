class_name EyeAdaptation
extends RefCounted
## Eye adaptation (M8 atmosphere [call] 2026-10-25): an exposure gain that follows the light at the viewer, so civil
## twilight is not rendered black by the day exposure, while night stays night. Exposure only: one factor on every
## physical luminance of the scene (the atmosphere's luminance scale, the Sun light, Moon, Sun disc) - no white
## balance, no colour or contrast change. One rule for emitters without a physical luminance (the plumes, the city
## lights, the stars' own brightness): display-referred, not multiplied by the gain (review M2/M3) - their on-screen
## brightness is a display call; the stars' visibility reads the true sky luminance behind the star and, as its floor,
## the luminance the eye is adapted to (adaptation_cd_m2, global atmosphere_adaptation_cd_m2). Fidelity flag:
## Atmosphere.eye_adaptation_enabled (on in the game; off in atmosphere_check.gd, whose samples keep the fixed day
## exposure).
##
## Adaptation luminance L_a (Atmosphere.adaptation_luminance, the core's CPU evaluation): the larger of a white
## Lambertian at the eye under the direct Sun at normal incidence and the sky, (E_sun,n + E_sky) / pi, and the Earth in
## view (the same plate equivalent of the visible ground and limb, x the cap's view factor); above the atmosphere also
## the Moon in view (the regolith as the Moon shaders draw it, sunlit and earthshine-lit, x its cap's view factor) and
## the Earth's disc in front of the Sun (Moon-eye fix 2026-10-25). The reference L_ref is the
## plate under the top-of-atmosphere Sun - the state DAY_EXPOSURE is defined for. Gain: Weber adaptation, L_ref / L_a,
## clamped to [1, GAIN_MAX]. Above ~80 cd/m^2 the cone threshold grows in proportion to the adaptation luminance
## (Weber's law; Ferwerda et al. 1996, "A model of visual adaptation for realistic image synthesis", SIGGRAPH 96, the
## photopic threshold-versus-intensity function). In orbit on the day side L_a >= L_ref: gain 1, the day look; on the
## ground at noon ~1.1.

## [call] Upper bound of the gain - a display decision, not physiology: Weber adaptation itself would allow up to
## ~L_ref / 80 cd/m^2 ~ 500. There is no mesopic / scotopic model (rod vision: dim, colourless, Purkinje shift), so the
## gain stops where twilight would otherwise render as bright as day: the ratio of the CPU zenith luminances at the pad
## with the Sun at 60 deg and at 0 deg (3834 / 142 cd/m^2 = 27; Tasks 6+7 review). With it civil twilight renders dim
## (Sun -3 deg: zenith ~0.1 x the noon zenith) and nautical twilight dark, so night stays night-like.
const GAIN_MAX := 25.0
## [E] Time constant of light adaptation (gain falling, the scene got brighter), s: the eye's fast neural adaptation
## acts within ~0.1 s and the bleaching of the cone pigment within seconds (Pattanaik et al. 2000, "Time-dependent
## visual adaptation for fast realistic image display", SIGGRAPH 2000; Ferwerda et al. 1996).
const LIGHT_TIME_CONSTANT_S := 0.5
## [E] Time constant of dark adaptation (gain rising), s: the cones' photopigment regeneration, ~110 s in Pattanaik et
## al. 2000 (cone dark adaptation reaches its plateau after ~10 min; Ferwerda et al. 1996).
const DARK_TIME_CONSTANT_S := 110.0

## The current gain (1 = the fixed day exposure).
var gain := 1.0
var _snap_pending := true
## [call] Floor of the paced adaptation luminance (cd/m^2), for its log-space pacing only: far below a dark night sky
## (22 mag/arcsec^2 ~ 1.7e-4 cd/m^2), so the star threshold there is the sky's (Schaefer: +0.06 mag at the floor).
const LUMINANCE_FLOOR_CD_M2 := 1.0e-6
## The luminance the eye is adapted to (cd/m^2), paced like the gain but unclamped (final review I1): the star
## threshold's background floor. The gain's clamp to [1, GAIN_MAX] must not feed back here (L_ref / 25 ~ 1600 cd/m^2
## would hide every star at night).
var adaptation_cd_m2 := LUMINANCE_FLOOR_CD_M2
var _luminance_snap_pending := true


## The next step of adaptation_cd_m2 toward the target luminance over dt_s: exponential in log luminance, the light
## time constant when the scene got brighter, the dark one when it got darker (the same rule as advance); a pending
## snap jumps to the target.
func advance_luminance(target_cd_m2: float, dt_s: float) -> float:
	var target := maxf(target_cd_m2, LUMINANCE_FLOOR_CD_M2)
	if _luminance_snap_pending:
		adaptation_cd_m2 = target
		_luminance_snap_pending = false
		return adaptation_cd_m2
	var tau := LIGHT_TIME_CONSTANT_S if target > adaptation_cd_m2 else DARK_TIME_CONSTANT_S
	var weight := 1.0 - exp(-maxf(dt_s, 0.0) / tau)
	adaptation_cd_m2 = exp(lerpf(log(adaptation_cd_m2), log(target), weight))
	return adaptation_cd_m2


## The fully adapted gain for the adaptation luminance and the reference (both cd/m^2).
static func target_gain(adaptation_cd_m2: float, reference_cd_m2: float) -> float:
	if adaptation_cd_m2 <= 0.0:
		return GAIN_MAX
	return clampf(reference_cd_m2 / adaptation_cd_m2, 1.0, GAIN_MAX)


## The next adaptation step toward `target` over dt_s (exponential in log gain; the light or the dark time constant).
## A pending snap (start, view change) jumps to the target.
func advance(target: float, dt_s: float) -> float:
	if _snap_pending:
		gain = target
		_snap_pending = false
		return gain
	var tau := LIGHT_TIME_CONSTANT_S if target < gain else DARK_TIME_CONSTANT_S
	var weight := 1.0 - exp(-maxf(dt_s, 0.0) / tau)
	gain = exp(lerpf(log(gain), log(target), weight))
	return gain


## The next advance jumps to its target (a new view: the viewer is taken as adapted to it).
func snap() -> void:
	_snap_pending = true
	_luminance_snap_pending = true
