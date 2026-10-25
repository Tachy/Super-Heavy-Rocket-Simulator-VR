class_name EyeAdaptation
extends RefCounted
## Eye adaptation (M8 atmosphere [call] 2026-10-25): an exposure gain that follows the light at the viewer, so civil
## twilight is not rendered black by the day exposure, while night stays night. Exposure only: one factor on every
## physical luminance of the scene (the atmosphere's luminance scale, the Sun light, Moon, Sun disc, plumes) - no white
## balance, no colour or contrast change. Fidelity flag: Atmosphere.eye_adaptation_enabled (on in the game; off in
## atmosphere_check.gd, whose samples keep the fixed day exposure).
##
## Adaptation luminance L_a: a white Lambertian surface at the eye lit by the direct Sun at normal incidence and the
## sky's horizontal irradiance, (E_sun,n + E_sky) / pi (cd/m^2, the core's CPU evaluation; Atmosphere.update). The
## reference L_ref is the same surface under the top-of-atmosphere Sun - the state DAY_EXPOSURE is defined for.
## Gain: Weber adaptation, L_ref / L_a, clamped to [1, GAIN_MAX]. Above ~80 cd/m^2 the cone threshold grows in
## proportion to the adaptation luminance (Weber's law; Ferwerda et al. 1996, "A model of visual adaptation for
## realistic image synthesis", SIGGRAPH 96, the photopic threshold-versus-intensity function), and GAIN_MAX keeps every
## adapted scene inside that range (L_a >= L_ref / GAIN_MAX ~ 1700 cd/m^2). In orbit on the day side L_a >= L_ref:
## gain 1, the day look; on the ground at noon ~1.1.

## [call] Upper bound of the gain: the ratio of the CPU zenith luminances at the pad with the Sun at 60 deg and at
## 0 deg (3834 / 142 cd/m^2 = 27; Tasks 6+7 review). A larger gain would render the sky at sunset brighter than the
## noon sky; with it civil twilight renders dim (Sun -3 deg: zenith ~0.1 x the noon zenith) and nautical twilight dark,
## so night stays night-like. Adaptation beyond it (mesopic / scotopic vision) is not an exposure: it is left out.
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
