class_name EyeAdaptation
extends RefCounted
## Eye adaptation and night vision (M8 atmosphere; user decision 2026-10-25 "like the eye"): what a human eye adapted to
## the light at the viewer perceives - the day look in daylight, a dim twilight, and at night a dim, desaturated,
## bluish image (rod vision, the Purkinje shift), while a moonless night stays dark. Fidelity flag:
## Atmosphere.eye_adaptation_enabled (on in the game; off in atmosphere_check.gd's comparison samples: the fixed day
## exposure).
##
## Model [call: chosen among Ferwerda 1996, Pattanaik 2000 and Kirk & O'Brien 2011]: Ferwerda, Pattanaik, Shirley &
## Greenberg 1996, "A model of visual adaptation for realistic image synthesis", SIGGRAPH 96 - threshold matching
## (Ward's contrast operator, its Eq. 3: a world luminance at the world observer's threshold maps to the display
## observer's threshold), separately for the cones (photopic t.v.i., Eq. 4) and the rods (scotopic t.v.i., Eq. 5), and
## combined over the mesopic range (Eq. 6): display = m_c L + k m_r L_s, the rods achromatic. Above ~80 cd/m^2 the cone
## threshold is proportional to the adaptation luminance (Weber's law), so the cone part is the former Weber gain
## exactly; below, the cone threshold flattens (absolute sensitivity) and the rods take over - Weber's law fails at
## scotopic levels, which is what keeps a moonless night dark (no light, no image; the threshold does not fall further).
##
## In the renderer:
## - the cone gain g_c = L_ref / (10^1.255 t_p(L_a)) multiplies every physical luminance in the shaders (the pre-exposure
##   Atmosphere.exposure_gain, as the former gain: the tables, the Sun light, Moon, Sun disc); L_ref is the white plate
##   under the TOA Sun, the state DAY_EXPOSURE is defined for, so in the Weber range g_c = L_ref / L_a (the display
##   observer's adaptation is the day look's) [call]; at least 1 (the fixed day exposure in sunlit space, as before);
## - the rods: a full-screen post step (VisionEffect, a CompositorEffect after the transparent pass, before glow and the
##   tone mapper; the same for both VR eyes) adds rod_scale x max(w_s . rgb, 0) x ROD_TINT to every pixel, rod_scale =
##   k g_r / g_c with the rod gain g_r = L_ref / (10^1.255 t_s(L_as)) (Ferwerda's m_r: the display observer is
##   photopic) and the scotopic luminance from the pixel's linear RGB (SCOTOPIC_WEIGHTS);
## - display-referred emitters (plume, the stars' own brightness, UI markers, the VR panel) take the
##   inverse of that step in their shaders (shaders/vision.gdshaderinc), so they look as before.
## Inputs: the photopic and scotopic adaptation luminances L_a, L_as (Atmosphere.adaptation_luminance, view-independent),
## paced in log space: light adaptation tau 0.5 s, dark adaptation 110 s (Pattanaik et al. 2000, "Time-dependent visual
## adaptation for fast realistic image display", SIGGRAPH 2000; Ferwerda et al. 1996).

## [S: Ferwerda et al. 1996 Eq. 4] the cones' threshold-versus-intensity function: log10 t_p(L_a) (cd/m^2) =
## -0.72 for log L_a <= -2.6, log L_a - 1.255 for log L_a >= 1.9, else (0.249 log L_a + 0.65)^2.7 - 0.72.
const CONE_LOG_FLOOR := -0.72
const CONE_LOG_LOW := -2.6
const CONE_LOG_WEBER := 1.9
const CONE_WEBER_OFFSET := 1.255
## [S: Ferwerda et al. 1996 Eq. 5] the rods': log10 t_s(L_as) (scotopic cd/m^2) = -2.86 for log L_as <= -3.94,
## log L_as - 0.395 for log L_as >= -1.44, else (0.405 log L_as + 1.6)^2.18 - 2.86.
const ROD_LOG_FLOOR := -2.86
const ROD_LOG_LOW := -3.94
const ROD_LOG_WEBER := -1.44
const ROD_WEBER_OFFSET := 0.395
## [S: Jensen, Premoze, Shirley, Thompson, Ferwerda & Stark 2000, "Night rendering", Univ. of Utah UUCS-00-016, sec.
## 3.1] the rods' saturation s over the mesopic range 0.01 .. 4 cd/m^2: s = 0 below log Y = -2, 1 above 0.6, else
## 3 x^2 - 2 x^3 with x = (log Y + 2) / 2.6. Ferwerda's k (Eq. 6, "from 1 to 0 across the mesopic range") = 1 - s,
## evaluated on the photopic adaptation luminance [call: one k for the whole view, not per pixel - VR-stable].
const MESOPIC_LOG_LOW := -2.0
const MESOPIC_LOG_SPAN := 2.6
## [D: tools/validation/scotopic_rgb_weights.py] scotopic luminance (scotopic cd/m^2) of linear sRGB channels in
## photopic cd/m^2: the CIE 1951 V'(lambda) (K'm = 1700 lm/W) projected onto the sRGB colour matching functions,
## least squares in S/P over black bodies 2000-25 000 K and their lambda^-4 (sky) versions: worst 2.2 % in S/P
## (illuminant A 1.413 -> +1.4 %, 6504 K 2.446 -> -0.6 %); S/P of RGB white 2.43.
const SCOTOPIC_WEIGHTS := Vector3(-0.0525, 1.6016, 0.8794)
## Rod vision's colour, linear sRGB at unit luminance: chromaticity (x, y) = (0.25, 0.25) [S: Jensen, Premoze,
## Shirley, Thompson, Ferwerda & Stark 2000, "Night rendering", sec. 3.1: their rendering choice, south-west of the
## mean (0.29, 0.28) they measured in the "dark" parts of ten realist night paintings]. Rods are achromatic (Ferwerda
## 1996); the blue cast is the rendering convention for night vision (Jensen et al. 2000; Thompson, Shirley & Ferwerda
## 2002, "A spatial post-processing algorithm for images of night scenes", JGT 7(1)) [call: the measured mean is
## purple (R > G) - a lavender cast on the twilight sky]. X = x / y, Z = (1 - x - y) / y through IEC 61966-2-1's
## XYZ -> sRGB.
const ROD_TINT := Vector3(0.7062, 0.9899, 1.9657)
## [call] Numeric safety bound of the cone gain (not a display decision): the t.v.i. floor bounds it at
## L_ref x 10^(1.255 - 0.72) ~ 1.24e4 already; this only guards a bad reference. The Sun disc clamps its own scene value
## (sun.gdshader) so a sudden Sun in a dark-adapted view stays inside half-float range.
const GAIN_MAX := 1.0e5
## [E] Time constant of light adaptation (the scene got brighter), s: the eye's fast neural adaptation acts within
## ~0.1 s and the bleaching of the cone pigment within seconds (Pattanaik et al. 2000; Ferwerda et al. 1996).
const LIGHT_TIME_CONSTANT_S := 0.5
## [E] Time constant of dark adaptation (the scene got darker), s: the cones' photopigment regeneration, ~110 s in
## Pattanaik et al. 2000 (cone dark adaptation reaches its plateau after ~10 min; Ferwerda et al. 1996). The rods'
## slower recovery (~30 min) is not modelled separately [call: one pacing for both].
const DARK_TIME_CONSTANT_S := 110.0
## [call] Floor of the paced adaptation luminances (cd/m^2), for their log-space pacing only: far below a dark night sky
## (22 mag/arcsec^2 ~ 1.7e-4 cd/m^2), so the star threshold there is the sky's (Schaefer: +0.06 mag at the floor); the
## t.v.i. functions are flat far above it.
const LUMINANCE_FLOOR_CD_M2 := 1.0e-6

## The cone gain this frame (the pre-exposure on every physical luminance; 1 = the fixed day exposure).
var gain := 1.0
## The rods' share on screen this frame: rod_scale (see above); 0 in daylight.
var rod_scale := 0.0
## The paced photopic adaptation luminance (cd/m^2): the cone gain, the mesopic share and the star threshold's
## background floor (global atmosphere_adaptation_cd_m2).
var adaptation_cd_m2 := LUMINANCE_FLOOR_CD_M2
## The paced scotopic adaptation luminance (scotopic cd/m^2): the rod gain.
var scotopic_cd_m2 := LUMINANCE_FLOOR_CD_M2
var _snap_pending := true


## The next step toward the target adaptation luminances (photopic cd/m^2, scotopic cd/m^2) over dt_s: exponential in
## log luminance, the light time constant when the scene got brighter, the dark one when it got darker; a pending snap
## jumps to the targets. Then the gains for the reference luminance (cd/m^2, the white plate under the TOA Sun). The
## photopic target is capped at the reference: above it the gain is 1 anyway, and an eye paced from above it would wait
## (dark constant) until it fell below the reference before the gain moved - the former code paced the clamped gain
## (night-vision review M3).
func advance(target_cd_m2: float, target_scotopic_cd_m2: float, reference_cd_m2: float, dt_s: float) -> void:
	var target := clampf(target_cd_m2, LUMINANCE_FLOOR_CD_M2, maxf(reference_cd_m2, LUMINANCE_FLOOR_CD_M2))
	var target_scotopic := maxf(target_scotopic_cd_m2, LUMINANCE_FLOOR_CD_M2)
	if _snap_pending:
		adaptation_cd_m2 = target
		scotopic_cd_m2 = target_scotopic
		_snap_pending = false
	else:
		adaptation_cd_m2 = _paced(adaptation_cd_m2, target, dt_s)
		scotopic_cd_m2 = _paced(scotopic_cd_m2, target_scotopic, dt_s)
	gain = cone_gain(adaptation_cd_m2, reference_cd_m2)
	rod_scale = rod_share(adaptation_cd_m2) * rod_gain(scotopic_cd_m2, reference_cd_m2) / gain


static func _paced(current: float, target: float, dt_s: float) -> float:
	var tau := LIGHT_TIME_CONSTANT_S if target > current else DARK_TIME_CONSTANT_S
	var weight := 1.0 - exp(-maxf(dt_s, 0.0) / tau)
	return exp(lerpf(log(current), log(target), weight))


## log10 of the cones' detection threshold (cd/m^2) at the adaptation luminance (Ferwerda Eq. 4).
static func cone_log_threshold(adaptation_cd_m2: float) -> float:
	var l := log(maxf(adaptation_cd_m2, LUMINANCE_FLOOR_CD_M2)) / log(10.0)
	if l <= CONE_LOG_LOW:
		return CONE_LOG_FLOOR
	if l >= CONE_LOG_WEBER:
		return l - CONE_WEBER_OFFSET
	return pow(0.249 * l + 0.65, 2.7) + CONE_LOG_FLOOR


## log10 of the rods' detection threshold (scotopic cd/m^2) at the scotopic adaptation luminance (Ferwerda Eq. 5).
static func rod_log_threshold(scotopic_cd_m2: float) -> float:
	var l := log(maxf(scotopic_cd_m2, LUMINANCE_FLOOR_CD_M2)) / log(10.0)
	if l <= ROD_LOG_LOW:
		return ROD_LOG_FLOOR
	if l >= ROD_LOG_WEBER:
		return l - ROD_WEBER_OFFSET
	return pow(0.405 * l + 1.6, 2.18) + ROD_LOG_FLOOR


## The cone gain: L_ref / (10^1.255 t_p(L_a)), in [1, GAIN_MAX] - in the Weber range L_ref / L_a.
static func cone_gain(adaptation_cd_m2: float, reference_cd_m2: float) -> float:
	return clampf(reference_cd_m2 / pow(10.0, CONE_WEBER_OFFSET + cone_log_threshold(adaptation_cd_m2)), 1.0, GAIN_MAX)


## The rod gain (display scene units per scotopic cd/m^2, relative like the cone gain): L_ref / (10^1.255 t_s(L_as)).
static func rod_gain(scotopic_cd_m2: float, reference_cd_m2: float) -> float:
	return reference_cd_m2 / pow(10.0, CONE_WEBER_OFFSET + rod_log_threshold(scotopic_cd_m2))


## Ferwerda's k: the rods' weight, 1 in scotopic vision, 0 in photopic, the Jensen et al. smooth step between.
static func rod_share(adaptation_cd_m2: float) -> float:
	var x := clampf((log(maxf(adaptation_cd_m2, LUMINANCE_FLOOR_CD_M2)) / log(10.0) - MESOPIC_LOG_LOW)
			/ MESOPIC_LOG_SPAN, 0.0, 1.0)
	return 1.0 - x * x * (3.0 - 2.0 * x)


## The next advance jumps to its targets (a new view: the viewer is taken as adapted to it).
func snap() -> void:
	_snap_pending = true
