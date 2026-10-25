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
##   tone mapper; the same for both VR eyes) takes the cones' chroma to `chroma` (below) and adds
##   rod_scale x (w_s . rgb) x ROD_TINT to every pixel (linear: w_s >= 0), rod_scale =
##   k g_r / g_c with the rod gain g_r = L_ref / (10^1.255 t_s(L_as)) (Ferwerda's m_r: the display observer is
##   photopic) and the scotopic luminance from the pixel's linear RGB (SCOTOPIC_WEIGHTS);
## - display-referred emitters (plume, the stars' own brightness, UI markers, the VR panel) take the
##   inverse of that step in their shaders (shaders/vision.gdshaderinc), so they look as before;
## - the suprathreshold brightness and chroma factors (Stevens and Hunt effects, CIECAM02; see APPEARANCE_*): the
##   brightness on the pre-exposure (display-referred emitters do not take it), the chroma in the post step.
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
## least squares in S/P over black bodies 2000-25 000 K and their lambda^-4 (sky) versions, every weight >= 0: worst
## 4.2 % in S/P (the 25 000 K sky; illuminant A 1.413 -> +1.4 %, 6504 K 2.446 -> -1.7 %); S/P of RGB white 2.40.
## Non-negative [call, 2026-10-25 brightness term]: the rod step is linear (no clamp), so the display-referred inverse
## stays exact under blending; a negative red weight (the unconstrained fit: -0.0525, 1.6016, 0.8794, worst 2.2 %)
## would make a pure red scene pixel subtract rod light.
const SCOTOPIC_WEIGHTS := Vector3(0.0, 1.4730, 0.9238)
## [S: ITU-R BT.709] luminance weights of linear sRGB (the axis the chroma factor desaturates toward).
const LUMINANCE_WEIGHTS := Vector3(0.2126, 0.7152, 0.0722)
## Suprathreshold appearance (user decision 2026-10-25, "brightness/colourfulness term"): Ferwerda's threshold matching
## is ratio-preserving in the Weber range (g_c = L_ref / L_a), so every adapted scene there looked as bright and as
## colourful as noon - a civil dusk rendered brighter than noon. A real eye sees a dim scene dimmer and less colourful
## even when adapted: the Stevens effect (brightness grows with the luminance level) and the Hunt effect
## (colourfulness grows with it). Model [call: chosen among Stevens & Stevens 1963 / Tumblin-Rushmeier, Pattanaik et al.
## 2000 and Irawan et al. 2005 - the simplest view-independent one]: CIECAM02's luminance-level adaptation factor F_L
## [S: CIE 159:2004, "A colour appearance model for colour management systems: CIECAM02"; Moroney, Fairchild, Hunt,
## Li, Luo & Newman 2002, "The CIECAM02 color appearance model", IS&T/SID CIC 10]:
##     F_L = 0.2 k^4 (5 L_A) + 0.1 (1 - k^4)^2 (5 L_A)^(1/3),  k = 1 / (5 L_A + 1),
## at a constant relative stimulus (J, C: the eye adapted) brightness Q = (4 / c) (J / 100)^0.5 (A_w + 4) F_L^0.25 and
## colourfulness M = C F_L^0.25, with the white's achromatic response A_w = (2 R'_a + G'_a + B'_a / 20 - 0.305) N_bb
## (R'_a = G'_a = B'_a = 400 F_L^0.42 / (27.13 + F_L^0.42) + 0.1 for the adapted white; N_bb = 0.725 (Y_w / Y_b)^0.2).
## The display factors: brightness B = Q_w(L_a) / Q_w(L_day) on the displayed luminance [call: the brightness ratio
## applied as a luminance ratio - matching it through the display observer's ~1/3-power brightness would ask for B^3,
## 6e-5 for a full-Moon cone image], chroma S = (F_L(L_a) / F_L(L_day))^0.25 on the cones' chroma at constant
## luminance; both 1 at and above L_day (the day look, noon identical). L_A = L_a x Y_b / 100: L_a is the scene's white
## (the sunlit plate, half the brightest sky) [call]. Held below their ranges, so nothing is counted twice:
## - B below the Weber range (L_a < 10^CONE_LOG_WEBER = 79 cd/m^2), where Ferwerda's flattening cone threshold dims the
##   display already (the product stays monotonic in L_a);
## - S below the mesopic upper limit (10^(MESOPIC_LOG_LOW + MESOPIC_LOG_SPAN) = 4 cd/m^2), where the rods' achromatic
##   share (Eq. 6) takes over the desaturation (and the display-referred inverse's 1 / S amplification stays bounded
##   in the rgba16f buffer).
## Values (L_a cd/m^2 -> B, S): 1e4 and above 1, 1; 3264 0.79, 0.91; Sun -3 deg 181 0.44, 0.72; 79 and below
## 0.368; 4 and below S 0.52.
## B multiplies the whole displayed image (the cone gain's pre-exposure in the shaders x B: the rods, relative to that
## pixel, follow); S acts in the post step (VisionEffect) on the cones' image before the rods are added.
const APPEARANCE_BACKGROUND_PERCENT := 20.0  ## [S: CIE 159:2004] Y_b of an average scene (grey world)
## [call] The day look's luminance level: a white plate under ~3.1e4 lux (the Sun ~15 deg up in a clear sky); every
## daylight adaptation above it keeps B = S = 1 (noon at the pad: L_a 3.8e4).
const APPEARANCE_DAY_CD_M2 := 1.0e4
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
## The suprathreshold display factors this frame (brightness_factor, chroma_factor of the paced adaptation): 1 by day.
var brightness := 1.0
var chroma := 1.0
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
	brightness = brightness_factor(adaptation_cd_m2)
	chroma = chroma_factor(adaptation_cd_m2)


## The post step's chroma map C c = Y + chroma (c - Y) (Y = LUMINANCE_WEIGHTS . c, on every channel) ...
static func chroma_apply(c: Vector3, chroma_factor: float) -> Vector3:
	var y := LUMINANCE_WEIGHTS.dot(c)
	return Vector3.ONE * y + (c - Vector3.ONE * y) * chroma_factor


## ... and its inverse: (c - (1 - chroma) Y) / chroma (vision.gdshaderinc).
static func chroma_inverse(c: Vector3, chroma_factor: float) -> Vector3:
	return (c - Vector3.ONE * ((1.0 - chroma_factor) * LUMINANCE_WEIGHTS.dot(c))) / chroma_factor


## The whole post step on a scene pixel (VisionEffect): the cones' chroma, then the rods.
static func post_step(c: Vector3, scale: float, chroma_factor: float) -> Vector3:
	return chroma_apply(c, chroma_factor) + ROD_TINT * (scale * SCOTOPIC_WEIGHTS.dot(c))


## CIECAM02's luminance-level adaptation factor F_L for a scene white of this luminance (cd/m^2; L_A = white x Y_b / 100).
static func luminance_level_factor(white_cd_m2: float) -> float:
	var five_la := 5.0 * maxf(white_cd_m2, LUMINANCE_FLOOR_CD_M2) * APPEARANCE_BACKGROUND_PERCENT / 100.0
	var k4 := pow(1.0 / (five_la + 1.0), 4.0)
	return 0.2 * k4 * five_la + 0.1 * (1.0 - k4) * (1.0 - k4) * pow(five_la, 1.0 / 3.0)


## CIECAM02 brightness of the adapted white (J = 100) without the surround's constant 4 / c: (A_w + 4) F_L^0.25.
static func white_brightness(white_cd_m2: float) -> float:
	var f_l := luminance_level_factor(white_cd_m2)
	var x := pow(f_l, 0.42)
	var response := 400.0 * x / (27.13 + x) + 0.1  # R'_a = G'_a = B'_a of the adapted white
	var n_bb := 0.725 * pow(100.0 / APPEARANCE_BACKGROUND_PERCENT, 0.2)
	var a_w := (2.0 * response + response + response / 20.0 - 0.305) * n_bb
	return (a_w + 4.0) * pow(f_l, 0.25)


## The display's brightness factor B (Stevens effect): Q_w(L_a) / Q_w(L_day), L_a held at the Weber range's lower end
## below it; 1 at and above APPEARANCE_DAY_CD_M2.
static func brightness_factor(adaptation_cd_m2: float) -> float:
	var level := clampf(adaptation_cd_m2, pow(10.0, CONE_LOG_WEBER), APPEARANCE_DAY_CD_M2)
	return white_brightness(level) / white_brightness(APPEARANCE_DAY_CD_M2)


## The cones' chroma factor S (Hunt effect): (F_L(L_a) / F_L(L_day))^0.25, L_a held at the mesopic upper limit below
## it; 1 at and above APPEARANCE_DAY_CD_M2.
static func chroma_factor(adaptation_cd_m2: float) -> float:
	var level := clampf(adaptation_cd_m2, pow(10.0, MESOPIC_LOG_LOW + MESOPIC_LOG_SPAN), APPEARANCE_DAY_CD_M2)
	return pow(luminance_level_factor(level) / luminance_level_factor(APPEARANCE_DAY_CD_M2), 0.25)


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
