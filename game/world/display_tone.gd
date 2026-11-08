class_name DisplayTone
extends RefCounted
## The display tone step (look fixes T5, 2026-11-07): a chroma-preserving tone curve in place of AgX's per-channel
## desaturation, with the daylight Hunt factor. CPU mirror of DisplayToneEffect's shader (tests, offline numbers);
## the two must stay identical.
##
## Why: Godot's AgX (tonemap.glsl: inset matrix, the allenwp curve per channel, outset) takes 16-26 % of the noon sky's and
## the ocean's colour ratio B/R at mid-high scene values and turns their OkLab hue by ~7 deg (look investigation
## 2026-11-07, F3). A plain luminance curve keeps B/R but its clip (a channel above 1 pulled toward grey in linear RGB)
## turns saturated blues violet. Here, per pixel x (scene-linear Rec.709 after the eye's post step):
## 1. the luminance Y = BT.709 . x goes through AgX's own grey curve (the allenwp curve, Godot's contrast and white):
##    g = awp(Y) - the tonality of the former image on the grey axis, unchanged;
## 2. the colour keeps its chromaticity: d0 = x g / Y. OkLab [S: B. Ottosson 2020, "A perceptual color space for image
##    processing", bottosson.github.io/posts/oklab] is homogeneous of degree 1/3, so d0 has x's OkLab hue exactly and
##    its chroma / lightness ratio;
## 3. the daylight Hunt factor (EyeAdaptation.display_hunt, >= 1): OkLab chroma x S at constant lightness and hue;
## 4. path to white [call]: the chroma fades to 0 (smoothstep in log2 Y) from PATH_TO_WHITE_START (scene 2, above a
##    sunlit white plate at DAY_EXPOSURE 1.5) to AgX's white (16.29) - the Sun's disc, glints and the brightest highlights
##    end in display white as with AgX;
## 5. gamut mapping at constant OkLab lightness and hue [call]: C_max(L, h) is the largest chroma AgX can still show
##    (in the sRGB cube and reachable from a non-negative AgX input, bisection); C / C_max is compressed softly above
##    GAMUT_KNEE (exponential shoulder toward 1, C1 at the knee) - no per-channel clip, so no hue shift toward violet;
## 6. pre-distortion: the step writes P(d) = INSET^-1 awp^-1(OUTSET^-1 d), the scene value Godot's AgX maps to d exactly
##    (AgX stays the engine's tone mapper and the sRGB encoding is unchanged; on the grey axis P(d) = x). Above AgX's
##    white (Y >= white, display white) the pixel is lifted along grey to at least white in every AgX channel instead,
##    x + max(0, white - min(INSET x)) (INSET maps grey to grey): white on screen, its largest channel at most doubled
##    (max x >= Y >= white).
## Glow (fix round 1, review I1): Godot's glow reads this buffer before AgX with a largest-channel threshold
## (copy.glsl). P(d) is steep near AgX's gamut edge (awp^-1(0.95) = 8.8), so a saturated display colour can ask for a
## channel far above the scene's (the noon aureole: scene max 2.35 -> 4.9-9.35, a blue glow that was not there). The
## Hunt boost of step 3 is therefore bounded by the glow: the boosted chroma is gamut-mapped into the colours whose
## largest written channel stays at or below max(the glow threshold, the scene's largest channel), but never below the
## chroma the step shows without the boost (unbounded). The Hunt boost then starts no glow and strengthens none (written
## <= max(threshold, max x, P(the S = 1 colour))); at S <= 1 the step is as without the bound, so display-referred
## emitters (whose S = 1 colour is AgX's colour of their former value c, P = c) keep their former glow (glow_bound; INF
## while glow is off).
## Display-referred emitters (vision.gdshaderinc) take the inverse of the step (tone_inverse, then the Hunt factor's
## OkLab chroma / S): they are shown as AgX showed them before - within ~1 % alone on their pixel (the Hunt inverse
## changes Y at constant OkLab lightness), approximate where they blend with the scene (the step is not linear).

## [S: Ottosson 2020] linear sRGB -> LMS and LMS^(1/3) -> OkLab (Basis columns = the matrix's columns).
const OKLAB_M1 := Basis(Vector3(0.4122214708, 0.2119034982, 0.0883024619),
		Vector3(0.5363325363, 0.6806995451, 0.2817188376), Vector3(0.0514459929, 0.1073969566, 0.6299787005))
const OKLAB_M2 := Basis(Vector3(0.2104542553, 1.9779984951, 0.0259040371),
		Vector3(0.7936177850, -2.4285922050, 0.7827717662), Vector3(-0.0040720468, 0.4505937099, -0.8086757660))
## [S: Godot 4.7 servers/rendering/renderer_rd/shaders/effects/tonemap.glsl tonemap_agx] the combined Rec.709 ->
## Rec.2020 -> AgX inset matrix and the AgX outset -> Rec.709 matrix (GLSL mat3 columns = Basis columns).
const AGX_INSET := Basis(Vector3(0.544814746488245, 0.140416948464053, 0.0888104196149096),
		Vector3(0.373787398372697, 0.754137554567394, 0.178871756420858),
		Vector3(0.0813978551390581, 0.105445496968552, 0.732317823964232))
const AGX_OUTSET := Basis(Vector3(1.96488741169489, -0.299313364904742, -0.164352742528393),
		Vector3(-0.855988495690215, 1.32639796461980, -0.238183969428088),
		Vector3(-0.108898916004672, -0.0270845997150571, 1.40253671195648))
## [S: Godot 4.7 tonemap.glsl allenwp_curve, environment_storage.cpp] the curve's crossover point (18 % grey).
const AWP_CROSSOVER := 0.18
## [S: ITU-R BT.709] luminance weights.
const LUMINANCE_WEIGHTS := Vector3(0.2126, 0.7152, 0.0722)
## [call] Scene luminance where the path to white starts (above the sunlit white plate, DAY_EXPOSURE 1.5).
const PATH_TO_WHITE_START := 2.0
## [call] Share of the gamut's chroma kept linear; above it the soft shoulder.
const GAMUT_KNEE := 0.75
## Bisection steps for C_max (0.4 / 2^12 = 1e-4 in OkLab chroma) and its start (sRGB's largest OkLab chroma is 0.32).
const GAMUT_STEPS := 12
const GAMUT_CHROMA_START := 0.4
## Numerical guards: the curve's inverse below its asymptote, the reachability test's slack (relative).
const CURVE_INVERSE_MAX := 0.999999
const REACH_EPSILON := 1e-5
## OkLab chroma below which a colour is taken as grey (the shader's float32 threshold, the same on the CPU).
const CHROMA_MIN := 1e-7

## Fidelity flag: the chroma-preserving step (true) or Godot's AgX alone (false; render_check --tone=agx).
static var enabled := true

## The fixed display mapping of step 1 (look fixes T13, unified appearance model): the grey curve g(Y) the step shows a
## scene luminance at. Colourimetric (relative) reproduction [S: R. W. G. Hunt, "The Reproduction of Colour", 6th ed.
## 2004, ch. 11.7: equal relative luminances for an observer adapted to the display's white as to the scene's]: the eye's
## adopted white renders at the scene units' white plate Atmosphere.DAY_EXPOSURE (EyeAdaptation, field rule), so
## g = Y / DAY_EXPOSURE - the adopted white at display white, the field (Y_b = 20 % of it) at 0.2 - up to the knee;
## above it an exponential shoulder, C1 at the knee, toward display white (highlights the display cannot show: glints,
## the Sun's disc, clouds in backscatter, locally adapted regions' brightest parts). It replaces AgX's own grey curve
## (allenwp: +1.0 EV on the field, the white plate at 0.69 and a ~10:1 cloud / sea contrast compressed to ~4:1, the
## Earth from space too white against DSCOVR EPIC; look fixes T7 step 2). AgX stays the engine's encoder (step 6).
## [S: ITU-R BT.2035 (2013): the reference display white 100 cd/m^2 is the display's white; DISPLAY_WHITE_CD_M2]
## The slope: display per scene luminance below the knee (1 / DAY_EXPOSURE: the white plate at display white).
const DISPLAY_SLOPE := 1.0 / Atmosphere.DAY_EXPOSURE
## [call: SDR camera practice puts the knee point at ~80-90 % of the signal; here in display-linear luminance] The
## display value where the shoulder starts.
const DISPLAY_KNEE := 0.8
## Fidelity flag: the fixed display curve (true) or AgX's grey curve as before T13 (false; --display_curve=agx).
static var display_linear := not OS.get_cmdline_user_args().has("--display_curve=agx")


## Step 1's grey curve: the fixed display mapping (linear to the knee, exponential shoulder) or AgX's (display_linear off).
static func display_curve(y: float, p: PackedFloat64Array) -> float:
	if not display_linear:
		return curve(y, p)
	y = maxf(y, 0.0)
	var linear := DISPLAY_SLOPE * y
	if linear <= DISPLAY_KNEE:
		return linear
	return 1.0 - (1.0 - DISPLAY_KNEE) * exp(-(linear - DISPLAY_KNEE) / (1.0 - DISPLAY_KNEE))


## The display curve's inverse on [0, 1).
static func display_curve_inverse(d: float, p: PackedFloat64Array) -> float:
	if not display_linear:
		return curve_inverse(d, p)
	d = clampf(d, 0.0, CURVE_INVERSE_MAX)
	if d <= DISPLAY_KNEE:
		return d / DISPLAY_SLOPE
	return (DISPLAY_KNEE - (1.0 - DISPLAY_KNEE) * log((1.0 - d) / (1.0 - DISPLAY_KNEE))) / DISPLAY_SLOPE


## The allenwp curve's parameters for an AgX contrast and white (Godot's environment_storage.cpp, SDR output max 1):
## [contrast, toe_a, slope, w, white].
static func curve_parameters(contrast: float, agx_white: float) -> PackedFloat64Array:
	var white := maxf(2.0, agx_white)
	var x0 := AWP_CROSSOVER
	var toe_a := ((1.0 / x0) - 1.0) * pow(x0, contrast)
	var denominator := pow(x0, contrast) + toe_a
	var slope := (contrast * pow(x0, contrast - 1.0) * toe_a) / (denominator * denominator)
	var w := (white - x0) * (white - x0) / (1.0 - x0) * slope
	return PackedFloat64Array([contrast, toe_a, slope, w, white])


## AgX's per-channel curve (allenwp, output max 1), clipped at 1 as in tonemap_agx.
static func curve(x: float, p: PackedFloat64Array) -> float:
	x = maxf(x, 0.0)
	if x < AWP_CROSSOVER:
		var t := pow(x, p[0])
		return t / (t + p[1])
	var s := x - AWP_CROSSOVER
	var slope_s := p[2] * s
	return minf(slope_s * (1.0 + s / p[3]) / (1.0 + slope_s / (1.0 - AWP_CROSSOVER)) + AWP_CROSSOVER, 1.0)


## The curve's inverse on [0, 1): the toe's closed form, the shoulder's quadratic (stable root).
static func curve_inverse(y: float, p: PackedFloat64Array) -> float:
	y = clampf(y, 0.0, CURVE_INVERSE_MAX)
	if y < AWP_CROSSOVER:
		return pow(p[1] * y / (1.0 - y), 1.0 / p[0])
	var d := y - AWP_CROSSOVER
	var a := p[2] / p[3]
	var b := p[2] * (1.0 - d / (1.0 - AWP_CROSSOVER))
	return 2.0 * d / (b + sqrt(maxf(b * b + 4.0 * a * d, 0.0))) + AWP_CROSSOVER


## Godot's AgX on a scene value (tonemap_agx, without the engine's exposure / glow): the display-linear colour.
static func agx(x: Vector3, p: PackedFloat64Array) -> Vector3:
	var c := AGX_INSET * x.max(Vector3.ZERO)
	return AGX_OUTSET * Vector3(curve(c.x, p), curve(c.y, p), curve(c.z, p))


## The scene value AgX maps to the display-linear colour d.
static func predistort(d: Vector3, p: PackedFloat64Array) -> Vector3:
	var u := AGX_OUTSET.inverse() * d
	return AGX_INSET.inverse() * Vector3(curve_inverse(u.x, p), curve_inverse(u.y, p), curve_inverse(u.z, p))


static func _cbrt(v: float) -> float:
	return signf(v) * pow(absf(v), 1.0 / 3.0)


static func to_oklab(c: Vector3) -> Vector3:
	var lms := OKLAB_M1 * c
	return OKLAB_M2 * Vector3(_cbrt(lms.x), _cbrt(lms.y), _cbrt(lms.z))


static func from_oklab(lab: Vector3) -> Vector3:
	var lms := OKLAB_M2.inverse() * lab
	return OKLAB_M1.inverse() * (lms * lms * lms)


## Whether AgX can show d: in the sRGB cube, AgX's curve outputs u = OUTSET^-1 d non-negative, reached from a
## non-negative scene value whose largest channel is at most bound (the glow bound; slack relative to the colour's size:
## the GPU's float32 at bright values, exact near black).
static func reachable(d: Vector3, p: PackedFloat64Array, bound := INF) -> bool:
	var slack := REACH_EPSILON * maxf(d.x, maxf(d.y, d.z))
	if d.x < -slack or d.y < -slack or d.z < -slack or d.x > 1.0 + REACH_EPSILON or d.y > 1.0 + REACH_EPSILON 			or d.z > 1.0 + REACH_EPSILON:
		return false
	var u := AGX_OUTSET.inverse() * d
	if u.x < -slack or u.y < -slack or u.z < -slack:
		return false
	var x := predistort(d, p)
	var largest := maxf(x.x, maxf(x.y, x.z))
	slack = REACH_EPSILON * largest
	return x.x >= -slack and x.y >= -slack and x.z >= -slack and largest <= bound * (1.0 + REACH_EPSILON)


## The largest OkLab chroma AgX can show at this lightness and hue direction (cos h, sin h) within the glow bound.
static func chroma_max(lightness: float, hue: Vector2, p: PackedFloat64Array, bound := INF) -> float:
	var lo := 0.0
	var hi := GAMUT_CHROMA_START
	for i in range(GAMUT_STEPS):
		var mid := 0.5 * (lo + hi)
		if reachable(from_oklab(Vector3(lightness, mid * hue.x, mid * hue.y)), p, bound):
			lo = mid
		else:
			hi = mid
	return lo


## The soft gamut shoulder on r = C / C_max: linear below the knee, 1 - (1 - k) exp(-(r - k) / (1 - k)) above.
static func gamut_shoulder(r: float) -> float:
	if r < GAMUT_KNEE:
		return r
	return 1.0 - (1.0 - GAMUT_KNEE) * exp(-(r - GAMUT_KNEE) / (1.0 - GAMUT_KNEE))


## The glow bound of the environment (header, Glow): the largest channel Godot's glow does not pick up - its HDR
## threshold over the glow strength (copy.glsl compares max(rgb) x strength); INF while glow is off.
static func glow_bound(environment: Environment) -> float:
	if environment == null or not environment.glow_enabled:
		return INF
	return environment.glow_hdr_threshold / maxf(environment.glow_strength, 1e-6)


## The display-linear colour the step shows for a scene value x (steps 1-5 above) with the Hunt factor hunt (>= 1) and
## the glow threshold glow (header, Glow: the Hunt boost only as far as no written channel exceeds max(glow, max x)).
static func target(x: Vector3, hunt: float, p: PackedFloat64Array, glow := INF) -> Vector3:
	x = x.max(Vector3.ZERO)
	var bound := maxf(glow, maxf(x.x, maxf(x.y, x.z)))
	var y := LUMINANCE_WEIGHTS.dot(x)
	if y <= 0.0:
		return Vector3.ZERO
	var white := p[4]
	if y >= white:
		return Vector3.ONE
	var g := display_curve(y, p)
	var lab := to_oklab(x * (g / y))
	var lightness := minf(lab.x, 1.0)
	var c := Vector2(lab.y, lab.z).length()
	if c < CHROMA_MIN:
		return Vector3.ONE * g
	var hue := Vector2(lab.y, lab.z) / c
	var t := clampf((log(y / PATH_TO_WHITE_START)) / log(white / PATH_TO_WHITE_START), 0.0, 1.0)
	c *= 1.0 - t * t * (3.0 - 2.0 * t)
	var shown := gamut_map(lightness, hue, c * hunt, p, bound)
	# The glow bound limits only the Hunt boost: never less chroma than the step without it (unbounded) shows.
	if shown < c:
		shown = maxf(shown, gamut_map(lightness, hue, c, p))
	return from_oklab(Vector3(lightness, shown * hue.x, shown * hue.y)).clamp(Vector3.ZERO, Vector3.ONE)


## Step 5 on an OkLab chroma c at this lightness and hue within the bound: kept below the gamut's knee (C_max >= C / knee:
## one test, as the shader), else the soft shoulder on C / C_max.
static func gamut_map(lightness: float, hue: Vector2, c: float, p: PackedFloat64Array, bound := INF) -> float:
	if reachable(from_oklab(Vector3(lightness, hue.x * c / GAMUT_KNEE, hue.y * c / GAMUT_KNEE)), p, bound):
		return c
	var c_max := chroma_max(lightness, hue, p, bound)
	return gamut_shoulder(c / c_max) * c_max if c_max > 1e-9 else 0.0


## The whole step on a scene pixel: the value it writes for AgX (step 6 above), glow the glow bound.
static func apply(x: Vector3, hunt: float, p: PackedFloat64Array, glow := INF) -> Vector3:
	x = x.max(Vector3.ZERO)
	var y := LUMINANCE_WEIGHTS.dot(x)
	if y <= 0.0:
		return Vector3.ZERO
	if y >= p[4]:
		var c := AGX_INSET * x
		return x + Vector3.ONE * maxf(0.0, p[4] - minf(c.x, minf(c.y, c.z)))
	return predistort(target(x, hunt, p, glow), p).max(Vector3.ZERO)


## The Hunt factor's inverse for a display-referred emitter (vision.gdshaderinc): OkLab chroma / hunt.
static func hunt_inverse(c: Vector3, hunt: float) -> Vector3:
	if hunt == 1.0:
		return c
	var lab := to_oklab(c)
	return from_oklab(Vector3(lab.x, lab.y / hunt, lab.z / hunt))


## The display-referred emitters' inverse of the tone step (vision.gdshaderinc vision_tone_inverse, with hunt_inverse
## after it): the scene value the step shows as AgX would have shown c - AgX's colour d = AgX(c), scaled to the scene
## luminance whose grey the curve shows at d's luminance (the step keeps the chromaticity below its gamut knee and the
## path to white). Exact (with hunt_inverse after it within ~1 %) for an emitter alone on its pixel below scene 2 and
## inside the knee; at and above AgX's white c itself (white either way).
static func tone_inverse(c: Vector3, p: PackedFloat64Array) -> Vector3:
	c = c.max(Vector3.ZERO)
	if LUMINANCE_WEIGHTS.dot(c) >= p[4]:
		return c
	var d := agx(c, p).clamp(Vector3.ZERO, Vector3.ONE)
	var y_d := LUMINANCE_WEIGHTS.dot(d)
	if y_d <= 0.0:
		return Vector3.ZERO
	return d * (display_curve_inverse(y_d, p) / y_d)


## OkLab hue angle (rad) of a colour.
static func hue_angle(c: Vector3) -> float:
	var lab := to_oklab(c)
	return atan2(lab.z, lab.y)
