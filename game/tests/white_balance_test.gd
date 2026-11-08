# The eye's chromatic adaptation to the Sun's white (EyeAdaptation.white_balance, look fixes T2; eye-adaptation.md):
# - the display's own white (RGB grey, D65) is left alone (identity within 1e-6);
# - a white Lambertian surface under the TOA Sun (the colour of the core's solar spectrum, as the game takes it from the
#   atmosphere tables) is drawn grey at its luminance (channels within 1e-6 relative): a sunlit cloud seen from space is
#   white [S: crew photographs and DSCOVR EPIC natural colour show sunlit cloud tops neutral];
# - the TOA Sun itself is warmer than D65 (B/R 0.80-0.90 [D: the core's spectrum in linear sRGB 0.856]) - the cause of
#   the salmon clouds before (start view: cloud radiance B/R 0.79-0.84 measured, look fixes T2 report);
# - light reddened by the air stays warm: the Sun through Rayleigh air mass 4 at 700 hPa (B/R 0.51 colourimetric) is
#   still clearly warm after the balance (B/R < 0.75) - the terminator and the sunset keep their colour;
# - the field's illuminant at the pad at noon (Sun 48 deg, clear) is the global horizontal daylight, Sun + sky with the
#   sky's true weight: its CCT within NOON_CCT_MIN_K .. NOON_CCT_MAX_K (look fixes T5b fix round 2, review I1);
# - the post step's map and the display-referred inverse (vision.gdshaderinc, Sherman-Morrison) cancel with rods,
#   chroma < 1 and the white balance together (within 1e-6).
# Headless: godot --headless --path game --script res://tests/white_balance_test.gd
extends SceneTree

const TOLERANCE := 1.0e-6
## [S: Hernandez-Andres, Romero, Nieves & Lee 2001, JOSA A 18(6) 1325 - clear-sky global daylight at Sun elevations
## above ~40 deg about 5500-6500 K; CIE 015:2018 D55 .. D65; Judd et al. 1964 daylight] The noon band of the global
## horizontal illuminant's CCT. [call] Lower edge 300 K below: the model's three-wavelength (680 / 550 / 440 nm)
## spectrum and its aerosol (tau 0.1 at 550 nm, Angstrom 1.3) give 5510 K (direct beam 4940 K, sky 10 500 K, sky 17.5 %
## of the horizontal illuminance).
const NOON_CCT_MIN_K := 5200.0
const NOON_CCT_MAX_K := 6500.0
## Starbase pad (as sunset_adaptation_test), the Sun at 48 deg elevation (noon in early November).
const PAD_LAT_DEG := 25.9967
const PAD_LON_DEG := -97.1543
const NOON_SUN_ELEVATION_DEG := 48.0

var _failures := 0


func _initialize() -> void:
	_run.call_deferred()


func _run() -> void:
	_check_identity()
	var core := StarsimCore.new()
	if not core.open(Paths.data_dir().path_join("spice")):
		push_error("SPICE kernels missing")
		quit(1)
		return
	var atmosphere := Atmosphere.new()
	atmosphere.start_loading(core, Paths.config_dir(), Paths.data_dir().path_join("atmosphere"))
	if not atmosphere.wait_until_loaded():
		push_error("atmosphere tables not loaded: " + atmosphere.status_text())
		quit(1)
		return
	var sun := Vector3(atmosphere.parameters["solar_illuminance_rgb"]) \
			* Vector3(atmosphere.parameters["sun_radiance_to_luminance"])
	_check_sun(sun)
	_check_reddened(sun)
	_check_inverse(EyeAdaptation.white_balance(sun))
	_check_degree()
	_check_partial(sun)
	_check_moonlight(sun, atmosphere.moon_light_colour())
	_check_pacing(sun)
	_check_inverse(EyeAdaptation.white_balance(sun * atmosphere.moon_light_colour(), EyeAdaptation.degree_of_adaptation(0.05)))
	_check_noon_illuminant(core, atmosphere, sun)
	atmosphere.shutdown()
	print("white_balance_test: %s (%d failures)" % ["PASS" if _failures == 0 else "FAIL", _failures])
	quit(1 if _failures > 0 else 0)


func _check(label: String, ok: bool, detail: String) -> void:
	if not ok:
		_failures += 1
	print("white_balance_test: %-44s %s %s" % [label, detail, "ok" if ok else "FAIL"])


func _check_identity() -> void:
	var w := EyeAdaptation.white_balance(Vector3(2.0, 2.0, 2.0))
	var worst := 0.0
	for k in range(3):
		var d: Vector3 = (w[k] - Basis.IDENTITY[k]).abs()
		worst = maxf(worst, maxf(d.x, maxf(d.y, d.z)))
	_check("D65 grey: identity", worst < TOLERANCE, "largest element error %s" % worst)


func _check_sun(sun: Vector3) -> void:
	var ratio := sun.z / sun.x
	_check("TOA Sun warmer than D65", ratio > 0.80 and ratio < 0.90, "linear sRGB %s, B/R %.4f" % [sun / sun.y, ratio])
	var plate := EyeAdaptation.white_balance(sun) * sun
	var y := EyeAdaptation.LUMINANCE_WEIGHTS.dot(sun)
	var worst := 0.0
	for k in range(3):
		worst = maxf(worst, absf(plate[k] / y - 1.0))
	_check("sunlit white -> grey at its luminance", worst < TOLERANCE,
			"balanced %s, luminance %.6f, largest channel error %s" % [plate / y, y, worst])


func _check_reddened(sun: Vector3) -> void:
	# The Sun's colour through Rayleigh air mass 4 at 700 hPa (optical depth 0.0088 lambda_um^-4.05 at sea level
	# [S: Bodhaine et al. 1999 fit]) at the tables' channel wavelengths (628, 546, 444 nm).
	var lambdas := Vector3(0.628, 0.546, 0.444)
	var t := Vector3.ZERO
	for k in range(3):
		t[k] = exp(-4.0 * 0.7 * 0.0088 * pow(lambdas[k], -4.05))
	var reddened := sun * t
	var balanced := EyeAdaptation.white_balance(sun) * reddened
	var ratio := balanced.z / balanced.x
	_check("air-reddened Sun stays warm", ratio < 0.75 and ratio > 0.3,
			"B/R %.3f before, %.3f balanced" % [reddened.z / reddened.x, ratio])


## CIECAM02's photopic D (F 1, average surround): 1 - e^(-42/92) / 3.6 = 0.8241 at L_A -> 0, 0.9022 at the white
## 270 cd/m^2 (the cabin, L_A 54), 1 within 1e-6 by day (4.3e4). The mesopic D (T5b fix round 3) = m D_photopic with
## CIE 191:2010's m: for S/P 1 L_mes = L_A, so m = 0.7670 + 0.3334 log10 L_A exactly (1e-9): 0.7670 at 1 cd/m^2,
## 0.4336 at 0.1, 0 at <= 0.005, 1 at >= 5.06; for S/P 3 the iteration's fixed point holds (1e-9). Photopic unchanged
## (D = D_photopic within 1e-12) for every white >= 25.3 cd/m^2 (L_A >= 5.06) at S/P 0.5-3.5 - noon, cabin, the lit
## Moon, space. Continuous (largest step 0.005 per 0.01 dex over 1e-4 .. 1e5 cd/m^2, m's slope 0.0033 per 0.01 dex
## x D_photopic) and increasing in L_a.
func _check_degree() -> void:
	var dark := EyeAdaptation.photopic_degree_of_adaptation(0.0)
	var cabin := EyeAdaptation.photopic_degree_of_adaptation(270.0)
	var day := EyeAdaptation.photopic_degree_of_adaptation(4.3e4)
	_check("photopic D at L_A 0", absf(dark - 0.8241) < 1.0e-4, "%.5f (0.8241)" % dark)
	_check("photopic D at 270 cd/m^2 (cabin)", absf(cabin - 0.9022) < 1.0e-4, "%.5f (0.9022)" % cabin)
	_check("photopic D by day", absf(day - 1.0) < 1.0e-6, "%.8f" % day)
	for l in [1.0, 0.1, 0.02, 2.0]:
		var m := EyeAdaptation.mesopic_photopic_weight(l, l)
		var expected := clampf(0.7670 + 0.3334 * log(l) / log(10.0), 0.0, 1.0)
		_check("CIE 191 m at L_mes %s (S/P 1)" % l, absf(m - expected) < 1.0e-9, "%.6f (%.6f)" % [m, expected])
	_check("CIE 191 m scotopic", EyeAdaptation.mesopic_photopic_weight(0.004, 0.004) == 0.0, "0 at 0.004 cd/m^2")
	_check("CIE 191 m photopic", EyeAdaptation.mesopic_photopic_weight(5.1, 5.1) == 1.0, "1 at 5.1 cd/m^2")
	var v0 := EyeAdaptation.MESOPIC_V_PRIME_LAMBDA0
	var l_p := 0.3
	var l_s := 0.9
	var m3 := EyeAdaptation.mesopic_photopic_weight(l_p, l_s)
	var l_mes := (m3 * l_p + (1.0 - m3) * l_s * v0) / (m3 + (1.0 - m3) * v0)
	_check("CIE 191 fixed point at S/P 3", absf(m3 - (0.7670 + 0.3334 * log(l_mes) / log(10.0))) < 1.0e-9,
			"m %.6f, L_mes %.5f" % [m3, l_mes])
	var unchanged := 0.0
	for sp in [0.5, 1.0, 2.4, 3.5]:
		for white in [25.3, 270.0, 4.3e4, 1.2e5]:
			unchanged = maxf(unchanged, absf(EyeAdaptation.degree_of_adaptation(white, white * sp)
					- EyeAdaptation.photopic_degree_of_adaptation(white)))
	_check("photopic D unchanged (white >= 25.3 cd/m^2)", unchanged < 1.0e-12, "largest %s" % unchanged)
	var worst_step := 0.0
	var monotonic := true
	for sp in [0.5, 1.0, 2.4, 3.0]:
		var last := -1.0
		for k in range(-400, 501):
			var white := pow(10.0, k * 0.01)
			var d := EyeAdaptation.degree_of_adaptation(white, white * sp)
			if last >= 0.0:
				worst_step = maxf(worst_step, absf(d - last))
				monotonic = monotonic and d >= last - 1.0e-12
			last = d
	_check("D continuous in L_a", worst_step < 0.005, "largest step %.5f per 0.01 dex" % worst_step)
	_check("D increasing in L_a", monotonic, "1e-4 .. 1e5 cd/m^2, S/P 0.5-3")
	var dusk := EyeAdaptation.degree_of_adaptation(12.2, 12.2 * 3.0)
	var night := EyeAdaptation.degree_of_adaptation(0.044, 0.044 * 2.0)
	print("  D: dusk (L_a 12.2, S/P 3) %.4f (photopic %.4f), full-Moon night (L_a 0.044, S/P 2) %.4f, moonless %.4f"
			% [dusk, EyeAdaptation.photopic_degree_of_adaptation(12.2), night,
			EyeAdaptation.degree_of_adaptation(1.0e-4, 3.0e-4)])
	_check("mesopic dusk D below the photopic", dusk < EyeAdaptation.photopic_degree_of_adaptation(12.2) - 0.05,
			"%.4f" % dusk)


## Partial adaptation: D = 0 is the identity, the cone gains are linear in D (CIE 159's D Y_w / R_w + 1 - D), D = 1 the
## former full balance.
func _check_partial(sun: Vector3) -> void:
	var none := EyeAdaptation.white_balance(sun, 0.0)
	var worst := 0.0
	for k in range(3):
		var d: Vector3 = (none[k] - Basis.IDENTITY[k]).abs()
		worst = maxf(worst, maxf(d.x, maxf(d.y, d.z)))
	_check("D 0: identity", worst < TOLERANCE, "largest element error %s" % worst)
	var full := EyeAdaptation.white_gains_of(sun, 1.0)
	var half := EyeAdaptation.white_gains_of(sun, 0.5)
	var expected := (full + Vector3.ONE) * 0.5
	_check("D 0.5: gains halfway", (half - expected).length() < TOLERANCE, "%s vs %s" % [half, expected])


## The full Moon seen from the ground at night (look fixes T5b review Q3): its light is the Sun's x the regolith's colour
## (moon_light_colour) x the air at elevation 83 deg (Rayleigh 0.0973 at 550 nm, lambda^-4.05; aerosol 0.1 at 550 nm,
## Angstrom 1.3 - the config's values, review Q3), and so is the night's illuminant (moonlight). The disc itself is
## photopic (~2500 cd/m^2): seen in range (a narrow frame, L_a ~2e4) the eye adapts with D 1 - neutral, 0.97-1.03; with
## the photopic floor D 0.82 pale cream, B/R 0.80-0.92 [E: observers' "pale cream / silvery"; review Q3 0.83-0.89] - kept
## as the check of the CAT itself. The night's mesopic D (T5b fix round 3, L_a 0.05, S/P 2: m ~0.13) leaves the
## moonlight close to its colourimetric warmth (between colourimetric and the photopic-floor cream); the rods' post step
## carries the night's look. Former fixed TOA-Sun white: B/R about 0.52.
func _check_moonlight(sun: Vector3, moon_colour: Vector3) -> void:
	var lambdas := Vector3(0.628, 0.546, 0.444)
	var air_mass := 1.0 / sin(deg_to_rad(83.0))
	var t := Vector3.ZERO
	for k in range(3):
		var tau := 0.0973 * pow(lambdas[k] / 0.55, -4.05) + 0.1 * pow(lambdas[k] / 0.55, -1.3)
		t[k] = exp(-air_mass * tau)
	var moonlight := sun * moon_colour * t
	var floor_d := EyeAdaptation.white_balance(moonlight, EyeAdaptation.photopic_degree_of_adaptation(0.05)) * moonlight
	var night := EyeAdaptation.white_balance(moonlight, EyeAdaptation.degree_of_adaptation(0.05, 0.1)) * moonlight
	var full := EyeAdaptation.white_balance(moonlight, 1.0) * moonlight
	var toa := EyeAdaptation.white_balance(sun, 1.0) * moonlight
	_check("full Moon at the photopic floor D: pale cream", floor_d.z / floor_d.x > 0.80 and floor_d.z / floor_d.x < 0.92,
			"B/R %.3f (D 1: %.3f, TOA white: %.3f, colourimetric %.3f)" % [floor_d.z / floor_d.x, full.z / full.x,
			toa.z / toa.x, moonlight.z / moonlight.x])
	_check("moonlight at the night's mesopic D: between colourimetric and the floor cream",
			night.z / night.x > moonlight.z / moonlight.x and night.z / night.x < floor_d.z / floor_d.x,
			"B/R %.3f (D %.4f)" % [night.z / night.x, EyeAdaptation.degree_of_adaptation(0.05, 0.1)])
	_check("moonlight at D 1: neutral", absf(full.z / full.x - 1.0) < 0.03, "B/R %.4f" % [full.z / full.x])
	_check("moonlight colour keeps the Sun's luminance", absf(EyeAdaptation.LUMINANCE_WEIGHTS.dot(sun * moon_colour)
			/ EyeAdaptation.LUMINANCE_WEIGHTS.dot(sun) - 1.0) < TOLERANCE, "%s" % moon_colour)
	# Cabin lamps (D65 white, CockpitView.LAMP_COLOUR): adopted at any D, a white lamp is the display's white.
	var lamp := EyeAdaptation.white_balance(Vector3.ONE, EyeAdaptation.degree_of_adaptation(270.0)) * Vector3.ONE
	_check("cabin lamp (D65) neutral", (lamp - Vector3.ONE).length() < TOLERANCE, "%s" % lamp)


## Pacing (EyeAdaptation.advance_white): from a snap on the TOA Sun to a D65 field, the log gains move 1 - e^-1 of the
## way in CHROMATIC_TIME_CONSTANT_S and 90 % in 60 s (Fairchild & Reniff 1995); a snap jumps.
func _check_pacing(sun: Vector3) -> void:
	var eye := EyeAdaptation.new()
	eye.adaptation_cd_m2 = 4.3e4  # D = 1
	eye.advance_white(sun, 0.0)
	var start := eye.white_gains
	eye.advance_white(Vector3.ONE, EyeAdaptation.CHROMATIC_TIME_CONSTANT_S)
	var share := log(eye.white_gains.z) / log(start.z)  # the target's log gain is 0 (D65)
	_check("pacing: one time constant", absf(share - exp(-1.0)) < 1.0e-9, "left %.6f of the log distance" % share)
	eye.advance_white(sun, 0.0)
	eye.snap()
	eye.advance_white(sun, 0.0)
	eye.advance_white(Vector3.ONE, 60.0)
	share = log(eye.white_gains.z) / log(start.z)
	_check("pacing: 90 % in 60 s", absf(share - 0.1) < 0.005, "left %.4f" % share)
	eye.snap()
	eye.advance_white(Vector3.ONE, 0.0)
	_check("snap jumps", (eye.white_gains - Vector3.ONE).length() < TOLERANCE, "%s" % eye.white_gains)


func _check_inverse(white: Basis) -> void:
	var scale := 0.3
	var chroma_factor := 0.7
	var cone := EyeAdaptation.cone_map(chroma_factor, white)
	var cone_inverse := cone.inverse()
	var tint_q := cone_inverse * EyeAdaptation.ROD_TINT
	var u := tint_q * (scale / (1.0 + scale * EyeAdaptation.SCOTOPIC_WEIGHTS.dot(tint_q)))
	var worst := 0.0
	for c in [Vector3(1, 0, 0), Vector3(0.2, 0.5, 0.9), Vector3(3.0, 1.0, 0.1)]:
		# the emitter's inverse (vision.gdshaderinc), then the post step (VisionEffect)
		var q: Vector3 = cone_inverse * c
		var written := q - u * EyeAdaptation.SCOTOPIC_WEIGHTS.dot(q)
		var shown := EyeAdaptation.post_step(written, scale, chroma_factor, white)
		worst = maxf(worst, (shown - c).length() / c.length())
	_check("post step o display-referred inverse = id", worst < TOLERANCE, "largest relative error %s" % worst)


## Correlated colour temperature of a linear Rec.709 colour (K) [S: McCamy 1992, Color Res. Appl. 17(2) 142; within
## ~2 K of the Planckian locus over 2856-6504 K].
static func _cct_k(rgb: Vector3) -> float:
	var x_ := 0.4124 * rgb.x + 0.3576 * rgb.y + 0.1805 * rgb.z
	var y_ := 0.2126 * rgb.x + 0.7152 * rgb.y + 0.0722 * rgb.z
	var z_ := 0.0193 * rgb.x + 0.1192 * rgb.y + 0.9505 * rgb.z
	var x := x_ / (x_ + y_ + z_)
	var y := y_ / (x_ + y_ + z_)
	var n := (x - 0.3320) / (0.1858 - y)
	return 449.0 * n * n * n + 3525.0 * n * n + 6823.3 * n + 5520.33


## The field's illuminant at the pad at noon (Atmosphere.field_illuminant after adaptation_luminance): the horizontal
## plate's global daylight - CCT in the noon band, bluer than the direct beam alone and warmer than the sky alone (the
## sky enters with its weight on the horizontal); the luminance plate (the Sun at normal incidence) unchanged.
func _check_noon_illuminant(core: StarsimCore, atmosphere: Atmosphere, toa: Vector3) -> void:
	var lat := deg_to_rad(PAD_LAT_DEG)
	var lon := deg_to_rad(PAD_LON_DEG)
	var up := Vector3(cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat))
	var north := Vector3(-sin(lat) * cos(lon), -sin(lat) * sin(lon), cos(lat))
	var e := deg_to_rad(NOON_SUN_ELEVATION_DEG)
	var sun_dir := (up * sin(e) + north * cos(e)).normalized()
	var eye := HeightSource.geodetic_to_fixed(PAD_LAT_DEG, PAD_LON_DEG, 2.0)
	var follows := EyeAdaptation.adopted_white_follows_scene
	EyeAdaptation.adopted_white_follows_scene = true
	atmosphere.adaptation_luminance(eye, sun_dir, 1.0, 1.0)
	var white := atmosphere.field_illuminant()
	EyeAdaptation.adopted_white_follows_scene = follows
	var lux: Dictionary = core.atmosphere_illuminance(eye, up, sun_dir)
	var direct: Vector3 = toa * core.atmosphere_sun_transmittance(eye, sun_dir)
	var sky: Vector3 = lux["sky_lux"]
	var cct := _cct_k(white)
	_check("noon illuminant: global daylight CCT", cct >= NOON_CCT_MIN_K and cct <= NOON_CCT_MAX_K,
			"%.0f K, B/R %.3f (band %.0f-%.0f K; direct beam %.0f K, sky %.0f K)" % [cct, white.z / white.x,
			NOON_CCT_MIN_K, NOON_CCT_MAX_K, _cct_k(direct), _cct_k(sky)])
	var global: Vector3 = lux["sun_lux"] + sky
	var expected := global / EyeAdaptation.LUMINANCE_WEIGHTS.dot(global)
	_check("noon illuminant = horizontal Sun + sky", (white - expected).length() < 1.0e-6,
			"%s vs %s (sky %.1f %% of the horizontal)" % [white, expected,
			100.0 * EyeAdaptation.LUMINANCE_WEIGHTS.dot(sky) / EyeAdaptation.LUMINANCE_WEIGHTS.dot(global)])
