# Headless test of the Pad B lamps (TowerLights, spec G): schedule (civil dusk to dawn), the FAA flash, the shadow budget,
# the deck illuminance of the rendered lights against the fixture model, the day state. Needs data/models/starbase_olp2.
# Run: godot --headless --path game --script res://tests/tower_lights_test.gd
extends SceneTree

const MODEL_SITE := "starbase_olp1"
const SCENE_PER_CD := 3.0e-4  ## a typical night value of the game's exposure (cone gain x brightness x day base)
const DECK_TOLERANCE := 0.30  ## plan Task 4: the lit deck's illuminance within 30 % of the fixture model
const FAA_FPM_MIN := 20.0  ## [S] AC 150/5345-43 L-864
const FAA_FPM_MAX := 40.0

var _failures := 0


func _initialize() -> void:
	_check_functions()
	var models := StructureModels.load_for(MODEL_SITE)
	var lamps := TowerLights.load_config()
	_check(not lamps.is_empty(), "lamps file loads")
	if models.is_empty() or lamps.is_empty():
		print("  (no models: view cases skipped; run python tools/structures/build_models.py)")
	else:
		_check_view(lamps)
	print("TOWER LIGHTS TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _check_functions() -> void:
	var dusk := -6.0
	var fade := 1.0
	_check(TowerLights.night_factor(deg_to_rad(30.0), dusk, fade) == 0.0, "noon: lamps off")
	_check(TowerLights.night_factor(deg_to_rad(-5.9), dusk, fade) == 0.0, "Sun -5.9 deg (before civil dusk): off")
	_check(TowerLights.night_factor(deg_to_rad(-6.5), dusk, fade) > 0.0, "Sun -6.5 deg (after civil dusk): on")
	_check(TowerLights.night_factor(deg_to_rad(-7.5), dusk, fade) == 1.0, "Sun -7.5 deg: full")
	_check(TowerLights.night_factor(deg_to_rad(-18.0), dusk, fade) == 1.0, "night: full")
	_check(is_equal_approx(TowerLights.candela_omni(4.0 * PI), 1.0), "omni: 4 pi lm = 1 cd")
	# The flux of the cone profile (rim linear in the solid angle) equals the lumens.
	var half := deg_to_rad(35.0)
	var axis := TowerLights.candela_cone(10000.0, half)
	var steps := 4000
	var flux := 0.0
	var omega := TAU * (1.0 - cos(half))
	for i in range(steps):
		var cos_a := 1.0 - (i + 0.5) / steps * (1.0 - cos(half))
		flux += axis * TowerLights.cone_factor(cos_a, half) * omega / steps
	_check(absf(flux / 10000.0 - 1.0) < 0.002, "cone profile flux %.4f of the lumens" % (flux / 10000.0))
	_check(is_equal_approx(TowerLights.energy_of(PI, 2.0), 2.0), "energy = cd / pi x exposure")
	# FAA flash: 30 per minute, counted over a minute at fine steps.
	var on_edges := 0
	var previous := false
	for i in range(6000):
		var on := TowerLights.flash_is_on(i * 0.01, 30.0, 0.5)
		if on and not previous:
			on_edges += 1
		previous = on
	_check(on_edges >= FAA_FPM_MIN and on_edges <= FAA_FPM_MAX and on_edges == 30, "flash rate %d per minute [S 20-40]" % on_edges)
	var white := TowerLights.cct_color(6500.0)
	_check(white.r > 0.93 and white.g > 0.93 and white.b > 0.93, "6500 K is near white (%s)" % white)
	_check(TowerLights.cct_color(5000.0).b < TowerLights.cct_color(5000.0).r, "5000 K is warmer than white")
	_check(TowerLights.cct_color(2700.0).b < 0.6 * TowerLights.cct_color(2700.0).r, "2700 K is warm")


func _check_view(lamps: Dictionary) -> void:
	var view := TowerView.new()
	view.site_id = MODEL_SITE
	root.add_child(view)
	var geometry := _geometry()
	view.build(geometry)
	var lights := view.tower_lights
	_check(lights != null, "view builds the lamps")
	if lights == null:
		return
	var sun_up: Vector3 = geometry["up"]  # the Sun straight overhead
	var sun_low: Vector3 = (geometry["up"] * -0.2 + geometry["east"]).normalized()  # -11.5 deg
	# Day: no lamp burns, no light adds to the scene; the obstruction lenses flash (white).
	view.update_lights(sun_up, SCENE_PER_CD, 0.0)
	_check(lights.burning_count() == 0, "day: no lamp burns")
	var day_energy := 0.0
	for light in lights.lights:
		day_energy += light.light_energy if light.visible else 0.0
	_check(day_energy == 0.0, "day: no light contributes (%f)" % day_energy)
	var obstruction := 0
	var flashing_day := 0
	for fixture in lights.fixtures():
		if fixture["kind"] == "obstruction":
			obstruction += 1
			flashing_day += 1 if (fixture["lens"] as MeshInstance3D).visible else 0
	_check(obstruction == 8 and flashing_day == obstruction, "day: all %d obstruction lenses flash on (%d)" % [obstruction, flashing_day])
	view.update_lights(sun_up, SCENE_PER_CD, 1.0)  # inside the dark part of the period
	var lit_off := 0
	for fixture in lights.fixtures():
		lit_off += 1 if fixture["kind"] == "obstruction" and (fixture["lens"] as MeshInstance3D).visible else 0
	_check(lit_off == 0, "flash: dark between flashes")
	# Night: everything burns, the shadow budget holds.
	view.update_lights(sun_low, SCENE_PER_CD, 0.0)
	var settings: Dictionary = lamps["lights"]
	var expected_burning := 0
	for fixture in lights.fixtures():
		expected_burning += 0 if fixture["kind"] == "obstruction" else 1
	_check(lights.burning_count() == expected_burning and expected_burning > 20, "night: %d lamps burn" % lights.burning_count())
	_check(lights.shadowed_count() == int(settings["max_shadowed"]), "shadow-casting lights: %d" % lights.shadowed_count())
	var shadow_enabled := 0
	for light in lights.lights:
		shadow_enabled += 1 if light.shadow_enabled else 0
	_check(shadow_enabled <= int(settings["max_shadowed"]), "never more than max_shadowed (%d)" % shadow_enabled)
	var red_lit := 0
	for fixture in lights.fixtures():
		if fixture["kind"] == "obstruction" and (fixture["light"] as Light3D).visible:
			red_lit += 1
			_check(absf(float(fixture["cd"]) - float(settings["red_cd"])) < 1.0e-6, "obstruction light: red_cd")
	_check(red_lit == obstruction, "night: the obstruction lights glow with the flash (%d)" % red_lit)
	# Deck illuminance: the rendered lights (energy -> cd -> Godot's attenuation and cone) against the fixture model.
	var deck := Vector3(0.0, 0.0, float(geometry["mount_height_m"]))
	var up := Vector3(0.0, 0.0, 1.0)
	var model_lux := 0.0
	var rendered_lux := 0.0
	for fixture in lights.fixtures():
		if fixture["kind"] == "obstruction":
			continue
		var to_deck: Vector3 = deck - fixture["site"]
		var distance := to_deck.length()
		var direction := to_deck / distance
		var cos_incidence := direction.dot(-up)  # a horizontal deck facing up: the light arrives from above
		var cd: float = fixture["cd"]
		var light: Light3D = fixture["light"]
		var rendered_cd := light.light_energy * PI / SCENE_PER_CD  # the lamp's axis intensity as set
		var window := TowerLights.godot_attenuation(distance, _range_of(light)) * distance * distance
		if fixture["kind"] == "flood":
			var aim: Vector3 = fixture["aim"]
			var half := deg_to_rad((light as SpotLight3D).spot_angle)
			var cos_off := aim.dot(direction)
			if cos_off <= cos(half):
				continue
			# fixture model: lumens evenly in the cone (axis value lumens / solid angle)
			var uniform_cd := _lumens_of(lamps, fixture) / (TAU * (1.0 - cos(half)))
			model_lux += TowerLights.illuminance_lux(uniform_cd, distance, cos_incidence)
			rendered_lux += TowerLights.illuminance_lux(rendered_cd * TowerLights.cone_factor(cos_off, half) * window,
					distance, cos_incidence)
		else:
			model_lux += TowerLights.illuminance_lux(cd, distance, cos_incidence)
			rendered_lux += TowerLights.illuminance_lux(rendered_cd * window, distance, cos_incidence)
	_check(model_lux > 5.0, "fixture model: %.1f lux on the mount deck (floodlit work area: 100-300 lux class)" % model_lux)
	var ratio := rendered_lux / model_lux
	_check(absf(ratio - 1.0) <= DECK_TOLERANCE, "rendered deck illuminance %.1f lux = %.3f x the fixture model (%.1f)" % [
			rendered_lux, ratio, model_lux])
	# The lamp light the eye adapts to: the deck's luminance under the model illuminance, 0 by day.
	_check(absf(lights.deck_luminance_cd - TowerLights.DECK_ALBEDO / PI * model_lux) < 1.0e-6 * model_lux,
			"adaptation luminance %.1f cd/m^2 = albedo / pi x the model deck lux" % lights.deck_luminance_cd)
	_check(lights.night_factor_now() == 1.0, "night factor 1 at Sun -11 deg")
	view.update_lights(sun_up, SCENE_PER_CD, 0.0)
	_check(lights.night_factor_now() == 0.0 and view.lamp_adaptation_cd_m2() == 0.0, "day: no lamp adaptation")
	view.update_lights(sun_low, SCENE_PER_CD, 0.0)
	# A different exposure scales the lights' energy: the lamps are physical light.
	var energy_before := (lights.lights[0] as Light3D).light_energy
	view.update_lights(sun_low, 2.0 * SCENE_PER_CD, 0.0)
	_check(absf((lights.lights[0] as Light3D).light_energy / energy_before - 2.0) < 1.0e-6, "energy follows the exposure")
	# Boxes site: no lamps.
	var boxes := TowerView.new()
	boxes.site_id = "lc39a"
	root.add_child(boxes)
	boxes.build(geometry)
	_check(boxes.tower_lights == null, "a box site has no lamps")


func _range_of(light: Light3D) -> float:
	return (light as SpotLight3D).spot_range if light is SpotLight3D else (light as OmniLight3D).omni_range


func _lumens_of(lamps: Dictionary, fixture: Dictionary) -> float:
	for group: Dictionary in lamps["lights.group"]:
		if group["name"] == fixture["group"]:
			return float(group["lumens"])
	return 0.0


func _check(ok: bool, what: String) -> void:
	if ok:
		print("  ok   ", what)
	else:
		print("  FAIL ", what)
		_failures += 1


## starbase_olp1's tower as FlightSim.site_tower reports it (site frame), as tower_view_test.gd.
func _geometry() -> Dictionary:
	var azimuth := deg_to_rad(315.0)
	var toward := Vector3(sin(azimuth), cos(azimuth), 0.0)
	var across := Vector3(-toward.y, toward.x, 0.0)
	var up := Vector3(0.0, 0.0, 1.0)
	var centre := toward * 30.0 + up * 73.0
	return {
		"present": true,
		"origin_itrf_m": Vector3(-5.5e6, -0.7e6, 2.8e6),
		"east": Vector3(0.0, 1.0, 0.0), "north": Vector3(0.0, 0.0, 1.0), "up": Vector3(1.0, 0.0, 0.0),
		"tower_direction": toward, "across": across,
		"tower_center_m": centre, "tower_size_m": Vector3(12.0, 12.0, 146.0),
		"mount_height_m": 20.0, "mount_inner_radius_m": 5.5, "mount_outer_radius_m": 13.0,
		"arm_length_m": 36.0, "arm_width_m": 2.0, "arm_thickness_m": 2.5, "arm_open_rad": deg_to_rad(40.0),
		"hinge_plus_m": toward * 24.0 + across * 5.8 + up * 95.0,
		"hinge_minus_m": toward * 24.0 - across * 5.8 + up * 95.0,
	}
