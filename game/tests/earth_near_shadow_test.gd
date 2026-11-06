# EarthNearShadow (moon procedural surface plan Task 16): the range rule, when the Earth's near shadows own the Sun light
# and the persisted setting. Headless, exit code 1 on failure. The shadows themselves: render_check --shadow_length (GPU).
# godot --headless --path game --script res://tests/earth_near_shadow_test.gd
extends SceneTree

var _failures := 0


func _check(name_: String, ok: bool, detail := "") -> void:
	print("earth_near_shadow_test: %s %s %s" % ["ok  " if ok else "FAIL", name_, detail])
	if not ok:
		_failures += 1


func _initialize() -> void:
	# Range: the tower's shadow x 1.25, quantised up in 2^(1/4) steps, at least 500 m, + 4 x the eye's height, <= 4 km.
	var e40 := EarthNearShadow.shadow_range_m(2.0, deg_to_rad(40.0))
	_check("Sun 40 deg on the ground: the 500 m floor + 4 x 2 m", is_equal_approx(e40, 508.0), "%.2f m" % e40)
	var e10 := EarthNearShadow.shadow_range_m(0.0, deg_to_rad(10.0))
	var shadow10 := 1.25 * EarthNearShadow.CASTER_HEIGHT_M / tan(deg_to_rad(10.0))
	_check("Sun 10 deg: covers 1.25 x the tower's shadow within one quantum step", e10 >= shadow10
			and e10 < shadow10 * pow(2.0, 0.25) + 1.0e-6, "%.1f m for %.1f m" % [e10, shadow10])
	var e10b := EarthNearShadow.shadow_range_m(0.0, deg_to_rad(10.2))
	_check("a slowly setting Sun keeps the quantised range", is_equal_approx(e10, e10b), "%.1f / %.1f m" % [e10, e10b])
	_check("high eye: at most 4 km", is_equal_approx(EarthNearShadow.shadow_range_m(5000.0, deg_to_rad(30.0)), 4000.0))
	_check("Sun on the horizon: at most 4 km", EarthNearShadow.shadow_range_m(0.0, 0.0) <= 4000.0)

	# Activity: on near the ground by day; off with the setting, the Moon's shadows active, high up, the Sun down.
	var light := DirectionalLight3D.new()
	var shadow := EarthNearShadow.new()
	var was := EarthNearShadow.enabled
	EarthNearShadow.enabled = true
	shadow.update(light, false, 2.0, deg_to_rad(30.0), Atmosphere.ASTRONOMICAL_UNIT_M)
	_check("on at the pad by day", shadow.active and light.shadow_enabled
			and absf(light.light_angular_distance - 0.533) < 0.002, "angular distance %.4f deg" % light.light_angular_distance)
	_check("shader global on with the fade range", (ShaderGlobals.value(&"earth_near_shadow") as Vector4).x == 1.0)
	_check("full strength up to 20 km", is_equal_approx(light.shadow_opacity, 1.0))
	shadow.update(light, false, 25000.0, deg_to_rad(30.0), Atmosphere.ASTRONOMICAL_UNIT_M)
	_check("fading between 20 and 30 km (no step)", shadow.active and light.shadow_enabled
			and absf(light.shadow_opacity - 0.5) < 1.0e-6, "opacity %.3f" % light.shadow_opacity)
	shadow.update(light, false, 20000.0, deg_to_rad(30.0), Atmosphere.ASTRONOMICAL_UNIT_M)
	var just_below := EarthNearShadow.strength(20000.0 + 1.0)
	_check("continuous at 20 km", shadow.active and is_equal_approx(light.shadow_opacity, 1.0) and just_below > 0.999999)
	shadow.update(light, false, 35000.0, deg_to_rad(30.0), Atmosphere.ASTRONOMICAL_UNIT_M)
	_check("off above 30 km", not shadow.active and not light.shadow_enabled and is_equal_approx(light.shadow_opacity, 1.0))
	shadow.update(light, false, 2.0, deg_to_rad(-5.0), Atmosphere.ASTRONOMICAL_UNIT_M)
	_check("off with the Sun down", not shadow.active and not light.shadow_enabled)
	light.shadow_enabled = true  # the Moon's settings on the light: left alone
	shadow.update(light, true, 2.0, deg_to_rad(30.0), Atmosphere.ASTRONOMICAL_UNIT_M)
	_check("the Moon's near shadows own the light", not shadow.active and light.shadow_enabled)
	# Final review m1: the eye leaves the Earth inside the fade (25 km) and the Moon's shadows take the light in the next
	# frame (teleport, site switch): the fade's opacity must not stay on the Moon's shadows.
	shadow.update(light, false, 25000.0, deg_to_rad(30.0), Atmosphere.ASTRONOMICAL_UNIT_M)
	shadow.update(light, true, 2.0, deg_to_rad(30.0), Atmosphere.ASTRONOMICAL_UNIT_M)
	_check("the Moon's shadows get full opacity after a fading Earth frame", is_equal_approx(light.shadow_opacity, 1.0),
			"opacity %.3f" % light.shadow_opacity)
	EarthNearShadow.enabled = false
	shadow.update(light, false, 2.0, deg_to_rad(30.0), Atmosphere.ASTRONOMICAL_UNIT_M)
	_check("off with the setting", not shadow.active and not light.shadow_enabled)
	EarthNearShadow.enabled = was
	light.free()

	# The persisted setting (a test file of its own: never the user's).
	var path := "user://earth_near_shadow_test_%d.cfg" % OS.get_process_id()
	EarthNearShadow.save_enabled_setting(false, path)
	var off := EarthNearShadow.load_enabled_setting(path)
	EarthNearShadow.save_enabled_setting(true, path)
	var on := EarthNearShadow.load_enabled_setting(path)
	DirAccess.remove_absolute(ProjectSettings.globalize_path(path))
	var missing := EarthNearShadow.load_enabled_setting(path)
	_check("setting earth_near_shadows persists", not off and on and missing, "off %s, on %s, no file %s" % [off, on, missing])

	print("earth_near_shadow_test: %s" % ("PASS" if _failures == 0 else "FAILED (%d)" % _failures))
	quit(1 if _failures > 0 else 0)
