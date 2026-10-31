class_name TowerLights
extends Node3D
## The lamps of a Pad B site (spec G): floodlight banks, tower and deck lamps as Godot spot / omni lights in physical units,
## obstruction lights (FAA flash), lenses as emissive discs (light_lens.gdshader, far away the emissive point).
## Fixtures come from config/structures/starbase_olp2_lights.toml. A lamp's luminous flux -> luminous intensity (cd) ->
## the light's energy like the Sun's (Atmosphere.sun_light_at): Godot's attenuation 1 / d^2 with a white Lambertian
## surface returns energy x cos / d^2 as scene value, physics gives L = I cos / (pi d^2) cd/m^2, so
## energy = I / pi x scene_per_cd_m2 (the one exposure incl. the eye adaptation's gain: the adaptation sees the lamps
## as any other physical light, only through what it sees). Local frame: the site's render axes (TowerView's own).

const LIGHTS_FILE := "structures/starbase_olp2_lights.toml"
const LENS_SHADER := preload("res://shaders/light_lens.gdshader")
## [E] Godot's spot-cone falloff 1 - rim^k (rim 0 on the axis, 1 at the edge): k = 4 gives a flat-topped flood beam with a
## soft rim, like a lens optic (the default 1 would peak on the axis and miss the flux model by a third).
const ANGLE_FALLOFF := 4.0
const MIN_ENERGY_CHANGE := 1.0e-3  ## relative change of the exposure that re-sets the light energies
## [E] Lens housing: a dark box round the lens, this much deeper than wide (a cosmetic, silhouette only).
const HOUSING_DEPTH_FACTOR := 0.6
const HOUSING_OVERSIZE := 1.5
## Red lens: linear colour of a red-filtered beacon (hue only; the intensity is the AC's cd).
const RED := Vector3(1.0, 0.04, 0.02)

var config: Dictionary = {}
var lights: Array[Light3D] = []
var lenses: Array[MeshInstance3D] = []
var _fixtures: Array[Dictionary] = []  ## per fixture: {kind, light, lens, cd, tint, position (site frame), aim, lumens}
var _night_factor := 0.0
var _flash_on := false
var _last_scene_per_cd := -1.0
var _lens_mesh := QuadMesh.new()
var _housing_material: Material = null
var _shadowed := 0
## Luminance (cd/m^2) of a lit white-ish surface (albedo DECK_ALBEDO) on the mount deck under all lamps at full power: the
## lamp light the eye adapts to when it looks at the site. From the fixture model (file lumens), set by build().
var deck_luminance_cd := 0.0
## [E] Weathered concrete / steel deck: a dark-grey surface (the lit mount in padb_night_floodlights_mount reads mid-grey).
const DECK_ALBEDO := 0.3


## Reads the lamps file; {} without it.
static func load_config() -> Dictionary:
	var path := Paths.config_dir().path_join(LIGHTS_FILE)
	if not FileAccess.file_exists(path):
		return {}
	return TomlLite.load_file(path)


# --- physics (static, tested) -----------------------------------------------------------------------------------------

## Luminous intensity (cd) on the axis of a lamp of `lumens` in a cone of half angle `half_angle_rad` whose profile is
## Godot's 1 - rim^k (rim linear in the solid angle: mean 1 - 1 / (k + 1) of the axis value): the profile's flux is `lumens`.
static func candela_cone(lumens: float, half_angle_rad: float) -> float:
	return lumens / (TAU * (1.0 - cos(half_angle_rad)) * (1.0 - 1.0 / (ANGLE_FALLOFF + 1.0)))


## Luminous intensity (cd) of an omnidirectional lamp.
static func candela_omni(lumens: float) -> float:
	return lumens / (4.0 * PI)


## Light energy (Godot, attenuation 2) of a lamp of `cd` for the game's exposure `scene_per_cd_m2` (see the class comment).
static func energy_of(cd: float, scene_per_cd_m2: float) -> float:
	return cd / PI * scene_per_cd_m2


## Illuminance (lux) of a lamp of `cd` at `distance_m`, the surface turned `cos_incidence` to the beam (the fixture model).
static func illuminance_lux(cd: float, distance_m: float, cos_incidence: float) -> float:
	return cd * maxf(cos_incidence, 0.0) / (distance_m * distance_m)


## Godot's omni / spot attenuation (scene_forward_lights_inc.glsl get_omni_attenuation) at `distance_m` for `range_m`, decay 2.
static func godot_attenuation(distance_m: float, range_m: float) -> float:
	var nd := pow(distance_m / range_m, 4.0)
	var window := maxf(1.0 - nd, 0.0)
	return window * window / (distance_m * distance_m)


## Godot's spot-cone factor at `cos_angle` off the axis for a cone of half angle `half_angle_rad`.
static func cone_factor(cos_angle: float, half_angle_rad: float) -> float:
	var cone := cos(half_angle_rad)
	var rim := maxf(1.0e-4, (1.0 - maxf(cos_angle, cone)) / (1.0 - cone))
	return 1.0 - pow(rim, ANGLE_FALLOFF)


## 0 by day, 1 at `dusk_deg + fade_deg` below the Sun: lamps burn between civil dusk and dawn (Sun's elevation, rad).
static func night_factor(sun_elevation_rad: float, dusk_deg: float, fade_deg: float) -> float:
	var depression := rad_to_deg(-sun_elevation_rad) + dusk_deg  # dusk_deg is negative: 0 at the threshold
	return clampf(depression / fade_deg, 0.0, 1.0)


## FAA beacon: on for `on_s` at the start of each period (60 / flash_per_min s), t_s the clock.
static func flash_is_on(t_s: float, flash_per_min: float, on_s: float) -> bool:
	var period := 60.0 / flash_per_min
	return fposmod(t_s, period) < on_s


## Colour of a black body (linear sRGB, max channel 1): Kim et al. 2002 chromaticity (1667-25000 K), sRGB primaries.
static func cct_color(kelvin: float) -> Color:
	var t := clampf(kelvin, 1667.0, 25000.0)
	var x := 0.0
	if t <= 4000.0:
		x = -0.2661239e9 / (t * t * t) - 0.2343589e6 / (t * t) + 0.8776956e3 / t + 0.179910
	else:
		x = -3.0258469e9 / (t * t * t) + 2.1070379e6 / (t * t) + 0.2226347e3 / t + 0.240390
	var y := 0.0
	if t <= 2222.0:
		y = -1.1063814 * x * x * x - 1.34811020 * x * x + 2.18555832 * x - 0.20219683
	elif t <= 4000.0:
		y = -0.9549476 * x * x * x - 1.37418593 * x * x + 2.09137015 * x - 0.16748867
	else:
		y = 3.0817580 * x * x * x - 5.87338670 * x * x + 3.75112997 * x - 0.37001483
	var big_y := 1.0
	var big_x := x / y * big_y
	var big_z := (1.0 - x - y) / y * big_y
	var r := 3.2404542 * big_x - 1.5371385 * big_y - 0.4985314 * big_z
	var g := -0.9692660 * big_x + 1.8760108 * big_y + 0.0415560 * big_z
	var b := 0.0556434 * big_x - 0.2040259 * big_y + 1.0572252 * big_z
	var peak := maxf(r, maxf(g, b))
	return Color(maxf(r, 0.0) / peak, maxf(g, 0.0) / peak, maxf(b, 0.0) / peak)


# --- build ---------------------------------------------------------------------------------------------------------------

## Fixtures of the file at the tower of `geometry` (FlightSim.site_tower), in this node's site-render frame.
func build(lamps: Dictionary, geometry: Dictionary) -> void:
	for child in get_children():
		remove_child(child)
		child.queue_free()
	lights.clear()
	lenses.clear()
	_fixtures.clear()
	_shadowed = 0
	config = lamps
	_lens_mesh.size = Vector2(1.0, 1.0)
	_housing_material = LitAerial.material(Color(0.05, 0.05, 0.055), 0.6, 0.4)
	var toward: Vector3 = geometry["tower_direction"]
	var across: Vector3 = geometry["across"]
	var size: Vector3 = geometry["tower_size_m"]
	var centre: Vector3 = geometry["tower_center_m"]
	var face_m: float = centre.dot(toward) - 0.5 * size.x
	var settings: Dictionary = lamps["lights"]
	var budget := int(settings["max_shadowed"])
	for group: Dictionary in lamps["lights.group"]:
		var ref_m := face_m if str(group.get("ref", "face")) == "face" else 0.0
		var mirrors := [1.0, -1.0] if bool(group.get("mirror", false)) else [1.0]
		var count := int(group.get("count", 1))
		for side: float in mirrors:
			for i in range(count):
				var along: float = float(group["along_m"])
				var lateral: float = float(group["across_m"]) + i * float(group.get("step_across_m", 0.0))
				var z: float = float(group["z_m"]) + i * float(group.get("step_z_m", 0.0))
				var site := toward * (ref_m + along) + across * (side * lateral) + Vector3(0.0, 0.0, z)
				var aim := Vector3.ZERO
				if group.has("aim"):
					var a: Array = group["aim"]
					aim = (toward * float(a[0]) + across * (side * float(a[1])) + Vector3(0.0, 0.0, float(a[2]))).normalized()
				var shadowed := false
				if int(group.get("shadow", 0)) > i and budget > _shadowed:
					shadowed = true
					_shadowed += 1
				_add_fixture(group, site, aim, shadowed, settings)
	deck_luminance_cd = DECK_ALBEDO / PI * deck_illuminance_model_lux(Vector3(0.0, 0.0, float(geometry["mount_height_m"])))


## Fixture model: lux on the horizontal mount deck centre (mount_height_m) from every lamp that reaches it.
func deck_illuminance_model_lux(deck_site: Vector3) -> float:
	var lux := 0.0
	for fixture: Dictionary in _fixtures:
		if fixture["kind"] == "obstruction":
			continue
		var to_deck: Vector3 = deck_site - fixture["site"]
		var distance := to_deck.length()
		var direction := to_deck / distance
		var cos_incidence := -direction.z
		if fixture["kind"] == "flood":
			var half: float = fixture["half_angle_rad"]
			if (fixture["aim"] as Vector3).dot(direction) <= cos(half):
				continue
			lux += illuminance_lux(float(fixture["lumens"]) / (TAU * (1.0 - cos(half))), distance, cos_incidence)
		else:
			lux += illuminance_lux(float(fixture["cd"]), distance, cos_incidence)
	return lux


func night_factor_now() -> float:
	return _night_factor


func shadowed_count() -> int:
	return _shadowed


func fixtures() -> Array[Dictionary]:
	return _fixtures


func _add_fixture(group: Dictionary, site: Vector3, aim: Vector3, shadowed: bool, settings: Dictionary) -> void:
	var kind := str(group["kind"])
	var fixture := {"kind": kind, "group": str(group["name"]), "site": site, "aim": aim, "cd": 0.0, "tint": Vector3.ONE,
		"lumens": float(group.get("lumens", 0.0)), "half_angle_rad": deg_to_rad(float(group.get("half_angle_deg", 0.0)))}
	var lens_m: float = float(group.get("lens_m", 0.3))
	var light: Light3D = null
	if kind == "obstruction":
		fixture["cd"] = float(settings["red_cd"])
		fixture["tint"] = RED
		var omni := OmniLight3D.new()
		omni.omni_attenuation = 2.0
		omni.omni_range = float(group["range_m"])
		light = omni
	else:
		var lumens: float = float(group["lumens"])
		var tint := cct_color(float(group["cct_k"]))
		fixture["tint"] = Vector3(tint.r, tint.g, tint.b)
		if kind == "flood":
			var half := deg_to_rad(float(group["half_angle_deg"]))
			fixture["cd"] = candela_cone(lumens, half)
			var spot := SpotLight3D.new()
			spot.spot_angle = rad_to_deg(half)
			spot.spot_attenuation = 2.0  # inverse square: the physics of a point source
			spot.spot_range = float(group["range_m"])
			spot.spot_angle_attenuation = ANGLE_FALLOFF
			spot.light_specular = 0.5
			light = spot
		else:
			fixture["cd"] = candela_omni(lumens)
			var omni := OmniLight3D.new()
			omni.omni_attenuation = 2.0
			omni.omni_range = float(group["range_m"])
			light = omni
	var tint_v: Vector3 = fixture["tint"]
	light.light_color = Color(tint_v.x, tint_v.y, tint_v.z).linear_to_srgb()
	light.shadow_enabled = shadowed
	light.light_energy = 0.0
	light.visible = false
	light.distance_fade_enabled = shadowed
	light.distance_fade_shadow = float(config["lights"]["shadow_range_m"])
	light.distance_fade_begin = 1.0e6
	light.distance_fade_length = 1.0
	light.name = "%s_%d" % [fixture["group"], _fixtures.size()]
	light.position = Frames.to_render(site)
	if kind == "flood":
		var dir := Frames.to_render(aim)
		light.basis = Basis.looking_at(dir, Vector3.UP if absf(dir.y) < 0.99 else Vector3.RIGHT)
	add_child(light)
	lights.append(light)
	fixture["light"] = light
	# The lens: the glowing disc, a dark housing behind it.
	var lens := MeshInstance3D.new()
	lens.name = "lens_" + light.name
	lens.mesh = _lens_mesh
	var material := ShaderMaterial.new()
	material.shader = LENS_SHADER
	material.set_shader_parameter("lens_m", lens_m)
	material.set_shader_parameter("min_angle_rad", float(settings["point_angle_rad"]))
	material.set_shader_parameter("tint", fixture["tint"])
	material.set_shader_parameter("intensity_cd", 0.0)
	lens.material_override = material
	lens.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	lens.extra_cull_margin = 16384.0  # a billboard grows with distance: never culled by its small mesh bounds
	lens.position = Frames.to_render(site)
	add_child(lens)
	lenses.append(lens)
	fixture["lens"] = lens
	if kind != "obstruction":
		var housing := MeshInstance3D.new()
		var box := BoxMesh.new()
		box.size = Vector3(lens_m * HOUSING_OVERSIZE, lens_m * HOUSING_OVERSIZE, lens_m * HOUSING_DEPTH_FACTOR)
		housing.mesh = box
		housing.material_override = _housing_material
		housing.name = "housing_" + light.name
		var forward := Frames.to_render(aim) if aim != Vector3.ZERO else Vector3(0.0, 0.0, 1.0)
		housing.position = light.position - forward * (0.5 * lens_m * HOUSING_DEPTH_FACTOR + 0.02)
		housing.basis = Basis.looking_at(forward, Vector3.UP if absf(forward.y) < 0.99 else Vector3.RIGHT)
		add_child(housing)
	_fixtures.append(fixture)


# --- run time ------------------------------------------------------------------------------------------------------------

## Sun's elevation at the site (rad), the game's exposure (cd/m^2 -> scene), the clock (TDB s). Lamps burn when night_factor
## > 0, the obstruction lenses flash always (white by day).
func update_state(sun_elevation_rad: float, scene_per_cd_m2: float, time_s: float) -> void:
	if config.is_empty():
		return
	var settings: Dictionary = config["lights"]
	var factor := night_factor(sun_elevation_rad, float(settings["dusk_deg"]), float(settings["fade_deg"]))
	var flash := flash_is_on(time_s, float(settings["flash_per_min"]), float(settings["flash_on_s"]))
	var exposure_changed := absf(scene_per_cd_m2 - _last_scene_per_cd) > MIN_ENERGY_CHANGE * maxf(_last_scene_per_cd, 1.0e-12)
	if factor == _night_factor and flash == _flash_on and not exposure_changed:
		return
	_night_factor = factor
	_flash_on = flash
	_last_scene_per_cd = scene_per_cd_m2
	for fixture: Dictionary in _fixtures:
		var light: Light3D = fixture["light"]
		var lens: MeshInstance3D = fixture["lens"]
		var material := lens.material_override as ShaderMaterial
		var cd: float = fixture["cd"]
		var lens_cd := 0.0
		if fixture["kind"] == "obstruction":
			var day := factor == 0.0
			lens_cd = float(settings["white_day_cd"]) if day else cd * factor
			material.set_shader_parameter("tint", Vector3.ONE if day else fixture["tint"])
			var burning := flash and factor > 0.0  # the red lamp's glow on the steel: night only
			light.visible = burning
			light.light_energy = energy_of(cd, scene_per_cd_m2) * factor if burning else 0.0
			lens_cd = lens_cd if flash else 0.0
		else:
			lens_cd = cd * factor
			light.visible = factor > 0.0
			light.light_energy = energy_of(cd, scene_per_cd_m2) * factor
		material.set_shader_parameter("intensity_cd", lens_cd)
		lens.visible = lens_cd > 0.0


## Lamps burning now (visible lights excluding the obstruction ones).
func burning_count() -> int:
	var n := 0
	for fixture: Dictionary in _fixtures:
		if fixture["kind"] != "obstruction" and (fixture["light"] as Light3D).visible:
			n += 1
	return n
