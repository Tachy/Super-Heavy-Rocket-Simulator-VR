class_name World
extends Node3D
## Earth-Moon-Sun world at the true (TDB) instant of the simulation clock.
##
## Every frame: query the float64 core (ephemeris, orientations), compute the camera's true GCRF
## position, then place everything relative to it (floating origin, scaled space for far bodies).

const EARTH_SEMI_MAJOR_AXIS_M := 6378137.0
const EARTH_FLATTENING := 1.0 / 298.257223563
const EARTH_MEAN_RADIUS_M := 6371000.0
const MOON_RADIUS_M := 1737400.0
const SUN_RADIUS_M := 695700000.0
const GROUND_EYE_HEIGHT_M := 2.0
## The solar disk is drawn behind all scaled-space bodies (1.5e7 m) but in front of the stars (2.5e7 m).
const SUN_RENDER_DISTANCE_M := 2.0e7

## Focus targets selectable in the UI.
const FOCUS_EARTH := "earth"
const FOCUS_MOON := "moon"
const FOCUS_EARTH_MOON := "earth_moon"

var core := StarsimCore.new()
var clock := SimClock.new()
var camera := OrbitCamera.new()
var earth := CelestialBody.new()
var moon := CelestialBody.new()
var sun_disk := MeshInstance3D.new()
var sun_light := DirectionalLight3D.new()
var stars := StarField.new()
var moon_orbit := OrbitLine.new()
var earth_label := Label3D.new()
var moon_label := Label3D.new()
var sites: Array = []  ## launch sites from config/launch_sites.toml
var focus := FOCUS_EARTH
var ground_site: Dictionary = {}

## Per-frame state, readable by UI and tests.
var moon_gcrf_m := Vector3.ZERO
var sun_apparent_from_earth_m := Vector3.ZERO
var moon_apparent_from_earth_m := Vector3.ZERO

var _earth_material := ShaderMaterial.new()
var _moon_material := ShaderMaterial.new()


func _ready() -> void:
	var data_dir := Paths.data_dir()
	if not core.open(data_dir.path_join("spice")):
		push_error("World: SPICE kernels missing - run tools/data_pipeline/fetch.py")
		return
	sites = TomlLite.load_file(Paths.config_dir().path_join("launch_sites.toml")).get("site", [])

	add_child(clock)
	add_child(camera)
	_setup_environment()
	_setup_bodies(data_dir)
	stars.load_catalogue(data_dir.path_join("stars/stars.csv"))
	add_child(stars)
	add_child(moon_orbit)
	moon_orbit.visible = false
	_setup_label(earth_label, "Erde")
	_setup_label(moon_label, "Mond")
	clock.set_time(core.utc_to_tdb_s(Time.get_datetime_string_from_system(true)))


func _process(_delta: float) -> void:
	if not core.is_open():
		return
	update_world()


## Recomputes the whole scene for clock.tdb_s. Public so tests can render a fixed instant.
func update_world() -> void:
	var t := clock.tdb_s
	earth.fixed_to_gcrf = core.earth_fixed_to_gcrf(t)
	moon.fixed_to_gcrf = core.moon_fixed_to_gcrf(t)
	moon_gcrf_m = core.body_position_gcrf_m("moon", t)
	moon.center_gcrf_m = moon_gcrf_m

	_update_camera(t)
	var cam := camera.position_gcrf_m

	# Apparent directions: as seen from the Earth's centre for Earth shading, from the camera for
	# what the camera sees directly (solar disk, light direction).
	sun_apparent_from_earth_m = core.apparent_body_position_gcrf_m("sun", t, Vector3.ZERO, Vector3.ZERO)
	moon_apparent_from_earth_m = core.apparent_body_position_gcrf_m("moon", t, Vector3.ZERO, Vector3.ZERO)
	var sun_from_camera := core.apparent_body_position_gcrf_m("sun", t, cam, Vector3.ZERO)

	earth.place(cam)
	moon.place(cam)
	_earth_material.set_shader_parameter("sun_position_local", earth.to_local_frame(sun_apparent_from_earth_m))
	_earth_material.set_shader_parameter("moon_position_local", earth.to_local_frame(moon_apparent_from_earth_m))
	_moon_material.set_shader_parameter("sun_position_local", moon.to_local_frame(sun_apparent_from_earth_m - moon_gcrf_m))
	_moon_material.set_shader_parameter("earth_position_local", moon.to_local_frame(-moon_gcrf_m))
	_moon_material.set_shader_parameter("camera_position_local", moon.to_local_frame(cam - moon_gcrf_m))

	_place_sun(sun_from_camera)
	moon_orbit.update_line(core, t, cam)
	earth_label.position = earth.position
	moon_label.position = moon.position


## Ground view from an arbitrary site (dictionary like the entries of config/launch_sites.toml).
func set_ground_site(site: Dictionary) -> void:
	focus = site.get("id", "custom")
	ground_site = site
	camera.mode = OrbitCamera.Mode.GROUND
	moon_orbit.visible = false
	earth_label.visible = false
	moon_label.visible = false


func set_focus(new_focus: String) -> void:
	focus = new_focus
	ground_site = {}
	camera.mode = OrbitCamera.Mode.ORBIT
	moon_orbit.visible = focus == FOCUS_EARTH_MOON
	earth_label.visible = moon_orbit.visible
	moon_label.visible = moon_orbit.visible
	match focus:
		FOCUS_EARTH:
			camera.distance_m = 2.2e7
		FOCUS_MOON:
			camera.distance_m = 7.0e6
		FOCUS_EARTH_MOON:
			camera.distance_m = 1.1e9
			camera.pitch = -1.2
		_:
			for site in sites:
				if site["id"] == focus:
					ground_site = site
					camera.mode = OrbitCamera.Mode.GROUND


func _update_camera(t: float) -> void:
	if camera.mode == OrbitCamera.Mode.GROUND:
		var lat := deg_to_rad(ground_site["latitude_deg"])
		var lon := deg_to_rad(ground_site["longitude_deg"])
		var rotation := earth.fixed_to_gcrf
		camera.site_gcrf_m = core.geodetic_to_gcrf_m(ground_site["latitude_deg"], ground_site["longitude_deg"],
				ground_site["height_m"] + GROUND_EYE_HEIGHT_M, t)
		camera.site_up_gcrf = rotation * Vector3(cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat))
		camera.site_east_gcrf = rotation * Vector3(-sin(lon), cos(lon), 0.0)
		camera.site_north_gcrf = rotation * Vector3(-sin(lat) * cos(lon), -sin(lat) * sin(lon), cos(lat))
	else:
		camera.focus_center_gcrf_m = moon_gcrf_m if focus == FOCUS_MOON else Vector3.ZERO
	camera.update_pose()


func _place_sun(sun_from_camera_m: Vector3) -> void:
	var distance := sun_from_camera_m.length()
	var direction := Frames.to_render(sun_from_camera_m / distance)
	var render_distance := SUN_RENDER_DISTANCE_M
	sun_disk.position = direction * render_distance
	sun_disk.scale = Vector3.ONE * SUN_RADIUS_M * render_distance / distance
	sun_light.basis = Basis.looking_at(-direction, Vector3.UP if absf(direction.y) < 0.99 else Vector3.RIGHT)


func _setup_bodies(data_dir: String) -> void:
	_earth_material.shader = load("res://shaders/earth.gdshader")
	_earth_material.set_shader_parameter("day_texture", _load_texture(data_dir.path_join("textures/earth_day.jpg")))
	_earth_material.set_shader_parameter("night_texture", _load_texture(data_dir.path_join("textures/earth_night.jpg")))
	_earth_material.set_shader_parameter("moon_radius_m", MOON_RADIUS_M)
	earth.name = "Earth"
	earth.setup(EARTH_SEMI_MAJOR_AXIS_M, EARTH_FLATTENING, _earth_material)
	add_child(earth)

	_moon_material.shader = load("res://shaders/moon.gdshader")
	_moon_material.set_shader_parameter("albedo_texture", _load_texture(data_dir.path_join("textures/moon_color.png")))
	_moon_material.set_shader_parameter("earth_radius_m", EARTH_MEAN_RADIUS_M)
	moon.name = "Moon"
	moon.setup(MOON_RADIUS_M, 0.0, _moon_material)
	add_child(moon)

	var sphere := SphereMesh.new()
	sphere.radius = 1.0
	sphere.height = 2.0
	var sun_material := ShaderMaterial.new()
	sun_material.shader = load("res://shaders/sun.gdshader")
	sun_disk.mesh = sphere
	sun_disk.material_override = sun_material
	sun_disk.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	sun_disk.name = "Sun"
	add_child(sun_disk)
	sun_light.light_energy = 1.0
	add_child(sun_light)


## Screen-space label for the map view, where bodies are only a few pixels large.
func _setup_label(label: Label3D, text: String) -> void:
	label.text = text
	label.fixed_size = true
	label.pixel_size = 0.0012
	label.billboard = BaseMaterial3D.BILLBOARD_ENABLED
	label.no_depth_test = true
	label.offset = Vector2(0, 22)
	label.modulate = Color(0.75, 0.85, 1.0)
	label.visible = false
	add_child(label)


func _setup_environment() -> void:
	var environment := Environment.new()
	environment.background_mode = Environment.BG_COLOR
	environment.background_color = Color.BLACK
	environment.ambient_light_source = Environment.AMBIENT_SOURCE_DISABLED
	environment.tonemap_mode = Environment.TONE_MAPPER_AGX
	environment.glow_enabled = true
	environment.glow_hdr_threshold = 4.0
	environment.glow_intensity = 0.6
	var world_environment := WorldEnvironment.new()
	world_environment.environment = environment
	add_child(world_environment)


static func _load_texture(path: String) -> Texture2D:
	var image := Image.load_from_file(path)
	if image == null:
		push_error("World: missing texture %s - run tools/data_pipeline/prepare.py" % path)
		return null
	image.generate_mipmaps()
	return ImageTexture.create_from_image(image)
