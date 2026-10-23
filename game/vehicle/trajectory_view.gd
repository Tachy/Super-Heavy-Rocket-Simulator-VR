class_name TrajectoryView
extends MeshInstance3D
## Planned trajectory (target orbit of the guidance, green). GCRF samples are refreshed periodically and placed relative
## to the camera every frame with the same scaled-space mapping as the bodies. The coasting prediction of the current
## state (orange) is not drawn (user 2026-10-23): re-predicted every refresh under thrust, it jumped constantly.
## Drawn as a ribbon of constant pixel width (res://shaders/orbit_ribbon.gdshader): depth-tested, cut behind the Earth's
## limb and faded where the line of sight grazes the atmosphere (user 2026-10-23: the 1-pixel line looked stepped and was
## drawn over the Earth and the vehicles).

const REFRESH_S := 0.5
const SAMPLES := 720  ## 0.5 deg per segment on a circular orbit: no visible corners at the ribbon's width
## [E] fade band above the surface for lines of sight through the atmosphere: the Karman line (100 km), where the
## visible limb haze ends
const ATMOSPHERE_FADE_HEIGHT_M := 100000.0
const SHADER := preload("res://shaders/orbit_ribbon.gdshader")

var _planned := PackedVector3Array()
var _age := INF
var _mesh := ArrayMesh.new()
var _material := ShaderMaterial.new()


func _ready() -> void:
	mesh = _mesh
	cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	_material.shader = SHADER
	_material.set_shader_parameter("earth_radius_m", World.EARTH_MEAN_RADIUS_M)
	_material.set_shader_parameter("atmosphere_height_m", ATMOSPHERE_FADE_HEIGHT_M)


func update_view(sim: FlightSim, camera_gcrf_m: Vector3, delta: float) -> void:
	_mesh.clear_surfaces()
	if not sim.is_active():
		return
	_age += delta
	if _age >= REFRESH_S:
		_age = 0.0
		_planned = sim.target_orbit_points(SAMPLES)
	_material.set_shader_parameter("earth_center_world", Frames.to_render(-camera_gcrf_m))  # GCRF is Earth-centred
	RibbonMesh.add(_mesh, _planned, camera_gcrf_m, true, _material)
