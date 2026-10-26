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
## [E] The ribbon is rebuilt (720 segments in GDScript: ~5 ms) only with new points or after the camera moved this far
## from the build's anchor; in between the node is moved with the camera. Only the scaled-space pull-in of points
## beyond CelestialBody.SCALED_SPACE_DISTANCE_M then aims from the anchor: <= 1 km / 1.5e7 m = 0.004 deg, ~0.15 px
## in the headset (the fragment shader's limb test works in view space and is exact).
const REBUILD_DISTANCE_M := 1000.0
const SHADER := preload("res://shaders/orbit_ribbon.gdshader")

var _planned := PackedVector3Array()
var _age := INF
var _anchor_gcrf_m := Vector3.INF  ## camera of the last build
var _mesh := ArrayMesh.new()
var _material := ShaderMaterial.new()


func _ready() -> void:
	mesh = _mesh
	cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	_material.shader = SHADER
	_material.set_shader_parameter("earth_radius_m", World.EARTH_MEAN_RADIUS_M)
	_material.set_shader_parameter("atmosphere_height_m", ATMOSPHERE_FADE_HEIGHT_M)


func update_view(sim: FlightSim, camera_gcrf_m: Vector3, delta: float) -> void:
	if not sim.is_active():
		_mesh.clear_surfaces()
		_planned = PackedVector3Array()
		_anchor_gcrf_m = Vector3.INF
		return
	_age += delta
	var refreshed := false
	if _age >= REFRESH_S:
		_age = 0.0
		var points := sim.target_orbit_points(SAMPLES)
		refreshed = points != _planned  # a fixed target orbit gives the same points: no rebuild
		_planned = points
	_material.set_shader_parameter("earth_center_world", Frames.to_render(-camera_gcrf_m))  # GCRF is Earth-centred
	place(_planned, refreshed, camera_gcrf_m)


## Builds the ribbon at the camera when the points are new or the camera left REBUILD_DISTANCE_M around the last
## build, else moves the built ribbon with the camera.
func place(points_gcrf: PackedVector3Array, points_changed: bool, camera_gcrf_m: Vector3) -> void:
	if points_changed or camera_gcrf_m.distance_to(_anchor_gcrf_m) > REBUILD_DISTANCE_M:
		_mesh.clear_surfaces()
		RibbonMesh.add(_mesh, points_gcrf, camera_gcrf_m, true, _material)
		_anchor_gcrf_m = camera_gcrf_m
		position = Vector3.ZERO
	else:
		position = Frames.to_render(_anchor_gcrf_m - camera_gcrf_m)
