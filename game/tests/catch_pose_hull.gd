class_name CatchPoseHull
extends RefCounted
## render_check --catch_hull=<deck station m>: a static booster hull with its fins hanging in a TowerView's closed arms, for
## comparing the closed pose with real frames (a picture, no physics: a flown catch is the mission's business).
## Fins at azimuth 0 / 180 over the arms, the fin at azimuth 90 away from the tower (launch-structures-measurements.md).

const HULL_RADIUS_M := 4.5  ## [S] 9 m booster diameter
const FIN_STATION_M := 64.0  ## config/vehicles/starship_block3.toml [stage.grid_fins] station_m
const FIN_AZIMUTHS_DEG: Array[float] = [0.0, 90.0, 180.0]


static func attach(view: TowerView, deck_station_m: float) -> void:
	var model_dir := Paths.data_dir().path_join("models/starship_block3")
	var hull: Dictionary = StructureModels._meshes_of(model_dir.path_join("booster.gltf"), "")
	var fin: Dictionary = StructureModels._meshes_of(model_dir.path_join("grid_fin.gltf"), "")
	if hull.is_empty() or fin.is_empty():
		push_error("catch_hull: no vehicle models in " + model_dir)
		return
	fin.erase("plate")  # the far stand-in
	var steel := LitAerial.material(Color(0.62, 0.63, 0.65), 0.35, 0.8)
	var dark := LitAerial.material(Color(0.04, 0.04, 0.05), 0.7, 0.2)
	var geometry: Dictionary = view._geometry
	var toward: Vector3 = geometry["tower_direction"]
	var across: Vector3 = geometry["across"]
	var up := Vector3(0.0, 0.0, 1.0)
	var mount := Node3D.new()
	mount.name = "CatchHull"
	mount.basis = Frames.basis_to_render(Basis(up, across, -toward))  # body X up the axis, azimuth 0 = +across, 90 = away
	mount.position = Frames.to_render(up * (float(geometry["hinge_plus_m"].z) - deck_station_m))
	var body := StructureModels.model_node("Hull", hull, {}, steel)
	mount.add_child(body)
	for azimuth_deg in FIN_AZIMUTHS_DEG:
		var azimuth := deg_to_rad(azimuth_deg)
		var node := Node3D.new()
		node.transform = Transform3D(Basis(Vector3(1.0, 0.0, 0.0), azimuth),
				Vector3(FIN_STATION_M, HULL_RADIUS_M * cos(azimuth), HULL_RADIUS_M * sin(azimuth)))
		for part: String in fin:
			var instance := MeshInstance3D.new()
			instance.mesh = fin[part]
			instance.material_override = dark
			node.add_child(instance)
		body.add_child(node)
	view.add_child(mount)
