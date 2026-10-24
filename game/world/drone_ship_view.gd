class_name DroneShipView
extends Node3D
## The drone ship of a landing zone with a deck (FlightSim.landing_zone has_deck), built procedurally: a hull box
## (hull_length_m x hull_beam_m waterplane, from freeboard + draft below the deck up to the deck slab), the deck slab
## (deck_length_m x deck_width_m incl. the wings) whose top is the physics' deck plane, and low walls along the long
## sides; matte grey, no markings.
## Child of the Earth body, placed at the landing zone origin (the deck top at rest) in Earth-fixed coordinates like
## TowerView; local coordinates are the site frame (east, north, up) mapped to render axes (Frames.to_render). The ship
## is posed by the snapshot's deck frame (deck_bow / deck_port / deck_normal, heave at the deck centre): the same
## contact::deck_frame the leg contact uses, so the drawn deck is the physics deck. The pose is the snapshot's, drawn at
## the snapshot instant like the vehicles (World.update_world).

## [E] Thickness of the deck slab (m): the hull box ends below it, so the faces never overlap.
const DECK_THICKNESS_M := 1.0
## [E] Height and thickness of the walls along the long sides (m) (drawn only: not part of the physics deck).
const WALL_HEIGHT_M := 2.0
const WALL_THICKNESS_M := 0.5

var _key := []  # the zone this ship was built for: origin, heading, deck size
var _deck := Node3D.new()  # the moving part: deck frame in the site frame
var _hull_material := StandardMaterial3D.new()
var _deck_material := StandardMaterial3D.new()


## True when `zone` (FlightSim.landing_zone) is the drone ship this view was built for.
func matches(zone: Dictionary) -> bool:
	return _key == _zone_key(zone)


func build(zone: Dictionary) -> void:
	_key = _zone_key(zone)
	_hull_material.albedo_color = Color(0.30, 0.31, 0.32)
	_hull_material.roughness = 0.9
	_deck_material.albedo_color = Color(0.42, 0.42, 0.41)
	_deck_material.roughness = 0.95
	position = Frames.to_render(zone["origin_itrf_m"])
	basis = Frames.basis_to_render(Basis(zone["east"], zone["north"], zone["up"]))
	add_child(_deck)
	# deck coordinates: x bow, y port, z deck normal; origin the deck centre on the deck top
	var length: float = zone["deck_length_m"]
	var width: float = zone["deck_width_m"]
	var hull_height: float = float(zone["freeboard_m"]) + float(zone["hull_draft_m"]) - DECK_THICKNESS_M
	_deck.add_child(_box(Vector3(zone["hull_length_m"], zone["hull_beam_m"], hull_height),
			Vector3(0.0, 0.0, -DECK_THICKNESS_M - 0.5 * hull_height), _hull_material))
	_deck.add_child(_box(Vector3(length, width, DECK_THICKNESS_M), Vector3(0.0, 0.0, -0.5 * DECK_THICKNESS_M), _deck_material))
	for side in [1.0, -1.0]:
		_deck.add_child(_box(Vector3(length, WALL_THICKNESS_M, WALL_HEIGHT_M),
				Vector3(0.0, side * (0.5 * width - 0.5 * WALL_THICKNESS_M), 0.5 * WALL_HEIGHT_M), _hull_material))
	update_pose(zone)


## zone: FlightSim.landing_zone of this ship; poses the deck (heave along up, tilted axes).
func update_pose(zone: Dictionary) -> void:
	var bow: Vector3 = zone.get("deck_bow", Vector3.ZERO)
	if bow == Vector3.ZERO:
		return
	_deck.basis = Frames.basis_to_render(Basis(bow, zone["deck_port"], zone["deck_normal"]))
	_deck.position = Frames.to_render(Vector3(0.0, 0.0, zone.get("deck_heave_m", 0.0)))


## The posed deck's global transform (deck axes in render axes, origin the deck centre on its top): render_check.
func deck_transform() -> Transform3D:
	return _deck.global_transform


static func _zone_key(zone: Dictionary) -> Array:
	return [zone.get("origin_itrf_m"), zone.get("heading_deg"), zone.get("deck_length_m"), zone.get("deck_width_m"),
			zone.get("hull_length_m"), zone.get("hull_beam_m"), zone.get("hull_draft_m"), zone.get("freeboard_m")]


## Box with `size_local` along the deck axes (x bow, y port, z normal), centred at `center_deck`.
func _box(size_local: Vector3, center_deck: Vector3, material: Material) -> MeshInstance3D:
	var mesh := BoxMesh.new()
	mesh.size = Vector3(size_local.y, size_local.z, size_local.x)  # local (a, b, c) -> render (b, c, a)
	var node := MeshInstance3D.new()
	node.mesh = mesh
	node.material_override = material
	node.position = Frames.to_render(center_deck)
	return node
