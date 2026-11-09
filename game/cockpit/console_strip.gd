class_name ConsoleStrip
extends RefCounted
## The console as a flat 2D strip (docs/subsystems/cockpit.md, "Console pointer"): the three coplanar screens and the key rows under
## them seen straight on. Strip coordinates (u, v) in metres from the centre screen's centre: u along the crew's right, v along the
## console's up. Every touchable surface (screen or key) is a rectangle of the strip with its own 3D face (centre, normal, size), so a
## strip point maps to a 3D point ON a surface: the gaps between screens and the module faces between keys are not surfaces, a point
## there clamps to the nearest surface rectangle. Cabin frame (capsule stage body frame), like CockpitConfig.

const SCREEN_IDS: Array[String] = ["left", "centre", "right"]

var origin := Vector3.ZERO  ## centre screen centre
var normal := Vector3.ZERO  ## toward the crew
var up := Vector3.ZERO
var right := Vector3.ZERO  ## toward the crew's right
var surfaces: Array[Dictionary] = []  ## {id, kind "screen"|"key", rect Rect2 (strip), centre Vector3, size Vector2, button_id, panel}
var bounds := Rect2()  ## the hull of all surfaces


static func from_config(config: CockpitConfig) -> ConsoleStrip:
	var strip := ConsoleStrip.new()
	var centre: Dictionary = config.screens.get("centre", {})
	if centre.is_empty():
		return strip
	strip.origin = centre["centre"]
	strip.normal = (centre["normal"] as Vector3).normalized()
	strip.up = (centre["up"] as Vector3).normalized()
	strip.right = (-strip.normal).cross(strip.up).normalized()
	for id in SCREEN_IDS:
		var screen: Dictionary = config.screens.get(id, {})
		if not screen.is_empty():
			strip._add("screen", id, screen["centre"], screen["size"], "", id)
	for button: Dictionary in config.buttons:
		# The screens are coplanar (one frame for the whole console): the key face is the button's centre, its rectangle the size_m.
		var k_id := String(button.get("id", ""))
		strip._add("key", "button:" + k_id, button["centre"], button["size"], k_id, String(button.get("panel", "")))
	return strip


func _add(kind: String, id: String, centre: Vector3, size: Vector2, button_id: String, panel: String) -> void:
	var p := to_strip(centre)
	var rect := Rect2(p - size * 0.5, size)
	surfaces.append({"id": id, "kind": kind, "rect": rect, "centre": centre, "size": size, "button_id": button_id, "panel": panel})
	bounds = rect if surfaces.size() == 1 else bounds.merge(rect)


## The strip coordinates of a cabin point (its projection on the console plane).
func to_strip(point_m: Vector3) -> Vector2:
	var d := point_m - origin
	return Vector2(d.dot(right), d.dot(up))


## The nearest point of the surfaces' union to `p` (p itself when it lies on a surface; strip edges, screen gaps and module faces
## clamp).
func clamp(p: Vector2) -> Vector2:
	return _nearest(p)["point"]


## The surface under `p` after clamping: {surface_id, button_id, point_m (3D, on the surface's face), normal, uv (0..1, (0, 0) top
## left as the screen meshes), strip (the clamped point), distance_m (how far p was from the surface)}.
func to_surface(p: Vector2) -> Dictionary:
	if surfaces.is_empty():
		return {}
	var hit := _nearest(p)
	var s: Dictionary = surfaces[hit["index"]]
	var q: Vector2 = hit["point"]
	var rect: Rect2 = s["rect"]
	var centre_strip := rect.position + rect.size * 0.5
	var size: Vector2 = s["size"]
	var point: Vector3 = (s["centre"] as Vector3) + right * (q.x - centre_strip.x) + up * (q.y - centre_strip.y)
	return {"surface_id": s["id"], "button_id": s["button_id"], "point_m": point, "normal": normal,
			"uv": Vector2((q.x - centre_strip.x) / size.x + 0.5, 0.5 - (q.y - centre_strip.y) / size.y),
			"strip": q, "distance_m": hit["distance"]}


## The strip point a ray (cabin frame) hits on the console plane; false when the ray does not meet it in front of the origin.
func pick(ray_origin: Vector3, ray_dir: Vector3) -> Dictionary:
	var denom := ray_dir.dot(normal)
	if absf(denom) < 1e-6:
		return {"hit": false}
	var t := (origin - ray_origin).dot(normal) / denom
	if t <= 0.0:
		return {"hit": false}
	return {"hit": true, "strip": to_strip(ray_origin + ray_dir * t)}


## The surface whose centre is `surface_id` ("left", "button:cam1"); {} when unknown.
func surface(surface_id: String) -> Dictionary:
	for s in surfaces:
		if s["id"] == surface_id:
			return s
	return {}


func _nearest(p: Vector2) -> Dictionary:
	var best := -1
	var best_d := INF
	var best_point := p
	for i in surfaces.size():
		var rect: Rect2 = surfaces[i]["rect"]
		var q := Vector2(clampf(p.x, rect.position.x, rect.end.x), clampf(p.y, rect.position.y, rect.end.y))
		var d := p.distance_to(q)
		# A key stands in front of nothing but the shelf; between equal distances a key wins over a screen (keys are the smaller targets).
		if d < best_d - 1e-9 or (absf(d - best_d) <= 1e-9 and surfaces[i]["kind"] == "key" and surfaces[best]["kind"] == "screen"):
			best = i
			best_d = d
			best_point = q
	return {"index": best, "point": best_point, "distance": best_d}
