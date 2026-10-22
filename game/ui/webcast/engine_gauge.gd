class_name EngineGauge
extends Control
## Engine layout of the lowest stage that has engines on its cross-section (body y to the right, body z down), each
## engine lit while it fires (vacuum engines drawn larger, as their nozzles are); around it two arcs with the stage's
## remaining main-tank propellant: LOX on the left, CH4 on the right, both filling from the bottom up. Drawn around
## the control's position.

const STAGE_RADIUS_PX := 50.0   ## stage cross-section outline
const TANK_RADIUS_PX := 64.0    ## propellant arcs
## Each arc spans this angle from the bottom gap up to the top gap.
const TANK_SPAN_RAD := 0.75 * PI
const TANK_GAP_RAD := 0.125 * PI  ## half the gap at the bottom (labels) and at the top
## [E] Engine dot radius as a fraction of the engine spacing (half the spacing would make them touch), limits (px).
const ENGINE_DOT_FRACTION := 0.42
const ENGINE_DOT_MIN_PX := 2.5
const ENGINE_DOT_MAX_PX := 12.0
## [E] Vacuum engines' dot size relative to sea-level engines (larger nozzle exit).
const VACUUM_DOT_SCALE := 1.4
## [E] Engine positions are drawn on this fraction of the outline radius: the outer ring of a booster sits close to
## the hull, its dots still need room inside the outline.
const ENGINE_AREA_FRACTION := 0.82
const ENGINE_RIM_PX := 3.0  ## gap between the outer engines and the stage outline
const LABEL_SIZE := 13

var dots: Array = []  ## from layout(): {"position": Vector2 (stage radius = 1), "firing": bool, "vacuum": bool}
var oxidizer_fraction := -1.0  ## < 0: no oxidizer tank, arc hidden
var fuel_fraction := -1.0


func _init() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE


func show_stage(engines: Array, stages: Array) -> void:
	var stage := lowest_engine_stage(engines)
	dots = layout(engines, stage, _stage_radius_m(stages, stage, engines))
	oxidizer_fraction = -1.0
	fuel_fraction = -1.0
	if stage >= 0 and stage < stages.size():
		var s: Dictionary = stages[stage]
		oxidizer_fraction = _fraction(s, "oxidizer_kg", "oxidizer_capacity_kg")
		fuel_fraction = _fraction(s, "fuel_kg", "fuel_capacity_kg")
	queue_redraw()


## Index of the lowest stage with engines (stage 0 is the bottom of the stack), -1 without engines.
static func lowest_engine_stage(engines: Array) -> int:
	var lowest := -1
	for e: Dictionary in engines:
		var s := int(e["stage"])
		if lowest < 0 or s < lowest:
			lowest = s
	return lowest


## The stage's engines on its cross-section: body (y, z) / radius, firing and vacuum flags.
static func layout(engines: Array, stage: int, radius_m: float) -> Array:
	var result: Array = []
	for e: Dictionary in engines:
		if int(e["stage"]) != stage:
			continue
		var p: Vector3 = e["position_m"]
		result.append({"position": Vector2(p.y, p.z) / maxf(radius_m, 1e-6), "firing": bool(e["firing"]),
				"vacuum": bool(e.get("vacuum", false))})
	return result


func _draw() -> void:
	var c := Vector2.ZERO
	WebcastStyle.draw_ring(self, c, STAGE_RADIUS_PX, WebcastStyle.DIM)
	var dot_px := _dot_radius_px()
	for dot: Dictionary in dots:
		var p: Vector2 = dot["position"] * ENGINE_AREA_FRACTION * STAGE_RADIUS_PX
		var r := dot_px * _size_of(dot)
		if dot["firing"]:
			WebcastStyle.draw_dot(self, p, r)
		else:
			WebcastStyle.draw_arc_stroke(self, p, r, 0.0, TAU, WebcastStyle.DIM, 1.5)
	var font := WebcastStyle.label_font()
	var bottom_left := 0.5 * PI + TANK_GAP_RAD   # screen angles, clockwise from +x
	var bottom_right := 0.5 * PI - TANK_GAP_RAD
	if oxidizer_fraction >= 0.0:
		WebcastStyle.draw_arc_stroke(self, c, TANK_RADIUS_PX, bottom_left, bottom_left + TANK_SPAN_RAD, WebcastStyle.DIM)
		WebcastStyle.draw_arc_stroke(self, c, TANK_RADIUS_PX, bottom_left, bottom_left + oxidizer_fraction * TANK_SPAN_RAD)
		WebcastStyle.draw_text(self, Vector2(-14.0, TANK_RADIUS_PX + 8.0), "LOX", font, LABEL_SIZE, WebcastStyle.WHITE,
				WebcastStyle.Align.RIGHT)
	if fuel_fraction >= 0.0:
		WebcastStyle.draw_arc_stroke(self, c, TANK_RADIUS_PX, bottom_right, bottom_right - TANK_SPAN_RAD, WebcastStyle.DIM)
		WebcastStyle.draw_arc_stroke(self, c, TANK_RADIUS_PX, bottom_right, bottom_right - fuel_fraction * TANK_SPAN_RAD)
		WebcastStyle.draw_text(self, Vector2(14.0, TANK_RADIUS_PX + 8.0), "CH4", font, LABEL_SIZE, WebcastStyle.WHITE,
				WebcastStyle.Align.LEFT)


## Dot radius (px, sea-level size) from the engine spacing, so that a 33-engine booster and a 6-engine ship both read
## clearly: neighbours keep a gap, the outer engines stay inside the stage outline.
func _dot_radius_px() -> float:
	var radius := ENGINE_DOT_MAX_PX
	for i in dots.size():
		var a: Vector2 = dots[i]["position"] * ENGINE_AREA_FRACTION * STAGE_RADIUS_PX
		radius = minf(radius, (STAGE_RADIUS_PX - ENGINE_RIM_PX - a.length()) / _size_of(dots[i]))
		for j in range(i + 1, dots.size()):
			var b: Vector2 = dots[j]["position"] * ENGINE_AREA_FRACTION * STAGE_RADIUS_PX
			radius = minf(radius, 2.0 * ENGINE_DOT_FRACTION * a.distance_to(b) / (_size_of(dots[i]) + _size_of(dots[j])))
	return maxf(radius, ENGINE_DOT_MIN_PX)


static func _size_of(dot: Dictionary) -> float:
	return VACUUM_DOT_SCALE if dot["vacuum"] else 1.0


## Stage radius from its diameter (engines outside it, none expected, widen it).
static func _stage_radius_m(stages: Array, stage: int, engines: Array) -> float:
	var radius := 0.0
	if stage >= 0 and stage < stages.size():
		radius = 0.5 * float(stages[stage].get("diameter_m", 0.0))
	for e: Dictionary in engines:
		if int(e["stage"]) == stage:
			var p: Vector3 = e["position_m"]
			radius = maxf(radius, Vector2(p.y, p.z).length())
	return maxf(radius, 1.0)


static func _fraction(stage: Dictionary, amount_key: String, capacity_key: String) -> float:
	var capacity := float(stage.get(capacity_key, 0.0))
	if capacity <= 0.0:
		return -1.0
	return clampf(float(stage.get(amount_key, 0.0)) / capacity, 0.0, 1.0)
