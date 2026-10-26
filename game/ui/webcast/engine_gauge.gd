class_name EngineGauge
extends Control
## Engine layout of the lowest stage that has main engines, on its cross-section (body y to the right, body z down),
## the same size as the round gauges (user 2026-10-22); each engine lit while it fires, vacuum engines drawn larger
## (as their nozzles are). Drawn around the control's position. Auxiliary engines (launch escape, landing thrusters)
## are filtered out before (GaugeRules.main_engines).

const STAGE_RADIUS_PX := RoundGauge.RADIUS_PX  ## stage cross-section outline
## [E] Engine dot radius as a fraction of the engine spacing (half the spacing would make them touch), limits (px).
const ENGINE_DOT_FRACTION := 0.42
const ENGINE_DOT_MIN_PX := 2.0
const ENGINE_DOT_MAX_PX := 9.0
## [E] Vacuum engines' dot size relative to sea-level engines (larger nozzle exit).
const VACUUM_DOT_SCALE := 1.4
## [E] Engine positions are drawn on this fraction of the outline radius: the outer ring of a booster sits close to
## the hull, its dots still need room inside the outline.
const ENGINE_AREA_FRACTION := 0.82
const ENGINE_RIM_PX := 2.0  ## gap between the outer engines and the stage outline
const UNLIT_STROKE_PX := 1.5
const LABEL := "Triebwerke"

var dots: Array = []  ## from layout(): {"position": Vector2 (stage radius = 1), "firing": bool, "vacuum": bool}


func _init() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE


func show_stage(engines: Array, stages: Array) -> void:
	var stage := lowest_engine_stage(engines)
	dots = layout(engines, stage, _stage_radius_m(stages, stage, engines))
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
	FrameProfiler.begin(&"ui_webcast_draw")
	_draw_unprofiled()
	FrameProfiler.end(&"ui_webcast_draw")


func _draw_unprofiled() -> void:
	WebcastStyle.draw_ring(self, Vector2.ZERO, STAGE_RADIUS_PX, WebcastStyle.DIM)
	var dot_px := _dot_radius_px()
	for dot: Dictionary in dots:
		var p: Vector2 = dot["position"] * ENGINE_AREA_FRACTION * STAGE_RADIUS_PX
		var r := dot_px * _size_of(dot)
		if dot["firing"]:
			WebcastStyle.draw_dot(self, p, r)
		else:
			WebcastStyle.draw_arc_stroke(self, p, r, 0.0, TAU, WebcastStyle.DIM, UNLIT_STROKE_PX)
	WebcastStyle.draw_text(self, Vector2(0.0, RoundGauge.RADIUS_PX + RoundGauge.LABEL_GAP_PX), LABEL.to_upper(),
			WebcastStyle.label_font(), RoundGauge.LABEL_SIZE)


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
