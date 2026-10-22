class_name TimelineWindow
extends RefCounted
## Time window of the mission-timeline arc and its label thinning. Pure functions, no nodes.

const MIN_SPAN_S := 60.0
const EASE_TIME_S := 2.0
const MIN_LABEL_PX := 70.0
## Margin on both ends, as a fraction of the span. [E]
const MARGIN_FRACTION := 0.1
## Number of upcoming events the window reaches.
const FUTURE_EVENTS := 3
## A further event is left out when it lies more than this many times the span so far beyond the previous one
## (a far-off event, e.g. days away, would squash the near ones to a point). [E]
const MAX_GAP_RATIO := 20.0

## Label priority, high to low; kinds not listed rank last.
const PRIORITY: Array[String] = [
	"liftoff", "meco", "seco", "stage_separation", "docking", "undocking",
	"caught", "landed", "splashdown", "impact",
	"max_q", "entry_interface", "boostback", "landing_burn", "parachutes", "crew_ejection",
	"launch", "ignition", "cutoff", "engine_start",
]
## Equal rank for the landing outcomes.
const _EQUAL_RANK := {"caught": "caught", "landed": "caught", "splashdown": "caught", "impact": "caught"}


## (start, end) of the window around `now_tdb`: last past event .. third next event, with margins.
static func target_span(events_tdb: PackedFloat64Array, now_tdb: float) -> Vector2:
	var past := -INF
	var future: Array[float] = []
	for t in events_tdb:
		if t <= now_tdb:
			past = maxf(past, t)
		else:
			future.append(t)
	future.sort()
	var has_past := is_finite(past)
	var lo := past if has_past else now_tdb
	var hi := now_tdb
	var reached := 0
	for t in future:
		if reached >= FUTURE_EVENTS:
			break
		if t - hi > MAX_GAP_RATIO * maxf(hi - lo, MIN_SPAN_S):
			break
		hi = t
		reached += 1
	var span := maxf(hi - lo, MIN_SPAN_S)
	var start := lo - MARGIN_FRACTION * span
	if reached == 0:
		return Vector2(start, now_tdb + maxf(MARGIN_FRACTION * span, MIN_SPAN_S))
	return Vector2(start, hi + MARGIN_FRACTION * span)


## Exponential approach of the displayed window to the target (frame-rate independent).
static func ease(current: Vector2, target: Vector2, delta_real_s: float) -> Vector2:
	return current.lerp(target, 1.0 - exp(-maxf(delta_real_s, 0.0) / EASE_TIME_S))


## Events (dictionaries with "kind" and "tdb_s") inside `window`, thinned so that no two labels are closer than
## MIN_LABEL_PX on an arc of `arc_px`. The higher-priority kind stays; equal priority keeps the earlier event.
static func visible_labels(events: Array, window: Vector2, arc_px: float) -> Array:
	var span := window.y - window.x
	if span <= 0.0:
		return []
	var inside: Array = []
	for e in events:
		var t := float(e["tdb_s"])
		if t >= window.x and t <= window.y:
			inside.append(e)
	inside.sort_custom(func(a, b): return float(a["tdb_s"]) < float(b["tdb_s"]))
	var order := range(inside.size())
	order.sort_custom(func(a, b):
		var ra := _rank(inside[a]["kind"])
		var rb := _rank(inside[b]["kind"])
		return ra < rb or (ra == rb and a < b))
	var kept: Array[int] = []
	for i in order:
		var px := (float(inside[i]["tdb_s"]) - window.x) / span * arc_px
		var free := true
		for k in kept:
			if absf((float(inside[k]["tdb_s"]) - window.x) / span * arc_px - px) < MIN_LABEL_PX:
				free = false
				break
		if free:
			kept.append(i)
	kept.sort()
	var result: Array = []
	for i in kept:
		result.append(inside[i])
	return result


static func _rank(kind: String) -> int:
	var idx := PRIORITY.find(_EQUAL_RANK.get(kind, kind))
	return PRIORITY.size() if idx < 0 else idx
