class_name TimelineWindow
extends RefCounted
## Time-to-arc mapping of the mission-timeline arc and its label thinning. Pure functions, no nodes.
## User 2026-10-22: "now" is always the arc's apex, the future to the right, the past mirrored to the left; no strict
## time scale ("one only needs to know what comes next"), but spacing roughly follows the time spans: a compressive
## (logarithmic) mapping. Arc positions run from -1 (left end) over 0 (apex, now) to +1 (right end).

const MIN_SPAN_S := 60.0
const EASE_TIME_S := 2.0
const MIN_LABEL_PX := 70.0
## Outer part of the arc length over which line, dots and labels fade out at each end (smoothstep). [E] user
## 2026-10-22: the arc ends fade as in the webcasts.
const ARC_FADE_FRACTION := 0.15
## Number of upcoming events the arc reaches. No gap rule (the former MAX_GAP_RATIO): the compression fits a
## next event days away next to one seconds away.
const FUTURE_EVENTS := 3
## [E] The nearest next event sits at least this far from the apex (fraction of the half-arc) when several are
## shown; the compression grows until it does.
const NEAR_MIN_POSITION := 0.25
## Bisection steps of the compression knee (log-spaced between KNEE_MIN_S and the reach).
const KNEE_ITERATIONS := 50
const KNEE_MIN_S := 1.0e-3

## Label priority, high to low; kinds not listed rank last.
const PRIORITY: Array[String] = [
	"liftoff", "meco", "seco", "stage_separation", "docking", "undocking",
	"caught", "landed", "splashdown", "impact",
	"max_q", "entry_interface", "boostback", "landing_burn", "parachutes", "crew_ejection",
	"launch", "ignition", "cutoff", "engine_start",
]
## Equal rank for the landing outcomes.
const _EQUAL_RANK := {"caught": "caught", "landed": "caught", "splashdown": "caught", "impact": "caught"}


## The arc's scale for `now_tdb`: Vector2(knee_s, reach_s) for arc_position. The reach is the time to the furthest of
## the next FUTURE_EVENTS events (with none, the time since the last past event; with neither, MIN_SPAN_S; never
## below MIN_SPAN_S / 2): it maps to the inner edge of the fade zone. The knee sets the compression: as mild as
## possible (knee = reach) unless the nearest next event would then lie closer than NEAR_MIN_POSITION to the apex.
static func target_scale(events_tdb: PackedFloat64Array, now_tdb: float) -> Vector2:
	var past := -INF
	var future: Array[float] = []
	for t in events_tdb:
		if t <= now_tdb:
			past = maxf(past, t)
		else:
			future.append(t)
	future.sort()
	var reach := MIN_SPAN_S
	if not future.is_empty():
		reach = future[mini(FUTURE_EVENTS, future.size()) - 1] - now_tdb
	elif is_finite(past):
		reach = now_tdb - past
	reach = maxf(reach, 0.5 * MIN_SPAN_S)
	var knee := reach
	if not future.is_empty():
		var nearest := future[0] - now_tdb
		if arc_position(nearest, Vector2(knee, reach)) < NEAR_MIN_POSITION:
			var lo := log(KNEE_MIN_S)  # bisection on log(knee): the position of the nearest grows as the knee shrinks
			var hi := log(reach)
			for i in KNEE_ITERATIONS:
				var mid := 0.5 * (lo + hi)
				if arc_position(nearest, Vector2(exp(mid), reach)) < NEAR_MIN_POSITION:
					hi = mid
				else:
					lo = mid
			knee = exp(lo)
	return Vector2(knee, reach)


## [E] A predicted next event that moves by more than this fraction of the shown reach counts as a new event set.
const RESCALE_TOLERANCE := 0.05

## Instance state of the scale target (one per arc): the set of next events it was chosen for and the scale.
var _next_key := PackedFloat64Array()
var _target := Vector2.ZERO


## Scale target that stays fixed while the next events stay the same (fix 2: a target recomputed every frame kept the
## nearest event pinned at NEAR_MIN_POSITION, so it jumped to the apex in its last seconds). It is chosen anew by
## target_scale when an event passes now, a new one is predicted, or a predicted time moves by more than
## RESCALE_TOLERANCE of the reach; in between every event glides steadily towards the apex.
func scale_target(events_tdb: PackedFloat64Array, now_tdb: float) -> Vector2:
	var key := next_events(events_tdb, now_tdb)
	if _target == Vector2.ZERO or _changed(key):
		_next_key = key
		_target = target_scale(events_tdb, now_tdb)
	return _target


## Forgets the chosen scale (a new mission or a time jump).
func reset() -> void:
	_next_key = PackedFloat64Array()
	_target = Vector2.ZERO


## The next FUTURE_EVENTS event times after `now_tdb`, sorted.
static func next_events(events_tdb: PackedFloat64Array, now_tdb: float) -> PackedFloat64Array:
	var future: Array[float] = []
	for t in events_tdb:
		if t > now_tdb:
			future.append(t)
	future.sort()
	return PackedFloat64Array(future.slice(0, FUTURE_EVENTS))


func _changed(key: PackedFloat64Array) -> bool:
	if key.size() != _next_key.size():
		return true
	for i in key.size():
		if absf(key[i] - _next_key[i]) > RESCALE_TOLERANCE * _target.y:
			return true
	return false


## Arc position of an event `dt_s` from now (negative: past) for `scale` = Vector2(knee_s, reach_s): monotone, 0 at
## now, +-visible_half_fraction() at +-reach, logarithmic beyond the knee. |position| > 1 lies off the arc.
static func arc_position(dt_s: float, scale: Vector2) -> float:
	var knee := maxf(scale.x, KNEE_MIN_S)
	return signf(dt_s) * visible_half_fraction() * log(1.0 + absf(dt_s) / knee) / log(1.0 + scale.y / knee)


## Part of the half-arc (apex to end) that is not faded: the reach maps to its edge.
static func visible_half_fraction() -> float:
	return 1.0 - 2.0 * ARC_FADE_FRACTION


## Exponential approach of the displayed scale to the target (frame-rate independent): events glide.
static func ease(current: Vector2, target: Vector2, delta_real_s: float) -> Vector2:
	return current.lerp(target, 1.0 - exp(-maxf(delta_real_s, 0.0) / EASE_TIME_S))


## Events (dictionaries with "kind" and "tdb_s") on the arc (|position| <= 1), thinned so that no two labels are
## closer than MIN_LABEL_PX on an arc of `arc_px`. The higher-priority kind stays; equal priority keeps the earlier
## event. Sorted by time.
static func visible_labels(events: Array, now_tdb: float, scale: Vector2, arc_px: float) -> Array:
	var inside: Array = []
	for e in events:
		if absf(arc_position(float(e["tdb_s"]) - now_tdb, scale)) <= 1.0:
			inside.append(e)
	inside.sort_custom(func(a, b): return float(a["tdb_s"]) < float(b["tdb_s"]))
	var px: Array[float] = []
	for e in inside:
		px.append(0.5 * (arc_position(float(e["tdb_s"]) - now_tdb, scale) + 1.0) * arc_px)
	var order := range(inside.size())
	order.sort_custom(func(a, b):
		var ra := _rank(inside[a]["kind"])
		var rb := _rank(inside[b]["kind"])
		return ra < rb or (ra == rb and a < b))
	var kept: Array[int] = []
	for i in order:
		var free := true
		for k in kept:
			if absf(px[k] - px[i]) < MIN_LABEL_PX:
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
