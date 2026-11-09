class_name HandMotion
extends RefCounted
## The suited hand's smoothed motion (docs/subsystems/cockpit.md, "Console pointer"): user wish 2026-11-07, "geglättete Bewegungen,
## keine eckigen und ruckartigen". The hand position follows its target as a critically damped spring (exact solution, stable at any
## frame time, no overshoot for a target that was at rest or ahead of the motion); a tap is a short eased press and release along
## the surface normal. State is the position and velocity only.

## [E] The response to a step reaches half of it after this time (user wish: smooth, not sluggish; a 30 cm move settles in ~0.4 s).
const HALF_LIFE_S := 0.08
## Solution of (1 + x) e^-x = 0.5 for the critically damped step response: omega = X_HALF / half life.
const X_HALF := 1.678347
const PRESS_S := 0.12  ## [E] plan Task 8: the finger goes in
const RELEASE_S := 0.15  ## [E] plan Task 8: and out
## [E] Hover distance of the fingertip over the surface, and the depth of a tap (the tip ends ON the plane).
const HOVER_M := 0.025

var position := Vector3.ZERO
var velocity := Vector3.ZERO
var _tap_time_s := -1.0  ## < 0: idle; 0..PRESS_S press; PRESS_S..PRESS_S + RELEASE_S release
var _tap_queued := false


static func omega() -> float:
	return X_HALF / HALF_LIFE_S


## Puts the hand at `p` at rest.
func reset(p: Vector3) -> void:
	position = p
	velocity = Vector3.ZERO
	_tap_time_s = -1.0
	_tap_queued = false


## One frame: the position after `delta` seconds towards `target`.
func step(target: Vector3, delta: float) -> Vector3:
	if delta > 0.0:
		var w := omega()
		var offset := position - target
		var j := velocity + offset * w
		var decay := exp(-w * delta)
		position = target + (offset + j * delta) * decay
		velocity = (velocity - j * (w * delta)) * decay
		_advance_tap(delta)
	return position


## Starts a tap (a tap during one is queued and starts when the finger is out).
func tap() -> void:
	if _tap_time_s < 0.0:
		_tap_time_s = 0.0
	else:
		_tap_queued = true


func is_tapping() -> bool:
	return _tap_time_s >= 0.0


## How far the tap has moved the fingertip towards the surface from the hover height (m): 0 .. HOVER_M, peak at the end of the press.
func tap_depth() -> float:
	if _tap_time_s < 0.0:
		return 0.0
	if _tap_time_s <= PRESS_S:
		return HOVER_M * _ease(_tap_time_s / PRESS_S)
	return HOVER_M * (1.0 - _ease((_tap_time_s - PRESS_S) / RELEASE_S))


func _advance_tap(delta: float) -> void:
	if _tap_time_s < 0.0:
		return
	_tap_time_s += delta
	if _tap_time_s >= PRESS_S + RELEASE_S:
		_tap_time_s = 0.0 if _tap_queued else -1.0
		_tap_queued = false


## Smootherstep: zero speed and acceleration at both ends (no jerk at contact or lift-off).
static func _ease(x: float) -> float:
	var t := clampf(x, 0.0, 1.0)
	return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
