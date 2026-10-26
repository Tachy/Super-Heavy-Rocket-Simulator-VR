# Headless test of World.clock_held_back: the world clock waits for the flight sim only when the sim's snapshot is
# behind the instant it was asked for in the previous frame (its normal one-frame trail is no lag). Frame-time jitter at
# high time warp must not hold the clock back (that halved the flight's speed).
# Run: godot --headless --path game --script res://tests/clock_hold_test.gd
extends SceneTree

var _failures := 0


func _initialize() -> void:
	# A sim that keeps up reaches exactly last frame's target, whatever this frame's delta (11 / 22 ms alternating in VR
	# reprojection) and the warp (1000x: a frame step of 11-22 s).
	var target := 1000.0
	var held := 0
	for frame in 200:
		var delta := 0.011 if frame % 2 == 0 else 0.022
		var reached := target  # the snapshot of last frame's request
		target += delta * 1000.0
		if World.clock_held_back(reached, reached):
			held += 1
	_check(held == 0, "a sim that keeps up never holds the clock (held %d of 200 frames at 1000x with jitter)" % held)
	_check(World.clock_held_back(5000.0, 5000.0 - 2.0), "a sim 2 s behind last frame's target holds the clock")
	_check(not World.clock_held_back(5000.0, 5000.0 - 0.01), "10 ms behind is within the slack")
	_check(not World.clock_held_back(NAN, 4000.0), "no previous target (first frame, after a jump): no hold")
	print("CLOCK HOLD TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _check(condition: bool, message: String) -> void:
	if condition:
		print("  ok: ", message)
	else:
		print("  FAIL: ", message)
		_failures += 1
