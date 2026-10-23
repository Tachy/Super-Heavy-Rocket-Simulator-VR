# Headless test of the TomlLite reader (single-line arrays).
# Run: godot --headless --path game --script res://tests/toml_lite_test.gd
extends SceneTree

var _failures := 0


func _initialize() -> void:
	var d := TomlLite.parse('a = ["x", "y"]\nb = [1.5, 2]\nc = []\ne = ["a,b", "c"]\nf = [true, false, "z"]\n[t]\nd = "q # not a comment"')
	_check(d["a"] is Array and d["a"] == ["x", "y"], "string array")
	_check(d["b"] is Array and d["b"] == [1.5, 2.0], "number array")
	_check(d["c"] is Array and d["c"] == [], "empty array")
	_check(d["e"] == ["a,b", "c"], "comma inside a string")
	_check(d["f"] == [true, false, "z"], "mixed array")
	_check(d["t"]["d"] == "q # not a comment", "hash inside a string")
	print("TOML LITE TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _check(condition: bool, message: String) -> void:
	if condition:
		print("  ok: ", message)
	else:
		print("  FAIL: ", message)
		_failures += 1
