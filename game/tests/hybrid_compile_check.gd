extends SceneTree
## Compiles every hybrid march kernel variant (CloudEffect.HYBRID_FAMILIES x _centre / _prefilter / _refine) and the
## baked volume prototype's kernels (CloudBaked, desktop and stereo) once and
## exits 1 when one fails (the hybrid march prototype compiles its kernels lazily in the game: a broken variant would
## only show on the first toggle). Needs a GPU (no --headless):
## godot --path game --script res://tests/hybrid_compile_check.gd

var _effect: CloudEffect
var _frames := 0
var _failed := -1


func _process(_delta: float) -> bool:
	_frames += 1
	if _frames == 1:
		_effect = CloudEffect.new()  # compiles the march's kernels on the render thread
		return false
	if _frames == 3:
		RenderingServer.call_on_render_thread(_compile_all)
		return false
	if _failed < 0:
		return false
	print("hybrid_compile_check: %s" % ("PASSED" if _failed == 0 else "FAILED (%d)" % _failed))
	_effect = null
	quit(0 if _failed == 0 else 1)
	return true


func _compile_all() -> void:
	var failed := 0
	if _effect._rd == null:
		push_error("hybrid_compile_check: no RenderingDevice (or the march's kernels failed)")
		_failed = 1
		return
	# -- --baked_only: the baked volume's kernels alone (a quick check while working on them)
	var families: Array = [] if "--baked_only" in OS.get_cmdline_user_args() else CloudEffect.HYBRID_FAMILIES.values()
	for family: String in families:
		for centre in ["", "_centre"]:
			for prefilter in ["", "_prefilter"]:
				for refine in ["", "_refine"]:
					var name: String = family + centre + prefilter + refine
					if not _effect._ensure_hybrid_kernel(name):
						print("hybrid_compile_check: %s FAILED" % name)
						failed += 1
	# The baked volume prototype (CloudBaked; compiled on its first use in the game): desktop and stereo kernels.
	for stereo in [false, true]:
		var baked := CloudBaked.new()
		if not baked.setup(_effect._rd, stereo):
			print("hybrid_compile_check: baked (stereo %s) FAILED" % stereo)
			failed += 1
		baked.free_all()
	_failed = failed
