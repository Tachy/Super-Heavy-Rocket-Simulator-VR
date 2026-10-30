extends SceneTree
## Compiles every kernel of both cloud march methods once and exits 1 when one fails: the old march's (CloudEffect:
## the march's variants, the temporal pass, the upsample, the outline pass), the baked volume's (CloudBaked, desktop and
## stereo; compiled on its first use in the game) and the shadow maps' (old and baked fill). A broken variant would
## otherwise only show on its first use (an F7 switch, the first stereo frame). Needs a GPU (no --headless):
## godot --path game --script res://tests/cloud_compile_check.gd [-- --baked_only]

var _effect: CloudEffect
var _shadow: CloudShadowEffect
var _frames := 0
var _failed := -1


func _process(_delta: float) -> bool:
	_frames += 1
	if _frames == 1:
		_effect = CloudEffect.new()  # its RenderingDevice and the old march's kernels on the render thread
		_shadow = CloudShadowEffect.new()  # the shadow maps' kernels (old and baked fill), likewise
		return false
	if _frames == 3:
		RenderingServer.call_on_render_thread(_compile_all)
		return false
	if _failed < 0:
		return false
	print("cloud_compile_check: %s" % ("PASSED" if _failed == 0 else "FAILED (%d)" % _failed))
	_effect = null
	_shadow = null
	quit(0 if _failed == 0 else 1)
	return true


func _compile_all() -> void:
	var failed := 0
	if _effect._rd == null:
		push_error("cloud_compile_check: no RenderingDevice (or the old march's kernels failed)")
		_failed = 1
		return
	# -- --baked_only: the baked volume's kernels alone (a quick check while working on them)
	if not "--baked_only" in OS.get_cmdline_user_args():
		for name: String in CloudEffect.old_march_kernel_names():
			if not _effect._shaders.has(name):
				print("cloud_compile_check: old march %s FAILED" % name)
				failed += 1
	for stereo in [false, true]:
		var baked := CloudBaked.new()
		if not baked.setup(_effect._rd, stereo):
			print("cloud_compile_check: baked (stereo %s) FAILED" % stereo)
			failed += 1
		baked.free_all()
	if not (_shadow._pipeline.is_valid() and _shadow._baked_pipeline.is_valid()):
		print("cloud_compile_check: shadow maps (old %s, baked fill %s) FAILED"
				% [_shadow._pipeline.is_valid(), _shadow._baked_pipeline.is_valid()])
		failed += 1
	_failed = failed
