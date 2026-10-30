extends SceneTree
## Compiles every cloud kernel once and exits 1 when one fails: the pass's upsample (CloudEffect), the baked volume's
## (CloudBaked, desktop and stereo) and the shadow maps' fill (CloudShadowEffect). A broken variant would otherwise only
## show on its first use (the first stereo frame). The builds run on workers (CloudKernels) while this script polls them
## frame by frame - as the game does: a thread blocked on them deadlocked. Needs a GPU (no --headless):
## godot --path game --script res://tests/cloud_compile_check.gd
## Environment STARSIM_CLOUD_SHADER_CACHE=0 compiles everything (no cached binaries).

const MAX_FRAMES := 6000

var _effect: CloudEffect
var _shadow: CloudShadowEffect
var _baked: Array[CloudKernels] = []  ## desktop, stereo
var _frames := 0
var _failed := -1


func _process(_delta: float) -> bool:
	_frames += 1
	if _frames == 1:
		_effect = CloudEffect.new()  # its RenderingDevice and the pass kernels' build (render thread)
		_shadow = CloudShadowEffect.new()  # the shadow maps' kernel, likewise
		for stereo in [false, true]:
			_baked.append(CloudKernels.new(CloudBaked.kernel_specs(stereo), "baked" + (" stereo" if stereo else "")))
		return false
	if _failed < 0 and _frames > 2 and _built():
		RenderingServer.call_on_render_thread(_create_all)
	if _failed < 0 and _frames < MAX_FRAMES:
		return false
	print("cloud_compile_check: %s" % ("PASSED" if _failed == 0 else "FAILED (%d)" % maxi(_failed, 1)))
	_effect = null
	_shadow = null
	quit(0 if _failed == 0 else 1)
	return true


## True once every build is done.
func _built() -> bool:
	if _effect._rd == null:
		return true
	var sets: Array[CloudKernels] = [_effect._pass_kernels]
	sets.append_array(_baked)
	if _shadow._kernels != null:
		sets.append(_shadow._kernels)
	for k in sets:
		if k == null or not k.done():
			return false
	return true


## Render thread: every kernel created from its build.
func _create_all() -> void:
	if _failed >= 0:
		return
	var failed := 0
	if _effect._rd == null:
		push_error("cloud_compile_check: no RenderingDevice")
		_failed = 1
		return
	if not _effect._pass_ready():
		print("cloud_compile_check: the upsample FAILED (%d)" % _effect._pass_state)
		failed += 1
	for name: String in CloudEffect.PASS_KERNELS:
		if not _effect._shaders.has(name):
			print("cloud_compile_check: %s FAILED" % name)
			failed += 1
	for i in range(_baked.size()):
		var baked := CloudBaked.new()
		if not baked.setup(_effect._rd, _baked[i]):
			print("cloud_compile_check: baked (stereo %s) FAILED" % (i == 1))
			failed += 1
		baked.free_all()
	if not (_shadow._kernels_ready() and _shadow._pipeline.is_valid()):
		print("cloud_compile_check: the shadow maps' fill FAILED")
		failed += 1
	_failed = failed
