# Lunar picker end to end (lunar sites plan Task 13; needs a GPU, no --headless, the Apollo 15 tiles on disk): the real
# game with the real core - the time panel's mission box set to a lunar landing, the picker's preset Apollo 15, "search
# dates", window 1 chosen and started through the picker (terrain switch + FlightSim.start_mission). Saves the picker
# with its listed rows, then flies the started mission and saves the view after the TLI and after the touchdown.
# godot --path game --script res://tests/lunar_picker_e2e.gd -- --out=<dir> [--mission=hls_lunar_llo] [--site=apollo_15]
#       [--stay_h=24]
# Exit code 0 when the mission started, landed (lunar step landed / surface stay) and the images were written.
extends SceneTree

## Upper bound on each wait (frames).
const MAX_FRAMES := 20000
## Frames the view settles before an image.
const SETTLE_FRAMES := 240
## The flight is advanced in these chunks so the log shows its progress (s).
const FLIGHT_CHUNK_S := 6.0 * 3600.0

var _args := {}
var _main: Node
var _panel: TimePanel
var _phase := "boot"
var _frames := 0
var _wait := 0
var _started_ok := false
var _start_report := {}
var _window := {}
var _exit := 1


func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		var parts := arg.trim_prefix("--").split("=", true, 1)
		_args[parts[0]] = parts[1] if parts.size() > 1 else ""
	_main = load("res://main.tscn").instantiate()
	root.add_child(_main)


func _process(_delta: float) -> bool:
	_frames += 1
	if _frames > MAX_FRAMES * 4:
		return _fail("timed out in phase %s" % _phase)
	var world: World = _main.get_node_or_null("World")
	if world == null or not world.booted:
		return false
	match _phase:
		"boot":
			_panel = _find_panel(_main)
			if _panel == null:
				return false
			if not _select_mission(String(_args.get("mission", "hls_lunar_llo"))):
				return _fail("mission not offered in the time panel")
			var picker: LunarPicker = _panel._lunar_picker
			picker.select_preset(String(_args.get("site", "apollo_15")))
			var fields := picker.read_fields()
			picker.set_fields(picker._lat_edit.text, picker._lon_edit.text, String(_args.get("stay_h", "24")))
			print("lunar_picker_e2e: fields ", fields, " stay ", picker._stay_edit.text)
			picker.search()
			_phase = "searching"
			_wait = 0
		"searching":
			var picker: LunarPicker = _panel._lunar_picker
			_wait += 1
			if picker.state == LunarPicker.State.LISTED:
				print("lunar_picker_e2e: search done after %d frames, status '%s'" % [_wait, picker.status_text()])
				for line in picker.list_texts():
					print("lunar_picker_e2e:   ", line)
				if picker.windows.is_empty():
					return _fail("no window listed")
				picker.select_window(0)
				_window = picker.windows[0]
				print("lunar_picker_e2e: window 1 ", _window)
				_phase = "listed"
				_wait = 0
			elif picker.state == LunarPicker.State.IDLE and _wait > 10:
				return _fail("search ended without a list: " + picker.status_text())
			elif _wait > MAX_FRAMES:
				return _fail("search did not finish")
		"listed":
			_wait += 1
			if _wait == SETTLE_FRAMES / 4:
				_save("picker_e2e_list.png")
				var picker: LunarPicker = _panel._lunar_picker
				world.flight.mission_started.connect(func(ok: bool, report: Dictionary) -> void:
					_started_ok = ok
					_start_report = report
					_phase = "started" if ok else "refused")
				picker.start_chosen()
				_phase = "starting"
				_wait = 0
		"starting":
			_wait += 1
			world.flight.sim.advance_to(world.clock.tdb_s)
			if _wait > MAX_FRAMES:
				return _fail("start did not finish: " + _panel._lunar_picker.status_text())
		"refused":
			return _fail("mission refused: " + I18n.report_text(_start_report))
		"started":
			var sim := world.flight.sim
			print("lunar_picker_e2e: mission started at %.1f (window start %.1f), status '%s'" % [
					sim.mission_start_tdb_s(), float(_window["mission_start_tdb_s"]), _panel._lunar_picker.status_text()])
			world.set_focus(World.FOCUS_VEHICLE)
			world.clock.warp = 0.0
			var tli := _event_tdb_s(sim, "tli")
			var target := (tli if not is_nan(tli) else sim.mission_start_tdb_s()) + 3600.0
			_fly(world, target)
			_phase = "coast"
			_wait = 0
		"coast":
			world.clock.warp = 0.0
			_wait += 1
			if _wait == SETTLE_FRAMES:
				_print_state(world, "after the TLI")
				_save("picker_e2e_coast.png")
				_fly(world, float(_window["touchdown_tdb_s"]) - 3.0 * 3600.0)
				_phase = "descent"
				_wait = 0
		"descent":
			# fly on until the touchdown is recorded (the session's lunar step), at most 12 h past the listed touchdown
			var sim := world.flight.sim
			var end := float(_window["touchdown_tdb_s"]) + 12.0 * 3600.0
			var t := sim.advance_to(world.clock.tdb_s)
			var landed_at := _event_tdb_s(sim, "landed", true)
			if not is_nan(landed_at):
				print("lunar_picker_e2e: touchdown recorded %.3f h from the listed one" % [
						(landed_at - float(_window["touchdown_tdb_s"])) / 3600.0])
				_fly(world, landed_at + 300.0)
				_phase = "landed"
				_wait = 0
			elif t >= end or not sim.is_active():
				_print_state(world, "no touchdown")
				return _fail("no touchdown within 12 h of the listed one")
			else:
				_fly(world, minf(t + 600.0, end))
		"landed":
			world.clock.warp = 0.0
			_wait += 1
			if _wait == SETTLE_FRAMES * 2:
				_print_state(world, "landed")
				_save("picker_e2e_landed.png")
				_exit = 0
				quit(_exit)
				return true
	return false


func _find_panel(node: Node) -> TimePanel:
	if node is TimePanel:
		return node
	for child in node.get_children():
		var found := _find_panel(child)
		if found != null:
			return found
	return null


## The time panel's place / vehicle / mission boxes set to the first combination that offers `mission_id`.
func _select_mission(mission_id: String) -> bool:
	for p in _panel._places.item_count:
		_panel._places.select(p)
		_panel._refill_vehicles()
		for v in _panel._vehicles.item_count:
			_panel._vehicles.select(v)
			_panel._refill_missions()
			for m in _panel._mission_list.size():
				if _panel._mission_list[m]["id"] == mission_id:
					_panel._missions.select(m)
					_panel._update_lunar_picker()
					return true
	return false


## The first event of `kind` in the mission timeline (recorded only when `recorded_only`), NAN if none.
func _event_tdb_s(sim: FlightSim, kind: String, recorded_only := false) -> float:
	for e: Dictionary in sim.mission_timeline().get("events", []):
		if e["kind"] == kind and (not recorded_only or not e["predicted"]):
			return float(e["tdb_s"])
	return NAN


## The simulation advanced to `target` (in chunks, logged), the clock set there.
func _fly(world: World, target: float) -> void:
	var sim := world.flight.sim
	var t := sim.advance_to(target)
	while t < target - 0.01 and sim.is_active():
		var step := minf(t + FLIGHT_CHUNK_S, target)
		for i in 1000:
			sim.advance_to(step)
			sim.wait_idle(5.0)
			if sim.advance_to(step) >= step - 0.01 or not sim.is_active():
				break
		t = sim.advance_to(target)
	world.clock.set_time(sim.advance_to(target))


func _print_state(world: World, label: String) -> void:
	var sim := world.flight.sim
	var index := world.view_target.resolve(sim)
	print("lunar_picker_e2e: %s at %.1f: events %s" % [label, sim.advance_to(world.clock.tdb_s),
			sim.mission_timeline().get("events", []).map(func(e): return "%s%s" % [e["kind"], "?" if e["predicted"] else ""])])
	print("lunar_picker_e2e: telemetry ", sim.telemetry(index) if index >= 0 else {})


func _save(name: String) -> void:
	var dir := String(_args.get("out", "user://"))
	DirAccess.make_dir_recursive_absolute(dir)
	var path := dir.path_join(name)
	var error := root.get_viewport().get_texture().get_image().save_png(path)
	print("lunar_picker_e2e: saved %s (error %d)" % [path, error])


func _fail(message: String) -> bool:
	push_error("lunar_picker_e2e: " + message)
	print("lunar_picker_e2e: FAILED ", message)
	quit(1)
	return true
