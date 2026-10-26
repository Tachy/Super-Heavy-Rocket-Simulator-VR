class_name FrameProfiler
## Named main-thread sections of a frame for the perf log (`--perf-log`, PerfLog): begin / end around a block add its
## wall time to this frame's total of that section; PerfLog takes the frame's totals once per frame. Off (the default)
## both calls return at once, so the instrumented code costs nothing measurable in play.

## The columns of the perf log, in update order (World.update_world with flight_views split into its fv_* parts,
## then the UI's and the XR manager's _process; ui_overlay, ui_navball, ui_webcast_draw are _draw, after _process).
const SECTIONS: Array[StringName] = [&"flight_advance", &"weather", &"towers_zone", &"ephemeris", &"camera",
		&"atmosphere", &"clouds", &"refraction", &"earth_terrain", &"water", &"moon", &"shader_globals", &"sun",
		&"flight_views", &"fv_trajectory", &"fv_trail", &"fv_instruments", &"fv_vehicles", &"moon_orbit", &"ui_hud",
		&"ui_overlay", &"ui_navball", &"ui_time_panel", &"ui_webcast", &"ui_webcast_draw", &"xr"]

static var enabled := false
static var _starts := {}  ## section -> usec at begin
static var _frame := {}  ## section -> usec summed this frame


static func begin(section: StringName) -> void:
	if enabled:
		_starts[section] = Time.get_ticks_usec()


static func end(section: StringName) -> void:
	if not enabled or not _starts.has(section):
		return
	_frame[section] = int(_frame.get(section, 0)) + Time.get_ticks_usec() - int(_starts[section])
	_starts.erase(section)


## This frame's totals (section -> usec); clears them for the next frame.
static func take_frame() -> Dictionary:
	var frame := _frame
	_frame = {}
	return frame
