class_name CloudViews
extends RefCounted
## The cloud world-volume plan's gate views V1-V7 as render_check arguments (render_check --cloud_view=<id>; the spike
## and the gate run them through the stereo harness, StereoMockXr). VIEWS holds one argument list per view id; a view
## the plan marks with * also exists with analog weather (<id>_analog: --weather=analog) and with the ERA5 snapshot of
## 2025-10-25 15:00 UTC (<id>_era5: --weather=<data/weather/cache>/ERA5_FILE); args() and ids() build those from VIEWS.
## An argument given on the command line after --cloud_view wins over the view's own (render_check parses the view's
## first), so a run can lengthen V2's --frames_for.

const WEATHER_CACHE_DIR := "data/weather/cache"
## The cache file the weather service writes for 2025-10-25 15:00 UTC (key = hours since 1900-01-01; found by
## `render_check --utc=2025-10-25T15:00:00 --weather=online --lat=25.997 --lon=-97.157`, task 1 step 6).
const ERA5_FILE := "era5_1102887.wthr"

## Views with analog / ERA5 variants (the plan's *).
const VARIED := ["V1", "V3", "V4_20km", "V4_80km", "V5_nadir", "V5_limb"]

const VIEWS := {
	# pad, broken Cu
	"V1": ["--lat=25.997", "--lon=-97.157", "--utc=2024-06-15T21:00:00", "--weather=slab:1000,2500,0.0003,0.5",
			"--azimuth=90", "--elevation=10", "--fov=100"],
	# ascent through the deck
	"V2": ["--vehicle=starship_block3", "--site=starbase_olp1", "--autopilot", "--fly=25",
			"--weather=slab:1500,2500,0.0003,0.6", "--frames_every=0.5", "--frames_for=120"],
	# 9 km above a broken deck
	"V3": ["--lat=25.997", "--lon=-97.157", "--utc=2024-06-15T21:00:00", "--weather=slab:1500,2500,0.0003,0.5",
			"--eye_height=9000", "--azimuth=0", "--elevation=-10"],
	# 20 km and 80 km, looking down at the slab (cloud fraction 0.5)
	"V4_20km": ["--lat=25.997", "--lon=-97.157", "--utc=2024-06-15T21:00:00", "--weather=slab:1500,2500,0.0003,0.5",
			"--eye_height=20000", "--elevation=-15"],
	"V4_80km": ["--lat=25.997", "--lon=-97.157", "--utc=2024-06-15T21:00:00", "--weather=slab:1500,2500,0.0003,0.5",
			"--eye_height=80000", "--elevation=-20"],
	# 400 km: nadir and limb (analog weather; the slab views end at V4)
	"V5_nadir": ["--utc=2024-06-15T10:00:00", "--lat=0", "--lon=0", "--weather=analog",
			"--eye_height=400000", "--elevation=-90", "--fov=40"],
	"V5_limb": ["--utc=2024-06-15T10:00:00", "--lat=0", "--lon=0", "--weather=analog",
			"--eye_height=400000", "--elevation=-19", "--fov=40"],
	# backlit (the Sun in view) and at sunset (the Sun 3.0 deg up:
	# 2024-04-10T00:26:00 UTC, 3.01 deg in render_check)
	"V6_backlit": ["--lat=30", "--lon=-95", "--weather=slab:1000,2000,1e-4,0.5", "--utc=2024-04-09T23:30:00",
			"--look_at_sun"],
	"V6_sunset": ["--lat=30", "--lon=-95", "--weather=slab:1000,2000,1e-4,0.5", "--utc=2024-04-10T00:26:00",
			"--look_at_sun"],
	# booster catch: --view=1 is the booster (one Tab press from the ship; found by flying the mission to the event: view 0
	# is the ship at 183 km, view 1 the booster). 45 s before the catch the booster is at 9.8 km, 650 m/s down: the run
	# covers the descent from above the deck (1.5 - 2.5 km) through it to the tower.
	"V7": ["--webcast", "--mission=starship_leo", "--at_event=caught,-45", "--view=1",
			"--weather=slab:1500,2500,0.0003,0.6",
			"--frames_every=0.5", "--frames_for=40"],
}


## The argument list of a view id ("V3", "V3_analog", "V3_era5", "V4_80km_era5", ...); empty for an unknown id.
static func args(id: String) -> PackedStringArray:
	if VIEWS.has(id):
		return PackedStringArray(VIEWS[id])
	for suffix in ["_analog", "_era5"]:
		var base := id.trim_suffix(suffix)
		if id.ends_with(suffix) and base in VARIED:
			var weather := "--weather=analog" if suffix == "_analog" else "--weather=" + era5_path()
			var result := PackedStringArray()
			for argument: String in VIEWS[base]:
				result.append(weather if argument.begins_with("--weather=") else argument)
			return result
	return PackedStringArray()


## Every view id, the variants included.
static func ids() -> PackedStringArray:
	var result := PackedStringArray(VIEWS.keys())
	for base: String in VARIED:
		result.append(base + "_analog")
		result.append(base + "_era5")
	return result


## The absolute path of the ERA5 snapshot (the repo's data/weather/cache; render_check runs from the game project).
static func era5_path() -> String:
	return ProjectSettings.globalize_path("res://").path_join("..").path_join(WEATHER_CACHE_DIR).path_join(ERA5_FILE) \
			.simplify_path()
