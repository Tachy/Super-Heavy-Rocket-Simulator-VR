# Falcon 9 c5: Game Interface Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The game interface (`FlightSim` bindings) delivers everything the Falcon 9 missions need for rendering and the
HUD: mission start, landing zone / drone ship geometry, leg and grid-fin geometry and state, touchdown judgement.

**Scope (user 2026-10-14):** the graphical presentation and the UI are defined by the user first; this plan only builds
the interface. The drawing tasks (vehicle legs/fins, landing-zone and drone-ship views, mission menu, HUD lines) follow
once the UI is specified.

**Architecture:** The bindings (`FlightSim`) get thin read-outs of what the core already has (mission start, landing
zone with its deck, leg and grid-fin geometry and state); all drawing stays in GDScript (`vehicle_view.gd`, a new
`landing_zone_view.gd`), no physics in the game. A camera key follows any body, so the returning first stage can be
watched while the upper stage flies on.

**Tech Stack:** C++20 GDExtension (godot-cpp, double), Godot 4.7 GDScript, TOML config.

**Spec:** `docs/superpowers/specs/2026-10-13-falcon9-landing-design.md` (section "c5 design: game")

## Global Constraints

- UI text German, code/comments/docs English; bindings stay thin (no physics or business logic).
- The game only reads simulation state and sends commands. Physics never runs in GDScript.
- What is drawn is what the physics uses (geometry from the core's config via the bindings, as `site_tower`).
- Rendering: floating origin; positions differenced in double before float32.
- No new magic numbers in the core; drawing-only dimensions (fin thickness, hull depth) are named constants in the view.

## Review Focus

1. Starting a mission while another flight is active: the old views, landing-zone drawing and trajectory must go.
   Test: smoke test starts `falcon9_rtls` then `falcon9_asds` on one `FlightSim`; `landing_zone()` reports the deck.
2. A Starship flight (no landing zone): `landing_zone()` must report `present = false` and nothing is drawn.
   Test: smoke test after `launch_from_site("starship_block3", ...)`.
3. Bodies without legs or fins (upper stage, Starship ship): `body_legs` / `body_grid_fins` return empty data, no errors.
   Test: smoke test on body 0 of a Starship stack (fins yes, legs none).
4. A plain launch after a mission on the same `FlightSim`: the mission's landing zone must not linger.
   Test: smoke test `launch_from_site("falcon9_block5", "ccsfs_slc40")` after `start_mission` -> no landing zone.
5. Invalid body indices (a stage that no longer exists): empty results with a pushed error, no crash.
   Test: smoke test `body_legs(7)` / `body_grid_fins(7)` return empty.

---

### Task 1: Bindings for missions, landing zone, legs and grid fins

**Files:**
- Modify: `bindings/src/flight_sim.hpp`, `bindings/src/flight_sim.cpp`
- Test: `game/tests/smoke_test.gd`

**Interfaces:**
- Produces (GDScript-visible on `FlightSim`):
  - `start_mission(mission_id: String, tdb_s: float) -> bool` - `config::load_ascent_mission` + `sim::start_ascent_mission`.
  - `mission_site_id() -> String` - launch site id of the current flight (from `launch_from_site` or the mission).
  - `landing_zone() -> Dictionary` - `{present}`; when present `{origin_itrf_m, east, north, up (ITRF unit vectors),
    pad_radius_m, has_deck}` and with a deck `{deck_length_m, deck_width_m, freeboard_m, heading_deg, sea_height_m,
    latitude_deg, longitude_deg}`. Origin = the zone's surface (deck top for a ship).
  - `body_legs(index) -> Dictionary` - `{present}`; when present `{deployment (0..1), touchdown (German, "" while
    none), base_station_m, hull_radius_m, foot_station_m, footprint_radius_m, azimuths_deg: Array}`.
  - `body_grid_fins(index) -> Array` - `[{station_m, azimuth_deg, radius_m, area_m2, deflection_deg}]`.
  - `return_state(index)` additionally carries `legs` = `body_legs(index)` (HUD).

- [ ] **Step 1: Write the failing smoke checks** (append before the PASSED line in `smoke_test.gd`):

```gdscript
	# Falcon 9 missions (c5): mission start, landing zone with and without a deck, legs and grid fins.
	var f9 := FlightSim.new()
	f9.setup(config_dir)
	_check(f9.start_mission("falcon9_rtls", tdb), "Falcon 9 RTLS mission starts")
	var zone: Dictionary = f9.landing_zone()
	_check(zone.get("present", false) and not zone.get("has_deck", true), "RTLS: landing zone without a deck")
	_check(absf(float(zone.get("pad_radius_m", 0.0)) - 42.7) < 1e-9, "RTLS: LZ-40 pad radius from the config")
	_check(f9.mission_site_id() == "ccsfs_slc40", "RTLS: launch site of the mission")
	_check(f9.start_mission("falcon9_asds", tdb), "Falcon 9 drone-ship mission starts on the same FlightSim")
	zone = f9.landing_zone()
	_check(zone.get("has_deck", false) and absf(float(zone.get("deck_length_m", 0.0)) - 91.0) < 1e-9,
			"ASDS: the drone ship's deck from the config")
	var legs: Dictionary = f9.body_legs(0)
	_check(legs.get("present", false) and (legs.get("azimuths_deg", []) as Array).size() == 4
			and float(legs.get("deployment", 1.0)) == 0.0, "Falcon 9 stack: four stowed legs")
	_check(f9.body_grid_fins(0).size() == 4, "Falcon 9 stack: four grid fins")
	_check(not f9.start_mission("no_such_mission", tdb), "unknown mission is refused")
	var ship := FlightSim.new()
	ship.setup(config_dir)
	_check(ship.launch_from_site("starship_block3", "starbase_olp1", 0.0, tdb), "Starship launches")
	_check(not ship.landing_zone().get("present", true), "Starship: no landing zone")
	_check(not ship.body_legs(0).get("present", true), "Starship: no legs")
```
(`config_dir` = the globalized config path already used for the tower check; hoist it into a variable.)

- [ ] **Step 2: Run the smoke test** - Expected: script error / FAIL (methods missing).
- [ ] **Step 3: Implement** the methods in `flight_sim.{hpp,cpp}` (bind them in `_bind_methods`). `start_mission`
  wraps loading and starting in try/catch (push_error, false); it remembers `mission_site_id_`. `landing_zone` uses
  `session_->landing_zone()`, `frames::geodetic_to_fixed`, and the ENU basis (east = (-sin lon, cos lon, 0), north,
  up from geodetic lat/lon); sea height = deck height - freeboard. `body_legs` reads the lowest stage's
  `config.landing_legs`, `Vehicle::leg_deployment()`, `session_->leg_touchdown(i)` (`to_string`, "" for None).
- [ ] **Step 4: Build with bindings and run the smoke test** - Expected: BUILD_OK, SMOKE TEST PASSED.
- [ ] **Step 5: Commit** `feat(bindings): Falcon 9 missions, landing zone, legs and grid fins for the game`

### Task 2: Docs

- [ ] **Step 1:** `CLAUDE.md`: Falcon 9 bullet notes the bindings (c5 interface), game drawing pending the user's UI.
- [ ] **Step 2:** Full ctest + smoke test; commit `docs: Falcon 9 game interface (c5)`.
