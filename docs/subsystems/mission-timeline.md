# Mission timeline

History and the reasons behind superseded decisions: `docs/history/mission-timeline.md`.
Plan: `docs/superpowers/archive/plans/2026-10-22-external-view.md` (Tasks 1-4).

## Model
Pure data in `sim::MissionTimeline` (`mission_timeline.hpp`): recorded (past) and predicted events, each with body
serial and the acting part's name.
- **T-0 rules:** the first recorded Liftoff; on a pad mission the scheduled liftoff; else the first recorded Ignition /
  Undocking, then the earliest predicted one. The mission clock is `now - T-0` (T- before, T+ after).
- **Recorded:** by `FlightSession` at exact moments (liftoff, stage / payload / trunk separation; the staging
  sequence's MECO at hot staging and the upper stage's ignition at its command - the observer skips an Ignition
  recorded within two samples -, hard capture, undocking at the physical separation - mechanism phase Separated,
  `hooks_open_s` 180 s after the command -, crew ejection) and by `MissionEventObserver` (0.1 s sampling while
  ticking, after each coast: engine starts / cutoffs, booster return phases, Max Q, entry interface, parachutes,
  landing outcomes; a catch is stamped with the pins' first touch, `CatchTowerSim::first_contact_epoch`: the catch
  moment as the webcast calls it, ruling 2026-10-22). A planned burn flown on RCS translation (no main engine fires:
  Crew Dragon, a small Starship burn on cold gas) records Ignition and Cutoff at the burn executor's start and end
  (both in one tick when it starts and ends in the same tick; a disengage of a lit burn records its Cutoff), named by
  the upper stage (its thrusters); RCS attitude control and proximity translation are no events.
- **Predicted** (`predicted_mission_events`, `predicted_events.cpp`; set at the end of every `advance_to`): the
  scheduled liftoff (while the ascent autopilot is armed in PreLaunch), a lunar ascent's liftoff, the docked pair's
  separation (`FlightSession::undock_separation_epoch`: the ISS mission's undocking epoch + hooks opening, or the
  hooks' end while they open), the pending burn's ignition (from the executor's time to ignition: a finite burn starts
  before its impulsive node), the deorbit burn and entry interface of a ship / capsule return (the entry interface
  also while the burn is lit), the not yet commanded burns of the lunar outbound, descent (incl. PDI) and return plans
  (with the braking burn and the entry interface) and of the rendezvous plan, and the campaign supervisor's next launch
  (Launch). A plan burn within `same_burn_s` 600 s [E] of the pending or a commanded burn is that burn (re-targeted at
  the command) and not shown twice. Re-plans move the predicted events.
- **Onboard predictions** (user 2026-10-22: every event the onboard computer has determined, as soon as it has; only
  from what the guidance computes or its own criterion run forward - no nominal profiles), with the kind and body
  serial the event will be recorded with; an event a body has once (MECO, separation, SECO, Boostback, landing burn,
  parachutes, catch / landing / splashdown) is no longer predicted once recorded for that body:
  - MECO and stage separation (AutopilotState::Booster, from the end of the open loop's pitch kick - kick start +
    `kick_duration_s`; the cutoff threshold is the return's live "MECO now" plan held constant over the forecast, and
    that plan swings in the first ~16 s, so MECO, separation and the upper ignition are withheld until the kick has
    ended; the timeline test asserts none of them is shown before): the booster's propellant above the cutoff
    propellant the autopilot set on its last tick (`booster_cutoff_propellant_kg`: the return's need or the reserve)
    at the stage's actual flow now scaled with the throttle the open loop will command - the operational cap and the
    max-Q limiter run forward on a point mass (`booster_burn_time_s` in `predicted_events.cpp`: the open loop's
    vertical rise, kick and gravity turn, the vehicle's drag table, US76, central gravity, 0.5 s steps [E]; free
    launches: within 0.02 s (Starship) / 0.32 s (Falcon 9) of the flown MECO from the kick's end on). The separation
    and the upper stage's ignition follow the staging sequence's timing (`separation_after_meco_s`,
    `upper_ignition_after_meco_s`; in the state Staging from the sequence's own epochs). Measured (planned kick, shown
    from the kick's end): starship_leo +5.59 s at T+30 s, at most 6.58 s over every shown sample, 2.21 s within 10 s;
    Falcon 9 RTLS +7.9 s at T+30 s (with the schedule: +7.0 / +7.4 s at T+30 s). The prediction is late by the
    return's cutoff propellant, which grows with the downrange speed (the value of the moment is used). Not on the
    pad: the return's cutoff propellant is only planned in flight.
  - SECO: `AscentGuidance::cutoff_epoch` (closed-loop update + PEG time to go) from the upper stage's first
    closed-loop tick: -1.9 s then, within 0.3 s over the last 120 s.
  - Booster return (`BoosterReturnGuidance::forecast`): the boostback's cutoff (velocity to be gained less the
    tail-off, rocket equation at full thrust; -1.6 s at its start, 0.2 s within 10 s); the landing burn (LandingBurn,
    from the apex: the ballistic impact prediction less the fall, at the sink speed now, over the height the
    landing-burn prediction takes - within 5.4 s, 1.4 s in the last 30 s; with an entry burn (Falcon 9) only after
    it); the touchdown (Caught / Landed: that ignition plus the burn prediction's duration, then the landing burn's
    time to go - the SCvx plan's end; Super Heavy: -16 s at the apex, 4.3 s within 30 s, 0.9 s within 10 s of the
    pins' first touch).
  - Ship / capsule entry (`EntryGuidance::gate_epoch`: the corrector's or the terminal steering's predicted time to
    the gate): the ship's landing burn (flip at the gate; ship_return_starbase: +28 s 26 min ahead, 0.3 s within
    2 min) or the capsule's drogues (Parachutes; Crew Dragon: within 3.5 s, 1.8 s at the end). Then
    `ShipLandingGuidance::catch_epoch` (the landing burn plan's end; -2.9 s 18 s ahead, -1.3 s at the end).
  - The entry interface of a capsule return whose trunk is still on carries the capsule's serial: the trunk keeps the
    stack's at the jettison, the capsule becomes the next body (`FlightSession::next_body_serial`).
  - Not predicted (no guidance holds a time): the boostback ignition (the flip ends on attitude), Falcon 9's entry
    burn, the splashdown (parachute descent), the docking (the final approach has holds) and a propellant transfer's
    end. Max Q is never predicted.
- **Transfer telemetry:** `FlightBody` telemetry carries `speed_inertial_mps` (|v| about the reference body, not the
  Earth's centre: the lunar orbit's speed is shown about the Moon), `distance_earth_m` / `distance_moon_m` (from the
  centres) and `speed_earth_mps` (|v_gcrf|) / `speed_moon_mps` (|v_gcrf - v_moon|). They are in the telemetry CSV and
  the legacy HUD and drive the webcast's transfer situation (`gauge_rules.gd`: altitude > 5000 km and apoapsis
  > 100000 km [E], with hysteresis `transfer_exit_factor` 0.8; all gauge lists, thresholds and looks live in
  `config/ui/webcast_gauges.toml`, read by `GaugeConfig`).

## Code
- `mission_timeline.hpp` (`sim::MissionTimeline`), `predicted_events.cpp` (`predicted_mission_events`,
  `booster_burn_time_s`), `MissionEventObserver`, `FlightSession::schedule_liftoff` and the pad hold (see
  `docs/subsystems/vehicle-data.md`), guidance forecasts `AscentGuidance::cutoff_epoch`,
  `BoosterReturnGuidance::forecast`, `EntryGuidance::gate_epoch`, `ShipLandingGuidance::catch_epoch`.

## Standing decisions
- Catch moment = the pins' first touch (user ruling 2026-10-22).
- Predict only what the onboard computer has determined, no nominal profiles (user 2026-10-22).
- Lunar orbit speed is shown about the Moon (`speed_inertial_mps` about the reference body).

## Validation
- `[timeline]` (`test_mission_timeline.cpp`): T-0 rules; the recorded ascent, booster return, flip and catch, cold
  staging, lunar-return loft, undocking; scheduled liftoff with pad hold (one 4000-tick call coasts 5.96 h, altitude
  within 0.1 m of the pad); waiting ISS launch; countdown of a waiting ascent; predicted deorbit events before and
  after the re-plan (3 h before the burn); the Dragon's RCS burn (Ignition / Cutoff, predicted ignition within 1 s);
  predicted vs recorded undocking; the onboard predictions of MECO / separation / SECO (starship_leo), MECO
  (Falcon 9), the booster's boostback cutoff, landing burn and catch, the ship's landing burn and catch (tolerances
  from the measurements above), the capsule's entry interface serial through the trunk jettison.

## Diagnostics
- Hidden probe `"Return prediction probe"` (STARSIM_VERBOSE=1, STARSIM_PROBE_CAPSULE for the Crew Dragon): a ship /
  capsule return from orbit with the predictions' errors; `STARSIM_DEBUG_PREDICTIONS` prints every sample.
