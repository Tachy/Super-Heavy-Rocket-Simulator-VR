#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include "starsim/config/drone_ship_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/ascent_guidance.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::sim {

// How the first stage comes back (ascent trajectory and staging, Task 4; chosen by plan_launch).
enum class RecoveryKind { TowerCatch, LandingZone, DroneShip };

struct RecoveryChoice {
    RecoveryKind kind = RecoveryKind::TowerCatch;
    std::string zone_or_ship_id;  // TowerCatch: the launch site; LandingZone: its site id; DroneShip: drone_ships.toml
    double latitude_rad = 0.0;    // where the stage comes down (the tower's pad, the zone, the ship's station)
    double longitude_rad = 0.0;
    double heading_rad = 0.0;     // DroneShip: the deck's long axis, clockwise from north (least_deck_motion_heading_rad:
                                  // along the ground track unless a moving sea makes another heading tilt less)
};

// [D] The drone ship waits at the first stage's impact predicted this long after its separation (the derivation of
// config/missions/falcon9_asds.toml: the staging sequence's push is over, the coast's RCS trim moves the point ~0.1 km).
inline constexpr double drone_ship_prediction_after_separation_s = 10.0;

// [E] Deck headings tried by plan_launch's sea-state check before the drone-ship trials (0..180 deg: the deck motion
// statistics are symmetric under a half turn). The tilt varies smoothly with the heading (cos / sin of the wave angle).
inline constexpr double sea_check_heading_step_rad = 5.0 * constants::deg_to_rad;
// A grid heading replaces the ground track only when its significant tilt is smaller by more than this.
inline constexpr double sea_check_tilt_tie_rad = 1e-9;

// The landing zone or drone ship of `recovery` as the session's landing site (none for a tower catch), read from
// `session`'s config directory. A drone ship floats on `session`'s sea at its station (FlightSession::sea_height_m:
// the terrain's fine EGM2008 geoid when set, else the coarse one).
std::optional<LaunchSite> recovery_landing_site(const FlightSession& session, const RecoveryChoice& recovery);

// The onboard computer's sea-state check of a drone-ship landing (sea state and deck motion, Task 4): the deck motion
// statistics (contact::DeckMotion::statistics, 2 sigma, spectral) of `ship` with its long axis along `heading_rad`
// (clockwise from north) in `like`'s sea, against the ship's limits (max_deck_tilt_rad, max_heave_rate_mps). Returns
// the German reason ("Seegang im Landegebiet zu hoch: ..." with each value above its limit and the limit) or nullopt:
// within the limits, a calm sea or none, or the fidelity flag deck_motion off.
std::optional<std::string> sea_state_refusal(const FlightSession& like, const config::DroneShipConfig& ship,
                                             double heading_rad);

// The heading a drone ship holding station turns its deck to (sea state and deck motion, Task 4 fix round 1): in a
// moving sea (`like`'s, fidelity flag deck_motion on) the heading of least significant deck tilt on the grid 0..180 deg
// in sea_check_heading_step_rad (a ship heads into the sea); a tie (within sea_check_tilt_tie_rad) keeps
// `ground_track_heading_rad`. A calm sea, none, or the flag off: `ground_track_heading_rad` unchanged.
double least_deck_motion_heading_rad(const FlightSession& like, const config::DroneShipConfig& ship,
                                     double ground_track_heading_rad);

// A launch from a pad the onboard computer plans before liftoff (ascent trajectory and staging, Task 3).
// Threads: the planner reads `like`'s interpolated ephemeris and orientation, whose caches fill lazily and are not
// thread-safe - run it on the thread that ticks `like` (the simulation thread, inside a posted command), never beside it.
struct AscentProfileRequest {
    std::string vehicle_id;
    std::string site_id;
    double heading_rad = 0.0;  // body +y on the pad
    time::Epoch liftoff;
    gnc::AscentTarget target;
    std::optional<config::MissionPayload> payload;  // a payload vehicle on top of the stack (exclusive with payload_kg)
    double payload_kg = 0.0;
    bool fairing = true;
    RecoveryChoice recovery;  // the first stage's return (plan_launch chooses it)
    // The stack as placed instead of vehicle_id with payload_kg / fairing / payload (a study's changed vehicle, a free
    // launch of a loaded configuration): flown as given and never memoised (its id names the file it was changed from).
    std::optional<config::VehicleConfig> stack;
};

// The first stage's impact predicted without a boostback, drone_ship_prediction_after_separation_s after its
// separation (drag and the entry burn, no steering: BoosterReturnGuidance::predict_impact).
struct BoosterImpact {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double heading_rad = 0.0;  // the ground track at the impact, clockwise from north
};

// The first-stage pitch program the planner chose: the kick that leaves the upper stage the most propellant at SECO
// with the booster's return planned at MECO (meco_propellant_kg) and the target orbit reached.
struct AscentProfile {
    bool feasible = false;
    std::string reason;  // German, shown to the player when not feasible
    double kick_angle_rad = 0.0;
    double upper_propellant_at_seco_kg = 0.0;
    int trial_flights = 0;  // flown for this answer (0: from the memo)
    // plan_launch: the target flown (the inclination raised to the site's latitude when below it) and the recovery
    RecoveryChoice recovery;
    gnc::AscentTarget target;
    // A drone-ship request: the stage's impact on the chosen kick's trial (the ship's station)
    std::optional<BoosterImpact> booster_impact;
};

// [E] The kick range searched: wide around the optima (Starship to 200 km / 26.5 deg: 14.9 deg; Falcon 9 RTLS to the ISS
// plane: 11.6 deg); the Starship sweep (test "[ascent_planner]") reaches the orbit only between 7 and 21 deg.
inline constexpr double min_kick_angle_rad = 4.0 * constants::deg_to_rad;
inline constexpr double max_kick_angle_rad = 25.0 * constants::deg_to_rad;
// [E] The search stops when the optimum is known to this (spec: ~0.2 deg of the true optimum).
inline constexpr double kick_tolerance_rad = 0.1 * constants::deg_to_rad;

// One trial ascent: a fresh session with `like`'s ephemeris, Earth orientation, gravity model and config dir, the
// request's stack on its pad, booster return on (MECO at the return's planned propellant), the ascent autopilot with
// the kick `kick_angle_rad`, flown at the flight's tick to the upper stage's cutoff (the booster's return itself is not
// flown: the MECO plan settles it). Returns the propellant (oxidizer + fuel) left in the upper stage once the engines'
// tail-off has ended; NaN when the flight is infeasible (no cutoff before the stage runs dry, the orbit off the target
// by more than 5 km as the guidance aims it - a circular target on the radius flown over the next revolution, an
// elliptic one on the osculating apsides at cutoff -, the booster's return not planned or its MECO plan unusable: not finite, or a boostback without its Lambert arc;
// a drone-ship request whose stage impact cannot be predicted).
double fly_ascent_trial(const FlightSession& like, const AscentProfileRequest& request, double kick_angle_rad);

// The kick of the request's ascent: searched on trial flights (fly_ascent_trial) in [min_kick_angle_rad,
// max_kick_angle_rad] to kick_tolerance_rad - Newton steps on a parabola through the best three trials, golden-section
// steps as the safeguard. Memoised per process by the request (its liftoff epoch excluded: the optimum does not depend
// on it), so a repeated mission start flies no trial. Test processes also share plans through a disk cache
// (ascent_plan_cache.hpp, keyed with the liftoff epoch; only when STARSIM_PLAN_CACHE_DIR is set).
AscentProfile plan_ascent_profile(const FlightSession& like, const AscentProfileRequest& request);

// The launch as the onboard computer plans it (spec goal 2): a target inclination below the site's latitude is raised to
// it (due east, the lowest reachable); the recovery is chosen, then the kick (plan_ascent_profile):
//  - a stage caught by a tower ("tower_catch", Super Heavy): the site's tower; none, or no plan with the catch: refused;
//  - a stage on legs ("leg_landing", Falcon 9): the site's landing zone (RTLS) when the plan with the RTLS return is
//    feasible, else the drone ship at the stage's predicted no-boostback impact (booster_impact), heading along the
//    ground track (least_deck_motion_heading_rad: in a moving sea the heading of least deck tilt); neither: refused.
//    The drone ship is checked against the sea (sea_state_refusal) before its trials, with the least-tilt heading at
//    the provisional station, and again at the chosen station with the deck heading of the recovery; a refusal
//    refuses the launch.
// A plan is feasible only when every trial checked the return at MECO for real: a finite propellant plan and, with a
// boostback, its Lambert arc found (BoosterReturnGuidance::boostback_delta_v_mps) - a failed arc counts no boostback.
// What the RTLS / catch reserve holds is the MECO plan's: the stage's landing_propellant_kg plus run-down, flip and the
// boostback by the rocket equation on the Lambert arc with boostback_loss_fraction on top - not a flown return. The Lambert-arc
// check itself is weak (an arc is found for almost any MECO state); a return that does not fit shows as an early MECO
// (the plan keeps more propellant) and the trial then fails the orbit check.
// request.recovery is ignored (chosen here); the result carries the recovery and the target flown.
AscentProfile plan_launch(const FlightSession& like, AscentProfileRequest request);

// Every launch from a pad (Task 5: missions, mission chains, the free launch): `session`'s stack, placed on the request's
// site and clamped, is planned (plan_launch with `session` as `like`) and, when feasible, flown - the chosen landing
// zone or drone ship set, the booster's return on, the ascent autopilot engaged with the target flown and the planned
// kick. Refused: the autopilot is left as it is (off on a fresh pad), a scheduled liftoff called off. Returns the plan.
AscentProfile engage_planned_ascent(FlightSession& session, const AscentProfileRequest& request);

// The ascent autopilot as the onboard computer engages it. On a stack clamped on a configured launch site
// (FlightSession::launch_from_pad with a LaunchSite::id) the launch is planned first (engage_planned_ascent on the pad
// stack: the recovery, the reachable target and the kick, on trial flights on this thread - the countdown holds
// meanwhile) and flown with the booster's return; a refused plan leaves the autopilot off, calls off a scheduled liftoff
// and returns the German reason. Not clamped (in flight): engaged at once with the default `PitchProgram`, not planned
// (the guidance takes over from the state now; a staging sequence under way keeps the state Staging). Returns ""
// when engaged.
std::string engage_pad_ascent(FlightSession& session, const gnc::AscentTarget& target);

}  // namespace starsim::sim
