#pragma once

#include <filesystem>
#include "starsim/config/drone_ship_config.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/splashdown_zone_config.hpp"
#include "starsim/contact/landing_surface.hpp"
#include "starsim/sim/catch_tower_sim.hpp"
#include "starsim/sim/leg_landing_sim.hpp"
#include "starsim/sim/lunar_site.hpp"
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "starsim/aero/radiative_heating.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/environment/gravity.hpp"
#include "starsim/environment/reference_body.hpp"
#include "starsim/gnc/ascent_guidance.hpp"
#include "starsim/gnc/attitude_controller.hpp"
#include "starsim/gnc/booster_return.hpp"
#include "starsim/gnc/burn_node.hpp"
#include "starsim/gnc/deorbit_planner.hpp"
#include "starsim/gnc/entry_guidance.hpp"
#include "starsim/gnc/lunar_ascent_guidance.hpp"
#include "starsim/gnc/lunar_landing_guidance.hpp"
#include "starsim/gnc/orbital_attitude.hpp"
#include "starsim/gnc/parachute_sequencer.hpp"
#include "starsim/gnc/ship_landing.hpp"
#include "starsim/gnc/sun_hold.hpp"
#include "starsim/mission/lunar_descent.hpp"
#include "starsim/mission/lunar_orbit_transfer.hpp"
#include "starsim/mission/lunar_outbound.hpp"
#include "starsim/mission/nrho_reference.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace starsim::sim {

// Normalised pilot input (HOTAS / keyboard): stick -1..1, throttle 0..1.
// pitch > 0: nose towards the launch heading (stick forward); yaw > 0: nose to the right of it.
struct PilotInput {
    double pitch = 0.0;
    double yaw = 0.0;
    double roll = 0.0;
    double throttle = 1.0;
    // RCS translation, body-axis fractions -1..1 (manual flight; zero keeps every existing flight unchanged).
    math::Vec3 translation;
};

enum class ControlMode {
    RateCommand,  // stick commands body rates; the controller drives the gimbals (stability augmentation)
    DirectGimbal  // stick commands gimbal angles directly
};

// Return: a ship after its deorbit burn (coast to the entry interface, entry, flip, catch).
// PayloadSeparation: after the upper stage's cutoff, waiting to release a payload vehicle (d2).
// Lunar: a lunar flight between its burns (FlightSession::plan_lunar_outbound / plan_lunar_orbit_transfer).
enum class AutopilotState { Off, PreLaunch, Booster, UpperStage, PayloadSeparation, Orbit, Burn, Return, Lunar };

const char* to_string(AutopilotState state);

// Steps of a capsule's return from orbit (FlightSession::plan_capsule_return).
// Lost: the capsule impacted or broke up before resting on the water (autopilot off).
enum class CapsuleReturnStep {
    None, DeorbitBurn, TrunkJettison, NoseConeClose, Coast, Entry, Gate, Drogues, Mains, SplashedDown, Lost
};
const char* to_string(CapsuleReturnStep step);

// Steps of a lunar flight (plan_lunar_outbound / plan_lunar_orbit_transfer). Tli, MidCourse, PoweredFlyby, Insertion and
// StationKeeping: that burn is under way (the autopilot state is Burn meanwhile); the others wait for their next event.
// PilotCorrection: the crew flies a mid-course correction (lunar_pilot_cue).
// Descent from the LLO (plan_lunar_landing, l4): DescentWait -> PlaneTrim (if any) -> DescentWait -> Doi -> DescentCoast
// -> PoweredDescent -> Landed (via SurfaceStay when a stay is requested); PlaneTrim and Doi: that burn is under way.
// Ascent (plan_lunar_ascent, l4): (SurfaceStay until the liftoff ->) Ascent -> AscentCoast -> Circularize (the burn) ->
// InLlo; AbortAscent: an abort to orbit from PoweredDescent, then AscentCoast as the ascent.
enum class LunarStep {
    None, Departure, Tli, Coast, MidCourse, PilotCorrection, PoweredFlyby, Insertion, Trim, InLlo, InNrho, StationKeeping,
    TransferOut, TransferIn, DescentWait, PlaneTrim, Doi, DescentCoast, PoweredDescent, Landed, SurfaceStay, Ascent,
    AscentCoast, Circularize, AbortAscent, Lost
};
const char* to_string(LunarStep step);

// How a powered lunar descent ended (lunar l4).
struct LunarTouchdown {
    LegTouchdown outcome = LegTouchdown::None;
    double offset_m = 0.0;     // horizontal distance of the footprint centre from the touchdown point (judgement)
    double sink_mps = 0.0;     // at the first leg contact, relative to the Moon
    double lateral_mps = 0.0;  // at the first leg contact
};

// A lunar landing from the LLO (plan_lunar_landing, l4).
struct LunarLandingSetup {
    mission::LunarDescentParameters descent;
    // The braking range from the mass at planning (l4 mass correction); empty: descent.braking_range_m as set.
    std::optional<mission::BrakingRangeParameters> braking_range = mission::BrakingRangeParameters{};
    gnc::LunarLandingParameters guidance = gnc::hls_landing_parameters();
    double surface_stay_s = 0.0;          // [E] mission parameter (spec): resting after "gelandet"
    double descent_delta_v_mps = 2100.0;  // [D] budget: DOI + braking + approach (C2/C3 flights: 18 + 2068 m/s from 566 t PDI, 2062 from 500 t)
    double braking_delta_v_mps = 1150.0;  // [D] of it on the vacuum Raptors (C2 flight: 566 -> 417.5 t at 380 s = 1134 m/s)
    double ascent_delta_v_mps = 1900.0;   // [E] budget: ascent + circularisation (spec ~1.9 km/s, Apollo LM ~1.85)
    double reserve_kg = 10000.0;          // [E] residuals and unusable propellant
    std::optional<gnc::LunarAscentParameters> ascent;  // lift off at the stay's end (none: stay Landed)
};

// The whole lunar landing mission as one autopilot sequence (mission 1, l4): after the outbound flight the vehicle
// (NRHO variant: first transfers NRHO -> LLO) plans the landing, stays, ascends into the LLO and (NRHO variant)
// transfers back to the NRHO. Each leg is planned from the real state when the previous one ends ("planning nodes").
struct LunarMissionSequence {
    LunarLandingSetup landing;  // with `ascent` set
    bool via_nrho = false;
    std::shared_ptr<const mission::NrhoReference> nrho;  // NRHO variant
    mission::LloTarget llo;
};

// Flight director cue of a pilot-flown mid-course correction (lunar l3, Step 3b; displayed with l6 / M7).
struct PilotBurnCue {
    gnc::BurnNode planned;
    math::Vec3 remaining_delta_v_gcrf_mps;  // planned minus realised (current velocity - gravity-only coast of the call)
    double attitude_error_rad = 0.0;        // body +x against the remaining delta-v
    double window_end_s = 0.0;              // seconds until the window closes
};

// A burn of a lunar flight as commanded (impulsive node; PilotCorrection: the plan handed to the crew).
struct LunarBurnRecord {
    LunarStep step = LunarStep::None;
    gnc::BurnNode node;
    double mass_kg = 0.0;  // at the command
};

struct AutopilotStatus {
    AutopilotState state = AutopilotState::Off;
    gnc::GuidanceCommand guidance;
    gnc::BurnExecutor::Status burn;  // valid in state Burn
};

struct LaunchSite {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    std::optional<config::TowerConfig> tower;  // catch tower (stacks then stand on its launch mount)
    std::optional<config::SeaConfig> sea;      // coastline (aborted landings)
    double pad_radius_m = 0.0;                 // landing zone: concrete pad radius
    std::optional<contact::LandingSurface::Deck> deck;  // a drone ship: its deck (height_m: the deck), the sea around it
};

LaunchSite launch_site_from_config(const config::LaunchSiteConfig& site);
// A drone ship holding station at (latitude, longitude) as a landing zone: its deck at sea height + freeboard, the long
// axis along `heading_rad` (clockwise from north). `sea` marks the water beside the deck's side edge: the landing
// guidance's abort aims there.
LaunchSite drone_ship_site(const config::DroneShipConfig& ship, double latitude_rad, double longitude_rad,
                           double sea_height_m, double heading_rad);

enum class LandingOutcome;

// Release of a mission's payload vehicle (config::stack_payload) from the upper stage.
struct PayloadSeparation {
    std::string first_stage_id;  // the payload's lowest stage (e.g. "trunk")
    double delay_s = 30.0;       // after the upper stage's cutoff
    double dv_mps = 0.3;         // spring push (relative speed)
    double nose_cone_open_s = 600.0;  // after the separation; < 0: stays closed
};

// All flying bodies of one mission (the stack, and after staging booster and ship) on a common
// fixed tick. The pilot controls one body; separated bodies coast with their engines off.
// Not thread-safe (SPICE): use from the simulation thread only.
class FlightSession {
public:
    static constexpr double tick_s = 0.005;  // 200 Hz
    static constexpr double max_coast_s = 600.0;
    static constexpr double burn_preparation_s = 120.0;  // ticks resume this long before an ignition
    static constexpr double small_burn_margin_s = 60.0;     // [E] trims / station keeping: planned after the preparation
    // [E] TLI centre after 120 s preparation + half a ~7 min burn; margin for the finite-TLI re-solve. A window < 30 min
    // away waits one parking revolution.
    static constexpr double min_departure_lead_s = 1800.0;
    // [E] a returning ship gets its entry guidance this long before the entry interface (turn to the entry attitude
    // on RCS: ~1.5 min for the 125 deg from the burn attitude)
    static constexpr double entry_lead_s = 300.0;
    static constexpr double max_pitch_yaw_rate_radps = 5.0 * 3.14159265358979 / 180.0;
    static constexpr double max_roll_rate_radps = 10.0 * 3.14159265358979 / 180.0;

    // `gravity_config`: fidelity flags of the gravity model (default: Earth + J2, Moon, Sun).
    // `moon_orientation` (MOON_ME): the Moon becomes a reference body (lunar l1: ground, air, telemetry and LVLH relative
    // to the nearest body); without it everything is relative to the Earth.
    FlightSession(const ephemeris::Ephemeris& ephemeris, const frames::BodyOrientation& earth,
                  std::filesystem::path config_dir, environment::PointMassGravityConfig gravity_config = {},
                  const frames::BodyOrientation* moon_orientation = nullptr);

    // Places a vehicle on a pad (engines off, clamped). `heading_rad`: direction of body +y.
    void launch_from_pad(const std::string& vehicle_id, const LaunchSite& site, double heading_rad,
                         time::Epoch epoch);
    // The same with a loaded (possibly modified, e.g. the mission's payload) vehicle configuration.
    void launch_from_pad(const config::VehicleConfig& config, const LaunchSite& site, double heading_rad,
                         time::Epoch epoch);
    // Places a vehicle (stages from `first_stage` upwards) on a given orbit, nose along the velocity.
    // ... or from a loaded configuration (all its stages; a mission's propellant loads).
    void place_in_orbit(const config::VehicleConfig& config, const math::StateVector& state_gcrf, time::Epoch epoch);
    void place_in_orbit(const std::string& vehicle_id, const math::StateVector& state_gcrf, time::Epoch epoch,
                        size_t first_stage = 0);
    // Fidelity flag: catch towers of launch sites (default on). Takes effect at the next set_launch_site().
    void set_towers(bool enabled) { towers_enabled_ = enabled; }
    // Site of the mission (tower, landing target). launch_from_pad() sets it too.
    void set_launch_site(const LaunchSite& site);
    const LaunchSite& launch_site() const { return launch_site_; }
    // Landing zone of the mission: first stages that land on legs ("leg_landing") return to it, and the legs land on
    // its ground plane (with its shoreline). Set after the launch site (set_launch_site resets the leg landing).
    void set_landing_zone(const LaunchSite& zone);
    const std::optional<LaunchSite>& landing_zone() const { return landing_zone_; }
    // Recovery area of a capsule at sea: its entry guidance aims the drogue gate over the zone's centre.
    void set_splashdown_zone(const config::SplashdownZoneConfig& zone) { splashdown_zone_ = zone; }
    const std::optional<config::SplashdownZoneConfig>& splashdown_zone() const { return splashdown_zone_; }
    // Entry target of a capsule: the zone's centre at sea level (WGS84 height 0); throws std::logic_error without a zone.
    gnc::EntryTarget capsule_entry_target() const;
    // Places a vehicle with a free state (scenarios, tests); the body becomes the controlled one.
    void place_body(const config::VehicleConfig& config, const vehicle::RigidBodyState& state, time::Epoch epoch);
    // Landing site of a returning `booster`: catch target when the site has a tower and the booster has pins; a stage
    // with legs ("leg_landing") lands on the landing zone's pad; coastline when configured.
    gnc::LandingSite return_site(const vehicle::Vehicle& booster) const;
    // The lowest stage of `vehicle` gets return guidance after separation (booster return on, and either caught by a
    // tower at this site or landing on its legs on the landing zone).
    bool returns_after_staging(const vehicle::Vehicle& vehicle) const;
    // Return parameters of a first stage: the guidance's (set_booster_return) with the stage's return profile for this
    // mission's landing target (drone ship or not).
    gnc::BoosterReturnParameters return_parameters(const config::StageConfig& stage) const;
    const vehicle::FlightEnvironment& environment() const { return env_; }
    // Reference body of body `index` now (lunar l1).
    ephemeris::Body reference_body(size_t index) const;
    const CatchTowerSim* tower() const { return tower_.get(); }
    CatchTowerSim* tower() { return tower_.get(); }
    bool active() const { return !bodies_.empty(); }
    time::Epoch epoch() const { return epoch_; }

    // Integrates up to `target` (at most `max_ticks` steps); returns the epoch reached. Fixed 200 Hz ticks
    // while anything is commanded, adaptive coast segments (<= max_coast_s) otherwise.
    time::Epoch advance_to(time::Epoch target, int max_ticks = 4000);

    void set_input(const PilotInput& input) { input_ = input; }
    void set_control_mode(ControlMode mode) { mode_ = mode; }
    ControlMode control_mode() const { return mode_; }

    // Ascent autopilot: ignition, liftoff at full thrust, booster profile, MECO at the landing reserve,
    // hot staging, closed-loop upper-stage guidance to the target orbit, cutoff. Pilot input is ignored
    // while engaged.
    void engage_ascent_autopilot(const gnc::AscentTarget& target, const gnc::PitchProgram& program = {});
    // Also ends an attitude mode and a capsule return (d2 minor: a mode set before outlived the autopilot).
    void disengage_autopilot() {
        autopilot_.state = AutopilotState::Off;
        attitude_mode_ = {};
        capsule_return_step_ = CapsuleReturnStep::None;
        lunar_step_ = LunarStep::None;
        lunar_pilot_.reset();
        lunar_landing_.reset();  // the pilot flies: no descent guidance command afterwards
        lunar_ascent_.reset();
        lunar_ascent_parameters_.reset();
        lunar_descent_plan_.reset();
        lunar_landing_setup_.reset();
        lunar_mission_.reset();
        lunar_mission_leg_ = LunarMissionLeg::None;
    }
    // Return of the booster to the launch pad after staging (flip, boostback, landing burn). Takes effect
    // at the next staging; the booster is then guided independently of the controlled body.
    void set_booster_return(bool enabled, gnc::BoosterReturnParameters parameters = {});
    // Scenarios: a returning booster placed with a free state, guided by its return guidance from `start_phase`
    // (it becomes the controlled body; pilot input is ignored while it has return guidance).
    void place_returning_booster(const config::VehicleConfig& config, const vehicle::RigidBodyState& state,
                                 time::Epoch epoch, gnc::BoosterReturnParameters parameters, gnc::ReturnPhase start_phase);
    // Scenarios: a ship (single stage with flaps and a heat shield) placed with a free state flies its entry guidance
    // to the gate above the launch site's pad (b3 adds the flip and the catch). It becomes the controlled body; pilot
    // input is ignored while it has entry guidance.
    void place_entering_ship(const config::VehicleConfig& config, const vehicle::RigidBodyState& state, time::Epoch epoch,
                             gnc::EntryGuidanceParameters parameters = {});
    const gnc::EntryGuidance* entry_guidance(size_t index) const { return bodies_.at(index).entry_guidance.get(); }
    // Parachute sequence of body `index` (its lowest stage has [[stage.parachute]] sets: a capsule alone), nullptr if
    // none. The Dracos stay off once a set is out (user 2026-10-14: no steering under the parachutes).
    const gnc::ParachuteSequencer* parachute_sequencer(size_t index) const { return bodies_.at(index).parachutes.get(); }
    // Horizontal distance (m) of body `index` from the splashdown zone's centre; throws std::logic_error without a zone.
    double splashdown_miss_m(size_t index) const;
    // Scenarios: a capsule (a stage with [stage.aero] entry_profile = "capsule") placed with a free state flies its entry
    // guidance on its RCS (gnc::capsule_entry_torque_fraction) to the drogue gate over the splashdown zone, then holds
    // the lift up (d4 adds the parachutes). It becomes the controlled body.
    void place_entering_capsule(const config::VehicleConfig& config, const vehicle::RigidBodyState& state,
                                time::Epoch epoch,
                                gnc::EntryGuidanceParameters parameters = gnc::EntryGuidanceParameters::capsule());
    // Scenarios: a ship placed at the gate (belly down) flips and lands on its landing guidance (catch at this site's
    // tower). An entering ship gets the same guidance when its entry guidance reaches the gate.
    void place_landing_ship(const config::VehicleConfig& config, const vehicle::RigidBodyState& state, time::Epoch epoch,
                            gnc::ShipLandingParameters parameters = gnc::ship_landing_parameters());
    const gnc::ShipLandingGuidance* ship_landing(size_t index) const { return bodies_.at(index).ship_landing.get(); }
    // Return of the controlled ship from orbit to this site's tower (b4): the onboard computer plans the deorbit burn
    // (gnc::plan_deorbit) and flies it on the vehicle's deorbit engine set; the ship then coasts, gets its entry guidance
    // `entry_lead_s` before the entry interface and is caught by the flip and landing guidance. False if the plan is
    // refused (reason in ship_return_plan()) or the burn cannot be flown.
    bool plan_ship_return(const gnc::DeorbitParameters& parameters = {});
    const gnc::DeorbitPlan* ship_return_plan() const { return deorbit_plan_ ? &*deorbit_plan_ : nullptr; }
    // Return of the controlled capsule (a top stage with entry_profile = "capsule", e.g. Crew Dragon on its trunk) to the
    // splashdown zone (d3): the onboard computer plans the deorbit burn (gnc::plan_deorbit, finite burn on the RCS along
    // +x, predicted with the capsule's mass after the trunk jettison) and flies it; the trunk is jettisoned
    // trunk_jettison_delay_s after the burn, the nose cone closes nose_cone_close_delay_s later, the capsule turns to its
    // trim attitude and gets its entry guidance entry_lead_s before the interface, which takes it to the drogue gate.
    // False if refused (reason in ship_return_plan()). Order of events [S]: SpaceX since the return to West Coast
    // recovery - deorbit burn, then trunk jettison (the trunk falls uprange of the capsule).
    bool plan_capsule_return(gnc::DeorbitParameters parameters = capsule_deorbit_parameters());
    static gnc::DeorbitParameters capsule_deorbit_parameters();
    CapsuleReturnStep capsule_return_step() const { return capsule_return_step_; }
    static constexpr double trunk_jettison_delay_s = 120.0;   // [E] after the deorbit burn
    static constexpr double nose_cone_close_delay_s = 120.0;  // [E] after the trunk jettison
    static constexpr double trunk_separation_dv_mps = 0.3;    // [E] spring push (as the payload separation)
    static constexpr double capsule_return_reserve_kg = 50.0; // [E] RCS propellant kept for the entry (d3: ~31 kg used)
    // Lunar outbound flight of the controlled vehicle (lunar l3): from the Earth parking orbit the onboard computer plans
    // TLI, mid-course corrections, (NRHO) the powered swing-by and the insertion, and flies them on the autopilot
    // (state Lunar between the burns); LLO apsis trims follow the LOI, NRHO station keeping at every apolune. False if a
    // lunar flight or another autopilot phase is under way, without the Moon orientation or vacuum engines, or when the
    // plan is refused (reason in lunar_outbound_plan() and lunar_status()).
    bool plan_lunar_outbound(const mission::LunarOutboundParameters& parameters);
    // From the LLO (step InLlo) or the NRHO (InNrho) to the other one: two burns, the second re-planned on arrival.
    bool plan_lunar_orbit_transfer(mission::LunarOrbitKind target, std::shared_ptr<const mission::NrhoReference> nrho,
                                   mission::LloTarget llo = {});
    LunarStep lunar_step() const { return lunar_step_; }
    const mission::OutboundPlan* lunar_outbound_plan() const { return lunar_plan_ ? &*lunar_plan_ : nullptr; }
    const std::string& lunar_status() const { return lunar_status_; }  // German status line
    const std::vector<LunarBurnRecord>& lunar_burns() const { return lunar_burns_; }  // of this lunar flight
    // Pilot-flown mid-course correction (LunarOutboundParameters::pilot_corrections): the cue while the step is
    // PilotCorrection, and the pilot's report that it is flown (the next correction is re-planned from the real state).
    std::optional<PilotBurnCue> lunar_pilot_cue() const;
    void complete_pilot_correction();
    // Powered descent of the controlled body from now (scenarios; the lunar landing step machine calls it at PDI): needs
    // the Moon orientation, a lunar landing site (set_lunar_landing_site) and a Moon-referenced body with legs. False
    // with a German lunar_status() otherwise.
    bool begin_powered_descent(const mission::TouchdownPoint& point, double approach_azimuth_rad,
                               gnc::LunarLandingParameters parameters = gnc::hls_landing_parameters());
    // Descent of the controlled vehicle from a closed lunar orbit (autopilot Off / Orbit or lunar step InLlo): plans the
    // site pass, plane trim, DOI and PDI (mission::LunarDescentPlanner on the lunar landing site's terrain), checks the
    // propellant for descent and ascent (mission::lunar_budget) and flies the steps; after "gelandet" the vehicle rests
    // for surface_stay_s (step SurfaceStay, then Landed). False with a German lunar_status() when refused (a descent
    // under way keeps its plan and status).
    bool plan_lunar_landing(const LunarLandingSetup& setup);
    const mission::LunarDescentPlan* lunar_descent_plan() const {
        return lunar_descent_plan_ ? &*lunar_descent_plan_ : nullptr;
    }
    const gnc::LunarLandingGuidance* lunar_landing_guidance() const { return lunar_landing_.get(); }
    const std::optional<LunarTouchdown>& lunar_touchdown() const { return lunar_touchdown_; }
    // Ascent of the controlled vehicle standing landed on the Moon (lunar step Landed / SurfaceStay, or a body the leg
    // simulation judged "gelandet"), `liftoff_delay_s` from now (meanwhile step SurfaceStay): liftoff, PEG to the
    // insertion, coast to the aposelene, circularisation -> InLlo. False with a German lunar_status() (not landed,
    // propellant short for orbit).
    bool plan_lunar_ascent(const gnc::LunarAscentParameters& parameters = {}, double liftoff_delay_s = 0.0);
    const gnc::LunarAscentGuidance* lunar_ascent_guidance() const { return lunar_ascent_.get(); }
    // A closed lunar orbit clear of the highest terrain: both apsides (radii from the Moon's centre) above
    // moon_mean_radius_m + moon_max_terrain_height_m (the ascent's sub-orbital cutoff guard).
    static bool lunar_orbit_clears_terrain(double periselene_m, double aposelene_m);
    // Chains the legs of a lunar landing mission after the flight planned now (start_lunar_landing_mission); empty
    // ends it. A refused leg ends the sequence in its orbit with the planner's reason in lunar_status().
    void set_lunar_mission(std::optional<LunarMissionSequence> sequence);
    // The sequence's last leg reached (status "Mission abgeschlossen: ...").
    bool lunar_mission_complete() const { return lunar_mission_leg_ == LunarMissionLeg::Complete; }
    // The sequence ended early on a refused leg (status "Mission beendet: <reason>").
    bool lunar_mission_ended() const { return lunar_mission_leg_ == LunarMissionLeg::Ended; }
    static constexpr double default_ascent_delta_v_mps = 1900.0;  // [E] without a landing setup (spec ~1.9 km/s)
    static constexpr double default_ascent_reserve_kg = 10000.0;  // [E] as LunarLandingSetup::reserve_kg
    static constexpr double sun_repoint_interval_s = 6.0 * 3600.0;  // [E] coast: Sun pointing re-aimed this often
    static constexpr int max_llo_trims = 4;
    static constexpr double descent_attitude_lead_s = 600.0;  // [E] retrograde hold before PDI (~180 deg at 0.5 deg/s)                          // [E]
    static constexpr double llo_trim_interval_s = 600.0;             // [E] after a trim burn, the next check
    // Entry guidance of a body (nullptr if it has none): its latest command (setpoint for the flight director).
    const gnc::EntryCommand* entry_status(size_t index) const;
    // Interactive real-time play: landing re-plans never hold up the simulation (default: off = deterministic).
    void set_realtime_planning(bool realtime);
    const gnc::BoosterReturnGuidance* return_guidance(size_t index) const { return bodies_.at(index).return_guidance.get(); }
    // Return guidance of a body (nullptr if it has none) and its latest command.
    const gnc::ReturnCommand* return_status(size_t index) const;
    const AutopilotStatus& autopilot() const { return autopilot_; }
    const gnc::AscentGuidance* ascent_guidance() const { return guidance_.get(); }
    double gm_earth_m3ps2() const { return gm_earth_; }
    // The session's models (mission planning outside the session, e.g. place_lunar_landing_mission).
    const environment::PointMassGravity& gravity() const { return gravity_; }
    const ephemeris::Ephemeris& ephemeris() const { return *env_.ephemeris; }
    const frames::BodyOrientation* moon_orientation() const { return moon_orientation_; }  // nullptr: Earth only

    // Manoeuvre execution on the controlled body (RCS orientation, timed ignition, closed-loop cutoff).
    // Burns smaller than the main engines' start + shutdown impulse (they would overshoot), or on a vehicle without main
    // engines, fly on RCS translation along +x when the vehicle has thrusters along it, else along -x (Crew Dragon's
    // nose Dracos), with the thrusters' force and mass flow, steered by the orbit controller; without thrusters along
    // either they are rejected.
    bool execute_burn(const gnc::BurnNode& node);
    // On a given engine set (config group ids of the lowest stage with engines, e.g. the deorbit set): full throttle if
    // its start/shutdown impulse is at most a quarter of the burn, else minimum throttle; false if neither fits. The hull
    // is steered so that the set's thrust axis through the CG (gnc::thrust_axis_body) points along the burn.
    bool execute_burn(const gnc::BurnNode& node, const std::vector<std::string>& groups);
    // Plans and executes; false if not on an elliptic orbit or the burn is too small.
    bool plan_circularization_at_apoapsis();
    // Smallest delta-v the main engines of the controlled body can deliver (start + shutdown transients
    // of the weakest configuration: one engine group at minimum throttle).
    double minimum_burn_dv_mps() const;

    // Engine configuration used for a burn.
    struct BurnConfiguration {
        std::vector<std::string> groups;
        double throttle = 1.0;
        double thrust_n = 0.0;        // vacuum
        double mass_flow_kgps = 0.0;
        double shutdown_s = 0.0;
        double minimum_dv_mps = 0.0;  // impulse of the start + shutdown transients
        bool rcs = false;             // RCS translation (no main engines)
        math::Vec3 thrust_axis_body{1.0, 0.0, 0.0};  // through the CG at the start (asymmetric sets: tilted)
    };
    const BurnConfiguration& burn_configuration() const { return burn_config_; }
    const gnc::BurnExecutor* burn() const { return burn_ ? &*burn_ : nullptr; }

    // Coasting prediction of a body (no thrust): GCRF positions every `step_s`, until impact or `duration_s`.
    std::vector<math::Vec3> predict_trajectory(size_t index, double duration_s, double step_s) const;

    // Failure injection for tests and training.
    void fail_engine(size_t body_index, size_t stage_index, size_t engine_index);
    // Gives a returning body's landing or catch up (pilot's abort): it ditches at sea and the tower keeps its arms open.
    // Before the landing burn the abort is remembered and the burn plans for the sea. False when the guidance cannot
    // abort (not returning, no sea at the site, already down or on the arms): then nothing changes.
    // Lunar l4: the controlled body in step PoweredDescent aborts to orbit (step AbortAscent) when the propellant
    // suffices (LunarAscentGuidance::abort_delta_v_mps against the rocket equation, reserve as the landing setup's),
    // else false with "Abbruch nicht möglich: ..." and the landing continues.
    bool abort_landing(size_t body_index);
    // Attitude mode of the controlled body while no autopilot phase steers it (state Off / Orbit; d2). Any mode but Free
    // keeps the session ticking (pulsed RCS hold); Free lets it coast. Pilot input is ignored while a mode is held.
    void set_attitude_mode(const gnc::AttitudeMode& mode) { attitude_mode_ = mode; }
    const gnc::AttitudeMode& attitude_mode() const { return attitude_mode_; }
    // Scenarios and tests: sets a body's angular rate (body axes).
    void apply_angular_rate(size_t body_index, const math::Vec3& rate_body_radps);
    // Payload vehicle of the mission: released `delay_s` after the upper stage's cutoff with a spring push; it becomes
    // the controlled body, holds its attitude inertially and opens its nose cone after `nose_cone_open_s` (< 0: closed).
    // Takes effect at the next cutoff of the ascent autopilot; a new launch resets it.
    void set_payload_separation(std::optional<PayloadSeparation> separation) { payload_separation_ = std::move(separation); }
    // Releases the payload now (pilot action, or the autopilot): false without a payload separation set or when the
    // controlled body carries no such stage above its lowest.
    bool separate_payload();
    // Crew Dragon's nose cone (thrusters behind it fire only while it is open); no effect without one.
    void set_nose_cone(size_t body_index, bool open);
    // Landing legs of a body: start the deployment (guidance, pilot), or place them in a state (scenarios).
    void deploy_legs(size_t body_index);
    void set_leg_deployment(size_t body_index, double fraction);
    // Landing on legs: the surface the legs land on (default: the site's ground plane with its shoreline; a drone
    // ship's deck replaces it), and how a body's touchdown ended.
    void set_landing_surface(double latitude_rad, double longitude_rad, double height_m, contact::LandingSurface surface);
    // On any reference body (lunar l2; the Moon needs the Moon orientation).
    void set_landing_surface(const LegSite& site, contact::LandingSurface surface);
    // Lunar terrain for ground, leg contact and the tick-near-ground decision at the Moon (lunar l2; needs the Moon
    // orientation). nullptr: the sphere. Drops a lunar landing site (set it again on the new terrain).
    void set_moon_terrain(std::shared_ptr<const environment::LunarTerrain> terrain);
    // Leg landings at a lunar site on its terrain (set_moon_terrain first), tracking bodies within the site's tile.
    void set_lunar_landing_site(const LunarSite& site);
    const LegLandingSim& leg_landing() const { return *legs_; }
    LegTouchdown leg_touchdown(size_t body_index) const { return legs_->touchdown(*bodies_.at(body_index).body); }
    std::vector<double> leg_crushed_m(size_t body_index) const { return legs_->crushed_m(*bodies_.at(body_index).body); }
    // Dispersion injection (tests, training): an instantaneous velocity change of a body, e.g. the residual of a burn
    // (tail-off, misalignment) beyond what the closed-loop cutoff removes.
    void apply_velocity_error(size_t body_index, const math::Vec3& delta_v_gcrf_mps);
    // Truth-model error injection (tests, training): the hull's aerodynamic force of a body scaled by `factor`; the
    // onboard models (impact prediction, landing planner) keep the nominal tables.
    void set_aero_dispersion(size_t body_index, double factor);
    // Likewise thrust and mass flow of every engine of a body (Vehicle::set_thrust_factor); guidance stays nominal.
    void set_thrust_dispersion(size_t body_index, double factor);
    // Truth Earth air density over US76 (lunar l5 campaigns): truth only; the onboard predictor keeps US76.
    void set_air_density_factor(double factor);

    // Pilot actions on the controlled body.
    void ignite();             // start all engine groups of the lowest stage that still has engines
    void cutoff();             // stop all engines
    void release_clamps();
    void stage();              // separate the lowest stage; the upper part becomes the controlled body

    size_t body_count() const { return bodies_.size(); }
    size_t controlled_index() const { return controlled_; }
    const vehicle::FlightBody& body(size_t index) const { return *bodies_.at(index).body; }
    vehicle::FlightTelemetry telemetry(size_t index) const;
    // How the flight of body `index` ended so far (Flying while it is in the air).
    LandingOutcome landing_outcome(size_t index) const;
    // Seconds since liftoff (clamp release); before liftoff: since the vehicle was placed on the pad.
    double mission_time_s() const { return epoch_ - launch_epoch_; }

private:
    struct Flying {
        std::unique_ptr<vehicle::FlightBody> body;
        std::map<std::string, bool> running_groups;  // "<stage>/<group>" -> run
        std::unique_ptr<gnc::BoosterReturnGuidance> return_guidance;  // autonomous return (separated booster)
        gnc::ReturnCommand return_command;
        std::unique_ptr<gnc::EntryGuidance> entry_guidance;  // entering ship (scenario)
        gnc::EntryCommand entry_command;
        std::unique_ptr<gnc::ShipLandingGuidance> ship_landing;  // ship from the gate: flip and catch (its command:
                                                                 // return_command)
        std::unique_ptr<gnc::ParachuteSequencer> parachutes;     // lowest stage with parachute sets (capsule)
    };

    vehicle::VehicleCommand command_for(size_t index, double ambient_pressure_pa) const;
    void run_autopilot();
    void run_return_guidance(Flying& flying);
    void run_ship_landing(Flying& flying);
    void run_parachutes(Flying& flying);
    void apply_return_command(const Flying& flying, const gnc::AttitudeController& controller,
                              double ambient_pressure_pa, vehicle::VehicleCommand& command) const;
    double coast_allowance_s() const;  // 0 if fixed ticks are needed now
    void start_entry_guidance(Flying& flying, const gnc::EntryGuidanceParameters& parameters);
    time::Epoch entry_start_epoch() const;  // returning ship: entry interface (coasting from now) - entry_lead_s
    std::vector<vehicle::FlightBody*> body_pointers() const;

    std::filesystem::path config_dir_;
    const frames::BodyOrientation& earth_;
    environment::PointMassGravity gravity_;
    environment::Us76Atmosphere atmosphere_;
    std::optional<environment::ReferenceBody> moon_;  // lunar l1: with a Moon orientation
    std::shared_ptr<const environment::LunarTerrain> moon_terrain_;  // lunar l2
    aero::RadiativeHeatingTable radiative_heating_;  // lunar l5: Earth air radiation (config/aero)
    vehicle::FlightEnvironment env_;
    gnc::AttitudeController controller_;
    // Returning booster: faster attitude loop for the landing-burn divert (gimbal limits unchanged).
    gnc::AttitudeController return_controller_{gnc::AttitudeController::Gains{1.5, 0.9, true}};
    // Its coast phase (turning engines first on the RCS, d1): slews limited to 2 deg/s [E] - the RCS supply is finite
    // (Falcon 9 nitrogen); a full-authority turn spent it (and the momentum the air adds at 70 km) before the entry burn.
    gnc::AttitudeController return_coast_controller_{
        gnc::AttitudeController::Gains{1.5, 0.9, true, false, 0.0, 2.0 * constants::deg_to_rad}};
    // Entering ship: flaps + RCS, slow (the flaps move at ~20 deg/s: 0.8 rad/s limit-cycles in roll, b1). [E] actuator
    // delay 0.5 s (the flaps' slew): without it the belly flop's nose turn (5 deg/s) set off a roll limit cycle and the
    // ship departed (b4 campaign).
    gnc::AttitudeController entry_controller_{gnc::AttitudeController::Gains{0.5, 0.9, false, true, 0.5}};
    // Ship flip and landing burn: gimbals with flaps and RCS, faster than in the entry [E].
    // [E] actuator delay: the gimbals slew 30 deg (stop to stop) at 20 deg/s, half of that on average
    gnc::AttitudeController ship_landing_controller_{gnc::AttitudeController::Gains{1.5, 0.9, false, true, 0.75}};
    // Orbit attitude modes on pulsed RCS: slow (1 deg/s slews on every axis [E]: the RCS supply is finite; an unlimited
    // roll of the capsule spent 3.5 kg on a 180 deg turn), with a hold deadband [E] 0.5 deg / 0.05 deg/s so the hold
    // settles into a slow limit cycle instead of chattering (d2).
    gnc::AttitudeController orbit_controller_{gnc::AttitudeController::Gains{
        0.3, 0.9, false, false, 0.0, 1.0 * constants::deg_to_rad, 0.5 * constants::deg_to_rad,
        0.05 * constants::deg_to_rad, 1.0 * constants::deg_to_rad}};
    // Lunar coast (attitude modes in the state Lunar): slower slews (0.5 deg/s [E]) and a tight rate deadband
    // (0.002 deg/s [E]) - the vehicle then coasts freely for hours, and the orbit hold's 0.05 deg/s residual turned
    // the HLS by ~1000 deg between two Sun re-points (0.4-0.6 t of cold gas each).
    gnc::AttitudeController lunar_coast_controller_{gnc::AttitudeController::Gains{
        0.3, 0.9, false, false, 0.0, 0.5 * constants::deg_to_rad, 0.5 * constants::deg_to_rad,
        0.002 * constants::deg_to_rad, 0.5 * constants::deg_to_rad}};
    gnc::AttitudeMode attitude_mode_;
    std::optional<PayloadSeparation> payload_separation_;
    time::Epoch separation_epoch_;            // state PayloadSeparation: release at
    std::optional<time::Epoch> nose_cone_epoch_;  // the released payload opens its nose cone at
    size_t nose_cone_body_ = 0;
    bool attitude_mode_active() const;  // a mode is held now (not Free, no autopilot phase steering)
    std::vector<Flying> bodies_;
    size_t controlled_ = 0;
    PilotInput input_;
    ControlMode mode_ = ControlMode::RateCommand;
    time::Epoch epoch_;
    time::Epoch launch_epoch_;
    LaunchSite launch_site_;
    std::optional<LaunchSite> landing_zone_;
    std::optional<config::SplashdownZoneConfig> splashdown_zone_;
    bool towers_enabled_ = true;
    bool realtime_planning_ = false;
    std::unique_ptr<CatchTowerSim> tower_;
    std::unique_ptr<LegLandingSim> legs_;  // always present (site ground plane by default)
    void reset_leg_landing();              // new bodies: no stale tracks
    void reset_mission_state();            // new flight: autopilot off, no ascent guidance, burn or ship return
    std::optional<gnc::BoosterReturnParameters> booster_return_;
    double gm_earth_;
    AutopilotStatus autopilot_;
    std::unique_ptr<gnc::AscentGuidance> guidance_;
    std::optional<gnc::BurnExecutor> burn_;
    BurnConfiguration burn_config_;
    math::Vec3 last_rcs_acceleration_;  // RCS translation of the controlled body in the last tick (GCRF)
    std::optional<gnc::DeorbitPlan> deorbit_plan_;  // last ship return plan (also when refused)
    bool ship_return_ = false;                      // the burn is the deorbit burn of a ship return
    gnc::DeorbitParameters deorbit_parameters_;
    time::Epoch entry_start_;                       // state Return: entry guidance from here
    CapsuleReturnStep capsule_return_step_ = CapsuleReturnStep::None;
    time::Epoch capsule_step_epoch_;                // the next timed step (trunk jettison, nose cone)
    math::Quat capsule_hold_attitude_;              // with the trunk on (no capsule aero yet): hold the burn's attitude
    void run_capsule_return(Flying& flying);

    // Lunar flight (flight_session_lunar.cpp).
    LunarStep lunar_step_ = LunarStep::None;
    time::Epoch lunar_step_epoch_;  // next timed event of the step machine
    std::optional<mission::LunarOutboundParameters> lunar_parameters_;
    std::optional<mission::OutboundPlan> lunar_plan_;
    std::optional<mission::TwoBurnTransfer> lunar_transfer_;
    mission::LunarOrbitKind lunar_transfer_target_ = mission::LunarOrbitKind::Llo;
    std::shared_ptr<const mission::NrhoReference> lunar_nrho_;
    mission::LloTarget lunar_llo_;
    size_t lunar_mcc_index_ = 0;
    bool lunar_flyby_done_ = false;
    int lunar_trims_ = 0;
    time::Epoch lunar_tli_end_;
    time::Epoch lunar_next_repoint_;
    std::string lunar_status_;
    gnc::SunHold lunar_sun_hold_;
    bool lunar_debug_ = false;     // STARSIM_DEBUG_LUNAR_SESSION, read once at construction
    std::string lunar_debug_last_;  // last status printed by the diagnostics
    struct LunarPilotCall {
        gnc::BurnNode planned;
        time::Epoch window_end;
        // The state at the call coasted on gravity alone, kept at the last tick: current velocity minus its velocity is
        // the realised delta-v.
        math::StateVector reference;
        time::Epoch reference_epoch;
    };
    std::optional<LunarPilotCall> lunar_pilot_;
    const frames::BodyOrientation* moon_orientation_ = nullptr;
    void run_lunar(Flying& flying);  // the step machine and its diagnostics (STARSIM_DEBUG_LUNAR_SESSION)
    void run_lunar_steps(Flying& flying);
    void lunar_coast_event();                 // Coast: the timed event is due (MCC, swing-by, insertion)
    void lunar_enter_coast();                 // back to Coast: Sun pointing, the next event's epoch
    time::Epoch lunar_next_mcc_epoch() const;  // TLI end + the next MCC offset (far future if none is left)
    time::Epoch lunar_ignition_lead(const gnc::BurnNode& node) const;  // ignition - preparation on the vacuum Raptors
    // Executes and records the burn (step: what it is); the burn returns to the state Lunar.
    bool lunar_burn(const gnc::BurnNode& node, bool main_engines, LunarStep step);
    std::vector<LunarBurnRecord> lunar_burns_;
    void lunar_abort(const std::string& status);
    void reset_lunar();
    // The descent steps from DescentWait to Lost (flight_session_lunar_descent.cpp).
    void run_lunar_descent_steps(Flying& flying);
    void lunar_sun_coast(Flying& flying);  // Sun pointing of a coast (Coast, DescentWait) and its re-points
    void lunar_descent_wait();             // DescentWait for the next descent burn (trim, then DOI)
    bool lunar_descent_burn(const gnc::BurnNode& node, LunarStep step);  // trim / DOI, recorded
    // A small lunar burn (trims, DOI, circularisation): the single gimballed sea-level Raptor (the approach set) when
    // its start/shutdown impulse fits, else the session's choice (RCS or a Raptor set).
    bool lunar_small_burn(const gnc::BurnNode& node);
    std::optional<mission::LunarDescentPlan> lunar_descent_plan_;
    std::optional<LunarLandingSetup> lunar_landing_setup_;
    time::Epoch lunar_landed_epoch_;
    bool lunar_descent_trimmed_ = false;  // the plan's plane trim is flown (the next wait is for the DOI)
    std::unique_ptr<gnc::LunarLandingGuidance> lunar_landing_;
    std::optional<LunarTouchdown> lunar_touchdown_;
    mission::TouchdownPoint lunar_touchdown_point_;
    // The ascent steps (flight_session_lunar_ascent.cpp).
    void run_lunar_ascent_steps(Flying& flying);
    void lunar_liftoff(Flying& flying);  // releases the landed body, starts the ascent guidance (step Ascent)
    bool lunar_abort_to_orbit();         // propellant check, then step AbortAscent; false with the status set
    std::unique_ptr<gnc::LunarAscentGuidance> lunar_ascent_;
    std::optional<gnc::LunarAscentParameters> lunar_ascent_parameters_;  // planned liftoff (SurfaceStay until then)
    // The mission sequence (set_lunar_mission): which leg the next InLlo / InNrho ends.
    enum class LunarMissionLeg { None, Outbound, Surface, Return, Complete, Ended };
    std::optional<LunarMissionSequence> lunar_mission_;
    LunarMissionLeg lunar_mission_leg_ = LunarMissionLeg::None;
    void lunar_mission_next();  // in InLlo / InNrho with the autopilot idle: plans the next leg
    bool lunar_auto_abort_tried_ = false;  // the landing guidance gave up: the abort to orbit was tried once
    math::StateVector controlled_state() const;

    // `forced`: only this set (config group ids), at full and at minimum throttle.
    std::vector<BurnConfiguration> burn_configurations(const std::vector<std::string>* forced = nullptr) const;
    // `rcs_fallback`: below every configuration's minimum impulse the RCS flies the burn (not for a forced set).
    // `max_transient_share`: a configuration fits when its start/shutdown impulse is at most this share of the burn.
    bool start_burn(const gnc::BurnNode& node, const std::vector<BurnConfiguration>& configurations,
                    bool rcs_fallback = true, double max_transient_share = 0.25);
    // [E] lunar_small_burn: the single Raptor's transients may be up to 0.6 of the burn (the executor's cutoff allows for
    // the tail-off) - a 10.7 m/s plane trim at ~570 t missed the quarter rule by 0.01 m/s and took 9.4 t of cold gas; the
    // 18 m/s circularisation at 151 t (transients 0.56) flew on the RCS, ~15 min off-centre: 93.7 x 101.9 km (l4 C4).
    static constexpr double small_burn_transient_share = 0.6;
};

}  // namespace starsim::sim
