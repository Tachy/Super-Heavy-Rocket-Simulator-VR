#pragma once

#include <filesystem>
#include <functional>
#include <optional>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/mission/lunar_budget.hpp"
#include "starsim/mission/lunar_orbit_transfer.hpp"
#include "starsim/mission/lunar_outbound.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/mission_start.hpp"

namespace starsim::sim {

// Places the mission's HLS (`vehicle`: an override of the mission's vehicle) in its circular parking orbit at `epoch`,
// the plane through the Moon at arrival (epoch + flight_time_s; LunarTransferTargeter::parking_orbit_raan), and gives
// the session the Moon's terrain (data/ next to `config_dir`) and the mission's landing site. Throws std::runtime_error
// (German) when no such plane exists (inclination below the Moon's declination), the session has no Moon orientation
// or a terrain file is missing (the message names tools/data_pipeline/fetch.py).
void place_lunar_landing_mission(FlightSession& session, const config::LunarLandingMission& mission,
                                 const std::filesystem::path& config_dir, time::Epoch epoch,
                                 const config::VehicleConfig* vehicle = nullptr);

// The landing, surface stay and ascent of the mission ([descent], [surface], [ascent], [budget]).
LunarLandingSetup lunar_landing_setup(const config::LunarLandingMission& mission);

// The budget legs of the mission (refuelling r6: the campaign's HLS demand), in flight order. NRHO: the leg
// "nrho_plane_change" = `nrho_plane_change_mps` (lunar_nrho_site_departure: the site's share of the departure burn).
// After the ascent, when positive, the leg "stay_plane_change" = `stay_plane_change_mps` (lunar_stay_plane_change_mps:
// the Moon's rotation over the stay, turned back in the LLO; one ignition).
std::vector<mission::BudgetLeg> lunar_mission_legs(const config::LunarLandingMission& mission,
                                                   const mission::OutboundPlan& plan, double vacuum_isp_s,
                                                   double approach_isp_s, double nrho_plane_change_mps = 0.0,
                                                   double stay_plane_change_mps = 0.0);

// The plane change the surface stay causes (lunar sites plan, ruling R15). The ascent flies into the plane through the
// liftoff point closest to the landing LLO's plane (gnc::ascent_plane_through: no plane change in the ascent); the
// orbit the mission returns to starts from that landing plane - the LLO the HLS ends in (the LLO mission), the LLO the
// [budget] llo_to_nrho leg was measured from (the NRHO mission). The turn back is the site's distance from the landing
// plane at liftoff, di = asin(|n . s(touchdown + stay)|), flown as a pure plane change at a node of the circular LLO,
// dv = 2 v_llo sin(di / 2) [S: Vallado, Fundamentals of Astrodynamics and Applications, 4th ed., sec. 6.4 "Inclination
// Only"]. The LLO plane is inertially fixed over the stay (the force model has no lunar gravity harmonics). Without a
// known plane (`llo_plane_normal_moon_inertial` empty) the bound: the site's inertial turn over the stay, the angle
// between s(touchdown) and s(touchdown + stay) (a plane holding s(touchdown) is never farther from s(liftoff)).
double lunar_stay_plane_change_mps(const frames::BodyOrientation& moon, const config::LunarSiteConfig& site,
                                   const std::optional<math::Vec3>& llo_plane_normal_moon_inertial,
                                   time::Epoch touchdown, double stay_s, double llo_radius_m, double gm_moon_m3ps2);

// lunar_stay_plane_change_mps for the mission planned in `session` (its site, outbound plan and, NRHO, `departure`):
// the plane and touchdown of the NRHO departure, else of the outbound plan's LLO (llo_plane_normal_moon_inertial,
// llo_touchdown), else the bound at the mission's planned touchdown (or the first pass after the arrival). 0 without
// a landing site or Moon orientation.
double lunar_mission_stay_plane_change_mps(const FlightSession& session, const config::LunarLandingMission& mission,
                                           time::Epoch epoch,
                                           const std::optional<mission::NrhoSiteDeparture>& departure,
                                           std::optional<time::Epoch> touchdown_override = std::nullopt);

// The budget margin of an LLO mission planned in `session` (after start_lunar_landing_mission) with the stay's plane
// change taken at `stay_touchdown` - the touchdown the descent actually flies (the start budgets the planned pass).
double lunar_llo_budget_margin_kg(const FlightSession& session, const config::LunarLandingMission& mission,
                                  time::Epoch epoch, time::Epoch stay_touchdown);

// Rocket-equation budget of the whole mission from the planned outbound flight (TLI, swing-by, insertion / LOI) and the
// mission's nominal legs (NRHO -> LLO, descent, ascent, LLO -> NRHO). `vacuum_isp_s`: the vacuum Raptors (outbound,
// transfers, braking phase, ascent); `approach_isp_s`: the rest of the descent (the last braking set's Isp - the
// landing thrusters since the l4 mass correction). Refuelling f3: the reserve grows by `settling_gas_kg` per in-orbit
// ignition (FlightSession::settling_gas_bound_kg).
mission::BudgetResult lunar_mission_budget(const config::LunarLandingMission& mission, const mission::OutboundPlan& plan,
                                           double initial_mass_kg, double dry_mass_kg, double vacuum_isp_s,
                                           double approach_isp_s, double settling_gas_kg,
                                           double nrho_plane_change_mps = 0.0, double stay_plane_change_mps = 0.0);

// NRHO variant (lunar sites plan, ruling R12): the site-dependent extra delta-v of the NRHO -> LLO departure - the turn
// from the NRHO plane at the departure perilune onto the plane through the site (mission::nrho_site_departure; the
// [budget] nrho_to_llo_delta_v_mps is the coplanar transfer). `outbound`: the mission's parameters with the reference
// (lunar_outbound_parameters), `plan` its outbound plan (the NRHO is joined at its insertion). Empty for an LLO mission
// or when the reference holds no departure perilune.
std::optional<mission::NrhoSiteDeparture> lunar_nrho_site_departure(const config::LunarLandingMission& mission,
                                                                    const mission::LunarOutboundParameters& outbound,
                                                                    const mission::OutboundPlan& plan,
                                                                    const ephemeris::Ephemeris& ephemeris,
                                                                    const frames::BodyOrientation& moon,
                                                                    const config::LunarSiteConfig& site,
                                                                    time::Epoch epoch);

// The in-flight ignitions of the legs (refuelling f3: each settles first; f4: per leg as flown - DOI + PDI, the
// ascent's circularization, two-burn NRHO transfers). A lower bound: mid-course corrections, a plane trim and NRHO
// station keeping are not budgeted legs.
size_t lunar_in_orbit_ignitions(const std::vector<mission::BudgetLeg>& legs);

// start_mission's lunar branch (`vehicle`: an override, e.g. the 204 t HLS): place, plan the outbound flight, check a
// touchdown point within the slope limit at the site for the approach the flight will fly (the plan's LLO plane over
// the site at its touchdown, llo_approach_azimuth; an inclination-targeted or NRHO-reached LLO: a polar pass, both
// directions), check the budget, then chain the legs (FlightSession::set_lunar_mission). Refused (return_planned false,
// German refusal, autopilot off): placement or terrain failed, outbound plan refused, no touchdown point, "Mission
// nicht durchführbar: " + budget. The pass over the site is checked again in the LLO when the descent is planned.
// `site`: land there instead of the mission's [mission] site (a free site, config::lunar_site_at; its tile must be in
// data/terrain/sites). A planned start carries the budget margin and the planned touchdown.
MissionStart start_lunar_landing_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                         const config::LunarLandingMission& mission, time::Epoch epoch,
                                         const config::VehicleConfig* vehicle = nullptr,
                                         const config::LunarSiteConfig* site = nullptr);

// start_lunar_landing_mission in steps (follow-up U-7): the vehicle is placed and status start.checking_landing_site
// given at once; the first poll (or wait) plans the outbound flight and starts the touchdown search (on the composed
// surface with the hazard check: on a background thread - LOLA alone is decided in that poll), the later polls plan the
// rest from the search's result - the same start as the synchronous call. A failed placement is decided at once.
DeferredMissionStart begin_lunar_landing_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                                 const config::LunarLandingMission& mission, time::Epoch epoch,
                                                 const config::VehicleConfig* vehicle = nullptr,
                                                 const config::LunarSiteConfig* site = nullptr);

// The mission's circular parking state at `epoch` (GCRF) for `inclination_rad`: the plane through the Moon at arrival
// (epoch + flight_time_s), as the placement uses it; std::nullopt when the Moon's declination exceeds the inclination.
std::optional<math::StateVector> lunar_parking_state(const FlightSession& session, const config::LunarLandingMission& mission,
                                                     time::Epoch epoch, double inclination_rad);

// The lunar landing mission from the controlled body as it flies (refuelling r5: the HLS after the depot): terrain and
// site set, the touchdown point checked, the outbound flight planned from its live state, the budget closed with its
// real mass and tank load, the legs chained. Nothing is placed or cleared. Refused (German, autopilot off) as
// start_lunar_landing_mission, and when the session's epoch is past `epoch` ("zu spät").
MissionStart continue_lunar_landing_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                            const config::LunarLandingMission& mission, time::Epoch epoch);
// The same, deferred: the outbound plan from the live state at once, the touchdown search (with the hazard check up to
// the full search radius, 16-30 s) on a background thread; poll / wait decide it (done at once when no search runs).
DeferredMissionStart begin_continue_lunar_landing_mission(FlightSession& session,
                                                          const std::filesystem::path& config_dir,
                                                          const config::LunarLandingMission& mission,
                                                          time::Epoch epoch);
// [E] Sim time a background touchdown search of the continuation may take before it is waited for (as
// FlightSession::ship_return_budget_s): far above the 16-30 s worst case at warp 1, far below the outbound plan's
// 1800 s lead to its first burn (min_lead_s), so the plan is still ahead when it is committed.
inline constexpr double lunar_continuation_budget_s = 60.0;
// The refuelling continuation (IssMissionSequence::continuation, WaitContinue): begins the deferred continuation at its
// first call; with the session's real-time planning on it returns nothing (still deciding, called again next step, the
// vehicle flies on) until the search is done or lunar_continuation_budget_s of sim time has passed, then waits; off
// (tests, campaigns, time warp) it decides at once on the sim thread - the same result either way.
std::function<std::optional<report::Report>(FlightSession&)> lunar_landing_continuation(
    std::filesystem::path config_dir, config::LunarLandingMission mission, time::Epoch epoch);

// The touchdown the mission started at `epoch` is planned for: `epoch` + [descent] touchdown_after_epoch_h (a duration
// from the mission's start, so a start at another epoch keeps its timing); empty without it.
std::optional<time::Epoch> lunar_planned_touchdown(const config::LunarLandingMission& mission, time::Epoch epoch);

// Without a planned touchdown: the first pass after the LLO arrival the session can take - the trims' settling
// revolutions, then the descent planner's minimum lead and 1.25 revolutions.
double lunar_first_pass_after_arrival_s(const config::LunarLandingMission& mission, double gm_moon_m3ps2);
// The part of it until the session is in the LLO and plans the descent (the trims' settling revolutions, [E]).
double lunar_llo_settling_s(const config::LunarLandingMission& mission, double gm_moon_m3ps2);

// Outbound parameters of the mission started at `epoch` (FlightSession::plan_lunar_outbound); the NRHO variant builds
// its reference (multiple shooting, ~15 s) with the first perilune at the arrival. LLO without [lunar_orbit]
// inclination_deg: the plane is chosen through `site` at lunar_planned_touchdown, else at the first pass after the
// arrival (without a site: polar).
mission::LunarOutboundParameters lunar_outbound_parameters(const config::LunarLandingMission& mission,
                                                           const environment::GravityModel& gravity,
                                                           const ephemeris::Ephemeris& ephemeris, time::Epoch epoch,
                                                           const config::LunarSiteConfig* site = nullptr);

}  // namespace starsim::sim
