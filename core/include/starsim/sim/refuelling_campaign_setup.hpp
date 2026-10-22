#pragma once

// Refuelling r6: the campaign calculator's models from config and the r5 reference flights - real launch windows
// (mission::plan_launch_window against the depot's J2-advanced orbit), the depot's heat-balance boil-off with the day's
// Sun, its drag makeup (US76), the tanker's net delivery with the levers, and the HLS demand from the lunar mission's
// planned outbound flight.

#include <filesystem>
#include <optional>
#include <string>

#include "starsim/config/refuelling_reference.hpp"
#include "starsim/ephemeris/ephemeris.hpp"
#include "starsim/frames/body_orientation.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/mission/iss_launch_window.hpp"
#include "starsim/mission/refuelling_campaign.hpp"
#include "starsim/time/epoch.hpp"
#include "starsim/time/time_scale_converter.hpp"

namespace starsim::sim {

// The calculator's levers (spec r6; std::nullopt: the reference flight's value).
struct CampaignLevers {
    std::optional<double> depot_height_m;      // circular; reference: tanker_flight's [depot]
    std::optional<double> tanker_dry_mass_kg;  // reference: the measured tanker
    std::optional<double> tanker_reserve_kg;   // main tanks kept for the return; reference: main at undock
    std::optional<double> phasing_limit_s;     // tanker and HLS; reference: the missions' limits (48 h)
    std::optional<double> depot_initial_kg;    // reference: depot_launch's residue
    double depot_insulation_scale = 1.0;       // x the depot's wall resistance
    double launch_interval_s = 86400.0;        // [E] pad turnaround
    double hls_wait_s = 0.0;                   // HLS docked -> TLI (its boil-off is demanded)
    bool reference_phasing = false;            // f1: only the r5 phasing orbit (170 x 190 km) instead of the candidates
};

struct CampaignSources {
    const ephemeris::Ephemeris& ephemeris;
    const frames::BodyOrientation& earth;
    const frames::BodyOrientation& moon;
    const time::TimeScaleConverter& time;
    std::filesystem::path config_dir;
};

// The smallest HLS mass that closes the lunar mission from its planned outbound flight (r6 plan decision 2).
struct CampaignHlsDemand {
    double required_mass_kg = 0.0, dry_mass_kg = 0.0, capacity_kg = 0.0, headers_kg = 0.0;
};

// Refuelling f2: a re-plan from the flown campaign - t = 0 at `epoch`, the depot from its flown state
// (orbit::station_mean_elements_from_state: its mean height for the drag and the shifted phasing orbits), its fill, the
// last flight (the launch interval and the port spacing count from it), no launch earlier than launch_lead_s, and the
// HLS demand of the first plan (empty: planned, as without a live start).
struct CampaignLive {
    time::Epoch epoch;
    math::StateVector depot_gcrf;
    double depot_propellant_kg = 0.0;
    std::optional<time::Epoch> previous_launch, previous_arrival;
    double launch_lead_s = 3600.0;  // [E] pad readiness after a decision
    std::optional<CampaignHlsDemand> demand;
};

struct CampaignReport {
    mission::CampaignResult result;
    time::Epoch start;  // t = 0: the tanker mission's earliest_utc
    std::string lunar_mission_id;
    time::Epoch lunar_epoch;
    double depot_height_m = 0.0, depot_capacity_kg = 0.0;
    double hls_required_mass_kg = 0.0, hls_dry_mass_kg = 0.0;
    double hls_propellant_at_docking_kg = 0.0, hls_capacity_kg = 0.0;
    double demand_kg = 0.0;
    double tanker_base_delivered_kg = 0.0;  // reference phasing time, levers applied
    double altitude_cost_kg = 0.0;          // per tanker (signed: < 0 below the reference)
    double reference_phasing_s = 0.0;
    double drag_makeup_kgps = 0.0;
    CampaignHlsDemand hls_demand;
    mission::LaunchWindowParameters tanker_window, hls_window;  // as the windows were planned (f2: flown with them)
};

// The first lever out of range, as a German message (empty: all valid): heights, times and the insulation > 0, the
// reserve, the initial fill and the HLS wait >= 0, all finite (review I3: one check for the CLI and the game).
std::string campaign_lever_error(const CampaignLevers& levers);

// f2: the flown campaign's calibration of a tanker's delivery by its phasing orbit's class - above the depot when the
// orbit's periapsis lies above the depot's mean radius, else (and without a phasing orbit: the separation orbit) below.
double campaign_delivery_offset_kg(const config::RefuellingReference::Tanker& tanker,
                                   const std::optional<mission::PhasingOrbit>& orbit, double depot_radius_m);

// The campaign of the r5 reference flights with the levers applied (r6 plan decisions 1-7). Refusals (lunar outbound
// plan, HLS tanks too small, levers out of range) come back as result.closes == false with a German reason; config
// errors throw. `live` (f2): re-planned from the flown campaign's state (the depot height lever is then refused: the
// flown depot's own height counts).
CampaignReport plan_refuelling_campaign(const CampaignSources& sources, const CampaignLevers& levers,
                                        const CampaignLive* live = nullptr);

}  // namespace starsim::sim
