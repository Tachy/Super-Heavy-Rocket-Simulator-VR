#pragma once

// Refuelling r6: the tanker campaign calculator - a pure event loop over injected models (the physics comes from
// sim::plan_refuelling_campaign).

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "starsim/mission/phasing_schedule.hpp"

namespace starsim::mission {

struct CampaignFlight {
    double launch_s = 0.0;      // from the campaign start
    double arrival_s = 0.0;     // at the depot (Waypoint 0)
    double delivered_kg = 0.0;  // tankers: what it hands over (before the transfer vent)
    // the HLS: what its phasing orbit costs beyond its reference window's - the depot must hold that much more at its
    // arrival (refuelling f1)
    double extra_demand_kg = 0.0;
    std::optional<PhasingOrbit> phasing_orbit;  // the window's choice (f1; none: the separation orbit)
};

struct CampaignModels {
    std::function<double(double t_s, double depot_propellant_kg)> boil_off_kgps;
    std::function<double(double t_s)> drag_makeup_kgps;
    // The first window at or after `earliest_launch_s`; std::nullopt: none within the search.
    std::function<std::optional<CampaignFlight>(double earliest_launch_s)> next_tanker;
    std::function<std::optional<CampaignFlight>(double earliest_launch_s)> next_hls;
};

struct CampaignInputs {
    double depot_capacity_kg = 0.0;  // main tanks
    double depot_initial_kg = 0.0;
    double demand_kg = 0.0;                     // what the depot must hold when the HLS arrives
    double attitude_gas_kgps = 20.0 / 86400.0;  // [E] attitude hold (the flights fly an ideal LVLH depot)
    double transfer_vent_fraction = 0.015;      // [E] pressurant + vented displaced gas (r5: 1-2 %, not modelled)
    double launch_interval_s = 86400.0;         // [E] pad turnaround between launches
    double port_spacing_s = 6.0 * 3600.0;       // [E] one depot port: docking, transfer, departure (~3 h flown)
    double window_retry_s = 3600.0;             // [E] a window too close to the last arrival: search on from here
    double step_s = 86400.0;                    // loss integration step (daily)
    int max_tankers = 60;
};

struct CampaignLosses {
    double boil_off_kg = 0.0, drag_makeup_kg = 0.0, attitude_gas_kg = 0.0, transfer_vent_kg = 0.0;
};

struct CampaignSample {
    double t_s = 0.0;
    double depot_propellant_kg = 0.0;
};

struct CampaignTanker {
    CampaignFlight flight;
    double received_kg = 0.0;
    double depot_after_kg = 0.0;
};

struct CampaignResult {
    bool closes = false;
    std::string reason;  // German when it does not close
    int tanker_flights = 0;
    double duration_s = 0.0;  // campaign start -> HLS arrival
    std::optional<CampaignFlight> hls;
    double depot_at_hls_kg = 0.0;
    std::vector<CampaignTanker> tankers;
    std::vector<CampaignSample> fill;  // after every step and every arrival
    CampaignLosses losses;             // until the HLS arrival (or the end)
};

// The tanker campaign (refuelling r6, spec "r6 design"): from t = 0 the depot loses boil-off, drag makeup and
// attitude gas in steps of at most step_s; tankers launch at their windows (launch_interval_s apart, arrivals
// port_spacing_s apart) and add delivered x (1 - vent), clamped to the tank volume; once the fill reaches the demand,
// the HLS's window is checked - the campaign closes when the depot still holds the demand at its arrival. Ends with a
// German reason when the demand exceeds the volume, a window is missing, a tanker adds nothing, or max_tankers.
CampaignResult refuelling_campaign(const CampaignInputs& inputs, const CampaignModels& models);

}  // namespace starsim::mission
