#include "starsim/sim/ascent_mission.hpp"

#include <algorithm>
#include <stdexcept>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/propellant_loading.hpp"
#include "starsim/config/vehicle_config.hpp"

namespace starsim::sim {
namespace {

// The [propellant] loads into the named stages (the others stay full).
void load_propellant(config::VehicleConfig& vehicle, const std::map<std::string, double>& propellant_load_kg) {
    for (const auto& [stage_id, kg] : propellant_load_kg) {
        auto stage = std::find_if(vehicle.stages.begin(), vehicle.stages.end(),
                                  [&](const config::StageConfig& s) { return s.id == stage_id; });
        if (stage == vehicle.stages.end()) {
            throw std::invalid_argument("[propellant] names no stage '" + stage_id + "'");
        }
        double capacity_kg = 0.0;
        for (const auto& t : stage->tanks) {
            if (t.contents != config::TankContents::Rcs) capacity_kg += t.capacity_kg;
        }
        if (kg > capacity_kg) {
            throw std::invalid_argument("[propellant] " + stage_id + " above its capacity");
        }
        config::fill_tanks(*stage, kg);
    }
}

}  // namespace

config::VehicleConfig build_ascent_stack(config::VehicleConfig vehicle, const std::filesystem::path& config_dir,
                                         const AscentStackSpec& spec) {
    vehicle.stages.back().payload_mass_kg = spec.payload_kg;
    if (!spec.fairing) vehicle.stages.back().fairing_mass_kg = 0.0;
    load_propellant(vehicle, spec.propellant_load_kg);
    if (spec.payload) {
        vehicle = config::stack_payload(std::move(vehicle), config::load_vehicle(config_dir, spec.payload->vehicle_id));
    }
    return vehicle;
}

AscentProfile start_ascent_mission(FlightSession& session, const config::AscentMission& mission,
                                   const std::filesystem::path& config_dir, time::Epoch epoch,
                                   const config::VehicleConfig* configured, bool keep_station) {
    std::optional<PayloadSeparation> separation;
    if (const auto& p = mission.payload) {
        const auto payload_vehicle = config::load_vehicle(config_dir, p->vehicle_id);
        separation = PayloadSeparation{payload_vehicle.stages.front().id, p->separation_delay_s, p->separation_dv_mps,
                                       p->nose_cone_open_s};
    }
    AscentProfileRequest request;
    request.vehicle_id = mission.vehicle_id;
    request.site_id = mission.site_id;
    request.heading_rad = mission.heading_rad;
    request.liftoff = epoch;
    request.target = {mission.periapsis_height_m, mission.apoapsis_height_m, mission.inclination_rad, mission.northerly};
    request.payload_kg = mission.payload_kg;
    request.fairing = mission.fairing;
    request.payload = mission.payload;
    AscentStackSpec spec{mission.payload_kg, mission.fairing, mission.payload,
                         {mission.propellant_load_kg.begin(), mission.propellant_load_kg.end()}};
    config::VehicleConfig vehicle;
    try {
        vehicle = build_ascent_stack(configured ? *configured : config::load_vehicle(config_dir, mission.vehicle_id),
                                     config_dir, spec);
    } catch (const std::invalid_argument& e) {
        throw std::invalid_argument("mission " + mission.id + ": " + e.what());
    }
    // a changed vehicle or a recorded propellant load: planned as it stands
    if (configured || !mission.propellant_load_kg.empty()) request.stack = vehicle;
    session.launch_from_pad(vehicle, launch_site_from_config(config::load_launch_site(config_dir, mission.site_id)),
                            mission.heading_rad, epoch, keep_station);
    session.set_payload_separation(separation);  // after launch_from_pad (a new launch resets it)
    return engage_planned_ascent(session, request);
}

AscentLaunch start_ascent_mission_for_liftoff(FlightSession& session, const config::AscentMission& mission,
                                              const std::filesystem::path& config_dir, time::Epoch liftoff,
                                              std::optional<time::Epoch> wait_from, bool fixed_liftoff) {
    time::Epoch pad = wait_from && *wait_from - liftoff < 0.0 ? *wait_from : liftoff;
    auto profile = start_ascent_mission(session, mission, config_dir, pad);
    if (!wait_from || !profile.feasible) return {pad, profile};
    // the engines need the lead before T-0: a fixed liftoff moves the placement earlier, a countdown T-0 later
    const double lead_s = session.liftoff_ignition_lead_s();
    time::Epoch t0 = liftoff;
    if (liftoff - pad < lead_s) {
        if (fixed_liftoff) {
            pad = liftoff - lead_s;
            // placed again: the plan from the memo - except a mission with a [propellant] load, which is never
            // memoised (planned as it stands) and is planned a second time here
            profile = start_ascent_mission(session, mission, config_dir, pad);
        } else {
            t0 = pad + lead_s;
        }
    }
    session.schedule_liftoff(t0);
    return {pad, profile};
}

}  // namespace starsim::sim
