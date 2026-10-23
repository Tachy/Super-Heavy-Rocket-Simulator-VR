#include "starsim/sim/ascent_mission.hpp"

#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/vehicle_config.hpp"

namespace starsim::sim {

config::VehicleConfig ascent_stack(config::VehicleConfig vehicle, const std::filesystem::path& config_dir,
                                   double payload_kg, bool fairing, const std::optional<config::MissionPayload>& payload) {
    vehicle.stages.back().payload_mass_kg = payload_kg;
    if (!fairing) vehicle.stages.back().fairing_mass_kg = 0.0;
    if (payload) {
        vehicle = config::stack_payload(std::move(vehicle), config::load_vehicle(config_dir, payload->vehicle_id));
    }
    return vehicle;
}

AscentProfile start_ascent_mission(FlightSession& session, const config::AscentMission& mission,
                                   const std::filesystem::path& config_dir, time::Epoch epoch,
                                   const config::VehicleConfig* configured, bool keep_station) {
    std::optional<PayloadSeparation> separation;
    std::optional<config::VehicleConfig> payload_vehicle;
    if (const auto& p = mission.payload) {
        payload_vehicle = config::load_vehicle(config_dir, p->vehicle_id);
        separation = PayloadSeparation{payload_vehicle->stages.front().id, p->separation_delay_s, p->separation_dv_mps,
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
    auto vehicle = configured ? *configured : config::load_vehicle(config_dir, mission.vehicle_id);
    vehicle.stages.back().payload_mass_kg = mission.payload_kg;
    if (!mission.fairing) vehicle.stages.back().fairing_mass_kg = 0.0;
    if (payload_vehicle) vehicle = config::stack_payload(std::move(vehicle), *payload_vehicle);
    if (configured) request.stack = vehicle;  // a changed vehicle: planned as it stands
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
            profile = start_ascent_mission(session, mission, config_dir, pad);  // the plan from the memo
        } else {
            t0 = pad + lead_s;
        }
    }
    session.schedule_liftoff(t0);
    return {pad, profile};
}

}  // namespace starsim::sim
