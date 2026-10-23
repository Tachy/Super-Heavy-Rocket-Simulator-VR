#include "starsim/sim/ascent_mission.hpp"

#include "starsim/config/drone_ship_config.hpp"
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

void start_ascent_mission(FlightSession& session, const config::AscentMission& mission,
                          const std::filesystem::path& config_dir, time::Epoch epoch,
                          const config::VehicleConfig* configured, bool keep_station) {
    const auto vehicle = ascent_stack(configured ? *configured : config::load_vehicle(config_dir, mission.vehicle_id),
                                      config_dir, mission.payload_kg, mission.fairing, mission.payload);
    std::optional<PayloadSeparation> separation;
    if (const auto& p = mission.payload) {
        separation = PayloadSeparation{config::load_vehicle(config_dir, p->vehicle_id).stages.front().id,
                                       p->separation_delay_s, p->separation_dv_mps, p->nose_cone_open_s};
    }
    session.launch_from_pad(vehicle, launch_site_from_config(config::load_launch_site(config_dir, mission.site_id)),
                            mission.heading_rad, epoch, keep_station);
    if (!mission.landing_zone_id.empty()) {
        session.set_landing_zone(launch_site_from_config(config::load_launch_site(config_dir, mission.landing_zone_id)));
    }
    if (const auto& ship = mission.drone_ship) {
        session.set_landing_zone(drone_ship_site(config::load_drone_ship(config_dir, ship->id), ship->latitude_rad,
                                                 ship->longitude_rad, ship->sea_height_m, ship->heading_rad));
    }
    // Return guidance only where the stage can come back (a tower at the site, or a landing zone for a stage on legs).
    session.set_booster_return(true);
    gnc::PitchProgram program;
    program.kick_angle_rad = mission.kick_angle_rad;
    session.set_payload_separation(separation);  // after launch_from_pad (a new launch resets it)
    session.engage_ascent_autopilot(
        {mission.periapsis_height_m, mission.apoapsis_height_m, mission.inclination_rad, mission.northerly}, program);
}

time::Epoch start_ascent_mission_for_liftoff(FlightSession& session, const config::AscentMission& mission,
                                             const std::filesystem::path& config_dir, time::Epoch liftoff,
                                             std::optional<time::Epoch> wait_from, bool fixed_liftoff) {
    time::Epoch pad = wait_from && *wait_from - liftoff < 0.0 ? *wait_from : liftoff;
    start_ascent_mission(session, mission, config_dir, pad);
    if (!wait_from) return pad;
    // the engines need the lead before T-0: a fixed liftoff moves the placement earlier, a countdown T-0 later
    const double lead_s = session.liftoff_ignition_lead_s();
    time::Epoch t0 = liftoff;
    if (liftoff - pad < lead_s) {
        if (fixed_liftoff) {
            pad = liftoff - lead_s;
            start_ascent_mission(session, mission, config_dir, pad);
        } else {
            t0 = pad + lead_s;
        }
    }
    session.schedule_liftoff(t0);
    return pad;
}

}  // namespace starsim::sim
