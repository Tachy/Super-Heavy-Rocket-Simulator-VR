#include "starsim/sim/ascent_mission.hpp"

#include "starsim/config/drone_ship_config.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/vehicle_config.hpp"

namespace starsim::sim {

void start_ascent_mission(FlightSession& session, const config::AscentMission& mission,
                          const std::filesystem::path& config_dir, time::Epoch epoch) {
    auto vehicle = config::load_vehicle(config_dir, mission.vehicle_id);
    vehicle.stages.back().payload_mass_kg = mission.payload_kg;
    if (!mission.fairing) vehicle.stages.back().fairing_mass_kg = 0.0;
    session.launch_from_pad(vehicle, launch_site_from_config(config::load_launch_site(config_dir, mission.site_id)),
                            mission.heading_rad, epoch);
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
    program.booster_throttle = mission.booster_throttle;
    session.engage_ascent_autopilot(
        {mission.periapsis_height_m, mission.apoapsis_height_m, mission.inclination_rad, mission.northerly}, program);
}

}  // namespace starsim::sim
