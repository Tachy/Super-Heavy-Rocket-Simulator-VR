#include "starsim/sim/capsule_return_mission.hpp"

#include "starsim/config/splashdown_zone_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"

namespace starsim::sim {

void place_capsule_return_mission(FlightSession& session, const config::CapsuleReturnMission& m,
                                  const std::filesystem::path& config_dir, time::Epoch epoch) {
    auto cfg = config::load_vehicle(config_dir, m.vehicle_id);
    for (auto& stage : cfg.stages) {
        for (auto& tank : stage.tanks) {
            if (tank.contents == config::TankContents::Rcs) tank.mass_kg = m.rcs_propellant_kg;
        }
    }
    const double re = constants::wgs84_semi_major_axis_m;
    const double rp = re + m.periapsis_height_m, ra = re + m.apoapsis_height_m;
    // circular orbits: the argument of latitude is the anomaly from the ascending node
    const orbit::KeplerianElements elements{0.5 * (rp + ra), (ra - rp) / (ra + rp), m.inclination_rad, m.raan_rad, 0.0,
                                            m.argument_of_latitude_rad};
    session.place_in_orbit(cfg, orbit::state_from_elements(elements, session.gm_earth_m3ps2()), epoch);
    session.set_splashdown_zone(config::load_splashdown_zone(config_dir, m.zone_id));
    session.set_nose_cone(0, true);
}

gnc::DeorbitParameters capsule_return_parameters(const config::CapsuleReturnMission& m) {
    auto p = FlightSession::capsule_deorbit_parameters();
    p.entry_interface_height_m = m.entry_interface_height_m;
    p.entry_flight_path_rad = m.entry_flight_path_rad;
    p.nominal_bank_rad = m.nominal_bank_rad;
    p.max_crossrange_m = m.max_crossrange_m;
    p.search_window_s = m.search_window_s;
    return p;
}

}  // namespace starsim::sim
