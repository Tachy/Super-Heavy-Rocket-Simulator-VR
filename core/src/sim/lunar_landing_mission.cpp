#include "starsim/sim/lunar_landing_mission.hpp"

#include <memory>
#include <stdexcept>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/lunar_transfer.hpp"
#include "starsim/mission/nrho_reference.hpp"
#include "starsim/orbit/kepler.hpp"

namespace starsim::sim {

void place_lunar_landing_mission(FlightSession& session, const config::LunarLandingMission& m,
                                 const std::filesystem::path& config_dir, time::Epoch epoch) {
    if (!session.moon_orientation()) {
        throw std::runtime_error("Mondlandung: die Sitzung hat keine Mondorientierung");
    }
    config::load_lunar_site(config_dir, m.site_id);  // the landing site must exist (l4 lands there)
    const mission::LunarTransferTargeter targeter(session.gravity(), session.ephemeris(), *session.moon_orientation());
    const auto raan = targeter.parking_orbit_raan(epoch + m.flight_time_s, m.parking_inclination_rad);
    if (!raan) {
        throw std::runtime_error("Kein Parkorbit durch die Mondbahnebene: Bahnneigung unter der Deklination des Mondes");
    }
    // circular: the argument of latitude is the anomaly from the ascending node
    const orbit::KeplerianElements elements{constants::wgs84_semi_major_axis_m + m.parking_height_m, 0.0,
                                            m.parking_inclination_rad, *raan, 0.0, m.argument_of_latitude_rad};
    session.place_in_orbit(m.vehicle_id, orbit::state_from_elements(elements, session.gm_earth_m3ps2()), epoch);
}

mission::LunarOutboundParameters lunar_outbound_parameters(const config::LunarLandingMission& m,
                                                           const environment::GravityModel& gravity,
                                                           const ephemeris::Ephemeris& ephemeris, time::Epoch epoch) {
    mission::LunarOutboundParameters p;
    p.arrival_epoch = epoch + m.flight_time_s;
    p.earliest_departure_s = m.earliest_departure_s;
    p.mcc_after_tli_s = m.mcc_after_tli_s;
    if (m.lunar_orbit == "nrho") {
        p.orbit = mission::LunarOrbitKind::Nrho;
        const mission::NrhoSeed seed{{m.nrho_apolune_position, m.nrho_apolune_velocity}, m.nrho_revolutions,
                                     m.nrho_synodic_months};
        p.nrho = std::make_shared<mission::NrhoReference>(
            gravity, ephemeris, seed, p.arrival_epoch,
            mission::NrhoReferenceOptions{.revolutions = m.nrho_reference_revolutions});
    } else {
        p.orbit = mission::LunarOrbitKind::Llo;
        p.llo = {m.llo_altitude_m, m.llo_inclination_rad};
    }
    return p;
}

}  // namespace starsim::sim
