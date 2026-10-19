#pragma once
// ISS i3 tests: start states of a coelliptic rendezvous - the ISS from the OEM fixture and a Crew Dragon in its
// separation orbit `lead_deg` behind the ISS, in the plane the ISS plane will have at `plane_match_s` (a real launch
// aims there: the J2 node regression is ~0.58 deg/day faster at 205 km than at 415 km).

#include <cmath>
#include <filesystem>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/gravity.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/orbit/oem.hpp"
#include "starsim/orbit/synthetic_station_orbit.hpp"
#include "starsim/sim/station_start.hpp"
#include "starsim/spice/spice_adapters.hpp"

namespace starsim::rendezvous_test {

inline const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
inline const std::filesystem::path fixture_dir = std::filesystem::path(STARSIM_FIXTURE_DIR) / "iss_oem";

struct Start {
    sim::StationStart iss;
    math::StateVector dragon;
    time::Epoch epoch;
};

// Secular RAAN rate (J2) of a near-circular orbit.
inline double raan_rate(double a, double inclination) {
    orbit::StationMeanElements mean;
    mean.semi_major_axis_m = a;
    mean.inclination_rad = inclination;
    return orbit::j2_secular_rates(mean, constants::earth_gm_m3ps2, constants::earth_j2,
                                   environment::earth_j2_reference_radius_m).raan_radps;
}

inline Start start(const spice::SpiceTimeConverter& clock, const frames::BodyOrientation& earth, double lead_deg,
                   double plane_match_s) {
    Start s;
    s.epoch = clock.from_utc({2026, 10, 16, 0, 0, 0.0});
    const auto archive = orbit::OemArchive::scan(fixture_dir, clock);
    s.iss = sim::iss_start(config::load_vehicle(config_dir, "iss"), archive, clock, earth, s.epoch,
                           sim::IssOrbitSource::Oem);
    constexpr double gm = constants::earth_gm_m3ps2;
    const auto e_iss = orbit::elements_from_state(s.iss.state_gcrf, gm);
    orbit::KeplerianElements e;
    const double perigee = constants::wgs84_semi_major_axis_m + 200e3, apogee = constants::wgs84_semi_major_axis_m + 210e3;
    e.semi_major_axis_m = 0.5 * (perigee + apogee);  // [S] 200 x 210 km separation orbit (falcon9_crew_dragon)
    e.eccentricity = (apogee - perigee) / (apogee + perigee);
    e.inclination_rad = e_iss.inclination_rad;
    e.raan_rad = e_iss.raan_rad;
    e.argument_of_periapsis_rad = e_iss.argument_of_periapsis_rad + e_iss.true_anomaly_rad - lead_deg * constants::deg_to_rad;
    e.true_anomaly_rad = 0.0;  // at perigee
    s.dragon = orbit::state_from_elements(e, gm);
    // Plane bias: the Dragon's node regresses faster; shifted ahead so that both planes coincide after plane_match_s.
    const double bias = (raan_rate(e_iss.semi_major_axis_m, e_iss.inclination_rad) -
                         raan_rate(e.semi_major_axis_m, e.inclination_rad)) * plane_match_s;
    const auto turn = math::Quat::from_axis_angle({0.0, 0.0, 1.0}, bias);
    s.dragon = {turn.rotate(s.dragon.position_m), turn.rotate(s.dragon.velocity_mps)};
    return s;
}

}  // namespace starsim::rendezvous_test
