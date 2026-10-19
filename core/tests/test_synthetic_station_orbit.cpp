#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/orbit/oem.hpp"
#include "starsim/orbit/synthetic_station_orbit.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "starsim/time/utc_date_time.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {
const std::string fixture = std::string(STARSIM_FIXTURE_DIR) + "/iss_oem/ISS.OEM_J2K_EPH_2026-10-14T18-30-01.txt";
constexpr double gm = constants::earth_gm_m3ps2;
constexpr double re = constants::wgs84_semi_major_axis_m;
constexpr double j2 = constants::earth_j2;
constexpr double deg = constants::deg_to_rad;

double unwrap_delta(double a, double b) {  // a - b in (-pi, pi]
    return std::remainder(a - b, constants::two_pi);
}
}  // namespace

TEST_CASE("J2 RAAN rate of an ISS orbit is about -5 deg/day", "[iss][synthetic]") {
    orbit::StationMeanElements m;
    m.semi_major_axis_m = re + 415.0e3;
    m.inclination_rad = 51.64 * deg;
    const auto rates = orbit::j2_secular_rates(m, gm, j2, re);
    CHECK(rates.raan_radps * 86400.0 / deg == Catch::Approx(-5.0).margin(0.1));
    const double n = std::sqrt(gm / std::pow(m.semi_major_axis_m, 3));
    CHECK(rates.latitude_radps == Catch::Approx(n).epsilon(2e-3));  // J2 changes it by ~0.1 %
}

TEST_CASE("Mean elements of the OEM: ISS altitude and inclination", "[iss][synthetic]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto earth = spice::SpiceOrientation::earth();
    const auto oem = orbit::Oem::load(fixture, time);
    const auto mean = orbit::mean_elements_from_oem(oem, gm, earth);
    CHECK((mean.semi_major_axis_m - re) / 1e3 == Catch::Approx(419.9).margin(3.0));  // OEM event: 418.6 x 421.1 km heights
    CHECK(mean.inclination_rad / deg == Catch::Approx(51.64).margin(0.02));
    CHECK(mean.eccentricity < 0.003);
    // the true pole: GCRF inclination is ~0.13 deg off in 2026 (precession since J2000)
    const auto gcrf = orbit::elements_from_state(oem.row_state(0), gm);
    CHECK(std::abs(gcrf.inclination_rad - mean.inclination_rad) / deg > 0.05);
}

TEST_CASE("J2-advanced RAAN matches the OEM's own node drift over 14 days", "[iss][synthetic][validation]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto earth = spice::SpiceOrientation::earth();
    const auto oem = orbit::Oem::load(fixture, time);
    const auto mean = orbit::mean_elements_from_oem(oem, gm, earth);
    // the OEM's last revolution: mean elements of a filtered copy starting 14 days later
    const auto late_start = oem.row_epoch(0) + 14.0 * 86400.0;
    const auto late =
        orbit::mean_elements_from_oem(oem.filtered([&](size_t i) { return oem.row_epoch(i) >= late_start; }), gm, earth);
    const auto predicted = orbit::advance_mean_elements(mean, late.epoch, gm, j2, re);
    const double error_deg = unwrap_delta(predicted.raan_rad, late.raan_rad) / deg;
    if (std::getenv("STARSIM_VERBOSE")) std::printf("RAAN after 14 d: J2 %.4f deg off the OEM\n", error_deg);
    CHECK(std::abs(error_deg) < 0.2);  // [E] J2 only vs the operator's full field: ~70 deg of drift, 0.3 %
}

TEST_CASE("Synthetic state: the scenario's argument of latitude, the mean orbit", "[iss][synthetic]") {
    test::spice_session();
    const auto earth = spice::SpiceOrientation::earth();
    orbit::StationMeanElements m;
    m.epoch = time::Epoch::from_tdb_seconds_past_j2000(845.0e6);  // 2026-10
    m.semi_major_axis_m = re + 415.0e3;
    m.eccentricity = 0.0005;
    m.inclination_rad = 51.64 * deg;
    m.raan_rad = 120.0 * deg;
    m.argument_of_perigee_rad = 80.0 * deg;
    m.argument_of_latitude_rad = 10.0 * deg;
    const auto later = orbit::advance_mean_elements(m, m.epoch + 3600.0, gm, j2, re, 200.0 * deg);
    CHECK(later.argument_of_latitude_rad == Catch::Approx(200.0 * deg));
    const auto s = orbit::synthetic_station_state(later, gm, earth);
    const auto to_equator = transpose(orbit::true_equator_to_gcrf(earth, later.epoch));
    const auto el = orbit::elements_from_state({to_equator * s.position_m, to_equator * s.velocity_mps}, gm);
    CHECK(el.inclination_rad == Catch::Approx(m.inclination_rad).margin(1e-12));
    CHECK(unwrap_delta(el.raan_rad, later.raan_rad) == Catch::Approx(0.0).margin(1e-12));
    CHECK(unwrap_delta(el.argument_of_periapsis_rad + el.true_anomaly_rad, 200.0 * deg) ==
          Catch::Approx(0.0).margin(1e-9));
}

TEST_CASE("True equator frame: x is the GCRF x axis projected onto the true equator", "[iss][synthetic]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto earth = spice::SpiceOrientation::earth();
    // 2026 (pole ~530" from the GCRF pole) and J2000 (pole ~0.1" off: a node on the GCRF equator is undefined there)
    for (const auto& utc : {time::UtcDateTime{2026, 10, 14, 12, 0, 0.0}, time::UtcDateTime{2000, 1, 1, 12, 0, 0.0}}) {
        const auto epoch = time.from_utc(utc);
        const auto r = orbit::true_equator_to_gcrf(earth, epoch);
        const math::Vec3 pole = earth.fixed_to_gcrf(epoch).rotation * math::Vec3{0.0, 0.0, 1.0};
        const math::Vec3 x = math::column(r, 0);
        CHECK(std::abs(dot(x, pole)) < 1e-12);
        CHECK(x.x > 0.99999);
        CHECK(std::abs(x.y) < 1e-6);  // only the second-order product of the pole offsets
        CHECK(norm(math::column(r, 2) - pole) < 1e-12);
    }
}

TEST_CASE("Config reference RAAN is the fixture OEM's mean RAAN at reference_utc", "[iss][synthetic]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto earth = spice::SpiceOrientation::earth();
    const auto oem = orbit::Oem::load(fixture, time);
    const auto iss = config::load_vehicle(STARSIM_CONFIG_DIR, "iss");
    const auto& reference = *iss.stages.front().reference_orbit;
    const auto mean = orbit::mean_elements_from_oem(oem, gm, earth);
    const auto at = orbit::advance_mean_elements(mean, time.from_utc(*time::UtcDateTime::parse_iso(reference.reference_utc)),
                                                 gm, j2, re);
    if (std::getenv("STARSIM_VERBOSE")) std::printf("mean RAAN at reference_utc: %.4f deg\n", at.raan_rad / deg);
    CHECK(std::abs(unwrap_delta(at.raan_rad, reference.raan_rad)) / deg < 0.001);
    // close to the usual GCRF RAAN of the first row (69.36 deg): the frames differ by the pole tilt only
    const auto gcrf = orbit::elements_from_state(oem.row_state(0), gm);
    CHECK(std::abs(unwrap_delta(gcrf.raan_rad, reference.raan_rad)) / deg < 0.5);
}
