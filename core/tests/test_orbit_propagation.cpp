#include <cmath>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/gravity.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using environment::PointMassGravity;
using environment::PointMassGravityConfig;

namespace {

constexpr double deg = constants::deg_to_rad;

// Earth orientation with the pole on the GCRF z-axis: isolates J2 theory from precession.
class AlignedEarth final : public frames::BodyOrientation {
public:
    frames::FixedToInertial fixed_to_gcrf(time::Epoch) const override {
        return {math::Mat3::identity(), {}};
    }
};

const time::Epoch epoch0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);  // 2026-08

PointMassGravityConfig two_body_config() {
    PointMassGravityConfig config;
    config.moon = config.sun = config.earth_j2 = false;
    return config;
}

}  // namespace

TEST_CASE("Two-body numerical propagation matches the analytic solution", "[orbit]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const AlignedEarth earth;
    const PointMassGravity gravity(ephemeris, earth, two_body_config());
    const double gm = ephemeris.gm_m3ps2(ephemeris::Body::Earth);

    const auto start = orbit::state_from_elements({6.798e6, 0.0005, 51.64 * deg, 30.0 * deg, 0.0, 0.0}, gm);
    const double one_day = 86400.0;
    const auto numeric = orbit::propagate_coast(gravity, start, epoch0, epoch0 + one_day);
    const auto analytic = orbit::propagate_two_body(start, gm, one_day);

    std::printf("[validation] two-body LEO 1 day: |dr| = %.3e m, %ld evaluations\n",
                norm(numeric.state.position_m - analytic.position_m), numeric.evaluations);
    CHECK(norm(numeric.state.position_m - analytic.position_m) < 0.05);  // 16 orbits, rtol 1e-12
    CHECK(orbit::specific_energy(numeric.state, gm) == Catch::Approx(orbit::specific_energy(start, gm)).epsilon(1e-9));
}

TEST_CASE("J2 nodal regression of an ISS orbit matches first-order theory", "[orbit][validation]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const AlignedEarth earth;
    PointMassGravityConfig config = two_body_config();
    config.earth_j2 = true;
    const PointMassGravity gravity(ephemeris, earth, config);
    const double gm = ephemeris.gm_m3ps2(ephemeris::Body::Earth);

    const orbit::KeplerianElements iss{6.798e6, 0.0005, 51.64 * deg, 30.0 * deg, 0.0, 0.0};
    const auto start = orbit::state_from_elements(iss, gm);

    // Least-squares slope of RAAN(t), sampled every 30 s over one day (averages short-period terms).
    double sum_t = 0, sum_raan = 0, sum_tt = 0, sum_traan = 0;
    int samples = 0;
    double next_sample = 0.0;
    const integrators::StepObserver<6> observer = [&](const integrators::Dop853<6>& solver) {
        while (next_sample <= solver.t()) {
            const auto el = orbit::elements_from_state(math::unpack(solver.dense_output(next_sample)), gm);
            sum_t += next_sample;
            sum_raan += el.raan_rad;
            sum_tt += next_sample * next_sample;
            sum_traan += next_sample * el.raan_rad;
            ++samples;
            next_sample += 30.0;
        }
    };
    integrators::integrate<6>(orbit::ballistic_dynamics(gravity, epoch0), 0.0, math::pack(start), 86400.0,
                              orbit::default_coast_options(), {}, observer);
    const double slope = (samples * sum_traan - sum_t * sum_raan) / (samples * sum_tt - sum_t * sum_t);

    const double p = iss.semi_major_axis_m * (1.0 - iss.eccentricity * iss.eccentricity);
    const double n = std::sqrt(gm / std::pow(iss.semi_major_axis_m, 3));
    const double r_ratio = environment::earth_j2_reference_radius_m / p;
    const double theory = -1.5 * n * constants::earth_j2 * r_ratio * r_ratio * std::cos(iss.inclination_rad);

    std::printf("[validation] J2 RAAN drift: simulated %.4f deg/day, theory %.4f deg/day\n",
                slope * 86400.0 * constants::rad_to_deg, theory * 86400.0 * constants::rad_to_deg);
    CHECK(slope == Catch::Approx(theory).epsilon(0.01));
}

TEST_CASE("Periapsis event fires at the Kepler time of periapsis passage", "[orbit]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const AlignedEarth earth;
    const PointMassGravity gravity(ephemeris, earth, two_body_config());
    const double gm = ephemeris.gm_m3ps2(ephemeris::Body::Earth);

    const orbit::KeplerianElements gto{2.4e7, 0.72, 28.5 * deg, 10.0 * deg, 180.0 * deg, 200.0 * deg};
    const auto start = orbit::state_from_elements(gto, gm);
    // Time from true anomaly 200 deg to periapsis via mean anomaly.
    const double e = gto.eccentricity;
    const double big_e = 2.0 * std::atan(std::sqrt((1 - e) / (1 + e)) * std::tan(gto.true_anomaly_rad / 2.0));
    const double mean_anomaly = std::fmod(big_e - e * std::sin(big_e) + constants::two_pi, constants::two_pi);
    const double expected = (constants::two_pi - mean_anomaly) / std::sqrt(gm / std::pow(gto.semi_major_axis_m, 3));

    const std::vector<integrators::Event<6>> events{
        {"periapsis",
         [](double, const integrators::State<6>& y) { return y[0] * y[3] + y[1] * y[4] + y[2] * y[5]; },
         integrators::Crossing::Rising}};
    const auto result = orbit::propagate_coast(gravity, start, epoch0, epoch0 + 2.0 * 86400.0, events);
    REQUIRE(result.event_index == 0);
    CHECK((result.epoch - epoch0) == Catch::Approx(expected).margin(1e-3));
}
