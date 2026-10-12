// Model validation against a real spacecraft: Artemis I (Orion) reconstructed trajectory from
// JPL Horizons. Free-flight arcs are propagated with our gravity model and compared to the
// reconstruction. Unmodelled: solar radiation pressure (~1e-8 m/s^2 -> ~10 m/day), attitude-control
// thruster residuals, higher gravity harmonics. Orion trimmed its trajectory every ~12-24 h; those
// manoeuvres show up as km-level jumps in the scan (STARSIM_VERBOSE=1), so only clean arcs are asserted.
// Manoeuvre-free 6 h arcs agree to 4.3-4.9 m throughout, consistent with unmodelled SRP.

#include <cstdio>
#include <cstdlib>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/gravity.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {

struct Sample {
    time::Epoch epoch;
    math::StateVector state;
};

std::vector<Sample> load_arc(const char* fixture) {
    std::vector<Sample> samples;
    for (const auto& row : test::read_fixture(fixture)) {
        samples.push_back({test::epoch_from_jd_tdb(row[0]),
                           {{row[1] * 1e3, row[2] * 1e3, row[3] * 1e3}, {row[4] * 1e3, row[5] * 1e3, row[6] * 1e3}}});
    }
    return samples;
}

// Propagates from samples[first] through `hours` hourly samples; returns the max position error.
double arc_error_m(const environment::GravityModel& gravity, const std::vector<Sample>& samples, size_t first,
                   size_t hours) {
    math::StateVector state = samples[first].state;
    double worst = 0.0;
    for (size_t k = first + 1; k <= first + hours && k < samples.size(); ++k) {
        state = orbit::propagate_coast(gravity, state, samples[k - 1].epoch, samples[k].epoch).state;
        worst = std::max(worst, norm(state.position_m - samples[k].state.position_m));
    }
    return worst;
}

void print_scan(const environment::GravityModel& gravity, const std::vector<Sample>& samples, size_t hours,
                const char* label) {
    if (!std::getenv("STARSIM_VERBOSE")) return;
    const spice::SpiceTimeConverter time;
    for (size_t first = 0; first + hours < samples.size(); first += 3) {
        std::printf("  %s %s +%zuh: max |dr| = %10.1f m\n", label, time.to_utc(samples[first].epoch).to_iso_string().c_str(),
                    hours, arc_error_m(gravity, samples, first, hours));
    }
}

}  // namespace

TEST_CASE("Artemis I outbound coast (Earth-centred)", "[orbit][validation]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const environment::PointMassGravity gravity(ephemeris, earth);
    const auto samples = load_arc("horizons_artemis1_outbound_geocentric.csv");
    print_scan(gravity, samples, 6, "outbound");

    // Longest manoeuvre-free arc (found by the 6 h scan): 2022-11-18 07:00 - 2022-11-19 01:00 TDB.
    // Bound 50 m: unmodelled SRP (~1.9e-8 m/s^2) gives ~39 m after 18 h.
    const double error = arc_error_m(gravity, samples, 39, 18);
    std::printf("[validation] Artemis I outbound 18 h free flight: max |dr| = %.1f m\n", error);
    CHECK(error < 50.0);

    if (std::getenv("STARSIM_VERBOSE")) {
        // Model ablation on the clean arc: which force matters how much?
        const size_t first = 39;
        for (int variant = 0; variant < 5; ++variant) {
            environment::PointMassGravityConfig config;
            const char* name = "full";
            if (variant == 1) { config.sun = false; name = "no sun"; }
            if (variant == 2) { config.moon = false; name = "no moon"; }
            if (variant == 3) { config.earth_j2 = false; name = "no J2"; }
            if (variant == 4) { config.sun = config.moon = config.earth_j2 = false; name = "two-body"; }
            const environment::PointMassGravity g(ephemeris, earth, config);
            std::printf("  ablation %-8s 1h %8.1f m  6h %9.1f m  18h %10.1f m\n", name,
                        arc_error_m(g, samples, first, 1), arc_error_m(g, samples, first, 6),
                        arc_error_m(g, samples, first, 18));
        }
    }
}

TEST_CASE("Artemis I distant retrograde orbit (Moon-centred)", "[orbit][validation]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    environment::PointMassGravityConfig config;
    config.central_body = ephemeris::Body::Moon;
    const environment::PointMassGravity gravity(ephemeris, earth, config);
    const auto samples = load_arc("horizons_artemis1_dro_selenocentric.csv");
    print_scan(gravity, samples, 6, "DRO");

    // Longest manoeuvre-free arc: 2022-11-29 18:00 - 2022-11-30 15:00 TDB.
    const double error = arc_error_m(gravity, samples, 90, 21);
    std::printf("[validation] Artemis I DRO 21 h free flight (Moon-centred): max |dr| = %.1f m\n", error);
    // Error grows as t^2 (constant unmodelled acceleration ~1.9e-8 m/s^2 = Orion's solar radiation
    // pressure): 4.9 m after 6 h -> ~60 m after 21 h. Bound: 0.5 * 3e-8 m/s^2 * (21 h)^2 = 86 m.
    CHECK(error < 86.0);
}
