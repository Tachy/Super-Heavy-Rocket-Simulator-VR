// CSPICE from two threads (l6 Task 2): the simulation thread and rendering (StarsimCore) both call the adapters.
#include <catch2/catch_test_macros.hpp>
#include <atomic>
#include <exception>
#include <thread>
#include <vector>

#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

TEST_CASE("CSPICE adapters are safe from two threads", "[spice][threads]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    constexpr int n = 4000;
    std::vector<math::Vec3> moon_ref(n);
    for (int i = 0; i < n; ++i) {
        moon_ref[i] = ephemeris.state_gcrf(ephemeris::Body::Moon, time::Epoch::from_tdb_seconds_past_j2000(i * 60.0))
                          .position_m;
    }
    std::atomic<int> mismatches = 0, spurious_errors = 0;
    std::thread worker([&] {  // valid calls only: must never throw nor differ
        for (int i = 0; i < n; ++i) {
            try {
                const auto p = ephemeris.state_gcrf(ephemeris::Body::Moon,
                                                    time::Epoch::from_tdb_seconds_past_j2000(i * 60.0)).position_m;
                if (p.x != moon_ref[i].x || p.y != moon_ref[i].y || p.z != moon_ref[i].z) ++mismatches;
                (void)earth.fixed_to_gcrf(time::Epoch::from_tdb_seconds_past_j2000(i * 60.0));
            } catch (...) {
                ++spurious_errors;
            }
        }
    });
    int expected_errors = 0;
    for (int i = 0; i < n; ++i) {  // every 10th call fails on purpose (far outside de440s: 1849-2150)
        try {
            if (i % 10 == 0) {
                (void)ephemeris.state_gcrf(ephemeris::Body::Moon, time::Epoch::from_tdb_seconds_past_j2000(1.0e11));
            } else {
                (void)earth.fixed_to_gcrf(time::Epoch::from_tdb_seconds_past_j2000(i * 30.0));
            }
        } catch (const std::exception&) {
            ++expected_errors;
        }
    }
    worker.join();
    CHECK(mismatches == 0);
    CHECK(spurious_errors == 0);
    CHECK(expected_errors == n / 10);
}
