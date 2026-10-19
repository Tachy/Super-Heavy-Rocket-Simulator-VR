// sim::SessionRunner (l6 Task 3): the simulation thread.
#include <catch2/catch_test_macros.hpp>
#include <atomic>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <vector>

#include "starsim/sim/session_runner.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using namespace std::chrono_literals;

namespace {
struct RunnerFixture {
    bool kernels = (test::spice_session(), true);
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    std::atomic<int> observed = 0;
    std::atomic<double> last_epoch_s = -1.0e300;
    std::atomic<bool> epochs_monotone = true;
    sim::SessionRunner runner{std::make_unique<sim::FlightSession>(ephemeris, earth, STARSIM_CONFIG_DIR),
                              [this](const sim::FlightSession&, time::Epoch e) {
                                  const double s = e.tdb_seconds_past_j2000();
                                  if (s < last_epoch_s) epochs_monotone = false;
                                  last_epoch_s = s;
                                  ++observed;
                              }};
};
const auto t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);  // 2026-08
const sim::LaunchSite starbase{0.4537, -1.6956, 10.0};             // 25.997 deg N, 97.154 deg W
}  // namespace

TEST_CASE("runner advances a session to the target", "[session_runner]") {
    RunnerFixture f;
    const auto id = f.runner.post([](sim::FlightSession& s) {
        s.launch_from_pad("starship_block3", starbase, 0.0, t0);
    });
    f.runner.set_target(t0 + 5.0);
    REQUIRE(f.runner.wait_idle(30.0));
    CHECK(f.runner.result(id).done);
    CHECK_FALSE(f.runner.result(id).threw);
    // advance_to stops within one tick of the target (never past it)
    CHECK(f.last_epoch_s <= (t0 + 5.0).tdb_seconds_past_j2000());
    CHECK(f.last_epoch_s > (t0 + 5.0 - sim::FlightSession::tick_s).tdb_seconds_past_j2000());
    CHECK(f.epochs_monotone);
    CHECK(f.observed >= 2);
}

TEST_CASE("runner commands keep their order and report exceptions", "[session_runner]") {
    RunnerFixture f;
    std::vector<int> order;
    const auto a = f.runner.post([&](sim::FlightSession&) { order.push_back(1); });
    const auto b = f.runner.post([](sim::FlightSession&) { throw std::runtime_error("Absicht"); });
    const auto c = f.runner.post([&](sim::FlightSession&) { order.push_back(3); });
    REQUIRE(f.runner.wait_idle(10.0));
    CHECK(order == std::vector<int>{1, 3});
    CHECK(f.runner.result(b).threw);
    CHECK(f.runner.result(b).error == "Absicht");
    const auto finished = f.runner.take_finished();
    REQUIRE(finished.size() == 3);
    CHECK(finished[0].first == a);
    CHECK(finished[2].first == c);
}

TEST_CASE("runner keeps only the latest input and reports a long command as busy", "[session_runner]") {
    RunnerFixture f;
    std::atomic<int> latest_runs = 0, latest_value = 0;
    f.runner.post([](sim::FlightSession&) { std::this_thread::sleep_for(400ms); });  // a "plan"
    std::this_thread::sleep_for(100ms);
    CHECK(f.runner.busy_s() > 0.05);
    for (int i = 1; i <= 50; ++i) {
        f.runner.set_latest([&, i](sim::FlightSession&) {
            ++latest_runs;
            latest_value = i;
        });
    }
    f.runner.post([](sim::FlightSession& s) { s.launch_from_pad("starship_block3", starbase, 0.0, t0); });
    f.runner.set_target(t0 + 1.0);
    REQUIRE(f.runner.wait_idle(30.0));
    CHECK(latest_runs == 1);
    CHECK(latest_value == 50);
    CHECK(f.runner.busy_s() == 0.0);
}

TEST_CASE("runner destroyed during a long command", "[session_runner]") {
    std::atomic<bool> second_ran = false;
    const auto begin = std::chrono::steady_clock::now();
    {
        RunnerFixture f;
        f.runner.post([](sim::FlightSession&) { std::this_thread::sleep_for(300ms); });
        f.runner.post([&](sim::FlightSession&) { second_ran = true; });
        std::this_thread::sleep_for(50ms);
    }  // destructor: waits for the first, drops the second
    CHECK_FALSE(second_ran);
    CHECK(std::chrono::steady_clock::now() - begin < 2s);
}

TEST_CASE("runner survives a throwing input and keeps running commands", "[session_runner]") {
    RunnerFixture f;
    f.runner.set_latest([](sim::FlightSession&) { throw std::runtime_error("Absicht"); });
    REQUIRE(f.runner.wait_idle(10.0));
    CHECK(f.runner.last_error() == "Absicht");
    std::atomic<bool> ran = false;
    const auto id = f.runner.post([&](sim::FlightSession&) { ran = true; });
    REQUIRE(f.runner.wait_idle(10.0));
    CHECK(ran);
    CHECK(f.runner.result(id).done);
    CHECK_FALSE(f.runner.result(id).threw);
}
