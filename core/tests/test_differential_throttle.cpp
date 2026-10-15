// Differential throttling of fixed, canted engines (lunar l4, user 2026-10-15): the HLS's twelve landing thrusters are
// throttled individually - their thrust lines miss the CG by ~9 m each, so equal throttles only cancel by symmetry and
// small differences steer the vehicle (an RCS of 1.5 kN thrusters could not; a real lander must regulate them anyway).

#include <cmath>
#include <memory>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/attitude_controller.hpp"
#include "starsim/vehicle/flight_body.hpp"

using namespace starsim;
using math::Vec3;

namespace {

vehicle::FlightBody hls_on_landing_thrusters(const std::string& group, double throttle) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls");
    for (auto& t : cfg.stages.front().tanks) {
        t.mass_kg = 400e3 * (t.contents == config::TankContents::Oxidizer ? 3.6 / 4.6 : 1.0 / 4.6);
    }
    vehicle::FlightBody body(std::make_unique<vehicle::Vehicle>(cfg, STARSIM_CONFIG_DIR),
                             {Vec3{1.8e6, 0.0, 0.0}, Vec3{}, math::Quat{}, {}});
    vehicle::VehicleCommand command;
    command.groups.push_back({group, true, throttle});
    for (int i = 0; i < 400; ++i) body.vehicle().update_engines(0.01, command, 0.0);  // start-up, throttle settles
    return body;
}

Vec3 trim_torque(const vehicle::FlightBody& body, const std::vector<double>& trim) {
    const double cg = body.vehicle().mass_properties().cg_station_m;
    Vec3 torque;
    size_t i = 0;
    for (const auto& stage : body.vehicle().stages()) {
        for (const auto& e : stage.engines) {
            if (i < trim.size()) torque += vehicle::throttle_torque_nm(e, cg) * trim[i];
            ++i;
        }
    }
    return torque;
}

}  // namespace

TEST_CASE("A canted landing thruster's thrust line misses the CG: ~1.3 MNm per thruster at full thrust",
          "[vehicle][differential_throttle]") {
    const auto body = hls_on_landing_thrusters("hls/landing", 0.7);
    const double cg = body.vehicle().mass_properties().cg_station_m;
    Vec3 sum;
    for (const auto& e : body.vehicle().stages().front().engines) {
        if (!e.landing_only) continue;
        const Vec3 t = vehicle::throttle_torque_nm(e, cg);
        CHECK(std::abs(t.x) < 1.0);                // inward cant: no roll
        CHECK(norm(t) > 0.8e6);
        sum += t;
    }
    CHECK(norm(sum) < 1.0);  // equal throttles cancel by symmetry
}

TEST_CASE("Pitch/yaw torque goes to the landing thrusters' throttles, roll and the excess to the RCS",
          "[gnc][differential_throttle]") {
    const auto body = hls_on_landing_thrusters("hls/landing", 0.7);
    const size_t engines = body.vehicle().stages().front().engines.size();
    SECTION("within the authority: no RCS for pitch and yaw, total thrust unchanged") {
        const Vec3 wanted{2.0e4, 1.0e6, -0.5e6};
        const auto out = gnc::AttitudeController::allocate(body, 0.0, wanted);
        REQUIRE(out.throttle_trim.size() == engines);
        const Vec3 achieved = trim_torque(body, out.throttle_trim);
        CHECK(achieved.y == Catch::Approx(wanted.y).epsilon(1e-3));
        CHECK(achieved.z == Catch::Approx(wanted.z).epsilon(1e-3));
        CHECK(out.rcs_torque_fraction.y == Catch::Approx(0.0).margin(1e-4));  // allocation regularisation
        CHECK(out.rcs_torque_fraction.z == Catch::Approx(0.0).margin(1e-4));
        CHECK(out.rcs_torque_fraction.x > 0.0);  // roll stays on the RCS
        double sum = 0.0;
        for (double t : out.throttle_trim) sum += t;
        CHECK(sum == Catch::Approx(0.0).margin(1e-9));
    }
    SECTION("beyond it: trims within the throttle range, the rest to the RCS") {
        const auto out = gnc::AttitudeController::allocate(body, 0.0, Vec3{0.0, 1.0e8, 0.0});
        REQUIRE(out.throttle_trim.size() == engines);
        for (double t : out.throttle_trim) CHECK(std::abs(t) <= 0.3 + 1e-9);  // 0.7 within [0.4, 1.0]
        CHECK(out.rcs_torque_fraction.y > 1.0);  // saturated (the vehicle clamps it)
    }
}

TEST_CASE("The vehicle applies the trims to its engines' throttles", "[vehicle][differential_throttle]") {
    auto body = hls_on_landing_thrusters("hls/landing", 0.7);
    vehicle::VehicleCommand command;
    command.groups.push_back({"hls/landing", true, 0.7});
    const size_t engines = body.vehicle().stages().front().engines.size();
    command.engine_throttle_trim.assign(engines, 0.0);
    size_t first = engines, second = engines;
    for (size_t i = 0; i < engines; ++i) {
        if (!body.vehicle().stages().front().engines[i].landing_only) continue;
        if (first == engines) first = i; else if (second == engines) second = i;
    }
    command.engine_throttle_trim[first] = 0.2;
    command.engine_throttle_trim[second] = -0.5;  // below the minimum: clamped
    for (int i = 0; i < 100; ++i) body.vehicle().update_engines(0.01, command, 0.0);
    const auto& e = body.vehicle().stages().front().engines;
    CHECK(e[first].throttle == Catch::Approx(0.9));
    CHECK(e[second].throttle == Catch::Approx(e[second].performance->config().throttle_min));
}

TEST_CASE("Gimballed engines keep their gimbal allocation (no trims)", "[gnc][differential_throttle]") {
    const auto body = hls_on_landing_thrusters("hls/sea_level_1", 0.7);
    const auto out = gnc::AttitudeController::allocate(body, 0.0, Vec3{0.0, 1.0e6, 0.0});
    CHECK(out.throttle_trim.empty());
    CHECK(out.gimbal.pitch_rad != 0.0);
}
