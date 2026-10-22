// Crew escape (docs/superpowers/specs/2026-10-22-crew-escape-design.md), Task 1: the crew-escape config and the seat /
// parachutist bodies.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "starsim/config/crew_escape_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/flight_body.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;
using math::Vec3;

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

// A copy of the config tree's crew_escape file with one text replaced (unique directory per test).
std::filesystem::path patched(const std::string& name, const std::string& from, const std::string& to) {
    std::ifstream in(config_dir / "crew_escape" / "starship_crew.toml");
    std::string text{std::istreambuf_iterator<char>(in), {}};
    const auto at = text.find(from);
    REQUIRE(at != std::string::npos);
    text.replace(at, from.size(), to);
    const auto dir = std::filesystem::temp_directory_path() / ("starsim_crew_escape_" + name);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "crew_escape");
    std::filesystem::copy(config_dir / "vehicles", dir / "vehicles", std::filesystem::copy_options::recursive);
    std::ofstream(dir / "crew_escape" / "starship_crew.toml") << text;
    return dir;
}

class CentralGravity final : public environment::GravityModel {
public:
    ephemeris::Body central_body() const override { return ephemeris::Body::Earth; }
    Vec3 acceleration_mps2(const Vec3& r, time::Epoch) const override {
        return r * (-constants::earth_gm_m3ps2 / std::pow(norm(r), 3));
    }
};

class FixedEarth final : public frames::BodyOrientation {
public:
    frames::FixedToInertial fixed_to_gcrf(time::Epoch) const override { return {math::Mat3::identity(), {}}; }
};

}  // namespace

TEST_CASE("Crew escape config loads", "[crew_escape][config]") {
    const auto c = config::load_crew_escape(config_dir, "starship_crew");
    CHECK(c.id == "starship_crew");
    CHECK(c.crew_count == 4);
    CHECK(c.seats.size() == 4);
    CHECK(norm(c.rail_direction_body) == Catch::Approx(1.0).margin(1e-12));
    CHECK(c.envelope_max_eas_mps == Catch::Approx(308.7).margin(0.5));
    CHECK(c.stagger_s == 0.4);
    CHECK(c.plume_half_angle_rad == Catch::Approx(15.0 * constants::deg_to_rad));
    CHECK(config::thrust_at(c.rocket, -1.0) == 0.0);
    CHECK(config::thrust_at(c.rocket, 0.35) > 0.0);
    CHECK(config::thrust_at(c.rocket, 1.0) == 0.0);
    // the tube and the sustainer sum to the constant rocket-catapult thrust
    CHECK(config::thrust_at(c.catapult, 0.175) + config::thrust_at(c.rocket, 0.175) == Catch::Approx(22241.0));
    CHECK_NOTHROW(config::load_vehicle(config_dir, c.seat_vehicle_id));
    CHECK_NOTHROW(config::load_vehicle(config_dir, c.parachutist_vehicle_id));
}

TEST_CASE("Crew escape config refuses bad values", "[crew_escape][config]") {
    using Catch::Matchers::ContainsSubstring;
    CHECK_THROWS_WITH(config::load_crew_escape(patched("nan", "stagger_s = 0.4", "stagger_s = nan"), "starship_crew"),
                      ContainsSubstring("stagger_s"));
    CHECK_THROWS_WITH(config::load_crew_escape(patched("negative", "stagger_s = 0.4", "stagger_s = -0.4"), "starship_crew"),
                      ContainsSubstring("stagger_s"));
    CHECK_THROWS_WITH(
        config::load_crew_escape(patched("vehicle", "seat_vehicle = \"ejection_seat\"", "seat_vehicle = \"nope\""),
                                 "starship_crew"),
        ContainsSubstring("nope"));
    CHECK_THROWS_WITH(
        config::load_crew_escape(patched("order", "[0.17, 0.0], [0.18, 22241.0]", "[0.18, 0.0], [0.17, 22241.0]"),
                                 "starship_crew"),
        ContainsSubstring("rocket"));
    CHECK_THROWS_WITH(
        config::load_crew_escape(patched("seats", "[[seat]]\nposition_m = [43.0, 0.0, 3.5]\n[[seat]]\nposition_m = [44.2, "
                                                  "0.0, 3.5]\n[[seat]]\nposition_m = [45.4, 0.0, 3.5]\n[[seat]]\n"
                                                  "position_m = [46.6, 0.0, 3.5]\n",
                                         ""),
                                 "starship_crew"),
        ContainsSubstring("seat"));
}

TEST_CASE("Seat bodies reach their canopy sink rates", "[crew_escape][config][validation]") {
    CentralGravity gravity;
    environment::Us76Atmosphere atmosphere;
    FixedEarth earth;
    const vehicle::FlightEnvironment env{gravity, atmosphere, earth, nullptr, false};
    const Vec3 r = frames::geodetic_to_fixed({0.0, 0.0, 1500.0});
    vehicle::RigidBodyState s;
    s.position_gcrf_m = r;
    s.velocity_gcrf_mps = math::normalized(r) * -50.0;
    s.attitude = math::Quat{1.0, 0.0, 0.0, 0.0};  // body +x up: hanging under the canopy
    vehicle::FlightBody body(
        std::make_unique<vehicle::Vehicle>(config::load_vehicle(config_dir, "parachutist"), STARSIM_CONFIG_DIR), s);
    body.set_ground_height(0.0);
    REQUIRE(body.vehicle().deploy_parachutes("main"));
    time::Epoch t;
    const auto height = [&] { return frames::fixed_to_geodetic(body.state().position_gcrf_m).height_m; };
    while (height() > 50.0 && !body.impacted() && !body.resting()) {
        body.step(env, t, 0.005, {});
        t += 0.005;
    }
    const double sink = -dot(body.state().velocity_gcrf_mps, math::normalized(body.state().position_gcrf_m));
    std::printf("[validation] parachutist main: %.2f m/s at %.0f m (published ~6-6.7 m/s)\n", sink, height());
    CHECK_FALSE(body.vehicle().parachute_torn());
    CHECK(sink == Catch::Approx(6.5).margin(0.5));  // [S] GQ 5000 ~22 ft/s, aeroconical ~20 ft/s
}
