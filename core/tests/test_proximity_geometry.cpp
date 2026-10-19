// ISS i4: proximity geometry - waypoint paths, corridor, docking attitude, capture envelope.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "starsim/gnc/proximity_geometry.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/rendezvous_geometry.hpp"

using namespace starsim;
using math::Vec3;
constexpr double deg = constants::deg_to_rad;

TEST_CASE("Keep-out radius matches the mission geometry", "[iss][proximity]") {
    CHECK(gnc::ProximityParameters{}.keep_out_radius_m == mission::keep_out_sphere_radius_m);
}

TEST_CASE("Approach from the port axis", "[iss][proximity]") {
    CHECK(gnc::docking_approach({0.998, 0.0, -0.05}) == gnc::DockingApproach::VBar);
    CHECK(gnc::docking_approach({0.1, 0.0, -0.99}) == gnc::DockingApproach::RBar);
    CHECK_THROWS_AS(gnc::docking_approach({0.0, 1.0, 0.0}), std::invalid_argument);
}

TEST_CASE("V-bar path: Waypoint 0, 1, 2", "[iss][proximity]") {
    const gnc::ProximityParameters p;
    const auto path = gnc::proximity_path(gnc::DockingApproach::VBar, p);
    REQUIRE(path.size() == 3);
    CHECK(path[0].name == "Wegpunkt 0");
    CHECK(path[0].frame == gnc::WaypointFrame::LvlhFromCg);
    CHECK(path[0].position_m.z == 400.0);
    CHECK(path[1].name == "Wegpunkt 1");
    CHECK(path[1].frame == gnc::WaypointFrame::TargetPort);
    CHECK(path[1].position_m.x == 220.0);
    CHECK(path[2].position_m.x == 20.0);
    CHECK(path[2].on_axis);
    CHECK(path[1].hold_s == 300.0);
}

TEST_CASE("R-bar path flies around the front at 250 m", "[iss][proximity]") {
    const gnc::ProximityParameters p;
    const auto path = gnc::proximity_path(gnc::DockingApproach::RBar, p);
    // Waypoint 0, front (0 deg), 30 deg, 60 deg, Waypoint 1 (zenith axis), Waypoint 2
    REQUIRE(path.size() == 6);
    CHECK(path[1].name == "Umflug 1");
    CHECK(path[1].position_m.x == Catch::Approx(250.0));
    CHECK(path[2].position_m.x == Catch::Approx(250.0 * std::cos(30 * deg)));
    CHECK(path[2].position_m.z == Catch::Approx(-250.0 * std::sin(30 * deg)));  // up = -z
    for (size_t k = 1; k <= 3; ++k) CHECK(norm(path[k].position_m) == Catch::Approx(250.0));
    CHECK(path[4].name == "Wegpunkt 1");
    CHECK(path[4].frame == gnc::WaypointFrame::TargetPort);
}

TEST_CASE("Corridor: inside, hold, retreat, outside the sphere", "[iss][proximity]") {
    const gnc::ProximityParameters p;
    const double t10 = std::tan(10 * deg);
    CHECK(gnc::corridor_status({50.0, 0.9 * 50 * t10, 0.0}, 80.0, p) == gnc::CorridorStatus::Inside);
    CHECK(gnc::corridor_status({50.0, 1.5 * 50 * t10, 0.0}, 80.0, p) == gnc::CorridorStatus::Hold);
    CHECK(gnc::corridor_status({50.0, 0.0, -2.5 * 50 * t10}, 80.0, p) == gnc::CorridorStatus::Retreat);
    CHECK(gnc::corridor_status({0.5, 0.25, 0.0}, 40.0, p) == gnc::CorridorStatus::Inside);  // min half width 0.3 m
    CHECK(gnc::corridor_status({-0.2, 0.0, 0.0}, 40.0, p) == gnc::CorridorStatus::Retreat);
    CHECK(gnc::corridor_status({50.0, 40.0, 0.0}, 250.0, p) == gnc::CorridorStatus::OutsideKos);
}

TEST_CASE("Docking attitude opposes the axes and matches the ups", "[iss][proximity]") {
    // Dragon port: axis body +x, up body -z -> port frame columns (x, -y, -z) in the body
    const auto chaser = math::Mat3::from_columns({1, 0, 0}, {0, -1, 0}, {0, 0, -1});
    // an arbitrary target port frame
    const auto turn = math::Quat::from_axis_angle(normalized(Vec3{0.3, -0.5, 0.8}), 1.1);
    const auto target = turn.to_matrix();
    const auto q = gnc::docking_attitude_gcrf(chaser, target);
    const Vec3 x_c = q.rotate(chaser * Vec3{1, 0, 0}), z_c = q.rotate(chaser * Vec3{0, 0, 1});
    CHECK(dot(x_c, target * Vec3{1, 0, 0}) == Catch::Approx(-1.0).margin(1e-12));
    CHECK(dot(z_c, target * Vec3{0, 0, 1}) == Catch::Approx(1.0).margin(1e-12));
}

TEST_CASE("Capture envelope", "[iss][proximity]") {
    gnc::RelativePortState s;
    s.position_m = {0.0, 0.05, -0.03};
    s.velocity_mps = {-0.075, 0.01, 0.0};
    CHECK(gnc::capture_envelope_check(s).inside);
    auto fast = s;
    fast.velocity_mps.x = -0.12;
    CHECK_FALSE(gnc::capture_envelope_check(fast).inside);
    CHECK(gnc::capture_envelope_check(fast).reason.find("axial") != std::string::npos);
    auto slow = s;
    slow.velocity_mps.x = -0.03;
    CHECK_FALSE(gnc::capture_envelope_check(slow).inside);
    auto off = s;
    off.position_m.y = 0.12;
    CHECK_FALSE(gnc::capture_envelope_check(off).inside);
    auto tilted = s;
    tilted.misalignment_rad = 5 * deg;
    CHECK_FALSE(gnc::capture_envelope_check(tilted).inside);
    auto spinning = s;
    spinning.angular_rate_radps = {0.0, 0.3 * deg, 0.0};
    CHECK_FALSE(gnc::capture_envelope_check(spinning).inside);
}

TEST_CASE("Capture envelope: IDSS roll misalignment and roll rate", "[iss][proximity][docking]") {
    gnc::RelativePortState s;
    s.position_m = {0.0, 0.02, 0.0};
    s.velocity_mps = {-0.075, 0.0, 0.0};
    s.closing_speed_mps = 0.075;
    CHECK(gnc::capture_envelope_check(s).inside);
    s.clocking_rad = 4.5 * constants::deg_to_rad;
    CHECK_FALSE(gnc::capture_envelope_check(s).inside);
    CHECK(gnc::capture_envelope_check(s).reason.rfind("Rollwinkel zu groß", 0) == 0);
    s.clocking_rad = 0.0;
    s.angular_rate_radps = {0.25 * constants::deg_to_rad, 0.0, 0.0};  // roll rate about the docking axis
    CHECK(gnc::capture_envelope_check(s).reason.rfind("Rollrate zu groß", 0) == 0);
    s.angular_rate_radps = {0.0, 0.15 * constants::deg_to_rad, 0.0};  // pitch/yaw inside, roll 0
    CHECK(gnc::capture_envelope_check(s).inside);
}
