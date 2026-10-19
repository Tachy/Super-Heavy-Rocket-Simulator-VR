// ISS i2: GCRF kinematics of a docking port on a flying body (vehicle/docking_port).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <stdexcept>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/docking_port.hpp"
#include "starsim/vehicle/flight_body.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;
using math::Vec3;

namespace {
const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

std::unique_ptr<vehicle::FlightBody> body_of(const std::string& id, const vehicle::RigidBodyState& state) {
    return std::make_unique<vehicle::FlightBody>(
        std::make_unique<vehicle::Vehicle>(config::load_vehicle(config_dir, id), config_dir), state);
}
}  // namespace

TEST_CASE("Port position, velocity and axes of a turning Dragon", "[iss][rendezvous][port]") {
    // a quarter turn about GCRF z maps body (x, y, z) to GCRF (-y, x, z); spinning at 0.01 rad/s about body z
    const Vec3 position{7.0e6, 1.0e3, -2.0e3}, velocity{10.0, 7600.0, 5.0}, omega_body{0.0, 0.0, 0.01};
    auto dragon = body_of("crew_dragon", {position, velocity,
                                          math::Quat::from_axis_angle({0.0, 0.0, 1.0}, 0.5 * constants::pi),
                                          omega_body});
    const auto& v = dragon->vehicle();
    const Vec3 cg = v.mass_properties().cg_point_m();
    const double capsule_base = v.stages().back().base_station_m;  // 3.7 m: the trunk below
    CHECK(capsule_base == Catch::Approx(3.7));
    const Vec3 lever_body = Vec3{capsule_base + 4.0, 0.0, 0.0} - cg;
    const Vec3 lever_gcrf{-lever_body.y, lever_body.x, lever_body.z};

    const auto k = vehicle::port_kinematics(*dragon, "idss");
    CHECK(norm(k.position_gcrf_m - (position + lever_gcrf)) < 1e-9);
    const Vec3 omega_gcrf{0.0, 0.0, 0.01};
    CHECK(norm(k.velocity_gcrf_mps - (velocity + cross(omega_gcrf, lever_gcrf))) < 1e-12);
    CHECK(norm(k.angular_velocity_gcrf_radps - omega_gcrf) < 1e-15);
    CHECK(norm(math::column(k.axes_gcrf, 0) - Vec3{0.0, 1.0, 0.0}) < 1e-12);   // body +x
    CHECK(norm(math::column(k.axes_gcrf, 2) - Vec3{0.0, 0.0, -1.0}) < 1e-12);  // up = body -z
    CHECK(norm(math::column(k.axes_gcrf, 1) - Vec3{1.0, 0.0, 0.0}) < 1e-12);   // up x axis
    CHECK(math::determinant(k.axes_gcrf) == Catch::Approx(1.0));
}

TEST_CASE("A port behind a closed nose cone is covered", "[iss][rendezvous][port]") {
    auto dragon = body_of("crew_dragon", {{7.0e6, 0.0, 0.0}, {0.0, 7600.0, 0.0}, math::Quat{}, {}});
    CHECK(vehicle::port_kinematics(*dragon, "idss").covered);
    dragon->vehicle().set_nose_cone_open(true);
    CHECK_FALSE(vehicle::port_kinematics(*dragon, "idss").covered);
    auto iss = body_of("iss", {{6.8e6, 0.0, 0.0}, {0.0, 7660.0, 0.0}, math::Quat{}, {}});
    CHECK_FALSE(vehicle::port_kinematics(*iss, "ida2").covered);
    // IDA-3 sits on the station stage (base 0) at its configured position relative to the CG
    const auto k = vehicle::port_kinematics(*iss, "ida3");
    const Vec3 expected = Vec3{66.0, 0.0, -5.0} - iss->vehicle().mass_properties().cg_point_m();
    CHECK(norm(k.position_gcrf_m - Vec3{6.8e6, 0.0, 0.0} - expected) < 1e-9);
}

TEST_CASE("An unknown port id is a German error", "[iss][rendezvous][port]") {
    auto iss = body_of("iss", {{6.8e6, 0.0, 0.0}, {0.0, 7660.0, 0.0}, math::Quat{}, {}});
    CHECK_THROWS_AS(vehicle::port_kinematics(*iss, "ida1"), std::invalid_argument);
}
