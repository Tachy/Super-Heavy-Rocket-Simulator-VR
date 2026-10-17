// Entry attitude setpoint (angle of attack, bank) and flap trim / control allocation.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/gnc/entry_attitude.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using Catch::Approx;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;
constexpr double belly = 270.0 * deg;  // heat shield towards body -z
}  // namespace

TEST_CASE("Entry attitude: alpha 90 deg points the nose up with the belly into the flow", "[entry]") {
    const Vec3 v{7000.0, 0.0, 0.0}, up{0.0, 0.0, 1.0};
    const auto a = gnc::entry_attitude(v, up, 90.0 * deg, 0.0, belly);
    CHECK(a.axis_gcrf.z == Approx(1.0).margin(1e-12));
    const auto q = gnc::entry_attitude_quaternion(v, up, 90.0 * deg, 0.0, belly);
    const Vec3 belly_gcrf = q.rotate(Vec3{0.0, 0.0, -1.0});
    CHECK(belly_gcrf.x == Approx(1.0).margin(1e-12));  // the heat shield leads
    CHECK(q.rotate(Vec3{1.0, 0.0, 0.0}).z == Approx(1.0).margin(1e-12));
}

TEST_CASE("Entry attitude: alpha 60, bank 30 deg tilts the lift vector", "[entry]") {
    const Vec3 v{7000.0, 0.0, 0.0}, up{0.0, 0.0, 1.0};
    const auto q = gnc::entry_attitude_quaternion(v, up, 60.0 * deg, 30.0 * deg, belly);
    const Vec3 x = q.rotate(Vec3{1.0, 0.0, 0.0});
    CHECK(x.x == Approx(std::cos(60.0 * deg)).margin(1e-12));  // alpha between axis and velocity
    CHECK(std::abs(x.y) == Approx(std::sin(60.0 * deg) * std::sin(30.0 * deg)).margin(1e-12));
    CHECK(x.z == Approx(std::sin(60.0 * deg) * std::cos(30.0 * deg)).margin(1e-12));
    // the belly stays on the windward side, perpendicular to the axis
    const Vec3 b = q.rotate(Vec3{0.0, 0.0, -1.0});
    CHECK(dot(b, x) == Approx(0.0).margin(1e-12));
    CHECK(b.x == Approx(std::sin(60.0 * deg)).margin(1e-12));
    // the controller's roll reference is where body +y must point
    const auto a = gnc::entry_attitude(v, up, 60.0 * deg, 30.0 * deg, belly);
    const Vec3 y = q.rotate(Vec3{0.0, 1.0, 0.0});
    CHECK(dot(a.roll_reference_gcrf, y) == Approx(1.0).margin(1e-12));
}

// ---------------------------------------------------------------- flap trim and control

#include <cstdlib>
#include <iostream>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/gnc/attitude_controller.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
const time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);

class ZeroGravity final : public environment::GravityModel {
public:
    ephemeris::Body central_body() const override { return ephemeris::Body::Earth; }
    Vec3 acceleration_mps2(const Vec3&, time::Epoch) const override { return {}; }
};

class FixedEarth final : public frames::BodyOrientation {
public:
    frames::FixedToInertial fixed_to_gcrf(time::Epoch) const override { return {math::Mat3::identity(), {}}; }
};

struct World {
    ZeroGravity gravity;
    environment::Us76Atmosphere atmosphere;
    FixedEarth earth;
    vehicle::FlightEnvironment env{gravity, atmosphere, earth, nullptr, false};
};

// Ship alone with only its landing propellant, flying horizontally (+x) at `height_m` over GCRF +z, in the entry
// attitude for `alpha` (bank 0).
std::unique_ptr<vehicle::FlightBody> entry_ship(double height_m, double speed_mps, double alpha_rad, World& world) {
    auto cfg = config::load_vehicle(config_dir, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    double full = 0.0;
    for (const auto& tank : cfg.stages[0].tanks) full += tank.mass_kg;
    for (auto& tank : cfg.stages[0].tanks) tank.mass_kg *= cfg.stages[0].landing_propellant_kg / full;
    for (auto& f : cfg.stages[0].flaps) f.initial_deflection_rad = 0.5 * f.max_deflection_rad;
    const Vec3 v{speed_mps, 0.0, 0.0}, up{0.0, 0.0, 1.0};
    const vehicle::RigidBodyState state{{0.0, 0.0, 6356752.3 + height_m}, v,
                                        gnc::entry_attitude_quaternion(v, up, alpha_rad, 0.0, belly), {}};
    auto body = std::make_unique<vehicle::FlightBody>(std::make_unique<vehicle::Vehicle>(cfg, config_dir), state);
    body->step(world.env, t0, 1e-6, {});  // air data for the controller
    return body;
}

// Iterates the allocation for zero net torque with ideal actuators; returns the trimmed deflections and the
// remaining torque (what the RCS would have to give).
struct Trim {
    std::vector<double> deflection_rad;
    Vec3 remainder_fraction;  // RCS command
};

Trim trim(vehicle::FlightBody& body) {
    Trim t;
    for (int i = 0; i < 60; ++i) {
        const auto out = gnc::AttitudeController::allocate(body, 0.0, {}, false, true);
        auto& flaps = body.vehicle().stages().back().flaps;
        for (size_t k = 0; k < flaps.size(); ++k) flaps[k].deflection_rad = out.flap_rad.at(k);
        t.remainder_fraction = out.rcs_torque_fraction;
    }
    for (const auto& f : body.vehicle().stages().back().flaps) t.deflection_rad.push_back(f.deflection_rad);
    return t;
}

}  // namespace

TEST_CASE("The ship trims belly first with control margin on its flaps", "[entry][flap][trim]") {
    World world;
    struct Case { double height_m, speed_mps, alpha_deg; };
    const Case cases[] = {{65000.0, 6500.0, 55.0}, {65000.0, 6500.0, 60.0}, {65000.0, 6500.0, 70.0},
                          {5000.0, 128.0, 75.0}};
    for (const auto& c : cases) {
        auto body = entry_ship(c.height_m, c.speed_mps, c.alpha_deg * deg, world);
        const Trim t = trim(*body);
        if (std::getenv("STARSIM_VERBOSE")) {
            std::cout << "alpha " << c.alpha_deg << " h " << c.height_m << ": flaps [deg]";
            for (double d : t.deflection_rad) std::cout << " " << d / deg;
            std::cout << "  rcs " << t.remainder_fraction.x << " " << t.remainder_fraction.y << " "
                      << t.remainder_fraction.z << "\n";
        }
        CAPTURE(c.alpha_deg, c.height_m);
        CHECK(norm(t.remainder_fraction) < 0.01);  // trimmed by the flaps alone
        for (double d : t.deflection_rad) {
            CHECK(d >= 15.0 * deg);
            CHECK(d <= 75.0 * deg);
        }
    }
}

TEST_CASE("Flaps produce commanded pitch, roll and yaw torques; saturation goes to RCS", "[entry][flap]") {
    World world;
    auto body = entry_ship(65000.0, 6500.0, 60.0 * deg, world);
    trim(*body);
    const auto trimmed = body->vehicle().stages().back().flaps;
    const auto out0 = gnc::AttitudeController::allocate(*body, 0.0, {}, false, true);
    // Authority scale: one aft flap's moment change over 10 deg.
    const double scale = 1e6;
    for (int axis = 0; axis < 3; ++axis) {
        Vec3 tau;
        (axis == 0 ? tau.x : axis == 1 ? tau.y : tau.z) = scale;
        const auto out = gnc::AttitudeController::allocate(*body, 0.0, tau, false, true);
        CAPTURE(axis);
        bool changed = false;
        for (size_t k = 0; k < trimmed.size(); ++k) {
            changed |= std::abs(out.flap_rad[k] - out0.flap_rad[k]) > 1e-4;
            UNSCOPED_INFO("flap " << k << " at " << trimmed[k].deflection_rad / deg << " deg, step "
                                  << (out.flap_rad[k] - out0.flap_rad[k]) / deg << " deg");
        }
        CHECK(norm(out.rcs_torque_fraction) < 0.01);
        CHECK(changed);
    }
    const auto saturated = gnc::AttitudeController::allocate(*body, 0.0, Vec3{0.0, 1e10, 0.0}, false, true);
    for (size_t k = 0; k < trimmed.size(); ++k) {
        CHECK(saturated.flap_rad[k] >= 0.0);
        CHECK(saturated.flap_rad[k] <= trimmed[k].max_deflection_rad);
    }
    CHECK(std::abs(saturated.rcs_torque_fraction.y) > 0.0);
}
