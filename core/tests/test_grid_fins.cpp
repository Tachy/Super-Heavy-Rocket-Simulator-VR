// Grid fins: aerodynamic model, vehicle dynamics (damping) and torque allocation.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/aero/grid_fin.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/gnc/attitude_controller.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/flight_body.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;
const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
const time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);

aero::GridFinTable table() { return aero::GridFinTable(config::load_csv_table(config_dir / "aero/grid_fin.csv")); }

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

// Super Heavy alone, falling engines first at 10 km over the equator (body x = local up = GCRF x).
std::unique_ptr<vehicle::FlightBody> falling_booster(bool with_fins, const Vec3& rate_body = {}) {
    auto cfg = config::load_vehicle(config_dir, "starship_block3");
    cfg.stages.erase(cfg.stages.begin() + 1);
    if (!with_fins) cfg.stages[0].grid_fins.azimuths_rad.clear();
    const vehicle::RigidBodyState state{{6378137.0 + 10000.0, 0.0, 0.0}, {-300.0, 0.0, 0.0}, {}, rate_body};
    return std::make_unique<vehicle::FlightBody>(std::make_unique<vehicle::Vehicle>(cfg, config_dir), state);
}

}  // namespace

TEST_CASE("Grid fin force: direction, magnitude, limit, rarefied flow", "[aero][gridfin]") {
    const auto t = table();
    const aero::GridFinGeometry fin{60.0, 0.0, 6.0, 10.0};  // azimuth 0: radial +y, tangential +z
    const aero::FinAirData air{0.4135, 299.5, 0.0};          // ~10 km
    const Vec3 tail_first{-200.0, 0.0, 0.0};
    const double q = 0.5 * air.density_kg_m3 * 200.0 * 200.0;
    const double mach = 200.0 / air.speed_of_sound_mps;

    SECTION("neutral fin in axial flow carries no load") {
        CHECK(norm(aero::grid_fin_force(fin, 0.0, tail_first, air, t).force_body_n) < 1e-6);
    }
    SECTION("deflection gives a normal force q S CN, along +t when moving tail first") {
        const double d = 10.0 * deg;
        const auto f = aero::grid_fin_force(fin, d, tail_first, air, t);
        const double expected = q * fin.area_m2 * t.normal_force_slope_per_rad(mach) * std::sin(d);
        CHECK(norm(f.force_body_n) == Catch::Approx(expected).epsilon(1e-9));
        CHECK(f.force_body_n.z > 0.99 * expected * std::cos(d));
        CHECK(f.application_point_m.y == Catch::Approx(6.0));
        // Nose first the same deflection pushes the other way.
        CHECK(aero::grid_fin_force(fin, d, -tail_first, air, t).force_body_n.z < 0.0);
    }
    SECTION("effectiveness matches the force derivative") {
        const double h = 1e-4;
        const double numeric = (aero::grid_fin_force(fin, h, tail_first, air, t).force_body_n.z -
                                aero::grid_fin_force(fin, -h, tail_first, air, t).force_body_n.z) /
                               (2.0 * h);
        CHECK(aero::grid_fin_effectiveness(fin, tail_first, air, t) == Catch::Approx(numeric).epsilon(1e-6));
    }
    SECTION("force saturates beyond the limit angle") {
        const double limit = t.limit_rad(mach);
        const double at_limit = norm(aero::grid_fin_force(fin, limit, tail_first, air, t).force_body_n);
        CHECK(norm(aero::grid_fin_force(fin, limit + 10.0 * deg, tail_first, air, t).force_body_n) ==
              Catch::Approx(at_limit).epsilon(1e-9));
    }
    SECTION("free-molecular flow: no fin force") {
        const aero::FinAirData rarefied{air.density_kg_m3, air.speed_of_sound_mps, 1.0};
        CHECK(norm(aero::grid_fin_force(fin, 10.0 * deg, tail_first, rarefied, t).force_body_n) == 0.0);
    }
}

TEST_CASE("Grid fins damp the rotation of a falling booster", "[vehicle][gridfin]") {
    World world;
    const Vec3 rate{0.0, 0.05, -0.03};
    auto with = falling_booster(true, rate);
    auto without = falling_booster(false, rate);
    with->step(world.env, t0, 0.005, {});
    without->step(world.env, t0, 0.005, {});
    const Vec3 extra = with->state().angular_velocity_body_radps - without->state().angular_velocity_body_radps;
    CHECK(dot(extra, rate) < 0.0);  // the fins oppose the rotation
}

TEST_CASE("Attitude controller allocates torque to the grid fins", "[gnc][gridfin]") {
    World world;
    auto body = falling_booster(true);
    body->step(world.env, t0, 0.005, {});  // air data for the controller
    const auto t = table();
    const double cg = body->vehicle().mass_properties().cg_station_m;
    auto fin_torque = [&](const std::vector<double>& deflections) {
        Vec3 torque;
        size_t i = 0;
        for (const auto& stage : body->vehicle().stages()) {
            for (const auto& fin : stage.grid_fins) {
                const double d = i < deflections.size() ? deflections[i] : 0.0;
                const auto f = aero::grid_fin_force(fin.geometry, d, body->air_data().air_velocity_body_mps,
                                                    body->air_data().air, t);
                torque += cross(f.application_point_m - Vec3{cg, 0.0, 0.0}, f.force_body_n);
                ++i;
            }
        }
        return torque;
    };
    // Linear range (a few degrees); larger demands meet the sin / limit nonlinearity of the fins.
    for (const Vec3& desired : {Vec3{0.0, 1e6, 0.0}, Vec3{0.0, 0.0, -1e6}, Vec3{1e5, 0.0, 0.0}}) {
        const auto out = gnc::AttitudeController::allocate(*body, 0.0, desired, true);
        REQUIRE(out.grid_fin_rad.size() == 3);
        const Vec3 achieved = fin_torque(out.grid_fin_rad) - fin_torque({});
        CHECK(norm(achieved - desired) < 0.05 * norm(desired));
        CHECK(norm(out.rcs_torque_fraction) < 0.05);  // nothing left for the cold-gas thrusters
    }
    // Without permission the fins stay neutral.
    CHECK(gnc::AttitudeController::allocate(*body, 0.0, Vec3{0.0, 5e6, 0.0}).grid_fin_rad.empty());
}
