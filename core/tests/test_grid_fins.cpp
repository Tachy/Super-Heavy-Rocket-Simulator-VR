// Grid fins: aerodynamic model, vehicle dynamics (damping) and torque allocation.

#include <cmath>
#include <string>
#include <tuple>

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
    world.env.grid_fin_drag_couple = false;  // the damping alone: the drag couple pitches whatever the rate
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

TEST_CASE("Grid fins add their axial drag to the tail-first drag curve", "[aero][grid_fins]") {
    // Lattice fins are blunt in the axial direction: their drag on the frontal lattice area is of the order of the
    // body's own and must slow the falling booster (recorded boosters are transonic ~30 s before the landing burn).
    const auto cfg = config::load_vehicle(config_dir, "starship_block3");
    const vehicle::Vehicle v(cfg, config_dir);
    const auto& stage = cfg.stages.front();
    const aero::AxialDragCurve body(config::load_csv_table(config_dir / stage.aero.descent_drag_table));
    const aero::AxialDragCurve ascent(config::load_csv_table(config_dir / stage.aero.ascent_drag_table));
    const double reference_area_m2 = constants::pi * 0.25 * v.diameter_m() * v.diameter_m();
    const double fins_area_m2 = static_cast<double>(stage.grid_fins.azimuths_rad.size()) * stage.grid_fins.area_m2;
    for (const double mach : {0.5, 1.0, 2.0, 4.0}) {
        CAPTURE(mach);
        const double ca = table().axial_coefficient(mach);
        CHECK(ca > 0.5);
        const double expected = body.cd(mach, false) + fins_area_m2 * ca / reference_area_m2;
        CHECK(v.aero().drag_curve(true).cd(mach, false) == Catch::Approx(expected).epsilon(1e-9));
        CHECK(v.aero().drag_curve(true).cd(mach, true) ==
              Catch::Approx(body.cd(mach, true) + fins_area_m2 * ca / reference_area_m2).epsilon(1e-9));
        CHECK(v.aero().drag_curve(false).cd(mach, false) == Catch::Approx(ascent.cd(mach, false)));  // ascent table as is
    }
}

TEST_CASE("Starship's fins carry the drawn area and the descent total stays as identified", "[aero][grid_fins]") {
    const auto cfg = config::load_vehicle(config_dir, "starship_block3");
    const vehicle::Vehicle v(cfg, config_dir);
    const auto& fins = cfg.stages.front().grid_fins;
    CHECK(fins.azimuths_rad.size() == 3);
    CHECK(fins.area_m2 == 12.63);
    CHECK(fins.radius_m == 7.19);
    // The total the flights fixed: the identified body table of 2026-10-13 + the fins' share with 3 x 13.5 m^2.
    const aero::AxialDragCurve identified_body(
        config::CsvTable{{"mach", "cd_axial_power_on", "cd_axial_power_off"},
                         {{0.0, 0.900, 0.900},
                          {0.6, 0.950, 0.950},
                          {0.8, 1.000, 1.000},
                          {0.95, 1.150, 1.150},
                          {1.1, 1.450, 1.450},
                          {1.5, 1.380, 1.380},
                          {2.0, 1.420, 1.420},
                          {3.0, 1.750, 1.750},
                          {5.0, 1.880, 1.880},
                          {10.0, 1.880, 1.880},
                          {25.0, 1.880, 1.880}}});
    const double reference_area_m2 = constants::pi * 0.25 * v.diameter_m() * v.diameter_m();
    for (double mach = 0.0; mach <= 25.0; mach += mach < 6.0 ? 0.05 : 1.0) {
        CAPTURE(mach);
        const double total = identified_body.cd(mach, false) + 3.0 * 13.5 * table().axial_coefficient(mach) / reference_area_m2;
        CHECK(std::abs(v.aero().drag_curve(true).cd(mach, false) - total) < 1e-3);
        CHECK(std::abs(v.aero().drag_curve(true).cd(mach, true) - total) < 1e-3);
    }
}

TEST_CASE("The unbalanced fin's axial drag acts at its centre of pressure", "[aero][grid_fins]") {
    const auto t = table();
    const aero::GridFinGeometry fin{60.0, 90.0 * deg, 7.19, 12.63};
    const aero::FinAirData air{0.35, 300.0, 0.0};
    SECTION("the fin's share of the hull's axial force") {
        const double q = 0.5 * air.density_kg_m3 * 776.0 * 776.0;
        const double mach = 776.0 / air.speed_of_sound_mps;
        const double expected = q * fin.area_m2 * t.axial_coefficient(mach);
        CHECK(aero::grid_fin_axial_force_n(fin, {-776.0, 0.0, 0.0}, air, t) == Catch::Approx(expected));
        CHECK(aero::grid_fin_axial_force_n(fin, {776.0, 0.0, 0.0}, air, t) == Catch::Approx(-expected));
        // As the hull's axial term: cos^2 of the angle of attack, none in free-molecular flow.
        CHECK(aero::grid_fin_axial_force_n(fin, {-776.0 * std::cos(0.3), 776.0 * std::sin(0.3), 0.0}, air, t) ==
              Catch::Approx(expected * std::cos(0.3) * std::cos(0.3)));
        CHECK(aero::grid_fin_axial_force_n(fin, {-776.0, 0.0, 0.0}, {air.density_kg_m3, air.speed_of_sound_mps, 1.0}, t) == 0.0);
    }
    SECTION("a falling Super Heavy pitches about +y; the flag switches it; the force stays") {
        const auto rate_of = [](bool couple, const std::string& id) {
            World world;
            world.env.grid_fin_drag_couple = couple;
            auto cfg = config::load_vehicle(config_dir, id);
            cfg.stages.resize(1);
            const vehicle::RigidBodyState state{{6378137.0 + 10000.0, 0.0, 0.0}, {-600.0, 0.0, 0.0}, {}, {}};
            vehicle::FlightBody body(std::make_unique<vehicle::Vehicle>(cfg, config_dir), state);
            const double dt = 0.005;
            body.step(world.env, t0, dt, {});
            return std::make_tuple(body.state().angular_velocity_body_radps, body.state().velocity_gcrf_mps,
                                   body.vehicle().mass_properties().inertia_kg_m2);
        };
        const auto [w_on, v_on, inertia] = rate_of(true, "starship_block3");
        const auto [w_off, v_off, inertia_off] = rate_of(false, "starship_block3");
        (void)inertia_off;
        // Fin azimuth 90 stands at body +z, the drag on it along +x: moment r x F about +y.
        const environment::Us76Atmosphere atmosphere;
        const auto a = atmosphere.at(10000.0);
        const aero::GridFinGeometry booster_fin{64.0, 90.0 * deg, 7.19, 12.63};
        const double force =
            aero::grid_fin_axial_force_n(booster_fin, {-600.0, 0.0, 0.0}, {a.density_kg_m3, a.speed_of_sound_mps, 0.0}, t);
        CHECK(force > 0.0);
        CHECK((w_on.y - w_off.y) == Catch::Approx(7.19 * force / inertia.y * 0.005).epsilon(0.02));
        CHECK(std::abs(w_on.z - w_off.z) < 1e-9);
        CHECK(std::abs(w_on.x - w_off.x) < 1e-9);
        // A couple: the translation of the first tick is the same (the drag stays in the axial curve).
        CHECK(norm(v_on - v_off) < 1e-6);
    }
    SECTION("a symmetric layout has no couple, exactly: Falcon 9 is bit-identical with the flag on and off") {
        World on, off;
        off.env.grid_fin_drag_couple = false;
        auto cfg = config::load_vehicle(config_dir, "falcon9_block5");
        cfg.stages.resize(1);
        const vehicle::RigidBodyState state{{6378137.0 + 10000.0, 0.0, 0.0}, {-600.0, 0.0, 0.0}, {}, {0.01, 0.02, -0.01}};
        vehicle::FlightBody a(std::make_unique<vehicle::Vehicle>(cfg, config_dir), state);
        vehicle::FlightBody b(std::make_unique<vehicle::Vehicle>(cfg, config_dir), state);
        for (int i = 0; i < 200; ++i) {
            a.step(on.env, t0 + 0.005 * i, 0.005, {});
            b.step(off.env, t0 + 0.005 * i, 0.005, {});
        }
        CHECK(a.state().angular_velocity_body_radps.y == b.state().angular_velocity_body_radps.y);
        CHECK(a.state().angular_velocity_body_radps.z == b.state().angular_velocity_body_radps.z);
        CHECK(a.state().velocity_gcrf_mps.x == b.state().velocity_gcrf_mps.x);
    }
}

TEST_CASE("A drag curve plus an increment interpolates both tables", "[aero]") {
    config::CsvTable t;
    t.columns = {"mach", "cd_axial_power_on", "cd_axial_power_off"};
    t.rows = {{0.0, 1.0, 1.1}, {2.0, 1.2, 1.3}};
    const aero::AxialDragCurve base(t);
    const auto sum = base.plus({0.0, 1.0, 3.0}, {0.5, 1.0, 0.0});
    CHECK(sum.cd(1.0, false) == Catch::Approx(1.2 + 1.0));
    CHECK(sum.cd(0.5, true) == Catch::Approx(1.05 + 0.75));
    CHECK(sum.cd(2.0, false) == Catch::Approx(1.3 + 0.5));
    CHECK(sum.cd(3.0, false) == Catch::Approx(1.3 + 0.0));
}
