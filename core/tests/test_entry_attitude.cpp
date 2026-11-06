// Entry attitude setpoint (angle of attack, bank) and flap trim / control allocation.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "starsim/config/propellant_loading.hpp"
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
#include "starsim/gnc/entry_residue_limit.hpp"
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
std::unique_ptr<vehicle::FlightBody> entry_ship(double height_m, double speed_mps, double alpha_rad, World& world,
                                                double propellant_kg = -1.0) {
    auto cfg = config::load_vehicle(config_dir, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    config::fill_tanks(cfg.stages[0], propellant_kg >= 0.0 ? propellant_kg : cfg.stages[0].landing_propellant_kg);
    for (auto& f : cfg.stages[0].flaps) f.initial_deflection_rad = 0.5 * f.max_deflection_rad;
    const Vec3 v{speed_mps, 0.0, 0.0}, up{0.0, 0.0, 1.0};
    const vehicle::RigidBodyState state{{0.0, 0.0, 6356752.3 + height_m}, v,
                                        gnc::entry_attitude_quaternion(v, up, alpha_rad, 0.0, belly), {}};
    auto body = std::make_unique<vehicle::FlightBody>(std::make_unique<vehicle::Vehicle>(cfg, config_dir), state);
    // air data for the controller. Long enough at low speed: this start lies on the pole, where the geodetic height of a
    // displacement of 1e-4 m is not a number.
    body->step(world.env, t0, speed_mps < 1000.0 ? 1e-3 : 1e-6, {});
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

// The cases come from the vehicle's own flight envelope, not from fixed angles. Hypersonic (65 km, 6500 m/s, Mach ~21):
// the alpha schedule (ship_entry_alpha.csv, ~54 deg there) and 6 deg above it ([E]: an off-schedule excursion the
// hypersonic controller can see; the former 60 deg case). The earlier 70 deg point at Mach 21 was a stale premise: the
// guidance never commands it (steering about the schedule, terminal_alpha_authority_deg, acts only in the terminal phase
// below Mach 5), and since the mesh centre-of-pressure table (47904b61) and the 177 t mass / CG (357c3b67) the forward
// pair needs 79.7 deg to hold it. Terminal phase (Mach 4.5, 40 km): between the corridor points 45 km / 2500 m/s and
// 35 km / 900 m/s of the residue-limit case below, at the schedule plus the steering authority from config.
// Low and slow: 5 km, 128 m/s, 75 deg.
TEST_CASE("The ship trims belly first with control margin on its flaps", "[entry][flap][trim]") {
    World world;
    const auto cfg = config::load_vehicle(config_dir, "starship_block3");
    const auto& ship = cfg.stages.back();
    REQUIRE(ship.entry.has_value());
    const auto schedule = gnc::load_alpha_schedule(config_dir, ship.aero.entry_alpha_table);
    const double authority_rad = ship.entry->terminal_alpha_authority_rad;
    constexpr double excursion_rad = 6.0 * deg;  // [E]
    const auto at_mach = [&](double height_m, double mach) {
        return std::pair{mach * world.atmosphere.at(height_m).speed_of_sound_mps, schedule.at(mach)};
    };
    struct Case { double height_m, speed_mps, alpha_deg; };
    std::vector<Case> cases;
    {
        const double mach = 6500.0 / world.atmosphere.at(65000.0).speed_of_sound_mps;
        const double nominal = schedule.at(mach);
        cases.push_back({65000.0, 6500.0, nominal / deg});
        cases.push_back({65000.0, 6500.0, (nominal + excursion_rad) / deg});
    }
    {
        const auto [speed, nominal] = at_mach(40000.0, 4.5);
        cases.push_back({40000.0, speed, (nominal + authority_rad) / deg});
    }
    cases.push_back({5000.0, 128.0, 75.0});
    for (const auto& c : cases) {
        auto body = entry_ship(c.height_m, c.speed_mps, c.alpha_deg * deg, world);
        const Trim t = trim(*body);
        if (std::getenv("STARSIM_VERBOSE")) {
            std::cout << "alpha " << c.alpha_deg << " h " << c.height_m << " v " << c.speed_mps << ": flaps [deg]";
            for (double d : t.deflection_rad) std::cout << " " << d / deg;
            std::cout << "  rcs " << t.remainder_fraction.x << " " << t.remainder_fraction.y << " "
                      << t.remainder_fraction.z << "\n";
        }
        CAPTURE(c.alpha_deg, c.height_m, c.speed_mps);
        CHECK(norm(t.remainder_fraction) < 0.01);  // trimmed by the flaps alone
        for (double d : t.deflection_rad) {
            CHECK(d >= 15.0 * deg);
            CHECK(d <= 75.0 * deg);
        }
    }
}

// Diagnostic (planner plan Task 12): the flap trim along the alpha schedule over the propellant on board (headers full,
// the rest main-tank residue): deflections and the RCS remainder per point.
TEST_CASE("Diagnostic: flap trim along the schedule over the propellant", "[.diag_trim]") {
    World world;
    struct Point { double height_m, speed_mps, alpha_deg; };
    const Point points[] = {{65000.0, 6500.0, 55.0}, {35000.0, 900.0, 60.0}, {27000.0, 560.0, 66.0},
                            {25000.0, 445.0, 70.0},  {20000.0, 330.0, 75.0}, {15000.0, 240.0, 80.0}};
    for (double propellant_t : {35.0, 40.0, 45.0, 50.0, 55.0, 62.0}) {
        for (const auto& p : points) {
            auto body = entry_ship(p.height_m, p.speed_mps, p.alpha_deg * deg, world, propellant_t * 1000.0);
            const Trim t = trim(*body);
            std::cout << "prop " << propellant_t << " t, cg " << body->vehicle().mass_properties().cg_station_m << " m, h "
                      << p.height_m / 1000.0 << " km v " << p.speed_mps << " alpha " << p.alpha_deg << ": flaps";
            for (double d : t.deflection_rad) std::cout << " " << d / deg;
            std::cout << "  rcs " << norm(t.remainder_fraction) << "\n";
        }
    }
}

// Planner plan Task 12b: the main-tank residue the ship may carry into the entry. The limit comes from the entry aero
// table's trim along the schedule; the flown control allocation must agree: at the limit the flaps alone trim every
// point of the schedule (the RCS stays free for the bank), with the campaign's heaviest residue they do not.
TEST_CASE("The entry residue limit: the flaps alone trim the schedule up to it", "[entry][flap][trim]") {
    World world;
    const auto ship_with = [&](double propellant_kg) {
        auto body = entry_ship(65000.0, 6500.0, 55.0 * deg, world, propellant_kg);
        return body;
    };
    const auto schedule_of = [&](const vehicle::Vehicle& v) {
        return gnc::load_alpha_schedule(config_dir, v.stages().front().config.aero.entry_alpha_table);
    };
    {
        auto headers_only = ship_with(-1.0);  // full headers, empty main tanks: nothing to vent
        const auto limit = gnc::entry_residue_limit(headers_only->vehicle(), schedule_of(headers_only->vehicle()));
        CHECK(limit.main_propellant_kg == Approx(0.0).margin(1.0));
        CHECK(limit.excess_kg() == 0.0);
    }
    auto heavy = ship_with(62000.0);  // the campaign's heaviest: 35 t headers + 27 t main
    const auto schedule = schedule_of(heavy->vehicle());
    const double headers_kg = heavy->vehicle().propellant_kg() - heavy->vehicle().main_propellant_kg();
    const auto limit = gnc::entry_residue_limit(heavy->vehicle(), schedule);
    std::cout << "entry residue limit: main " << limit.main_propellant_kg / 1000.0 << " t, trimmable "
              << limit.max_main_propellant_kg / 1000.0 << " t, excess " << limit.excess_kg() / 1000.0 << " t\n";
    REQUIRE(limit.main_propellant_kg == Approx(62000.0 - headers_kg).margin(1.0));
    CHECK(limit.excess_kg() > 0.0);
    CHECK(limit.max_main_propellant_kg > 0.0);

    // the table's verdict on both sides of the limit
    auto at_limit = heavy->vehicle().copy_upper(0);
    gnc::draw_main_tanks(*at_limit, limit.excess_kg());
    CHECK(at_limit->main_propellant_kg() == Approx(limit.max_main_propellant_kg).margin(1.0));
    CHECK(at_limit->propellant_kg() - at_limit->main_propellant_kg() == Approx(headers_kg).margin(1e-6));  // headers kept
    CHECK(gnc::EntryAeroTable(*at_limit, schedule).schedule_trim_margin_rad() > 0.0);
    auto beyond = heavy->vehicle().copy_upper(0);
    gnc::draw_main_tanks(*beyond, limit.excess_kg() - 1000.0);
    CHECK(gnc::EntryAeroTable(*beyond, schedule).schedule_trim_margin_rad() <= 0.0);

    // the flown allocation along the schedule (heights and speeds of a belly-first descent, the schedule's alpha)
    struct Point { double height_m, speed_mps; };
    const Point points[] = {{65000.0, 6500.0}, {45000.0, 2500.0}, {35000.0, 900.0}, {30000.0, 700.0},
                            {27000.0, 560.0},  {25000.0, 445.0},  {22000.0, 380.0}, {20000.0, 330.0},
                            {18000.0, 300.0},  {15000.0, 240.0},  {12000.0, 200.0}};
    double worst_at_limit = 0.0, worst_heavy = 0.0;
    for (const auto& p : points) {
        const double alpha = schedule.at(p.speed_mps / world.atmosphere.at(p.height_m).speed_of_sound_mps);
        auto light = entry_ship(p.height_m, p.speed_mps, alpha, world, headers_kg + limit.max_main_propellant_kg);
        const Trim t = trim(*light);
        worst_at_limit = std::max(worst_at_limit, norm(t.remainder_fraction));
        auto full = entry_ship(p.height_m, p.speed_mps, alpha, world, 62000.0);
        worst_heavy = std::max(worst_heavy, norm(trim(*full).remainder_fraction));
        if (std::getenv("STARSIM_VERBOSE")) {
            std::cout << "  h " << p.height_m / 1000.0 << " km alpha " << alpha / deg << ": flaps";
            for (double d : t.deflection_rad) std::cout << " " << d / deg;
            std::cout << "  rcs " << norm(t.remainder_fraction) << "\n";
        }
    }
    std::cout << "  RCS share of the trim: at the limit " << worst_at_limit << ", 62 t " << worst_heavy << "\n";
    CHECK(worst_at_limit < 0.01);  // the flaps alone (as the trim margin case above)
    CHECK(worst_heavy > 0.01);
}

TEST_CASE("Flaps produce commanded pitch, roll and yaw torques; saturation goes to RCS", "[entry][flap]") {
    // Demand per alpha, measured on the true moments (Mach 21, 65 km, 35 t): on the schedule (52-55 deg here) the flaps deliver 3e5 N m
    // on all axes, RCS < 1e-4. At 60 deg, 7 deg above the schedule, the forward pair trims at 77 deg, 13 deg from its stop: roll ends at
    // 1.4e5 N m and yaw between 2e5 and 3e5, so 1e5 N m there. The hull's mesh centre of pressure sets this, not the allocator.
    const auto [alpha_deg, scale] = GENERATE(table<double, double>({{55.0, 3e5}, {60.0, 1e5}}));
    CAPTURE(alpha_deg, scale);
    World world;
    auto body = entry_ship(65000.0, 6500.0, alpha_deg * deg, world);
    trim(*body);
    const auto trimmed = body->vehicle().stages().back().flaps;
    const auto out0 = gnc::AttitudeController::allocate(*body, 0.0, {}, false, true);
    // Authority scale: one aft flap's moment change over a few deg. The flaps cannot deliver 1e6 N m of yaw here
    // (iterated on the true moments, l5 Task 10: an aft flap ends folded, ~2 x the RCS left) - the earlier one-call
    // check only confirmed the linear model.
    auto& flaps = body->vehicle().stages().back().flaps;
    for (int axis = 0; axis < 3; ++axis) {
        Vec3 tau;
        (axis == 0 ? tau.x : axis == 1 ? tau.y : tau.z) = scale;
        // The allocation steps within what the rate-limited flaps reach in a few control periods; the flaps follow
        // their commands (here at once) until the moment is delivered.
        flaps = trimmed;
        gnc::ControlOutput out;
        for (int call = 0; call < 10; ++call) {
            out = gnc::AttitudeController::allocate(*body, 0.0, tau, false, true);
            for (size_t k = 0; k < flaps.size(); ++k) flaps[k].deflection_rad = out.flap_rad[k];
        }
        flaps = trimmed;
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

// One flap's true moment about the CG in the body's air (the allocator's own model is not reachable from here).
Vec3 true_flap_moment_nm(const vehicle::FlightBody& body, const vehicle::Flap& flap, double deflection_rad) {
    const auto& stage = body.vehicle().stages().back();
    const auto& air = body.air_data();
    const Vec3 arm = aero::flap_centroid(flap.geometry, deflection_rad) -
                     Vec3{body.vehicle().mass_properties().cg_station_m, 0.0, 0.0};
    return cross(arm, aero::flap_force(flap.geometry, deflection_rad, air.air_velocity_body_mps, air.air, *stage.flap_table).force_body_n);
}

// Vehicle physics Task 6d. The guidance campaign's case 8 (61.6 t) departed at Mach 0.67, alpha 80.5 deg: a yaw demand of
// about 1e6 N m drove a forward flap down to its edge-on deflection, where it has neither force nor slope. With it gone
// roll and yaw can no longer be told apart (aft pair alone +-1.3e5 N m of yaw at zero roll, all four +-8.4e5), roll ran to
// -0.06 rad/s and the sideslip it made gave a hull moment beyond the flaps. Same air here, landing propellant: trim at
// 54 / 54 / 44 / 44 deg, and the same demand took the flap from 44 to 16 deg, slope 125 of 1.5e6 N m/rad.
// Planner plan Task 4: the 212 t ship (177 t dry, CG 24.98 m) trims here at 63.6 / 63.6 / 43.5 / 43.5 deg; its flaps'
// yaw reach at unchanged roll and pitch is far smaller than the 290 t ship's 8.4e5 N m: the true moments on a 1 deg grid
// with the forward flaps above their authority floor (36.4 deg) give +-1.8e5 N m (roll / pitch within 5e3 N m; +-2.4e5
// within 2e4), and the allocation delivers 2.0e5 settled (RCS fraction 1e-4) but not 2.5e5. The former within-reach demand
// 3e5 lies beyond it (it left an RCS fraction 1.6): the within-reach demand is 0.7 of the reach (measured below), 3e5 joins the beyond-reach
// ones (floor and no limit cycle checked).
TEST_CASE("A yaw demand does not drive a flap into the range where it has no authority", "[entry][flap]") {
    World world;
    auto body = entry_ship(13000.0, 200.0, 80.5 * deg, world);
    trim(*body);
    auto& flaps = body->vehicle().stages().back().flaps;
    const auto trimmed = flaps;
    const auto slope_of = [&](const vehicle::Flap& f, double d, double span) {
        return norm(true_flap_moment_nm(*body, f, d + 0.5 * span) - true_flap_moment_nm(*body, f, d - 0.5 * span)) / span;
    };
    const auto peak_slope = [&](const vehicle::Flap& f) {
        double peak = 0.0;
        for (double d = 2.5 * deg; d < f.max_deflection_rad; d += 5.0 * deg) peak = std::max(peak, slope_of(f, d, 5.0 * deg));
        return peak;
    };
    constexpr double follow = 0.5 * deg;  // per call, a rate-limited actuator; the allocation's box is 5 deg
    // Fix round 1: the flaps' own yaw reach at unchanged roll / pitch from this ship's config, not an offline number: a
    // demand far beyond flaps + RCS settles with the RCS yaw at its maximum and roll / pitch held (the allocation cuts
    // the yaw, see the next case); the true flap + hull yaw moment there is the flaps' reach (2.15e5 N m for the 212 t
    // ship). Within reach: 0.7 of it ([E] margin for the linearised steps; 1.5e5 N m).
    const double flap_reach_nm = [&] {
        flaps = trimmed;
        for (int call = 0; call < 200; ++call) {
            const auto o = gnc::AttitudeController::allocate(*body, 0.0, Vec3{0.0, 0.0, 1e6}, false, true);
            for (size_t k = 0; k < flaps.size(); ++k) {
                flaps[k].deflection_rad += std::clamp(o.flap_rad[k] - flaps[k].deflection_rad, -follow, follow);
            }
        }
        Vec3 aero = gnc::AttitudeController::modelled_hull_moment_nm(*body);
        for (const auto& f : flaps) aero += true_flap_moment_nm(*body, f, f.deflection_rad);
        return aero.z;
    }();
    const double within_reach_nm = 0.7 * flap_reach_nm;
    CAPTURE(flap_reach_nm);
    REQUIRE(flap_reach_nm > 0.0);
    for (const double yaw_nm : {1e6, -1e6, 3e5, -3e5, within_reach_nm, -within_reach_nm}) {
        flaps = trimmed;
        CAPTURE(yaw_nm);
        double last_move = 0.0;
        std::vector<double> lowest(flaps.size(), 2.0), highest(flaps.size(), -1.0);
        gnc::ControlOutput out;
        for (int call = 0; call < 200; ++call) {
            out = gnc::AttitudeController::allocate(*body, 0.0, Vec3{0.0, 0.0, yaw_nm}, false, true);
            last_move = 0.0;
            for (size_t k = 0; k < flaps.size(); ++k) {
                const double move = std::clamp(out.flap_rad[k] - flaps[k].deflection_rad, -follow, follow);
                flaps[k].deflection_rad += move;
                last_move = std::max(last_move, std::abs(move));
                if (call >= 180) {
                    lowest[k] = std::min(lowest[k], flaps[k].deflection_rad);
                    highest[k] = std::max(highest[k], flaps[k].deflection_rad);
                }
            }
        }
        for (size_t k = 0; k < flaps.size(); ++k) {
            const double d = flaps[k].deflection_rad, slope = slope_of(flaps[k], d, 1.0 * deg), peak = peak_slope(flaps[k]);
            if (std::getenv("STARSIM_VERBOSE")) {
                std::cout << "yaw " << yaw_nm << " flap " << k << " trim " << trimmed[k].deflection_rad / deg << " -> " << d / deg
                          << " deg, slope " << slope << " of peak " << peak << ", excursion " << (highest[k] - lowest[k]) / deg
                          << ", rcs " << norm(out.rcs_torque_fraction) << "\n";
            }
            CAPTURE(k, d / deg, slope, peak);
            CHECK(slope >= 0.2 * peak);  // the allocator's floor is a quarter, on a 5 deg grid
            CHECK(highest[k] - lowest[k] <= 1.01 * follow);  // at most the follow's own step, no wider cycle
        }
        if (std::abs(yaw_nm) <= within_reach_nm) {  // within the flaps' reach: delivered and settled
            CHECK(norm(out.rcs_torque_fraction) < 0.01);
            CHECK(last_move < 0.01 * deg);
        }
    }
}

// Fix round 1 of planner plan Task 4 (controller ruling): beyond the flaps' yaw reach the least-squares remainder spilled
// into roll (RCS roll fraction 1.5 at 3e5 N m of yaw). Flaps first, then the RCS, then the cut - on the yaw axis only:
// the net roll stays the demand (zero), the RCS fractions stay within +-1, the cut is reported (anti-windup) and the
// yaw delivered is at least the RCS's own yaw torque from the config.
TEST_CASE("A yaw demand beyond flaps and RCS is cut in yaw, not traded into roll", "[entry][flap]") {
    World world;
    auto body = entry_ship(13000.0, 200.0, 80.5 * deg, world);
    trim(*body);
    auto& flaps = body->vehicle().stages().back().flaps;
    const auto trimmed = flaps;
    const Vec3 rcs_max = body->vehicle().max_rcs_torque_nm();
    for (const double yaw_nm : {1e6, -1e6, 6e5, -6e5}) {
        flaps = trimmed;
        CAPTURE(yaw_nm);
        gnc::ControlOutput out;
        for (int call = 0; call < 200; ++call) {
            out = gnc::AttitudeController::allocate(*body, 0.0, Vec3{0.0, 0.0, yaw_nm}, false, true);
            for (size_t k = 0; k < flaps.size(); ++k) {
                flaps[k].deflection_rad += std::clamp(out.flap_rad[k] - flaps[k].deflection_rad, -0.5 * deg, 0.5 * deg);
            }
        }
        // diagnostics: the net moment at the settled flaps (true flap + hull moments) plus the RCS
        Vec3 aero = gnc::AttitudeController::modelled_hull_moment_nm(*body);
        for (const auto& f : flaps) aero += true_flap_moment_nm(*body, f, f.deflection_rad);
        const Vec3 net{aero.x + out.rcs_torque_fraction.x * rcs_max.x, aero.y + out.rcs_torque_fraction.y * rcs_max.y,
                       aero.z + out.rcs_torque_fraction.z * rcs_max.z};
        if (std::getenv("STARSIM_VERBOSE")) {
            std::cout << "yaw " << yaw_nm << ": rcs " << out.rcs_torque_fraction.x << " " << out.rcs_torque_fraction.y << " "
                      << out.rcs_torque_fraction.z << ", unmet " << out.unmet_torque_nm.x << " " << out.unmet_torque_nm.y
                      << " " << out.unmet_torque_nm.z << ", net " << net.x << " " << net.y << " " << net.z << "\n";
        }
        CAPTURE(out.rcs_torque_fraction.x, out.rcs_torque_fraction.y, out.rcs_torque_fraction.z, out.unmet_torque_nm.z);
        for (const double f : {out.rcs_torque_fraction.x, out.rcs_torque_fraction.y, out.rcs_torque_fraction.z}) {
            CHECK(std::abs(f) <= 1.0);
        }
        // roll and pitch: their demand (zero) - nothing cut there, the RCS not spent on a share of the yaw
        CHECK(out.unmet_torque_nm.x == 0.0);
        CHECK(out.unmet_torque_nm.y == 0.0);
        CHECK(std::abs(out.rcs_torque_fraction.x) < 0.02);
        CHECK(std::abs(out.rcs_torque_fraction.y) < 0.02);
        // yaw: the RCS at its reach from config, the rest cut on the yaw axis (same sign as the demand)
        CHECK(out.rcs_torque_fraction.z == Approx(yaw_nm > 0.0 ? 1.0 : -1.0));
        CHECK(out.unmet_torque_nm.z * yaw_nm > 0.0);
        CHECK(std::abs(out.unmet_torque_nm.z) < std::abs(yaw_nm) - rcs_max.z);
    }
}

TEST_CASE("Flaps pinned at their stops come back into the range", "[entry][flap][trim]") {
    // l5 Task 10: the flap force goes with sin^2 of the incidence, its derivative vanishes at both stops - flaps left
    // at 90/0/0/90 by thin air never came back and their roll moment went to the RCS.
    World world;
    auto body = entry_ship(65000.0, 6500.0, 60.0 * deg, world);
    auto& flaps = body->vehicle().stages().back().flaps;
    REQUIRE(flaps.size() == 4);
    for (size_t k = 0; k < flaps.size(); ++k) flaps[k].deflection_rad = (k == 0 || k == 3) ? flaps[k].max_deflection_rad : 0.0;
    const Trim t = trim(*body);
    CHECK(norm(t.remainder_fraction) < 0.01);
    for (double d : t.deflection_rad) {
        CHECK(d >= 15.0 * deg);
        CHECK(d <= 75.0 * deg);
    }
}

#include "starsim/gnc/entry_aero_table.hpp"

TEST_CASE("The entry aero table's flap trim cancels the pitch moment", "[entry][flap][trim]") {
    World world;
    auto body = entry_ship(65000.0, 6500.0, 60.0 * deg, world);
    const gnc::AlphaSchedule schedule{{1.0, 30.0}, {60.0 * deg, 60.0 * deg}};
    const gnc::EntryAeroTable table(body->vehicle(), schedule);
    const auto rest = table.trim_flap_rad(norm(body->air_data().air_velocity_body_mps) / body->air_data().air.speed_of_sound_mps);
    auto& flaps = body->vehicle().stages().back().flaps;
    REQUIRE(rest.size() == flaps.size());
    for (size_t k = 0; k < flaps.size(); ++k) flaps[k].deflection_rad = rest[k];
    // the flaps holding, the RCS would cancel the modelled moment: ~none left
    const auto out = gnc::AttitudeController::allocate(*body, 0.0, {}, false, false);
    CAPTURE(rest[0] / deg, rest[1] / deg, rest[2] / deg, rest[3] / deg);
    CHECK(std::abs(out.rcs_torque_fraction.y) < 0.05);
    CHECK(std::abs(out.rcs_torque_fraction.x) < 0.01);  // symmetric pairs: no roll, no yaw
    CHECK(std::abs(out.rcs_torque_fraction.z) < 0.01);
}

TEST_CASE("Flaps without authority rest in the given position", "[entry][flap][trim]") {
    World world;
    gnc::AttitudeController::Gains gains;
    gains.flaps = true;
    gains.natural_frequency_rad_s = 0.5;
    gains.flap_rest_dynamic_pressure_pa = 200.0;
    const gnc::AttitudeController controller(gains);
    gnc::AttitudeController::Feedforward ff;
    ff.flap_rest_rad = {30.0 * deg, 35.0 * deg, 40.0 * deg, 45.0 * deg};
    for (const double height : {110000.0, 65000.0}) {
        auto body = entry_ship(height, 7000.0, 60.0 * deg, world);
        auto& flaps = body->vehicle().stages().back().flaps;
        for (size_t k = 0; k < flaps.size(); ++k) flaps[k].deflection_rad = (k == 0 || k == 3) ? flaps[k].max_deflection_rad : 0.0;
        const Vec3 axis = body->state().attitude.rotate(Vec3{1.0, 0.0, 0.0});
        const Vec3 roll = body->state().attitude.rotate(Vec3{0.0, 1.0, 0.0});
        const auto out = controller.steer(*body, 0.0, axis, roll, &ff);
        const double q = 0.5 * body->air_data().air.density_kg_m3 * std::pow(norm(body->air_data().air_velocity_body_mps), 2);
        CAPTURE(height, q);
        REQUIRE(out.flap_rad.size() == ff.flap_rest_rad.size());
        double difference = 0.0;
        for (size_t k = 0; k < out.flap_rad.size(); ++k) difference += std::abs(out.flap_rad[k] - ff.flap_rest_rad[k]);
        if (q < 200.0) {
            CHECK(difference == Approx(0.0).margin(1e-12));
        } else {
            CHECK(difference > 1.0 * deg);  // allocated
        }
    }
}
