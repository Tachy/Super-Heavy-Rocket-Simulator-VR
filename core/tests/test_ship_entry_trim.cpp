// Pitch trim of the entering ship against the flap stops, and the pairs tools/aero_derive/derive_ship_entry.py loads the
// drawn mesh with.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>

#include "starsim/aero/flap.hpp"
#include "starsim/config/propellant_loading.hpp"
#include "starsim/gnc/entry_aero_table.hpp"
#include "starsim/gnc/entry_guidance.hpp"
#include "starsim/gnc/entry_predictor.hpp"
#include "starsim/gnc/entry_residue_limit.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double reference_density_kg_m3 = 1.0e-3;  // only ratios to q are printed
constexpr double reference_speed_of_sound_mps = 300.0;

std::unique_ptr<vehicle::Vehicle> ship(double propellant_kg) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    config::fill_tanks(cfg.stages[0], propellant_kg);
    return std::make_unique<vehicle::Vehicle>(cfg, STARSIM_CONFIG_DIR);
}

// Pitch moment about the axial CG per dynamic pressure [m^3] at the given flap pair; positive: nose towards the lee.
struct Loads {
    double hull_pitch_m3 = 0.0, flap_pitch_m3 = 0.0;
    math::Vec3 hull_m2, flaps_m2;
};

Loads loads(const vehicle::Vehicle& v, double mach, double alpha, double aft, double forward) {
    const auto& stage = v.stages().front();
    const double cg = v.mass_properties().cg_station_m;
    const double az = stage.config.aero.belly_azimuth_rad;
    const math::Vec3 belly{0.0, std::cos(az), std::sin(az)};
    const math::Vec3 pitch_axis = cross(math::Vec3{1.0, 0.0, 0.0}, belly);
    const double speed = mach * reference_speed_of_sound_mps;
    const math::Vec3 u = math::Vec3{std::cos(alpha), 0.0, 0.0} + belly * std::sin(alpha);
    const double q = 0.5 * reference_density_kg_m3 * speed * speed;
    const auto hull = v.aero().compute(u * speed, reference_density_kg_m3, reference_speed_of_sound_mps, false, 0.0);
    Loads out;
    out.hull_m2 = hull.force_body_n / q;
    out.hull_pitch_m3 = dot(cross(math::Vec3{hull.normal_force_station_m - cg, 0.0, 0.0}, hull.force_body_n), pitch_axis) / q;
    const aero::FinAirData air{reference_density_kg_m3, reference_speed_of_sound_mps, 0.0};
    for (const auto& f : stage.flaps) {
        const bool forward_pair = aero::flap_centroid(f.geometry, 0.5 * f.max_deflection_rad).x > cg;
        const auto r = aero::flap_force(f.geometry, forward_pair ? forward : aft, u * speed, air, *stage.flap_table);
        out.flaps_m2 += r.force_body_n / q;
        out.flap_pitch_m3 += dot(cross(r.application_point_m - math::Vec3{cg, 0.0, 0.0}, r.force_body_n), pitch_axis) / q;
    }
    return out;
}

}  // namespace

TEST_CASE("The pitch trim table agrees with the moments it was solved on", "[entry][flap][trim]") {
    const auto v = ship(35000.0);
    const gnc::AlphaSchedule schedule = gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, "aero/ship_entry_alpha.csv");
    const gnc::EntryAeroTable table(*v, schedule);
    for (const double mach : {0.5, 2.0, 5.0, 8.0, 15.0, 25.0}) {
        const double alpha = schedule.at(mach);
        const auto d = table.trim_flap_rad(mach, alpha);
        REQUIRE(d.size() == 4);
        CAPTURE(mach);
        const auto l = loads(*v, mach, alpha, d[0], d[2]);
        CHECK(std::abs(l.hull_pitch_m3 + l.flap_pitch_m3) < 30.0);  // 3 % of the flaps' moment: the tables' interpolation
    }
}

namespace {

// Whether the pairs can balance the pitch moment at this Mach and alpha, from the moments at the two stop pairs alone.
bool pitch_trimmable(const vehicle::Vehicle& v, double mach, double alpha) {
    const double stop = v.stages().front().flaps.front().max_deflection_rad;
    const auto sum = [](const Loads& l) { return l.hull_pitch_m3 + l.flap_pitch_m3; };
    return (sum(loads(v, mach, alpha, stop, 0.0)) < 0.0) != (sum(loads(v, mach, alpha, 0.0, stop)) < 0.0);
}

}  // namespace

TEST_CASE("The terminal steering never asks for an alpha the flaps cannot trim", "[entry][flap][trim][guidance]") {
    const gnc::AlphaSchedule schedule = gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, "aero/ship_entry_alpha.csv");
    constexpr double search_step_rad = 1.0 * deg;  // the table's search step
    constexpr double table_alpha_min_rad = 40.0 * deg;
    for (const double propellant : {35000.0, 64000.0}) {  // both ends of the entry's mass and CG
        const auto v = ship(propellant);
        gnc::EntryPredictorModel model;
        model.aero = gnc::EntryAeroTable(*v, schedule, v->stages().front().config.entry->terminal_mach);
        for (const double mach : {0.3, 0.7, 0.9, 1.2, 1.5, 2.0, 3.0, 4.0}) {
            CAPTURE(propellant, mach);
            const double nominal = std::min(schedule.at(mach), constants::pi / 2.0);
            const double lowest = model.aero.lowest_trimmed_alpha_rad(mach);
            CHECK(lowest <= nominal + 1e-9);
            if (lowest < nominal) {  // otherwise the schedule itself is the edge (64 t: it does not trim there)
                CHECK(pitch_trimmable(*v, mach, lowest));
                // the search ends at the table's 40 deg (Mach 3 and 4 at 35 t trim below it): no edge there
                if (lowest > table_alpha_min_rad + search_step_rad) {
                    CHECK_FALSE(pitch_trimmable(*v, mach, lowest - search_step_rad));
                }
            }
            // Most lift is the floor or the [E] authority below the schedule; the predictor flies the same function.
            const double most_lift = model.steering_alpha(1.0, mach);
            CHECK(most_lift >= lowest - 1e-9);
            CHECK(most_lift <= nominal + 1e-9);
            CHECK(model.steering_alpha(0.5, mach) == Catch::Approx(nominal));
            if (nominal - most_lift > 1e-3) CHECK(model.steering_fraction(most_lift, mach) == Catch::Approx(1.0));
            // Least lift is 90 deg or the highest alpha the flaps trim with control margin, never below the schedule.
            const double highest = model.aero.highest_trimmed_alpha_rad(mach);
            CHECK(highest >= nominal - 1e-9);
            CHECK(highest <= constants::pi / 2.0 + 1e-9);
            // independent of the walk: above the schedule the flaps bracket the pitch trim there (the margin itself is
            // checked on the flown allocation: test_entry_attitude)
            if (highest > nominal + 1e-9) CHECK(pitch_trimmable(*v, mach, highest));
            const double least_lift = model.steering_alpha(0.0, mach);
            const bool flat = highest - nominal < gnc::EntryPredictorModel::least_lift_min_span_rad;
            CHECK(least_lift == Catch::Approx(flat ? nominal : highest));
            if (!flat) CHECK(model.steering_fraction(least_lift, mach) == Catch::Approx(0.0));
        }
    }
}

// Review I1 (user decision 2026-11-06 "Physik akzeptieren"): the ship vented to the residue limit enters with the
// headers full and the most main-tank residue the flaps trim along the schedule (~41 t); there at Mach 0.9-1.5 the
// schedule itself lacks the 15 deg control margin, so no trimmable attitude carries less lift than the schedule. The
// least-lift half is flat (locally saturated): the steering stays well-defined - no division by its zero span, and a
// Newton step on a Jacobian without radial slope has no radial component (it steers only along the slope it has). The
// fraction is not clamped to the schedule's: the predicted miss's radial slope comes from the rest of the descent,
// where the cap opens again (that clamp left the heavy-residue return without a pass).
TEST_CASE("At the residue limit the terminal steering's least-lift half is saturated, not singular",
          "[entry][flap][trim][guidance]") {
    const gnc::AlphaSchedule schedule = gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, "aero/ship_entry_alpha.csv");
    const auto heavy = ship(62000.0);  // the campaign's heaviest (test_entry_attitude's residue-limit case)
    const auto limit = gnc::entry_residue_limit(*heavy, schedule);
    const double headers_kg = heavy->propellant_kg() - heavy->main_propellant_kg();
    const double entry_propellant_kg = headers_kg + limit.max_main_propellant_kg;
    CAPTURE(entry_propellant_kg);
    CHECK(entry_propellant_kg > 38000.0);  // ~41 t (the review's figure), well above the 35 t of full headers
    CHECK(entry_propellant_kg < 45000.0);
    const auto v = ship(entry_propellant_kg);
    gnc::EntryPredictorModel model;
    model.aero = gnc::EntryAeroTable(*v, schedule, v->stages().front().config.entry->terminal_mach);
    constexpr double mach = 1.2;
    const double nominal = schedule.at(mach);
    CHECK(model.aero.highest_trimmed_alpha_rad(mach) - nominal < gnc::EntryPredictorModel::least_lift_min_span_rad);
    // saturated: the half f < 0.5 flies the schedule, its inverse is the schedule's fraction (no division by ~0)
    for (const double f : {0.0, 0.1, 0.3, 0.5}) CHECK(model.steering_alpha(f, mach) == Catch::Approx(nominal));
    for (const double alpha : {nominal, nominal + 5.0 * deg, constants::pi / 2.0}) {
        const double f = model.steering_fraction(alpha, mach);
        CHECK(std::isfinite(f));
        CHECK(f == Catch::Approx(0.5));
    }
    // At this mass and Mach the flaps hold no alpha below the schedule either (the floor is the schedule: the residue
    // limit's one pair at its stop), so the outer half is flat too: here only the lift's direction steers, the radial
    // slope of the predicted miss comes from the rest of the descent (the guided case in test_entry_guidance).
    CHECK(model.steering_alpha(1.0, mach) <= nominal + 1e-12);
    CHECK(model.steering_min_fraction(mach) == gnc::EntryPredictorModel::origin_fraction);
    // the differences stay in the unit disk; a step with no radial slope (the miss depends on the lift's direction
    // only) is finite and has no radial component
    const double floor = model.steering_min_fraction(mach);
    const math::Vec3 x{1.0, 0.0, 0.0}, y{0.0, 1.0, 0.0};
    for (const math::Vec3 p : {math::Vec3{0.5, 0.0, 0.0}, math::Vec3{0.0, -0.55, 0.0}, math::Vec3{0.2, 0.2, 0.0},
                               math::Vec3{0.005, 0.0, 0.0}}) {
        CAPTURE(p.x, p.y);
        for (const math::Vec3& e : {x, y}) {
            const double s = gnc::EntryPredictorModel::steering_difference(p, e, 0.1);
            CHECK(norm(p + e * s) <= 1.0);
        }
        // radial slope 0: the Jacobian sees only the angle (d miss / d p along p is zero)
        const math::Vec3 radial = math::normalized(p), across{-radial.y, radial.x, 0.0};
        const double j11 = 1000.0 * across.x, j12 = 1000.0 * across.y;  // miss east = 1000 m * the angle's change
        const auto next = gnc::entry_terminal_steering_step(p, j11, j12, 0.0, 0.0, {300.0, 0.0, 0.0}, 0.3, floor);
        CHECK(std::isfinite(next.x));
        CHECK(std::isfinite(next.y));
        CHECK(norm(next) >= floor - 1e-12);
        CHECK(norm(next) <= 1.0 + 1e-12);
        if (norm(p) >= floor) CHECK(std::abs(dot(next - p, radial)) < 1e-9);
    }
}

// Hidden: writes every Mach / alpha trim pair for the derivation: STARSIM_TRIM_CSV=<file>.
TEST_CASE("Ship entry trim dump", "[.][trim_dump]") {
    const gnc::AlphaSchedule schedule = gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, "aero/ship_entry_alpha.csv");
    const char* path = std::getenv("STARSIM_TRIM_CSV");
    std::ofstream file(path ? path : "trim_dump.csv");
    file << "propellant_kg,cg_m,mach,alpha_deg,scheduled,aft_deg,fwd_deg,residual_m3,hull_pitch_m3,flap_pitch_m3,"
            "hull_axial_m2,hull_normal_m2,flaps_axial_m2,flaps_normal_m2\n";
    for (const double propellant : {35000.0, 64000.0}) {
        const auto v = ship(propellant);
        const gnc::EntryAeroTable table(*v, schedule);
        const auto row = [&](double mach, double alpha, bool scheduled) {
            const auto d = table.trim_flap_rad(mach, alpha);
            const auto l = loads(*v, mach, alpha, d[0], d[2]);
            file << propellant << "," << v->mass_properties().cg_station_m << "," << mach << "," << alpha / deg << ","
                 << scheduled << "," << d[0] / deg << "," << d[2] / deg << "," << l.hull_pitch_m3 + l.flap_pitch_m3 << ","
                 << l.hull_pitch_m3 << "," << l.flap_pitch_m3 << "," << l.hull_m2.x << "," << l.hull_m2.z << ","
                 << l.flaps_m2.x << "," << l.flaps_m2.z << "\n";
        };
        for (const double mach : {0.3, 0.8, 1.2, 1.5, 2.0, 2.5, 3.0, 4.0, 8.0, 15.0, 25.0}) {
            row(mach, schedule.at(mach), true);
            for (double a = 40.0; a <= 140.0 + 1e-9; a += 20.0) row(mach, a * deg, false);
        }
    }
}

// Hidden: the terminal steering's trimmed alpha range per Mach (schedule, lowest, highest with control margin).
TEST_CASE("Ship entry trimmed alpha range dump", "[.][trim_range_dump]") {
    const gnc::AlphaSchedule schedule = gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, "aero/ship_entry_alpha.csv");
    for (const double propellant : {35000.0, 38000.0, 41000.0}) {
        const auto v = ship(propellant);
        const gnc::EntryAeroTable table(*v, schedule);
        for (const double mach : {0.3, 0.5, 0.7, 0.8, 0.9, 1.0, 1.2, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 6.0, 8.0}) {
            std::cout << "prop " << propellant / 1000.0 << " t mach " << mach << ": schedule " << schedule.at(mach) / deg
                      << " lowest " << table.lowest_trimmed_alpha_rad(mach) / deg << " highest "
                      << table.highest_trimmed_alpha_rad(mach) / deg << "\n";
        }
    }
}

// Hidden: the table's build time, mean over builds at 35 and 41 t: the cap over the whole grid (default), up to the
// guidance's terminal Mach (as EntryGuidance builds it), none (as the residue limit's bisection builds it).
TEST_CASE("Ship entry aero table build time", "[.][trim_build_time]") {
    const gnc::AlphaSchedule schedule = gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, "aero/ship_entry_alpha.csv");
    for (const double propellant : {35000.0, 41000.0}) {
        const auto v = ship(propellant);
        for (const double steering_mach_max : {std::numeric_limits<double>::infinity(),
                                               v->stages().front().config.entry->terminal_mach, 0.0}) {
            constexpr int builds = 5;
            const auto start = std::chrono::steady_clock::now();
            for (int i = 0; i < builds; ++i) {
                const gnc::EntryAeroTable table(*v, schedule, steering_mach_max);
                CHECK(table.cd_area_m2(1.0) > 0.0);
            }
            const std::chrono::duration<double, std::milli> took = std::chrono::steady_clock::now() - start;
            std::cout << "prop " << propellant / 1000.0 << " t, cap to Mach " << steering_mach_max << ": "
                      << took.count() / builds << " ms per build\n";
        }
    }
}
