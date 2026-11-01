// Pitch trim of the entering ship against the flap stops, and the pairs tools/aero_derive/derive_ship_entry.py loads the
// drawn mesh with.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

#include "starsim/aero/flap.hpp"
#include "starsim/config/propellant_loading.hpp"
#include "starsim/gnc/entry_aero_table.hpp"
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
