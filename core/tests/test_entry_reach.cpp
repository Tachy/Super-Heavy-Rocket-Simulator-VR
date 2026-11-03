// The deorbit planner's crossrange reach from the entry vehicle's hypersonic L/D (planner plan Task 7).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "starsim/config/propellant_loading.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/entry_aero_table.hpp"
#include "starsim/gnc/entry_reach.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double radius_m = constants::wgs84_semi_major_axis_m;

// L/D of the table over the hypersonic glide (Mach 8 .. 30, where the equilibrium glide builds its crossrange).
void lift_to_drag_band(const gnc::EntryAeroTable& table, double& low, double& high) {
    low = 1e9;
    high = 0.0;
    for (double mach = 8.0; mach <= 30.0; mach += 0.5) {
        const double e = table.cl_area_m2(mach) / table.cd_area_m2(mach);
        low = std::min(low, e);
        high = std::max(high, e);
    }
}

gnc::EntryPredictorModel ship_model() {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    config::fill_tanks(cfg.stages[0], cfg.stages[0].landing_propellant_kg);  // at the entry: the landing propellant
    const vehicle::Vehicle ship(cfg, STARSIM_CONFIG_DIR);
    gnc::EntryPredictorModel m;
    m.aero = gnc::EntryAeroTable(ship, gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, cfg.stages[0].aero.entry_alpha_table));
    m.mass_kg = ship.mass_properties().mass_kg;
    return m;
}

gnc::EntryPredictorModel dragon_model() {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon");
    cfg.stages.erase(cfg.stages.begin());  // the capsule after the trunk jettison
    const vehicle::Vehicle capsule(cfg, STARSIM_CONFIG_DIR);
    gnc::EntryPredictorModel m;
    m.aero = gnc::EntryAeroTable::capsule(capsule);
    m.mass_kg = capsule.mass_properties().mass_kg;
    return m;
}

}  // namespace

TEST_CASE("Constant-L/D crossrange reach against the hand formula", "[deorbit][entry]") {
    // (pi^2 / 48) (L/D)^2 sin(2 sigma) R: L/D 1, 45 deg -> 0.2056 R = 1311.4 km
    CHECK(gnc::constant_lift_crossrange_reach_m(1.0, 45.0 * deg, 6378137.0) == Catch::Approx(1311.4e3).epsilon(1e-4));
    CHECK(gnc::constant_lift_crossrange_reach_m(0.5, 30.0 * deg, 6378137.0) ==
          Catch::Approx(0.25 * std::sin(60.0 * deg) / std::sin(90.0 * deg) * 1311.4e3).epsilon(1e-4));
}

TEST_CASE("Crossrange follows the vehicle L/D", "[deorbit][entry][ship][dragon]") {
    const double circular_mps = std::sqrt(constants::earth_gm_m3ps2 / radius_m);
    struct Case {
        const char* name;
        gnc::EntryPredictorModel model;
        double bank_rad;  // the return missions' nominal bank ([deorbit] nominal_bank_deg)
    };
    for (const auto& c : {Case{"ship", ship_model(), 50.0 * deg}, Case{"Crew Dragon", dragon_model(), 30.0 * deg}}) {
        double low = 0.0, high = 0.0;
        lift_to_drag_band(c.model.aero, low, high);
        const double reach = gnc::entry_crossrange_reach_m(c.model, c.bank_rad, circular_mps);
        const double reach_low = gnc::constant_lift_crossrange_reach_m(low, c.bank_rad, radius_m);
        const double reach_high = gnc::constant_lift_crossrange_reach_m(high, c.bank_rad, radius_m);
        std::printf("%s: %.0f t, hypersonic L/D %.3f .. %.3f, bank %.0f deg: reach %.1f km (closed form %.1f .. %.1f km); "
                    "from 7.6 km/s %.1f km\n",
                    c.name, c.model.mass_kg / 1000.0, low, high, c.bank_rad / deg, reach / 1000.0, reach_low / 1000.0,
                    reach_high / 1000.0, gnc::entry_crossrange_reach_m(c.model, c.bank_rad, 7600.0) / 1000.0);
        CAPTURE(c.name);
        // The integral flies the table's L/D over the glide: within the closed forms of its hypersonic band (the
        // supersonic tail adds little).
        CHECK(reach > 0.9 * reach_low);
        CHECK(reach < 1.1 * reach_high);
        // Steeper bank, less vertical lift for the glide but more to the side: sin(2 sigma) at constant L/D.
        CHECK(gnc::entry_crossrange_reach_m(c.model, 45.0 * deg, circular_mps) >
              gnc::entry_crossrange_reach_m(c.model, 20.0 * deg, circular_mps));
        // A slower entry leaves less glide.
        CHECK(gnc::entry_crossrange_reach_m(c.model, c.bank_rad, 0.95 * circular_mps) < reach);
    }
    // The reach scales with (L/D)^2: the ship (L/D ~0.6-0.8) reaches an order of magnitude farther than the capsule (0.2).
    CHECK(gnc::entry_crossrange_reach_m(ship_model(), 50.0 * deg, circular_mps) >
          8.0 * gnc::entry_crossrange_reach_m(dragon_model(), 30.0 * deg, circular_mps));
}
