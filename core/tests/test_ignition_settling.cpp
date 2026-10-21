// Refuelling f3: settling before an in-orbit ignition (settling state, PMD tanks, gas ingestion, settling push).

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/boil_off.hpp"
#include "starsim/vehicle/propellant_transfer.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

// The tanker's ship stage alone with `lox_kg` / `ch4_kg` in its main tanks.
std::unique_ptr<vehicle::Vehicle> tanker(double lox_kg = 50.0e3, double ch4_kg = 15.0e3) {
    auto v = std::make_unique<vehicle::Vehicle>(config::top_stage_config(config::load_vehicle(config_dir, "starship_tanker")),
                                                config_dir);
    v->set_tank_mass(0, "lox_main", lox_kg);
    v->set_tank_mass(0, "ch4_main", ch4_kg);
    return v;
}

// 200 km, body z radial, no rotation (as test_refuelling_transfer.cpp); `tide` false: far from any body.
vehicle::LiquidEnvironment orbit_liquid(const math::Vec3& specific_force_body_mps2, bool tide = true) {
    const double r = tide ? constants::wgs84_semi_major_axis_m + 200.0e3 : 1.0e15;
    return {specific_force_body_mps2, {}, {0.0, 0.0, r}, math::Quat{}};
}

}  // namespace

TEST_CASE("Settling: a +x push settles the main tanks in sqrt(2L/a)", "[ignition_settling][vehicle]") {
    auto v = tanker();
    REQUIRE(v->needs_settling(0));
    v->stages()[0].settled = 0.0;
    const double a = 0.010;
    const auto liquid = orbit_liquid({a, 0.0, 0.0});
    const double t = vehicle::settling_time_s(v->settling_travel_m(0), a);
    CHECK(t == Catch::Approx(std::sqrt(2.0 * v->settling_travel_m(0) / a)));
    v->update_settling(liquid, 0.5 * t);
    CHECK(v->stages()[0].settled == Catch::Approx(0.5).margin(0.02));  // the tide adds < 2 %
    CHECK_FALSE(v->stage_settled(0));
    v->update_settling(liquid, 0.6 * t);
    CHECK(v->stage_settled(0));
}

TEST_CASE("Settling: without thrust the liquid leaves the outlets", "[ignition_settling][vehicle]") {
    auto v = tanker();
    const auto drag_nose_first = orbit_liquid({-2.0e-5, 0.0, 0.0});  // drag pushes the liquid forward
    v->update_settling(drag_nose_first, 60.0);
    CHECK(v->stages()[0].settled < 1.0);
    v->update_settling(drag_nose_first, 1.0e5);
    CHECK(v->stages()[0].settled == 0.0);
    v->stages()[0].settled = 0.4;
    v->update_settling(orbit_liquid({}, false), 1.0e5);  // no acceleration: the liquid stays where it is
    CHECK(v->stages()[0].settled == Catch::Approx(0.4));
}

TEST_CASE("Settling: PMD tanks are always covered, header feed needs no settling", "[ignition_settling][vehicle]") {
    auto v = tanker();
    for (const auto& t : v->stages()[0].tanks) {
        INFO(t.config.id);
        CHECK(t.config.propellant_management_device == (t.config.role == config::TankRole::Header));
    }
    v->select_feed(0, vehicle::FeedSource::Header);
    CHECK_FALSE(v->needs_settling(0));
    v->stages()[0].settled = 0.0;
    CHECK(v->stage_settled(0));
    v->select_feed(0, vehicle::FeedSource::Main);
    CHECK_FALSE(v->stage_settled(0));
    // main tanks dry: nothing to settle
    auto dry = tanker(0.0, 0.0);
    CHECK_FALSE(dry->needs_settling(0));
}

TEST_CASE("Settling: the travel is the ullage of the longest main tank, at least 1 m", "[ignition_settling][vehicle]") {
    auto v = tanker();
    double longest = 0.0;
    for (const auto& t : v->stages()[0].tanks) {
        if (t.config.role != config::TankRole::Main || !(t.mass_kg > 0.0)) continue;
        longest = std::max(longest, t.length_m * (1.0 - t.mass_kg / t.config.capacity_kg));
    }
    CHECK(longest > 10.0);
    CHECK(v->settling_travel_m(0) == Catch::Approx(longest));
    for (const auto& t : std::vector<vehicle::Tank>(v->stages()[0].tanks)) {
        if (t.config.role == config::TankRole::Main) v->set_tank_mass(0, t.config.id, t.config.capacity_kg);
    }
    CHECK(v->settling_travel_m(0) == Catch::Approx(vehicle::settling_min_travel_m));
}

TEST_CASE("Vehicle configs flag the PMD tanks", "[ignition_settling][config]") {
    const auto dragon = config::load_vehicle(config_dir, "crew_dragon");
    bool found = false;
    for (const auto& s : dragon.stages) {
        for (const auto& t : s.tanks) {
            if (t.id == "mmh_nto") {
                found = true;
                CHECK(t.propellant_management_device);
            }
        }
    }
    CHECK(found);
    for (const std::string id : {"starship_depot", "starship_block3", "starship_hls"}) {
        INFO(id);
        for (const auto& t : config::load_vehicle(config_dir, id).stages.back().tanks) {
            CHECK(t.propellant_management_device == (t.role == config::TankRole::Header));
        }
    }
}
