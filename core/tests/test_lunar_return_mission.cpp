// Mission config lunar_return (l5): loading, the start state in the LLO / NRHO, the entry speed from the corridor and
// the propellant fixed point; refusals without a corridor and beyond the tanks.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "lunar_return_test_support.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/lunar_ascent_guidance.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/lunar_return_mission.hpp"
#include "starsim/sim/mission_start.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::StartsWith;

namespace {

constexpr double deg = constants::deg_to_rad;

void print_budget(const char* what, const sim::LunarReturnBudget& b) {
    if (!std::getenv("STARSIM_VERBOSE")) return;
    std::printf("[lunar return] %s: closes %d (%s), propellant %.1f t, TEI %.1f m/s, braking %.1f m/s, entry %.0f m/s, "
                "corridor %.2f .. %.2f deg (middle %.2f, width %.2f, %s), bank window at %.2f deg: %.1f .. %.1f deg\n",
                what, b.closes ? 1 : 0, b.reason.c_str(), b.propellant_kg / 1000.0, b.tei_delta_v_mps,
                b.braking_delta_v_mps, b.entry_speed_mps, b.corridor.steep_rad / deg, b.corridor.shallow_rad / deg,
                b.corridor.middle_rad / deg, b.corridor.width_rad / deg, b.corridor.limiting.c_str(),
                b.bank_window.flight_path_rad / deg, b.bank_window.low_rad / deg, b.bank_window.high_rad / deg);
}

}  // namespace

TEST_CASE("Lunar return missions load", "[lunar][return][config]") {
    const auto m = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_return_llo");
    CHECK(config::mission_kind(STARSIM_CONFIG_DIR, "ship_lunar_return_llo") == config::MissionKind::LunarReturn);
    CHECK(m.stage_id == "ship");
    CHECK(m.orbit == "llo");
    CHECK_FALSE(m.aerocapture);
    CHECK(m.braking_height_m == Catch::Approx(2000e3));
    CHECK(m.mcc_after_tei_s.size() == 2);
    const auto a = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_aircatch_llo");
    CHECK(a.aerocapture);
    CHECK(a.aerocapture_apoapsis_m == Catch::Approx(400e3));
    CHECK(a.deorbit.search_window_s > 0.0);
    const auto n = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_return_nrho");
    CHECK(n.orbit == "nrho");
    CHECK(n.nrho_revolutions == 9);
    // Refuelling r2 (user 2026-10-20): the bare Block 3 ship cannot keep its propellant for days - its lunar missions
    // fly without boil-off until r5 (Moon radiation, thermal attitude).
    CHECK_FALSE(m.boil_off);
    CHECK_FALSE(a.boil_off);
    CHECK_FALSE(n.boil_off);
    CHECK_THROWS_AS(config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "hls_lunar_llo"), config::ConfigError);
}

// Planner plan 2026-11-03 Task 6: the aircatch's perigee raise + circularisation and b4 deorbit are mission choices
// ([return] capture_raise_mps / capture_deorbit_mps), not code constants; the start's propellant follows them by the
// rocket equation on the deorbit engines.
TEST_CASE("Lunar return burns come from the mission", "[lunar][return][config][session]") {
    const auto m = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_aircatch_llo");
    CHECK(m.aerocapture_raise_mps == 120.0);
    CHECK(m.aerocapture_deorbit_mps == 100.0);
    test::LunarReturnSession t;
    const time::Epoch epoch = t.time.from_utc(m.epoch_utc);
    const auto site = config::load_launch_site(STARSIM_CONFIG_DIR, m.site_id);
    t.session.set_launch_site(sim::launch_site_from_config(site));
    const auto site_itrf = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    const auto ship = sim::lunar_return_ship(m, STARSIM_CONFIG_DIR);
    const auto budget = [&](const config::LunarReturnMission& mission) {
        auto setup = sim::lunar_return_setup(mission, site_itrf, t.session.gravity(), t.ephemeris, epoch);
        return sim::lunar_return_budget(t.session, STARSIM_CONFIG_DIR, mission, ship, epoch, setup);
    };
    const auto nominal = budget(m);
    auto more = m;
    more.aerocapture_raise_mps += 100.0;
    const auto raised = budget(more);
    REQUIRE(nominal.closes);
    REQUIRE(raised.closes);
    // 100 m/s more on the deorbit engines multiply everything the burns before carry (dry + landing propellant and the
    // propellant of TEI, MCC and braking) by exp(100 / ve); MCC, coast and reserve terms differ by < 1 % of the step
    const vehicle::Vehicle full(ship, STARSIM_CONFIG_DIR);
    const double dry_kg = full.mass_properties().mass_kg - full.propellant_kg();
    std::vector<std::string> deorbit;
    for (const auto& g : ship.stages.front().deorbit_engine_groups) deorbit.push_back("ship/" + g);
    const double ve_mps = gnc::groups_vacuum_isp_s(full, deorbit) * constants::standard_gravity_mps2;
    REQUIRE(ve_mps > 0.0);
    const double coast_kg = m.coast_attitude_kg_per_day * m.flight_time_s / 86400.0;
    const double carried_kg = nominal.propellant_kg + dry_kg - m.reserve_kg - coast_kg;
    const double expected_kg = carried_kg * std::expm1(100.0 / ve_mps);
    INFO("step " << (raised.propellant_kg - nominal.propellant_kg) / 1e3 << " t, expected " << expected_kg / 1e3 << " t");
    CHECK(raised.propellant_kg - nominal.propellant_kg == Catch::Approx(expected_kg).epsilon(0.02));
}

TEST_CASE("Lunar return start tanks the ship for its return and plans it", "[lunar][return][session][slow]") {
    test::LunarReturnSession t;
    sim::LunarReturnBudget budget;
    const auto m = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_return_llo");
    const auto s = sim::start_lunar_return_mission(t.session, STARSIM_CONFIG_DIR, m, t.time.from_utc(m.epoch_utc),
                                                   nullptr, &budget);
    print_budget("LLO direct", budget);
    INFO(s.refusal);
    REQUIRE(budget.closes);
    CHECK(s.return_planned);
    // [D] TEI ~0.9 + braking ~1.1 km/s (l5 Task 6; the plan estimated 1.5) + MCC + coast + landing + 10 t reserve:
    // ~190 t on 177 t dry (planner plan Task 3b; ~250 t on 255 t dry before). The load must at least carry the
    // planned burns by the rocket equation on the vacuum Raptors from the entry mass (dry + header landing propellant),
    // plus that landing propellant - a floor that follows the dry mass (MCC, coast and reserve come on top):
    // 212 t x (exp(2.0 km/s / (380 s g0)) - 1) + 35 t = ~186 t at 177 t dry.
    const auto& v = t.session.body(t.session.controlled_index()).vehicle();
    const double dry_kg = v.mass_properties().mass_kg - v.propellant_kg();
    double landing_kg = 0.0;
    test::block3_with_ship([&](config::StageConfig& s) { landing_kg = s.landing_propellant_kg; });
    const double ve_mps = gnc::groups_vacuum_isp_s(v, {"ship/vacuum"}) * constants::standard_gravity_mps2;
    REQUIRE(ve_mps > 0.0);
    const double burns_floor_kg =
        (dry_kg + landing_kg) * std::expm1((budget.tei_delta_v_mps + budget.braking_delta_v_mps) / ve_mps) + landing_kg;
    INFO("floor " << burns_floor_kg / 1e3 << " t");
    CHECK(budget.tei_delta_v_mps + budget.braking_delta_v_mps > 0.0);
    CHECK(v.propellant_kg() > burns_floor_kg);
    CHECK(v.propellant_kg() < 400e3);
    CHECK(v.propellant_kg() == Catch::Approx(budget.propellant_kg).margin(1.0));
    CHECK(budget.entry_speed_mps > 9000.0);
    CHECK(budget.corridor.width_rad >= m.min_corridor_width_rad - 1e-9);
    // a ~100 km polar LLO about the Moon
    const auto& st = t.session.body(t.session.controlled_index()).state();
    const auto moon = t.ephemeris.state_gcrf(ephemeris::Body::Moon, s.epoch);
    const double altitude_m = norm(st.position_gcrf_m - moon.position_m) - constants::moon_mean_radius_m;
    CHECK(std::abs(altitude_m - 100e3) < 1e3);
    CHECK(t.session.lunar_step() == sim::LunarStep::ReturnDeparture);
}

TEST_CASE("Lunar return start refuses without an entry corridor", "[lunar][return][session]") {
    // Review Focus 2: heat-rate limit lowered to 0.3 MW/m^2: no admissible entry speed above the planner's floor
    test::LunarReturnSession t;
    const auto vehicle = test::block3_with_ship([](config::StageConfig& s) { s.heat_shield.max_heat_rate_w_m2 = 0.3e6; });
    const auto m = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_return_llo");
    const auto s = sim::start_lunar_return_mission(t.session, STARSIM_CONFIG_DIR, m, t.time.from_utc(m.epoch_utc), &vehicle);
    CHECK_FALSE(s.return_planned);
    CHECK_THAT(s.refusal, StartsWith("Kein Eintrittskorridor"));
}

TEST_CASE("Lunar return start refuses a return the tanks cannot hold", "[lunar][return][session]") {
    // tanks of 100 t (a sixteenth): the return needs ~190 t on 177 t dry (Task 3b; at 255 t dry an eighth, 200 t, was
    // short of ~250 t) - a heavier dry mass fails earlier: no entry corridor
    test::LunarReturnSession t;
    const auto vehicle = test::block3_with_ship([](config::StageConfig& s) {
        for (auto& tank : s.tanks) {
            tank.mass_kg *= 0.0625;
            tank.capacity_kg *= 0.0625;  // smaller tanks, not a partial load
        }
    });
    const auto m = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_return_llo");
    const auto s = sim::start_lunar_return_mission(t.session, STARSIM_CONFIG_DIR, m, t.time.from_utc(m.epoch_utc), &vehicle);
    CHECK_FALSE(s.return_planned);
    CHECK_THAT(s.refusal, ContainsSubstring("Treibstoff"));
}

// Header-tank plan review: TEI, mid-course corrections and the braking burn feed from the main tanks; the headers
// hold the landing propellant. 25 t short of the fixed point the total aboard still covers the burns (headers
// counted), the main tanks do not - the plan must refuse instead of flaming out in the braking burn.
TEST_CASE("Lunar return plan counts the main tanks only", "[lunar][return][session][header]") {
    test::LunarReturnSession t;
    const auto m = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_return_llo");
    const time::Epoch epoch = t.time.from_utc(m.epoch_utc);
    const auto site = config::load_launch_site(STARSIM_CONFIG_DIR, m.site_id);
    t.session.set_launch_site(sim::launch_site_from_config(site));
    const auto site_itrf = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    const auto ship = sim::lunar_return_ship(m, STARSIM_CONFIG_DIR);
    auto setup = sim::lunar_return_setup(m, site_itrf, t.session.gravity(), t.ephemeris, epoch);
    const auto b = sim::lunar_return_budget(t.session, STARSIM_CONFIG_DIR, m, ship, epoch, setup);
    REQUIRE(b.closes);
    sim::place_lunar_return_ship(t.session, m, ship, epoch, b.propellant_kg - 25000.0, setup.ret);
    CHECK_FALSE(t.session.plan_lunar_return(setup));
    CHECK_THAT(t.session.lunar_status(), ContainsSubstring("Treibstoff"));
}

TEST_CASE("An inclined LLO contains the return asymptote; below its declination none does", "[lunar][return]") {
    // Polar is the south-pole landing's choice, not the return's: any inclination >= |declination of the asymptote|
    // (Moon equator) holds a plane through it.
    test::LunarReturnSession t;
    auto m = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_return_llo");
    const time::Epoch epoch = t.time.from_utc(m.epoch_utc);
    const auto site = config::load_launch_site(STARSIM_CONFIG_DIR, m.site_id);
    const auto site_itrf = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    const auto setup = sim::lunar_return_setup(m, site_itrf, t.session.gravity(), t.ephemeris, epoch);
    const math::Vec3 s = mission::return_asymptote(t.ephemeris, epoch + 3600.0, setup.ret.flight_time_guess_s,
                                                   setup.ret.target.site_itrf_m, t.earth);
    const auto rot = t.moon.fixed_to_gcrf(epoch).rotation;
    const math::Vec3 pole{rot(0, 2), rot(1, 2), rot(2, 2)};
    const double declination = std::asin(dot(s, pole));
    for (const double inclination : {std::abs(declination) + 5.0 * deg, 150.0 * deg}) {
        m.llo_inclination_rad = inclination;
        const auto st = sim::lunar_return_start_state(t.session, m, epoch, setup.ret);
        const auto moon = t.ephemeris.state_gcrf(ephemeris::Body::Moon, epoch);
        const math::Vec3 n = math::normalized(cross(st.position_m - moon.position_m, st.velocity_mps - moon.velocity_mps));
        CHECK(std::abs(dot(n, s)) < std::sin(0.1 * deg));
        CHECK(std::abs(std::acos(dot(n, pole)) - inclination) < 0.1 * deg);
    }
    m.llo_inclination_rad = 0.5 * std::abs(declination);
    CHECK_THROWS_AS(sim::lunar_return_start_state(t.session, m, epoch, setup.ret), std::runtime_error);
}

TEST_CASE("The aligned LLO contains the return asymptote", "[lunar][return]") {
    test::LunarReturnSession t;
    const auto m = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_return_llo");
    const time::Epoch epoch = t.time.from_utc(m.epoch_utc);
    const auto site = config::load_launch_site(STARSIM_CONFIG_DIR, m.site_id);
    const auto site_itrf = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    const auto setup = sim::lunar_return_setup(m, site_itrf, t.session.gravity(), t.ephemeris, epoch);
    sim::place_lunar_return_ship(t.session, m, sim::lunar_return_ship(m, STARSIM_CONFIG_DIR), epoch, 300e3, setup.ret);
    const auto& st = t.session.body(t.session.controlled_index()).state();
    const auto moon = t.ephemeris.state_gcrf(ephemeris::Body::Moon, epoch);
    const math::Vec3 r = st.position_gcrf_m - moon.position_m, v = st.velocity_gcrf_mps - moon.velocity_mps;
    const math::Vec3 n = math::normalized(cross(r, v));
    const math::Vec3 s = mission::return_asymptote(t.ephemeris, epoch + 3600.0, setup.ret.flight_time_guess_s,
                                                   setup.ret.target.site_itrf_m, t.earth);
    CHECK(std::abs(std::acos(dot(n, s)) - 0.5 * constants::pi) < 0.1 * deg);
    const auto rot = t.moon.fixed_to_gcrf(epoch).rotation;
    const math::Vec3 pole{rot(0, 2), rot(1, 2), rot(2, 2)};
    CHECK(std::abs(std::acos(dot(n, pole)) - 0.5 * constants::pi) < 0.1 * deg);  // polar
    CHECK(std::abs(norm(r) - constants::moon_mean_radius_m - 100e3) < 1e3);
    const double circular_mps = std::sqrt(t.ephemeris.gm_m3ps2(ephemeris::Body::Moon) / norm(r));
    CHECK(std::abs(norm(v) - circular_mps) < 1.0);
    CHECK(std::abs(dot(math::normalized(r), math::normalized(v))) < 1e-6);
    CHECK(t.session.body(t.session.controlled_index()).vehicle().propellant_kg() == Catch::Approx(300e3).margin(1.0));
}

TEST_CASE("start_mission dispatches a lunar return", "[lunar][return][session]") {
    // the aerocapture variant (its budget adds the perigee raise and the b4 deorbit on the deorbit engine)
    test::LunarReturnSession t;
    const auto s = sim::start_mission(t.session, STARSIM_CONFIG_DIR, "ship_lunar_aircatch_llo", {}, t.time);
    CHECK(s.kind == config::MissionKind::LunarReturn);
    INFO(s.refusal);
    REQUIRE(s.return_planned);
    const auto& v = t.session.body(t.session.controlled_index()).vehicle();
    if (std::getenv("STARSIM_VERBOSE")) {
        std::printf("[lunar return] LLO aircatch: propellant %.1f t\n", v.propellant_kg() / 1000.0);
    }
    // 186.5 t on 177 t dry (planner plan Task 3b; above 200 t on 255 t dry)
    CHECK(v.propellant_kg() > 150e3);
    CHECK(v.propellant_kg() < 400e3);
}

TEST_CASE("Lunar return start from the NRHO plans the departure burn and the powered flyby",
          "[lunar][return][session][slow]") {
    // Artemis-like: departure burn + return powered flyby at 100 km (l5 Task 6: ~0.48 km/s in all)
    test::LunarReturnSession t;
    sim::LunarReturnBudget budget;
    const auto m = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_return_nrho");
    const auto s = sim::start_lunar_return_mission(t.session, STARSIM_CONFIG_DIR, m, t.time.from_utc(m.epoch_utc),
                                                   nullptr, &budget);
    print_budget("NRHO direct", budget);
    INFO(s.refusal);
    REQUIRE(budget.closes);
    CHECK(budget.tei_delta_v_mps > 300.0);
    CHECK(budget.tei_delta_v_mps < 600.0);
    const auto& v = t.session.body(t.session.controlled_index()).vehicle();
    // [D] 177 t dry (planner plan Task 3b; 150 t was set for 255 t dry) + 35 t landing + reserve after TEI ~0.45 and
    // braking ~0.83 km/s at Isp ~375 s: ~137 t + the coast's boil-off (flown 144.5 t, planner plan Task 12)
    CHECK(v.propellant_kg() > 130e3);
    CHECK(v.propellant_kg() < 400e3);
}

// Probe (refuelling r5, plan decision 9): the Block 3 lunar return from the LLO with boil-off on (the mission keeps
// boil_off = false; this prints what the flag would cost).
TEST_CASE("Probe: Block 3 lunar return with boil-off", "[.r5_probe]") {
    test::LunarReturnSession t;
    sim::LunarReturnBudget budget;
    auto m = config::load_lunar_return_mission(STARSIM_CONFIG_DIR, "ship_lunar_return_llo");
    m.boil_off = true;
    const auto s = sim::start_lunar_return_mission(t.session, STARSIM_CONFIG_DIR, m, t.time.from_utc(m.epoch_utc),
                                                   nullptr, &budget);
    INFO(s.refusal);
    REQUIRE(s.refusal.empty());
    const double start_kg = t.session.body(t.session.controlled_index()).vehicle().propellant_kg();
    const auto t0 = t.session.epoch();
    while (t.session.epoch() - t0 < 8.0 * 86400.0) {
        t.session.advance_to(t.session.epoch() + 600.0);
        const auto& b = t.session.body(t.session.controlled_index());
        if (b.caught() || b.impacted() || b.resting() || t.session.lunar_step() == sim::LunarStep::None) break;
    }
    const auto& b = t.session.body(t.session.controlled_index());
    std::printf("[r5 probe] Block 3 LLO return with boil-off: start %.1f t, end %.1f t (main %.1f t), %.1f d, caught %d, "
                "impacted %d, status '%s', abort '%s'\n",
                start_kg / 1e3, b.vehicle().propellant_kg() / 1e3, b.vehicle().main_propellant_kg() / 1e3,
                (t.session.epoch() - t0) / 86400.0, b.caught(), b.impacted(), t.session.lunar_status().c_str(),
                t.session.burn_abort_reason().c_str());
}
