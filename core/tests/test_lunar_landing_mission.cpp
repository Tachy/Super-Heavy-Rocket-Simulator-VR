// Mission config lunar_landing: loading, placement in the parking orbit through the Moon's plane, start_mission
// dispatch, and refusal without a plane.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <thread>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/lunar_ascent_guidance.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/lunar_orbit_transfer.hpp"
#include "starsim/sim/lunar_landing_mission.hpp"
#include "starsim/sim/lunar_site.hpp"
#include "starsim/sim/lunar_windows.hpp"
#include "starsim/sim/mission_start.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;

TEST_CASE("Lunar landing missions load", "[config][lunar]") {
    constexpr double deg = constants::deg_to_rad;
    CHECK(config::mission_kind(STARSIM_CONFIG_DIR, "hls_lunar_llo") == config::MissionKind::LunarLanding);
    const auto llo = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_llo");
    CHECK(llo.vehicle_id == "starship_hls");
    CHECK(llo.site_id == "mons_mouton_im2");
    CHECK(llo.lunar_orbit == "llo");
    CHECK_THAT(llo.parking_height_m, WithinAbs(200e3, 1e-6));
    CHECK_THAT(llo.parking_inclination_rad, WithinAbs(28.5 * deg, 1e-12));
    CHECK_THAT(llo.flight_time_s, WithinAbs(4.0 * 86400.0, 1e-6));
    CHECK_THAT(llo.earliest_departure_s, WithinAbs(1800.0, 1e-9));
    REQUIRE(llo.mcc_after_tli_s.size() == 2);
    CHECK_THAT(llo.mcc_after_tli_s[1], WithinAbs(2.5 * 86400.0, 1e-6));
    CHECK_THAT(llo.llo_altitude_m, WithinAbs(100e3, 1e-6));
    CHECK_FALSE(llo.llo_inclination_rad);  // Task 7 (lunar sites): the plane is chosen through the site
    REQUIRE(llo.touchdown_after_epoch_s);  // 2026-11-22T01:20 UTC from epoch_utc 2026-11-13T00:00 (9 d 1 h 20 min)
    CHECK_THAT(*llo.touchdown_after_epoch_s, WithinAbs(((9.0 * 24.0 + 1.0) * 60.0 + 20.0) * 60.0, 1e-3));
    // [windows] (lunar sites Task 8): 90 d, 5 windows, preferred Sun 5-20 deg
    CHECK_THAT(llo.windows.horizon_s, WithinAbs(90.0 * 86400.0, 1e-6));
    CHECK(llo.windows.count == 5);
    CHECK_THAT(llo.windows.sun_preferred_min_rad, WithinAbs(5.0 * constants::deg_to_rad, 1e-12));
    CHECK_THAT(llo.windows.sun_preferred_max_rad, WithinAbs(20.0 * constants::deg_to_rad, 1e-12));
    const auto nrho = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_nrho");
    CHECK(nrho.lunar_orbit == "nrho");
    CHECK(nrho.nrho_revolutions == 9);
    CHECK(nrho.nrho_synodic_months == 2);
    CHECK(nrho.nrho_reference_revolutions == 6);
    CHECK_THAT(nrho.nrho_apolune_position.z, WithinAbs(-0.1821, 1e-12));
    CHECK_THAT(nrho.nrho_apolune_velocity.y, WithinAbs(-0.1033, 1e-12));
    CHECK_THROWS_AS(config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "crew_dragon_return"), config::ConfigError);
}

TEST_CASE("start_mission plans the HLS outbound flight", "[sim][lunar][mission]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const auto moon = spice::SpiceOrientation::moon_mean_earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon};
    const auto start = sim::start_mission(session, STARSIM_CONFIG_DIR, "hls_lunar_llo", {}, time);
    INFO(report::describe(start.refusal));
    CHECK(start.kind == config::MissionKind::LunarLanding);
    CHECK(start.return_planned);
    CHECK(start.refusal.ok());
    CHECK(session.lunar_step() == sim::LunarStep::Departure);
    CHECK(session.epoch().tdb_seconds_past_j2000() == time.from_utc({2026, 11, 13, 0, 0, 0.0}).tdb_seconds_past_j2000());  // the mission's epoch_utc: daylight at the site (was 2026-11-02; task 6c)
    // circular 200 km parking orbit
    const auto& s = session.body(session.controlled_index()).state();
    CHECK_THAT(norm(s.position_gcrf_m) - constants::wgs84_semi_major_axis_m, WithinAbs(200e3, 1.0));
}

// Lunar sites plan Task 7: the LLO plane is chosen through the site at the mission's touchdown (no inclination in the
// mission). For Mons Mouton (84.79 S) that plane is the polar one the mission flew before within the 20 km the old
// polar plane passed the site by, and the mission still closes.
TEST_CASE("mons mouton mission unchanged: inclination within 1 deg of polar and the same feasibility", "[sim][lunar][mission]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const auto moon = spice::SpiceOrientation::moon_mean_earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon};
    const auto start = sim::start_mission(session, STARSIM_CONFIG_DIR, "hls_lunar_llo", {}, time);
    INFO(report::describe(start.refusal));
    REQUIRE(start.return_planned);
    CHECK(start.refusal.ok());
    const auto* plan = session.lunar_outbound_plan();
    REQUIRE(plan);
    REQUIRE(plan->llo_plane_normal_moon_inertial);
    const auto m = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_llo");
    const auto touchdown = *sim::lunar_planned_touchdown(m, time.from_utc(m.epoch_utc));
    const auto rotation = moon.fixed_to_gcrf(touchdown).rotation;
    const math::Vec3 pole{rotation(0, 2), rotation(1, 2), rotation(2, 2)};
    const double inclination = std::acos(std::clamp(dot(*plan->llo_plane_normal_moon_inertial, pole), -1.0, 1.0));
    std::printf("[lunar_mission] Mons Mouton LLO: i %.3f deg, TLI %.1f m/s, LOI %.1f m/s\n", inclination / constants::deg_to_rad,
                norm(plan->tli.delta_v_gcrf_mps), norm(plan->insertion.delta_v_gcrf_mps));
    CHECK(std::abs(inclination - 0.5 * constants::pi) < 1.0 * constants::deg_to_rad);
}

// Follow-up U-7: the start with the touchdown search on a background thread reports its status while the search runs
// and then gives the synchronous start (the same plan, the same epoch).
TEST_CASE("The deferred lunar landing start gives the synchronous start", "[sim][lunar][mission]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const auto moon = spice::SpiceOrientation::moon_mean_earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession deferred_session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon};
    auto pending = sim::start_mission_deferred(deferred_session, STARSIM_CONFIG_DIR, "hls_lunar_llo", {}, time);
    REQUIRE_FALSE(pending.done);  // the composed surface with the hazard check: the search runs on its thread
    CHECK(pending.status.code == report::Code::start_checking_landing_site);
    REQUIRE(pending.poll);
    REQUIRE(pending.wait);
    CHECK(deferred_session.active());  // placed before the search
    std::optional<sim::MissionStart> deferred;
    int polls = 0;
    for (; !deferred && polls < 60000; ++polls) {  // <= 10 min
        deferred = pending.poll(deferred_session);
        if (!deferred) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    REQUIRE(deferred);
    std::printf("[lunar_mission] deferred start: %d polls\n", polls);
    sim::FlightSession sync_session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon};
    const auto sync = sim::start_mission(sync_session, STARSIM_CONFIG_DIR, "hls_lunar_llo", {}, time);
    CAPTURE(report::describe(deferred->refusal), report::describe(sync.refusal));
    CHECK(deferred->return_planned == sync.return_planned);
    CHECK(report::same(deferred->refusal, sync.refusal));
    CHECK(deferred->epoch.tdb_seconds_past_j2000() == sync.epoch.tdb_seconds_past_j2000());
    CHECK(deferred_session.lunar_step() == sync_session.lunar_step());
    REQUIRE(deferred_session.lunar_outbound_plan());
    REQUIRE(sync_session.lunar_outbound_plan());
    CHECK(norm(deferred_session.lunar_outbound_plan()->tli.delta_v_gcrf_mps) ==
          norm(sync_session.lunar_outbound_plan()->tli.delta_v_gcrf_mps));
}

TEST_CASE("A parking orbit below the Moon's declination is refused", "[sim][lunar][mission]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const auto moon = spice::SpiceOrientation::moon_mean_earth();
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon};
    auto mission = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_llo");
    mission.parking_inclination_rad = 2.0 * constants::deg_to_rad;  // the Moon is at ~20 deg declination then
    const auto epoch = spice::SpiceTimeConverter().from_utc({2026, 11, 2, 0, 0, 0.0});
    CHECK_THROWS_WITH(sim::place_lunar_landing_mission(session, mission, STARSIM_CONFIG_DIR, epoch),
                      ContainsSubstring("lunar.mission_no_parking_plane"));
}

TEST_CASE("A lunar landing mission needs a session with the Moon as a reference body", "[sim][lunar][mission]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR};
    const auto mission = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_llo");
    const auto epoch = spice::SpiceTimeConverter().from_utc({2026, 11, 2, 0, 0, 0.0});
    CHECK_THROWS_WITH(sim::place_lunar_landing_mission(session, mission, STARSIM_CONFIG_DIR, epoch),
                      ContainsSubstring("lunar.landing_no_moon_orientation"));
}

TEST_CASE("Lunar landing missions load their descent, surface, ascent and budget sections", "[config][lunar]") {
    const auto m = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_llo");
    CHECK(m.pdi_altitude_above_site_m == 15e3);
    CHECK(m.braking_range_m == 450e3);
    CHECK(m.surface_stay_s > 0.0);
    CHECK(m.insertion_altitude_above_site_m == 15e3);
    CHECK(m.descent_delta_v_mps == 2100.0);
    CHECK(m.reserve_kg == 10000.0);
}

// The LLO variant closes with the 120 t HLS; the NRHO variant does not (user 2026-10-16, l4 Task 9: its flight ran dry
// on the LLO -> NRHO leg - 986 m/s after the ~4 d site wait, the cheap window closes ~2.5 d after the LLO arrival); the
// 204 t HLS closes neither.
TEST_CASE("Mission 1 closes in the LLO with the 120 t HLS; NRHO and 204 t are refused", "[sim][lunar][lunar_budget_mission]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    auto earth = spice::SpiceOrientation::earth();
    auto moon_me = spice::SpiceOrientation::moon_mean_earth();
    spice::SpiceTimeConverter time;
    for (const char* id : {"hls_lunar_llo", "hls_lunar_nrho"}) {
        const auto m = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, id);
        const bool closes = std::string(id) == "hls_lunar_llo";
        {
            sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
            const auto s = sim::start_lunar_landing_mission(session, STARSIM_CONFIG_DIR, m, time.from_utc(m.epoch_utc));
            CAPTURE(id, report::describe(s.refusal));
            CHECK(s.return_planned == closes);
            if (!closes) {
                CHECK(s.refusal.code == report::Code::lunar_mission_infeasible);
                // the NRHO stays as it is: the site is out of its reach on this budget (plan 2026-11-06 Task 7)
                REQUIRE(s.refusal.causes.size() == 1u);
                CHECK(s.refusal.causes.front().code == report::Code::lunar_nrho_site_unreachable);
            }
        }
        {
            auto heavy = config::load_vehicle(STARSIM_CONFIG_DIR, m.vehicle_id);
            heavy.stages.front().dry_mass_kg = 204e3;   // the independent bottom-up estimate (spec)
            sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
            const auto s = sim::start_lunar_landing_mission(session, STARSIM_CONFIG_DIR, m, time.from_utc(m.epoch_utc), &heavy);
            CAPTURE(id, report::describe(s.refusal));
            CHECK_FALSE(s.return_planned);
            CHECK(s.refusal.code == report::Code::lunar_mission_infeasible);
            CHECK(session.lunar_step() == sim::LunarStep::None);
        }
    }
}

// Ruling R12 (lunar sites plan): the NRHO stays as it is; a site the polar LLO the NRHO descent flies into does not
// pass within the wait costs a plane change, in the departure burn or in the LLO, whichever is cheaper. Hand check per
// site at the pass found (independent of mission::nrho_site_departure's vector formulation): (a) the turn about the
// perilune radius is the angle between the horizontal directions towards the pole (the polar plane) and towards the
// site; each burn by the law of cosines on the horizontal speeds (perilune and vis-viva aposelene speed) plus the radial
// speed removed, the polar burn subtracted (no credit below it); (b) in the LLO the site's angle from the polar plane,
// 2 v sin(di / 2). Budget: the extra leg on the vacuum Raptors scales the final mass by
// exp(-dv / (g0 Isp)) (rocket equation).
TEST_CASE("The NRHO budget charges the plane change to the site: shortfalls differ by site", "[sim][lunar][lunar_budget_mission]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    auto earth = spice::SpiceOrientation::earth();
    auto moon = spice::SpiceOrientation::moon_mean_earth();
    spice::SpiceTimeConverter time;
    const auto m = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_nrho");
    const auto epoch = time.from_utc(m.epoch_utc);
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon};
    sim::place_lunar_landing_mission(session, m, STARSIM_CONFIG_DIR, epoch);
    const auto outbound = sim::lunar_outbound_parameters(m, session.gravity(), session.ephemeris(), epoch);
    REQUIRE(session.plan_lunar_outbound(outbound));
    const auto& plan = *session.lunar_outbound_plan();
    const auto& v = session.body(session.controlled_index()).vehicle();
    const auto& sets = sim::lunar_landing_setup(m).guidance.braking_sets;
    const double isp = gnc::groups_vacuum_isp_s(v, sets.front()), approach_isp = gnc::groups_vacuum_isp_s(v, sets.back());
    const double mass = v.mass_properties().mass_kg, dry = mass - v.propellant_kg();
    const double gm = ephemeris.gm_m3ps2(ephemeris::Body::Moon);
    const double r_llo = constants::moon_mean_radius_m + 100e3;

    struct Hand {
        double extra_mps, turn_rad;
        bool in_llo;
    };
    struct Result {
        double extra_mps, short_kg, final_kg;
    };
    const auto evaluate = [&](const config::LunarSiteConfig& site) {
        const char* site_id = site.id.c_str();
        const auto d = sim::lunar_nrho_site_departure(m, outbound, plan, ephemeris, moon, site, epoch);
        REQUIRE(d);
        CHECK(d->departure > plan.insertion.epoch);
        const auto rel = mission::moon_relative(ephemeris, outbound.nrho->state_gcrf(d->departure), d->departure);
        const math::Vec3 r_hat = math::normalized(rel.position_m);
        const double r = norm(rel.position_m);
        const double v_apo = std::sqrt(gm * (2.0 / r - 2.0 / (r + r_llo)));  // vis-viva, a = (r + r_llo) / 2
        const double v_r = dot(rel.velocity_mps, r_hat);
        const math::Vec3 v_h_vec = rel.velocity_mps - r_hat * v_r;
        const double v_h = norm(v_h_vec);
        const math::Vec3 ahead = v_h_vec / v_h;
        const auto burn = [&](const math::Vec3& direction) {
            const double c = dot(direction, ahead);
            return std::sqrt(v_r * v_r + v_h * v_h + v_apo * v_apo - 2.0 * v_h * v_apo * c);
        };
        // the doc's numbers: perilune, coplanar burn, and the burn into the polar plane through the perilune point
        const auto r0 = moon.fixed_to_gcrf(d->departure).rotation;
        const math::Vec3 pole{r0(0, 2), r0(1, 2), r0(2, 2)};
        math::Vec3 to_pole = math::normalized(pole - r_hat * dot(pole, r_hat));
        if (dot(to_pole, ahead) < 0.0) to_pole = -to_pole;
        std::printf("[lunar_mission] NRHO perilune: lat %.1f deg, %.0f km up, %.0f m/s; departure coplanar %.0f m/s, into "
                    "the polar plane %.0f m/s (%.1f deg turn)\n",
                    std::asin(dot(r_hat, pole)) / constants::deg_to_rad, (r - constants::moon_mean_radius_m) / 1e3,
                    norm(rel.velocity_mps), burn(ahead), burn(to_pole),
                    std::acos(std::clamp(dot(to_pole, ahead), -1.0, 1.0)) / constants::deg_to_rad);
        const auto hand = [&](time::Epoch t) {
            const auto rt = moon.fixed_to_gcrf(t).rotation;
            const math::Vec3 s = rt * math::Vec3{std::cos(site.latitude_rad) * std::cos(site.longitude_rad),
                                                 std::cos(site.latitude_rad) * std::sin(site.longitude_rad),
                                                 std::sin(site.latitude_rad)};
            math::Vec3 to_site = math::normalized(s - r_hat * dot(s, r_hat));
            if (dot(to_site, to_pole) < 0.0) to_site = -to_site;  // turned, not reversed
            const double in_burn = std::max(burn(to_site) - burn(to_pole), 0.0);
            const double llo_turn = std::asin(std::abs(dot(math::normalized(cross(r_hat, pole)), s)));
            const double in_llo = 2.0 * std::sqrt(gm / r_llo) * std::sin(0.5 * llo_turn);
            if (in_llo < in_burn) return Hand{in_llo, llo_turn, true};
            return Hand{in_burn, std::acos(std::clamp(dot(to_site, to_pole), -1.0, 1.0)), false};
        };
        const auto at_pass = hand(d->touchdown);
        // the pass of least cost within the 4 d wait: no later than that, and no dearer than the first pass
        const double half_period = constants::pi * std::sqrt(std::pow(0.5 * (r + r_llo), 3) / gm);
        const time::Epoch first_pass = d->departure + half_period + sim::lunar_first_pass_after_arrival_s(m, gm);
        CHECK(d->touchdown >= first_pass);
        CHECK(d->touchdown <= first_pass + 4.0 * 86400.0 + 1.0);
        CHECK(d->extra_delta_v_mps <= hand(first_pass).extra_mps + 0.01);
        const auto budget = sim::lunar_mission_budget(m, plan, mass, dry, isp, approach_isp, session.settling_gas_bound_kg(),
                                                      d->extra_delta_v_mps);
        std::printf("[lunar_mission] NRHO -> LLO for %s: %s, turn %.2f deg (hand %.2f), +%.1f m/s (hand %.1f; first pass "
                    "%.1f), pass %.2f d after the first, short %.0f kg\n",
                    site_id, d->in_llo ? "in the LLO" : "in the departure burn", d->plane_change_rad / constants::deg_to_rad,
                    at_pass.turn_rad / constants::deg_to_rad, d->extra_delta_v_mps, at_pass.extra_mps,
                    hand(first_pass).extra_mps, (d->touchdown - first_pass) / 86400.0, -budget.margin_kg);
        CHECK(d->in_llo == at_pass.in_llo);
        CHECK_THAT(d->plane_change_rad, WithinAbs(at_pass.turn_rad, 1e-6));
        CHECK_THAT(d->extra_delta_v_mps, WithinAbs(at_pass.extra_mps, 0.01));
        return Result{d->extra_delta_v_mps, -budget.margin_kg, budget.final_mass_kg};
    };
    // the polar LLO reaches the south-pole site and Apollo 11 within the wait (the Moon turns them through its plane);
    // free sites on the equator off that longitude band
    const auto south = evaluate(config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2"));
    const auto apollo_11 = evaluate(config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_11"));
    const auto free_site = [](const char* id, double lon_deg) {
        config::LunarSiteConfig c;
        c.id = id;
        c.longitude_rad = lon_deg * constants::deg_to_rad;
        return c;
    };
    const auto west = evaluate(free_site("equator_45w", -45.0));
    // its antipode lies in the same planes through the Moon's centre: the same cost
    CHECK_THAT(evaluate(free_site("equator_135e", 135.0)).extra_mps, WithinAbs(west.extra_mps, 0.01));
    CHECK(south.extra_mps < 1.0);
    CHECK(apollo_11.extra_mps < 1.0);
    CHECK(west.extra_mps > 100.0);
    CHECK(west.short_kg - south.short_kg > 1000.0);  // site-dependent
    // the extra leg alone separates the two shortfalls: rocket equation on the vacuum Raptors
    const double hand_kg = south.final_kg * (1.0 - std::exp(-(west.extra_mps - south.extra_mps) /
                                                             (isp * constants::standard_gravity_mps2)));
    CHECK_THAT(west.short_kg - south.short_kg, WithinAbs(hand_kg, 1.0));
}

// Task 6c: from these epochs the Lambert first guess (aimed past one limb of the Moon) left the TLI Newton creeping along
// impacting arcs ("Kein Transfer zum Mond gefunden", 40 steps at 55 deg, -1,260 km periselene); the guess past the other
// limb converges. Both used to be refused.
TEST_CASE("The LLO outbound plan is found at the epochs where the first guess stalled", "[sim][lunar][mission]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    auto earth = spice::SpiceOrientation::earth();
    auto moon = spice::SpiceOrientation::moon_mean_earth();
    spice::SpiceTimeConverter time;
    auto m = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_llo");
    for (const int day : {12, 14}) {
        m.epoch_utc = {2026, 11, day, 0, 0, 0.0};
        sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon};
        const auto s = sim::start_lunar_landing_mission(session, STARSIM_CONFIG_DIR, m, time.from_utc(m.epoch_utc));
        CAPTURE(day, report::describe(s.refusal));
        CHECK(s.return_planned);
        CHECK(s.refusal.ok());
        REQUIRE(session.lunar_outbound_plan());
        CHECK_THAT(norm(session.lunar_outbound_plan()->tli.delta_v_gcrf_mps), WithinAbs(3.2e3, 0.2e3));  // [D] the same TLI as the other epochs (3,142-3,195 m/s in the probe)
    }
}

namespace {
// Mission 1 flown from the parking orbit to its end (one time-warp call) and checks how it ended.
void fly_mission_1(const char* id, double span_days, sim::LunarStep end_step) {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    auto earth = spice::SpiceOrientation::earth();
    auto moon_me = spice::SpiceOrientation::moon_mean_earth();
    spice::SpiceTimeConverter time;
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
    const auto s = sim::start_mission(session, STARSIM_CONFIG_DIR, id, {}, time);
    INFO(report::describe(s.refusal));
    REQUIRE(s.return_planned);
    const double m0 = session.body(0).vehicle().mass_properties().mass_kg;
    session.advance_to(s.epoch + span_days * 86400.0, 200'000'000);
    const auto& td = session.lunar_touchdown();
    INFO(report::describe(session.lunar_status()));
    REQUIRE(td);
    std::printf("[lunar_mission] %s: %s, touchdown %.2f m / %.2f m/s, %.1f t left of %.0f t (propellant %.1f t), %s\n",
                id, sim::id_of(td->outcome), td->offset_m, td->sink_mps,
                session.body(0).vehicle().mass_properties().mass_kg / 1000.0, m0 / 1000.0,
                session.body(0).vehicle().propellant_kg() / 1000.0, report::describe(session.lunar_status()).c_str());
    CHECK(td->outcome == sim::LegTouchdown::Landed);
    CHECK(td->offset_m < 1.0);
    CHECK(session.lunar_mission_complete());
    CHECK(session.lunar_step() == end_step);
}
}  // namespace

TEST_CASE("Mission 1, LLO variant: LEO to the lunar surface and back to the LLO", "[sim][lunar][lunar_mission_flight][terrain][data][slow]") {
    fly_mission_1("hls_lunar_llo", 40.0, sim::LunarStep::InLlo);
}

// Task 9 (lunar sites): the start options carry site, stay and window; the flight lands at the site, rests for the
// chosen stay (not the mission file's) and lifts off into the LLO.
TEST_CASE("Apollo 17 window 1 with a 72 h stay flies to touchdown, stays 72 h and lifts off into the LLO",
          "[sim][lunar][lunar_mission_flight][terrain][data][slow]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    auto earth = spice::SpiceOrientation::earth();
    auto moon_me = spice::SpiceOrientation::moon_mean_earth();
    spice::SpiceTimeConverter time;
    const auto site = config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_17");
    if (!sim::lunar_site_tiles_present(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR, site)) SKIP("Apollo 17 tile not installed");
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
    const double stay_s = 72.0 * 3600.0;
    const sim::LunarWindowRequest request{"hls_lunar_llo", site, stay_s, time.from_utc({2026, 11, 6, 0, 0, 0.0})};
    const auto found = sim::find_lunar_landing_windows(session, STARSIM_CONFIG_DIR, request, [](double) { return true; });
    if (found.refusal) INFO(report::describe(*found.refusal));
    REQUIRE_FALSE(found.windows.empty());
    const auto& w = found.windows.front();
    sim::StartOptions options;
    options.lunar = sim::LunarStartOverride{site, stay_s, w.mission_start, w.touchdown};
    const auto s = sim::start_mission(session, STARSIM_CONFIG_DIR, "hls_lunar_llo", {}, time, options);
    INFO(report::describe(s.refusal));
    REQUIRE(s.return_planned);
    CHECK(s.epoch == w.mission_start);
    const auto* placed = session.lunar_landing_site();
    REQUIRE(placed);
    CHECK_THAT(placed->config.latitude_rad, WithinAbs(site.latitude_rad, 1e-12));
    std::optional<time::Epoch> landed, lifted;
    for (time::Epoch t = w.mission_start; t < w.liftoff + 2.0 * 86400.0 && !(lifted && session.lunar_step() == sim::LunarStep::InLlo);
         t += 1200.0) {
        session.advance_to(t, 200'000'000);
        const auto step = session.lunar_step();
        if (!landed && (step == sim::LunarStep::Landed || step == sim::LunarStep::SurfaceStay)) landed = session.epoch();
        if (landed && !lifted && step == sim::LunarStep::Ascent) lifted = session.epoch();
    }
    INFO(report::describe(session.lunar_status()));
    REQUIRE(landed);
    REQUIRE(lifted);
    CHECK(std::abs(*landed - w.touchdown) < 3600.0);
    CHECK(std::abs((*lifted - *landed) - stay_s) < 1800.0);  // sampled every 20 min
    CHECK(session.lunar_touchdown()->outcome == sim::LegTouchdown::Landed);
    CHECK(session.lunar_step() == sim::LunarStep::InLlo);
}

// Probe for the mission epochs (plan l4 Task 9): STARSIM_LUNAR_MISSION=<id>, STARSIM_LUNAR_EPOCH_SHIFT_D=<days added
// to the TOML epoch>, STARSIM_LUNAR_SPAN_D=<days flown>; run with STARSIM_DEBUG_LUNAR_SESSION=1 and read the "descent
// plan" line. Finding: the descent wait hardly depends on the epoch (LLO 5.4-5.9 d for shifts -3..+6 d) - the LLO plane
// follows the arrival asymptote, fixed in the tidally locked Moon's frame. The NRHO variant is refused at the start.
TEST_CASE("Lunar mission epoch probe", "[.][lunar_epoch_probe]") {
    const char* id = std::getenv("STARSIM_LUNAR_MISSION");
    const char* shift = std::getenv("STARSIM_LUNAR_EPOCH_SHIFT_D");
    const char* span = std::getenv("STARSIM_LUNAR_SPAN_D");
    REQUIRE(id);
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    auto earth = spice::SpiceOrientation::earth();
    auto moon_me = spice::SpiceOrientation::moon_mean_earth();
    spice::SpiceTimeConverter time;
    const auto m = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, id);
    const auto epoch = time.from_utc(m.epoch_utc) + (shift ? std::atof(shift) : 0.0) * 86400.0;
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
    const auto s = sim::start_lunar_landing_mission(session, STARSIM_CONFIG_DIR, m, epoch);
    INFO(report::describe(s.refusal));
    REQUIRE(s.return_planned);
    session.advance_to(epoch + (span ? std::atof(span) : 10.0) * 86400.0, 200'000'000);
    std::printf("[epoch_probe] %s shift %s d: %s\n", id, shift ? shift : "0", report::describe(session.lunar_status()).c_str());
}
