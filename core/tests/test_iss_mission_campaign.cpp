// ISS i6 campaigns: the three missions through their start functions - orbit start to hard capture at both ports,
// the refusals, the pad mission end to end, the departure to splashdown.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

#include "rendezvous_test_support.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/iss_mission.hpp"
#include "starsim/sim/mission_start.hpp"
#include "test_support.hpp"

using namespace starsim;
using sim::IssMissionLeg;

namespace {
bool verbose() { return std::getenv("STARSIM_VERBOSE") != nullptr; }

struct Session {
    const spice::SpiceEphemeris ephemeris;
    const spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter clock;
    sim::FlightSession session{ephemeris, earth, rendezvous_test::config_dir};
    // the committed fixture stands in for data/iss_oem (git-ignored)
    const orbit::OemArchive archive = orbit::OemArchive::scan(rendezvous_test::fixture_dir, clock);
};

// Flies until `done` or `limit_s` (the sequence stops the coast at its leg changes).
template <class Done>
void fly(sim::FlightSession& s, Done done, double limit_s) {
    const auto start = s.epoch();
    while (!done() && s.epoch() - start < limit_s && s.iss_mission_leg() != IssMissionLeg::Ended) {
        s.advance_to(s.epoch() + 600.0, 10000000);
    }
}
}  // namespace

TEST_CASE("ISS mission: crew_dragon_iss from the window's separation to hard capture at IDA-2",
          "[iss][iss_mission][docking][slow]") {
    test::spice_session();
    Session f;
    const auto m = config::load_iss_rendezvous_mission(rendezvous_test::config_dir, "crew_dragon_iss");
    const auto wall = std::chrono::steady_clock::now();
    const auto st = sim::start_iss_rendezvous_mission(f.session, rendezvous_test::config_dir, m, f.archive, f.clock);
    const double planning_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall).count();
    INFO(st.refusal);
    REQUIRE(st.return_planned);
    REQUIRE(st.launch_window);
    CHECK(planning_s < 10.0);  // spec: a few seconds
    CHECK(st.epoch == st.launch_window->separation);
    const double kg0 = f.session.body(f.session.controlled_index()).vehicle().rcs_propellant_kg();
    fly(f.session, [&] { return f.session.iss_mission_leg() == IssMissionLeg::Docked; }, 30.0 * 3600.0);
    INFO(f.session.iss_mission_status());
    CHECK(f.session.iss_mission_leg() == IssMissionLeg::Docked);
    // the Dragon is held on the station now: its own tank still shows what the flight used
    const double used_kg = kg0 - f.session.body(f.session.controlled_index()).vehicle().rcs_propellant_kg();
    CHECK(f.session.epoch() - st.launch_window->liftoff < m.mission_limit_s + 3.0 * 3600.0);
    CHECK(used_kg < 2562.0 - 900.0);  // the deorbit + abort reserve stays (PhasingParameters::reserve_kg)
    if (verbose()) {
        std::printf("[iss_mission] crew_dragon_iss: planning %.1f s, docked %.1f h after liftoff, %.0f kg\n",
                    planning_s, (f.session.epoch() - st.launch_window->liftoff) / 3600.0, used_kg);
    }
}

TEST_CASE("ISS mission: crew_dragon_iss at IDA-3 with a set phase on the synthetic orbit",
          "[iss][iss_mission][docking][slow]") {
    test::spice_session();
    Session f;
    auto m = config::load_iss_rendezvous_mission(rendezvous_test::config_dir, "crew_dragon_iss");
    m.port = "ida3";
    m.iss_orbit = "synthetic";
    m.phase_rad = 40.0 * constants::deg_to_rad;
    const auto st = sim::start_iss_rendezvous_mission(f.session, rendezvous_test::config_dir, m, f.archive, f.clock);
    INFO(st.refusal);
    REQUIRE(st.return_planned);
    CHECK_FALSE(st.launch_window);
    CHECK(st.epoch == f.clock.from_utc(m.epoch_utc));
    fly(f.session, [&] { return f.session.iss_mission_leg() == IssMissionLeg::Docked; }, 30.0 * 3600.0);
    INFO(f.session.iss_mission_status());
    CHECK(f.session.iss_mission_leg() == IssMissionLeg::Docked);
    if (verbose()) {
        std::printf("[iss_mission] crew_dragon_iss IDA-3, 40 deg, synthetic: docked %.1f h after the start\n",
                    (f.session.epoch() - st.epoch) / 3600.0);
    }
}

TEST_CASE("ISS mission: refusals - OEM date outside the archive, unknown port", "[iss][iss_mission]") {
    test::spice_session();
    Session f;
    auto m = config::load_iss_rendezvous_mission(rendezvous_test::config_dir, "crew_dragon_iss");
    m.iss_orbit = "oem";
    m.epoch_utc = {2027, 3, 1, 0, 0, 0.0};  // beyond the fixture's window
    m.phase_rad = 30.0 * constants::deg_to_rad;
    CHECK_THROWS_AS(sim::start_iss_rendezvous_mission(f.session, rendezvous_test::config_dir, m, f.archive, f.clock),
                    std::runtime_error);
    CHECK(f.session.body_count() == 0);  // nothing placed (Review Focus 1)
    auto p = config::load_iss_rendezvous_mission(rendezvous_test::config_dir, "crew_dragon_iss");
    p.port = "ida9";
    CHECK_THROWS_AS(sim::start_iss_rendezvous_mission(f.session, rendezvous_test::config_dir, p, f.archive, f.clock),
                    std::runtime_error);
    CHECK(f.session.body_count() == 0);
}

TEST_CASE("ISS mission: start_mission dispatches crew_dragon_iss", "[iss][iss_mission]") {
    test::spice_session();
    Session f;
    const auto st = sim::start_mission(f.session, rendezvous_test::config_dir, "crew_dragon_iss", {}, f.clock);
    CHECK(st.kind == config::MissionKind::IssRendezvous);
    // data/iss_oem may be absent on a fresh checkout: "auto" then flies the synthetic orbit - either way planned
    INFO(st.refusal);
    CHECK(st.return_planned);
    CHECK(f.session.iss_mission_leg() == IssMissionLeg::Rendezvous);
    CHECK_FALSE(st.warnings.empty());  // at least the ISS source note
}

TEST_CASE("ISS mission: the window's separation estimate matches the flown ascent", "[iss][iss_mission][ascent][slow]") {
    test::spice_session();
    Session f;
    const auto m = config::load_iss_launch_mission(rendezvous_test::config_dir, "falcon9_crew_dragon_iss");
    const auto st = sim::start_iss_launch_mission(f.session, rendezvous_test::config_dir, m, f.archive, f.clock);
    INFO(st.refusal);
    REQUIRE(st.return_planned);
    REQUIRE(st.launch_window);
    const auto& w = *st.launch_window;
    auto& s = f.session;
    CHECK(s.iss_mission_leg() == IssMissionLeg::Ascent);
    // the pad at liftoff, for the flown in-plane arc
    const auto site = config::load_launch_site(rendezvous_test::config_dir, "ccsfs_slc40");
    const math::Vec3 pad = f.earth.fixed_to_gcrf(w.liftoff).position(
        frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m}));
    // fly to the separation (the sequence then plans the rendezvous on the next tick)
    while (s.iss_mission_leg() == IssMissionLeg::Ascent && s.epoch() - w.liftoff < 1800.0) {
        s.advance_to(s.epoch() + 1.0, 100000);
    }
    INFO(s.iss_mission_status());
    REQUIRE(s.iss_mission_leg() == IssMissionLeg::Rendezvous);
    const auto& dragon = s.body(s.controlled_index()).state();
    const double dt = s.epoch() - w.separation;
    const auto predicted = orbit::propagate_two_body(w.separation_state_gcrf, constants::earth_gm_m3ps2, dt);
    const math::Vec3 n_pred = math::normalized(cross(predicted.position_m, predicted.velocity_mps));
    const math::Vec3 n_flown = math::normalized(cross(dragon.position_gcrf_m, dragon.velocity_gcrf_mps));
    const double plane_deg = std::acos(std::clamp(dot(n_pred, n_flown), -1.0, 1.0)) / constants::deg_to_rad;
    const double phase_deg = std::acos(std::clamp(dot(math::normalized(predicted.position_m),
                                                      math::normalized(dragon.position_gcrf_m)), -1.0, 1.0)) /
                             constants::deg_to_rad;
    // the flown arc from the pad (projected into the flown plane) to the Dragon, back-dated to the separation epoch
    const math::Vec3 pad_in = math::normalized(pad - dot(pad, n_flown) * n_flown);
    const math::Vec3 r_hat = math::normalized(dragon.position_gcrf_m);
    const double arc_now = std::atan2(dot(cross(pad_in, r_hat), n_flown), dot(pad_in, r_hat));
    const double rate = norm(cross(dragon.position_gcrf_m, dragon.velocity_gcrf_mps)) /
                        dot(dragon.position_gcrf_m, dragon.position_gcrf_m);
    const double flown_arc_deg = (arc_now - rate * dt) / constants::deg_to_rad;
    const auto e = orbit::elements_from_state({dragon.position_gcrf_m, dragon.velocity_gcrf_mps},
                                              constants::earth_gm_m3ps2);
    if (verbose()) {
        std::printf("[iss_mission] separation %.1f s after liftoff (planned %.1f), arc %.3f deg (planned %.3f), "
                    "plane %.4f deg, phase %.3f deg, orbit %.1f x %.1f km\n",
                    s.epoch() - w.liftoff, w.separation - w.liftoff, flown_arc_deg,
                    m.window.insertion_arc_rad / constants::deg_to_rad, plane_deg, phase_deg,
                    (e.semi_major_axis_m * (1.0 - e.eccentricity) - constants::wgs84_semi_major_axis_m) / 1000.0,
                    (e.semi_major_axis_m * (1.0 + e.eccentricity) - constants::wgs84_semi_major_axis_m) / 1000.0);
    }
    CHECK(std::abs(s.epoch() - w.separation) < 30.0);
    CHECK(plane_deg < 0.05);  // [E] the PEG plane: 0.05 deg ~ 7 m/s of plane change, removed by the coelliptic burn
    CHECK(phase_deg < 0.5);   // [E] 0.5 deg ~ 2 min of catch-up: well inside the schedule's slack
}

TEST_CASE("ISS mission: falcon9_crew_dragon_iss from the pad to hard capture", "[iss][iss_mission][docking][slow]") {
    test::spice_session();
    Session f;
    const auto m = config::load_iss_launch_mission(rendezvous_test::config_dir, "falcon9_crew_dragon_iss");
    const auto st = sim::start_iss_launch_mission(f.session, rendezvous_test::config_dir, m, f.archive, f.clock);
    INFO(st.refusal);
    REQUIRE(st.launch_window);
    CHECK(st.epoch == st.launch_window->liftoff);
    CHECK(st.site_id == "ccsfs_slc40");
    auto& s = f.session;
    // the ascent on the tick (1 s calls), then the coast-capable rest
    while (s.iss_mission_leg() == IssMissionLeg::Ascent && s.epoch() - st.epoch < 1800.0) {
        s.advance_to(s.epoch() + 1.0, 100000);
    }
    const double kg0 = s.body(s.controlled_index()).vehicle().rcs_propellant_kg();
    fly(s, [&] { return s.iss_mission_leg() == IssMissionLeg::Docked; }, 30.0 * 3600.0);
    INFO(s.iss_mission_status());
    CHECK(s.iss_mission_leg() == IssMissionLeg::Docked);
    CHECK(s.epoch() - st.launch_window->liftoff < m.mission_limit_s + 3.0 * 3600.0);
    if (verbose()) {
        std::printf("[iss_mission] pad -> docked %.1f h after liftoff, %zu rendezvous burns, %d re-plans, %.0f kg\n",
                    (s.epoch() - st.launch_window->liftoff) / 3600.0, s.rendezvous_burns().size(),
                    s.rendezvous_replans(), kg0 - s.body(s.controlled_index()).vehicle().rcs_propellant_kg());
    }
}

TEST_CASE("ISS mission: no launch window - refused, nothing launched", "[iss][iss_mission][launch_window]") {
    test::spice_session();
    Session f;
    auto m = config::load_iss_launch_mission(rendezvous_test::config_dir, "falcon9_crew_dragon_iss");
    m.mission_limit_s = m.window.insertion_s + 2.0 * 3600.0;
    m.window.search_s = 86400.0;
    const auto st = sim::start_iss_launch_mission(f.session, rendezvous_test::config_dir, m, f.archive, f.clock);
    CHECK_FALSE(st.return_planned);
    CHECK(st.refusal.find("Kein Startfenster") != std::string::npos);
    CHECK(f.session.iss_mission_leg() == IssMissionLeg::None);
    CHECK(f.session.body_count() == 0);
}
