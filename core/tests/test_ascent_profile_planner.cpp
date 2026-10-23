// Pre-launch ascent planner (ascent trajectory and staging, Task 3): the first-stage pitch kick chosen by trial flights
// for the most upper-stage propellant at SECO, with the booster's return planned at MECO.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/booster_return.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/ascent_mission.hpp"
#include "starsim/sim/ascent_profile_planner.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;

time::Epoch liftoff() {
    test::spice_session();
    return spice::SpiceTimeConverter{}.from_utc({2026, 10, 12, 12, 0, 0.0});
}

const spice::SpiceEphemeris& ephemeris() {
    test::spice_session();
    static const spice::SpiceEphemeris e;
    return e;
}

const frames::BodyOrientation& earth() {
    test::spice_session();
    static const auto e = spice::SpiceOrientation::earth();
    return e;
}

sim::FlightSession& like_session() {
    static sim::FlightSession session(ephemeris(), earth(), STARSIM_CONFIG_DIR);
    return session;
}

// Starship (Block 3) from Starbase to 200 km / 26.5 deg (mission starship_leo), the booster caught at the tower.
// `extra_payload_kg`: a test that asserts trial flights gets a memo key of its own (the memo lives as long as the
// process: in a single-process run another test may already have planned the plain request).
sim::AscentProfileRequest starship_request(double extra_payload_kg = 0.0) {
    const auto m = config::load_ascent_mission(STARSIM_CONFIG_DIR, "starship_leo");
    sim::AscentProfileRequest r;
    r.vehicle_id = m.vehicle_id;
    r.site_id = m.site_id;
    r.heading_rad = m.heading_rad;
    r.liftoff = liftoff();
    r.target = {m.periapsis_height_m, m.apoapsis_height_m, m.inclination_rad, m.northerly};
    r.payload_kg = m.payload_kg + extra_payload_kg;
    r.fairing = m.fairing;
    return r;
}

// Falcon 9 with the payload of ascent mission `mission_id` (its site, target, payload; `extra_payload_kg`: an own memo
// key, see starship_request).
sim::AscentProfileRequest falcon9_request(const char* mission_id, double extra_payload_kg = 0.0) {
    const auto m = config::load_ascent_mission(STARSIM_CONFIG_DIR, mission_id);
    sim::AscentProfileRequest r;
    r.vehicle_id = m.vehicle_id;
    r.site_id = m.site_id;
    r.heading_rad = m.heading_rad;
    r.liftoff = liftoff();
    r.target = {m.periapsis_height_m, m.apoapsis_height_m, m.inclination_rad, m.northerly};
    r.payload_kg = m.payload_kg + extra_payload_kg;
    r.fairing = m.fairing;
    r.payload = m.payload;
    return r;
}

sim::AscentProfile timed_plan(const sim::AscentProfileRequest& request, bool launch = false) {
    const auto start = std::chrono::steady_clock::now();
    const auto plan = launch ? sim::plan_launch(like_session(), request) : sim::plan_ascent_profile(like_session(), request);
    const double wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    std::printf("[ascent_planner] kick %.2f deg, upper-stage propellant at SECO %.1f t, %d trial flights, %.1f s%s%s\n",
                plan.kick_angle_rad / deg, plan.upper_propellant_at_seco_kg / 1e3, plan.trial_flights, wall_s,
                plan.feasible ? "" : " - infeasible: ", plan.reason.c_str());
    if (launch) {
        std::printf("[ascent_planner] recovery %d '%s' at %.4f / %.4f deg, heading %.1f deg; inclination %.3f deg\n",
                    static_cast<int>(plan.recovery.kind), plan.recovery.zone_or_ship_id.c_str(),
                    plan.recovery.latitude_rad / deg, plan.recovery.longitude_rad / deg, plan.recovery.heading_rad / deg,
                    plan.target.inclination_rad / deg);
    }
    return plan;
}

bool starts_with(const std::string& text, const std::string& prefix) { return text.rfind(prefix, 0) == 0; }

// The first stage's impact on the planned launch, flown independently (the full return guidance on, the drone ship at
// its chosen station) and predicted as config/missions/falcon9_asds.toml derives it: 10 s after the separation, drag and
// the entry burn, no steering. Returns the horizontal distance to the ship (m).
double impact_miss_of_ship_m(const sim::AscentProfileRequest& request, const sim::AscentProfile& plan) {
    sim::FlightSession session(ephemeris(), earth(), STARSIM_CONFIG_DIR);
    const auto stack = sim::ascent_stack(config::load_vehicle(STARSIM_CONFIG_DIR, request.vehicle_id), STARSIM_CONFIG_DIR,
                                         request.payload_kg, request.fairing, request.payload);
    session.launch_from_pad(stack, sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, request.site_id)),
                            request.heading_rad, request.liftoff);
    const auto ship = sim::recovery_landing_site(STARSIM_CONFIG_DIR, plan.recovery);
    REQUIRE(ship);
    REQUIRE(ship->deck);
    session.set_landing_zone(*ship);
    session.set_booster_return(true);
    gnc::PitchProgram program;
    program.kick_angle_rad = plan.kick_angle_rad;
    session.engage_ascent_autopilot(plan.target, program);
    for (double t = 0.0; t < 300.0 && session.body_count() < 2; t += 0.05) session.advance_to(request.liftoff + t, 100);
    REQUIRE(session.body_count() == 2);
    session.advance_to(session.epoch() + sim::drone_ship_prediction_after_separation_s, 4000);
    const auto& stage = session.body(0);
    const auto p = session.return_parameters(stage.vehicle().stages().front().config);
    const auto burn = gnc::BoosterReturnGuidance::entry_burn_ahead(stage.vehicle(), p);
    const auto impact = gnc::BoosterReturnGuidance::predict_impact(stage, session.environment(), session.epoch(),
                                                                   ship->height_m, nullptr, &burn);
    REQUIRE(impact.valid);
    const auto deck = frames::geodetic_to_fixed({ship->latitude_rad, ship->longitude_rad, ship->height_m});
    const auto up = math::normalized(deck);
    const auto d = impact.position_itrf_m - deck;
    const double miss_m = norm(d - dot(d, up) * up);
    const auto geo = frames::fixed_to_geodetic(impact.position_itrf_m);
    std::printf("[ascent_planner] flown impact %.4f / %.4f deg: %.2f km from the ship\n", geo.latitude_rad / deg,
                geo.longitude_rad / deg, miss_m / 1e3);
    return miss_m;
}

}  // namespace

TEST_CASE("The planner's kick is the optimum of a kick sweep", "[sim][ascent_planner][slow]") {
    const auto request = starship_request(100.0);  // own memo key
    struct Point {
        double kick_deg, propellant_kg;
    };
    std::vector<Point> sweep;
    for (double k = 6.0; k <= 22.0 + 1e-9; k += 1.0) {
        const double p = sim::fly_ascent_trial(like_session(), request, k * deg);
        std::printf("[ascent_planner] sweep kick %4.1f deg: %.1f t\n", k, p / 1e3);
        sweep.push_back({k, p});
    }
    const auto best = std::max_element(sweep.begin(), sweep.end(), [](const Point& a, const Point& b) {
        return std::isnan(a.propellant_kg) || (!std::isnan(b.propellant_kg) && a.propellant_kg < b.propellant_kg);
    });
    REQUIRE(!std::isnan(best->propellant_kg));
    REQUIRE(best != sweep.begin());
    REQUIRE(best + 1 != sweep.end());
    // The sweep's optimum: the best 1-deg point refined at 0.2 deg (+-1 deg around it).
    Point fine = *best;
    for (double k = best->kick_deg - 0.8; k <= best->kick_deg + 0.8 + 1e-9; k += 0.2) {
        if (std::abs(k - best->kick_deg) < 1e-9) continue;
        const double p = sim::fly_ascent_trial(like_session(), request, k * deg);
        std::printf("[ascent_planner] fine kick %4.1f deg: %.3f t\n", k, p / 1e3);
        if (!std::isnan(p) && p > fine.propellant_kg) fine = {k, p};
    }

    const auto plan = timed_plan(request);
    REQUIRE(plan.feasible);
    CHECK(plan.trial_flights > 0);
    CHECK(std::abs(plan.kick_angle_rad / deg - fine.kick_deg) <= 0.2 + 1e-9);
    for (const auto& p : sweep) {
        if (!std::isnan(p.propellant_kg)) CHECK(plan.upper_propellant_at_seco_kg >= p.propellant_kg - 1000.0);
    }
}

TEST_CASE("The optimised arc has no kink at staging", "[sim][ascent_planner]") {
    const auto request = starship_request();
    const auto plan = timed_plan(request);
    REQUIRE(plan.feasible);

    // the planned kick flown as a mission
    const auto mission = config::load_ascent_mission(STARSIM_CONFIG_DIR, "starship_leo");
    auto& session = like_session();
    REQUIRE(sim::start_ascent_mission(session, mission, STARSIM_CONFIG_DIR, request.liftoff).kick_angle_rad ==
            plan.kick_angle_rad);
    std::optional<double> meco_fpa_rad, separation_s;
    double worst_below_rad = -1e9;  // flight-path angle - pitch, worst in the first 60 s after the separation
    for (double t = 0.5; t < 600.0; t += 0.5) {
        session.advance_to(request.liftoff + t, 1000);
        const auto state = session.autopilot().state;
        const auto tel = session.telemetry(session.controlled_index());
        if (!meco_fpa_rad && session.mission_time_s() > 10.0 && state != sim::AutopilotState::Booster) {
            meco_fpa_rad = tel.flight_path_angle_rad;  // the first tick after MECO (state Staging)
        }
        if (!separation_s && state == sim::AutopilotState::UpperStage && session.body_count() > 1) {
            separation_s = session.mission_time_s();
        }
        if (separation_s && session.mission_time_s() <= *separation_s + 60.0) {
            const auto& s = session.body(session.controlled_index()).state();
            const auto axis = s.attitude.rotate(math::Vec3{1.0, 0.0, 0.0});
            const double pitch_rad = std::asin(dot(axis, math::normalized(s.position_gcrf_m)));
            worst_below_rad = std::max(worst_below_rad, tel.flight_path_angle_rad - pitch_rad);
        }
        if (separation_s && session.mission_time_s() > *separation_s + 60.0) break;
    }
    REQUIRE(meco_fpa_rad);
    REQUIRE(separation_s);
    std::printf("[ascent_planner] MECO flight-path angle %.1f deg; upper stage at most %.1f deg below it in the first 60 s\n",
                *meco_fpa_rad / deg, worst_below_rad / deg);
    CHECK(*meco_fpa_rad > 10.0 * deg);  // [E]
    CHECK(*meco_fpa_rad < 40.0 * deg);  // [E]
    CHECK(worst_below_rad <= 15.0 * deg);  // [E] no kink (before: 50-70 deg below the velocity at a 64 deg MECO)
}

TEST_CASE("A repeated request is answered from the memo", "[sim][ascent_planner]") {
    auto request = starship_request(200.0);  // own memo key
    const auto first = timed_plan(request);
    REQUIRE(first.feasible);
    REQUIRE(first.trial_flights > 0);
    request.liftoff = request.liftoff + 600.0;  // a later start of the same launch
    const auto second = timed_plan(request);
    CHECK(second.trial_flights == 0);
    CHECK(second.feasible);
    CHECK(second.kick_angle_rad == first.kick_angle_rad);
    CHECK(second.upper_propellant_at_seco_kg == first.upper_propellant_at_seco_kg);

    // the same request from a session with another gravity model is planned anew (the trials fly that model)
    environment::PointMassGravityConfig no_j2;
    no_j2.earth_j2 = false;
    const spice::SpiceEphemeris other_ephemeris;  // ("ephemeris" names the namespace here)
    sim::FlightSession other(other_ephemeris, earth(), STARSIM_CONFIG_DIR, no_j2);
    const auto third = sim::plan_ascent_profile(other, request);
    CHECK(third.trial_flights > 0);
    CHECK(third.feasible);
}

// Task 4: the onboard computer chooses the booster recovery, refuses unreachable launches and flies due east when the
// target inclination is below the site's latitude.

TEST_CASE("Falcon 9 from SLC-40 to the ISS plane returns to LZ-40", "[sim][ascent_planner]") {
    const auto request = falcon9_request("falcon9_rtls");  // CRS-35 class: 12.5 t, no fairing, 200 km / 51.6 deg
    const auto plan = timed_plan(request, true);
    REQUIRE(plan.feasible);
    CHECK(plan.recovery.kind == sim::RecoveryKind::LandingZone);
    CHECK(plan.recovery.zone_or_ship_id == "ccsfs_lz40");
    const auto lz = config::load_launch_site(STARSIM_CONFIG_DIR, "ccsfs_lz40");
    CHECK(plan.recovery.latitude_rad == lz.latitude_rad);
    CHECK(plan.recovery.longitude_rad == lz.longitude_rad);
    CHECK(plan.target.inclination_rad == request.target.inclination_rad);  // above the site's latitude: unchanged
    CHECK_FALSE(plan.booster_impact);
}

TEST_CASE("Falcon 9 with a Starlink-class payload to 53 deg lands on the drone ship at its predicted impact point",
          "[sim][ascent_planner][slow]") {
    const auto request = falcon9_request("falcon9_asds");  // 16.5 t to 200 x 280 km / 53 deg from SLC-40
    const auto plan = timed_plan(request, true);
    REQUIRE(plan.feasible);
    // SLC-40 has LZ-40, but the RTLS return leaves the upper stage short of this orbit
    CHECK(plan.recovery.kind == sim::RecoveryKind::DroneShip);
    CHECK(plan.recovery.zone_or_ship_id == sim::default_drone_ship_id);
    REQUIRE(plan.booster_impact);
    CHECK(plan.recovery.latitude_rad == plan.booster_impact->latitude_rad);
    CHECK(plan.recovery.longitude_rad == plan.booster_impact->longitude_rad);
    // [E] the ground track north-east (launch azimuth ~43 deg for 53 deg from Florida)
    CHECK(std::abs(plan.recovery.heading_rad - 45.0 * deg) < 15.0 * deg);
    CHECK(impact_miss_of_ship_m(request, plan) < 2000.0);  // [E] brief: within 2 km of the no-boostback impact
}

TEST_CASE("Falcon 9 from a site without a landing zone uses the drone ship", "[sim][ascent_planner]") {
    auto request = falcon9_request("falcon9_rtls");
    request.site_id = "ksc_lc39a";  // a pad without a landing zone (its tower catches no Falcon 9)
    const auto plan = timed_plan(request, true);
    REQUIRE(plan.feasible);
    CHECK(plan.recovery.kind == sim::RecoveryKind::DroneShip);
    CHECK(plan.recovery.zone_or_ship_id == sim::default_drone_ship_id);
    REQUIRE(plan.booster_impact);
    CHECK(plan.recovery.latitude_rad == plan.booster_impact->latitude_rad);
}

TEST_CASE("Starship at a site without a tower is refused", "[sim][ascent_planner]") {
    auto request = starship_request();
    request.site_id = "ccsfs_slc40";
    const auto plan = timed_plan(request, true);
    CHECK_FALSE(plan.feasible);
    CHECK(plan.reason == "Kein Fangturm am Startplatz.");
    CHECK(plan.trial_flights == 0);
}

TEST_CASE("An unreachable target is refused", "[sim][ascent_planner]") {
    auto request = starship_request(300.0);  // own memo key
    request.target.periapsis_altitude_m = 2000e3;
    request.target.apoapsis_altitude_m = 2000e3;
    const auto plan = timed_plan(request, true);
    CHECK_FALSE(plan.feasible);
    CHECK(starts_with(plan.reason, "Ziel mit Booster-Rückkehr nicht erreichbar"));
    CHECK(plan.trial_flights > 0);
}

TEST_CASE("An inclination below the site latitude flies due east", "[sim][ascent_planner]") {
    auto request = starship_request();
    request.site_id = "ksc_lc39a";
    request.target.inclination_rad = 26.5 * deg;  // starship_leo's Starbase plane: below LC-39A's 28.6 deg
    const auto site = config::load_launch_site(STARSIM_CONFIG_DIR, "ksc_lc39a");
    const auto plan = timed_plan(request, true);
    REQUIRE(plan.feasible);
    CHECK(plan.target.inclination_rad == std::abs(site.latitude_rad));
    CHECK(plan.recovery.kind == sim::RecoveryKind::TowerCatch);
    CHECK(plan.recovery.zone_or_ship_id == "ksc_lc39a");
}

// Task 5: every launch from a pad goes through the planner - mission starts, mission chains and the free launch.

namespace {

// The sources of like_session() (the test cases below sit outside this namespace, where `ephemeris` names a namespace).
const spice::SpiceEphemeris& test_ephemeris() { return ephemeris(); }

std::string file_text(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

// A config dir holding only missions/starship_leo.toml: the configured file with `insert` after its "[mission]" line
// and `append` at its end (loader tests; a directory per case: ctest runs cases in parallel).
std::filesystem::path changed_starship_leo(const std::string& name, const std::string& insert, const std::string& append) {
    auto text = file_text(std::filesystem::path(STARSIM_CONFIG_DIR) / "missions" / "starship_leo.toml");
    const std::string header = "[mission]\n";
    const auto at = text.find(header);
    REQUIRE(at != std::string::npos);
    text.insert(at + header.size(), insert);
    const auto dir = std::filesystem::temp_directory_path() / ("starsim_ascent_planner_" + name);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "missions");
    std::ofstream(dir / "missions" / "starship_leo.toml", std::ios::binary) << text << append;
    return dir;
}

}  // namespace

TEST_CASE("Mission starts plan their ascent; no mission file carries a kick", "[sim][ascent_planner]") {
    // No ascent mission carries a profile or a recovery: the onboard computer plans both at the start.
    int ascents = 0;
    for (const auto& entry : std::filesystem::directory_iterator(std::filesystem::path(STARSIM_CONFIG_DIR) / "missions")) {
        const auto id = entry.path().stem().string();
        if (entry.path().extension() != ".toml") continue;
        const auto text = file_text(entry.path());
        // leo_ascent.toml (the free launch's target) is no mission: no [mission] table
        if (text.find("[mission]") == std::string::npos ||
            config::mission_kind(STARSIM_CONFIG_DIR, id) != config::MissionKind::Ascent) {
            continue;
        }
        ++ascents;
        INFO(id);
        for (const char* key : {"[pitch_program]", "kick_deg", "booster_throttle", "[drone_ship]", "landing_zone"}) {
            CHECK(text.find(key) == std::string::npos);
        }
        CHECK_NOTHROW(config::load_ascent_mission(STARSIM_CONFIG_DIR, id));
    }
    CHECK(ascents >= 7);
    // ... and the loader refuses them (a stale file would otherwise fly a kick nobody planned)
    CHECK_THROWS(config::load_ascent_mission(changed_starship_leo("kick", "", "\n[pitch_program]\nkick_deg = 14.0\n"),
                                             "starship_leo"));
    CHECK_THROWS(config::load_ascent_mission(
        changed_starship_leo("throttle", "booster_throttle = [[0.0, 0.8]]\n", ""), "starship_leo"));
    CHECK_THROWS(config::load_ascent_mission(changed_starship_leo("zone", "landing_zone = \"ccsfs_lz40\"\n", ""),
                                             "starship_leo"));
    CHECK_THROWS(config::load_ascent_mission(
        changed_starship_leo("ship", "", "\n[drone_ship]\nid = \"asds_atlantic\"\n"), "starship_leo"));

    // A mission start flies the planned kick with the chosen recovery (Falcon 9 RTLS: LZ-40).
    auto& session = like_session();
    const auto rtls = config::load_ascent_mission(STARSIM_CONFIG_DIR, "falcon9_rtls");
    const auto started = sim::start_ascent_mission(session, rtls, STARSIM_CONFIG_DIR, liftoff());
    REQUIRE(started.feasible);
    const auto plan = timed_plan(falcon9_request("falcon9_rtls"), true);  // the same request: from the memo
    CHECK(started.kick_angle_rad == plan.kick_angle_rad);
    REQUIRE(session.ascent_guidance());
    CHECK(session.ascent_guidance()->program().kick_angle_rad == plan.kick_angle_rad);
    CHECK(session.autopilot().state == sim::AutopilotState::PreLaunch);
    const auto lz = config::load_launch_site(STARSIM_CONFIG_DIR, "ccsfs_lz40");
    REQUIRE(session.landing_zone());
    CHECK(session.landing_zone()->latitude_rad == lz.latitude_rad);
    CHECK(session.landing_zone()->longitude_rad == lz.longitude_rad);

    // A refused plan refuses the start: the stack stays on the pad, the autopilot off.
    auto moved = config::load_ascent_mission(STARSIM_CONFIG_DIR, "starship_leo");
    moved.site_id = "ccsfs_slc40";  // no tower to catch Super Heavy
    const auto refused = sim::start_ascent_mission(session, moved, STARSIM_CONFIG_DIR, liftoff());
    CHECK_FALSE(refused.feasible);
    CHECK(refused.reason == "Kein Fangturm am Startplatz.");
    CHECK(session.autopilot().state == sim::AutopilotState::Off);
    CHECK(session.body(session.controlled_index()).clamped());
}

TEST_CASE("An ascent mission's [propellant] loads the named stages", "[sim][ascent_planner][mission]") {
    // the validation mission flight14_ascent carries flight 14's booster load (gauge 96.8 % of 3,650 t)
    const auto flight14 = config::load_ascent_mission(STARSIM_CONFIG_DIR, "flight14_ascent");
    REQUIRE(flight14.propellant_load_kg.size() == 1);
    CHECK(flight14.propellant_load_kg[0].first == "booster");
    CHECK(flight14.propellant_load_kg[0].second == 3533.2e3);
    CHECK(config::load_ascent_mission(STARSIM_CONFIG_DIR, "starship_leo").propellant_load_kg.empty());  // full
    CHECK_THROWS(config::load_ascent_mission(changed_starship_leo("load_negative", "", "\n[propellant]\nbooster = -1.0\n"),
                                             "starship_leo"));
    // a stage the vehicle does not have, or more than its tanks hold: refused before anything is planned
    sim::FlightSession session(test_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    auto mission = config::load_ascent_mission(STARSIM_CONFIG_DIR, "starship_leo");
    mission.propellant_load_kg = {{"nose", 1.0e3}};
    CHECK_THROWS_AS(sim::start_ascent_mission(session, mission, STARSIM_CONFIG_DIR, liftoff()), std::invalid_argument);
    mission.propellant_load_kg = {{"booster", 3651.0e3}};
    CHECK_THROWS_AS(sim::start_ascent_mission(session, mission, STARSIM_CONFIG_DIR, liftoff()), std::invalid_argument);
}

TEST_CASE("A free launch holds on the pad until the plan is in", "[sim][ascent_planner]") {
    sim::FlightSession session(test_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    const auto t0 = liftoff();
    const auto request = starship_request();
    session.launch_from_pad(request.vehicle_id,
                            sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, request.site_id)),
                            request.heading_rad, t0);
    session.schedule_liftoff(t0 + session.liftoff_ignition_lead_s());  // T-0 as early as the engines allow
    // The countdown passes T-0 before the plan is in (the game engages the autopilot once the launch command is done):
    // the stack holds on the pad - no ignition on the default pitch program.
    session.advance_to(t0 + 20.0);
    REQUIRE(session.body(0).clamped());
    CHECK(session.autopilot().state == sim::AutopilotState::Off);
    CHECK_FALSE(session.body(0).vehicle().any_engine_producing_thrust());

    const auto planned_at = session.epoch();
    const auto reason = session.engage_ascent_autopilot(request.target);  // plans on the clamped stack
    REQUIRE(reason.empty());
    CHECK(session.epoch() == planned_at);  // the countdown held while the computer planned
    // the free launch flies the vehicle file as configured: its own payload, fairing kept
    auto as_placed = request;
    as_placed.payload_kg = config::load_vehicle(STARSIM_CONFIG_DIR, request.vehicle_id).stages.back().payload_mass_kg;
    as_placed.fairing = true;
    const auto plan = timed_plan(as_placed, true);  // from the memo
    REQUIRE(plan.feasible);
    REQUIRE(session.ascent_guidance());
    CHECK(session.ascent_guidance()->program().kick_angle_rad == plan.kick_angle_rad);
    CHECK(session.ascent_guidance()->program().kick_angle_rad != gnc::PitchProgram{}.kick_angle_rad);

    // liftoff once the plan is in: the engines start now and the clamps open when they run
    std::optional<time::Epoch> lifted;
    for (double t = 0.05; t < 10.0 && !lifted; t += 0.05) {
        session.advance_to(planned_at + t, 100);
        if (!session.body(0).clamped()) lifted = session.epoch();
    }
    REQUIRE(lifted);
    CHECK(*lifted - planned_at >= session.liftoff_ignition_lead_s() - sim::FlightSession::liftoff_hold_margin_s - 1e-6);

    // A free launch the computer refuses: the reason, the stack stays on the pad with the countdown called off.
    sim::FlightSession refused(test_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    refused.launch_from_pad(request.vehicle_id,
                            sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "ccsfs_slc40")),
                            request.heading_rad, t0);
    refused.schedule_liftoff(t0 + 10.0);
    CHECK(refused.engage_ascent_autopilot(request.target) == "Kein Fangturm am Startplatz.");
    CHECK(refused.autopilot().state == sim::AutopilotState::Off);
    CHECK_FALSE(refused.scheduled_liftoff());
}
