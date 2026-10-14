// Crew Dragon under its parachutes in the flight session (d4 of docs/superpowers/specs/2026-10-14-rcs-crew-dragon-design.md):
// from the drogue gate to the water; review-focus cases; the return mission from orbit with the published timeline.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

#include "starsim/config/mission_config.hpp"
#include "starsim/config/splashdown_zone_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/parachute_sequencer.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/mat3.hpp"
#include "starsim/sim/capsule_return_mission.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/landing_outcome.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;

const spice::SpiceEphemeris& spice_ephemeris() {
    test::spice_session();
    static const spice::SpiceEphemeris e;
    return e;
}

const frames::BodyOrientation& spice_earth() {
    test::spice_session();
    static const auto e = spice::SpiceOrientation::earth();
    return e;
}

config::VehicleConfig capsule_config(double propellant_kg = 1400.0) {
    auto c = config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon");
    c.stages.erase(c.stages.begin());
    for (auto& t : c.stages.front().tanks) t.mass_kg = propellant_kg;
    return c;
}

// The capsule `height_m` above the zone centre, `speed_mps` relative to the Earth, `gamma_rad` below the horizontal
// heading north, heat shield first (body -x along the motion).
vehicle::RigidBodyState falling_state(const config::SplashdownZoneConfig& zone, double height_m, double speed_mps,
                                      double gamma_rad, time::Epoch epoch) {
    const Vec3 r = frames::geodetic_to_fixed({zone.latitude_rad, zone.longitude_rad, height_m});
    const Vec3 up = math::normalized(r);
    const Vec3 east{-std::sin(zone.longitude_rad), std::cos(zone.longitude_rad), 0.0};
    const Vec3 north = cross(up, east);
    const Vec3 v_rel = (north * std::cos(gamma_rad) - up * std::sin(gamma_rad)) * speed_mps;
    const auto t = spice_earth().fixed_to_gcrf(epoch);
    vehicle::RigidBodyState s;
    s.position_gcrf_m = t.rotation * r;
    s.velocity_gcrf_mps = t.rotation * v_rel + cross(vehicle::earth_rotation_vector(t), s.position_gcrf_m);
    const Vec3 x = math::normalized(t.rotation * v_rel) * -1.0;  // body +x against the motion
    const Vec3 y = math::normalized(cross(t.rotation * east, x));
    const Vec3 z = cross(x, y);
    math::Mat3 m;
    const Vec3 columns[3] = {x, y, z};
    for (int c = 0; c < 3; ++c) {
        m(0, c) = columns[c].x;
        m(1, c) = columns[c].y;
        m(2, c) = columns[c].z;
    }
    s.attitude = math::Quat::from_matrix(m).normalized();
    return s;
}

struct Descent {
    double drogue_height_m = -1.0, drogue_speed_mps = -1.0, drogue_s = -1.0;
    double main_height_m = -1.0, main_speed_mps = -1.0, main_s = -1.0;
    double splashdown_s = -1.0, sink_mps = -1.0, miss_m = -1.0, peak_g = 0.0;
    sim::LandingOutcome outcome = sim::LandingOutcome::Flying;
};

// Flies the controlled body (the capsule) of `session` until it is down or `max_s` passed, in `step_s` steps; records
// the events (deployment heights as the sequencer saw them). `final_step_s` is used in the last 100 m.
Descent fly_down(sim::FlightSession& session, double max_s = 900.0, double step_s = 0.05, double final_step_s = 0.05) {
    Descent d;
    const auto t0 = session.epoch();
    const size_t i = session.controlled_index();
    while (session.epoch() - t0 < max_s) {
        const bool low = session.telemetry(i).altitude_m < 100.0;
        session.advance_to(session.epoch() + (low ? final_step_s : step_s), 1000000);
        const auto& body = session.body(i);
        const auto tel = session.telemetry(i);
        d.peak_g = std::max(d.peak_g, norm(body.aero_acceleration_body_mps2()) / constants::standard_gravity_mps2);
        const auto* seq = session.parachute_sequencer(i);
        if (seq && d.drogue_s < 0.0 && seq->drogue_height_m()) {
            d.drogue_s = session.epoch() - t0;
            d.drogue_height_m = *seq->drogue_height_m();
            d.drogue_speed_mps = tel.speed_relative_mps;
        }
        if (seq && d.main_s < 0.0 && seq->main_height_m()) {
            d.main_s = session.epoch() - t0;
            d.main_height_m = *seq->main_height_m();
            d.main_speed_mps = tel.speed_relative_mps;
        }
        if (body.resting() || body.impacted()) {
            d.splashdown_s = session.epoch() - t0;
            break;
        }
    }
    const auto& body = session.body(i);
    if (body.touchdown_sink_mps()) d.sink_mps = *body.touchdown_sink_mps();
    d.outcome = session.landing_outcome(i);
    if (session.splashdown_zone()) d.miss_m = session.splashdown_miss_m(i);
    return d;
}

struct Scenario {
    sim::FlightSession session{spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR};
    config::SplashdownZoneConfig zone = config::load_splashdown_zone(STARSIM_CONFIG_DIR, "pacific_san_diego");
    time::Epoch t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 14, 6, 0, 0.0});
    Scenario(double height_m, double speed_mps, double gamma_rad, const config::VehicleConfig& cfg = capsule_config()) {
        session.set_splashdown_zone(zone);
        session.place_body(cfg, falling_state(zone, height_m, speed_mps, gamma_rad, t0), t0);
    }
};

}  // namespace

TEST_CASE("From the drogue gate the capsule splashes down under its mains", "[dragon][splashdown][session]") {
    Scenario s(5600.0, 166.0, 60.0 * deg);  // d3's gate: 166 m/s, steep
    const auto d = fly_down(s.session);
    std::printf("[validation] gate to water: drogues %.2f km at %.0f m/s, mains %.2f km at %.0f m/s after %.0f s, "
                "splashdown %.0f s after the mains at %.1f m/s, %.2f km from the zone centre, peak %.2f g\n",
                d.drogue_height_m / 1000.0, d.drogue_speed_mps, d.main_height_m / 1000.0, d.main_speed_mps,
                d.main_s - d.drogue_s, d.splashdown_s - d.main_s, d.sink_mps, d.miss_m / 1000.0, d.peak_g);
    CHECK(d.drogue_height_m == Catch::Approx(5500.0).epsilon(0.02));
    CHECK(d.main_height_m == Catch::Approx(1800.0).epsilon(0.02));
    CHECK(d.main_speed_mps == Catch::Approx(53.0).epsilon(0.15));          // [S] 119 mph
    CHECK(d.main_s - d.drogue_s < 90.0);                                    // [S] "less than a minute later";
    CHECK(d.main_s - d.drogue_s > 40.0);                                    //     Demo-2 2:44 -> 2:45 (minutes)
    CHECK(d.splashdown_s - d.main_s == Catch::Approx(180.0).epsilon(0.35));  // [S] Demo-2 2:45 -> 2:48
    CHECK(d.sink_mps == Catch::Approx(7.0).epsilon(0.30));                  // [S] 15-16 mph, spec tolerance 30 %
    CHECK(d.outcome == sim::LandingOutcome::SplashedDown);
    CHECK(d.peak_g < 3.6);
    CHECK_FALSE(s.session.body(0).crew_load_warning());
    CHECK_FALSE(s.session.body(0).vehicle().parachute_torn());
    CHECK(d.miss_m < 2000.0);  // the drift from 5.6 km over the zone stays inside the 2 km zone
}

TEST_CASE("The Dracos stay off under the parachutes", "[dragon][splashdown][session]") {
    Scenario s(5600.0, 166.0, 60.0 * deg);
    double rcs_after_drogues = -1.0;
    const auto t0 = s.session.epoch();
    while (s.session.epoch() - t0 < 120.0) {
        s.session.advance_to(s.session.epoch() + 0.05, 1000000);
        const auto* seq = s.session.parachute_sequencer(0);
        if (seq && seq->phase() != gnc::ParachutePhase::Armed && rcs_after_drogues < 0.0) {
            rcs_after_drogues = s.session.body(0).vehicle().rcs_propellant_kg();
        }
    }
    REQUIRE(rcs_after_drogues > 0.0);
    CHECK(s.session.body(0).vehicle().rcs_propellant_kg() == Catch::Approx(rcs_after_drogues).margin(1e-9));
}

TEST_CASE("Drogues open at their height under time warp", "[dragon][splashdown][session]") {
    Scenario s(12000.0, 200.0, 80.0 * deg);
    const auto d = fly_down(s.session, 900.0, 30.0);  // 30 s steps: coast segments wherever the session allows them
    const auto* seq = s.session.parachute_sequencer(0);
    REQUIRE(seq != nullptr);
    REQUIRE(seq->drogue_height_m());
    CHECK(*seq->drogue_height_m() == Catch::Approx(5500.0).margin(50.0));  // one tick of fall at most, not a coast
    REQUIRE(seq->main_height_m());
    CHECK(*seq->main_height_m() == Catch::Approx(1800.0).margin(10.0));
    CHECK(d.outcome == sim::LandingOutcome::SplashedDown);
}

TEST_CASE("A capsule placed below the main height opens its mains at once", "[dragon][splashdown][session]") {
    Scenario s(1200.0, 45.0, 85.0 * deg);
    s.session.advance_to(s.session.epoch() + 0.1);
    const auto& chutes = s.session.body(0).vehicle().stages().front().parachutes;
    CHECK(chutes[0].state == vehicle::ParachuteState::Stowed);
    CHECK(chutes[1].state == vehicle::ParachuteState::Open);
    const auto d = fly_down(s.session);
    CHECK(d.outcome == sim::LandingOutcome::SplashedDown);
}

TEST_CASE("Drogues held back by dynamic pressure: the mains tear and the capsule hits the water",
          "[dragon][splashdown][session]") {
    auto cfg = capsule_config();
    cfg.stages.front().parachutes[0].max_deploy_dynamic_pressure_pa = 100.0;  // never reached: the drogues never go
    Scenario s(5600.0, 166.0, 60.0 * deg, cfg);
    const auto d = fly_down(s.session);
    const auto& mains = s.session.body(0).vehicle().stages().front().parachutes[1];
    std::printf("no drogues: mains at %.0f m / %.0f m/s, peak %.0f kN per canopy, peak %.2f g, %s at %.1f m/s\n",
                d.main_height_m, d.main_speed_mps, mains.peak_load_n / 1000.0, d.peak_g, sim::to_string(d.outcome), d.sink_mps);
    CHECK(d.drogue_s < 0.0);
    CHECK(s.session.body(0).vehicle().parachute_torn());
    CHECK(d.sink_mps > 9.0);
    CHECK(d.outcome == sim::LandingOutcome::Impact);
}

TEST_CASE("A capsule on its trunk has no parachute sequencer", "[dragon][splashdown][session]") {
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    const auto zone = config::load_splashdown_zone(STARSIM_CONFIG_DIR, "pacific_san_diego");
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 14, 6, 0, 0.0});
    session.set_splashdown_zone(zone);
    session.place_body(config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon"),
                       falling_state(zone, 5000.0, 150.0, 60.0 * deg, t0), t0);
    session.advance_to(t0 + 1.0);
    CHECK(session.parachute_sequencer(0) == nullptr);
    CHECK_FALSE(session.body(0).vehicle().parachutes_out());
}

TEST_CASE("Crew Dragon return mission loads", "[dragon][mission][config]") {
    const auto m = config::load_capsule_return_mission(STARSIM_CONFIG_DIR, "crew_dragon_return");
    CHECK(m.vehicle_id == "crew_dragon");
    CHECK(m.zone_id == "pacific_san_diego");
    CHECK(m.periapsis_height_m == 420000.0);
    CHECK(m.inclination_rad == Catch::Approx(51.6 * deg));
    CHECK(m.entry_flight_path_rad == Catch::Approx(-2.0 * deg));
    CHECK(m.nominal_bank_rad == Catch::Approx(30.0 * deg));
    CHECK(m.max_crossrange_m == 40000.0);
    CHECK(m.rcs_propellant_kg == 2000.0);
    const auto p = sim::capsule_return_parameters(m);
    CHECK(p.entry_flight_path_rad == m.entry_flight_path_rad);
    CHECK(p.gate_height_m == Catch::Approx(5500.0));
}

TEST_CASE("Crew Dragon return: the first feasible orbit plane", "[.][dragon][mission]") {
    auto m = config::load_capsule_return_mission(STARSIM_CONFIG_DIR, "crew_dragon_return");
    test::spice_session();  // leap seconds for the UTC epoch
    const auto t0 = spice::SpiceTimeConverter{}.from_utc(m.epoch_utc);
    for (double r = 0.0; r < 360.0; r += 10.0) {
        m.raan_rad = r * deg;
        sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
        sim::place_capsule_return_mission(session, m, STARSIM_CONFIG_DIR, t0);
        if (session.plan_capsule_return(sim::capsule_return_parameters(m))) {
            std::printf("first feasible RAAN %.0f deg (burn after %.1f h)\n", r,
                        (session.ship_return_plan()->node.epoch - t0) / 3600.0);
            return;
        }
    }
    FAIL("no plane");
}

namespace {

struct MissionRun {
    bool planned = false;
    double wait_h = 0.0;
    double ignition_s = -1.0, cutoff_s = -1.0, trunk_s = -1.0, interface_s = -1.0;
    Descent descent;  // times relative to the gate (fly_down)
    double splashdown_after_ignition_s = -1.0;
    double rcs_left_kg = 0.0;
    sim::CapsuleReturnStep final_step = sim::CapsuleReturnStep::None;  // 1 s after the descent ended
    sim::AutopilotState final_autopilot = sim::AutopilotState::Off;
};

// Dispersion of a return: orbit phase offset, MMH/NTO load, deorbit burn execution (applied at cutoff).
struct MissionDispersion {
    double phase_offset_deg = 0.0;
    double rcs_scale = 1.0;
    double burn_scale = 0.0;            // relative magnitude error
    double pointing_rad = 0.0;          // pointing error
    double pointing_azimuth_rad = 0.0;  // about the burn direction, from the orbit normal
    bool without_mains = false;         // the capsule flies without its main parachutes
    double final_step_s = 0.05;         // advance_to steps in the last 100 m (one tick: a call ends on the contact tick)
};

// Places the mission (dispersed), plans and flies it to the water.
MissionRun fly_mission(const MissionDispersion& dispersion = {}) {
    auto m = config::load_capsule_return_mission(STARSIM_CONFIG_DIR, "crew_dragon_return");
    m.argument_of_latitude_rad += dispersion.phase_offset_deg * deg;
    m.rcs_propellant_kg *= dispersion.rcs_scale;
    test::spice_session();  // leap seconds for the UTC epoch
    const auto t0 = spice::SpiceTimeConverter{}.from_utc(m.epoch_utc);
    sim::FlightSession session(spice_ephemeris(), spice_earth(), STARSIM_CONFIG_DIR);
    auto vehicle = config::load_vehicle(STARSIM_CONFIG_DIR, m.vehicle_id);
    if (dispersion.without_mains) {
        std::erase_if(vehicle.stages.back().parachutes, [](const auto& p) { return p.role == "main"; });
    }
    sim::place_capsule_return_mission(session, m, std::move(vehicle), STARSIM_CONFIG_DIR, t0);
    MissionRun run;
    run.planned = session.plan_capsule_return(sim::capsule_return_parameters(m));
    if (!run.planned) return run;
    run.wait_h = (session.ship_return_plan()->node.epoch - t0) / 3600.0;
    // to the gate as in d3: big steps to the burn, 1 s after
    while (static_cast<int>(session.capsule_return_step()) < static_cast<int>(sim::CapsuleReturnStep::Gate) &&
           session.epoch() - t0 < 3.0 * 86400.0 && !session.body(session.controlled_index()).impacted()) {
        double step = 1.0;
        if (session.autopilot().state == sim::AutopilotState::Burn) {
            const double to_ignition = session.autopilot().burn.time_to_ignition_s;
            if (to_ignition > 900.0) step = to_ignition - 600.0;
            if (run.ignition_s < 0.0 && session.autopilot().burn.ignite) run.ignition_s = session.epoch() - t0;
        } else if (run.cutoff_s < 0.0 && run.ignition_s >= 0.0) {
            run.cutoff_s = session.epoch() - t0;
            const auto* plan = session.ship_return_plan();
            const auto& s = session.body(session.controlled_index()).state();
            const Vec3 u = math::normalized(plan->node.delta_v_gcrf_mps);
            const Vec3 a = math::normalized(cross(s.position_gcrf_m, s.velocity_gcrf_mps));
            const Vec3 b = cross(u, a);
            const Vec3 w = a * std::cos(dispersion.pointing_azimuth_rad) + b * std::sin(dispersion.pointing_azimuth_rad);
            session.apply_velocity_error(session.controlled_index(),
                                         (u * dispersion.burn_scale + w * std::tan(dispersion.pointing_rad)) * plan->delta_v_mps);
        }
        session.advance_to(session.epoch() + step, 1000000);
        if (run.trunk_s < 0.0 && session.body_count() == 2) run.trunk_s = session.epoch() - t0;
        if (run.interface_s < 0.0 && run.cutoff_s >= 0.0 &&
            session.telemetry(session.controlled_index()).altitude_m < m.entry_interface_height_m) {
            run.interface_s = session.epoch() - t0;
        }
    }
    const double gate_s = session.epoch() - t0;
    run.descent = fly_down(session, 900.0, 0.05, dispersion.final_step_s);
    run.splashdown_after_ignition_s = gate_s + run.descent.splashdown_s - run.ignition_s;
    run.rcs_left_kg = session.body(session.controlled_index()).vehicle().rcs_propellant_kg();
    session.advance_to(session.epoch() + 60.0, 1000000);  // one time-warp frame: the step machine still sees the outcome
    run.final_step = session.capsule_return_step();
    run.final_autopilot = session.autopilot().state;
    return run;
}

}  // namespace

TEST_CASE("Crew Dragon return mission splashes down in the zone on the published timeline", "[dragon][mission][splashdown]") {
    const auto r = fly_mission();
    REQUIRE(r.planned);
    const auto& d = r.descent;
    std::printf("[validation] Crew Dragon return: waited %.1f h; burn %.0f s, trunk %.0f s after cutoff, interface %.0f s "
                "after cutoff; drogues %.2f km / %.0f m/s, mains %.2f km / %.0f m/s %.0f s later, splashdown %.0f s after "
                "the mains at %.1f m/s, %.2f km from the zone centre; ignition -> splashdown %.1f min (Crew-10: 54); "
                "peak under canopies %.2f g; %.0f kg MMH/NTO left\n",
                r.wait_h, r.cutoff_s - r.ignition_s, r.trunk_s - r.cutoff_s, r.interface_s - r.cutoff_s,
                d.drogue_height_m / 1000.0, d.drogue_speed_mps, d.main_height_m / 1000.0, d.main_speed_mps,
                d.main_s - d.drogue_s, d.splashdown_s - d.main_s, d.sink_mps, d.miss_m / 1000.0,
                r.splashdown_after_ignition_s / 60.0, d.peak_g, r.rcs_left_kg);
    CHECK(d.outcome == sim::LandingOutcome::SplashedDown);
    CHECK(d.miss_m < 2000.0);                                          // user 2026-10-14: zone radius 2 km
    CHECK(d.drogue_height_m == Catch::Approx(5500.0).epsilon(0.15));    // [S] ~18,000 ft
    CHECK(d.drogue_speed_mps == Catch::Approx(156.0).epsilon(0.15));    // [S] ~350 mph
    CHECK(d.main_height_m == Catch::Approx(1830.0).epsilon(0.15));      // [S] ~6,000 ft
    CHECK(d.main_speed_mps == Catch::Approx(53.0).epsilon(0.15));       // [S] ~119 mph
    CHECK(d.sink_mps == Catch::Approx(7.0).epsilon(0.30));              // [S] 15-16 mph
    CHECK(r.splashdown_after_ignition_s / 60.0 == Catch::Approx(54.0).epsilon(0.15));  // [S] Crew-10 10:39 -> 11:33 EDT
    CHECK(d.peak_g < 3.6);
    CHECK(r.final_step == sim::CapsuleReturnStep::SplashedDown);
}

namespace {

// Dispersed returns on the mission's pass: an ISS-like orbit (51.6 deg) passes a given point within the capsule's 40 km
// crossrange only every few days (from the 8 orbit phases 0-315 deg of the mission orbit only phase 0 has a pass within
// 48 h, 2 within 96 h: a 45 deg phase shifts the ground track ~270 km at 32.6 N), so the phase is dispersed about the
// mission's (+-3 deg, ~18 km), with the MMH/NTO load (+-10 %) and the deorbit burn execution (+-1 %, 0.5 deg).
MissionDispersion dispersion_case(int index) {
    std::mt19937 rng(20261014);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    MissionDispersion d;
    for (int i = 0; i <= index; ++i) {
        d.phase_offset_deg = 3.0 * u(rng);
        d.rcs_scale = 1.0 + 0.1 * u(rng);
        d.burn_scale = 0.01 * u(rng);
        d.pointing_rad = 0.5 * deg * std::abs(u(rng));
        d.pointing_azimuth_rad = constants::pi * u(rng);
    }
    return d;
}

void fly_dispersed_campaign(int first, int count) {
    for (int i = first; i < first + count; ++i) {
        const auto c = dispersion_case(i);
        const auto r = fly_mission(c);
        std::printf("[validation] dragon return case %d (phase %+.2f deg, MMH/NTO %+.1f %%, burn %+.2f %% / %.2f deg): "
                    "planned %d, waited %.1f h, %s, %.2f km from the centre, %.1f m/s, peak %.2f g, %.1f min ignition -> "
                    "splashdown\n",
                    i, c.phase_offset_deg, (c.rcs_scale - 1.0) * 100.0, c.burn_scale * 100.0, c.pointing_rad / deg,
                    r.planned, r.wait_h, sim::to_string(r.descent.outcome), r.descent.miss_m / 1000.0,
                    r.descent.sink_mps, r.descent.peak_g, r.splashdown_after_ignition_s / 60.0);
        INFO("case " << i);
        CHECK(r.planned);
        CHECK(r.descent.outcome == sim::LandingOutcome::SplashedDown);
        CHECK(r.descent.miss_m < 2000.0);
    }
}

}  // namespace

TEST_CASE("Crew Dragon return campaign: cases 0-3", "[dragon][mission][validation]") { fly_dispersed_campaign(0, 4); }
TEST_CASE("Crew Dragon return campaign: cases 4-7", "[dragon][mission][validation]") { fly_dispersed_campaign(4, 4); }

// STARSIM_DRAGON_CASE=<0-7> (with STARSIM_DEBUG_ENTRY=1 / STARSIM_DEBUG_DEORBIT=1 for the guidance).
TEST_CASE("Crew Dragon return single case", "[.][dragon]") {
    const char* index = std::getenv("STARSIM_DRAGON_CASE");
    const auto r = fly_mission(dispersion_case(index ? std::atoi(index) : 0));
    std::printf("single case: %s, %.2f km from the centre, gate at %.2f km\n", sim::to_string(r.descent.outcome),
                r.descent.miss_m / 1000.0, r.descent.drogue_height_m / 1000.0);
}

// Review d4 (Important): the sequencer was created in the first tick, so a placed capsule's first coast segment used the
// 5 km tick height and carried it past the drogue height.
TEST_CASE("A capsule placed above the drogue height does not coast past it", "[dragon][splashdown][session]") {
    Scenario s(8000.0, 200.0, 80.0 * deg);
    fly_down(s.session, 900.0, 30.0);
    const auto* seq = s.session.parachute_sequencer(0);
    REQUIRE(seq != nullptr);
    REQUIRE(seq->drogue_height_m());
    CHECK(*seq->drogue_height_m() == Catch::Approx(5500.0).margin(50.0));
}

TEST_CASE("A capsule return without mains ends lost, not under its drogues", "[dragon][mission][splashdown]") {
    MissionDispersion d;
    d.without_mains = true;
    d.final_step_s = 1.5 * sim::FlightSession::tick_s;  // one tick per call (review d5 I2: the next call coasts)
    const auto r = fly_mission(d);
    REQUIRE(r.planned);
    CHECK(r.descent.outcome == sim::LandingOutcome::Impact);  // ~50 m/s under the drogues alone
    CHECK(r.final_step == sim::CapsuleReturnStep::Lost);
    CHECK(std::string(sim::to_string(r.final_step)) == "verloren");
    CHECK(r.final_autopilot == sim::AutopilotState::Off);
}
