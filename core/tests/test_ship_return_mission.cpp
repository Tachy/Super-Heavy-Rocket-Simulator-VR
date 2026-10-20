// Ship return from orbit (b4): config/missions/ship_return_starbase.toml in one session - the onboard computer plans
// the deorbit burn, the belly Raptor flies it, the ship coasts, enters on its entry guidance and is caught at the tower.

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

#include "return_scenario.hpp"
#include "starsim/config/propellant_loading.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double equatorial_radius_m = 6378137.0;

const spice::SpiceEphemeris& spice_ephemeris() {
    test::spice_session();
    static const spice::SpiceEphemeris e;
    return e;
}

const frames::BodyOrientation& earth() {
    test::spice_session();
    static const auto e = spice::SpiceOrientation::earth();
    return e;
}

// Places the mission's ship on its start orbit (nose along the velocity, belly to the Earth); returns the epoch.
time::Epoch place_mission(sim::FlightSession& session, const config::ShipReturnMission& m) {
    test::spice_session();
    const auto t0 = spice::SpiceTimeConverter{}.from_utc(m.epoch_utc);
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, m.site_id)));
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, m.vehicle_id);
    while (!cfg.stages.empty() && cfg.stages.front().id != m.stage_id) cfg.stages.erase(cfg.stages.begin());
    REQUIRE(!cfg.stages.empty());
    config::fill_tanks(cfg.stages[0], m.propellant_kg);

    const double rp = equatorial_radius_m + m.periapsis_height_m, ra = equatorial_radius_m + m.apoapsis_height_m;
    const orbit::KeplerianElements elements{0.5 * (rp + ra), (ra - rp) / (ra + rp), m.inclination_rad, m.raan_rad, 0.0,
                                            m.argument_of_latitude_rad};
    const auto s = orbit::state_from_elements(elements, session.gm_earth_m3ps2());
    const Vec3 x = math::normalized(s.velocity_mps);
    const Vec3 z = math::normalized(s.position_m);  // belly (body -z) to the Earth
    const Vec3 y = cross(z, x);
    math::Mat3 r;
    const Vec3 columns[3] = {x, y, z};
    for (int c = 0; c < 3; ++c) {
        r(0, c) = columns[c].x;
        r(1, c) = columns[c].y;
        r(2, c) = columns[c].z;
    }
    session.place_body(cfg, {s.position_m, s.velocity_mps, math::Quat::from_matrix(r), {}}, t0);
    return t0;
}

gnc::DeorbitParameters deorbit_parameters(const config::ShipReturnMission& m) {
    gnc::DeorbitParameters p;
    p.entry_interface_height_m = m.deorbit.entry_interface_height_m;
    p.entry_flight_path_rad = m.deorbit.entry_flight_path_rad;
    p.nominal_bank_rad = m.deorbit.nominal_bank_rad;
    p.max_crossrange_m = m.deorbit.max_crossrange_m;
    p.search_window_s = m.deorbit.search_window_s;
    return p;
}

struct ReturnRun {
    bool planned = false;
    bool gate = false;
    double gate_miss_m = 0.0;
    double entry_s = 0.0;  // entry interface to gate
    test::CatchRun landing;
    double propellant_kg = 0.0;
};

// Execution error of the deorbit burn, applied when it ends: magnitude (fraction of the burn) and pointing.
struct BurnError {
    double scale = 0.0;
    double pointing_rad = 0.0;
    double pointing_azimuth_rad = 0.0;  // about the burn direction, from the orbit normal
};

// Plans the return, flies to the gate (coarse steps through the coast, 1 s in the entry, 0.05 s near the gate) and
// on to the catch.
ReturnRun fly_return(sim::FlightSession& session, const gnc::DeorbitParameters& parameters, const BurnError& error = {}) {
    ReturnRun run;
    run.planned = session.plan_ship_return(parameters);
    const auto* plan = session.ship_return_plan();
    REQUIRE(plan != nullptr);
    std::printf("return plan: %s%s; burn in %.1f h, %.1f m/s (%.1f t), entry %.3f deg %.0f m/s, crossrange %.1f km\n",
                plan->feasible ? "feasible" : "refused: ", plan->reason.c_str(), (plan->node.epoch - session.epoch()) / 3600.0,
                plan->delta_v_mps, plan->propellant_kg / 1000.0, plan->entry_flight_path_rad / deg, plan->entry_speed_mps,
                plan->crossrange_m / 1000.0);
    if (!run.planned) return run;
    const time::Epoch interface = plan->entry_interface_epoch;
    while (session.autopilot().state == sim::AutopilotState::Burn) session.advance_to(session.epoch() + 60.0, 400000);
    if (error.scale != 0.0 || error.pointing_rad != 0.0) {
        const Vec3 u = math::normalized(plan->node.delta_v_gcrf_mps);
        const Vec3 a = math::normalized(cross(session.body(0).state().position_gcrf_m, session.body(0).state().velocity_gcrf_mps));
        const Vec3 b = cross(u, a);
        const Vec3 w = a * std::cos(error.pointing_azimuth_rad) + b * std::sin(error.pointing_azimuth_rad);
        session.apply_velocity_error(0, (u * error.scale + w * std::tan(error.pointing_rad)) * plan->delta_v_mps);
    }
    while (session.epoch() < interface - 400.0) session.advance_to(std::min(session.epoch() + 600.0, interface - 400.0), 400000);
    const bool verbose = std::getenv("STARSIM_VERBOSE") != nullptr;
    while (!session.body(0).impacted() && session.epoch() < interface + 3600.0) {
        const auto* entry = session.entry_guidance(0);
        if (entry && entry->phase() == gnc::EntryPhase::Gate) break;
        const double height = session.telemetry(0).altitude_m - session.launch_site().height_m;
        session.advance_to(session.epoch() + (height < 3000.0 ? 0.05 : 1.0), 400000);
        const long elapsed = static_cast<long>(session.epoch() - interface);
        if (verbose && entry && height >= 3000.0 && (elapsed % 20 == 0 || (height < 14000.0 && elapsed % 2 == 0))) {
            const auto* s = session.entry_status(0);
            const auto tel = session.telemetry(0);
            const auto& w = session.body(0).state().angular_velocity_body_radps;
            std::printf("t %+.0f %s h %.1f km v %.0f Mach %.2f alpha %.1f (aoa %.1f) bank %.1f downrange %.1f km cross %.1f km "
                        "gamma %.1f w %.3f %.3f %.3f\n",
                        session.epoch() - interface, gnc::to_string(s->phase), tel.altitude_m / 1000.0,
                        tel.speed_relative_mps, tel.mach, s->alpha_rad / deg, tel.angle_of_attack_rad / deg, s->bank_rad / deg,
                        s->downrange_error_m / 1000.0, s->crossrange_m / 1000.0, tel.flight_path_angle_rad / deg, w.x, w.y, w.z);
        }
    }
    const auto* entry = session.entry_guidance(0);
    run.gate = entry && entry->phase() == gnc::EntryPhase::Gate;
    run.entry_s = session.epoch() - interface;
    if (run.gate) {
        const auto tr = earth().fixed_to_gcrf(session.epoch());
        const Vec3 r = transpose(tr.rotation) * session.body(0).state().position_gcrf_m;
        const Vec3 up = math::normalized(r);
        const Vec3 d = r - entry->target().position_itrf_m;
        run.gate_miss_m = norm(d - up * dot(d, up));
        {
            const Vec3 nose_dir = entry->target().nose_direction_itrf;  // to the tower
            const Vec3 side = cross(up, nose_dir);
            const auto& st = session.body(0).state();
            const Vec3 v = transpose(tr.rotation) * (st.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(tr), st.position_gcrf_m));
            const Vec3 nose = transpose(tr.rotation) * st.attitude.rotate(Vec3{1.0, 0.0, 0.0});
            const auto tel = session.telemetry(0);
            std::printf("gate: along %.0f m across %.0f m (tower line), velocity along %.1f across %.1f down %.1f m/s, nose "
                        "along %.2f across %.2f up %.2f, alpha %.1f deg\n",
                        dot(d, nose_dir), dot(d, side), dot(v, nose_dir), dot(v, side), -dot(v, up), dot(nose, nose_dir),
                        dot(nose, side), dot(nose, up), tel.angle_of_attack_rad / deg);
        }
        run.landing = test::fly_landing(session, earth(), session.epoch(), 120.0);
    }
    run.propellant_kg = session.body(0).vehicle().propellant_kg();
    std::printf("return: gate %d after %.0f s from the entry interface, miss %.0f m; %s, offset %.2f m, lateral %.2f m/s, "
                "sink %.2f m/s, %.1f t left\n",
                run.gate, run.entry_s, run.gate_miss_m, sim::to_string(run.landing.tower_phase), run.landing.offset_m,
                run.landing.lateral_mps, run.landing.sink_mps, run.propellant_kg / 1000.0);
    return run;
}

}  // namespace

TEST_CASE("Ship return mission loads", "[ship][mission]") {
    const auto m = config::load_ship_return_mission(STARSIM_CONFIG_DIR, "ship_return_starbase");
    CHECK(m.site_id == "starbase_olp1");
    CHECK(m.stage_id == "ship");
    CHECK(m.periapsis_height_m == 200000.0);
    CHECK(std::abs(m.deorbit.entry_flight_path_rad + 1.0 * deg) < 1e-12);
    CHECK(m.deorbit.search_window_s == 48.0 * 3600.0);
}

// Refuelling r2 (found by boil-off): a burn whose engines run dry cannot finish - it ends with a reason instead of
// waiting for ever.
TEST_CASE("A burn on dry main tanks is aborted with a reason", "[ship][session][deorbit][boil_off]") {
    auto m = config::load_ship_return_mission(STARSIM_CONFIG_DIR, "ship_return_starbase");
    m.propellant_kg = 35000.0;  // the header tanks only: the main tanks the deorbit Raptor feeds from are empty
    sim::FlightSession session(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    place_mission(session, m);
    const auto& s = session.body(0).state();
    const gnc::BurnNode node{session.epoch() + 700.0, -math::normalized(s.velocity_gcrf_mps) * 60.0};
    REQUIRE(session.execute_burn(node, {"sea_level_3"}));
    for (int k = 0; k < 120 && session.autopilot().state == sim::AutopilotState::Burn; ++k) {
        session.advance_to(session.epoch() + 60.0, 400000);
    }
    CHECK(session.autopilot().state != sim::AutopilotState::Burn);
    INFO(session.burn_abort_reason());
    CHECK(session.burn_abort_reason().find("Treibstoff") != std::string::npos);
}

// Refuelling r2: the onboard computer counts the bare hull's boil-off until the burn - 7 t above the landing
// propellant leave too little methane for the burn after the ~22 h to the Starbase pass.
TEST_CASE("Ship return plan counts the boil-off until the pass", "[ship][session][deorbit][boil_off]") {
    auto m = config::load_ship_return_mission(STARSIM_CONFIG_DIR, "ship_return_starbase");
    m.propellant_kg = 42000.0;
    sim::FlightSession session(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    place_mission(session, m);
    CHECK_FALSE(session.plan_ship_return(deorbit_parameters(m)));
    REQUIRE(session.ship_return_plan() != nullptr);
    INFO(session.ship_return_plan()->reason);
    CHECK(session.ship_return_plan()->reason.find("verdampft") != std::string::npos);

    sim::FlightSession without(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    without.set_boil_off(false);
    place_mission(without, m);
    CHECK(without.plan_ship_return(deorbit_parameters(m)));
}

TEST_CASE("Ship returns from orbit into the tower's arms", "[ship][catch][session][mission]") {
    const auto m = config::load_ship_return_mission(STARSIM_CONFIG_DIR, "ship_return_starbase");
    sim::FlightSession session(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    place_mission(session, m);
    const auto run = fly_return(session, deorbit_parameters(m));
    REQUIRE(run.planned);
    CHECK(run.gate);
    CHECK(run.gate_miss_m < 100.0);
    CHECK_FALSE(run.landing.tower_struck);
    CHECK(run.landing.caught_within_tolerance());
}

namespace {

struct Case {
    double phase_deg = 0.0;
    double periapsis_km = 200.0;
    double apoapsis_km = 200.0;
    double propellant_t = 45.0;  // before the boil-off allowance (fly_campaign adds the predicted boil-off)
    BurnError error;
};

void fly_campaign(const std::vector<Case>& cases, const char* name) {
    const auto base = config::load_ship_return_mission(STARSIM_CONFIG_DIR, "ship_return_starbase");
    int index = 0;
    for (const auto& c : cases) {
        auto m = base;
        m.argument_of_latitude_rad = c.phase_deg * deg;
        m.periapsis_height_m = c.periapsis_km * 1000.0;
        m.apoapsis_height_m = c.apoapsis_km * 1000.0;
        m.propellant_kg = c.propellant_t * 1000.0;
        // Mission design (refuelling r2): the ship carries what the wait for its pass boils off on top of the pre-r2
        // load - the smallest load in 2 t steps that the onboard computer's plan (flown ahead, per propellant)
        // accepts - so the landing residue stays near the pre-r2 one.
        constexpr double sizing_step_kg = 2000.0;
        for (int sizing_round = 0; sizing_round < 10; ++sizing_round) {
            sim::FlightSession sizing(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
            place_mission(sizing, m);
            if (sizing.plan_ship_return(deorbit_parameters(m))) break;
            const auto* p = sizing.ship_return_plan();
            if (p == nullptr || !p->short_of_reserve) break;  // refused for another reason: the checks below say so
            m.propellant_kg += sizing_step_kg;
        }
        sim::FlightSession session(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
        const auto t0 = place_mission(session, m);
        std::printf("%s case %d: phase %.0f deg, orbit %.1f x %.1f km, %.1f t (%.1f t + boil-off), burn error %.2f %% / "
                    "%.2f deg\n",
                    name, index, c.phase_deg, c.periapsis_km, c.apoapsis_km, m.propellant_kg / 1000.0, c.propellant_t,
                    c.error.scale * 100.0, c.error.pointing_rad / deg);
        const auto run = fly_return(session, deorbit_parameters(m), c.error);
        const auto* plan = session.ship_return_plan();
        std::printf("%s case %d: waited %.1f h\n", name, index, plan ? (plan->node.epoch - t0) / 3600.0 : -1.0);
        INFO(name << " case " << index);
        CHECK(run.planned);
        CHECK(run.gate);
        CHECK(run.gate_miss_m < 100.0);
        CHECK_FALSE(run.landing.tower_struck);
        CHECK(run.landing.caught_within_tolerance());
        ++index;
    }
}

std::vector<Case> phase_cases(int first, int count) {
    std::vector<Case> cases;
    for (int i = first; i < first + count; ++i) {
        Case c;
        c.phase_deg = 45.0 * i;
        cases.push_back(c);
    }
    return cases;
}

std::vector<Case> dispersed_cases(int first, int count) {
    std::mt19937 rng(20261013);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    std::vector<Case> cases;
    for (int i = 0; i < first + count; ++i) {
        Case c;
        c.phase_deg = 180.0 + 180.0 * u(rng);
        c.periapsis_km = 200.0 + 10.0 * u(rng);
        c.apoapsis_km = c.periapsis_km + 10.0 + 10.0 * u(rng);
        c.propellant_t = 45.0 + 3.0 * u(rng);
        c.error.scale = 0.01 * u(rng);
        c.error.pointing_rad = 0.5 * deg * std::abs(u(rng));
        c.error.pointing_azimuth_rad = constants::pi * u(rng);
        if (i >= first) cases.push_back(c);
    }
    return cases;
}

}  // namespace

// Campaign: the orbit phase sets which pass is taken (wait, crossrange, arrival heading); dispersed orbits, propellant
// and deorbit-burn execution errors (+-1 % magnitude, 0.5 deg pointing) are flown out by the entry guidance.
TEST_CASE("Ship return campaign: orbit phase 0-135 deg", "[ship][mission][validation][slow]") { fly_campaign(phase_cases(0, 4), "phase"); }
TEST_CASE("Ship return campaign: orbit phase 180-315 deg", "[ship][mission][validation][slow]") { fly_campaign(phase_cases(4, 4), "phase"); }
TEST_CASE("Ship return campaign: dispersions 1-3", "[ship][mission][validation][slow]") { fly_campaign(dispersed_cases(0, 3), "dispersed"); }
TEST_CASE("Ship return campaign: dispersions 4-6", "[ship][mission][validation][slow]") { fly_campaign(dispersed_cases(3, 3), "dispersed"); }

// Diagnostics: one return from the orbit phase in STARSIM_RETURN_PHASE (deg).
TEST_CASE("Ship return single case", "[.][ship][mission]") {
    Case c;
    if (const char* p = std::getenv("STARSIM_RETURN_PHASE")) c.phase_deg = std::atof(p);
    if (const char* d = std::getenv("STARSIM_RETURN_DISPERSED")) c = dispersed_cases(std::atoi(d), 1).front();
    fly_campaign({c}, "single");
}
