// Ship return from orbit (b4): config/missions/ship_return_starbase.toml in one session - the onboard computer plans
// the deorbit burn, the belly Raptor flies it, the ship coasts, enters on its entry guidance and is caught at the tower.

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "return_scenario.hpp"
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
    double full = 0.0;
    for (const auto& tank : cfg.stages[0].tanks) full += tank.mass_kg;
    for (auto& tank : cfg.stages[0].tanks) tank.mass_kg *= m.propellant_kg / full;

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
    p.entry_interface_height_m = m.entry_interface_height_m;
    p.entry_flight_path_rad = m.entry_flight_path_rad;
    p.nominal_bank_rad = m.nominal_bank_rad;
    p.max_crossrange_m = m.max_crossrange_m;
    p.search_window_s = m.search_window_s;
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

// Plans the return, flies to the gate (coarse steps through the coast, 1 s in the entry, 0.05 s near the gate) and
// on to the catch.
ReturnRun fly_return(sim::FlightSession& session, const gnc::DeorbitParameters& parameters) {
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
    while (session.epoch() < interface - 400.0) session.advance_to(std::min(session.epoch() + 600.0, interface - 400.0), 400000);
    const bool verbose = std::getenv("STARSIM_VERBOSE") != nullptr;
    while (!session.body(0).impacted() && session.epoch() < interface + 3600.0) {
        const auto* entry = session.entry_guidance(0);
        if (entry && entry->phase() == gnc::EntryPhase::Gate) break;
        const double height = session.telemetry(0).altitude_m - session.launch_site().height_m;
        session.advance_to(session.epoch() + (height < 3000.0 ? 0.05 : 1.0), 400000);
        if (verbose && entry && static_cast<long>(session.epoch() - interface) % 20 == 0 && height >= 3000.0) {
            const auto* s = session.entry_status(0);
            const auto tel = session.telemetry(0);
            std::printf("t %+.0f %s h %.1f km v %.0f Mach %.2f alpha %.1f bank %.1f downrange %.1f km cross %.1f km\n",
                        session.epoch() - interface, gnc::to_string(s->phase), tel.altitude_m / 1000.0,
                        tel.speed_relative_mps, tel.mach, s->alpha_rad / deg, s->bank_rad / deg,
                        s->downrange_error_m / 1000.0, s->crossrange_m / 1000.0);
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
    CHECK(std::abs(m.entry_flight_path_rad + 1.0 * deg) < 1e-12);
    CHECK(m.search_window_s == 48.0 * 3600.0);
}

TEST_CASE("Ship returns from orbit into the tower's arms", "[ship][catch][session][mission]") {
    const auto m = config::load_ship_return_mission(STARSIM_CONFIG_DIR, "ship_return_starbase");
    sim::FlightSession session(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    place_mission(session, m);
    const auto run = fly_return(session, deorbit_parameters(m));
    REQUIRE(run.planned);
    CHECK(run.gate);
    CHECK(run.gate_miss_m < 300.0);  // the catch is the criterion: gate cross-track errors up to ~200 m are flown out (b3)
    CHECK_FALSE(run.landing.tower_struck);
    CHECK(run.landing.caught_within_tolerance());
}
