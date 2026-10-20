// Deorbit planner (b4): from a low Earth orbit the onboard computer finds the pass over Starbase and the retrograde
// burn that reaches the entry interface at b2's flight-path angle with the entry predictor ending at the gate.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <memory>

#include "starsim/config/propellant_loading.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/deorbit_planner.hpp"
#include "starsim/gnc/entry_aero_table.hpp"
#include "starsim/gnc/entry_guidance.hpp"
#include "starsim/gnc/ship_landing.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/flight_session.hpp"
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

const frames::BodyOrientation& earth() {
    test::spice_session();
    static const auto e = spice::SpiceOrientation::earth();
    return e;
}

struct Setup {
    std::unique_ptr<sim::FlightSession> session;
    std::unique_ptr<vehicle::FlightBody> ship;
    gnc::EntryTarget target;
    gnc::EntryPredictorModel model;
    math::StateVector orbit;
    time::Epoch t0;
    double exhaust_velocity_mps = 0.0;
    double propellant_kg = 0.0;
    double reserve_kg = 0.0;
};

// The ship (dry + `propellant_kg`) on a 200 km circular orbit at 26.5 deg, at argument of latitude `phase_rad`.
Setup setup(double phase_rad, double propellant_kg = 45000.0) {
    test::spice_session();
    Setup s;
    s.t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 13, 12, 0, 0.0});
    s.session = std::make_unique<sim::FlightSession>(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    s.session->set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1")));
    const double gm = s.session->gm_earth_m3ps2();
    const orbit::KeplerianElements elements{6378137.0 + 200e3, 0.0, 26.5 * deg, 0.0, 0.0, phase_rad};
    s.orbit = orbit::state_from_elements(elements, gm);

    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    config::fill_tanks(cfg.stages[0], propellant_kg);
    s.ship = std::make_unique<vehicle::FlightBody>(std::make_unique<vehicle::Vehicle>(cfg, STARSIM_CONFIG_DIR),
                                                   vehicle::RigidBodyState{s.orbit.position_m, s.orbit.velocity_mps, {}, {}});
    const auto& stage = s.ship->vehicle().stages().front();
    for (const auto& e : stage.engines) {
        if (e.group_id == "ship/sea_level_3") {
            s.exhaust_velocity_mps = e.performance->thrust_n(1.0, 0.0) / e.performance->mass_flow_kgps(1.0);
        }
    }
    s.propellant_kg = propellant_kg;
    s.reserve_kg = stage.config.landing_propellant_kg;
    s.target = gnc::ship_entry_target(s.session->return_site(s.ship->vehicle()), gnc::ship_landing_parameters());
    const auto schedule = gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, stage.config.aero.entry_alpha_table);
    s.model = gnc::EntryGuidance(*s.ship, schedule, s.target).model();
    return s;
}

gnc::DeorbitPlan plan(const Setup& s, const gnc::DeorbitParameters& parameters = {}) {
    return gnc::plan_deorbit(s.orbit, s.t0, s.session->environment().gravity, earth(), s.model, s.target,
                             s.ship->vehicle().mass_properties().mass_kg, s.exhaust_velocity_mps, s.propellant_kg,
                             s.reserve_kg, parameters);
}

}  // namespace

TEST_CASE("Deorbit planner hits the entry interface for Starbase", "[ship][deorbit]") {
    const auto s = setup(0.0);
    const auto p = plan(s);
    std::printf("deorbit plan: %s%s; burn %+.0f s, %.1f m/s (%.1f t), entry interface %+.0f s at %.3f deg, %.0f m/s, "
                "downrange %.2f km, crossrange %.1f km, %d pass(es)\n",
                p.feasible ? "feasible" : "refused: ", p.reason.c_str(), p.node.epoch - s.t0, p.delta_v_mps,
                p.propellant_kg / 1000.0, p.entry_interface_epoch - s.t0, p.entry_flight_path_rad / deg,
                p.entry_speed_mps, p.downrange_error_m / 1000.0, p.crossrange_m / 1000.0, p.passes_checked);
    REQUIRE(p.feasible);
    CHECK(std::abs(p.entry_flight_path_rad + 1.0 * deg) < 0.02 * deg);
    CHECK(std::abs(p.downrange_error_m) < 2000.0);
    CHECK(std::abs(p.crossrange_m) < 60000.0);
    CHECK(p.delta_v_mps > 40.0);
    CHECK(p.delta_v_mps < 150.0);
    CHECK(p.node.epoch - s.t0 >= 600.0);
    CHECK(p.node.epoch - s.t0 <= 2.0 * 86400.0);

    // independent re-propagation of the impulsive burn reaches 120 km at the planned epoch
    auto state = orbit::propagate_coast(s.session->environment().gravity, s.orbit, s.t0, p.node.epoch).state;
    state.velocity_mps += p.node.delta_v_gcrf_mps;
    const integrators::Event<6> interface{
        "entry interface",
        [&](double t_s, const integrators::State<6>& y) {
            const Vec3 r = transpose(earth().fixed_to_gcrf(p.node.epoch + t_s).rotation) * Vec3{y[0], y[1], y[2]};
            return frames::fixed_to_geodetic(r).height_m - 120000.0;
        },
        integrators::Crossing::Falling};
    const auto coast = orbit::propagate_coast(s.session->environment().gravity, state, p.node.epoch,
                                              p.node.epoch + 6000.0, std::span(&interface, 1));
    REQUIRE(coast.event_index == 0);
    CHECK(std::abs(coast.epoch - p.entry_interface_epoch) < 1.0);
}

TEST_CASE("Deorbit planner refuses without the landing propellant", "[ship][deorbit]") {
    const auto s = setup(0.0, 37000.0);  // the burn needs ~5 t, the landing 35 t
    const auto p = plan(s);
    INFO(p.reason);
    CHECK_FALSE(p.feasible);
    CHECK(p.reason.find("Treibstoff") != std::string::npos);
}

// Refuelling r2: the main tanks boil off while the ship waits for its pass; the propellant at the burn counts.
TEST_CASE("Deorbit planner counts the boil-off until the burn", "[ship][deorbit][boil_off]") {
    const auto s = setup(0.0);  // 10 t above the landing propellant, the burn ~6 t, the pass ~22 h away
    gnc::DeorbitParameters slow;
    slow.boil_off_kgps = 100.0 / constants::seconds_per_day;
    const auto feasible = plan(s, slow);
    CHECK(feasible.feasible);
    CHECK(feasible.boil_off_kg == Catch::Approx(slow.boil_off_kgps * (feasible.node.epoch - s.t0)));
    gnc::DeorbitParameters bare;
    bare.boil_off_kgps = 10000.0 / constants::seconds_per_day;  // a bare hull's order of magnitude
    const auto p = plan(s, bare);
    INFO(p.reason);
    CHECK_FALSE(p.feasible);
    CHECK(p.short_of_reserve);
    CHECK(p.reason.find("verdampft") != std::string::npos);
}

TEST_CASE("Deorbit planner finds no pass in a short window", "[ship][deorbit]") {
    const auto s = setup(0.0);
    gnc::DeorbitParameters parameters;
    parameters.search_window_s = 1800.0;
    const auto p = plan(s, parameters);
    INFO(p.reason);
    CHECK_FALSE(p.feasible);
    CHECK(p.reason.find("Überflug") != std::string::npos);
}

// Onboard tracking (l5 Task 10): the drag coefficient is solved from a day's arc of a coast with a known C_D A / m.
TEST_CASE("Tracking estimates the orbital drag from a day's arc", "[ship][deorbit]") {
    const auto s = setup(0.0);
    const auto& gravity = s.session->environment().gravity;
    const double truth = 1.2e-3;  // [m2/kg] ~120 m2 of C_D A on 100 t (b4 fixture's drift: ~212 s in 4 days)
    const time::Epoch now = s.t0 + 86400.0;
    const auto tracked = gnc::coast_with_drag(gravity, s.orbit, s.t0, now, truth);
    const auto drag_free = gnc::coast_with_drag(gravity, s.orbit, s.t0, now, 0.0);
    const double lead_m = dot(tracked.position_m - drag_free.position_m, math::normalized(tracked.velocity_mps));
    std::printf("drag lead after a day: %.1f km\n", lead_m / 1000.0);
    CHECK(lead_m > 10e3);  // drag makes the ship gain on the drag-free coast
    const double estimate = gnc::estimate_drag_area_per_mass(gravity, s.orbit, s.t0, tracked, now, 0.0);
    CHECK(std::abs(estimate / truth - 1.0) < 0.01);
    // A short arc cannot tell: the guess comes back.
    const auto minute = gnc::coast_with_drag(gravity, s.orbit, s.t0, s.t0 + 60.0, truth);
    CHECK(gnc::estimate_drag_area_per_mass(gravity, s.orbit, s.t0, minute, s.t0 + 60.0, 7e-4) == 7e-4);
}
