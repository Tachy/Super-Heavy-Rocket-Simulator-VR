// The flight session flies an entering ship on its entry guidance (full environment: SPICE Earth, gravity with J2,
// Moon and Sun) from the supersonic end of the entry west of Starbase to the gate above the pad.

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/gnc/entry_attitude.hpp"
#include "starsim/math/constants.hpp"
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

// The ship alone with its landing propellant in the main tanks (as the entry tests).
config::VehicleConfig entering_ship() {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    auto& ship = cfg.stages[0];
    double full = 0.0;
    for (const auto& tank : ship.tanks) full += tank.mass_kg;
    for (auto& tank : ship.tanks) tank.mass_kg *= ship.landing_propellant_kg / full;
    for (auto& f : ship.flaps) f.initial_deflection_rad = 0.5 * f.max_deflection_rad;
    return cfg;
}

}  // namespace

TEST_CASE("Session flies an entering ship to the gate over Starbase", "[ship][guidance][session]") {
    test::spice_session();
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 13, 12, 0, 0.0});
    sim::FlightSession session(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1")));
    const auto cfg = entering_ship();

    // 39 km west, 26 km up, flying east at Mach ~2.6, 8 deg down; belly first at the scheduled attitude (from 45 km
    // even the most lift ends 3.9 km short)
    const double speed = 780.0, gamma = -8.0 * deg;
    auto state = test::state_over_site(session.launch_site(), earth(), t0, Vec3{-39000.0, 0.0, 26000.0},
                                       Vec3{speed * std::cos(gamma), 0.0, speed * std::sin(gamma)});
    const auto t = earth().fixed_to_gcrf(t0);
    const Vec3 v_rel = state.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), state.position_gcrf_m);
    state.attitude = gnc::entry_attitude_quaternion(v_rel, math::normalized(state.position_gcrf_m), 65.0 * deg, 0.0,
                                                    cfg.stages[0].aero.belly_azimuth_rad);
    state.angular_velocity_body_radps = state.attitude.inverse_rotate(vehicle::earth_rotation_vector(t));
    session.place_entering_ship(cfg, state, t0);

    double time_s = 0.0;
    while (session.entry_guidance(0)->phase() != gnc::EntryPhase::Gate && !session.body(0).impacted() && time_s < 600.0) {
        time_s += 1.0;
        session.advance_to(t0 + time_s);
        if (std::getenv("STARSIM_VERBOSE") && static_cast<int>(time_s) % 10 == 0) {
            const auto* s = session.entry_status(0);
            const auto tel = session.telemetry(0);
            std::printf("t %.0f %s h %.0f Mach %.2f alpha %.1f bank %.1f downrange %.0f cross %.0f\n", time_s,
                        gnc::to_string(s->phase), tel.altitude_m, tel.mach, s->alpha_rad / deg, s->bank_rad / deg,
                        s->downrange_error_m, s->crossrange_m);
        }
    }
    const auto& site = session.launch_site();
    const auto tr = earth().fixed_to_gcrf(session.epoch());
    const Vec3 r = transpose(tr.rotation) * session.body(0).state().position_gcrf_m;
    const Vec3 pad = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    const Vec3 up = math::normalized(r);
    const Vec3 d = r - pad;
    const double miss = norm(d - up * dot(d, up));
    const auto* status = session.entry_status(0);
    REQUIRE(status != nullptr);
    std::printf("session entry: %s after %.0f s, height %.0f m, miss %.1f m\n", gnc::to_string(status->phase), time_s,
                dot(d, up), miss);
    CHECK(status->phase == gnc::EntryPhase::Gate);
    CHECK_FALSE(session.body(0).impacted());
    CHECK(miss < 150.0);
}
