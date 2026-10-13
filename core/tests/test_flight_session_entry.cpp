// The flight session flies an entering ship on its entry guidance (full environment: SPICE Earth, gravity with J2,
// Moon and Sun) from the supersonic end of the entry west of Starbase to the gate ahead of the tower, and on to the catch.

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

// 28 km west of the pad, 26 km up, flying east at Mach ~2.6, 8 deg down; belly first at the scheduled attitude. With
// energy to spare the belly flop falls steeply and turns the nose to the tower; from 33-39 km (most lift to reach the
// gate) the fall stays shallow too long and the gate is missed by up to 0.8 km (b3 ledger).
void place_supersonic_ship(sim::FlightSession& session, time::Epoch t0) {
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1")));
    const auto cfg = entering_ship();
    const double speed = 780.0, gamma = -8.0 * deg;
    auto state = test::state_over_site(session.launch_site(), earth(), t0, Vec3{-28000.0, 0.0, 26000.0},
                                       Vec3{speed * std::cos(gamma), 0.0, speed * std::sin(gamma)});
    const auto t = earth().fixed_to_gcrf(t0);
    const Vec3 v_rel = state.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), state.position_gcrf_m);
    state.attitude = gnc::entry_attitude_quaternion(v_rel, math::normalized(state.position_gcrf_m), 65.0 * deg, 0.0,
                                                    cfg.stages[0].aero.belly_azimuth_rad);
    state.angular_velocity_body_radps = state.attitude.inverse_rotate(vehicle::earth_rotation_vector(t));
    session.place_entering_ship(cfg, state, t0);
}

// Advances until the entry guidance reaches its gate (or the ship impacts, or 600 s); returns the seconds flown.
double fly_to_gate(sim::FlightSession& session, time::Epoch t0) {
    double time_s = 0.0;
    while (session.entry_guidance(0)->phase() != gnc::EntryPhase::Gate && !session.body(0).impacted() && time_s < 600.0) {
        // fine steps near the gate: the state there is the flip's start
        time_s += session.telemetry(0).altitude_m - session.launch_site().height_m < 3000.0 ? 0.05 : 1.0;
        session.advance_to(t0 + time_s);
        if (std::getenv("STARSIM_VERBOSE") && static_cast<int>(time_s) % 10 == 0) {
            const auto* s = session.entry_status(0);
            const auto tel = session.telemetry(0);
            std::printf("t %.0f %s h %.0f Mach %.2f alpha %.1f bank %.1f downrange %.0f cross %.0f\n", time_s,
                        gnc::to_string(s->phase), tel.altitude_m, tel.mach, s->alpha_rad / deg, s->bank_rad / deg,
                        s->downrange_error_m, s->crossrange_m);
        }
    }
    return time_s;
}

}  // namespace

TEST_CASE("Session flies an entering ship to the gate over Starbase", "[ship][guidance][session]") {
    test::spice_session();
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 13, 12, 0, 0.0});
    sim::FlightSession session(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    place_supersonic_ship(session, t0);
    const double time_s = fly_to_gate(session, t0);

    // the gate lies back from the tower's catch axis against the arrival heading (b3)
    const auto& target = session.entry_guidance(0)->target();
    const auto tr = earth().fixed_to_gcrf(session.epoch());
    const Vec3 r = transpose(tr.rotation) * session.body(0).state().position_gcrf_m;
    const Vec3 up = math::normalized(r);
    const Vec3 d = r - target.position_itrf_m;
    const double miss = norm(d - up * dot(d, up));
    const auto& site = session.launch_site();
    const Vec3 pad = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    const Vec3 lead = target.position_itrf_m - pad;
    const Vec3 nose = transpose(tr.rotation) * session.body(0).state().attitude.rotate(Vec3{1.0, 0.0, 0.0});
    const Vec3 nose_h = nose - up * dot(nose, up);
    const double nose_off_deg =
        std::atan2(norm(cross(nose_h, target.nose_direction_itrf)), dot(nose_h, target.nose_direction_itrf)) / deg;
    const auto* status = session.entry_status(0);
    REQUIRE(status != nullptr);
    std::printf("session entry: %s after %.0f s, height %.0f m, miss %.1f m (gate %.0f m from the pad), nose %.1f deg off the tower direction\n",
                gnc::to_string(status->phase), time_s, dot(d, up), miss, norm(lead - up * dot(lead, up)),
                nose_off_deg);
    CHECK(status->phase == gnc::EntryPhase::Gate);
    CHECK_FALSE(session.body(0).impacted());
    CHECK(norm(lead - up * dot(lead, up)) > 100.0);
    CHECK(miss < 150.0);
    CHECK(nose_off_deg < 15.0);  // the nose sector is +-10 deg
}

// The whole return below Mach 3 in one session: entry guidance to the gate (nose turned to the tower in the steep
// belly flop), flip, SCvx landing burn, catch.
TEST_CASE("Session flies an entering ship from Mach 2.6 into the tower's arms", "[ship][catch][session]") {
    test::spice_session();
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 13, 12, 0, 0.0});
    sim::FlightSession session(spice_ephemeris(), earth(), STARSIM_CONFIG_DIR);
    place_supersonic_ship(session, t0);
    const double time_s = fly_to_gate(session, t0);
    REQUIRE(session.entry_guidance(0)->phase() == gnc::EntryPhase::Gate);
    const auto gate = session.telemetry(0);
    std::printf("gate: height %.0f m, speed %.1f m/s, sink %.1f m/s, flight path %.1f deg, alpha %.1f deg, %.1f t\n",
                gate.altitude_m - session.launch_site().height_m, gate.speed_relative_mps, -gate.vertical_speed_mps,
                gate.flight_path_angle_rad / deg, gate.angle_of_attack_rad / deg, gate.propellant_kg / 1000.0);
    const auto run = test::fly_landing(session, earth(), t0 + time_s, 120.0);
    const auto* g = session.ship_landing(0);
    REQUIRE(g != nullptr);
    std::printf("entry to catch: gate after %.0f s, flip %.1f s ending at %.0f m; %s, offset %.2f m, lateral %.2f m/s, "
                "sink %.2f m/s, %.1f t propellant left\n",
                time_s, g->flip_duration_s(), g->flip_end_height_m(), sim::to_string(run.tower_phase), run.offset_m,
                run.lateral_mps, run.sink_mps, session.body(0).vehicle().propellant_kg() / 1000.0);
    CHECK_FALSE(run.tower_struck);
    CHECK(run.caught_within_tolerance());
}
