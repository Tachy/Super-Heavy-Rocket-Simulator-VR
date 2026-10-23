// Diagnostic (follow-ups Task 7b): the CRS-35 mission's first stage from T+120 s to the landing, every 0.5 s, for the
// comparison with the webcast table (tools/telemetry/boostback_fit.py --simulated). Hidden; writes the csv named by
// STARSIM_BOOSTBACK_DUMP (another mission: STARSIM_BOOSTBACK_MISSION, e.g. falcon9_rtls).

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/booster_return.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/ascent_mission.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/landing_outcome.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

TEST_CASE("Falcon 9 CRS-35 first stage profile dump", "[.][f9-boostback-dump]") {
    const char* out_path = std::getenv("STARSIM_BOOSTBACK_DUMP");
    if (!out_path) SKIP("set STARSIM_BOOSTBACK_DUMP=<file.csv>");
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    const auto t0 = time.from_utc({2026, 10, 13, 10, 33, 0.0});
    const char* mission_id =
        std::getenv("STARSIM_BOOSTBACK_MISSION") ? std::getenv("STARSIM_BOOSTBACK_MISSION") : "crs35_ascent";
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto mission = config::load_ascent_mission(STARSIM_CONFIG_DIR, mission_id);
    const auto plan = sim::start_ascent_mission(session, mission, STARSIM_CONFIG_DIR, t0);
    REQUIRE(plan.feasible);
    std::ofstream out(out_path);
    out << "t_s,phase,alt_km,speed_kmh,vz_mps,vh_down_mps,vh_cross_mps,gamma_deg,mass_t,prop_t,n2_kg,thrust_kn,engines,"
           "axis_up_deg,axis_down_deg,q_pa,mach,sensed_g,east_km,north_km\n";
    const auto site = sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "ccsfs_lz40"));
    Vec3 down_enu{};
    bool have_dir = false;
    for (double t = 0.25; t < 600.0; t += 0.25) {
        session.advance_to(t0 + t, 1000);
        const auto& b = session.body(0);
        const auto tel = session.telemetry(0);
        const auto tr = earth.fixed_to_gcrf(session.epoch());
        const auto& s = b.state();
        const Vec3 r_fixed = transpose(tr.rotation) * s.position_gcrf_m;
        const Vec3 v_fixed =
            transpose(tr.rotation) * (s.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(tr), s.position_gcrf_m));
        const auto geo = frames::fixed_to_geodetic(r_fixed);
        const double la = geo.latitude_rad, lo = geo.longitude_rad;
        const Vec3 east{-std::sin(lo), std::cos(lo), 0.0};
        const Vec3 north{-std::sin(la) * std::cos(lo), -std::sin(la) * std::sin(lo), std::cos(la)};
        const Vec3 up{std::cos(la) * std::cos(lo), std::cos(la) * std::sin(lo), std::sin(la)};
        const Vec3 v_enu{dot(v_fixed, east), dot(v_fixed, north), dot(v_fixed, up)};
        if (!have_dir && t > 120.0) {
            down_enu = math::normalized(Vec3{v_enu.x, v_enu.y, 0.0});
            have_dir = true;
        }
        const Vec3 cross_enu{-down_enu.y, down_enu.x, 0.0};
        const Vec3 axis_fixed = transpose(tr.rotation) * s.attitude.rotate(Vec3{1.0, 0.0, 0.0});
        const Vec3 axis_enu{dot(axis_fixed, east), dot(axis_fixed, north), dot(axis_fixed, up)};
        const auto* status = session.return_status(0);
        const Vec3 p = sim::site_enu_m(site, r_fixed);
        if (t < 120.0 || std::fmod(t, 0.5) != 0.0) continue;
        out << session.mission_time_s() << ',' << (status ? gnc::to_string(status->phase) : "-") << ','
            << tel.altitude_m / 1e3 << ',' << tel.speed_relative_mps * 3.6 << ',' << v_enu.z << ',' << dot(v_enu, down_enu)
            << ',' << dot(v_enu, cross_enu) << ',' << tel.flight_path_angle_rad / constants::deg_to_rad << ','
            << tel.mass_kg / 1e3 << ',' << tel.propellant_kg / 1e3 << ',' << b.vehicle().rcs_supply_kg() << ','
            << tel.thrust_n / 1e3 << ',' << tel.engines_running << ','
            << std::asin(std::clamp(axis_enu.z, -1.0, 1.0)) / constants::deg_to_rad << ','
            << dot(axis_enu, down_enu) << ',' << tel.dynamic_pressure_pa << ',' << tel.mach << ','
            << tel.sensed_acceleration_g << ',' << p.x / 1e3 << ',' << p.y / 1e3 << '\n';
        if (session.leg_touchdown(0) != sim::LegTouchdown::None || b.impacted()) break;
    }
}
