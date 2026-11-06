// Diagnostic (plan 2026-11-02, Task 14): the first stage from separation to 40 s after it, every 0.25 s: attitude, rate, inertia,
// N2, dynamic pressure, the hull's aerodynamic moment, the moment the grid fins could give, the RCS moment. Hidden; prints a table
// (mission: STARSIM_FLIP_MISSION, default falcon9_rtls).

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "starsim/aero/grid_fin.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/gnc/attitude_controller.hpp"
#include "starsim/gnc/booster_return.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/ascent_mission.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

TEST_CASE("Falcon 9 flip: moments over the turn", "[.][f9-flip-moments][ascent_planner]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const char* id = std::getenv("STARSIM_FLIP_MISSION") ? std::getenv("STARSIM_FLIP_MISSION") : "falcon9_rtls";
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto mission = config::load_ascent_mission(STARSIM_CONFIG_DIR, id);
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 13, 10, 40, 0.0});
    REQUIRE(sim::start_ascent_mission(session, mission, STARSIM_CONFIG_DIR, t0).feasible);
    double separation = -1.0;
    double n2_separation = 0.0;
    std::printf("dt_s phase off_velocity_deg elevation_deg rate_deg_s Iy_kgm2 cg_m mass_t n2_kg q_pa hull_kNm fins_max_kNm "
                "rcs_max_kNm engines thrust_kN\n");
    for (double t = 100.0; t < 260.0; t += 0.25) {
        session.advance_to(t0 + t, 1000);
        if (session.body_count() < 2) continue;
        if (separation < 0.0) separation = t;
        const auto& b = session.body(0);
        const auto& s = b.state();
        const auto tr = earth.fixed_to_gcrf(session.epoch());
        const Vec3 up = math::normalized(s.position_gcrf_m);
        const Vec3 v = s.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(tr), s.position_gcrf_m);
        const Vec3 axis = s.attitude.rotate(Vec3{1.0, 0.0, 0.0});
        const Vec3 forward = math::normalized(v - up * dot(v, up));  // downrange: elevation 0, up 90, uprange 180
        const double off = std::acos(std::clamp(dot(axis, math::normalized(v)), -1.0, 1.0)) / constants::deg_to_rad;
        const double elevation = std::atan2(dot(axis, up), dot(axis, forward)) / constants::deg_to_rad;
        const auto& w = s.angular_velocity_body_radps;
        const auto mass = b.vehicle().mass_properties();
        const auto& air = b.air_data();
        const double q = 0.5 * air.air.density_kg_m3 * dot(air.air_velocity_body_mps, air.air_velocity_body_mps);
        const Vec3 hull = gnc::AttitudeController::modelled_hull_moment_nm(b);
        double fins = 0.0;  // the pitch-and-yaw moment with all fins at full deflection, sum of the magnitudes
        for (const auto& stage : b.vehicle().stages()) {
            for (const auto& fin : stage.grid_fins) {
                const Vec3 arm = Vec3{fin.geometry.station_m - mass.cg_station_m, 0.0, 0.0} +
                                 aero::grid_fin_radial(fin.geometry) * fin.geometry.radius_m;
                const double k = aero::grid_fin_effectiveness(fin.geometry, air.air_velocity_body_mps, air.air,
                                                              *stage.grid_fin_table);
                fins += norm(cross(arm, aero::grid_fin_tangential(fin.geometry))) * std::abs(k) * fin.max_deflection_rad;
            }
        }
        const auto* status = session.return_status(0);
        const auto tel = session.telemetry(0);
        std::printf("%.2f %s %.1f %.1f %.2f %.3e %.1f %.1f %.1f %.1f %.1f %.1f %.1f %d %.0f\n", t - separation,
                    status ? gnc::id_of(status->phase) : "-", off, elevation,
                    std::hypot(w.y, w.z) / constants::deg_to_rad, mass.inertia_kg_m2.y, mass.cg_station_m, mass.mass_kg / 1e3,
                    b.vehicle().rcs_supply_kg(), q, norm(hull) / 1e3, fins / 1e3, b.vehicle().max_rcs_torque_nm().y / 1e3,
                    tel.engines_running, tel.thrust_n / 1e3);
        if (t - separation >= 40.0) break;
    }
    (void)n2_separation;
}
