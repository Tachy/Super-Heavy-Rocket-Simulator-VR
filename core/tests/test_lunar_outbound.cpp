// Outbound planning: TLI and insertion delta-v for the LLO and the NRHO vs the literature, MCC re-targeting after an
// execution error, LLO trims, and the propellant check.

#include <cmath>
#include <cstdio>
#include <memory>

#include <algorithm>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/environment/gravity.hpp"
#include "starsim/mission/lunar_outbound.hpp"
#include "starsim/mission/lunar_transfer.hpp"
#include "starsim/mission/nrho_reference.hpp"
#include "starsim/mission/position_targeter.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {
constexpr double deg = constants::deg_to_rad;
struct Fixture {
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon = spice::SpiceOrientation::moon_mean_earth();
    environment::PointMassGravity gravity{ephemeris, earth};
    mission::LunarTransferTargeter targeter{gravity, ephemeris, moon};
    mission::LunarOutboundPlanner planner{gravity, ephemeris, moon};
    time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 2, 0, 0, 0.0});
    math::StateVector parking(time::Epoch arrival) const {
        return orbit::state_from_elements({6378137.0 + 200e3, 0.0, 28.5 * deg,
                                           targeter.parking_orbit_raan(arrival, 28.5 * deg).value(), 0.0, 0.0},
                                          ephemeris.gm_m3ps2(ephemeris::Body::Earth));
    }
};
constexpr double hls_mass_kg = 1720e3;  // 120 t dry + 1,600 t
constexpr double rvac_isp_s = 380.0;    // [S] raptor_vacuum3.toml
}  // namespace

TEST_CASE("Outbound plan to the 100 km polar LLO", "[mission][lunar][outbound][validation]") {
    test::spice_session();
    Fixture fx;
    mission::LunarOutboundParameters p;
    p.arrival_epoch = fx.t0 + 4.0 * 86400.0;
    const auto plan = fx.planner.plan(fx.parking(p.arrival_epoch), fx.t0, p, hls_mass_kg, rvac_isp_s, 1600e3);
    INFO(report::describe(plan.reason));
    REQUIRE(plan.feasible);
    const double tli = norm(plan.tli.delta_v_gcrf_mps), loi = norm(plan.insertion.delta_v_gcrf_mps);
    std::printf("[validation] outbound LLO: TLI %.0f m/s, LOI %.0f m/s, %.0f t propellant\n", tli, loi,
                plan.propellant_kg / 1000.0);
    CHECK(tli > 3050.0);  // [S] spec: TLI ~3.1-3.2 km/s
    CHECK(tli < 3250.0);
    CHECK(loi > 800.0);  // [S] spec: LOI ~0.82-0.9 km/s
    CHECK(loi < 920.0);
}

namespace {
// The LLO through a site (plan 2026-11-06 Task 7): planned with the plane through the site at the touchdown, flown
// impulsively in the full model (TLI, LOI at the plan's periselene), coasted to the touchdown; the site's distance from
// the orbit plane then (m). Prints the plan's numbers.
double llo_site_distance_m(Fixture& fx, const char* site_id) {
    const auto site = config::load_lunar_site(STARSIM_CONFIG_DIR, site_id);
    mission::LunarOutboundParameters p;
    p.arrival_epoch = fx.t0 + 4.0 * 86400.0;
    p.llo_site = mission::LloSiteTarget{site.latitude_rad, site.longitude_rad, p.arrival_epoch + 8.0 * 3600.0};
    const auto parking = fx.parking(p.arrival_epoch);
    const auto plan = fx.planner.plan(parking, fx.t0, p, hls_mass_kg, rvac_isp_s, 1600e3);
    INFO(report::describe(plan.reason));
    REQUIRE(plan.feasible);
    REQUIRE(plan.llo_plane_normal_moon_inertial);
    const mission::PositionTargeter coast{fx.gravity};
    auto s = coast.coast(parking, fx.t0, plan.tli.epoch);
    s.velocity_mps += plan.tli.delta_v_gcrf_mps;
    s = coast.coast(s, plan.tli.epoch, plan.insertion.epoch);
    s.velocity_mps += plan.insertion.delta_v_gcrf_mps;
    REQUIRE(plan.llo_touchdown);
    const time::Epoch touchdown = *plan.llo_touchdown;
    s = coast.coast(s, plan.insertion.epoch, touchdown);
    const auto rel = mission::moon_relative(fx.ephemeris, s, touchdown);
    const math::Vec3 h = math::normalized(cross(rel.position_m, rel.velocity_mps));
    const auto rotation = fx.moon.fixed_to_gcrf(touchdown).rotation;
    const math::Vec3 site_dir = rotation * math::Vec3{std::cos(site.latitude_rad) * std::cos(site.longitude_rad),
                                                      std::cos(site.latitude_rad) * std::sin(site.longitude_rad),
                                                      std::sin(site.latitude_rad)};
    const math::Vec3 pole = rotation * math::Vec3{0.0, 0.0, 1.0};
    const double r = constants::moon_mean_radius_m;
    const double distance = r * std::asin(std::abs(dot(h, site_dir)));
    const double planned = r * std::asin(std::abs(dot(*plan.llo_plane_normal_moon_inertial, site_dir)));
    const auto el = orbit::elements_from_state(rel, fx.ephemeris.gm_m3ps2(ephemeris::Body::Moon));
    std::printf("[validation] LLO through %s: TLI %.1f m/s, LOI %.1f m/s, i %.2f deg, LLO %.1f x %.1f km, site %.2f km "
                "off the plane at touchdown (planned plane %.3f km), arrival %+.2f h\n",
                site_id, norm(plan.tli.delta_v_gcrf_mps), norm(plan.insertion.delta_v_gcrf_mps),
                std::acos(std::clamp(dot(h, pole), -1.0, 1.0)) / deg,
                (el.semi_major_axis_m * (1.0 - el.eccentricity) - r) / 1000.0,
                (el.semi_major_axis_m * (1.0 + el.eccentricity) - r) / 1000.0, distance / 1000.0, planned / 1000.0,
                (plan.insertion.epoch - p.arrival_epoch) / 3600.0);
    CHECK(dot(h, *plan.llo_plane_normal_moon_inertial) > 0.0);  // the planned sense of motion
    CHECK(norm(plan.insertion.delta_v_gcrf_mps) > 800.0);         // [S] spec: LOI ~0.82-0.9 km/s, as the polar LLO
    CHECK(norm(plan.insertion.delta_v_gcrf_mps) < 920.0);
    return distance;
}
}  // namespace

TEST_CASE("LLO for Apollo 11 at 0.7 N passes the site within 20 km at touchdown", "[mission][lunar][outbound][validation]") {
    test::spice_session();
    Fixture fx;
    CHECK(llo_site_distance_m(fx, "apollo_11") < 20e3);  // max_crossrange_km: the descent's pass search takes it
}

TEST_CASE("LLO for Apollo 15 at 26 N passes the site within 20 km at touchdown", "[mission][lunar][outbound][validation]") {
    test::spice_session();
    Fixture fx;
    CHECK(llo_site_distance_m(fx, "apollo_15") < 20e3);
}

TEST_CASE("The LLO plane through a site holds the site and the asymptote in the arrival's sense", "[mission][lunar][outbound]") {
    test::spice_session();
    Fixture fx;
    const auto rotation = fx.moon.fixed_to_gcrf(fx.t0).rotation;
    const math::Vec3 pole = rotation * math::Vec3{0.0, 0.0, 1.0};
    const double lat = 20.0 * deg, lon = 30.0 * deg;
    const math::Vec3 site = rotation * math::Vec3{std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
    const math::Vec3 asymptote = math::normalized(rotation * math::Vec3{-0.3, 1.0, 0.05});
    const auto n = mission::llo_plane_through_site(fx.moon, lat, lon, fx.t0, asymptote);
    CHECK(std::abs(norm(n) - 1.0) < 1e-12);
    CHECK(std::abs(dot(n, site)) < 1e-12);
    CHECK(std::abs(dot(n, asymptote)) < 1e-12);
    CHECK(dot(n, pole) >= 0.0);
    // the reversed asymptote gives the same plane (same sense: prograde)
    CHECK(dot(mission::llo_plane_through_site(fx.moon, lat, lon, fx.t0, -asymptote), n) > 1.0 - 1e-12);
    // the sense of a given arrival plane (turned from it, not reversed)
    CHECK(dot(mission::llo_plane_through_site(fx.moon, lat, lon, fx.t0, asymptote, -pole), n) < -1.0 + 1e-12);
    // a site on the asymptote: the plane through it of least inclination
    const math::Vec3 on_axis = mission::llo_plane_through_site(fx.moon, lat, lon, fx.t0, site);
    CHECK(std::abs(dot(on_axis, site)) < 1e-12);
    CHECK(std::acos(dot(on_axis, pole)) == Catch::Approx(lat).margin(1e-9));
}

TEST_CASE("Outbound plan to the NRHO", "[mission][lunar][outbound][nrho][validation][slow]") {
    test::spice_session();
    Fixture fx;
    mission::LunarOutboundParameters p;
    p.orbit = mission::LunarOrbitKind::Nrho;
    p.arrival_epoch = fx.t0 + 4.5 * 86400.0;
    p.nrho = std::make_shared<mission::NrhoReference>(
        fx.gravity, fx.ephemeris, mission::NrhoSeed{{{1.0221, 0.0, -0.1821}, {0.0, -0.1033, 0.0}}, 9, 2},
        p.arrival_epoch);
    const auto plan = fx.planner.plan(fx.parking(p.arrival_epoch), fx.t0, p, hls_mass_kg, rvac_isp_s, 1600e3);
    INFO(report::describe(plan.reason));
    REQUIRE(plan.feasible);
    REQUIRE(plan.powered_flyby);
    const double tli = norm(plan.tli.delta_v_gcrf_mps), plsb = norm(plan.powered_flyby->delta_v_gcrf_mps),
                 nri = norm(plan.insertion.delta_v_gcrf_mps);
    std::printf("[validation] outbound NRHO: TLI %.0f m/s, swing-by %.0f m/s (%.2f d before insertion), "
                "insertion %.0f m/s, %.0f t propellant\n",
                tli, plsb, (plan.insertion.epoch - plan.powered_flyby->epoch) / 86400.0, nri,
                plan.propellant_kg / 1000.0);
    CHECK(tli > 3050.0);
    CHECK(tli < 3250.0);
    // [S] Kikuchi, Matsumoto & Ueda, ISSFD 2024 (in-direct transfer, 9:2 NRHO): swing-by 173.3 + insertion
    // 216.9 = 390.2 m/s, insertion 1.2 d after the swing-by. Band for our inclination grid (no optimisation of the
    // swing-by altitude or the insertion anomaly): 300-550 m/s.
    CHECK(plsb + nri > 300.0);
    CHECK(plsb + nri < 550.0);
    CHECK(plan.insertion.epoch - plan.powered_flyby->epoch > 0.25 * 86400.0);
    CHECK(plan.insertion.epoch - plan.powered_flyby->epoch < 3.0 * 86400.0);

    // In flight: on the planned trajectory no MCC is needed, and the re-planned swing-by and insertion match the plan.
    const auto parking = fx.parking(p.arrival_epoch);
    const mission::PositionTargeter coast{fx.gravity};
    auto after_tli = coast.coast(parking, fx.t0, plan.tli.epoch);
    after_tli.velocity_mps += plan.tli.delta_v_gcrf_mps;
    CHECK_FALSE(fx.planner.mid_course(after_tli, plan.tli.epoch, plan.tli.epoch + 86400.0, p, plan));
    const auto swing = fx.planner.powered_flyby(after_tli, plan.tli.epoch, p, plan);
    REQUIRE(swing);
    CHECK(norm(swing->delta_v_gcrf_mps - plan.powered_flyby->delta_v_gcrf_mps) < 1.0);
    auto after_swing = coast.coast(after_tli, plan.tli.epoch, swing->epoch);
    after_swing.velocity_mps += swing->delta_v_gcrf_mps;
    const auto nrhoi = fx.planner.insertion(after_swing, swing->epoch, p, plan);
    REQUIRE(nrhoi);
    CHECK(nrhoi->epoch == plan.insertion.epoch);
    CHECK(norm(nrhoi->delta_v_gcrf_mps - plan.insertion.delta_v_gcrf_mps) < 1.0);
    // A mid-course correction after the swing-by would aim at the swing-by point in the past: refused
    // (final review I1; the solve did not converge before the explicit guard either).
    auto off_swing = after_swing;
    off_swing.velocity_mps += math::Vec3{0.0, 20.0, 0.0};
    const auto late = fx.planner.mid_course(off_swing, swing->epoch, swing->epoch + 6.0 * 3600.0, p, plan);
    CHECK_FALSE(late);
    // After the insertion the flight is on the reference.
    auto inserted = coast.coast(after_swing, swing->epoch, nrhoi->epoch);
    inserted.velocity_mps += nrhoi->delta_v_gcrf_mps;
    const auto later = nrhoi->epoch + 86400.0;
    CHECK(norm(coast.coast(inserted, nrhoi->epoch, later).position_m - p.nrho->state_gcrf(later).position_m) < 20e3);
}

TEST_CASE("A mid-course correction removes a TLI execution error", "[mission][lunar][outbound]") {
    test::spice_session();
    Fixture fx;
    mission::LunarOutboundParameters p;
    p.arrival_epoch = fx.t0 + 4.0 * 86400.0;
    const auto parking = fx.parking(p.arrival_epoch);
    const auto plan = fx.planner.plan(parking, fx.t0, p, hls_mass_kg, rvac_isp_s, 1600e3);
    REQUIRE(plan.feasible);
    const mission::PositionTargeter coast{fx.gravity};
    auto after = coast.coast(parking, fx.t0, plan.tli.epoch);
    after.velocity_mps += 1.002 * plan.tli.delta_v_gcrf_mps;  // 0.2 % overburn (~6 m/s)
    const auto t_mcc = plan.tli.epoch + 86400.0;
    const auto mcc = fx.planner.mid_course(after, plan.tli.epoch, t_mcc, p, plan);
    REQUIRE(mcc);
    auto corrected = coast.coast(after, plan.tli.epoch, t_mcc);
    corrected.velocity_mps += mcc->delta_v_gcrf_mps;
    const auto arrival = fx.targeter.evaluate(corrected, t_mcc, t_mcc, {});
    std::printf("MCC %.2f m/s -> periselene %.3f km, i %.3f deg\n", norm(mcc->delta_v_gcrf_mps),
                arrival.periselene_altitude_m / 1000.0, arrival.inclination_rad / deg);
    CHECK(std::abs(arrival.periselene_altitude_m - 100e3) < 500.0);
    CHECK(std::abs(arrival.inclination_rad - 90.0 * deg) < 0.05 * deg);
    // On the planned trajectory itself no correction is needed.
    auto nominal = coast.coast(parking, fx.t0, plan.tli.epoch);
    nominal.velocity_mps += plan.tli.delta_v_gcrf_mps;
    CHECK_FALSE(fx.planner.mid_course(nominal, plan.tli.epoch, t_mcc, p, plan));
}

TEST_CASE("Outbound plan is refused without the propellant", "[mission][lunar][outbound]") {
    test::spice_session();
    Fixture fx;
    mission::LunarOutboundParameters p;
    p.arrival_epoch = fx.t0 + 4.0 * 86400.0;
    // 200 t on board: 3,726 m/s * ln(320 / 120) = 3.65 km/s < 3.1 + 0.85 km/s. (Half tanks, 800 t, would still give
    // 7.6 km/s: enough for the outbound flight alone.)
    const auto plan = fx.planner.plan(fx.parking(p.arrival_epoch), fx.t0, p, 320e3, rvac_isp_s, 200e3);
    CHECK_FALSE(plan.feasible);
    CHECK(plan.reason.code == report::Code::lunar_outbound_insufficient_propellant);
}

TEST_CASE("LLO trim sets the opposite apsis", "[mission][lunar][outbound]") {
    test::spice_session();
    Fixture fx;
    mission::LunarOutboundParameters p;
    const double gm_moon = fx.ephemeris.gm_m3ps2(ephemeris::Body::Moon);
    const double r_moon = constants::moon_mean_radius_m;
    // 95 x 104 km, at periselene: the trim at periselene raises/lowers the aposelene to 100 km.
    const double rp = r_moon + 95e3, ra = r_moon + 104e3;
    const auto rel =
        orbit::state_from_elements({0.5 * (rp + ra), (ra - rp) / (ra + rp), 90.0 * deg, 0.3, 0.0, 0.0}, gm_moon);
    const auto moon = fx.ephemeris.state_gcrf(ephemeris::Body::Moon, fx.t0);
    const math::StateVector state{rel.position_m + moon.position_m, rel.velocity_mps + moon.velocity_mps};
    const auto trim = fx.planner.llo_trim(state, fx.t0, p);
    REQUIRE(trim);
    CHECK(trim->epoch - fx.t0 < 60.0);  // at this periselene (or within one minute of it)
    CHECK(norm(trim->delta_v_gcrf_mps) < 5.0);
}

TEST_CASE("LLO trim keeps the minimum lead: the next apsis after it", "[mission][lunar][outbound]") {
    test::spice_session();
    Fixture fx;
    mission::LunarOutboundParameters p;
    const double gm_moon = fx.ephemeris.gm_m3ps2(ephemeris::Body::Moon);
    const double r_moon = constants::moon_mean_radius_m;
    const double rp = r_moon + 95e3, ra = r_moon + 104e3;
    const auto rel =
        orbit::state_from_elements({0.5 * (rp + ra), (ra - rp) / (ra + rp), 90.0 * deg, 0.3, 0.0, 0.0}, gm_moon);
    const auto moon = fx.ephemeris.state_gcrf(ephemeris::Body::Moon, fx.t0);
    const math::StateVector state{rel.position_m + moon.position_m, rel.velocity_mps + moon.velocity_mps};
    const double half_period = constants::pi * std::sqrt(std::pow(0.5 * (rp + ra), 3) / gm_moon);
    const auto trim = fx.planner.llo_trim(state, fx.t0, p, 180.0);  // at periselene now: the aposelene next
    REQUIRE(trim);
    CHECK(trim->epoch - fx.t0 >= 180.0);
    CHECK(std::abs(trim->epoch - fx.t0 - half_period) < 60.0);
    CHECK(norm(trim->delta_v_gcrf_mps) < 5.0);
}
