// Outbound planning: TLI and insertion delta-v for the LLO and the NRHO vs the literature, MCC re-targeting after an
// execution error, LLO trims, and the propellant check.

#include <cmath>
#include <cstdio>
#include <memory>

#include <catch2/catch_test_macros.hpp>

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
    INFO(plan.reason);
    REQUIRE(plan.feasible);
    const double tli = norm(plan.tli.delta_v_gcrf_mps), loi = norm(plan.insertion.delta_v_gcrf_mps);
    std::printf("[validation] outbound LLO: TLI %.0f m/s, LOI %.0f m/s, %.0f t propellant\n", tli, loi,
                plan.propellant_kg / 1000.0);
    CHECK(tli > 3050.0);  // [S] spec: TLI ~3.1-3.2 km/s
    CHECK(tli < 3250.0);
    CHECK(loi > 800.0);  // [S] spec: LOI ~0.82-0.9 km/s
    CHECK(loi < 920.0);
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
    INFO(plan.reason);
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
    CHECK(plan.reason.find("Treibstoff") != std::string::npos);
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
