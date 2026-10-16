// Earth return from lunar orbit (l5): TEI, MCC and braking targeting in the full force model.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>

#include <catch2/catch_test_macros.hpp>

#include "ship_entry_scenario.hpp"
#include "starsim/aero/radiative_heating.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/environment/gravity.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/burn_node.hpp"
#include "starsim/gnc/entry_aero_table.hpp"
#include "starsim/mission/earth_return.hpp"
#include "starsim/mission/lunar_outbound.hpp"
#include "starsim/mission/nrho_reference.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::StateVector;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double ship_mass_kg = 573e3;       // [D] l5 plan: the ship in the LLO
constexpr double ship_propellant_kg = 318e3;  // [D]
constexpr double rvac_isp_s = 380.0;

struct Fixture {
    bool session = (test::spice_session(), true);  // kernels before the members that read them
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon = spice::SpiceOrientation::moon_mean_earth();
    environment::PointMassGravity gravity{ephemeris, earth};
    mission::EarthReturnPlanner planner{gravity, ephemeris, earth};
    time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 2, 0, 0, 0.0});
};

bool verbose() { return std::getenv("STARSIM_VERBOSE") != nullptr; }

// Starbase tower (config/launch_sites.toml starbase_olp1) on the ground, ITRF
Vec3 starbase_itrf() {
    const auto site = config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1");
    return frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
}

// 100 km polar LLO at t whose plane contains the given Moon-relative direction (unit, GCRF)
StateVector llo_containing(const Fixture& f, time::Epoch t, const Vec3& direction) {
    const auto rot = f.moon.fixed_to_gcrf(t).rotation;
    const Vec3 pole{rot(0, 2), rot(1, 2), rot(2, 2)};
    const Vec3 in_plane = math::normalized(direction - pole * dot(pole, direction));
    const double r = constants::moon_mean_radius_m + 100e3;
    const double v = std::sqrt(f.ephemeris.gm_m3ps2(ephemeris::Body::Moon) / r);
    // start at the plane's crossing of the equator, moving north
    const auto moon = f.ephemeris.state_gcrf(ephemeris::Body::Moon, t);
    return {moon.position_m + in_plane * r, moon.velocity_mps + pole * v};
}

Vec3 rotated(const Vec3& v, const Vec3& axis, double angle) {
    const Vec3 k = math::normalized(axis);
    return v * std::cos(angle) + cross(k, v) * std::sin(angle) + k * (dot(k, v) * (1.0 - std::cos(angle)));
}

// The ship's onboard entry model (ship entry scenario) with radiative heating and carried skips.
gnc::EntryPredictorModel ship_entry_model() {
    const auto schedule = gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, "aero/ship_entry_alpha.csv");
    test::ShipEntryScenario scenario(test::EntryStart{});
    const auto& vehicle = scenario.body().vehicle();
    gnc::EntryPredictorModel m;
    m.aero = gnc::EntryAeroTable(vehicle, schedule);
    m.mass_kg = vehicle.mass_properties().mass_kg;
    m.nose_radius_m = vehicle.stages().front().config.heat_shield.nose_radius_m;
    m.radiative = std::make_shared<const aero::RadiativeHeatingTable>(aero::RadiativeHeatingTable::load(STARSIM_CONFIG_DIR));
    m.carry_skips = true;
    return m;
}

void print_plan(const char* what, const mission::ReturnPlan& plan, double seconds) {
    if (!verbose()) return;
    std::printf("[return] %s: feasible %d (%s), TEI %.1f m/s, braking %.1f m/s, interface %.1f m/s %.3f deg, "
                "perigee %.1f km, crossrange %.2f km, downrange %.2f km, flight %.2f d, %.1f s wall\n",
                what, plan.feasible ? 1 : 0, plan.reason.c_str(), plan.tei_delta_v_mps, plan.braking_delta_v_mps,
                plan.arrival.speed_mps, plan.arrival.flight_path_rad / deg, plan.arrival.perigee_height_m / 1000.0,
                plan.arrival.crossrange_m / 1000.0, plan.arrival.downrange_error_m / 1000.0,
                (plan.arrival.interface_epoch - plan.tei.epoch) / 86400.0, seconds);
}

double seconds_since(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

}  // namespace

TEST_CASE("TEI from an aligned LLO costs the literature's delta-v and arrives in the corridor", "[lunar][return]") {
    Fixture f;
    mission::EarthReturnParameters p;
    p.target.site_itrf_m = starbase_itrf();
    p.target.flight_path_rad = -5.5 * deg;
    p.target.speed_mps = 0.0;  // no braking: the geometry alone first
    const auto s = llo_containing(
        f, f.t0, mission::return_asymptote(f.ephemeris, f.t0 + 3600.0, p.flight_time_guess_s, p.target.site_itrf_m, f.earth));
    const auto start = std::chrono::steady_clock::now();
    const auto plan = f.planner.plan(s, f.t0, p, ship_mass_kg, rvac_isp_s, ship_propellant_kg);
    print_plan("aligned LLO", plan, seconds_since(start));
    INFO(plan.reason);
    REQUIRE(plan.feasible);
    CHECK(plan.tei_delta_v_mps > 800.0);   // [S] spec: ~0.85-0.9 km/s from the LLO
    CHECK(plan.tei_delta_v_mps < 1000.0);
    CHECK(plan.arrival.reached);
    CHECK(std::abs(plan.arrival.flight_path_rad - p.target.flight_path_rad) < 0.05 * deg);
    CHECK(std::abs(plan.arrival.crossrange_m) < 5e3);
    CHECK(plan.arrival.speed_mps > 10.4e3);  // natural lunar-return entry speed (~11.0 km/s inertial, eastward)
    CHECK(plan.arrival.speed_mps < 11.2e3);
}

TEST_CASE("TEI with the braking burn arrives at the target speed", "[lunar][return]") {
    Fixture f;
    mission::EarthReturnParameters p;
    p.target.site_itrf_m = starbase_itrf();
    p.target.flight_path_rad = -5.0 * deg;
    p.target.speed_mps = 9600.0;
    const auto s = llo_containing(
        f, f.t0, mission::return_asymptote(f.ephemeris, f.t0 + 3600.0, p.flight_time_guess_s, p.target.site_itrf_m, f.earth));
    const auto start = std::chrono::steady_clock::now();
    const auto plan = f.planner.plan(s, f.t0, p, ship_mass_kg, rvac_isp_s, ship_propellant_kg);
    print_plan("braked", plan, seconds_since(start));
    INFO(plan.reason);
    REQUIRE(plan.feasible);
    CHECK(plan.arrival.braked);
    CHECK(std::abs(plan.arrival.speed_mps - 9600.0) < 50.0);  // vis-viva sizing; the in-flight braking re-plan is exact
    CHECK(std::abs(plan.arrival.flight_path_rad - p.target.flight_path_rad) < 0.05 * deg);
    // [D] vis-viva: ~11.0 -> ~10.0 km/s inertial at the interface is 9.67 -> 8.51 km/s at 2,000 km (the plan's
    // ~1.5 km/s estimate assumed 10.9 km/s relative)
    CHECK(plan.braking_delta_v_mps > 1000.0);
    CHECK(plan.braking_delta_v_mps < 1300.0);
}

TEST_CASE("TEI from the NRHO costs less than from the LLO", "[lunar][return][slow]") {
    // [S] spec: ~0.4-0.5 km/s from the NRHO. Reference built as lunar_outbound_parameters does (hls_lunar_nrho seed).
    Fixture f;
    mission::EarthReturnParameters p;
    p.orbit = mission::LunarOrbitKind::Nrho;
    p.nrho = std::make_shared<mission::NrhoReference>(
        f.gravity, f.ephemeris, mission::NrhoSeed{{{1.0221, 0.0, -0.1821}, {0.0, -0.1033, 0.0}}, 9, 2},
        f.t0 + 86400.0, mission::NrhoReferenceOptions{.revolutions = 2});
    p.target.site_itrf_m = starbase_itrf();
    p.target.flight_path_rad = -5.5 * deg;
    p.search_s = 2.0 * 86400.0;  // the first perilune is a day after the plan
    const auto start = std::chrono::steady_clock::now();
    const auto plan = f.planner.plan(p.nrho->state_gcrf(f.t0), f.t0, p, ship_mass_kg, rvac_isp_s, ship_propellant_kg);
    print_plan("NRHO", plan, seconds_since(start));
    INFO(plan.reason);
    REQUIRE(plan.feasible);
    CHECK(plan.tei_delta_v_mps > 300.0);
    CHECK(plan.tei_delta_v_mps < 600.0);
    CHECK(std::abs(plan.arrival.flight_path_rad - p.target.flight_path_rad) < 0.05 * deg);
    CHECK(std::abs(plan.arrival.crossrange_m) < 5e3);
}

TEST_CASE("Mid-course correction after a dispersed TEI is small", "[lunar][return]") {
    // TEI executed short and off its direction; MCC 1 a day later, after it the arrival within 0.05 deg / 5 km.
    // Closed-loop burn execution (sensed delta-v) leaves ~0.1 % / 0.1 deg [E] (Apollo TEI residuals were < 0.2 m/s):
    // ~4 m/s. The outbound campaign's 1 % / 0.5 deg (the uncorrected perigee 1,900 km up, the site 6,600 km out of the
    // plane) costs ~34 m/s.
    Fixture f;
    mission::EarthReturnParameters p;
    p.target.site_itrf_m = starbase_itrf();
    p.target.flight_path_rad = -5.5 * deg;
    const auto s = llo_containing(
        f, f.t0, mission::return_asymptote(f.ephemeris, f.t0 + 3600.0, p.flight_time_guess_s, p.target.site_itrf_m, f.earth));
    const auto plan = f.planner.plan(s, f.t0, p, ship_mass_kg, rvac_isp_s, ship_propellant_kg);
    REQUIRE(plan.feasible);
    const Vec3 dv = plan.tei.delta_v_gcrf_mps;
    const auto at_tei = orbit::propagate_coast(f.gravity, s, f.t0, plan.tei.epoch).state;
    struct Dispersion { double scale; double angle_rad; double max_mcc_mps; };
    for (const auto d : {Dispersion{0.999, 0.1 * deg, 30.0}, Dispersion{0.99, 0.5 * deg, 50.0}}) {
        auto after = at_tei;
        after.velocity_mps += rotated(dv * d.scale, cross(dv, Vec3{0.0, 0.0, 1.0}), d.angle_rad);
        const time::Epoch mcc_epoch = plan.tei.epoch + p.mcc_after_tei_s.front();
        const auto mcc = f.planner.mid_course(after, plan.tei.epoch, mcc_epoch, p);
        REQUIRE(mcc);
        const auto arrival = f.planner.evaluate(after, plan.tei.epoch, mcc->epoch, mcc->delta_v_gcrf_mps, p.target);
        if (verbose()) {
            std::printf("[return] MCC 1 after TEI %.1f %% short, %.1f deg off: %.2f m/s -> %.4f deg, crossrange %.2f km\n",
                        (1.0 - d.scale) * 100.0, d.angle_rad / deg, norm(mcc->delta_v_gcrf_mps),
                        arrival.flight_path_rad / deg, arrival.crossrange_m / 1000.0);
        }
        CHECK(norm(mcc->delta_v_gcrf_mps) < d.max_mcc_mps);
        REQUIRE(arrival.reached);
        CHECK(std::abs(arrival.flight_path_rad - p.target.flight_path_rad) < 0.05 * deg);
        CHECK(std::abs(arrival.crossrange_m) < 5e3);
    }
}

TEST_CASE("The braking burn re-planned near the Earth hits speed, angle and the gate range", "[lunar][return][slow]") {
    // From the TEI plan's coast to 1 h before the braking height: braking() converges; evaluate the result: speed
    // within 5 m/s, angle within 0.02 deg, predicted downrange error at the nominal bank within 5 km.
    Fixture f;
    mission::EarthReturnParameters p;
    p.target.site_itrf_m = starbase_itrf();
    p.target.flight_path_rad = -5.0 * deg;
    p.target.speed_mps = 9600.0;
    p.target.entry_model = ship_entry_model();
    p.target.gate.position_itrf_m = p.target.site_itrf_m;
    p.target.gate.ground_height_m = 10.0;
    // Moon at its southern standstill (-27.6 deg): the inbound perigee lies north, the Starbase site within an entry
    // range of it (at the plan epoch of the other cases, Moon +19 deg, the site stays >= ~4,000 km beyond a lift-down
    // entry at every arrival time).
    const time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 12, 0, 0, 0.0});
    const auto s = llo_containing(
        f, t0, mission::return_asymptote(f.ephemeris, t0 + 3600.0, p.flight_time_guess_s, p.target.site_itrf_m, f.earth));
    const auto start = std::chrono::steady_clock::now();
    const auto plan = f.planner.plan(s, t0, p, ship_mass_kg, rvac_isp_s, ship_propellant_kg);
    print_plan("direct (gate)", plan, seconds_since(start));
    INFO(plan.reason);
    REQUIRE(plan.feasible);
    auto inbound = orbit::propagate_coast(f.gravity, s, t0, plan.tei.epoch).state;
    inbound.velocity_mps += plan.tei.delta_v_gcrf_mps;
    const auto t_brake = f.planner.braking_epoch(inbound, plan.tei.epoch, p.target);
    REQUIRE(t_brake);
    const time::Epoch now = *t_brake - 3600.0;
    const auto state_now = orbit::propagate_coast(f.gravity, inbound, plan.tei.epoch, now).state;
    const auto burn = f.planner.braking(state_now, now, p.target);
    REQUIRE(burn);
    auto coasting = p.target;
    coasting.speed_mps = 0.0;  // the burn itself is the braking
    const auto arrival = f.planner.evaluate(state_now, now, burn->epoch, burn->delta_v_gcrf_mps, coasting);
    if (verbose()) {
        std::printf("[return] braking re-plan: %.1f m/s -> %.1f m/s %.3f deg, downrange %.2f km\n",
                    norm(burn->delta_v_gcrf_mps), arrival.speed_mps, arrival.flight_path_rad / deg,
                    arrival.downrange_error_m / 1000.0);
    }
    REQUIRE(arrival.reached);
    CHECK(std::abs(arrival.speed_mps - p.target.speed_mps) < 5.0);
    CHECK(std::abs(arrival.flight_path_rad - p.target.flight_path_rad) < 0.02 * deg);
    CHECK(std::abs(arrival.downrange_error_m) < 5e3);
}

TEST_CASE("TEI from a misaligned LLO waits for the window or refuses", "[lunar][return]") {
    // Review Focus 1: LLO plane 13 deg off the asymptote (the mission-1 ascent after a 24 h stay). search_s 1 day:
    // either feasible with tei_delta_v_mps reported (> the aligned case) or refused with a German reason containing
    // "TEI"; never a converged burn above max_tei_mps.
    Fixture f;
    mission::EarthReturnParameters p;
    p.target.site_itrf_m = starbase_itrf();
    p.target.flight_path_rad = -5.5 * deg;
    const Vec3 asymptote =
        mission::return_asymptote(f.ephemeris, f.t0 + 3600.0, p.flight_time_guess_s, p.target.site_itrf_m, f.earth);
    const auto rot = f.moon.fixed_to_gcrf(f.t0).rotation;
    const Vec3 pole{rot(0, 2), rot(1, 2), rot(2, 2)};
    const auto s = llo_containing(f, f.t0, rotated(asymptote, pole, 13.0 * deg));
    const auto start = std::chrono::steady_clock::now();
    const auto plan = f.planner.plan(s, f.t0, p, ship_mass_kg, rvac_isp_s, ship_propellant_kg);
    print_plan("misaligned LLO", plan, seconds_since(start));
    // The Moon moves ~13 deg a day: the needed asymptote turns into this plane within the day (the window comes).
    if (plan.feasible) {
        CHECK(plan.tei_delta_v_mps > 800.0);
        CHECK(plan.tei_delta_v_mps <= p.max_tei_mps);
    } else {
        CHECK(plan.reason.find("TEI") != std::string::npos);
    }
}

TEST_CASE("Perigee raise at apoapsis lifts the perigee to the target", "[gnc][burn]") {
    // 120 x 400 km orbit: node at the apoapsis, prograde, ~50-60 m/s to 200 km (spec); applied impulsively the
    // two-body perigee is 200 km within 100 m.
    const double gm = constants::earth_gm_m3ps2, R = constants::wgs84_semi_major_axis_m;
    const double rp = R + 120e3, ra = R + 400e3;
    const StateVector s{{rp, 0.0, 0.0}, {0.0, std::sqrt(gm * (2.0 / rp - 2.0 / (rp + ra))), 0.0}};
    const time::Epoch now = time::Epoch::from_tdb_seconds_past_j2000(0.0);
    const auto node = gnc::raise_periapsis_at_apoapsis(s, now, gm, R + 200e3);
    REQUIRE(node);
    const double dv = norm(node->delta_v_gcrf_mps);
    if (std::getenv("STARSIM_VERBOSE")) std::printf("[return] perigee raise 120 -> 200 km: %.2f m/s\n", dv);
    CHECK(dv > 20.0);
    CHECK(dv < 60.0);
    auto at = orbit::propagate_two_body(s, gm, node->epoch - now);
    CHECK(std::abs(norm(at.position_m) - ra) < 1.0);
    at.velocity_mps += node->delta_v_gcrf_mps;
    const auto el = orbit::elements_from_state(at, gm);
    CHECK(std::abs(el.semi_major_axis_m * (1.0 - el.eccentricity) - (R + 200e3)) < 100.0);
    CHECK_FALSE(gnc::raise_periapsis_at_apoapsis(s, now, gm, R + 100e3));  // already higher
}
