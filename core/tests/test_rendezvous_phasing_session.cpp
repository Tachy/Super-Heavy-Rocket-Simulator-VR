// ISS i3: the rendezvous flight in the flight session - Draco burns, drag tracking, re-plan, arrival at Waypoint 0.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "rendezvous_test_support.hpp"
#include "starsim/mission/rendezvous_geometry.hpp"
#include "starsim/orbit/clohessy_wiltshire.hpp"
#include "starsim/sim/flight_session.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
struct Flight {
    const spice::SpiceEphemeris ephemeris;
    const spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter clock;
    sim::FlightSession session{ephemeris, earth, rendezvous_test::config_dir};
    rendezvous_test::Start start;

    Flight(double lead_deg, double plane_match_s) {
        start = rendezvous_test::start(clock, earth, lead_deg, plane_match_s);
        session.place_in_orbit(config::load_vehicle(rendezvous_test::config_dir, "crew_dragon"), start.dragon,
                               start.epoch);
        session.add_station(start.iss.vehicle, start.iss.state_gcrf, start.iss.epoch, start.iss.impulses);
    }

    double min_range_m = 1e12;  // over the whole flight (fly), sampled every minute

    // Flies until the step machine ends; returns the smallest approach-ellipsoid metric before the approach initiation.
    double fly(double max_hours) {
        auto& s = session;
        const size_t iss = *s.station_index();
        double min_metric = 1e9;
        const auto end = start.epoch + max_hours * 3600.0;
        while (s.epoch() - end < 0.0 && s.rendezvous_step() != sim::RendezvousStep::None &&
               s.rendezvous_step() != sim::RendezvousStep::Waypoint0) {
            s.advance_to(std::min(s.epoch() + 60.0, end), 100000);
            min_range_m = std::min(min_range_m, norm(s.body(iss).state().position_gcrf_m -
                                                     s.body(s.controlled_index()).state().position_gcrf_m));
            const bool before_ai = std::none_of(s.rendezvous_burns().begin(), s.rendezvous_burns().end(), [](const auto& b) {
                return b.kind == mission::PhasingBurnKind::ApproachInitiation;
            });
            if (before_ai) {
                const auto& t = s.body(iss).state();
                min_metric = std::min(min_metric, mission::approach_ellipsoid_metric(
                                                      {t.position_gcrf_m, t.velocity_gcrf_mps},
                                                      s.body(s.controlled_index()).state().position_gcrf_m));
            }
        }
        return min_metric;
    }

    orbit::RelativeState relative() const {
        const auto& t = session.body(*session.station_index()).state();
        const auto& c = session.body(session.controlled_index()).state();
        return orbit::lvlh_relative_state({t.position_gcrf_m, t.velocity_gcrf_mps}, {c.position_gcrf_m, c.velocity_gcrf_mps});
    }

    void report(const char* name) const {
        for (const auto& b : session.rendezvous_burns()) {
            std::printf("[validation] %s: %-22s +%.2f h %.2f m/s\n", name, mission::german_name(b.kind),
                        (b.node.epoch - start.epoch) / 3600.0, norm(b.node.delta_v_gcrf_mps));
        }
        const auto r = relative();
        std::printf("[validation] %s: %s, re-plans %d, LVLH (%.1f, %.1f, %.1f) m (%.3f, %.3f, %.3f) m/s, %.0f kg used\n",
                    name, session.rendezvous_status().c_str(), session.rendezvous_replans(), r.position_m.x,
                    r.position_m.y, r.position_m.z, r.velocity_mps.x, r.velocity_mps.y, r.velocity_mps.z,
                    2562.0 - session.body(session.controlled_index()).vehicle().rcs_propellant_kg());
    }
};
}  // namespace

TEST_CASE("Rendezvous plan refusals in the session", "[rendezvous][phasing][session]") {
    test::spice_session();
    Flight f(330.0, 0.0);
    CHECK_FALSE(f.session.plan_rendezvous({}));  // beyond 28 h
    CHECK(f.session.rendezvous_status().find("nicht innerhalb") != std::string::npos);
    CHECK(f.session.autopilot().state == sim::AutopilotState::Off);
    CHECK_FALSE(f.session.body(f.session.controlled_index()).vehicle().nose_cone_open());  // a refusal leaves it closed
    sim::FlightSession lonely{f.ephemeris, f.earth, rendezvous_test::config_dir};
    lonely.place_in_orbit(config::load_vehicle(rendezvous_test::config_dir, "crew_dragon"), f.start.dragon, f.start.epoch);
    CHECK_FALSE(lonely.plan_rendezvous({}));
    CHECK(lonely.rendezvous_status() == "Kein Rendezvous: keine Station");
    // a Starship is planned too since refuelling r3 (Raptor and cold gas on its main tanks): here the phase refuses it
    sim::FlightSession ship{f.ephemeris, f.earth, rendezvous_test::config_dir};
    ship.place_in_orbit(config::load_vehicle(rendezvous_test::config_dir, "starship_block3"), f.start.dragon, f.start.epoch);
    ship.add_station(f.start.iss.vehicle, f.start.iss.state_gcrf, f.start.iss.epoch, f.start.iss.impulses);
    CHECK_FALSE(ship.plan_rendezvous({}));
    CHECK(ship.rendezvous_status().find("nicht innerhalb") != std::string::npos);
}

TEST_CASE("Rendezvous flight from 30 deg to Waypoint 0", "[rendezvous][phasing][session][validation]") {
    test::spice_session();
    Flight f(30.0, 3.0 * 3600.0);
    REQUIRE(f.session.plan_rendezvous({}));
    CHECK(f.session.autopilot().state == sim::AutopilotState::Rendezvous);
    // the Dracos turn the Dragon without a push only with the cone open (closed: Waypoint 0 missed by 130-390 m)
    CHECK(f.session.body(f.session.controlled_index()).vehicle().nose_cone_open());
    const double metric = f.fly(12.0);
    f.report("30 deg");
    REQUIRE(f.session.rendezvous_step() == sim::RendezvousStep::Waypoint0);
    const auto r = f.relative();
    CHECK(norm(r.position_m - Vec3{0.0, 0.0, 400.0}) < 100.0);  // [E] spec
    CHECK(norm(r.velocity_mps) < 0.5);                           // [E] spec
    CHECK(metric > 1.0);
    CHECK(f.min_range_m > mission::keep_out_sphere_radius_m);  // never inside the keep-out sphere
}

TEST_CASE("Rendezvous flight with a denser truth atmosphere re-plans and arrives", "[rendezvous][phasing][session][validation]") {
    test::spice_session();
    Flight f(150.0, 13.0 * 3600.0);
    f.session.set_air_density_factor(1.5);  // truth only: the planner starts from US76 and the C_D guess
    REQUIRE(f.session.plan_rendezvous({}));
    const double metric = f.fly(28.0);
    f.report("150 deg, air x1.5");
    REQUIRE(f.session.rendezvous_step() == sim::RendezvousStep::Waypoint0);
    CHECK(f.session.rendezvous_replans() >= 1);  // ~13 h at 205 km with 1.5x air: tens of km of phase error
    const auto r = f.relative();
    CHECK(norm(r.position_m - Vec3{0.0, 0.0, 400.0}) < 100.0);
    CHECK(norm(r.velocity_mps) < 0.5);
    CHECK(metric > 1.0);
    CHECK(f.min_range_m > mission::keep_out_sphere_radius_m);
}

TEST_CASE("Rendezvous crew-day flight with a denser truth atmosphere checks the boost and arrives",
          "[rendezvous][phasing][session][validation]") {
    test::spice_session();
    Flight f(120.0, 16.0 * 3600.0);
    f.session.set_air_density_factor(1.5);  // the phase burn flies on the C_D guess; ~15 h on the phase orbit follow
    mission::PhasingParameters p;
    p.earliest_arrival_s = 20.0 * 3600.0;
    REQUIRE(f.session.plan_rendezvous(p));
    REQUIRE(f.session.rendezvous_plan()->schedule.phasing_revolutions > 0);
    const double metric = f.fly(28.0);
    f.report("120 deg crew day, air x1.5");
    REQUIRE(f.session.rendezvous_step() == sim::RendezvousStep::Waypoint0);
    CHECK(f.session.epoch() - f.start.epoch >= p.earliest_arrival_s - 600.0);  // the crew day stays (one re-plan slack)
    // the coelliptic burn absorbs the phase orbit's drag error (~-34 km): 603 kg against ~570 kg planned (i3 review)
    CHECK(2562.0 - f.session.body(f.session.controlled_index()).vehicle().rcs_propellant_kg() < 2562.0 - p.reserve_kg);
    const auto r = f.relative();
    CHECK(norm(r.position_m - Vec3{0.0, 0.0, 400.0}) < 100.0);
    CHECK(norm(r.velocity_mps) < 0.5);
    CHECK(metric > 1.0);
    CHECK(f.min_range_m > mission::keep_out_sphere_radius_m);
}
