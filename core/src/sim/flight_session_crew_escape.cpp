// FlightSession: crew escape of a crewed ship return (docs/superpowers/specs/2026-10-22-crew-escape-design.md) - the
// give-up, the seat firing, the seat and parachutist bodies, their judging and the given-up ship's impact report.

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/attitude_slew.hpp"
#include "starsim/gnc/catch_feasibility.hpp"
#include "starsim/gnc/sea_target.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/landing_outcome.hpp"

namespace starsim::sim {

namespace {

void lose(CrewMemberState& m, std::string reason) {
    if (m.outcome != CrewOutcome::Flying) return;
    m.outcome = CrewOutcome::Lost;
    m.reason = std::move(reason);
}

}  // namespace

void FlightSession::set_crew_escape(std::optional<config::CrewEscapeConfig> config) {
    if (!config) {
        crew_.reset();
        return;
    }
    auto c = std::make_unique<CrewRuntime>();
    c->seat_vehicle = config::load_vehicle(config_dir_, config->seat_vehicle_id);
    c->parachutist_vehicle = config::load_vehicle(config_dir_, config->parachutist_vehicle_id);
    const auto n = static_cast<size_t>(config->crew_count);
    c->state.members.resize(n);
    c->sequencers.resize(n);
    c->rail_gcrf.resize(n);
    c->loads.resize(n);
    c->out_of_hull.assign(n, false);
    c->sink_mps.assign(n, 0.0);
    c->config = std::move(*config);
    crew_ = std::move(c);
}

double FlightSession::crew_height_m(const vehicle::FlightBody& body) const {
    const auto t = earth_.fixed_to_gcrf(epoch_);
    return frames::fixed_to_geodetic(transpose(t.rotation) * body.state().position_gcrf_m).height_m - body.ground_height_m();
}

void FlightSession::crew_arm(const std::string& reason, bool early) {
    auto& s = crew_->state;
    s.armed = true;
    s.early = early;
    s.give_up_reason = reason;
    s.give_up_time_s = mission_time_s();
}

void FlightSession::crew_fire_seat(size_t member) {
    auto& c = *crew_;
    const auto& ship = *bodies_[controlled_].body;
    const auto& s = ship.state();
    const auto mp = ship.vehicle().mass_properties();
    const math::Vec3 r_body = c.config.seats[member].position_stage_m - mp.cg_point_m();
    const math::Vec3 w_gcrf = s.attitude.rotate(s.angular_velocity_body_radps);
    const math::Vec3 rail = s.attitude.rotate(c.config.rail_direction_body);
    vehicle::RigidBodyState seat;
    seat.position_gcrf_m = s.position_gcrf_m + s.attitude.rotate(r_body);
    seat.velocity_gcrf_mps = s.velocity_gcrf_mps + cross(w_gcrf, s.attitude.rotate(r_body));
    seat.attitude = gnc::attitude_from_axes(rail, s.attitude.rotate(math::Vec3{0.0, 1.0, 0.0}));  // head along the rail
    seat.angular_velocity_body_radps = seat.attitude.inverse_rotate(w_gcrf);
    auto body = std::make_unique<vehicle::FlightBody>(std::make_unique<vehicle::Vehicle>(c.seat_vehicle, config_dir_), seat);
    body->set_ground_height(ship.ground_height_m());
    const auto telemetry = ship.telemetry(env_, epoch_);
    Flying flying{std::move(body), {}, nullptr, {}};
    flying.crew_member = true;
    bodies_.push_back(std::move(flying));
    auto& m = c.state.members[member];
    m.body_index = bodies_.size() - 1;
    m.fire_time_s = mission_time_s();
    m.mode = gnc::seat_mode(c.config, crew_height_m(ship), telemetry.dynamic_pressure_pa);
    c.sequencers[member].emplace(c.config, m.mode);
    c.rail_gcrf[member] = rail;
}

void FlightSession::crew_judge(size_t member) {
    auto& c = *crew_;
    auto& m = c.state.members[member];
    const auto& body = *bodies_[*m.body_index].body;
    const auto t = earth_.fixed_to_gcrf(epoch_);
    const math::Vec3 r_itrf = transpose(t.rotation) * body.state().position_gcrf_m;
    const math::Vec3 up = t.rotation * math::normalized(r_itrf);
    const math::Vec3 v_fixed =
        body.state().velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), body.state().position_gcrf_m);
    if (body.resting() || body.impacted()) {  // touchdown (first tick on the surface)
        if (!m.touchdown_enu_m) {
            m.touchdown_enu_m = site_enu_m(launch_site_, r_itrf);
            m.touchdown_over_water = launch_site_.sea && over_water(*launch_site_.sea, m.touchdown_enu_m->x, m.touchdown_enu_m->y);
            m.touchdown_sink_mps = c.sink_mps[member];
            char text[96];
            std::snprintf(text, sizeof text, "zu hart aufgesetzt (%.1f m/s)", m.touchdown_sink_mps);
            if (body.impacted() || m.touchdown_sink_mps > c.config.max_touchdown_sink_mps) lose(m, text);
            if (m.outcome == CrewOutcome::Flying) m.outcome = CrewOutcome::Rescued;
        }
        return;
    }
    c.sink_mps[member] = -dot(v_fixed, up);
    m.load_g = norm(body.specific_force_body_mps2()) / constants::standard_gravity_mps2;
    m.peak_load_g = std::max(m.peak_load_g, m.load_g);
    if (c.loads[member].exceeded(m.load_g, tick_s, c.config.max_load_g, c.config.max_load_duration_s)) {
        char text[96];
        std::snprintf(text, sizeof text, "Belastung über %.0f g", c.config.max_load_g);
        lose(m, text);
    }
    if (body.vehicle().parachute_torn()) lose(m, "Fallschirm gerissen");

    // Clearances: the ship's hull (once the seat left the hatch), its plume while engines burn, the tower.
    const auto& ship = *bodies_[controlled_].body;
    const auto& ss = ship.state();
    const auto mp = ship.vehicle().mass_properties();
    const math::Vec3 bottom = ss.position_gcrf_m + ss.attitude.rotate(math::Vec3{0.0, 0.0, 0.0} - mp.cg_point_m());
    const math::Vec3 top = ss.position_gcrf_m + ss.attitude.rotate(math::Vec3{ship.vehicle().length_m(), 0.0, 0.0} - mp.cg_point_m());
    const math::Vec3& p = body.state().position_gcrf_m;
    double clearance = 1e9;
    const char* what = "";
    const double hull = hull_clearance_m(bottom, top, 0.5 * ship.vehicle().diameter_m(), p);
    // counted once the seat is the minimum clearance out of the hatch (before that it rides out on its rails)
    if (!c.out_of_hull[member] && hull >= c.config.min_clearance_m) c.out_of_hull[member] = true;
    if (c.out_of_hull[member] && hull < clearance) clearance = hull, what = "Schiff";
    if (ship.vehicle().any_engine_producing_thrust()) {
        const double plume = plume_clearance_m(bottom, ss.attitude.rotate(math::Vec3{-1.0, 0.0, 0.0}),
                                               c.config.plume_half_angle_rad, c.config.plume_length_m, p);
        if (plume < clearance) clearance = plume, what = "Abgasstrahl";
    }
    if (tower_) {
        const auto& g = tower_->geometry();
        const double tower = contact::distance_to_box(g.tower_box(), g.to_site(r_itrf));
        if (tower < clearance) clearance = tower, what = "Turm";
    }
    m.min_clearance_m = std::min(m.min_clearance_m, clearance);
    if (clearance < c.config.min_clearance_m) lose(m, std::string("Zusammenstoß: ") + what);
}

void FlightSession::run_crew_escape() {
    auto& c = *crew_;
    auto& st = c.state;
    if (bodies_.empty() || controlled_ >= bodies_.size()) return;
    auto& ship = bodies_[controlled_];
    const bool ship_down = (ship.body->impacted() || ship.body->resting()) && !ship.body->caught();
    const bool returning = (ship.entry_guidance && !ship.entry_guidance->capsule()) || ship.ship_landing;
    if (!st.armed && ship.body->caught()) {  // caught with the crew aboard
        for (auto& m : st.members) {
            if (m.outcome == CrewOutcome::Flying) m.outcome = CrewOutcome::Rescued;
        }
        return;
    }

    // The give-up: the catch is not possible (entry, up to the gate) or a catch abort after the flip.
    if (!st.armed && !ship_down && returning) {
        if (ship.entry_guidance && !ship.ship_landing) {
            const auto f = gnc::catch_feasibility(ship.body->vehicle(), gnc::ship_landing_parameters().flip_groups,
                                                  c.config.min_catch_propellant_kg);
            if (!f.feasible) {
                crew_arm(f.reason, crew_height_m(*ship.body) > c.config.min_firing_height_m);
                const auto site = return_site(ship.body->vehicle());
                if (site.sea) ship.entry_guidance->give_up(gnc::sea_give_up_target(site, c.config.sea_target_margin_m));
            }
        } else if (ship.ship_landing) {
            const auto* scvx = ship.ship_landing->scvx();
            if (ship.return_command.abort || ship.ship_landing->upright_descent_active() || (scvx && scvx->aborting())) {
                crew_arm("Fang abgebrochen", false);
            }
        }
    }
    // Crew still aboard a ship that broke up or struck the ground: lost.
    if (!c.fire_epoch && ship_down && (st.armed || returning)) {
        for (auto& m : st.members) lose(m, "außerhalb der Schleudersitz-Grenzen");
    }
    // Firing: inside the envelope; an early give-up over water or at the minimum firing height.
    if (st.armed && !c.fire_epoch && !ship_down) {
        const double h = crew_height_m(*ship.body);
        const auto telemetry = ship.body->telemetry(env_, epoch_);
        const auto t = earth_.fixed_to_gcrf(epoch_);
        const auto enu = site_enu_m(launch_site_, transpose(t.rotation) * ship.body->state().position_gcrf_m);
        const bool water = launch_site_.sea && over_water(*launch_site_.sea, enu.x, enu.y);
        if (gnc::inside_seat_envelope(c.config, h, telemetry.dynamic_pressure_pa) &&
            (!st.early || water || h <= c.config.min_firing_height_m)) {
            c.fire_epoch = epoch_;
            st.fired = true;
        }
    }
    if (c.fire_epoch) {
        for (size_t k = 0; k < st.members.size(); ++k) {
            auto& m = st.members[k];
            if (m.body_index || m.outcome != CrewOutcome::Flying) continue;
            if (epoch_ - *c.fire_epoch < k * c.config.stagger_s) continue;
            if (ship_down) {
                lose(m, "außerhalb der Schleudersitz-Grenzen");
            } else {
                crew_fire_seat(k);
            }
        }
    }
    // Seats and parachutists: judge the last tick, then this tick's sequence actions.
    for (size_t k = 0; k < st.members.size(); ++k) {
        auto& m = st.members[k];
        if (!m.body_index || !c.sequencers[k]) continue;
        if (m.outcome == CrewOutcome::Flying || !m.touchdown_enu_m) crew_judge(k);
        auto& flying = bodies_[*m.body_index];
        auto& body = *flying.body;
        const bool surface = body.resting() || body.impacted();
        const auto telemetry = body.telemetry(env_, epoch_);
        const auto a = c.sequencers[k]->update(
            {epoch_ - (*c.fire_epoch + k * c.config.stagger_s), crew_height_m(body), telemetry.dynamic_pressure_pa, surface});
        m.phase = c.sequencers[k]->phase();
        if (surface) continue;
        if (a.rail_thrust_n > 0.0) body.apply_external_wrench(c.rail_gcrf[k] * a.rail_thrust_n, {});
        if (a.deploy_drogue) body.vehicle().deploy_parachutes("drogue");
        if (a.separate) {  // the occupant goes on under the main; the empty seat is not flown
            auto person = std::make_unique<vehicle::FlightBody>(
                std::make_unique<vehicle::Vehicle>(c.parachutist_vehicle, config_dir_), body.state());
            person->set_ground_height(body.ground_height_m());
            if (legs_) legs_->forget(flying.body.get());
            flying.body = std::move(person);
        }
        if (a.deploy_main) flying.body->vehicle().deploy_parachutes("main");
    }
    // The given-up ship's impact (its Flying again: firing a seat grew bodies_).
    if (st.armed && !st.ship_impact_enu_m) {
        const auto t = earth_.fixed_to_gcrf(epoch_);
        const auto& s = bodies_[controlled_].body->state();
        if (ship_down) {
            const auto enu = site_enu_m(launch_site_, transpose(t.rotation) * s.position_gcrf_m);
            st.ship_impact_enu_m = enu;
            st.ship_impact_speed_mps = c.ship_speed_mps;
            if (tower_) {
                const auto& g = tower_->geometry();
                st.ship_impact_tower_distance_m = contact::distance_to_box(g.tower_box(), enu);
            } else {
                st.ship_impact_tower_distance_m = std::hypot(enu.x, enu.y);
            }
            if (launch_site_.sea) {
                const auto& sea = *launch_site_.sea;
                st.ship_impact_shore_distance_m =
                    enu.x * std::sin(sea.direction_rad) + enu.y * std::cos(sea.direction_rad) - sea.shoreline_distance_m;
                st.ship_impact_over_water = over_water(sea, enu.x, enu.y);
            }
        } else {
            c.ship_speed_mps = norm(s.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), s.position_gcrf_m));
        }
    }
}

}  // namespace starsim::sim
