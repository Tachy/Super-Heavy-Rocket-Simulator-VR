#include "flight_snapshot.hpp"

#include <cmath>
#include <limits>

#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/mat3.hpp"
#include "starsim/mission/lunar_outbound.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/landing_outcome.hpp"
#include "starsim/sim/leg_landing_sim.hpp"
#include "starsim/vehicle/vehicle.hpp"

namespace starsim::bindings {

namespace {

godot::Vector3 to_godot(const math::Vec3& v) { return {v.x, v.y, v.z}; }

godot::Array body_stages(const sim::FlightSession& session, size_t index) {
    godot::Array stages;
    for (const auto& stage : session.body(index).vehicle().stages()) {
        godot::Dictionary d;
        d["name"] = godot::String(stage.config.name.c_str());
        d["id"] = godot::String(stage.config.id.c_str());
        d["base_station_m"] = stage.base_station_m;
        d["length_m"] = stage.config.length_m;
        d["diameter_m"] = stage.config.diameter_m;
        d["nose_length_m"] = stage.config.aero.nose_length_m;
        d["settled"] = stage.settled;  // refuelling f3: 0..1, the liquid over the main-tank outlets
        // main tanks: current load and full load
        double oxidizer_capacity_kg = 0.0, fuel_capacity_kg = 0.0;
        for (const auto& t : stage.tanks) {
            if (t.config.role != config::TankRole::Main) continue;
            if (t.config.contents == config::TankContents::Oxidizer) oxidizer_capacity_kg += t.config.capacity_kg;
            if (t.config.contents == config::TankContents::Fuel) fuel_capacity_kg += t.config.capacity_kg;
        }
        d["oxidizer_kg"] = stage.propellant_kg(config::TankContents::Oxidizer, config::TankRole::Main);
        d["fuel_kg"] = stage.propellant_kg(config::TankContents::Fuel, config::TankRole::Main);
        d["oxidizer_capacity_kg"] = oxidizer_capacity_kg;
        d["fuel_capacity_kg"] = fuel_capacity_kg;
        stages.push_back(d);
    }
    return stages;
}

godot::Array body_engines(const sim::FlightSession& session, size_t index) {
    godot::Array engines;
    int64_t stage_index = -1;
    for (const auto& stage : session.body(index).vehicle().stages()) {
        ++stage_index;
        for (const auto& e : stage.engines) {
            godot::Dictionary d;
            d["stage"] = stage_index;  // index into body_stages
            d["position_m"] = to_godot(e.gimbal_point_m);
            d["direction"] = to_godot(e.thrust_direction());
            d["firing"] = e.producing_thrust();
            d["throttle"] = e.throttle;
            d["spool"] = e.spool;
            d["vacuum"] = e.performance->config().reference_pressure_pa == 0.0;
            d["auxiliary"] = e.auxiliary();  // launch escape or landing thrusters: not main propulsion
            engines.push_back(d);
        }
    }
    return engines;
}

// Osculating two-body orbit about the reference body (FlightSim::telemetry): inclination against its equator (the
// body-fixed axes now), altitudes above its equatorial radius / the Moon sphere.
void add_orbit(const sim::FlightSession& session, size_t index, ephemeris::Body reference, godot::Dictionary& d) {
    const auto& state = session.body(index).state();
    const bool moon = reference == ephemeris::Body::Moon && session.moon_orientation();
    const math::StateVector inertial{state.position_gcrf_m, state.velocity_gcrf_mps};
    const auto relative = moon ? mission::moon_relative(session.ephemeris(), inertial, session.epoch()) : inertial;
    const auto& orientation = moon ? *session.moon_orientation() : session.earth_orientation();
    const math::Mat3 to_fixed_axes = math::transpose(orientation.fixed_to_gcrf(session.epoch()).rotation);
    const auto e = orbit::elements_from_state({to_fixed_axes * relative.position_m, to_fixed_axes * relative.velocity_mps},
                                              session.ephemeris().gm_m3ps2(moon ? ephemeris::Body::Moon
                                                                                : ephemeris::Body::Earth));
    const double radius_m = moon ? constants::moon_mean_radius_m : frames::wgs84.semi_major_axis_m;
    d["periapsis_altitude_km"] = (e.semi_major_axis_m * (1.0 - e.eccentricity) - radius_m) / 1000.0;
    d["apoapsis_altitude_km"] = e.eccentricity < 1.0
                                    ? (e.semi_major_axis_m * (1.0 + e.eccentricity) - radius_m) / 1000.0
                                    : std::numeric_limits<double>::infinity();
    d["inclination_deg"] = e.inclination_rad * constants::rad_to_deg;
}

godot::Dictionary telemetry(const sim::FlightSession& session, size_t index) {
    godot::Dictionary d;
    const auto t = session.telemetry(index);
    d["altitude_m"] = t.altitude_m;
    d["altitude_terrain_m"] = t.altitude_terrain_m;
    d["reference_body"] = t.reference_body == ephemeris::Body::Moon ? "moon" : "earth";
    add_orbit(session, index, t.reference_body, d);
    d["latitude_deg"] = t.latitude_rad * constants::rad_to_deg;
    d["longitude_deg"] = t.longitude_rad * constants::rad_to_deg;
    d["speed_inertial_mps"] = t.speed_inertial_mps;
    d["speed_relative_mps"] = t.speed_relative_mps;
    d["distance_earth_m"] = t.distance_earth_m;
    d["speed_earth_mps"] = t.speed_earth_mps;
    if (!std::isnan(t.distance_moon_m)) {  // only with a Moon in the session
        d["distance_moon_m"] = t.distance_moon_m;
        d["speed_moon_mps"] = t.speed_moon_mps;
    }
    d["vertical_speed_mps"] = t.vertical_speed_mps;
    d["flight_path_angle_deg"] = t.flight_path_angle_rad * constants::rad_to_deg;
    d["mach"] = t.mach;
    d["knudsen"] = t.knudsen;
    d["dynamic_pressure_pa"] = t.dynamic_pressure_pa;
    d["angle_of_attack_deg"] = t.angle_of_attack_rad * constants::rad_to_deg;
    d["mass_kg"] = t.mass_kg;
    d["propellant_kg"] = t.propellant_kg;
    d["propellant_capacity_kg"] = session.body(index).vehicle().propellant_capacity_kg();  // main + header tanks
    d["thrust_n"] = t.thrust_n;
    d["sensed_acceleration_g"] = t.sensed_acceleration_g;
    d["engines_running"] = t.engines_running;
    const auto& body = session.body(index);
    d["clamped"] = body.clamped();
    d["impacted"] = body.impacted();
    d["heat_rate_w_m2"] = body.heat_rate_w_m2();
    d["heat_load_j_m2"] = body.heat_load_j_m2();
    d["heat_warning"] = body.heat_warning();
    d["breakup_reason"] = godot::String::utf8(body.breakup_reason().c_str());
    d["skipped_out"] = body.skipped_out();
    if (const auto* e = session.entry_status(index)) {  // entry guidance setpoint (flight director)
        d["entry_phase"] = godot::String::utf8(gnc::to_string(e->phase));
        d["entry_alpha_deg"] = e->alpha_rad * constants::rad_to_deg;
        d["entry_bank_deg"] = e->bank_rad * constants::rad_to_deg;
        d["entry_downrange_km"] = e->downrange_error_m / 1000.0;
        d["entry_crossrange_km"] = e->crossrange_m / 1000.0;
        d["entry_constrained"] = e->constrained;
    }
    return d;
}

godot::Array body_flaps(const sim::FlightSession& session, size_t index) {
    godot::Array flaps;
    for (const auto& stage : session.body(index).vehicle().stages()) {
        for (const auto& f : stage.flaps) {
            godot::Dictionary d;
            d["hinge_azimuth_deg"] = f.geometry.hinge_azimuth_rad * constants::rad_to_deg;
            d["station_fwd_m"] = f.geometry.station_fwd_m;
            d["station_aft_m"] = f.geometry.station_aft_m;
            d["span_m"] = f.geometry.span_m;
            d["deflection_deg"] = f.deflection_rad * constants::rad_to_deg;  // 0 folded leeward, 90 broadside
            flaps.push_back(d);
        }
    }
    return flaps;
}

godot::Dictionary body_legs(const sim::FlightSession& session, size_t index) {
    godot::Dictionary d;
    const auto& vehicle = session.body(index).vehicle();
    d["present"] = vehicle.has_landing_legs();
    if (!vehicle.has_landing_legs()) return d;
    const auto& stage = vehicle.stages().front();
    const auto& legs = stage.config.landing_legs;
    d["deployment"] = vehicle.leg_deployment();
    d["touchdown"] = godot::String::utf8(sim::to_string(session.leg_touchdown(index)));
    d["base_station_m"] = stage.base_station_m;
    d["hull_radius_m"] = 0.5 * stage.config.diameter_m;
    d["foot_station_m"] = legs.foot_station_m;
    d["footprint_radius_m"] = legs.footprint_radius_m;
    godot::Array azimuths;
    for (const double a : legs.azimuths_rad) azimuths.push_back(a * constants::rad_to_deg);
    d["azimuths_deg"] = azimuths;
    return d;
}

godot::Array body_grid_fins(const sim::FlightSession& session, size_t index) {
    godot::Array fins;
    for (const auto& stage : session.body(index).vehicle().stages()) {
        for (const auto& f : stage.grid_fins) {
            godot::Dictionary d;
            d["station_m"] = f.geometry.station_m;
            d["azimuth_deg"] = f.geometry.azimuth_rad * constants::rad_to_deg;
            d["radius_m"] = f.geometry.radius_m;
            d["area_m2"] = f.geometry.area_m2;
            d["deflection_deg"] = f.deflection_rad * constants::rad_to_deg;
            fins.push_back(d);
        }
    }
    return fins;
}

godot::Array body_rcs(const sim::FlightSession& session, size_t index) {
    godot::Array thrusters;
    const auto& vehicle = session.body(index).vehicle();
    const auto& duty = vehicle.rcs_duty();
    const auto& stages = vehicle.stages();
    size_t k = 0;
    for (size_t s = 0; s < stages.size(); ++s) {
        for (const auto& t : stages[s].rcs) {
            godot::Dictionary d;
            d["id"] = godot::String::utf8(t.id.c_str());
            d["stage"] = static_cast<int64_t>(s);
            d["position_m"] = to_godot(t.position_m);
            d["direction"] = to_godot(t.direction);
            d["thrust_n"] = t.thrust_n;
            d["firing"] = t.output > 0.0;
            d["duty"] = k < duty.size() ? duty[k] : 0.0;  // empty after stop_rcs: all off
            d["starved"] = t.starved;
            d["blocked"] = t.blocked;
            thrusters.push_back(d);
            ++k;
        }
    }
    return thrusters;
}

godot::Array body_parachutes(const sim::FlightSession& session, size_t index) {
    godot::Array sets;
    const auto& body = session.body(index);
    for (const auto& stage : body.vehicle().stages()) {
        for (const auto& p : stage.parachutes) {
            const double full = p.config.count * p.config.drag_area_m2;
            const math::Vec3 v = body.point_air_velocity_body_mps(p.attach_station_m);
            const double speed = norm(v);
            godot::Dictionary d;
            d["id"] = godot::String::utf8(p.config.id.c_str());
            d["role"] = godot::String::utf8(p.config.role.c_str());
            d["state"] = godot::String::utf8(vehicle::to_string(p.state));
            d["count"] = p.config.count;
            d["attach_station_m"] = p.attach_station_m;
            d["drag_area_m2"] = p.drag_area_m2;
            d["full_drag_area_m2"] = full;
            d["inflation"] = full > 0.0 ? p.drag_area_m2 / full : 0.0;
            d["direction"] = speed > 0.1 ? to_godot(v * (-1.0 / speed)) : godot::Vector3(1.0, 0.0, 0.0);
            d["peak_load_n"] = p.peak_load_n;
            d["max_load_n"] = p.config.max_load_n;
            sets.push_back(d);
        }
    }
    return sets;
}

godot::Dictionary capsule_state(const sim::FlightSession& session, size_t index) {
    godot::Dictionary d;
    const auto& vehicle = session.body(index).vehicle();
    const auto& top = vehicle.stages().back().config;
    const bool present = top.aero.entry_profile == "capsule" && !vehicle.payload_stacked();  // not on a launcher
    d["present"] = present;
    if (!present) return d;
    d["nose_cone"] = vehicle.has_nose_cone();
    d["nose_cone_open"] = vehicle.nose_cone_open();
    d["trunk_attached"] = vehicle.stages().size() > 1;
    d["return_step"] = godot::String::utf8(sim::to_string(session.capsule_return_step()));
    const auto* sequencer = session.parachute_sequencer(index);
    d["parachute_phase"] = sequencer ? godot::String::utf8(gnc::to_string(sequencer->phase())) : godot::String();
    d["parachute_torn"] = vehicle.parachute_torn();
    if (session.splashdown_zone()) d["splashdown_miss_m"] = session.splashdown_miss_m(index);
    d["outcome"] = godot::String::utf8(sim::to_string(session.landing_outcome(index)));
    return d;
}

godot::Dictionary return_state(const sim::FlightSession& session, size_t index, const godot::Dictionary& legs) {
    godot::Dictionary d;
    d["present"] = false;
    d["outcome"] = godot::String::utf8(sim::to_string(session.landing_outcome(index)));
    d["legs"] = legs;
    const auto* status = session.return_status(index);
    if (const auto* ship = session.ship_landing(index); !status && ship) {  // the ship from the gate (flip, catch, abort)
        const auto* scvx = ship->scvx();
        d["present"] = true;
        d["phase"] = godot::String::utf8(gnc::to_string(ship->phase()));
        d["guidance"] = godot::String::utf8(ship->upright_descent_active() ? "aufrecht" : "SCvx");
        d["abort"] = ship->upright_descent_active() || (scvx && scvx->aborting());
        d["upright_descent"] = ship->upright_descent_active();
        d["result"] = godot::String::utf8(sim::to_string(session.ship_return_result(index)));
        d["plans"] = scvx ? scvx->statistics().plans : 0;
        d["max_plan_ms"] = scvx ? scvx->statistics().max_solve_ms : 0.0;
        return d;
    }
    if (const auto* crew = session.crew_escape_state(); !status && crew && crew->armed && index == session.controlled_index()) {
        d["present"] = true;  // a crewed ship given up in the entry (no flip): flown to its impact
        d["phase"] = godot::String::utf8("aufgegeben");
        d["abort"] = true;
        d["result"] = godot::String::utf8(sim::to_string(session.ship_return_result(index)));
        return d;
    }
    if (!status) return d;
    d["present"] = true;
    d["phase"] = godot::String::utf8(gnc::to_string(status->phase));
    d["guidance"] = godot::String::utf8(status->scvx ? "SCvx" : "explizit");
    d["abort"] = status->abort;
    const auto* guidance = session.return_guidance(index);
    const auto* scvx = guidance ? guidance->scvx() : nullptr;
    d["plans"] = scvx ? scvx->statistics().plans : 0;
    d["max_plan_ms"] = scvx ? scvx->statistics().max_solve_ms : 0.0;
    return d;
}

godot::Dictionary landing_zone(const sim::FlightSession& session) {
    godot::Dictionary d;
    d["present"] = false;
    if (!session.landing_zone()) return d;
    const auto& zone = *session.landing_zone();
    const double lat = zone.latitude_rad, lon = zone.longitude_rad;
    d["present"] = true;
    d["origin_itrf_m"] = to_godot(frames::geodetic_to_fixed({lat, lon, zone.height_m}));
    d["east"] = godot::Vector3(-std::sin(lon), std::cos(lon), 0.0);
    d["north"] = godot::Vector3(-std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat));
    d["up"] = godot::Vector3(std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat));
    d["latitude_deg"] = lat * constants::rad_to_deg;
    d["longitude_deg"] = lon * constants::rad_to_deg;
    d["pad_radius_m"] = zone.pad_radius_m;
    d["has_deck"] = zone.deck.has_value();
    if (zone.deck) {
        d["deck_length_m"] = 2.0 * zone.deck->half_length_m;
        d["deck_width_m"] = 2.0 * zone.deck->half_width_m;
        d["freeboard_m"] = zone.deck->freeboard_m;
        d["heading_deg"] = zone.deck->heading_rad * constants::rad_to_deg;
        d["sea_height_m"] = zone.height_m - zone.deck->freeboard_m;
    }
    return d;
}

godot::Dictionary tower_state(const sim::FlightSession& session) {
    godot::Dictionary d;
    const auto* tower = session.tower();
    d["present"] = tower != nullptr;
    if (!tower) return d;
    d["opening_rad"] = tower->arms().opening_rad();
    d["carriage_offset_m"] = tower->arms().carriage_offset_m();
    d["phase"] = godot::String::utf8(sim::to_string(tower->phase()));
    return d;
}

godot::Dictionary splashdown_zone(const sim::FlightSession& session) {
    godot::Dictionary d;
    d["present"] = false;
    if (!session.splashdown_zone()) return d;
    const auto& z = *session.splashdown_zone();
    const double lat = z.latitude_rad, lon = z.longitude_rad;
    d["present"] = true;
    d["id"] = godot::String::utf8(z.id.c_str());
    d["name"] = godot::String::utf8(z.name.c_str());
    d["latitude_deg"] = lat * constants::rad_to_deg;
    d["longitude_deg"] = lon * constants::rad_to_deg;
    d["radius_m"] = z.radius_m;
    d["origin_itrf_m"] = to_godot(frames::geodetic_to_fixed({lat, lon, 0.0}));  // sea level (WGS84 height 0)
    d["east"] = godot::Vector3(-std::sin(lon), std::cos(lon), 0.0);
    d["north"] = godot::Vector3(-std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat));
    d["up"] = godot::Vector3(std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat));
    return d;
}

godot::Dictionary autopilot_status(const sim::FlightSession& session) {
    godot::Dictionary d;
    const auto& ap = session.autopilot();
    d["state"] = godot::String::utf8(sim::to_string(ap.state));
    d["engaged"] = ap.state != sim::AutopilotState::Off && ap.state != sim::AutopilotState::Orbit;
    d["phase"] = godot::String::utf8(gnc::to_string(ap.guidance.phase));
    d["time_to_go_s"] = ap.guidance.time_to_go_s;
    const bool burning = ap.state == sim::AutopilotState::Burn;
    d["direction"] = to_godot(burning ? ap.burn.direction_gcrf : ap.guidance.thrust_direction_gcrf);
    d["has_direction"] = ap.state == sim::AutopilotState::Booster || ap.state == sim::AutopilotState::Staging ||
                         ap.state == sim::AutopilotState::UpperStage || burning;
    d["settling"] = session.settling();  // refuelling f3: the cold-gas push (autopilot or manual) is on
    d["burn_abort_reason"] = godot::String::utf8(session.burn_abort_reason().c_str());
    if (burning) {
        godot::Dictionary b;
        b["time_to_ignition_s"] = ap.burn.time_to_ignition_s;
        b["remaining_dv_mps"] = ap.burn.remaining_dv_mps;
        b["burn_duration_s"] = ap.burn.burn_duration_s;
        b["total_dv_mps"] = session.burn() ? norm(session.burn()->node().delta_v_gcrf_mps) : 0.0;
        d["burn"] = b;
    }
    if (const auto* plan = session.ship_return_plan()) {
        godot::Dictionary r;
        r["feasible"] = plan->feasible;
        r["reason"] = godot::String::utf8(plan->reason.c_str());
        r["delta_v_mps"] = plan->delta_v_mps;
        r["propellant_t"] = plan->propellant_kg / 1000.0;
        r["burn_in_s"] = plan->node.epoch - session.epoch();
        r["entry_in_s"] = plan->entry_interface_epoch - session.epoch();
        r["crossrange_km"] = plan->crossrange_m / 1000.0;
        d["return"] = r;
    }
    if (session.capsule_return_step() != sim::CapsuleReturnStep::None) {
        d["capsule_step"] = godot::String::utf8(sim::to_string(session.capsule_return_step()));
    }
    return d;
}

FlightSnapshot::Body build_body(const sim::FlightSession& session, size_t index) {
    FlightSnapshot::Body b;
    const auto& body = session.body(index);
    const auto& q = body.state().attitude;
    b.position_gcrf_m = to_godot(body.state().position_gcrf_m);
    b.velocity_gcrf_mps = to_godot(body.state().velocity_gcrf_mps);
    b.attitude_gcrf = godot::Quaternion(q.x, q.y, q.z, q.w);
    b.cg_station_m = body.vehicle().mass_properties().cg_station_m;
    b.serial = session.body_serial(index);
    b.display_name = godot::String::utf8(body.vehicle().shown_name().c_str());
    b.stages = body_stages(session, index);
    b.engines = body_engines(session, index);
    b.flaps = body_flaps(session, index);
    b.grid_fins = body_grid_fins(session, index);
    b.rcs = body_rcs(session, index);
    b.parachutes = body_parachutes(session, index);
    b.legs = body_legs(session, index);
    b.capsule_state = capsule_state(session, index);
    b.telemetry = telemetry(session, index);
    b.return_state = return_state(session, index, b.legs);
    return b;
}

// The requested prediction: recomputed when the request changed or prediction_refresh_s has passed, otherwise the
// previous snapshot's points.
void fill_prediction(const sim::FlightSession& session, SnapshotContext& context, const FlightSnapshot* previous,
                     FlightSnapshot& s) {
    const auto& request = context.prediction_request;
    s.prediction_request = request;
    if (request.index < 0 || static_cast<size_t>(request.index) >= session.body_count()) return;
    const auto now = std::chrono::steady_clock::now();
    const bool same = previous != nullptr && previous->prediction_request == request && context.prediction_wall;
    if (same && std::chrono::duration<double>(now - *context.prediction_wall).count() < prediction_refresh_s) {
        s.prediction = previous->prediction;
        return;
    }
    for (const auto& p : session.predict_trajectory(static_cast<size_t>(request.index), request.duration_s,
                                                    request.step_s)) {
        s.prediction.push_back(to_godot(p));
    }
    context.prediction_wall = now;
}

godot::Dictionary absent() {
    godot::Dictionary d;
    d["present"] = false;
    return d;
}

}  // namespace

std::shared_ptr<const FlightSnapshot> empty_snapshot() {
    auto s = std::make_shared<FlightSnapshot>();
    s->landing_zone = absent();
    s->tower_state = absent();
    s->splashdown_zone = absent();
    s->lunar_state = absent();
    s->lunar_site = absent();
    s->iss_state = s->rendezvous_plan = s->docking_state = s->station_geometry = s->iss_mission = absent();
    s->transfer_state = s->depot_state = s->campaign_flight_state = s->crew_escape_state = absent();
    s->mission_timeline["t0_tdb_s"] = godot::Variant();  // the shape of build_mission_timeline: no T-0, no events
    s->mission_timeline["clock_s"] = godot::Variant();
    s->mission_timeline["events"] = godot::Array();
    return s;
}

std::shared_ptr<const FlightSnapshot> build_snapshot(const sim::FlightSession& session, time::Epoch epoch,
                                                     SnapshotContext& context, const FlightSnapshot* previous) {
    auto s = std::make_shared<FlightSnapshot>();
    s->site_id = context.site_id;
    s->mission_display_name = context.mission_display_name;
    s->mission_start_tdb_s = context.mission_start_tdb_s;
    s->epoch_tdb_s = epoch.tdb_seconds_past_j2000();
    s->landing_zone = landing_zone(session);
    s->tower_state = tower_state(session);
    s->splashdown_zone = splashdown_zone(session);
    s->active = session.active();
    s->lunar_state = build_lunar_state(session);
    s->lunar_site = build_lunar_site(session);
    s->iss_state = build_iss_state(session);
    s->rendezvous_plan = build_rendezvous_plan(session);
    s->docking_state = build_docking_state(session);
    s->station_geometry = build_station_geometry(session);
    s->iss_mission = build_iss_mission(session, context);
    s->transfer_state = build_transfer_state(session);
    s->depot_state = build_depot_state(session);
    s->campaign_flight_state = build_campaign_flight_state(session);
    s->crew_escape_state = build_crew_escape_state(session);
    s->mission_timeline = build_mission_timeline(session, epoch);
    s->moon_terrain = session.moon_terrain_shared();
    // a mission start plans its launch before the session holds the vehicle: the flag also without a flight
    s->autopilot_status["planning"] = context.planning;
    if (!s->active) return s;
    s->mission_time_s = session.mission_time_s();
    s->controlled_index = static_cast<int>(session.controlled_index());
    for (size_t i = 0; i < session.body_count(); ++i) s->bodies.push_back(build_body(session, i));
    s->autopilot_status = autopilot_status(session);
    s->autopilot_status["planning"] = context.planning;
    if (const auto* guidance = session.ascent_guidance()) {
        const math::Vec3 n = guidance->target_plane_normal_gcrf();
        s->has_target_orbit = true;
        s->target_plane_normal_gcrf = to_godot(n);
        s->target_radius_m = guidance->target_radius_m();
    }
    fill_prediction(session, context, previous, *s);
    return s;
}

}  // namespace starsim::bindings
