#include <cmath>

#include "flight_snapshot.hpp"
#include "starsim/contact/station_collision.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/docking_port.hpp"

namespace starsim::bindings {

namespace {

godot::Vector3 to_godot(const math::Vec3& v) { return {v.x, v.y, v.z}; }
godot::String german(const std::string& s) { return godot::String::utf8(s.c_str()); }
math::Vec3 axis_column(const math::Mat3& m, int c) { return {m(0, c), m(1, c), m(2, c)}; }
const char* mechanism_name(config::DockingMechanism m) {
    return m == config::DockingMechanism::ProbeDrogue ? "probe_drogue" : "idss";
}

godot::Dictionary absent() {
    godot::Dictionary d;
    d["present"] = false;
    return d;
}

godot::Array warnings(const sim::FlightSession& session) {
    godot::Array a;
    for (const auto& w : session.rendezvous_warnings()) a.push_back(german(w));
    for (const auto& w : session.proximity_warnings()) a.push_back(german(w));
    return a;
}

}  // namespace

godot::Dictionary build_iss_state(const sim::FlightSession& session) {
    const auto index = session.station_index();
    if (!index) return absent();
    const auto& body = session.body(*index);
    const auto& st = body.state();
    godot::Dictionary d;
    d["present"] = true;
    d["index"] = static_cast<int64_t>(*index);
    d["position_gcrf_m"] = to_godot(st.position_gcrf_m);
    d["velocity_gcrf_mps"] = to_godot(st.velocity_gcrf_mps);
    d["attitude_gcrf"] = godot::Quaternion(st.attitude.x, st.attitude.y, st.attitude.z, st.attitude.w);
    d["mass_kg"] = body.vehicle().mass_properties().mass_kg;  // with a docked vehicle: the compound
    d["docked_mass_kg"] = session.docked() ? session.body(session.controlled_index()).vehicle().mass_properties().mass_kg
                                           : 0.0;
    godot::Array ports;
    for (const auto& stage : body.vehicle().stages()) {
        for (const auto& p : stage.config.docking_ports) {
            const auto k = vehicle::port_kinematics(body, p.id);
            godot::Dictionary port;
            port["id"] = german(p.id);
            port["name"] = german(p.name);
            port["position_gcrf_m"] = to_godot(k.position_gcrf_m);
            port["axis_gcrf"] = to_godot(axis_column(k.axes_gcrf, 0));
            port["up_gcrf"] = to_godot(axis_column(k.axes_gcrf, 2));
            port["soft_capture"] = p.soft_capture == config::SoftCaptureMode::Active ? "active" : "passive";
            port["mechanism"] = mechanism_name(p.mechanism);
            godot::Array elements;  // probe/drogue: the probes or drogues, stage frame (r1 minor)
            for (const auto& e : p.elements_m) elements.push_back(to_godot(e));
            port["elements_m"] = elements;
            ports.push_back(port);
        }
    }
    d["ports"] = ports;
    d["warnings"] = warnings(session);
    return d;
}

godot::Dictionary build_rendezvous_plan(const sim::FlightSession& session) {
    const auto* plan = session.rendezvous_plan();
    if (!plan) return absent();
    godot::Dictionary d;
    d["present"] = true;
    d["feasible"] = plan->feasible;
    d["reason"] = german(plan->reason);
    d["step"] = german(sim::to_string(session.rendezvous_step()));
    d["status"] = german(session.rendezvous_status());
    d["replans"] = session.rendezvous_replans();
    d["total_delta_v_mps"] = plan->total_delta_v_mps;
    d["propellant_kg"] = plan->propellant_kg;
    d["waypoint0_tdb_s"] = plan->schedule.waypoint0_epoch.tdb_seconds_past_j2000();
    godot::Array burns;
    for (const auto& b : session.rendezvous_burns()) {  // flown
        godot::Dictionary e;
        e["kind"] = german(mission::german_name(b.kind));
        e["epoch_tdb_s"] = b.node.epoch.tdb_seconds_past_j2000();
        e["delta_v_mps"] = norm(b.node.delta_v_gcrf_mps);
        e["flown"] = true;
        burns.push_back(e);
    }
    const size_t flown = session.rendezvous_burns().size();
    for (size_t i = 0; i < plan->burns.size(); ++i) {  // still planned (the plan's burns after the flown ones)
        const auto& b = plan->burns[i];
        if (!session.rendezvous_burns().empty() && b.node.epoch <= session.rendezvous_burns().back().node.epoch) continue;
        godot::Dictionary e;
        e["kind"] = german(mission::german_name(b.kind));
        e["epoch_tdb_s"] = b.node.epoch.tdb_seconds_past_j2000();
        e["delta_v_mps"] = norm(b.node.delta_v_gcrf_mps);
        e["flown"] = false;
        burns.push_back(e);
    }
    d["burns_flown"] = static_cast<int64_t>(flown);
    d["burns"] = burns;
    return d;
}

godot::Dictionary build_docking_state(const sim::FlightSession& session) {
    const auto rel = session.relative_port_state();
    const auto docking = session.docking_phase();
    const auto proximity = session.proximity_phase();
    const auto departure = session.departure_phase();
    if (!rel && !docking && !proximity && !departure) return absent();
    godot::Dictionary d;
    d["present"] = true;
    d["proximity_phase"] = proximity ? german(gnc::to_string(*proximity)) : godot::String();
    d["docking_phase"] = docking ? german(contact::to_string(*docking)) : godot::String();
    d["departure_phase"] = departure ? german(gnc::to_string(*departure)) : godot::String();
    d["status"] = german(!session.docking_status().empty() ? session.docking_status() : session.proximity_status());
    if (rel) {
        d["position_port_m"] = to_godot(rel->position_m);
        d["velocity_port_mps"] = to_godot(rel->velocity_mps);
        d["range_m"] = rel->range_m;
        d["range_rate_mps"] = -rel->closing_speed_mps;
        d["offset_m"] = std::hypot(rel->position_m.y, rel->position_m.z);
        d["misalignment_deg"] = rel->misalignment_rad * constants::rad_to_deg;
        d["clocking_deg"] = rel->clocking_rad * constants::rad_to_deg;
    }
    using contact::DockingPhase;
    const bool latched = docking && (*docking == DockingPhase::SoftCapture || *docking == DockingPhase::Retraction ||
                                     *docking == DockingPhase::HooksClosing || *docking == DockingPhase::Docked ||
                                     *docking == DockingPhase::HooksOpening);
    d["latched"] = latched;
    d["hooks_closed"] = docking && *docking == DockingPhase::Docked;
    d["docked"] = session.docked();
    d["collision_impulse_ns"] = session.collision_impulse_ns().value_or(0.0);
    if (const auto loads = session.docking_loads()) {
        godot::Dictionary l;
        l["compression_n"] = loads->compression_n;
        l["shear_n"] = loads->shear_n;
        l["bending_nm"] = loads->bending_nm;
        d["loads"] = l;
    }
    return d;
}

godot::Dictionary build_station_geometry(const sim::FlightSession& session) {
    const auto index = session.station_index();
    if (!index) return absent();
    const auto& vehicle = session.body(*index).vehicle();
    config::VehicleConfig config;
    for (const auto& stage : vehicle.stages()) config.stages.push_back(stage.config);
    const auto shape = contact::station_shape(config);
    godot::Dictionary d;
    d["present"] = true;
    godot::Array boxes, capsules, ports;
    for (const auto& b : shape.boxes) {
        godot::Dictionary e;
        e["center_m"] = to_godot(b.center);
        e["axis_x"] = to_godot(axis_column(b.axes, 0));
        e["axis_y"] = to_godot(axis_column(b.axes, 1));
        e["axis_z"] = to_godot(axis_column(b.axes, 2));
        e["half_extents_m"] = to_godot(b.half_extents);
        boxes.push_back(e);
    }
    for (const auto& c : shape.capsules) {
        godot::Dictionary e;
        e["a_m"] = to_godot(c.a_m);
        e["b_m"] = to_godot(c.b_m);
        e["radius_m"] = c.radius_m;
        capsules.push_back(e);
    }
    double base = 0.0;
    for (const auto& stage : config.stages) {
        for (const auto& p : stage.docking_ports) {
            godot::Dictionary e;
            e["id"] = german(p.id);
            e["position_m"] = to_godot(p.position_m + math::Vec3{base, 0.0, 0.0});
            e["axis"] = to_godot(p.axis);
            e["up"] = to_godot(p.up);
            e["mechanism"] = mechanism_name(p.mechanism);
            godot::Array elements;  // probe/drogue: the probes or drogues, in the same frame as position_m
            for (const auto& el : p.elements_m) elements.push_back(to_godot(el + math::Vec3{base, 0.0, 0.0}));
            e["elements_m"] = elements;
            ports.push_back(e);
        }
        base += stage.length_m;
    }
    d["boxes"] = boxes;
    d["capsules"] = capsules;
    d["ports"] = ports;
    return d;
}

godot::Dictionary build_iss_mission(const sim::FlightSession& session, const SnapshotContext& context) {
    const auto& q = session.iss_mission();
    if (!q && session.iss_mission_leg() == sim::IssMissionLeg::None) return absent();
    godot::Dictionary d;
    d["present"] = true;
    d["leg"] = german(sim::to_string(session.iss_mission_leg()));
    d["status"] = german(session.iss_mission_status());
    d["port"] = q ? german(q->station_port) : godot::String();
    d["liftoff_tdb_s"] = context.iss_liftoff_tdb_s;
    d["separation_tdb_s"] = context.iss_separation_tdb_s;
    d["plane_bias_deg"] = context.iss_plane_bias_deg;
    d["undock_tdb_s"] = q && q->undock_epoch ? q->undock_epoch->tdb_seconds_past_j2000() : 0.0;
    d["deorbit_tdb_s"] = q && q->deorbit_epoch ? q->deorbit_epoch->tdb_seconds_past_j2000() : 0.0;
    d["warnings"] = context.mission_warnings.duplicate();
    return d;
}

}  // namespace starsim::bindings
