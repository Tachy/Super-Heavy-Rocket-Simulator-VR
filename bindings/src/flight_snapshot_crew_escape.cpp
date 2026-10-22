#include "flight_snapshot.hpp"
#include "starsim/sim/crew_escape.hpp"
#include "starsim/sim/landing_outcome.hpp"

namespace starsim::bindings {

namespace {

godot::Vector3 to_godot(const math::Vec3& v) { return {v.x, v.y, v.z}; }

godot::Dictionary member_dictionary(const sim::FlightSession& session, const sim::CrewMemberState& m) {
    godot::Dictionary d;
    d["phase"] = godot::String::utf8(gnc::to_string(m.phase));
    d["mode"] = godot::String::utf8(gnc::to_string(m.mode));
    d["fire_time_s"] = m.fire_time_s;
    d["load_g"] = m.load_g;
    d["peak_load_g"] = m.peak_load_g;
    d["min_clearance_m"] = m.min_clearance_m;
    d["outcome"] = godot::String::utf8(sim::to_string(m.outcome));
    d["reason"] = godot::String::utf8(m.reason.c_str());
    d["body_index"] = m.body_index ? static_cast<int64_t>(*m.body_index) : int64_t{-1};
    if (m.body_index && *m.body_index < session.body_count()) {
        const auto& body = session.body(*m.body_index);
        const auto& s = body.state();
        const auto t = session.telemetry(*m.body_index);
        d["position_gcrf_m"] = to_godot(s.position_gcrf_m);
        d["velocity_gcrf_mps"] = to_godot(s.velocity_gcrf_mps);
        d["attitude_gcrf"] = godot::Quaternion(s.attitude.x, s.attitude.y, s.attitude.z, s.attitude.w);
        d["latitude_rad"] = t.latitude_rad;
        d["longitude_rad"] = t.longitude_rad;
        d["height_m"] = t.altitude_m;
    }
    if (m.touchdown_enu_m) {
        d["touchdown_enu_m"] = to_godot(*m.touchdown_enu_m);
        d["touchdown_over_water"] = m.touchdown_over_water;
        d["touchdown_sink_mps"] = m.touchdown_sink_mps;
    }
    return d;
}

}  // namespace

godot::Dictionary build_crew_escape_state(const sim::FlightSession& session) {
    godot::Dictionary d;
    const auto* s = session.crew_escape_state();
    d["present"] = s != nullptr;
    if (!s) return d;
    d["armed"] = s->armed;
    d["fired"] = s->fired;
    d["early"] = s->early;
    d["reason"] = godot::String::utf8(s->give_up_reason.c_str());
    d["give_up_time_s"] = s->give_up_time_s;
    godot::Array members;
    for (const auto& m : s->members) members.push_back(member_dictionary(session, m));
    d["members"] = members;
    godot::Dictionary impact;
    impact["present"] = s->ship_impact_enu_m.has_value();
    if (s->ship_impact_enu_m) {
        impact["enu_m"] = to_godot(*s->ship_impact_enu_m);
        impact["speed_mps"] = s->ship_impact_speed_mps;
        impact["tower_distance_m"] = s->ship_impact_tower_distance_m;
        impact["shore_distance_m"] = s->ship_impact_shore_distance_m;
        impact["over_water"] = s->ship_impact_over_water;
    }
    d["ship_impact"] = impact;
    return d;
}

}  // namespace starsim::bindings
