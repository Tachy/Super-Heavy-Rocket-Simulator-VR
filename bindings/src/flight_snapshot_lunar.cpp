#include "flight_snapshot.hpp"

#include <cmath>

#include "starsim/math/constants.hpp"
#include "starsim/sim/leg_landing_sim.hpp"

namespace starsim::bindings {

namespace {

godot::Vector3 to_godot(const math::Vec3& v) { return {v.x, v.y, v.z}; }
godot::String german(const std::string& s) { return godot::String::utf8(s.c_str()); }

godot::Array lunar_burns(const sim::FlightSession& session) {
    godot::Array burns;
    for (const auto& b : session.lunar_burns()) {
        godot::Dictionary d;
        d["step"] = godot::String::utf8(sim::to_string(b.step));
        d["epoch_tdb_s"] = b.node.epoch.tdb_seconds_past_j2000();
        d["delta_v_mps"] = norm(b.node.delta_v_gcrf_mps);
        d["mass_kg"] = b.mass_kg;
        burns.push_back(d);
    }
    return burns;
}

godot::Dictionary lunar_descent(const sim::FlightSession& session) {
    godot::Dictionary d;
    const auto* plan = session.lunar_descent_plan();
    d["present"] = plan != nullptr;
    if (!plan) return d;
    const auto& p = plan->touchdown;
    d["feasible"] = plan->feasible;
    d["reason"] = german(plan->reason);
    d["pdi_epoch_tdb_s"] = plan->pdi_epoch.tdb_seconds_past_j2000();
    d["touchdown_latitude_deg"] = p.latitude_rad * constants::rad_to_deg;
    d["touchdown_longitude_deg"] = p.longitude_rad * constants::rad_to_deg;
    d["touchdown_height_m"] = p.height_m;
    d["slope_deg"] = p.slope_rad * constants::rad_to_deg;
    d["offset_m"] = p.offset_m;
    d["periselene_altitude_km"] = plan->periselene_altitude_m / 1000.0;
    return d;
}

godot::Dictionary lunar_touchdown(const sim::FlightSession& session) {
    godot::Dictionary d;
    const auto& touchdown = session.lunar_touchdown();
    d["present"] = touchdown.has_value();
    if (!touchdown) return d;
    d["outcome"] = godot::String::utf8(sim::to_string(touchdown->outcome));
    d["offset_m"] = touchdown->offset_m;
    d["sink_mps"] = touchdown->sink_mps;
    d["lateral_mps"] = touchdown->lateral_mps;
    return d;
}

godot::Dictionary lunar_return(const sim::FlightSession& session) {
    godot::Dictionary d;
    const auto* plan = session.lunar_return_plan();
    d["present"] = plan != nullptr;
    if (!plan) return d;
    d["feasible"] = plan->feasible;
    d["reason"] = german(plan->reason);
    d["tei_delta_v_mps"] = plan->tei_delta_v_mps;
    d["braking_delta_v_mps"] = plan->braking_delta_v_mps;
    d["interface_epoch_tdb_s"] = plan->arrival.interface_epoch.tdb_seconds_past_j2000();
    d["interface_speed_mps"] = plan->arrival.speed_mps;
    d["interface_angle_deg"] = plan->arrival.flight_path_rad * constants::rad_to_deg;
    d["replans"] = session.lunar_return_replans();
    return d;
}

godot::Dictionary pilot_cue(const sim::FlightSession& session) {
    godot::Dictionary d;
    const auto cue = session.lunar_pilot_cue();
    d["present"] = cue.has_value();
    if (!cue) return d;
    d["burn_epoch_tdb_s"] = cue->planned.epoch.tdb_seconds_past_j2000();
    d["remaining_delta_v_gcrf_mps"] = to_godot(cue->remaining_delta_v_gcrf_mps);
    d["remaining_mps"] = norm(cue->remaining_delta_v_gcrf_mps);
    d["attitude_error_deg"] = cue->attitude_error_rad * constants::rad_to_deg;
    d["window_end_s"] = cue->window_end_s;
    return d;
}

}  // namespace

godot::Dictionary build_lunar_state(const sim::FlightSession& session) {
    godot::Dictionary d;
    const bool present = session.active() &&
                         (session.lunar_step() != sim::LunarStep::None || !session.lunar_burns().empty() ||
                          session.lunar_descent_plan() || session.lunar_return_plan() ||
                          session.lunar_mission_complete() || session.lunar_mission_ended());
    d["present"] = present;
    if (!present) return d;
    d["step"] = godot::String::utf8(sim::to_string(session.lunar_step()));
    d["status"] = german(session.lunar_status());
    d["mission_complete"] = session.lunar_mission_complete();
    d["mission_ended"] = session.lunar_mission_ended();
    d["burns"] = lunar_burns(session);
    d["descent"] = lunar_descent(session);
    d["touchdown"] = lunar_touchdown(session);
    d["return"] = lunar_return(session);
    d["pilot_cue"] = pilot_cue(session);
    return d;
}

godot::Dictionary build_lunar_site(const sim::FlightSession& session) {
    godot::Dictionary d;
    const auto* site = session.lunar_landing_site();
    d["present"] = site != nullptr;
    if (!site) return d;
    const double lat = site->config.latitude_rad, lon = site->config.longitude_rad;
    d["id"] = german(site->config.id);
    d["name"] = german(site->config.name);
    d["latitude_deg"] = lat * constants::rad_to_deg;
    d["longitude_deg"] = lon * constants::rad_to_deg;
    d["height_m"] = site->height_m;
    const auto* terrain = session.moon_terrain();
    const math::Vec3 up{std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
    const double radius_m = terrain ? terrain->sphere_radius_m() + terrain->height_m(lat, lon)
                                    : constants::moon_mean_radius_m + site->height_m;
    d["position_moon_me_m"] = to_godot(radius_m * up);
    return d;
}

}  // namespace starsim::bindings
