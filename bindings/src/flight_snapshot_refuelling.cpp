#include "flight_snapshot.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::bindings {

namespace {

constexpr double kg_per_t = 1000.0;

godot::String german(const std::string& s) { return godot::String::utf8(s.c_str()); }

godot::Dictionary absent() {
    godot::Dictionary d;
    d["present"] = false;
    return d;
}

const char* phase_name(sim::TransferPhase phase) {
    switch (phase) {
        case sim::TransferPhase::Idle: return "Bereit";
        case sim::TransferPhase::Settling: return "Absetzen";
        case sim::TransferPhase::Chilldown: return "Vorkühlen";
        case sim::TransferPhase::Flowing: return "Umfüllen";
        case sim::TransferPhase::Paused: return "Pausiert";
        case sim::TransferPhase::Complete: return "Abgeschlossen";
        case sim::TransferPhase::Stopped: return "Gestoppt";
    }
    return "";
}

}  // namespace

godot::Dictionary build_transfer_state(const sim::FlightSession& session) {
    const auto& t = session.transfer_state();
    if (t.phase == sim::TransferPhase::Idle) return absent();
    godot::Dictionary d;
    d["present"] = true;
    d["phase"] = godot::String::utf8(phase_name(t.phase));
    d["status"] = german(t.status);
    d["elapsed_s"] = t.elapsed_s;
    d["delivered_oxidizer_kg"] = t.delivered_kg[sim::transfer_oxidizer];
    d["delivered_fuel_kg"] = t.delivered_kg[sim::transfer_fuel];
    d["flow_oxidizer_kgps"] = t.flow_kgps[sim::transfer_oxidizer];
    d["flow_fuel_kgps"] = t.flow_kgps[sim::transfer_fuel];
    d["settling_gas_kg"] = t.settling_gas_kg;
    d["settling_thrust_n"] = t.settling_thrust_n;
    d["chilldown_kg"] = t.chilldown_kg[sim::transfer_oxidizer] + t.chilldown_kg[sim::transfer_fuel];
    d["boil_off_kg"] = t.boil_off_kg;
    return d;
}

godot::Dictionary build_depot_state(const sim::FlightSession& session) {
    const auto index = session.station_index();
    if (!index) return absent();
    const auto& body = session.body(*index);
    const auto& stage = body.vehicle().stages().back();
    if (stage.config.transfer_connectors.empty()) return absent();  // a station, not a depot (the ISS)
    double capacity_kg = 0.0;
    for (const auto& t : stage.tanks) {
        if (t.config.role == config::TankRole::Main) capacity_kg += t.config.capacity_kg;
    }
    const double main_kg = body.vehicle().main_propellant_kg();
    const auto& rate = body.boil_off_rate();
    godot::Dictionary d;
    d["present"] = true;
    d["main_propellant_kg"] = main_kg;
    d["capacity_kg"] = capacity_kg;
    d["fill_fraction"] = capacity_kg > 0.0 ? main_kg / capacity_kg : 0.0;
    d["boil_off_kg_per_day"] = (rate.oxidizer_kgps + rate.fuel_kgps) * constants::seconds_per_day;
    d["docked"] = session.docked();
    return d;
}

godot::Dictionary campaign_dictionary(const sim::CampaignReport& r, const time::TimeScaleConverter& time) {
    const auto utc = [&](time::Epoch t) { return godot::String(time.to_utc(t).to_iso_string().substr(0, 19).c_str()); };
    godot::Dictionary d;
    d["closes"] = r.result.closes;
    d["reason"] = german(r.result.reason);
    d["tanker_flights"] = r.result.tanker_flights;
    d["duration_d"] = r.result.duration_s / constants::seconds_per_day;
    d["demand_t"] = r.demand_kg / kg_per_t;
    d["hls_required_t"] = r.hls_required_mass_kg / kg_per_t;
    d["depot_at_hls_t"] = r.result.depot_at_hls_kg / kg_per_t;
    godot::Array tankers;
    for (const auto& t : r.result.tankers) {
        godot::Dictionary e;
        e["launch_utc"] = utc(r.start + t.flight.launch_s);
        e["arrival_utc"] = utc(r.start + t.flight.arrival_s);
        e["delivered_t"] = t.flight.delivered_kg / kg_per_t;
        e["depot_after_t"] = t.depot_after_kg / kg_per_t;
        if (const auto& o = t.flight.phasing_orbit) {  // f1: the window's choice
            e["phasing_periapsis_km"] = (o->periapsis_radius_m - constants::wgs84_semi_major_axis_m) / 1e3;
            e["phasing_apoapsis_km"] = (o->apoapsis_radius_m - constants::wgs84_semi_major_axis_m) / 1e3;
        }
        tankers.push_back(e);
    }
    d["tankers"] = tankers;
    godot::Array fill;
    for (const auto& s : r.result.fill) {
        godot::Array p;
        p.push_back(s.t_s / constants::seconds_per_day);
        p.push_back(s.depot_propellant_kg / kg_per_t);
        fill.push_back(p);
    }
    d["fill"] = fill;
    godot::Dictionary losses;
    losses["boil_off_t"] = r.result.losses.boil_off_kg / kg_per_t;
    losses["drag_t"] = r.result.losses.drag_makeup_kg / kg_per_t;
    losses["gas_t"] = r.result.losses.attitude_gas_kg / kg_per_t;
    losses["vent_t"] = r.result.losses.transfer_vent_kg / kg_per_t;
    d["losses"] = losses;
    d["lunar_mission"] = godot::String(r.lunar_mission_id.c_str());
    d["lunar_epoch_utc"] = r.lunar_mission_id.empty() ? godot::String() : utc(r.lunar_epoch);
    return d;
}

}  // namespace starsim::bindings
