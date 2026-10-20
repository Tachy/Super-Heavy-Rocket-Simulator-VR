#include "starsim/mission/refuelling_campaign.hpp"

#include <algorithm>
#include <cstdio>

namespace starsim::mission {

namespace {

std::string tonnes(double kg) {
    char buffer[32];
    std::snprintf(buffer, sizeof buffer, "%.1f t", kg / 1000.0);
    return buffer;
}

struct Depot {
    double t_s = 0.0;
    double propellant_kg = 0.0;
    CampaignLosses losses;
};

// Losses from depot.t_s to `to_s` in steps of at most step_s (rates at each step's start).
void advance(Depot& d, double to_s, const CampaignInputs& in, const CampaignModels& m,
             std::vector<CampaignSample>* samples) {
    while (d.t_s < to_s) {
        const double dt = std::min(in.step_s, to_s - d.t_s);
        const double boil = m.boil_off_kgps(d.t_s, d.propellant_kg) * dt;
        const double drag = m.drag_makeup_kgps(d.t_s) * dt;
        const double gas = in.attitude_gas_kgps * dt;
        // an emptied depot loses only what it holds: book the losses in proportion
        const double total = boil + drag + gas;
        const double share = total > d.propellant_kg && total > 0.0 ? d.propellant_kg / total : 1.0;
        d.losses.boil_off_kg += boil * share;
        d.losses.drag_makeup_kg += drag * share;
        d.losses.attitude_gas_kg += gas * share;
        d.propellant_kg -= total * share;
        d.t_s += dt;
        if (samples) samples->push_back({d.t_s, d.propellant_kg});
    }
}

// The first window at or after `earliest` that arrives port_spacing_s after `last_arrival_s`.
std::optional<CampaignFlight> next_slot(const std::function<std::optional<CampaignFlight>(double)>& next,
                                        double earliest, double last_arrival_s, const CampaignInputs& in) {
    auto slot = next(earliest);
    while (slot && slot->arrival_s < last_arrival_s + in.port_spacing_s) {
        slot = next(slot->launch_s + in.window_retry_s);
    }
    return slot;
}

}  // namespace

CampaignResult refuelling_campaign(const CampaignInputs& in, const CampaignModels& m) {
    CampaignResult r;
    if (in.demand_kg > in.depot_capacity_kg) {
        r.reason = "Depotvolumen reicht nicht: " + tonnes(in.demand_kg) + " gefordert, " +
                   tonnes(in.depot_capacity_kg) + " Tankvolumen";
        return r;
    }
    Depot depot;
    depot.propellant_kg = in.depot_initial_kg;
    r.fill.push_back({0.0, depot.propellant_kg});
    double last_launch_s = -in.launch_interval_s;  // the first tanker may launch at t = 0
    double last_arrival_s = -in.port_spacing_s;
    double previous_after_kg = -1.0;
    const auto finish = [&](const std::string& reason) {
        r.reason = reason;
        r.losses = depot.losses;
        r.tanker_flights = static_cast<int>(r.tankers.size());
        return r;
    };
    while (true) {
        if (depot.propellant_kg >= in.demand_kg) {
            // enough now: does it last until the HLS arrives?
            const auto hls = next_slot(m.next_hls, last_launch_s + in.launch_interval_s, last_arrival_s, in);
            if (!hls) return finish("Kein Startfenster für das HLS");
            Depot projected = depot;
            advance(projected, hls->arrival_s, in, m, nullptr);
            if (projected.propellant_kg >= in.demand_kg) {
                advance(depot, hls->arrival_s, in, m, &r.fill);
                r.closes = true;
                r.hls = hls;
                r.duration_s = hls->arrival_s;
                r.depot_at_hls_kg = depot.propellant_kg;
                return finish("");
            }
        }
        if (static_cast<int>(r.tankers.size()) >= in.max_tankers) {
            return finish("Kampagne schließt nicht nach " + std::to_string(in.max_tankers) + " Tankerflügen");
        }
        const auto slot = next_slot(m.next_tanker, last_launch_s + in.launch_interval_s, last_arrival_s, in);
        if (!slot) return finish("Kein Startfenster für Tanker " + std::to_string(r.tankers.size() + 1));
        advance(depot, slot->arrival_s, in, m, &r.fill);
        CampaignTanker t;
        t.flight = *slot;
        const double offered = std::max(0.0, slot->delivered_kg);
        const double vent = offered * in.transfer_vent_fraction;
        t.received_kg = std::min(offered - vent, in.depot_capacity_kg - depot.propellant_kg);
        depot.losses.transfer_vent_kg += vent;
        depot.propellant_kg += t.received_kg;
        t.depot_after_kg = depot.propellant_kg;
        r.tankers.push_back(t);
        r.fill.push_back({depot.t_s, depot.propellant_kg});
        last_launch_s = slot->launch_s;
        last_arrival_s = slot->arrival_s;
        if (slot->delivered_kg <= 0.0) {
            return finish("Tanker liefert nichts: " + tonnes(slot->delivered_kg) +
                          " nach Aufstieg, Phasenflug und Reserve");
        }
        if (previous_after_kg >= 0.0 && depot.propellant_kg <= previous_after_kg &&
            depot.propellant_kg < in.demand_kg) {
            return finish("Verluste übersteigen die Lieferung: Depot " + tonnes(depot.propellant_kg) +
                          " nach Tanker " + std::to_string(r.tankers.size()));
        }
        previous_after_kg = depot.propellant_kg;
    }
}

}  // namespace starsim::mission
