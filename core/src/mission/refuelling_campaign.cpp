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
    if (!(in.transfer_vent_fraction >= 0.0 && in.transfer_vent_fraction < 1.0)) {
        r.reason = "Abblaseanteil des Transfers muss zwischen 0 und 1 liegen";
        return r;
    }
    if (in.demand_kg > in.depot_capacity_kg) {
        r.reason = "Depotvolumen reicht nicht: " + tonnes(in.demand_kg) + " gefordert, " +
                   tonnes(in.depot_capacity_kg) + " Tankvolumen";
        return r;
    }
    Depot depot;
    depot.propellant_kg = std::min(in.depot_initial_kg, in.depot_capacity_kg);  // review I3
    r.fill.push_back({0.0, depot.propellant_kg});
    // the first flight may launch at earliest_launch_s (t = 0); a live re-plan carries its last flight (f2)
    double last_launch_s = std::max(in.previous_launch_s, in.earliest_launch_s - in.launch_interval_s);
    double last_arrival_s = std::max(in.previous_arrival_s, -in.port_spacing_s);
    std::vector<double> after_kg;  // the fill after every tanker
    const auto finish = [&](const std::string& reason) {
        r.reason = reason;
        r.losses = depot.losses;
        r.tanker_flights = static_cast<int>(r.tankers.size());
        return r;
    };
    while (true) {
        // Does the depot last until the HLS arrives? Checked at every step, not only once the demand is in: the
        // HLS's extra demand may be negative - a phasing window cheaper than its reference (f4, f1 minor M3).
        const bool enough = depot.propellant_kg >= in.demand_kg;
        const auto hls = next_slot(m.next_hls, last_launch_s + in.launch_interval_s, last_arrival_s, in);
        if (enough && !hls) return finish("Kein Startfenster für das HLS");
        if (hls) {
            // the HLS's boil-off while it waits loaded for its launch, with its arrival's Sun (r6 minor)
            const double wait_kg = m.hls_wait_kg ? m.hls_wait_kg(hls->arrival_s) : 0.0;
            if (wait_kg > in.hls_tank_room_kg) {
                return finish("HLS-Tanks zu klein für die Wartezeit: " + tonnes(wait_kg) + " Verdampfung, " +
                              tonnes(in.hls_tank_room_kg) + " frei");
            }
            Depot projected = depot;
            advance(projected, hls->arrival_s, in, m, nullptr);
            // f1: the extra phasing demand and the wait come on top, grossed up by the vent; never below zero
            const double required_kg =
                std::max(0.0, in.demand_kg + gross_for_net_kg(hls->extra_demand_kg + wait_kg, in.transfer_vent_fraction));
            if (enough && projected.propellant_kg < required_kg && depot.propellant_kg >= in.depot_capacity_kg) {
                // review I2: full tanks, yet the losses until the HLS arrives eat into the demand
                return finish("Depotvolumen reicht nicht für die Verluste bis zur HLS-Ankunft: " +
                              tonnes(projected.propellant_kg) + " bei Ankunft, " + tonnes(required_kg) + " gefordert");
            }
            if (projected.propellant_kg >= required_kg) {
                advance(depot, hls->arrival_s, in, m, &r.fill);
                r.closes = true;
                r.hls = hls;
                r.duration_s = hls->arrival_s;
                r.depot_at_hls_kg = depot.propellant_kg;
                r.hls_wait_kg = wait_kg;
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
        const bool delivers = slot->delivered_kg > 0.0;  // false for NaN as well
        const double offered = delivers ? slot->delivered_kg : 0.0;
        const double vent = offered * in.transfer_vent_fraction;
        t.received_kg = std::min(offered - vent, in.depot_capacity_kg - depot.propellant_kg);
        depot.losses.transfer_vent_kg += vent;
        depot.propellant_kg += t.received_kg;
        t.depot_after_kg = depot.propellant_kg;
        r.tankers.push_back(t);
        r.fill.push_back({depot.t_s, depot.propellant_kg});
        last_launch_s = slot->launch_s;
        last_arrival_s = slot->arrival_s;
        if (!delivers) {
            return finish("Tanker liefert nichts: " + tonnes(slot->delivered_kg) +
                          " nach Aufstieg, Phasenflug und Reserve");
        }
        // review I4: against the fill two tankers back - alternating window gaps lose over one long gap and gain
        // over the pair
        if (after_kg.size() >= 2 && depot.propellant_kg <= after_kg[after_kg.size() - 2] &&
            depot.propellant_kg < in.demand_kg) {
            return finish("Verluste übersteigen die Lieferung: Depot " + tonnes(depot.propellant_kg) +
                          " nach Tanker " + std::to_string(r.tankers.size()));
        }
        after_kg.push_back(depot.propellant_kg);
    }
}

}  // namespace starsim::mission
