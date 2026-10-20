#include "starsim/sim/refuelling_campaign_text.hpp"

#include <cstdio>
#include <functional>
#include <map>
#include <sstream>
#include <stdexcept>

namespace starsim::sim {

namespace {

constexpr double seconds_per_hour = 3600.0;
constexpr double seconds_per_day = 86400.0;
constexpr double kg_per_t = 1000.0;
constexpr double m_per_km = 1000.0;

double parse_number(const std::string& flag, const std::string& text) {
    size_t used = 0;
    double v = 0.0;
    try {
        v = std::stod(text, &used);
    } catch (const std::exception&) {
        throw std::invalid_argument(flag + ": not a number: '" + text + "'");
    }
    if (used != text.size()) throw std::invalid_argument(flag + ": not a number: '" + text + "'");
    return v;
}

std::string line(const char* format, double a = 0.0, double b = 0.0, double c = 0.0, double d = 0.0) {
    char buffer[256];
    std::snprintf(buffer, sizeof buffer, format, a, b, c, d);
    return buffer;
}

}  // namespace

CampaignLevers campaign_levers_from_args(const std::vector<std::string>& args) {
    CampaignLevers l;
    struct Flag {
        double scale;
        bool zero_allowed;
        std::function<void(double)> set;
    };
    const std::map<std::string, Flag> flags{
        {"--height-km", {m_per_km, false, [&](double v) { l.depot_height_m = v; }}},
        {"--interval-h", {seconds_per_hour, false, [&](double v) { l.launch_interval_s = v; }}},
        {"--phasing-limit-h", {seconds_per_hour, false, [&](double v) { l.phasing_limit_s = v; }}},
        {"--tanker-dry-t", {kg_per_t, false, [&](double v) { l.tanker_dry_mass_kg = v; }}},
        {"--reserve-t", {kg_per_t, true, [&](double v) { l.tanker_reserve_kg = v; }}},
        {"--initial-t", {kg_per_t, true, [&](double v) { l.depot_initial_kg = v; }}},
        {"--insulation", {1.0, false, [&](double v) { l.depot_insulation_scale = v; }}},
        {"--hls-wait-d", {seconds_per_day, true, [&](double v) { l.hls_wait_s = v; }}},
    };
    for (size_t k = 0; k < args.size(); k += 2) {
        const auto it = flags.find(args[k]);
        if (it == flags.end()) throw std::invalid_argument("unknown flag '" + args[k] + "'");
        if (k + 1 >= args.size()) throw std::invalid_argument(args[k] + ": value missing");
        const double v = parse_number(args[k], args[k + 1]);
        if (!(it->second.zero_allowed ? v >= 0.0 : v > 0.0)) {
            throw std::invalid_argument(args[k] + ": out of range: " + args[k + 1]);
        }
        it->second.set(v * it->second.scale);
    }
    return l;
}

std::string campaign_report_text(const CampaignReport& r, const time::TimeScaleConverter& time) {
    std::ostringstream out;
    // to the second: YYYY-MM-DDTHH:MM:SS
    const auto utc_at = [&](time::Epoch t) { return time.to_utc(t).to_iso_string().substr(0, 19); };
    const auto utc = [&](double t_s) { return utc_at(r.start + t_s); };
    out << "Refuelling campaign (r6) from " << utc_at(r.start) << "\n";
    out << line("  depot %.0f km, tank volume %.1f t, drag makeup %.1f kg/day\n", r.depot_height_m / m_per_km,
                r.depot_capacity_kg / kg_per_t, r.drag_makeup_kgps * seconds_per_day);
    out << "  lunar mission " << r.lunar_mission_id;
    if (!r.lunar_mission_id.empty()) out << " (" << utc_at(r.lunar_epoch) << ")";
    out << "\n";
    out << line("  HLS needs %.1f t (dry %.1f t), docks with %.1f t propellant, tanks %.1f t\n",
                r.hls_required_mass_kg / kg_per_t, r.hls_dry_mass_kg / kg_per_t,
                r.hls_propellant_at_docking_kg / kg_per_t, r.hls_capacity_kg / kg_per_t);
    out << line("  demand from the depot %.1f t; tanker delivers %.1f t at the reference phasing (%.1f h), altitude "
                "cost %.2f t\n",
                r.demand_kg / kg_per_t, r.tanker_base_delivered_kg / kg_per_t, r.reference_phasing_s / seconds_per_hour,
                r.altitude_cost_kg / kg_per_t);
    out << "\n   #  launch (d)  arrival (d)  launch UTC            delivered (t)  depot after (t)\n";
    for (size_t k = 0; k < r.result.tankers.size(); ++k) {
        const auto& t = r.result.tankers[k];
        char buffer[160];
        std::snprintf(buffer, sizeof buffer, "  %2zu  %10.2f  %11.2f  %-20s  %13.1f  %15.1f\n", k + 1,
                      t.flight.launch_s / seconds_per_day, t.flight.arrival_s / seconds_per_day,
                      utc(t.flight.launch_s).c_str(), t.flight.delivered_kg / kg_per_t, t.depot_after_kg / kg_per_t);
        out << buffer;
    }
    if (r.result.hls) {
        out << line("\n  HLS launch %.2f d, docks %.2f d with the depot at %.1f t\n", r.result.hls->launch_s / seconds_per_day,
                    r.result.hls->arrival_s / seconds_per_day, r.result.depot_at_hls_kg / kg_per_t);
    }
    const auto& loss = r.result.losses;
    out << line("  losses: boil-off %.1f t, drag makeup %.2f t, attitude gas %.2f t, ", loss.boil_off_kg / kg_per_t,
                loss.drag_makeup_kg / kg_per_t, loss.attitude_gas_kg / kg_per_t);
    out << line("transfer vent %.1f t\n\n", loss.transfer_vent_kg / kg_per_t);
    if (r.result.closes) {
        out << "Tanker flights: " << r.result.tanker_flights
            << line(" (campaign %.1f days)\n", r.result.duration_s / seconds_per_day);
    } else {
        out << "Campaign does not close after " << r.result.tanker_flights << " tanker flights: " << r.result.reason
            << "\n";
    }
    return out.str();
}

}  // namespace starsim::sim
