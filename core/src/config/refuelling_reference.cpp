#include "starsim/config/refuelling_reference.hpp"

#include "toml_document.hpp"

namespace starsim::config {

namespace {
constexpr double kg_per_t = 1000.0;
}

RefuellingReference load_refuelling_reference(const std::filesystem::path& config_dir) {
    const detail::Document doc(config_dir / "refuelling" / "reference_flights.toml");
    const auto* depot = doc.root["depot"].as_table();
    const auto* tanker = doc.root["tanker"].as_table();
    const auto* hls = doc.root["hls"].as_table();
    if (!depot || !tanker || !hls) doc.fail("missing [depot], [tanker] or [hls]");
    const auto tonnes = [&](const toml::table& t, std::string_view key) {
        const double v = doc.number(t, key);
        if (!(v >= 0.0)) doc.fail("'" + std::string(key) + "' must be >= 0");
        return v * kg_per_t;
    };
    RefuellingReference r;
    r.depot.mission = doc.text(*depot, "mission");
    r.depot.initial_propellant_kg = tonnes(*depot, "initial_propellant_t");
    r.tanker.mission = doc.text(*tanker, "mission");
    r.tanker.dry_mass_kg = tonnes(*tanker, "dry_mass_t");
    r.tanker.residue_after_ascent_kg = tonnes(*tanker, "residue_after_ascent_t");
    r.tanker.rendezvous_kg = tonnes(*tanker, "rendezvous_t");
    r.tanker.delivered_oxidizer_kg = tonnes(*tanker, "delivered_oxidizer_t");
    r.tanker.delivered_fuel_kg = tonnes(*tanker, "delivered_fuel_t");
    r.tanker.chilldown_kg = tonnes(*tanker, "chilldown_t");
    r.tanker.settling_gas_kg = tonnes(*tanker, "settling_gas_t");
    r.tanker.main_at_undock_kg = tonnes(*tanker, "main_at_undock_t");
    r.tanker.main_at_catch_kg = tonnes(*tanker, "main_at_catch_t");
    r.tanker.dry_mass_sensitivity = doc.number(*tanker, "dry_mass_sensitivity");
    r.tanker.phasing_boil_off_kgps = tonnes(*tanker, "phasing_boil_off_t_per_day") / 86400.0;
    r.tanker.campaign_offset_below_kg = doc.number(*tanker, "campaign_offset_below_t") * kg_per_t;
    r.tanker.campaign_offset_above_kg = doc.number(*tanker, "campaign_offset_above_t") * kg_per_t;
    r.hls.mission = doc.text(*hls, "mission");
    r.hls.main_at_docking_kg = tonnes(*hls, "main_at_docking_t");
    r.hls.phasing_boil_off_kgps = tonnes(*hls, "phasing_boil_off_t_per_day") / 86400.0;
    return r;
}

}  // namespace starsim::config
