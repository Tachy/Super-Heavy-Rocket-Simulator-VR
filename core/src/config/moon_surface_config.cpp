#include "starsim/config/moon_surface_config.hpp"

#include <cmath>
#include <string>
#include <string_view>

#include "starsim/math/constants.hpp"
#include "toml_document.hpp"

namespace starsim::config {

double SizeFrequency::cumulative_density_per_m2(double diameter_m) const {
    if (form == "exponential") return density_per_m2 * std::exp(-exponent * diameter_m);
    return density_per_m2 * std::pow(diameter_m / reference_diameter_m, -exponent);
}

namespace {

// Reads keys of one table with German messages that name the full key ("craters.sfd.exponent").
class Reader {
public:
    Reader(const detail::Document& doc, std::string prefix) : doc_(doc), prefix_(std::move(prefix)) {}

    const toml::table& table(const toml::table& parent, std::string_view key) const {
        if (auto t = parent[key].as_table()) return *t;
        fail("Tabelle [" + name(key) + "] fehlt");
    }
    double number(const toml::table& t, std::string_view key) const {
        const auto v = t[key].value<double>();
        if (!v) fail("Schlüssel '" + name(key) + "' fehlt (Zahl erwartet)");
        if (!std::isfinite(*v)) fail("Schlüssel '" + name(key) + "' muss eine endliche Zahl sein");
        return *v;
    }
    double positive(const toml::table& t, std::string_view key) const {
        const double v = number(t, key);
        if (!(v > 0.0)) fail("Schlüssel '" + name(key) + "' muss positiv sein");
        return v;
    }
    double non_negative(const toml::table& t, std::string_view key) const {
        const double v = number(t, key);
        if (v < 0.0) fail("Schlüssel '" + name(key) + "' darf nicht negativ sein");
        return v;
    }
    double fraction(const toml::table& t, std::string_view key) const {
        const double v = non_negative(t, key);
        if (v > 1.0) fail("Schlüssel '" + name(key) + "' muss zwischen 0 und 1 liegen");
        return v;
    }
    std::string choice(const toml::table& t, std::string_view key, std::string_view a, std::string_view b) const {
        const auto v = t[key].value<std::string>();
        if (!v) fail("Schlüssel '" + name(key) + "' fehlt (Text erwartet)");
        if (*v != a && *v != b) {
            fail("Schlüssel '" + name(key) + "': \"" + *v + "\" ungültig (erlaubt: \"" + std::string(a) + "\", \"" +
                 std::string(b) + "\")");
        }
        return *v;
    }
    [[noreturn]] void fail(const std::string& what) const { doc_.fail(what); }
    std::string name(std::string_view key) const { return prefix_.empty() ? std::string(key) : prefix_ + "." + std::string(key); }

private:
    const detail::Document& doc_;
    std::string prefix_;
};

SizeFrequency read_sfd(const detail::Document& doc, const toml::table& parent, const std::string& prefix) {
    const Reader outer(doc, prefix);
    const Reader r(doc, prefix + ".sfd");
    const auto& t = outer.table(parent, "sfd");
    SizeFrequency s;
    s.form = r.choice(t, "form", "power", "exponential");
    s.reference_diameter_m = r.positive(t, "reference_diameter_m");
    s.density_per_m2 = r.positive(t, "density_per_m2");
    s.exponent = r.positive(t, "exponent");
    s.min_diameter_m = r.positive(t, "min_diameter_m");
    s.max_diameter_m = r.positive(t, "max_diameter_m");
    if (s.max_diameter_m <= s.min_diameter_m) r.fail("'" + r.name("max_diameter_m") + "' muss größer als min_diameter_m sein");
    return s;
}

}  // namespace

MoonSurfaceConfig load_moon_surface_config(const std::filesystem::path& file) {
    const detail::Document doc(file);
    const Reader top(doc, "");
    MoonSurfaceConfig cfg;

    const auto& craters = top.table(doc.root, "craters");
    const Reader c(doc, "craters");
    cfg.craters.sfd = read_sfd(doc, craters, "craters");
    cfg.craters.depth_to_diameter_fresh = c.positive(craters, "depth_to_diameter_fresh");
    cfg.craters.depth_to_diameter_degraded = c.positive(craters, "depth_to_diameter_degraded");
    cfg.craters.rim_height_to_diameter = c.non_negative(craters, "rim_height_to_diameter");
    cfg.craters.degraded_fraction = c.fraction(craters, "degraded_fraction");
    if (cfg.craters.depth_to_diameter_degraded > cfg.craters.depth_to_diameter_fresh) {
        c.fail("'craters.depth_to_diameter_degraded' darf nicht größer als depth_to_diameter_fresh sein");
    }

    const auto& roughness = top.table(doc.root, "roughness");
    const Reader g(doc, "roughness");
    cfg.roughness.hurst_exponent = g.positive(roughness, "hurst_exponent");
    if (cfg.roughness.hurst_exponent >= 1.0) g.fail("'roughness.hurst_exponent' muss kleiner als 1 sein");
    cfg.roughness.rms_height_at_baseline_m = g.positive(roughness, "rms_height_at_baseline_m");
    cfg.roughness.baseline_m = g.positive(roughness, "baseline_m");
    cfg.roughness.rms_height_measured_total_m = g.positive(roughness, "rms_height_measured_total_m");
    if (cfg.roughness.rms_height_at_baseline_m > cfg.roughness.rms_height_measured_total_m) {
        g.fail("'roughness.rms_height_at_baseline_m' (Rest) darf nicht größer als rms_height_measured_total_m sein");
    }

    const auto& rocks = top.table(doc.root, "rocks");
    const Reader k(doc, "rocks");
    cfg.rocks.sfd = read_sfd(doc, rocks, "rocks");
    cfg.rocks.burial_fraction_mean = k.fraction(rocks, "burial_fraction_mean");
    cfg.rocks.burial_fraction_sd = k.non_negative(rocks, "burial_fraction_sd");
    cfg.rocks.albedo_to_regolith = k.positive(rocks, "albedo_to_regolith");
    cfg.rocks.abundance_source = k.choice(rocks, "abundance_source", "uniform", "diviner");
    cfg.rocks.abundance_scale = k.positive(rocks, "abundance_scale");
    cfg.rocks.abundance_reference_area_fraction = k.fraction(rocks, "abundance_reference_area_fraction");
    cfg.rocks.abundance_reference_min_diameter_m = k.positive(rocks, "abundance_reference_min_diameter_m");

    const auto& hapke = top.table(doc.root, "hapke");
    const Reader h(doc, "hapke");
    cfg.hapke.w = h.positive(hapke, "w");
    if (cfg.hapke.w >= 1.0) h.fail("'hapke.w' muss kleiner als 1 sein");
    cfg.hapke.b = h.fraction(hapke, "b");
    cfg.hapke.c = h.number(hapke, "c");
    if (std::abs(cfg.hapke.c) > 1.0) h.fail("'hapke.c' muss zwischen -1 und 1 liegen (Hapke 2012)");
    cfg.hapke.bs0 = h.non_negative(hapke, "bs0");
    cfg.hapke.hs = h.non_negative(hapke, "hs");
    cfg.hapke.bc0 = h.non_negative(hapke, "bc0");
    cfg.hapke.hc = h.positive(hapke, "hc");
    cfg.hapke.theta_bar_rad = h.non_negative(hapke, "theta_bar_deg") * constants::deg_to_rad;
    cfg.hapke.porosity_k = h.positive(hapke, "porosity_k");
    return cfg;
}

}  // namespace starsim::config
