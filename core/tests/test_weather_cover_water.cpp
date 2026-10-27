// Cover from the total cloud cover and cumulus water from an adiabatic parcel (static clouds Task 9c):
// scale_cloud_cover_to_total, adiabatic_liquid_water_gradient_kg_m3_per_m, add_cumulus_adiabatic_water and the
// snapshot build / file / downsampling that carry the total cloud cover.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/cloud_render_texels.hpp"
#include "starsim/environment/weather_snapshot.hpp"
#include "starsim/environment/weather_snapshot_file.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using namespace starsim::environment;
namespace c = starsim::constants;

namespace {

constexpr std::size_t n_layers = static_cast<std::size_t>(weather_layer_count);

// A column of layer means on flat ground at 0 m with the standard atmosphere's temperature and pressure at the layer
// centres.
struct Column {
    std::array<double, n_layers> fraction{}, liquid{}, ice{}, temperature{}, pressure{};
    Column() {
        for (int k = 0; k < weather_layer_count; ++k) {
            const double z = 0.5 * (weather_layer_bottom_m(k) + weather_layer_bottom_m(k + 1));
            temperature[static_cast<std::size_t>(k)] = c::standard_temperature_k - c::standard_lapse_rate_k_per_m * z;
            pressure[static_cast<std::size_t>(k)] = standard_pressure_at_height_pa(z);
        }
    }
    // Cloud of fraction f and in-cloud liquid / ice water contents on the layers [first, last].
    void cloud(int first, int last, double f, double liquid_in_cloud, double ice_in_cloud = 0.0) {
        for (int k = first; k <= last; ++k) {
            fraction[static_cast<std::size_t>(k)] = f;
            liquid[static_cast<std::size_t>(k)] = f * liquid_in_cloud;
            ice[static_cast<std::size_t>(k)] = f * ice_in_cloud;
        }
    }
    double add(double lcl_m, double f_ad = c::cumulus_adiabatic_fraction) {
        return add_cumulus_adiabatic_water(fraction, liquid, ice, temperature, pressure, 0.0, lcl_m, f_ad);
    }
    double cover() const { return cloud_column_cover_max_random(fraction, liquid, ice, 0.0); }
    double liquid_path() const {
        double path = 0.0;
        for (int k = 0; k < weather_layer_count; ++k) {
            path += liquid[static_cast<std::size_t>(k)] * (weather_layer_bottom_m(k + 1) - weather_layer_bottom_m(k));
        }
        return path;
    }
};

// Sub-adiabatic liquid water path of the layers [first, last] from the base z_b to the boundary-layer top (ground 0),
// and the in-cloud one of the layers.
double floor_path(const Column& col, int first, int last, double base_m, double f_ad) {
    double path = 0.0;
    const double top_m = c::cloud_boundary_layer_top_m;
    for (int k = first; k <= last; ++k) {
        const double lo = std::min(std::max(weather_layer_bottom_m(k), base_m), top_m) - base_m;
        const double hi = std::min(weather_layer_bottom_m(k + 1), top_m) - base_m;
        const auto i = static_cast<std::size_t>(k);
        path += f_ad * adiabatic_liquid_water_gradient_kg_m3_per_m(col.temperature[i], col.pressure[i]) * 0.5 *
                (hi * hi - lo * lo);
    }
    return path;
}
double in_cloud_path(const Column& col, int first, int last) {
    double path = 0.0;
    for (int k = first; k <= last; ++k) {
        const auto i = static_cast<std::size_t>(k);
        path += col.liquid[i] / col.fraction[i] * (weather_layer_bottom_m(k + 1) - weather_layer_bottom_m(k));
    }
    return path;
}

// The test's own parcel: Romps' saturation pressure over liquid and latent heat (constants.hpp), mixing ratio over
// liquid, the moist air's hydrostatic pressure; each 0.1 m step solves c_pd dT + g dz + L(T_mid) dr_s = 0 for the new
// temperature by bisection (finite differences of r_s, no derivative).
double romps_es(double t) {
    const double rv = c::romps_vapour_gas_constant_j_kgk, t0 = c::romps_triple_point_temperature_k;
    const double cvv = c::romps_vapour_cv_j_kgk, cvl = c::romps_liquid_c_j_kgk;
    return c::romps_triple_point_pressure_pa * std::pow(t / t0, (cvv + rv - cvl) / rv) *
           std::exp((c::romps_vapour_internal_energy_at_triple_j_kg - (cvv - cvl) * t0) / rv * (1.0 / t0 - 1.0 / t));
}
double romps_latent_heat(double t) {
    return c::romps_vapour_internal_energy_at_triple_j_kg + c::romps_vapour_gas_constant_j_kgk * t +
           (c::romps_vapour_cv_j_kgk - c::romps_liquid_c_j_kgk) * (t - c::romps_triple_point_temperature_k);
}
double mixing_ratio(double t, double p) {
    const double e = romps_es(t);
    return c::romps_dry_air_gas_constant_j_kgk / c::romps_vapour_gas_constant_j_kgk * e / (p - e);
}
double moist_density(double t, double p) {
    const double e = romps_es(t);
    return (p - e) / (c::romps_dry_air_gas_constant_j_kgk * t) + e / (c::romps_vapour_gas_constant_j_kgk * t);
}

// Liquid water gradient of the numerical parcel over [0, h] m from (t, p): rho_d at the middle x (r_s(0) - r_s(h)) / h.
double parcel_gradient(double t, double p, double h) {
    const double cpd = c::romps_dry_air_cv_j_kgk + c::romps_dry_air_gas_constant_j_kgk, g = c::romps_gravity_mps2;
    const double dz = 0.1;
    const double r_start = mixing_ratio(t, p);
    double rho_d_mid = 0.0;
    for (int i = 0; i < static_cast<int>(std::lround(h / dz)); ++i) {
        const double r1 = mixing_ratio(t, p);
        const double p2 = p - g * moist_density(t, p) * dz;  // explicit, then re-evaluated at the midpoint
        double lo = t - 0.02, hi = t;
        double p_new = p2;
        for (int it = 0; it < 80; ++it) {
            const double t2 = 0.5 * (lo + hi);
            p_new = p - g * moist_density(0.5 * (t + t2), 0.5 * (p + p2)) * dz;
            const double residual = cpd * (t2 - t) + g * dz + romps_latent_heat(0.5 * (t + t2)) *
                                                                 (mixing_ratio(t2, p_new) - r1);
            (residual > 0.0 ? hi : lo) = t2;
        }
        t = 0.5 * (lo + hi);
        p = p_new;
        if (i == static_cast<int>(std::lround(h / dz)) / 2) {
            rho_d_mid = (p - romps_es(t)) / (c::romps_dry_air_gas_constant_j_kgk * t);
        }
    }
    return rho_d_mid * (r_start - mixing_ratio(t, p)) / h;
}

}  // namespace

TEST_CASE("The adiabatic liquid water gradient matches a numerical parcel", "[weather]") {
    struct Case { double t_k, p_pa; };
    for (const Case cs : {Case{288.0, 90000.0}, Case{283.0, 85000.0}, Case{275.0, 80000.0}, Case{298.0, 95000.0},
                          Case{268.0, 70000.0}}) {
        INFO(cs.t_k << " K, " << cs.p_pa << " Pa");
        // Over 20 m the gradient's own change (~0.3 % per 100 m) stays below the tolerance.
        const double numeric = parcel_gradient(cs.t_k, cs.p_pa, 20.0);
        // At the parcel's middle (10 m up): the analytic gradient there.
        const double analytic_start = adiabatic_liquid_water_gradient_kg_m3_per_m(cs.t_k, cs.p_pa);
        CHECK(analytic_start == Catch::Approx(numeric).epsilon(0.003));
    }
    // ~2 g/m^3/km in the boundary layer (cloud bases at 0-20 deg C, 800-950 hPa): within 1-2.5 g/m^3/km.
    for (const double t : {275.0, 280.0, 285.0, 290.0}) {
        for (const double p : {80000.0, 85000.0, 90000.0, 95000.0}) {
            const double gamma = adiabatic_liquid_water_gradient_kg_m3_per_m(t, p) * 1e6;  // g/m^3 per km
            INFO(t << " K, " << p << " Pa: " << gamma << " g/m^3/km");
            CHECK(gamma >= 1.0);
            CHECK(gamma <= 2.5);
        }
    }
    // Colder air holds less: monotone in temperature; invalid input gives 0.
    CHECK(adiabatic_liquid_water_gradient_kg_m3_per_m(270.0, 85000.0) <
          adiabatic_liquid_water_gradient_kg_m3_per_m(290.0, 85000.0));
    CHECK(adiabatic_liquid_water_gradient_kg_m3_per_m(std::nan(""), 85000.0) == 0.0);
    CHECK(adiabatic_liquid_water_gradient_kg_m3_per_m(280.0, 0.0) == 0.0);
}

TEST_CASE("A shallow cumulus reaches the sub-adiabatic water on its run path", "[weather]") {
    // Cumulus of fraction 0.3 on the layers 750 m .. 2 km (3..7), in-cloud 0.1 g/m^3 (the data's thin end), LCL 800 m.
    Column col;
    col.cloud(3, 7, 0.3, 1e-4);
    const Column before = col;
    const double base = 800.0;
    const double target = floor_path(col, 3, 7, base, c::cumulus_adiabatic_fraction);
    // The in-cloud path over the parts above the base (layer 3 holds its mean over the whole layer).
    const double in_cloud_before = in_cloud_path(col, 3, 7);
    REQUIRE(in_cloud_before < target);
    const double added = col.add(base);
    CHECK(in_cloud_path(col, 3, 7) == Catch::Approx(target).epsilon(1e-9));
    // One factor for the whole run: the profile is kept.
    for (int k = 3; k <= 7; ++k) {
        CHECK(col.liquid[static_cast<std::size_t>(k)] / before.liquid[static_cast<std::size_t>(k)] ==
              Catch::Approx(target / in_cloud_before).epsilon(1e-9));
    }
    // Cover and ice unchanged; the added grid-mean water path is the returned diagnostic.
    CHECK(col.fraction == before.fraction);
    CHECK(col.ice == before.ice);
    CHECK(added == Catch::Approx(col.liquid_path() - before.liquid_path()).epsilon(1e-12));
    CHECK(added > 0.0);
}

TEST_CASE("Cumulus water grows with the adiabatic fraction and stays bounded", "[weather]") {
    double previous = 0.0;
    for (const double f_ad : {0.0, 0.3, 0.4, 0.5}) {
        Column col;
        col.cloud(3, 7, 0.3, 1e-4);
        const double added = col.add(800.0, f_ad);
        if (f_ad == 0.0) {
            CHECK(added == 0.0);
        } else {
            CHECK(added > previous);
        }
        previous = added;
    }
    // A deep run (750 m .. 9 km): the parcel ends at the boundary-layer top - the whole run's in-cloud path reaches the
    // sub-adiabatic path up to 3 km, the run's water above counts but no parcel water is asked for there.
    Column deep;
    deep.cloud(3, 20, 0.3, 1e-4);
    const double deep_target = floor_path(deep, 3, 20, 800.0, c::cumulus_adiabatic_fraction);
    REQUIRE(in_cloud_path(deep, 3, 20) < deep_target);
    deep.add(800.0);
    CHECK(in_cloud_path(deep, 3, 20) == Catch::Approx(deep_target).epsilon(1e-9));
    // A very thin boundary-layer cumulus (0.01 g/m^3, 750 m .. 3 km) raised to a whole adiabatic parcel: no layer is
    // raised above the bound in-cloud, and the bound is reached.
    Column thin;
    thin.cloud(3, 11, 0.3, 1e-5);
    thin.add(800.0, 1.0);
    double highest = 0.0;
    for (int k = 3; k <= 11; ++k) highest = std::max(highest, thin.liquid[static_cast<std::size_t>(k)] / 0.3);
    CHECK(highest <= c::cumulus_liquid_water_max_kg_m3 * (1.0 + 1e-12));
    CHECK(highest == Catch::Approx(c::cumulus_liquid_water_max_kg_m3));
    // A layer the data already holds denser than the bound keeps its value.
    Column dense;
    dense.cloud(3, 11, 0.3, 1e-5);
    dense.liquid[8] = 0.3 * 2e-3;
    dense.add(800.0, 1.0);
    CHECK(dense.liquid[8] == 0.3 * 2e-3);
}

TEST_CASE("Decks, ice, deep-enough cumulus and clouds without an LCL base keep their water", "[weather]") {
    const auto unchanged = [](Column col, double lcl_m) {
        const Column before = col;
        const double added = col.add(lcl_m);
        CHECK(added == 0.0);
        CHECK(col.liquid == before.liquid);
        CHECK(col.fraction == before.fraction);
        CHECK(col.ice == before.ice);
    };
    Column deck;  // stratiform: fraction 1 (stratiform weight 1, cumulus weight 0)
    deck.cloud(3, 7, 1.0, 1e-4);
    unchanged(deck, 800.0);
    Column ice;  // ice share of the extinction 0.78: cirrus weight 1
    ice.cloud(3, 7, 0.3, 1e-5, 1e-4);
    unchanged(ice, 800.0);
    Column wet;  // in-cloud 1 g/m^3 over 1.2 km: deeper in water than the sub-adiabatic parcel
    wet.cloud(3, 7, 0.3, 1e-3);
    unchanged(wet, 800.0);
    Column cu;
    cu.cloud(3, 7, 0.3, 1e-4);
    unchanged(cu, 0.0);                   // saturated surface air: fog, no convective base
    unchanged(cu, weather_lcl_unknown_m);  // unknown
    unchanged(cu, 3500.0);                // LCL above the boundary layer
    Column high;  // the lowest liquid cloud starts above the boundary layer
    high.cloud(18, 20, 0.3, 1e-4);
    unchanged(high, 800.0);
    // Only the lowest run: a second cumulus run further up keeps its water.
    Column two;
    two.cloud(3, 7, 0.3, 1e-4);
    two.cloud(19, 21, 0.3, 1e-4);
    const Column before = two;
    CHECK(two.add(800.0) > 0.0);
    for (std::size_t k = 19; k <= 21; ++k) CHECK(two.liquid[k] == before.liquid[k]);
    // A deck with cumulus rising out of it (one run): the deck's layers keep their water, the cumulus above is raised.
    Column mixed;
    mixed.cloud(3, 5, 1.0, 1e-4);
    mixed.cloud(6, 9, 0.3, 1e-4);
    const Column mixed_before = mixed;
    CHECK(mixed.add(800.0) > 0.0);
    for (int k = 3; k <= 5; ++k) {
        CHECK(mixed.liquid[static_cast<std::size_t>(k)] == mixed_before.liquid[static_cast<std::size_t>(k)]);
    }
    for (int k = 6; k <= 9; ++k) {
        CHECK(mixed.liquid[static_cast<std::size_t>(k)] > mixed_before.liquid[static_cast<std::size_t>(k)]);
    }
    // The weight is continuous: a fraction-0.9 run (half stratiform) gets less than the cumulus, more than the deck.
    Column half;
    half.cloud(3, 7, 0.9, 1e-4);
    Column full;
    full.cloud(3, 7, 0.9, 1e-4);
    const double half_added = half.add(800.0);
    CHECK(half_added > 0.0);
    // The same column at weight 1 would reach the floor exactly; at weight 0.5 halfway in the factor.
    const double target = floor_path(full, 3, 7, 800.0, c::cumulus_adiabatic_fraction);
    const double s = target / in_cloud_path(full, 3, 7);
    CHECK(in_cloud_path(half, 3, 7) ==
          Catch::Approx(in_cloud_path(full, 3, 7) * (1.0 + 0.5 * (s - 1.0))).epsilon(1e-6));
}

TEST_CASE("The cover is scaled to the total cloud cover with the layer fractions in proportion", "[weather]") {
    // Two separated clouds: 0.2 low, 0.3 high: maximum-random 1 - 0.8 x 0.7 = 0.44.
    Column col;
    col.cloud(2, 4, 0.2, 1e-4);
    col.cloud(20, 22, 0.3, 1e-5, 1e-5);
    CHECK(col.cover() == Catch::Approx(0.44).epsilon(1e-12));
    const Column before = col;
    for (const double tcc : {0.6, 0.3, 0.95, 1.0}) {
        Column s = before;
        scale_cloud_cover_to_total(s.fraction, s.liquid, s.ice, 0.0, tcc);
        INFO("tcc " << tcc);
        CHECK(s.cover() == Catch::Approx(tcc).margin(1e-9));
        CHECK(s.liquid == before.liquid);  // the grid-mean water is kept
        CHECK(s.ice == before.ice);
        for (const double f : s.fraction) CHECK(f <= 1.0);
        if (s.fraction[20] < 1.0) CHECK(s.fraction[20] / s.fraction[2] == Catch::Approx(1.5).epsilon(1e-9));
    }
    // Touching layers overlap maximally: one cloud of fractions 0.2 / 0.4 / 0.3 has the cover 0.4.
    Column tower;
    tower.cloud(5, 5, 0.2, 1e-4);
    tower.cloud(6, 6, 0.4, 1e-4);
    tower.cloud(7, 7, 0.3, 1e-4);
    CHECK(tower.cover() == Catch::Approx(0.4).epsilon(1e-12));
    scale_cloud_cover_to_total(tower.fraction, tower.liquid, tower.ice, 0.0, 0.6);
    CHECK(tower.fraction[6] == Catch::Approx(0.6).epsilon(1e-9));
    CHECK(tower.fraction[5] == Catch::Approx(0.3).epsilon(1e-9));
    // Unknown cover, a clear column: unchanged. A cover of 0 with data cloud keeps each cloudy layer's smallest cover.
    for (const double unknown : {static_cast<double>(weather_total_cloud_cover_unknown), std::nan("")}) {
        Column u = before;
        scale_cloud_cover_to_total(u.fraction, u.liquid, u.ice, 0.0, unknown);
        CHECK(u.fraction == before.fraction);
    }
    Column clear;
    scale_cloud_cover_to_total(clear.fraction, clear.liquid, clear.ice, 0.0, 0.7);
    CHECK(clear.cover() == 0.0);
    Column none = before;
    scale_cloud_cover_to_total(none.fraction, none.liquid, none.ice, 0.0, 0.0);
    CHECK(none.fraction[2] == Catch::Approx(c::cloud_min_cover));
    CHECK(none.fraction[20] == Catch::Approx(c::cloud_min_cover));
    // A fraction without condensate (nothing shows there) neither counts nor is scaled.
    Column dry = before;
    dry.fraction[12] = 0.5;
    CHECK(dry.cover() == Catch::Approx(0.44).epsilon(1e-12));
    scale_cloud_cover_to_total(dry.fraction, dry.liquid, dry.ice, 0.0, 0.6);
    CHECK(dry.fraction[12] == 0.5);
    CHECK(dry.cover() == Catch::Approx(0.6).margin(1e-9));
    // Layers under the ground do not count.
    Column buried = before;
    CHECK(cloud_column_cover_max_random(buried.fraction, buried.liquid, buried.ice, 1500.0) ==
          Catch::Approx(0.3).epsilon(1e-12));
}

TEST_CASE("A built snapshot takes the total cloud cover and adds the cumulus water", "[weather]") {
    const int rows = 5, columns = 8, r = 2, col = 3;
    // Cloud below 2 km at 40 %, LCL ~870 m, the data's total cover 80 %.
    const auto unscaled = WeatherSnapshot::gfs_test_column(rows, columns, 2000.0, 1e-4, 40.0, 295.0, 288.0);
    const auto scaled = WeatherSnapshot::gfs_test_column(rows, columns, 2000.0, 1e-4, 40.0, 295.0, 288.0, 80.0);
    CHECK(unscaled.total_cloud_cover(r, col) == weather_total_cloud_cover_unknown);
    CHECK(scaled.total_cloud_cover(r, col) == Catch::Approx(0.8));
    const std::size_t cells = static_cast<std::size_t>(rows) * columns;
    const std::size_t cell = static_cast<std::size_t>(r) * columns + col;
    std::array<double, n_layers> fraction{}, liquid{}, ice{};
    const auto load = [&](const WeatherSnapshot& s) {
        for (std::size_t k = 0; k < n_layers; ++k) {
            const std::size_t i = (k * cells + cell) * 4;
            fraction[k] = decode_weather_cloud_fraction(s.texels()[i]);
            liquid[k] = decode_weather_water(s.texels()[i + 1]);
            ice[k] = decode_weather_water(s.texels()[i + 2]);
        }
        return cloud_column_cover_max_random(fraction, liquid, ice, s.ground_heights_m()[cell]);
    };
    CHECK(load(unscaled) == Catch::Approx(0.4).margin(0.01));
    // The 8-bit fraction codes: half a step per layer.
    CHECK(load(scaled) == Catch::Approx(0.8).margin(0.01));
    // Thin cumulus water (in-cloud ~0.25 g/m^3, 0.12 after the cover doubles) is raised: the diagnostic holds
    // the added path.
    CHECK(scaled.added_liquid_path_kg_m2(r, col) > 0.0f);
    CHECK(unscaled.added_liquid_path_kg_m2(r, col) > 0.0f);
}

TEST_CASE("Total cloud cover and added water survive the file and the downsampling", "[weather]") {
    const int rows = 9, columns = 8;
    const std::size_t cells = static_cast<std::size_t>(rows) * columns;
    std::vector<std::uint8_t> texels(n_layers * cells * 4, 0);
    std::vector<float> ground(cells, 0.0f), lcl(cells, 800.0f), tcc(cells, 0.5f), added(cells, 0.0f);
    for (std::size_t i = 0; i < cells; ++i) {
        tcc[i] = static_cast<float>(i % 5) * 0.25f;
        added[i] = static_cast<float>(i) * 1e-3f;
    }
    tcc[27] = weather_total_cloud_cover_unknown;  // row 3, column 3: inside the box of coarse cell (1, 1)
    const auto snapshot = WeatherSnapshot::from_parts({}, rows, columns, ground, texels, lcl, tcc, added);
    const auto file = std::filesystem::temp_directory_path() / "weather_tcc_round_trip_9c.wthr";
    write_weather_snapshot(file, snapshot);
    const auto back = read_weather_snapshot(file);
    REQUIRE(back.has_value());
    CHECK(back->total_cloud_covers() == tcc);
    CHECK(back->added_liquid_paths_kg_m2() == added);
    std::filesystem::remove(file);
    // Defaults: unknown cover, nothing added.
    const auto plain = WeatherSnapshot::from_parts({}, rows, columns, ground, texels, lcl);
    CHECK(plain.total_cloud_cover(3, 3) == weather_total_cloud_cover_unknown);
    CHECK(plain.added_liquid_path_kg_m2(3, 3) == 0.0f);
    // Box mean (the known columns of the box for the cover), as every other field of the downsampled set.
    const auto coarse = snapshot.downsampled(4);
    double sum = 0.0, weights = 0.0, sum_added = 0.0, weights_added = 0.0;
    for (int dr = -2; dr <= 2; ++dr) {
        for (int dc = -2; dc <= 2; ++dc) {
            const double w = (std::abs(dr) == 2 ? 0.5 : 1.0) * (std::abs(dc) == 2 ? 0.5 : 1.0);
            const std::size_t i =
                static_cast<std::size_t>(4 + dr) * columns + static_cast<std::size_t>((4 + dc + columns) % columns);
            sum_added += w * added[i];
            weights_added += w;
            if (tcc[i] < 0.0f) continue;
            sum += w * tcc[i];
            weights += w;
        }
    }
    CHECK(coarse.total_cloud_cover(1, 1) == Catch::Approx(sum / weights).epsilon(1e-6));
    CHECK(coarse.added_liquid_path_kg_m2(1, 1) == Catch::Approx(sum_added / weights_added).epsilon(1e-6));
}

TEST_CASE("A downsampled box keeps the total cloud cover of its cells", "[weather]") {
    // A checkerboard: half the cells hold their cloud (fraction 0.5) in layer 5, the other half in layer 6, each cell's
    // total cover 0.5. The box means (0.25 / 0.25, touching layers: maximum overlap 0.25) are scaled back to 0.5.
    const int rows = 9, columns = 8;
    const std::size_t cells = static_cast<std::size_t>(rows) * columns;
    std::vector<std::uint8_t> texels(n_layers * cells * 4, 0);
    for (int r = 0; r < rows; ++r) {
        for (int col = 0; col < columns; ++col) {
            const std::size_t layer = (r + col) % 2 == 0 ? 5 : 6, cell = static_cast<std::size_t>(r) * columns + col;
            const std::size_t i = (layer * cells + cell) * 4;
            texels[i] = encode_weather_cloud_fraction(0.5);
            texels[i + 1] = encode_weather_water(1e-4);
        }
    }
    const auto snapshot = WeatherSnapshot::from_parts({}, rows, columns, std::vector<float>(cells, 0.0f), texels, {},
                                                      std::vector<float>(cells, 0.5f));
    const auto coarse = snapshot.downsampled(4);
    const std::size_t coarse_cells = static_cast<std::size_t>(coarse.rows()) * coarse.columns();
    for (int r = 0; r < coarse.rows(); ++r) {
        for (int col = 0; col < coarse.columns(); ++col) {
            std::array<double, n_layers> fraction{}, liquid{}, ice{};
            for (std::size_t k = 0; k < n_layers; ++k) {
                const std::size_t i = (k * coarse_cells + static_cast<std::size_t>(r) * coarse.columns() + col) * 4;
                fraction[k] = decode_weather_cloud_fraction(coarse.texels()[i]);
                liquid[k] = decode_weather_water(coarse.texels()[i + 1]);
            }
            INFO("coarse cell " << r << ", " << col);
            CHECK(cloud_column_cover_max_random(fraction, liquid, ice, 0.0) == Catch::Approx(0.5).margin(0.005));
        }
    }
}
