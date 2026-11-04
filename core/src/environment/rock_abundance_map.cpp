#include "starsim/environment/rock_abundance_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "starsim/math/constants.hpp"

namespace starsim::environment {

namespace {

// Grid offsets are written by the pipeline as the label's numbers: a layout match is exact up to rounding.
constexpr double layout_tolerance_px = 1e-6;

}  // namespace

double RockAbundanceMap::at(double latitude_rad, double longitude_rad) const {
    const double line = std::clamp((lat_limit_rad - latitude_rad) / (2.0 * lat_limit_rad) * lines - 0.5, 0.0,
                                   static_cast<double>(lines - 1));
    double lon = std::fmod(longitude_rad, constants::two_pi);
    if (lon < 0.0) lon += constants::two_pi;
    const double sample = lon / constants::two_pi * samples - 0.5;
    const int l0 = std::min(static_cast<int>(std::floor(line)), lines - 1);
    const int l1 = std::min(l0 + 1, lines - 1);
    const double fl = line - l0;
    const int s0 = static_cast<int>(std::floor(sample));
    const double fs = sample - s0;
    double sum = 0.0, weight = 0.0;
    const auto add = [&](int l, int s, double w) {
        const int wrapped = ((s % samples) + samples) % samples;
        const double v = area_fraction[static_cast<size_t>(l) * static_cast<size_t>(samples) +
                                       static_cast<size_t>(wrapped)];
        if (std::isnan(v) || w <= 0.0) return;
        sum += w * v;
        weight += w;
    };
    add(l0, s0, (1.0 - fl) * (1.0 - fs));
    add(l0, s0 + 1, (1.0 - fl) * fs);
    add(l1, s0, fl * (1.0 - fs));
    add(l1, s0 + 1, fl * fs);
    return weight > 0.0 ? sum / weight : std::numeric_limits<double>::quiet_NaN();
}

RockAbundanceMap make_rock_abundance_map(const config::RockAbundanceFile& file) {
    const double ppd = file.pixels_per_degree;
    const bool layout_ok = file.lines >= 2 && file.samples >= 2 &&
                           std::abs(file.zero_line - (0.5 * file.lines - 0.5)) < layout_tolerance_px &&
                           std::abs(file.zero_sample + 0.5) < layout_tolerance_px &&
                           std::abs(file.max_latitude_deg * ppd - 0.5 * file.lines) < layout_tolerance_px &&
                           std::abs(360.0 * ppd - file.samples) < layout_tolerance_px &&
                           file.dn.size() == static_cast<size_t>(file.lines) * static_cast<size_t>(file.samples);
    if (!layout_ok) {
        throw std::invalid_argument("Felsenhäufigkeitskarte " + file.product + ": unerwartetes Kartenraster");
    }
    RockAbundanceMap map;
    map.lines = file.lines;
    map.samples = file.samples;
    map.lat_limit_rad = file.max_latitude_deg * constants::deg_to_rad;
    map.area_fraction.resize(file.dn.size());
    std::transform(file.dn.begin(), file.dn.end(), map.area_fraction.begin(), [&](std::int16_t dn) {
        return dn == file.missing_dn ? std::numeric_limits<float>::quiet_NaN()
                                     : static_cast<float>(dn * file.dn_scale);
    });
    return map;
}

std::shared_ptr<const RockAbundanceMap> load_rock_abundance_map(const config::RockPopulation& rocks,
                                                                const std::filesystem::path& terrain_dir) {
    if (rocks.abundance_source != "diviner") return nullptr;
    return std::make_shared<const RockAbundanceMap>(
        make_rock_abundance_map(config::load_rock_abundance_file(terrain_dir / config::rock_abundance_file_name)));
}

}  // namespace starsim::environment
