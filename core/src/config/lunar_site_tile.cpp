#include "starsim/config/lunar_site_tile.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <optional>
#include <stdexcept>

#include "starsim/math/constants.hpp"
#include "starsim/report/report.hpp"
#include "toml_document.hpp"

namespace starsim::config {

namespace {

constexpr double radius_m = constants::moon_mean_radius_m;
constexpr double height_dn_scale_m = 0.5;            // as the LOLA products and today's tiles
constexpr double albedo_dn_scale = 1.0 / 16384.0;   // relative albedo 1.0 = 16384 (ledger ruling R3)
constexpr double albedo_lowpass_width_m = 10000.0;  // the "own low-pass" an albedo tile is divided by

int bytes_per_sample(const std::string& sample_type) {
    if (sample_type == "int16") return 2;
    if (sample_type == "float32") return 4;
    throw std::runtime_error("lunar site tile: unknown sample type '" + sample_type + "'");
}

// ---------------------------------------------------------------------------------------------------------- products

LunarTerrainFile parse_file(const detail::Document& doc, const toml::table& t, bool polar) {
    LunarTerrainFile f;
    f.url = doc.text(t, "url");
    f.lines = static_cast<int>(doc.positive(t, "lines"));
    f.samples = static_cast<int>(doc.positive(t, "samples"));
    f.record_bytes = static_cast<std::uint64_t>(doc.positive(t, "record_bytes"));
    f.header_bytes = static_cast<std::uint64_t>(doc.number(t, "header_bytes"));
    if (polar) {
        f.pole_line = doc.number(t, "pole_line");
        f.pole_sample = doc.number(t, "pole_sample");
        f.scale_m = doc.positive(t, "scale_m");
    } else {
        f.pixels_per_degree = doc.positive(t, "pixels_per_degree");
        f.first_latitude_deg = doc.number(t, "first_latitude_deg");
        f.first_longitude_deg = doc.number(t, "first_longitude_deg");
    }
    return f;
}

LunarTerrainProduct parse_product(const detail::Document& doc, const toml::table& t) {
    LunarTerrainProduct p;
    p.id = doc.text(t, "id");
    p.kind = doc.text(t, "kind");
    p.projection = doc.text(t, "projection");
    const bool polar = p.projection == "polar_stereographic_south" || p.projection == "polar_stereographic_north";
    if (!polar && p.projection != "simple_cylindrical") doc.fail("product " + p.id + ": unknown projection");
    if (p.kind != "height" && p.kind != "albedo") doc.fail("product " + p.id + ": unknown kind '" + p.kind + "'");
    p.latitude_min_deg = doc.number(t, "latitude_min_deg");
    p.latitude_max_deg = doc.number(t, "latitude_max_deg");
    p.resolution_m = doc.positive(t, "resolution_m");
    p.sample_type = doc.text(t, "sample_type");
    bytes_per_sample(p.sample_type);
    p.scale = doc.number(t, "scale");
    p.offset = doc.number(t, "offset");
    p.source = doc.text(t, "source");
    p.invalid_below = doc.number_or(t, "invalid_below", p.invalid_below);
    p.tile_half_size_m = doc.number_or(t, "tile_half_size_m", 0.0);
    const auto* files = t["file"].as_array();
    if (!files || files->empty()) doc.fail("product " + p.id + ": no files");
    for (const auto& node : *files) p.files.push_back(parse_file(doc, *node.as_table(), polar));
    if (polar && p.files.size() != 1) doc.fail("product " + p.id + ": a polar product has exactly one file");
    for (const auto& f : p.files) {
        if (!polar && f.pixels_per_degree != p.files.front().pixels_per_degree) {
            doc.fail("product " + p.id + ": files differ in pixels_per_degree");
        }
    }
    return p;
}

std::vector<std::string> string_list(const detail::Document& doc, const toml::table& t, std::string_view key) {
    const auto* list = t[key].as_array();
    if (!list) doc.fail("missing array '" + std::string(key) + "'");
    std::vector<std::string> ids;
    for (const auto& node : *list) {
        const auto v = node.value<std::string>();
        if (!v) doc.fail("'" + std::string(key) + "' holds strings only");
        ids.push_back(*v);
    }
    return ids;
}

// ---------------------------------------------------------------------------------------------------------- planning

// The tile before the byte ranges: geometry plus the contiguous pieces in the files.
struct Piece {
    std::size_t file_index;
    std::uint64_t offset;  // first byte in the file
    std::uint64_t bytes;
    int line, sample, count;
};

struct Crop {
    SiteTileRequest request;  // geometry only
    std::vector<Piece> pieces;
};

double wrap_deg(double deg) { return std::fmod(std::fmod(deg, 360.0) + 360.0, 360.0); }

long long mod_ll(long long a, long long n) { return ((a % n) + n) % n; }

std::optional<Crop> crop_polar(const LunarTerrainProduct& p, double lat, double lon) {
    const bool north = p.projection == "polar_stereographic_north";
    if (north != (lat > 0.0)) return std::nullopt;
    const auto& f = p.files.front();
    const double sign = north ? 1.0 : -1.0;
    const double rho = 2.0 * radius_m * std::tan(0.25 * constants::pi - sign * 0.5 * lat);
    const double line_c = f.pole_line + sign * rho * std::cos(lon) / f.scale_m;
    const double sample_c = f.pole_sample + rho * std::sin(lon) / f.scale_m;
    const int half_px = static_cast<int>(std::lround(p.tile_half_size_m / f.scale_m));
    const int n = 2 * half_px + 1;
    const int first_line = static_cast<int>(std::lround(line_c)) - half_px;
    const int first_sample = static_cast<int>(std::lround(sample_c)) - half_px;
    if (first_line < 0 || first_sample < 0 || first_line + n > f.lines || first_sample + n > f.samples) return std::nullopt;
    // The product ends at its latitude limit (the file's corners hold lower latitudes: no data or other data). The
    // point of the crop farthest from the pole is one of its corners.
    const double lowest_abs_deg = std::min(std::abs(p.latitude_min_deg), std::abs(p.latitude_max_deg));
    for (const int dl : {0, n - 1}) {
        for (const int ds : {0, n - 1}) {
            const double y = (first_line + dl - f.pole_line) * f.scale_m;
            const double x = (first_sample + ds - f.pole_sample) * f.scale_m;
            const double corner_abs_deg = (0.5 * constants::pi - 2.0 * std::atan(std::hypot(x, y) / (2.0 * radius_m))) *
                                          constants::rad_to_deg;
            if (corner_abs_deg < lowest_abs_deg) return std::nullopt;
        }
    }
    Crop c;
    auto& r = c.request;
    r.projection = p.projection;
    r.lines = n;
    r.samples = n;
    r.pole_line = f.pole_line;
    r.pole_sample = f.pole_sample;
    r.scale_m = f.scale_m;
    r.first_line = first_line;
    r.first_sample = first_sample;
    const auto bpp = static_cast<std::uint64_t>(bytes_per_sample(p.sample_type));
    for (int row = 0; row < n; ++row) {
        c.pieces.push_back({0, f.header_bytes + static_cast<std::uint64_t>(first_line + row) * f.record_bytes +
                                   static_cast<std::uint64_t>(first_sample) * bpp,
                            static_cast<std::uint64_t>(n) * bpp, row, 0, n});
    }
    return c;
}

std::optional<Crop> crop_cylindrical(const LunarTerrainProduct& p, double lat, double lon) {
    const double ppd = p.files.front().pixels_per_degree;
    const double lat_deg = lat * constants::rad_to_deg;
    const double lon_deg = wrap_deg(lon * constants::rad_to_deg);
    const long long total_lines = std::llround((p.latitude_max_deg - p.latitude_min_deg) * ppd);
    const long long width = std::llround(360.0 * ppd);
    const double ns_m = radius_m * constants::deg_to_rad / ppd;  // metres per pixel along a meridian
    const long long half_l = std::llround(p.tile_half_size_m / ns_m);
    // A parallel's pixel is narrower by cos(latitude): the half width in samples is set at the crop's most poleward line.
    const double edge_lat_rad = std::min(std::abs(lat_deg) + static_cast<double>(half_l) / ppd, 89.0) * constants::deg_to_rad;
    const long long half_s = std::llround(p.tile_half_size_m / (ns_m * std::cos(edge_lat_rad)));
    const long long n_l = 2 * half_l + 1, n_s = 2 * half_s + 1;
    if (n_s > width) return std::nullopt;
    const long long first_line = std::llround((p.latitude_max_deg - lat_deg) * ppd - 0.5) - half_l;
    if (first_line < 0 || first_line + n_l > total_lines) return std::nullopt;
    const long long first_sample = mod_ll(std::llround(lon_deg * ppd - 0.5) - half_s, width);

    struct Placement {
        long long line0, sample0;
    };
    std::vector<Placement> place;
    for (const auto& f : p.files) {
        place.push_back({std::llround((p.latitude_max_deg - f.first_latitude_deg) * ppd),
                         std::llround(f.first_longitude_deg * ppd)});
    }
    Crop c;
    auto& r = c.request;
    r.projection = p.projection;
    r.lines = static_cast<int>(n_l);
    r.samples = static_cast<int>(n_s);
    r.pixels_per_degree = ppd;
    r.first_latitude_rad = (p.latitude_max_deg - (static_cast<double>(first_line) + 0.5) / ppd) * constants::deg_to_rad;
    r.first_longitude_rad = (static_cast<double>(first_sample) + 0.5) / ppd * constants::deg_to_rad;
    const auto bpp = static_cast<std::uint64_t>(bytes_per_sample(p.sample_type));
    for (long long row = 0; row < n_l; ++row) {
        const long long line = first_line + row;
        long long done = 0, s = first_sample;
        while (done < n_s) {
            std::size_t index = p.files.size();
            for (std::size_t i = 0; i < p.files.size(); ++i) {
                const auto& f = p.files[i];
                if (line >= place[i].line0 && line < place[i].line0 + f.lines && s >= place[i].sample0 &&
                    s < place[i].sample0 + f.samples) {
                    index = i;
                    break;
                }
            }
            if (index == p.files.size()) return std::nullopt;  // a hole between the files
            const auto& f = p.files[index];
            const long long take = std::min(n_s - done, place[index].sample0 + f.samples - s);
            c.pieces.push_back({index,
                                f.header_bytes + static_cast<std::uint64_t>(line - place[index].line0) * f.record_bytes +
                                    static_cast<std::uint64_t>(s - place[index].sample0) * bpp,
                                static_cast<std::uint64_t>(take) * bpp, static_cast<int>(row), static_cast<int>(done),
                                static_cast<int>(take)});
            done += take;
            s = (s + take) % width;
        }
    }
    return c;
}

std::string cache_key_of(const LunarTerrainProduct& p, double lat, double lon) {
    char buffer[128];
    std::snprintf(buffer, sizeof buffer, "_%lld_%lld_%lld", std::llround(lat * constants::rad_to_deg * 100.0),
                  mod_ll(std::llround(wrap_deg(lon * constants::rad_to_deg) * 100.0), 36000),
                  std::llround(p.tile_half_size_m / 1000.0));
    return p.kind + "_" + p.id + buffer;
}

// Orders the pieces by position in the files and joins neighbours into ranges by the policy.
void make_ranges(Crop& c, const LunarTerrainProduct& p, const RangePolicy& policy) {
    auto& r = c.request;
    std::vector<std::size_t> order(c.pieces.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        const auto &x = c.pieces[a], &y = c.pieces[b];
        return x.file_index != y.file_index ? x.file_index < y.file_index : x.offset < y.offset;
    });
    r.segments.resize(c.pieces.size());
    for (const std::size_t i : order) {
        const auto& piece = c.pieces[i];
        const std::uint64_t last = piece.offset + piece.bytes - 1;
        bool join = false;
        if (!r.ranges.empty()) {
            const auto& cur = r.ranges.back();
            join = cur.file_index == piece.file_index && piece.offset > cur.last &&
                   piece.offset - cur.last - 1 <= policy.max_gap_bytes &&
                   last - cur.first + 1 <= policy.max_block_bytes;
        }
        if (join) {
            r.ranges.back().last = last;
        } else {
            r.ranges.push_back({piece.file_index, p.files[piece.file_index].url, piece.offset, last});
        }
        r.segments[i] = {r.ranges.size() - 1, piece.offset - r.ranges.back().first, piece.line, piece.sample, piece.count};
    }
}

// ---------------------------------------------------------------------------------------------------------- assembly

// Position (latitude, longitude; radians) of a fractional tile position.
std::pair<double, double> tile_lat_lon(const SiteTileRequest& r, double line, double sample) {
    if (r.projection == "simple_cylindrical") {
        const double lat = r.first_latitude_rad - line / r.pixels_per_degree * constants::deg_to_rad;
        const double lon = r.first_longitude_rad + sample / r.pixels_per_degree * constants::deg_to_rad;
        return {lat, std::fmod(std::fmod(lon, constants::two_pi) + constants::two_pi, constants::two_pi)};
    }
    const double sign = r.projection == "polar_stereographic_north" ? 1.0 : -1.0;
    const double y = (line + r.first_line - r.pole_line) * r.scale_m;
    const double x = (sample + r.first_sample - r.pole_sample) * r.scale_m;
    const double lat = sign * (0.5 * constants::pi - 2.0 * std::atan(std::hypot(x, y) / (2.0 * r.sphere_radius_m)));
    return {lat, std::atan2(x, sign * y)};
}

// The windowed mean of `value` over the valid pixels, box of half widths (ry, rx) clipped at the tile edge (integral
// images; O(pixels)).
std::vector<double> box_mean(const std::vector<double>& value, const std::vector<char>& valid, int lines, int samples,
                             int ry, int rx) {
    const size_t stride = static_cast<size_t>(samples) + 1;
    std::vector<double> sum((static_cast<size_t>(lines) + 1) * stride, 0.0), cnt(sum.size(), 0.0);
    for (int l = 0; l < lines; ++l) {
        double row_sum = 0.0, row_cnt = 0.0;
        for (int s = 0; s < samples; ++s) {
            const size_t i = static_cast<size_t>(l) * static_cast<size_t>(samples) + static_cast<size_t>(s);
            if (valid[i]) {
                row_sum += value[i];
                row_cnt += 1.0;
            }
            const size_t k = (static_cast<size_t>(l) + 1) * stride + static_cast<size_t>(s) + 1;
            sum[k] = sum[k - stride] + row_sum;
            cnt[k] = cnt[k - stride] + row_cnt;
        }
    }
    std::vector<double> mean(value.size(), 0.0);
    for (int l = 0; l < lines; ++l) {
        const size_t l0 = static_cast<size_t>(std::max(0, l - ry)), l1 = static_cast<size_t>(std::min(lines, l + ry + 1));
        for (int s = 0; s < samples; ++s) {
            const size_t s0 = static_cast<size_t>(std::max(0, s - rx)), s1 = static_cast<size_t>(std::min(samples, s + rx + 1));
            const auto window = [&](const std::vector<double>& a) {
                return a[l1 * stride + s1] - a[l0 * stride + s1] - a[l1 * stride + s0] + a[l0 * stride + s0];
            };
            const double n = window(cnt);
            mean[static_cast<size_t>(l) * static_cast<size_t>(samples) + static_cast<size_t>(s)] =
                n > 0.0 ? window(sum) / n : 0.0;
        }
    }
    return mean;
}

std::int16_t to_dn(double value, double dn_scale) {
    return static_cast<std::int16_t>(std::clamp(std::llround(value / dn_scale), -32767LL, 32767LL));
}

}  // namespace

RangePolicy default_range_policy(std::string_view kind) {
    RangePolicy p;
    if (kind == "albedo") p.max_gap_bytes = 512ull << 10;  // > one row's unused bytes of every albedo file (<= 110 kB)
    return p;
}

std::vector<LunarTerrainProduct> load_lunar_terrain_products(const std::filesystem::path& config_dir) {
    const detail::Document doc(config_dir / "lunar_terrain_products.toml");
    const auto& sel = doc.table(doc.root, "selection");
    const double half_size[2] = {doc.positive(sel, "tile_half_size_m"), doc.positive(sel, "albedo_tile_half_size_m")};
    std::vector<LunarTerrainProduct> all;
    const auto* list = doc.root["product"].as_array();
    if (!list) doc.fail("no [[product]]");
    for (const auto& node : *list) all.push_back(parse_product(doc, *node.as_table()));
    std::vector<LunarTerrainProduct> ordered;
    const std::pair<const char*, const char*> orders[] = {{"height_order", "height"}, {"albedo_order", "albedo"}};
    for (const auto& [key, kind] : orders) {
        for (const auto& id : string_list(doc, sel, key)) {
            const auto it = std::find_if(all.begin(), all.end(), [&](const auto& p) { return p.id == id; });
            if (it == all.end()) doc.fail(std::string(key) + ": unknown product '" + id + "'");
            if (it->kind != kind) doc.fail(std::string(key) + ": product '" + id + "' is not of kind " + kind);
            auto p = *it;
            if (p.tile_half_size_m <= 0.0) p.tile_half_size_m = half_size[p.kind == "height" ? 0 : 1];
            ordered.push_back(std::move(p));
        }
    }
    return ordered;
}

SiteTileRequest plan_site_tile(const std::vector<LunarTerrainProduct>& products, std::string_view kind,
                               double latitude_rad, double longitude_rad) {
    return plan_site_tile(products, kind, latitude_rad, longitude_rad, default_range_policy(kind));
}

SiteTileRequest plan_site_tile(const std::vector<LunarTerrainProduct>& products, std::string_view kind,
                               double latitude_rad, double longitude_rad, const RangePolicy& policy) {
    for (const auto& p : products) {
        if (p.kind != kind) continue;
        auto crop = p.projection == "simple_cylindrical" ? crop_cylindrical(p, latitude_rad, longitude_rad)
                                                         : crop_polar(p, latitude_rad, longitude_rad);
        if (!crop) continue;
        auto& r = crop->request;
        r.product_id = p.id;
        r.kind = p.kind;
        r.latitude_rad = latitude_rad;
        r.longitude_rad = longitude_rad;
        r.half_size_m = p.tile_half_size_m;
        r.cache_key = cache_key_of(p, latitude_rad, longitude_rad);
        r.sphere_radius_m = radius_m;
        r.sample_type = p.sample_type;
        r.value_scale = p.scale;
        r.value_offset = p.offset;
        r.invalid_below = p.invalid_below;
        r.resolution_m = p.resolution_m;
        make_ranges(*crop, p, policy);
        return std::move(r);
    }
    throw report::Refusal(report::make(report::Code::lunar_site_no_product,
                                       {report::angle("latitude", latitude_rad), report::angle("longitude", longitude_rad)}));
}

TerrainGridFile assemble_site_tile(const SiteTileRequest& r, const std::vector<std::vector<std::uint8_t>>& bodies) {
    if (bodies.size() != r.ranges.size()) throw std::runtime_error("lunar site tile: one body per range expected");
    const int bpp = bytes_per_sample(r.sample_type);
    const size_t count = static_cast<size_t>(r.lines) * static_cast<size_t>(r.samples);
    std::vector<double> value(count, 0.0);
    std::vector<char> valid(count, 0), seen(count, 0);
    for (const auto& seg : r.segments) {
        const auto& body = bodies[seg.range_index];
        if (seg.body_offset + static_cast<std::uint64_t>(seg.count) * static_cast<std::uint64_t>(bpp) > body.size()) {
            throw std::runtime_error("lunar site tile: range body " + std::to_string(seg.range_index) + " is too short");
        }
        const std::uint8_t* src = body.data() + seg.body_offset;
        size_t at = static_cast<size_t>(seg.line) * static_cast<size_t>(r.samples) + static_cast<size_t>(seg.sample);
        for (int i = 0; i < seg.count; ++i, ++at, src += bpp) {
            double raw;
            if (bpp == 2) {
                std::int16_t v;
                std::memcpy(&v, src, 2);  // little endian (x86), as the other grid readers
                raw = v;
            } else {
                float v;
                std::memcpy(&v, src, 4);
                raw = v;
            }
            seen[at] = 1;
            valid[at] = std::isfinite(raw) && raw >= r.invalid_below;
            value[at] = raw * r.value_scale + r.value_offset;
        }
    }
    if (std::find(seen.begin(), seen.end(), 0) != seen.end()) {
        throw std::runtime_error("lunar site tile: the ranges do not cover the whole tile");
    }

    TerrainGridFile f;
    f.product = r.product_id;
    f.kind = r.kind;
    f.projection = r.projection;
    f.sphere_radius_m = r.sphere_radius_m;
    f.lines = r.lines;
    f.samples = r.samples;
    if (r.projection == "simple_cylindrical") {
        f.pixels_per_degree = r.pixels_per_degree;
        f.zero_line = r.pixels_per_degree * r.first_latitude_rad * constants::rad_to_deg;
        f.zero_sample = -r.pixels_per_degree * r.first_longitude_rad * constants::rad_to_deg;
    } else {
        f.pole_line = r.pole_line;
        f.pole_sample = r.pole_sample;
        f.scale_m = r.scale_m;
        f.first_line = r.first_line;
        f.first_sample = r.first_sample;
    }
    f.dn.resize(count);

    if (r.kind == "height") {
        if (std::find(valid.begin(), valid.end(), 0) != valid.end()) {
            throw std::runtime_error("lunar site tile: no-data pixel in a height tile");
        }
        f.dn_scale_m = height_dn_scale_m;
        for (size_t i = 0; i < count; ++i) f.dn[i] = to_dn(value[i], height_dn_scale_m);
        // 9 check points on a 3 x 3 lattice (bilinear from the stored DN, as LunarTerrain reads the tile).
        const auto stride = static_cast<size_t>(r.samples);
        for (const double fl : {0.25, 0.5, 0.75}) {
            for (const double fs : {0.25, 0.5, 0.75}) {
                const double line = fl * (r.lines - 1), sample = fs * (r.samples - 1);
                const int l0 = std::min(static_cast<int>(line), r.lines - 2), s0 = std::min(static_cast<int>(sample), r.samples - 2);
                const double dl = line - l0, ds = sample - s0;
                const auto* row0 = &f.dn[static_cast<size_t>(l0) * stride + static_cast<size_t>(s0)];
                const auto* row1 = row0 + stride;
                const double h = height_dn_scale_m * ((1.0 - dl) * ((1.0 - ds) * row0[0] + ds * row0[1]) +
                                                      dl * ((1.0 - ds) * row1[0] + ds * row1[1]));
                const auto [lat, lon] = tile_lat_lon(r, line, sample);
                f.checks.push_back({lat, lon, h});
            }
        }
        return f;
    }

    // Albedo: relative to the tile's own low-pass, so the global mosaic keeps its calibration and the tile adds detail.
    const bool cylindrical = r.projection == "simple_cylindrical";
    const double line_m = cylindrical ? radius_m * constants::deg_to_rad / r.pixels_per_degree : r.scale_m;
    const double centre_lat = cylindrical ? r.first_latitude_rad - 0.5 * (r.lines - 1) / r.pixels_per_degree * constants::deg_to_rad
                                          : 0.0;
    const double sample_m = cylindrical ? line_m * std::cos(centre_lat) : r.scale_m;
    const int ry = std::max(1, static_cast<int>(std::lround(0.5 * albedo_lowpass_width_m / line_m)));
    const int rx = std::max(1, static_cast<int>(std::lround(0.5 * albedo_lowpass_width_m / sample_m)));
    const auto low = box_mean(value, valid, r.lines, r.samples, ry, rx);
    std::vector<double> rel(count, 1.0);
    double sum = 0.0, n = 0.0;
    for (size_t i = 0; i < count; ++i) {
        if (!valid[i] || !(low[i] > 0.0)) continue;
        rel[i] = value[i] / low[i];
        sum += rel[i];
        n += 1.0;
    }
    if (n == 0.0) throw std::runtime_error("lunar site tile: the albedo tile has no valid pixel");
    const double mean = sum / n;
    f.dn_scale_m = albedo_dn_scale;
    for (size_t i = 0; i < count; ++i) f.dn[i] = to_dn(valid[i] && low[i] > 0.0 ? rel[i] / mean : 1.0, albedo_dn_scale);
    return f;
}

}  // namespace starsim::config
