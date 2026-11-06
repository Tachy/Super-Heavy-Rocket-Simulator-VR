// lunar_site_tile (lunar landing sites): plans and assembles the terrain / albedo tile around a landing site from the
// products of config/lunar_terrain_products.toml. The download itself is tools/data_pipeline (fetch.py).
// Usage:
//   lunar_site_tile plan <config dir> <height|albedo> <lat deg> <lon deg>
//       prints the request as JSON (product, cache key, byte ranges per file URL).
//   lunar_site_tile assemble <config dir> <request.json> <body dir> <out.toml>
//       reads the range bodies <body dir>/<range index>.bin (the bytes of ranges[i]) and writes the tile <out.toml> + .i16.
// Exit code 0 on success, 2 when no product holds the tile (lunar.site_no_product), 1 on any other error.
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "starsim/config/lunar_site_tile.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/report/report.hpp"

namespace {

using namespace starsim;

std::string quoted(const std::string& s) {
    std::string out = "\"";
    for (const char c : s) {
        if (c == '"' || c == '\\') out += '\\';
        out += c;
    }
    return out + "\"";
}

std::string request_json(const config::SiteTileRequest& r) {
    std::ostringstream j;
    char number[64];
    j << "{\n  \"product_id\": " << quoted(r.product_id) << ",\n  \"kind\": " << quoted(r.kind) << ",\n";
    std::snprintf(number, sizeof number, "%.17g", r.latitude_rad);
    j << "  \"latitude_rad\": " << number << ",\n";
    std::snprintf(number, sizeof number, "%.17g", r.longitude_rad);
    j << "  \"longitude_rad\": " << number << ",\n";
    std::snprintf(number, sizeof number, "%.17g", r.half_size_m);
    j << "  \"half_size_m\": " << number << ",\n  \"cache_key\": " << quoted(r.cache_key) << ",\n  \"ranges\": [\n";
    for (std::size_t i = 0; i < r.ranges.size(); ++i) {
        const auto& b = r.ranges[i];
        j << "    {\"index\": " << i << ", \"file_index\": " << b.file_index << ", \"url\": " << quoted(b.url)
          << ", \"first\": " << b.first << ", \"last\": " << b.last << "}" << (i + 1 < r.ranges.size() ? "," : "") << "\n";
    }
    j << "  ]\n}\n";
    return j.str();
}

// The text after `"key":` (the CLI reads back its own flat output only).
std::string json_value(const std::string& text, const std::string& key) {
    const auto at = text.find("\"" + key + "\":");
    if (at == std::string::npos) throw std::runtime_error("request.json: missing '" + key + "'");
    auto begin = text.find_first_not_of(' ', at + key.size() + 3);
    if (text[begin] == '"') {
        const auto end = text.find('"', begin + 1);
        return text.substr(begin + 1, end - begin - 1);
    }
    return text.substr(begin, text.find_first_of(",\n}", begin) - begin);
}

std::vector<std::uint8_t> read_bytes(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot read " + path.string());
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

int usage() {
    std::fprintf(stderr,
                 "usage: lunar_site_tile plan <config dir> <height|albedo> <lat deg> <lon deg>\n"
                 "       lunar_site_tile assemble <config dir> <request.json> <body dir> <out.toml>\n");
    return 1;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc < 2) return usage();
        const std::string command = argv[1];
        if (command == "plan" && argc == 6) {
            const auto products = config::load_lunar_terrain_products(argv[2]);
            const auto request = config::plan_site_tile(products, argv[3], std::atof(argv[4]) * constants::deg_to_rad,
                                                        std::atof(argv[5]) * constants::deg_to_rad);
            std::fputs(request_json(request).c_str(), stdout);
            return 0;
        }
        if (command == "assemble" && argc == 6) {
            const auto products = config::load_lunar_terrain_products(argv[2]);
            const auto text = [&] {
                std::ifstream in(argv[3]);
                if (!in) throw std::runtime_error(std::string("cannot read ") + argv[3]);
                return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
            }();
            // The request is planned again from its site: the same products give the same ranges.
            const auto request = config::plan_site_tile(products, json_value(text, "kind"),
                                                        std::strtod(json_value(text, "latitude_rad").c_str(), nullptr),
                                                        std::strtod(json_value(text, "longitude_rad").c_str(), nullptr));
            if (request.cache_key != json_value(text, "cache_key")) {
                throw std::runtime_error("request.json does not match the products any more (cache key " +
                                         request.cache_key + ")");
            }
            std::vector<std::vector<std::uint8_t>> bodies;
            for (std::size_t i = 0; i < request.ranges.size(); ++i) {
                bodies.push_back(read_bytes(std::filesystem::path(argv[4]) / (std::to_string(i) + ".bin")));
            }
            config::write_terrain_grid_file(config::assemble_site_tile(request, bodies), argv[5]);
            return 0;
        }
        return usage();
    } catch (const report::Refusal& e) {
        std::fprintf(stderr, "%s\n", report::describe(e.report()).c_str());
        return 2;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "lunar_site_tile: %s\n", e.what());
        return 1;
    }
}
