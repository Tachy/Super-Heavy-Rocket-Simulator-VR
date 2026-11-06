#include "starsim/config/rock_abundance_file.hpp"

#include <fstream>

#include "starsim/config/data_missing.hpp"
#include "toml_document.hpp"

namespace starsim::config {

namespace {

[[noreturn]] void missing(const std::filesystem::path& path) {
    throw DataMissing(report::make(report::Code::data_rock_abundance_missing),
                      "rock abundance map (Diviner) missing: " + path.string() +
                      " - run python tools/data_pipeline/fetch.py diviner_rock_abundance");
}

}  // namespace

RockAbundanceFile load_rock_abundance_file(const std::filesystem::path& toml_path) {
    if (!std::filesystem::exists(toml_path)) missing(toml_path);
    const detail::Document doc(toml_path);
    const auto& g = doc.table(doc.root, "grid");
    RockAbundanceFile f;
    f.product = doc.text(g, "product");
    if (doc.text(g, "projection") != "simple_cylindrical") doc.fail("projection must be 'simple_cylindrical'");
    f.lines = static_cast<int>(doc.positive(g, "lines"));
    f.samples = static_cast<int>(doc.positive(g, "samples"));
    f.pixels_per_degree = doc.positive(g, "pixels_per_degree");
    f.zero_line = doc.number(g, "zero_line");
    f.zero_sample = doc.number(g, "zero_sample");
    f.max_latitude_deg = doc.positive(g, "max_latitude_deg");
    f.dn_scale = doc.positive(g, "dn_scale");
    f.missing_dn = static_cast<int>(doc.number(g, "missing_dn"));
    const auto values = toml_path.parent_path() / doc.text(g, "values");
    if (!std::filesystem::exists(values)) missing(values);
    const auto count = static_cast<size_t>(f.lines) * static_cast<size_t>(f.samples);
    if (std::filesystem::file_size(values) != 2 * count) doc.fail("values file size does not match lines x samples");
    f.dn.resize(count);
    std::ifstream in(values, std::ios::binary);
    in.read(reinterpret_cast<char*>(f.dn.data()), static_cast<std::streamsize>(2 * count));  // little endian (x86)
    if (!in) doc.fail("cannot read " + values.string());
    return f;
}

}  // namespace starsim::config
